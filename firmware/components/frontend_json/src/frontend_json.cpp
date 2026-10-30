// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "frontend_json.hpp"
#include "rf_manager.hpp"
#include "rf_protocol.hpp"
#include "config_registry.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include <cstring>
#include <cstdio>
#ifdef CONFIG_CULFW32_BUILD_RF_MORITZ
#include "moritz_rf.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_ASKSIN
#include "asksin_rf.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_SLOWRF
#include "slowrf_rf.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_FASTRF
#include "fastrf_rf.hpp"
#endif

namespace culfw32 {

namespace {
constexpr const char* TAG = "fe-json";

constexpr ProtocolId kCompatible[] = {
    ProtocolId::Moritz, ProtocolId::AskSin, ProtocolId::SlowRf,
    ProtocolId::Native, ProtocolId::FastRf,
};

const char* proto_name(ProtocolId p) {
    switch (p) {
        case ProtocolId::Moritz:       return "Moritz";
        case ProtocolId::AskSin:       return "AskSin";
        case ProtocolId::SlowRf:       return "SlowRf";
        case ProtocolId::PulseCapture: return "PulseCapture";
        case ProtocolId::Native:       return "Native";
        case ProtocolId::FastRf:       return "FastRf";
        case ProtocolId::Mbus:         return "Mbus";
        case ProtocolId::Hmip:         return "Hmip";
        case ProtocolId::Dudette:      return "Dudette";
    }
    return "?";
}

// Kleiner Helper: cJSON → std::string + Newline + free.
// Defensive: cJSON_PrintUnformatted gibt NULL bei Heap-Alloc-Fail; jede
// std::string-Operation auf einem too-grossen Renderoutput kann bad_alloc
// werfen — Image ist mit -fno-exceptions gebaut, daher fuehrt jede Throw-Stelle
// zu __cxa_allocate_exception → abort() und Reboot. Wir bleiben hier defensiv:
// bei NULL-Print direkt einen 2-Byte-Fallback liefern statt einen weiteren
// std::string aus einem unguenstigen Kontext zu allokieren.
// Ab diesem Rueckstand (Bytes in der TxQueue) bekommt ein Client keinen
// Live-Push, bis er aufgeholt hat (push_tick_).
constexpr size_t kPushSkipBacklog = 1024;

std::string render_and_free(cJSON* obj) {
    char* s = cJSON_PrintUnformatted(obj);
    std::string out;
    if (s) {
        out.assign(s);
        cJSON_free(s);
        out.append("\r\n");
    } else {
        // Print konnte nicht allokieren — kein Throw provozieren. Statisches
        // Mini-Error-Event reicht; der Caller (transport->write) bekommt was
        // Gueltiges, der Client sieht eine maschinenlesbare Fehlermeldung.
        out = std::string("{\"event\":\"err\",\"msg\":\"lowmem\"}\r\n");
    }
    cJSON_Delete(obj);
    return out;
}

// Hex-String → Bytes. Tolerant gegen ' ', ':' (analog a-culfw `fromhex`).
// Returns Anzahl Bytes, 0 bei Fehler. max_out muss vorhanden sein.
size_t parse_hex(const char* s, uint8_t* out, size_t max_out) {
    size_t n = 0;
    int hi = -1;
    for (const char* p = s; *p && n < max_out; ++p) {
        char c = *p;
        if (c == ' ' || c == ':') continue;
        int nibble;
        if      (c >= '0' && c <= '9') nibble = c - '0';
        else if (c >= 'a' && c <= 'f') nibble = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') nibble = c - 'A' + 10;
        else return 0;          // ungültiges Zeichen
        if (hi < 0) hi = nibble;
        else { out[n++] = static_cast<uint8_t>((hi << 4) | nibble); hi = -1; }
    }
    if (hi >= 0) return 0;      // ungerade Anzahl Hex-Chars
    return n;
}

bool ieq(const char* a, const char* b) {
    while (*a && *b) {
        char x = *a, y = *b;
        if (x >= 'A' && x <= 'Z') x = static_cast<char>(x + 32);
        if (y >= 'A' && y <= 'Z') y = static_cast<char>(y + 32);
        if (x != y) return false;
        ++a; ++b;
    }
    return *a == 0 && *b == 0;
}

// esp_err_t → JSON-Errorstring. ESP_ERR_INVALID_STATE = LOVF (oder NotInRx —
// werden auf "LOVF" gemappt; LOVF ist der haeufigere Fall, ein Caller der
// Moritz-spezifisch erfahren will, dass der Chip nicht in RX ist, kann das
// ueber `?stats.moritz.tx_err` korrelieren).
const char* err_str(esp_err_t e) {
    switch (e) {
        case ESP_OK:                return nullptr;
        case ESP_ERR_INVALID_STATE: return "LOVF";
        case ESP_ERR_INVALID_SIZE:  return "LENERR";
        case ESP_ERR_INVALID_ARG:   return "INVALID_ARG";
        case ESP_ERR_TIMEOUT:       return "TIMEOUT";
        case ESP_ERR_NOT_SUPPORTED: return "NOT_SUPPORTED";
        case ESP_ERR_NO_MEM:        return "NO_MEM";
        default:                    return esp_err_to_name(e);
    }
}

} // anon

std::span<const ProtocolId> JsonFrontend::compatible_protocols() const {
    return std::span<const ProtocolId>(kCompatible, std::size(kCompatible));
}

void JsonFrontend::attach(const FrontendServices& services) {
    services_ = services;
    if (!mtx_) mtx_ = xSemaphoreCreateMutex();
}

void JsonFrontend::bind_transport(std::shared_ptr<Transport> t) {
    if (!t || !mtx_) return;
    xSemaphoreTake(mtx_, portMAX_DELAY);
    transports_.push_back(t);
    xSemaphoreGive(mtx_);
}

void JsonFrontend::attach_dynamic_transport(std::shared_ptr<Transport> t) {
    if (!t) return;
    // Antworten (dump ~28 KB) duerfen auf Abfluss warten: on_rx_data_ gibt
    // mtx_ vor der Befehlsbearbeitung frei, der Funk-RX-Task haengt also
    // nicht daran. Vor start(), damit schon die erste Anfrage so laeuft.
    t->set_reply_wait(true);
    bind_transport(t);
    // Den neuen Transport als RX-Callback verdrahten.
    Transport* raw = t.get();
    t->on_rx([this, raw](std::span<const uint8_t> bytes) {
        on_rx_data_(bytes, raw);
    });
    if (started_) {
        // Scheitert start() (TcpRxPool nimmt den Auftrag nicht an), schliesst
        // der Transport seinen Socket selbst, und on_closed kommt nie: ohne
        // detach bliebe er tot in der Liste (Review 0908 F1).
        if (t->start() != ESP_OK) {
            detach_dynamic_transport(raw);
            return;
        }
        send_hello_(raw);
    }
}

void JsonFrontend::detach_dynamic_transport(Transport* t) {
    if (!t || !mtx_) return;
    xSemaphoreTake(mtx_, portMAX_DELAY);
    for (size_t i = 0; i < transports_.size(); ++i) {
        if (transports_[i].get() == t) {
            transports_.erase(transports_.begin() + i);
            break;
        }
    }
    for (size_t i = 0; i < kMaxTransports; ++i) {
        if (lb_owner_[i] == t) {
            lb_owner_[i] = nullptr;
            line_buf_per_transport_[i].len  = 0;
            line_buf_per_transport_[i].drop = false;
        }
    }
    xSemaphoreGive(mtx_);
}

esp_err_t JsonFrontend::start() {
    if (started_) return ESP_OK;
    started_ = true;

    // RX-Frames vom Manager subscriben — fanout zu allen JSON-Transports.
    if (services_.rf_manager) {
        for (auto p : kCompatible) {
            services_.rf_manager->subscribe(p, [this](const RxEvent& ev) {
                on_rx_event_(ev);
            });
        }
    }

    // Eingang-Stream pro Transport verdrahten + Hello-Greeting senden.
    for (auto& t : transports_) {
        Transport* raw = t.get();
        t->on_rx([this, raw](std::span<const uint8_t> bytes) {
            on_rx_data_(bytes, raw);
        });
        t->start();
        send_hello_(raw);
    }
    // Live-Push-Timer starten — alle 5 s broadcasten wir die FLAG_READONLY-
    // Eintraege als {"event":"value",...}. Damit aktualisiert sich die Browser-
    // UI ohne Polling. Push laeuft auch wenn aktuell keine Clients haengen
    // (broadcast_ ist no-op bei leerem transports_).
    {
        esp_timer_create_args_t a = {};
        a.callback = &JsonFrontend::push_tick_trampoline_;
        a.arg      = this;
        a.name     = "json-push";
        esp_timer_handle_t h = nullptr;
        if (esp_timer_create(&a, &h) == ESP_OK) {
            push_timer_ = h;
            esp_timer_start_periodic(h, 5'000'000);   // 5 s
        }
    }

    ESP_LOGI(TAG, "ready, %u transport(s)", static_cast<unsigned>(transports_.size()));
    return ESP_OK;
}

void JsonFrontend::push_tick_trampoline_(void* arg) {
    static_cast<JsonFrontend*>(arg)->push_tick_();
}

void JsonFrontend::push_tick_() {
    if (transports_.empty() || !services_.config || !mtx_) return;
    const bool thin = push_pause_ && push_pause_();
    if (thin && !push_keep_) return;
    const size_t keep_len = thin ? std::strlen(push_keep_) : 0;
    // Liste einmal je Takt. Clients mit Rueckstand in ihrer TxQueue lassen
    // wir aus: der Push sind Live-Werte, der naechste kommt in 5 s. Sonst
    // wuchse ihr Rueckstand mit jedem Takt (~8 KB), bis der Puffer
    // ueberlaeuft und der Client getrennt wird.
    std::vector<std::shared_ptr<Transport>> snap;
    xSemaphoreTake(mtx_, portMAX_DELAY);
    snap = transports_;
    xSemaphoreGive(mtx_);
    services_.config->iter([&](const ConfigEntry& e) {
        if (!e.readonly()) return;
        if (thin && e.key.compare(0, keep_len, push_keep_) != 0) return;
        cJSON* o = cJSON_CreateObject();
        cJSON_AddStringToObject(o, "event", "value");
        cJSON_AddStringToObject(o, "key", e.key.c_str());
        cJSON_AddStringToObject(o, "value", services_.config->get(e.key).c_str());
        const std::string json = render_and_free(o);
        for (auto& t : snap) {
            if (t->tx_backlog() > kPushSkipBacklog) continue;
            t->write({reinterpret_cast<const uint8_t*>(json.data()), json.size()});
        }
    });
}

esp_err_t JsonFrontend::stop() {
    started_ = false;
    if (push_timer_) {
        esp_timer_stop(static_cast<esp_timer_handle_t>(push_timer_));
        esp_timer_delete(static_cast<esp_timer_handle_t>(push_timer_));
        push_timer_ = nullptr;
    }
    return ESP_OK;
}

void JsonFrontend::on_rx_data_(std::span<const uint8_t> bytes, Transport* src) {
    if (!mtx_) return;
    xSemaphoreTake(mtx_, portMAX_DELAY);

    // LineBuf zu Transport finden oder neu zuordnen.
    LineBuf* lb = nullptr;
    for (size_t i = 0; i < kMaxTransports; ++i) {
        if (lb_owner_[i] == src) { lb = &line_buf_per_transport_[i]; break; }
    }
    if (!lb) {
        for (size_t i = 0; i < kMaxTransports; ++i) {
            if (!lb_owner_[i]) {
                lb_owner_[i] = src;
                lb = &line_buf_per_transport_[i];
                lb->len  = 0;
                lb->drop = false;   // kein Ueberhang vom Vorbesitzer des Puffers
                break;
            }
        }
    }
    xSemaphoreGive(mtx_);
    if (!lb) return;

    for (uint8_t b : bytes) {
        if (b == '\r') continue;
        if (b == '\n') {
            // Ueberlange Zeile: Rest bis hierher verwerfen, nicht als neue Zeile
            // lesen (wie das culfw-Frontend seit 0.1.872 -- dort lief der
            // Ueberhang als eigener Befehl). Anders als dort wird die Zeile
            // selbst verworfen statt gekuerzt ausgefuehrt: ein angeschnittenes
            // JSON-Objekt ist nie parsebar.
            if (lb->drop) { lb->drop = false; lb->len = 0; continue; }
            if (lb->len > 0) {
                process_line_({lb->data.data(), lb->len}, src);
                lb->len = 0;
            }
            continue;
        }
        if (lb->drop) continue;
        if (lb->len < lb->data.size() - 1) {
            lb->data[lb->len++] = static_cast<char>(b);
        } else {
            // Overflow: Zeile verwerfen, Rest bis zum Zeilenende ignorieren.
            lb->len  = 0;
            lb->drop = true;
            send_err_(src, "line too long");
        }
    }
    // WebSocket-Frames sind message-oriented: ein Frame == eine Line ohne
    // \n-Trailer. Wenn nach dem Frame-Ende noch Bytes im Buffer stehen,
    // flushen wir sie hier als komplette Line. Ein TCP-Strom kommt dagegen
    // in beliebigen Stuecken (recv() liest je hoechstens 256 Byte) — dort
    // endet eine Zeile nur am \n, und der Buffer haelt sie ueber mehrere
    // Aufrufe. Bis 0.1.875 wurde auch dort geflusht: jede JSON-Zeile ueber
    // 256 Byte und jede auf zwei Segmente verteilte kam als Bruchstuecke an.
    // Ein zu langer Frame ist mit seinem Ende erledigt -- verworfen, nicht
    // geflusht.
    if (src->message_framed()) {
        if (lb->drop) {
            lb->drop = false;
            lb->len  = 0;
        } else if (lb->len > 0) {
            process_line_({lb->data.data(), lb->len}, src);
            lb->len = 0;
        }
    }
}

void JsonFrontend::process_line_(std::span<const char> line, Transport* src) {
    cJSON* root = cJSON_ParseWithLength(line.data(), line.size());
    if (!root) {
        send_err_(src, "invalid json");
        return;
    }
    cJSON* cmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");
    if (!cmd || !cJSON_IsString(cmd)) {
        send_err_(src, "missing cmd");
        cJSON_Delete(root);
        return;
    }
    const char* cmd_s = cmd->valuestring;

    if (std::strcmp(cmd_s, "ping") == 0) {
        cJSON* o = cJSON_CreateObject();
        cJSON_AddStringToObject(o, "event", "pong");
        emit_to_(src, render_and_free(o));
    } else if (std::strcmp(cmd_s, "get") == 0) {
        cJSON* k = cJSON_GetObjectItemCaseSensitive(root, "key");
        if (cJSON_IsString(k)) send_value_(src, k->valuestring);
        else                    send_err_(src, "missing key", "get");
    } else if (std::strcmp(cmd_s, "set") == 0) {
        cJSON* k = cJSON_GetObjectItemCaseSensitive(root, "key");
        cJSON* v = cJSON_GetObjectItemCaseSensitive(root, "value");
        if (!cJSON_IsString(k)) {
            send_err_(src, "missing key", "set");
        } else {
            std::string vs;
            if (cJSON_IsString(v)) vs = v->valuestring;
            else if (cJSON_IsNumber(v)) {
                char b[32];
                std::snprintf(b, sizeof(b), "%.0f", v->valuedouble);
                vs = b;
            } else if (cJSON_IsBool(v)) {
                vs = cJSON_IsTrue(v) ? "1" : "0";
            } else {
                send_err_(src, "missing value", "set");
                cJSON_Delete(root);
                return;
            }
            bool ok = services_.config && services_.config->set(k->valuestring, vs);
            send_ack_(src, "set", k->valuestring, ok);
        }
    } else if (std::strcmp(cmd_s, "dump") == 0) {
        send_dump_(src);
    } else if (std::strcmp(cmd_s, "tx") == 0) {
        handle_tx_(root, src);
    } else {
        send_err_(src, "unknown cmd");
    }
    cJSON_Delete(root);
}

void JsonFrontend::emit_to_(Transport* t, std::string_view json) {
    if (!t) return;
    t->write({reinterpret_cast<const uint8_t*>(json.data()), json.size()});
}

void JsonFrontend::broadcast_(std::string_view json) {
    if (!mtx_) return;
    // Snapshot der Liste unter Mutex, dann schreiben ausserhalb (vermeidet
    // Lock waehrend evtl. blockierendem Transport-Write).
    std::vector<std::shared_ptr<Transport>> snap;
    xSemaphoreTake(mtx_, portMAX_DELAY);
    snap = transports_;
    xSemaphoreGive(mtx_);
    for (auto& t : snap) {
        t->write({reinterpret_cast<const uint8_t*>(json.data()), json.size()});
    }
}

void JsonFrontend::send_hello_(Transport* t) {
    cJSON* o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "event", "hello");
    cJSON_AddStringToObject(o, "fw", "CULFW32");
    if (services_.config) {
        cJSON_AddNumberToObject(o, "registry_size", services_.config->size());
    }
    emit_to_(t, render_and_free(o));
}

void JsonFrontend::send_value_(Transport* t, std::string_view key) {
    if (!services_.config) { send_err_(t, "no registry"); return; }
    auto* e = services_.config->find(key);
    if (!e) { send_err_(t, "unknown key", std::string(key).c_str()); return; }
    auto val = services_.config->get(key);
    cJSON* o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "event", "value");
    cJSON_AddStringToObject(o, "key", std::string(key).c_str());
    cJSON_AddStringToObject(o, "value", val.c_str());
    emit_to_(t, render_and_free(o));
}

void JsonFrontend::send_ack_(Transport* t, const char* cmd, std::string_view key,
                              bool ok, const char* err) {
    cJSON* o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "event", "ack");
    cJSON_AddStringToObject(o, "cmd", cmd);
    cJSON_AddStringToObject(o, "key", std::string(key).c_str());
    cJSON_AddBoolToObject(o, "ok", ok);
    if (!ok && err) cJSON_AddStringToObject(o, "err", err);
    emit_to_(t, render_and_free(o));
}

void JsonFrontend::send_err_(Transport* t, const char* msg, const char* key) {
    cJSON* o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "event", "err");
    cJSON_AddStringToObject(o, "msg", msg);
    if (key) cJSON_AddStringToObject(o, "key", key);
    emit_to_(t, render_and_free(o));
}

void JsonFrontend::send_dump_(Transport* t) {
    if (!services_.config) { send_err_(t, "no registry"); return; }
    // Streaming-Dump statt monolithisch: pro Eintrag ein eigenes Event
    // (~200-400 B). Frueher hat ein single std::string-Buffer ~25 KB
    // alloziert, was bei Heap-Fragmentation einen bad_alloc → abort() im
    // libstdc++ ausloeste (Image ist mit -fno-exceptions, daher kein catch).
    // Stream-Pattern: dump_begin {count} → N × dump_item {...} → dump_end.
    // Der Web-UI-Client (web/app.js) sammelt items zwischen begin/end.
    cJSON* head = cJSON_CreateObject();
    cJSON_AddStringToObject(head, "event", "dump_begin");
    cJSON_AddNumberToObject(head, "count", services_.config->size());
    emit_to_(t, render_and_free(head));

    services_.config->iter([&](const ConfigEntry& e) {
        cJSON* it = cJSON_CreateObject();
        cJSON_AddStringToObject(it, "event", "dump_item");
        cJSON_AddStringToObject(it, "key", e.key.c_str());
        const char* type_s = "?";
        switch (e.type) {
            case ConfigEntry::Type::Bool:     type_s = "bool";     break;
            case ConfigEntry::Type::U8:       type_s = "u8";       break;
            case ConfigEntry::Type::U16:      type_s = "u16";      break;
            case ConfigEntry::Type::U32:      type_s = "u32";      break;
            case ConfigEntry::Type::Str:      type_s = "str";      break;
            case ConfigEntry::Type::Secret:   type_s = "secret";   break;
            case ConfigEntry::Type::Ipv4Cidr: type_s = "ipv4cidr"; break;
            case ConfigEntry::Type::IntEnum:  type_s = "intenum";  break;
        }
        cJSON_AddStringToObject(it, "type", type_s);
        cJSON_AddBoolToObject  (it, "persist",   e.persist());
        cJSON_AddBoolToObject  (it, "readonly",  e.readonly());
        cJSON_AddBoolToObject  (it, "sensitive", e.sensitive());
        cJSON_AddStringToObject(it, "value", services_.config->get(e.key).c_str());
        if (!e.help.empty()) cJSON_AddStringToObject(it, "help", e.help.c_str());
        if (!e.unit.empty()) cJSON_AddStringToObject(it, "unit", e.unit.c_str());
        if (e.type == ConfigEntry::Type::IntEnum && !e.enum_labels.empty()) {
            cJSON* labels = cJSON_AddArrayToObject(it, "enum_labels");
            for (auto& l : e.enum_labels) cJSON_AddItemToArray(labels, cJSON_CreateString(l.c_str()));
        }
        if (!e.hass_device_class.empty())
            cJSON_AddStringToObject(it, "hass_device_class", e.hass_device_class.c_str());
        emit_to_(t, render_and_free(it));
    });

    cJSON* tail = cJSON_CreateObject();
    cJSON_AddStringToObject(tail, "event", "dump_end");
    emit_to_(t, render_and_free(tail));
}

void JsonFrontend::send_tx_ack_(Transport* t, const char* proto, bool ok, const char* err) {
    cJSON* o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "event", "tx_ack");
    cJSON_AddStringToObject(o, "proto", proto ? proto : "?");
    cJSON_AddBoolToObject  (o, "ok", ok);
    if (!ok && err) cJSON_AddStringToObject(o, "err", err);
    emit_to_(t, render_and_free(o));
}

// {"cmd":"tx", "proto":"<name>", "hex":"<hex>", [protocol-specific extras]}
//
// Per Protocol:
//   moritz: hex byte0 = length-byte; bytes total = byte0+1; optional bool
//           "long_preamble" (Zs).
//   asksin: gleiches Schema (length-byte + payload).
//   slowrf: zusätzlich "kind":"fs20"|"ks300"|"em" plus "hex":"<bytes>";
//           optional "repeat":N (default 3).  EM erfordert exakt 9 Bytes.
//   fastrf: hex direkt (Length-Byte wird NICHT erwartet — fastrf packt das
//           Length-Byte selber im TXFIFO an byte0, analog FS20).
// Native ist nicht TX-faehig (ESP_ERR_NOT_SUPPORTED, NOT_SUPPORTED).
void JsonFrontend::handle_tx_(cJSON* root, Transport* src) {
    cJSON* proto_j = cJSON_GetObjectItemCaseSensitive(root, "proto");
    cJSON* hex_j   = cJSON_GetObjectItemCaseSensitive(root, "hex");
    if (!cJSON_IsString(proto_j) || !cJSON_IsString(hex_j)) {
        send_tx_ack_(src, "?", false, "missing proto/hex");
        return;
    }
    const char* proto_s = proto_j->valuestring;
    const char* hex_s   = hex_j->valuestring;

    auto* mgr = services_.rf_manager;
    if (!mgr) { send_tx_ack_(src, proto_s, false, "no manager"); return; }

    uint8_t bytes[64];
    size_t  nbytes = parse_hex(hex_s, bytes, sizeof(bytes));
    if (nbytes == 0) {
        send_tx_ack_(src, proto_s, false, "invalid hex");
        return;
    }

    // ProtocolId aus String. Wir aktivieren das Protokoll vor mgr->tx wenn es
    // nicht schon aktiv ist (analog ASCII-Frontend ensure_*_active). Ohne das
    // ist `drv_` in der RfProtocol-Klasse nullptr und tx() liefert INVALID_ARG.
    ProtocolId pid;
    if      (ieq(proto_s, "moritz")) pid = ProtocolId::Moritz;
    else if (ieq(proto_s, "asksin")) pid = ProtocolId::AskSin;
    else if (ieq(proto_s, "slowrf")) pid = ProtocolId::SlowRf;
    else if (ieq(proto_s, "fastrf")) pid = ProtocolId::FastRf;
    else if (ieq(proto_s, "native")) {
        send_tx_ack_(src, proto_s, false, "NOT_SUPPORTED"); return;
    } else {
        send_tx_ack_(src, proto_s, false, "unknown proto"); return;
    }
    auto active = mgr->active_protocol_on(0);
    if (!active || active->id() != pid) {
        if (auto ae = mgr->activate(pid, 0); ae != ESP_OK) {
            send_tx_ack_(src, proto_s, false, err_str(ae));
            return;
        }
    }

    esp_err_t e = ESP_ERR_NOT_SUPPORTED;

#ifdef CONFIG_CULFW32_BUILD_RF_MORITZ
    if (ieq(proto_s, "moritz") || ieq(proto_s, "Moritz")) {
        if (nbytes < 2 || nbytes > MoritzFrame::kMaxPayload) {
            send_tx_ack_(src, proto_s, false, "LENERR"); return;
        }
        // Length-Byte + payload-Konsistenz wie in handle_Z (frontend_culfw).
        if (bytes[0] + 1u != nbytes) {
            send_tx_ack_(src, proto_s, false, "LENERR"); return;
        }
        MoritzTxFrame f{};
        cJSON* lp = cJSON_GetObjectItemCaseSensitive(root, "long_preamble");
        f.long_preamble = cJSON_IsBool(lp) && cJSON_IsTrue(lp);
        f.len = bytes[0];
        std::memcpy(f.data, bytes, nbytes);
        e = mgr->tx(ProtocolId::Moritz, &f, sizeof(f));
        // Fuer Moritz die genauere Fehler-Diagnose aus last_tx_error() rausziehen.
        if (e != ESP_OK) {
            const char* err = err_str(e);
            // ESP_ERR_INVALID_STATE = "LOVF" (default mapping). MoritzRF setzt
            // bei NotInRx ebenfalls INVALID_STATE — wir koennen das hier
            // verfeinern.
            if (e == ESP_ERR_INVALID_STATE) {
                auto m = mgr->find_protocol(ProtocolId::Moritz);
                if (m && m->id() == ProtocolId::Moritz) {
                    auto* mr = static_cast<MoritzRF*>(m.get());
                    switch (mr->last_tx_error()) {
                        case MoritzRF::TxError::Lovf:    err = "LOVF"; break;
                        case MoritzRF::TxError::NotInRx: err = "NOT_IN_RX"; break;
                        default: break;
                    }
                }
            }
            send_tx_ack_(src, proto_s, false, err);
            return;
        }
        send_tx_ack_(src, proto_s, true, nullptr);
        return;
    }
#endif

#ifdef CONFIG_CULFW32_BUILD_RF_ASKSIN
    if (ieq(proto_s, "asksin")) {
        if (nbytes < 2 || nbytes > AskSinFrame::kMaxPayload) {
            send_tx_ack_(src, proto_s, false, "LENERR"); return;
        }
        if (bytes[0] + 1u != nbytes) {
            send_tx_ack_(src, proto_s, false, "LENERR"); return;
        }
        AskSinTxFrame f{};
        f.len = bytes[0];
        std::memcpy(f.data, bytes, nbytes);
        e = mgr->tx(ProtocolId::AskSin, &f, sizeof(f));
        send_tx_ack_(src, proto_s, e == ESP_OK, err_str(e));
        return;
    }
#endif

#ifdef CONFIG_CULFW32_BUILD_RF_SLOWRF
    if (ieq(proto_s, "slowrf")) {
        cJSON* kind_j = cJSON_GetObjectItemCaseSensitive(root, "kind");
        if (!cJSON_IsString(kind_j)) {
            send_tx_ack_(src, proto_s, false, "missing kind"); return;
        }
        const char* kind_s = kind_j->valuestring;
        SlowRfTxFrame f{};
        f.repeat  = 3;
        f.startcs = 0;
        if (ieq(kind_s, "fs20"))      { f.kind = SlowRfTxFrame::Kind::FS20;  f.startcs = 6; }
        else if (ieq(kind_s, "ks300")) f.kind = SlowRfTxFrame::Kind::KS300;
        else if (ieq(kind_s, "em"))    f.kind = SlowRfTxFrame::Kind::EM;
        else { send_tx_ack_(src, proto_s, false, "unknown kind"); return; }

        if (nbytes == 0 || nbytes > sizeof(f.data)) {
            send_tx_ack_(src, proto_s, false, "LENERR"); return;
        }
        // EM: strikt 9 Bytes (siehe SlowRfRF::tx + a-culfw em_send).
        if (f.kind == SlowRfTxFrame::Kind::EM && nbytes != 9) {
            send_tx_ack_(src, proto_s, false, "LENERR"); return;
        }
        // Optional repeat-Override (FHEM-typisch fuer K/F=3, EM=3; Caller darf
        // tunen). Clamp auf [1, 15].
        cJSON* rep_j = cJSON_GetObjectItemCaseSensitive(root, "repeat");
        if (cJSON_IsNumber(rep_j)) {
            int r = rep_j->valueint;
            if (r >= 1 && r <= 15) f.repeat = static_cast<uint8_t>(r);
        }
        f.len = static_cast<uint8_t>(nbytes);
        std::memcpy(f.data, bytes, nbytes);
        e = mgr->tx(ProtocolId::SlowRf, &f, sizeof(f));
        send_tx_ack_(src, proto_s, e == ESP_OK, err_str(e));
        return;
    }
#endif

#ifdef CONFIG_CULFW32_BUILD_RF_FASTRF
    if (ieq(proto_s, "fastrf")) {
        if (nbytes == 0 || nbytes > FastRfFrame::kMaxPayload) {
            send_tx_ack_(src, proto_s, false, "LENERR"); return;
        }
        FastRfTxFrame f{};
        f.len = static_cast<uint8_t>(nbytes);
        std::memcpy(f.data, bytes, nbytes);
        e = mgr->tx(ProtocolId::FastRf, &f, sizeof(f));
        send_tx_ack_(src, proto_s, e == ESP_OK, err_str(e));
        return;
    }
#endif

    if (ieq(proto_s, "native")) {
        // NativeRF::tx ist a-culfw-konform read-only — kein TX implementiert.
        send_tx_ack_(src, proto_s, false, "NOT_SUPPORTED");
        return;
    }

    send_tx_ack_(src, proto_s, false, "unknown proto");
}

void JsonFrontend::on_rx_event_(const RxEvent& ev) {
    if (transports_.empty()) return;     // nichts zu tun, sparen wir Heap-Allocs
    cJSON* o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "event", "rx");
    cJSON_AddStringToObject(o, "proto", proto_name(ev.proto));
    cJSON_AddNumberToObject(o, "slot",  ev.slot_id);
    cJSON_AddNumberToObject(o, "rssi_raw", ev.rssi_dbm);
    cJSON_AddNumberToObject(o, "ts_us", ev.timestamp_us);
    // Payload-spezifische Daten haengt am Protokoll-Typ — fuer jetzt nur den
    // Header. Ein vollwertiger Decoder pro ProtocolId (mit Hex-Dump der payload-
    // Bytes) kommt im Folge-Patch.
    broadcast_(render_and_free(o));
}

} // namespace culfw32
