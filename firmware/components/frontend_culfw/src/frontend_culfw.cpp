// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
// Portions derived from a-culfw (https://github.com/tostmann/a-culfw),
// GNU GPL version 2 or (at your option) any later version.

#include "frontend_culfw.hpp"
#include "culfw_version.hpp"
#include "rf_manager.hpp"
#include "nvs_eeprom.hpp"
#include "culfw_ee_addr.hpp"
#include "led_drv.hpp"
#include "moritz_rf.hpp"
#include "asksin_rf.hpp"
#include "slowrf_rf.hpp"
#include "native_rf.hpp"
#include "fastrf_rf.hpp"
#include "dudette_rf.hpp"
#ifdef CONFIG_CULFW32_BUILD_RF_WMBUS
#include "wmbus_rf.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_HMIP
#include "hmip_rf.hpp"
#endif
#include "wifi_manager.hpp"
#include "eth_manager.hpp"
#include "tcp_server.hpp"
#include "tcp_rx_pool.hpp"
#include "tcp_client_transport.hpp"
#include "transport_websocket.hpp"
#include "config_registry.hpp"
#include "rf_stats.hpp"
#include "system_monitor.hpp"
#include "transport_usbcdc.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cctype>
#include <array>

namespace culfw32 {

namespace {
constexpr const char* TAG = "culfw-emu";

constexpr ProtocolId kCompatible[] = {
    ProtocolId::AskSin,
    ProtocolId::Moritz,
    ProtocolId::SlowRf,
    ProtocolId::FastRf,
    ProtocolId::Native,
    ProtocolId::Mbus,
};

constexpr size_t kLineBufSize = 128;
constexpr size_t kMaxOutputLine = 256;

void write_prefixed(Transport* t, uint8_t slot_id, const char* s) {
    if (slot_id > 0) {
        char stars[8] = {};
        size_t n = (slot_id < sizeof(stars)) ? slot_id : sizeof(stars) - 1;
        std::memset(stars, '*', n);
        t->write({reinterpret_cast<const uint8_t*>(stars), n});
    }
    t->write({reinterpret_cast<const uint8_t*>(s), std::strlen(s)});
}

bool hex1(char c, uint8_t& out) {
    if (c >= '0' && c <= '9') { out = c - '0';      return true; }
    if (c >= 'a' && c <= 'f') { out = c - 'a' + 10; return true; }
    if (c >= 'A' && c <= 'F') { out = c - 'A' + 10; return true; }
    return false;
}

bool hex2(std::span<const char> in, uint8_t& out) {
    if (in.size() < 2) return false;
    uint8_t hi, lo;
    if (!hex1(in[0], hi) || !hex1(in[1], lo)) return false;
    out = (hi << 4) | lo;
    return true;
}

// Byte-genau a-culfw `fromhex(in, out, buflen)` (stringfunc.c:10).
// - liest Hex-Paare und legt sie in out ab
// - **skipt** ' ' (space) und ':' (colon) — sie zaehlen NICHT als char
// - bricht bei jedem anderen Nicht-Hex-Zeichen ab (\r/\n/Brief etc.)
// - case-insensitive (a-f und A-F)
// - "Single-Nibbles wont be converted": ungerade Hex-Anzahl am Ende -> letztes
//   nibble nicht gepusht
// Returns Anzahl tatsaechlich gepushter Bytes. Implementation-Konsistenz mit
// a-culfw ist Pflicht weil FHEM/Tools `Cw<aa> <vv>` und `W<aaaa> <vv>` mit
// Spaces dazwischen schicken.
// Ein Hex-Nibble oder -1.  parse_hex_run() bricht bei ungueltigen Zeichen ab,
// hier brauchen wir die Einzelpruefung (Adressen, Fragmentnummern).
int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

size_t parse_hex_run(std::span<const char> in, std::span<uint8_t> out) {
    size_t got = 0;
    uint8_t h = 0;
    int step = 0;
    for (size_t i = 0; i < in.size() && got < out.size(); ++i) {
        char c = in[i];
        if (c >= 'a' && c <= 'z') c = static_cast<char>(c - ('a' - 'A'));
        uint8_t nibble;
        if (c >= '0' && c <= '9')      nibble = static_cast<uint8_t>(c - '0');
        else if (c >= 'A' && c <= 'F') nibble = static_cast<uint8_t>(c - 'A' + 10);
        else {
            if (c == ' ' || c == ':') continue;
            break;
        }
        if (step) {
            out[got++] = h | nibble;
            h = 0;
            step = 0;
        } else {
            h = static_cast<uint8_t>(nibble << 4);
            step = 1;
        }
    }
    return got;
}
} // namespace

// Forward-Declaration: definiert weiter unten im Anonymous-Namespace, gemeinsam
// mit g_lines/g_lines_mtx/g_lines_count. Wird vom detach_dynamic_transport
// aufgerufen — getrennt deklariert weil die Member-Definition vor der
// Variablen-Definition steht.
namespace { void release_line_for(Transport* t); }

CulfwFrontend::CulfwFrontend() {
    transports_mtx_ = xSemaphoreCreateMutex();
}

std::span<const ProtocolId> CulfwFrontend::compatible_protocols() const {
    return {kCompatible, sizeof(kCompatible) / sizeof(kCompatible[0])};
}

void CulfwFrontend::attach(const FrontendServices& services) {
    services_ = services;
}

void CulfwFrontend::bind_transport(std::shared_ptr<Transport> t) {
    if (transports_mtx_) xSemaphoreTake(transports_mtx_, portMAX_DELAY);
    transports_.push_back(std::move(t));
    if (transports_mtx_) xSemaphoreGive(transports_mtx_);
}

std::vector<std::shared_ptr<Transport>> CulfwFrontend::transports_snapshot() {
    std::vector<std::shared_ptr<Transport>> snap;
    if (!transports_mtx_) return snap;
    xSemaphoreTake(transports_mtx_, portMAX_DELAY);
    snap = transports_;
    xSemaphoreGive(transports_mtx_);
    return snap;
}

esp_err_t CulfwFrontend::attach_dynamic_transport(std::shared_ptr<Transport> t) {
    if (!t) return ESP_ERR_INVALID_ARG;
    Transport* raw = t.get();
    // Antworten (??-Tabelle, ?stats) duerfen auf Abfluss warten: dispatch_line
    // laeuft ohne Frontend-Mutex (transports_mtx_ nur fuer Snapshots,
    // g_lines_mtx nur um den Zeilenpuffer), der Funk-RX-Task haengt also nicht
    // daran. Vor start(), damit schon die erste Zeile so beantwortet wird.
    t->set_reply_wait(true);
    t->on_rx([this, raw](std::span<const uint8_t> d) { handle_rx(d, raw); });
    // In die Liste VOR start(): der TcpRxPool-Worker (Prio 6) verdraengt den
    // Accept-Task (Prio 5) und erledigt eine kurze Verbindung (Anfrage,
    // Antwort, Trennen) samt on_closed -> detach_dynamic_transport, bevor
    // start() hierher zurueckkehrt. Bis 0.1.907 stand push_back dahinter:
    // detach fand nichts, der beendete Transport blieb in der Liste und hielt
    // seinen Socket bis zum Neustart (gemessen 15.10.2026).
    if (transports_mtx_) xSemaphoreTake(transports_mtx_, portMAX_DELAY);
    transports_.push_back(t);
    if (transports_mtx_) xSemaphoreGive(transports_mtx_);
    if (auto e = t->start(); e != ESP_OK) {
        detach_dynamic_transport(raw);
        return e;
    }
    return ESP_OK;
}

esp_err_t CulfwFrontend::detach_dynamic_transport(Transport* t) {
    if (!t) return ESP_ERR_INVALID_ARG;
    if (transports_mtx_) xSemaphoreTake(transports_mtx_, portMAX_DELAY);
    for (auto it = transports_.begin(); it != transports_.end(); ++it) {
        if (it->get() == t) { transports_.erase(it); break; }
    }
    if (transports_mtx_) xSemaphoreGive(transports_mtx_);
    // Auch den LineBuf freimachen — sonst bleibt der Slot belegt und
    // future TCP-Clients koennen nach 16 Connections nicht mehr ankoppeln.
    release_line_for(t);
    return ESP_OK;
}

// Moritz-Renderer im a-culfw-Format (rf_moritz.c:221).
//   Z<len:hex2><payload-bytes:hex2 each><rssi:hex2>\r\n
// Slot-Prefix `*` davor analog MULTICC_PREFIX().
static void render_moritz(const RxEvent& ev,
                          const std::vector<std::shared_ptr<Transport>>& transports) {
    auto* f = static_cast<const MoritzFrame*>(ev.payload);
    if (!f || f->len == 0 || f->len >= MoritzFrame::kMaxPayload) return;

    char buf[3 * (MoritzFrame::kMaxPayload + 4) + 4];   // Hex + RSSI + CRLF + Slack
    size_t pos = 0;
    auto append_hex = [&](uint8_t b) {
        if (pos + 2 < sizeof(buf)) {
            int n = std::snprintf(buf + pos, sizeof(buf) - pos, "%02X", b);
            if (n > 0) pos += static_cast<size_t>(n);
        }
    };

    buf[pos++] = 'Z';
    // a-culfw schreibt enc[0..len], also len+1 bytes (das Length-Byte selbst plus payload).
    for (size_t i = 0; i <= f->len; ++i) append_hex(f->data[i]);
    append_hex(f->rssi_raw);
    if (pos + 2 < sizeof(buf)) { buf[pos++] = '\r'; buf[pos++] = '\n'; }

    // Auf alle gebundenen Transports schreiben, mit Slot-Prefix.
    for (auto& t : transports) {
        write_prefixed(t.get(), ev.slot_id, "");   // nur Sterne
        t->write({reinterpret_cast<const uint8_t*>(buf), pos});
    }
}

// AskSin-Renderer im a-culfw-Format (rf_asksin.c:194ff).
//   A<len:hex2><payload-bytes:hex2 each><rssi:hex2>\r\n
// Slot-Prefix `*` davor analog MULTICC_PREFIX(). Frame ist bereits decrypted.
static void render_asksin(const RxEvent& ev,
                          const std::vector<std::shared_ptr<Transport>>& transports) {
    auto* f = static_cast<const AskSinFrame*>(ev.payload);
    if (!f || f->len == 0 || f->len >= AskSinFrame::kMaxPayload) return;

    char buf[3 * (AskSinFrame::kMaxPayload + 4) + 4];
    size_t pos = 0;
    auto append_hex = [&](uint8_t b) {
        if (pos + 2 < sizeof(buf)) {
            int n = std::snprintf(buf + pos, sizeof(buf) - pos, "%02X", b);
            if (n > 0) pos += static_cast<size_t>(n);
        }
    };

    buf[pos++] = 'A';
    for (size_t i = 0; i <= f->len; ++i) append_hex(f->data[i]);
    append_hex(f->rssi_raw);
    if (pos + 2 < sizeof(buf)) { buf[pos++] = '\r'; buf[pos++] = '\n'; }

    for (auto& t : transports) {
        write_prefixed(t.get(), ev.slot_id, "");   // nur Sterne
        t->write({reinterpret_cast<const uint8_t*>(buf), pos});
    }
}

// SlowRF-Renderer im a-culfw-Format (rf_receive.c:500ff).
//   F<bytes:hex2 each>[<rssi>]\r\n   für FS20
//   T<bytes>[<rssi>]\r\n             für FHT
//   K<bytes>[<rssi>]\r\n             für KS300 (mit Half-Nibble-Suffix wenn nibble)
//   E<bytes>[<rssi>]\r\n             für EM
//   H<bytes>[<rssi>]\r\n             für HMS (6 Bytes)
// Slot-Prefix `*` davor analog MULTICC_PREFIX().
// RSSI wird angehängt wenn tx_report&REP_RSSI gesetzt — Default: an wenn Bit 5
// in tx_report. Wir hängen es immer an (a-culfw default mit FHEM ist `X21` =
// REP_KNOWN|REP_RSSI), was FHEM korrekt verarbeitet.
static void render_slowrf(const RxEvent& ev,
                          const std::vector<std::shared_ptr<Transport>>& transports) {
    auto* f = static_cast<const SlowRfFrame*>(ev.payload);
    if (!f || f->len == 0 || f->len > sizeof(SlowRfFrame::data)) return;

    char tag = '?';
    switch (f->kind) {
        case SlowRfFrame::Kind::FS20:  tag = 'F'; break;
        case SlowRfFrame::Kind::FHT:   tag = 'T'; break;
        case SlowRfFrame::Kind::KS300: tag = 'K'; break;
        case SlowRfFrame::Kind::EM:    tag = 'E'; break;
        case SlowRfFrame::Kind::HMS:   tag = 'H'; break;
    }

    char buf[3 * (sizeof(SlowRfFrame::data) + 4) + 4];
    size_t pos = 0;
    auto append_hex = [&](uint8_t b) {
        if (pos + 2 < sizeof(buf)) {
            int n = std::snprintf(buf + pos, sizeof(buf) - pos, "%02X", b);
            if (n > 0) pos += static_cast<size_t>(n);
        }
    };

    buf[pos++] = tag;
    for (uint8_t i = 0; i < f->len; ++i) append_hex(f->data[i]);
    if (f->kind == SlowRfFrame::Kind::KS300 && f->nibble_half && f->len > 0) {
        // letztes Halb-Nibble nur als ein hex-char (a-culfw rf_receive.c:516).
        if (pos > 0) {
            // Strip last 2 hex chars (which were the full byte) and reappend as 1.
            pos -= 2;
            int n = std::snprintf(buf + pos, sizeof(buf) - pos, "%X",
                                  f->data[f->len - 1] & 0x0F);
            if (n > 0) pos += static_cast<size_t>(n);
        }
    }
    append_hex(f->rssi_raw);
    if (pos + 2 < sizeof(buf)) { buf[pos++] = '\r'; buf[pos++] = '\n'; }

    for (auto& t : transports) {
        write_prefixed(t.get(), ev.slot_id, "");
        t->write({reinterpret_cast<const uint8_t*>(buf), pos});
    }
}

// rf_fastrf-Renderer im a-culfw-Format (fastrf.c:84-90).
//   <raw-bytes>\r\n   (KEIN Hex-Encoding! a-culfw schreibt DC(buf[i]) direkt.)
// Slot-Prefix `*` davor analog MULTICC_PREFIX(). a-culfw flippt zudem den
// `display_channel = DISPLAY_USB` — wir haben das nicht, schreiben einfach
// auf alle gebundenen Transports (die's USB-CDC oder UART sein koennen).
static void render_fastrf(const RxEvent& ev,
                          const std::vector<std::shared_ptr<Transport>>& transports) {
    auto* f = static_cast<const FastRfFrame*>(ev.payload);
    if (!f || f->len == 0 || f->len > sizeof(FastRfFrame::data)) return;

    // Roh-Bytes + CRLF.
    std::vector<uint8_t> buf;
    buf.reserve(f->len + 2);
    buf.insert(buf.end(), f->data, f->data + f->len);
    buf.push_back('\r');
    buf.push_back('\n');

    for (auto& t : transports) {
        write_prefixed(t.get(), ev.slot_id, "");
        t->write({buf.data(), buf.size()});
    }
}

// rf_native-Renderer im a-culfw-Format (rf_native.c:192).
//   N<mode-hex2><payload-bytes-hex>\r\n
// Slot-Prefix `*` davor analog MULTICC_PREFIX(). Kein RSSI im Frame (PKTCTRL1=0,
// kein RSSI/LQI append) — a-culfw schreibt auch keinen RSSI bei N.
static void render_native(const RxEvent& ev,
                          const std::vector<std::shared_ptr<Transport>>& transports) {
    auto* f = static_cast<const NativeFrame*>(ev.payload);
    if (!f || f->len == 0 || f->len > sizeof(NativeFrame::data)) return;

    char buf[3 * (NativeFrame::kMaxPayload + 4) + 8];
    size_t pos = 0;
    auto append_hex = [&](uint8_t b) {
        if (pos + 2 < sizeof(buf)) {
            int n = std::snprintf(buf + pos, sizeof(buf) - pos, "%02X", b);
            if (n > 0) pos += static_cast<size_t>(n);
        }
    };

    buf[pos++] = 'N';
    append_hex(f->mode);
    for (uint8_t i = 0; i < f->len; ++i) append_hex(f->data[i]);
    if (pos + 2 < sizeof(buf)) { buf[pos++] = '\r'; buf[pos++] = '\n'; }

    for (auto& t : transports) {
        write_prefixed(t.get(), ev.slot_id, "");
        t->write({reinterpret_cast<const uint8_t*>(buf), pos});
    }
}

#ifdef CONFIG_CULFW32_BUILD_RF_WMBUS
// rf_mbus-Renderer im a-culfw-Format (rf_mbus.c:353):
//   b<hex bytes>[<lqi-hex2><rssi-hex2>]\r\n
// Slot-Prefix `*` davor analog MULTICC_PREFIX(). RSSI/LQI immer angehaengt
// (a-culfw default `X21` = REP_KNOWN|REP_RSSI; CULFW32 ist immer "an").
static void render_wmbus(const RxEvent& ev,
                         const std::vector<std::shared_ptr<Transport>>& transports) {
    auto* f = static_cast<const WmbusFrame*>(ev.payload);
    if (!f || f->len == 0 || f->len > sizeof(WmbusFrame::data)) return;

    char buf[3 * (WmbusFrame::kMaxPacket + 4) + 4];
    size_t pos = 0;
    auto append_hex = [&](uint8_t b) {
        if (pos + 2 < sizeof(buf)) {
            int n = std::snprintf(buf + pos, sizeof(buf) - pos, "%02X", b);
            if (n > 0) pos += static_cast<size_t>(n);
        }
    };

    buf[pos++] = 'b';
    for (uint16_t i = 0; i < f->len; ++i) append_hex(f->data[i]);
    append_hex(f->lqi_raw);
    append_hex(f->rssi_raw);
    if (pos + 2 < sizeof(buf)) { buf[pos++] = '\r'; buf[pos++] = '\n'; }

    for (auto& t : transports) {
        write_prefixed(t.get(), ev.slot_id, "");
        t->write({reinterpret_cast<const uint8_t*>(buf), pos});
    }
}
#endif




#ifdef CONFIG_CULFW32_BUILD_RF_HMIP
// rf_hmip-Renderer (PHY-only, kein Frame-Decoder, kein Decrypt).
//
//   P<LEN:hex2><payload-bytes:hex2 each><rssi:hex2><lqi:hex2> t=<ms> n=<seq>\r\n
//
// Positional-Teil spiegelt die AskSin-A-Zeile: LEN-included (f->data[0]=LEN
// zuerst), RSSI+LQI HINTEN — so greift dieselbe (LEN+1)*2-Laengen-Logik, die
// ein Host-Sniffer fuer A-Zeilen nutzt.
//
// Tag 'P' (mnemonic hmIP) statt 'H' — 'H' ist in a-culfw fuer HMS-Wireless-
// Sensoren belegt; FHEM 12_HMS.pm wuerde daraus Phantom-Devices anlegen.
// Vereinbarung Dirk 2026-05-03.
//
// Kein Header-Decode: die Zeile traegt nur die Rohbytes, ausgewertet wird beim
// Host. Die Nutzlast bleibt verschluesselt; entschluesselt wird hier nichts.
static void render_hmip(const RxEvent& ev,
                        const std::vector<std::shared_ptr<Transport>>& transports) {
    auto* f = static_cast<const HmipFrame*>(ev.payload);
    if (!f || f->len == 0 || f->len >= HmipFrame::kMaxPayload) return;

    char buf[3 * (HmipFrame::kMaxPayload + 4) + 96];   // Hex + RSSI/LQI + t=/n= + CRLF
    size_t pos = 0;
    auto append_hex = [&](uint8_t b) {
        if (pos + 2 < sizeof(buf)) {
            int n = std::snprintf(buf + pos, sizeof(buf) - pos, "%02X", b);
            if (n > 0) pos += static_cast<size_t>(n);
        }
    };

    buf[pos++] = 'P';
    for (size_t i = 0; i <= f->len; ++i) append_hex(f->data[i]);
    append_hex(f->rssi_raw);
    append_hex(f->lqi_raw);

    // Korrelations-Felder fuer Analyse. Der Hex-Teil davor bleibt UNVERAENDERT,
    // damit jeder Parser, der "Hex bis zum ersten Leerzeichen" liest, weiterlaeuft.
    //   t=<ms seit Boot>  monotone Empfangszeit -> Retransmit-Bursts sind am
    //                     zeitlichen Abstand erkennbar.
    //   n=<rx-counter>    fortlaufend pro gerendertem Frame -> Luecken (=
    //                     verlorene Frames) werden sichtbar.
    {
        static uint32_t s_hmip_rx_seq = 0;
        const unsigned long long t_ms =
            static_cast<unsigned long long>(esp_timer_get_time() / 1000);
        const unsigned long n_seq = static_cast<unsigned long>(++s_hmip_rx_seq);
        if (pos + 2 < sizeof(buf)) {
            int nw = std::snprintf(buf + pos, sizeof(buf) - pos,
                                   " t=%llu n=%lu", t_ms, n_seq);
            if (nw > 0) {
                pos += static_cast<size_t>(nw);
                if (pos > sizeof(buf) - 2) pos = sizeof(buf) - 2;  // Truncation-Guard
            }
        }
    }

    if (pos + 2 < sizeof(buf)) { buf[pos++] = '\r'; buf[pos++] = '\n'; }

    for (auto& t : transports) {
        write_prefixed(t.get(), ev.slot_id, "");
        t->write({reinterpret_cast<const uint8_t*>(buf), pos});
    }
}
#endif // CONFIG_CULFW32_BUILD_RF_HMIP

void CulfwFrontend::register_codecs() {
    if (!services_.rf_manager) return;
    // Dudette meldet nur Debug-Zeilen einer Anwendung hierher (kTypePrint);
    // HELLOs landen in der Bestandsliste und werden mit `Dl` gelesen.
    services_.rf_manager->subscribe(ProtocolId::Dudette,
        [this](const RxEvent& ev) {
            auto* p = static_cast<const DudetteRxEvent*>(ev.payload);
            if (!p || !p->text) return;
            char buf[96];
            std::snprintf(buf, sizeof(buf), "Dp %06lX %s\r\n",
                          static_cast<unsigned long>(p->addr), p->text);
            auto s = transports_snapshot();
            for (auto& t : s)
                if (t) t->write({reinterpret_cast<const uint8_t*>(buf),
                                 std::strlen(buf)});
        });
    services_.rf_manager->subscribe(ProtocolId::Moritz,
        [this](const RxEvent& ev) { auto s = transports_snapshot(); render_moritz(ev, s); });
    services_.rf_manager->subscribe(ProtocolId::AskSin,
        [this](const RxEvent& ev) { auto s = transports_snapshot(); render_asksin(ev, s); });
    services_.rf_manager->subscribe(ProtocolId::SlowRf,
        [this](const RxEvent& ev) { auto s = transports_snapshot(); render_slowrf(ev, s); });
    services_.rf_manager->subscribe(ProtocolId::Native,
        [this](const RxEvent& ev) { auto s = transports_snapshot(); render_native(ev, s); });
    services_.rf_manager->subscribe(ProtocolId::FastRf,
        [this](const RxEvent& ev) { auto s = transports_snapshot(); render_fastrf(ev, s); });
#ifdef CONFIG_CULFW32_BUILD_RF_WMBUS
    services_.rf_manager->subscribe(ProtocolId::Mbus,
        [this](const RxEvent& ev) { auto s = transports_snapshot(); render_wmbus(ev, s); });
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_HMIP
    services_.rf_manager->subscribe(ProtocolId::Hmip,
        [this](const RxEvent& ev) { auto s = transports_snapshot(); render_hmip(ev, s); });
#endif
}

namespace {

void handle_V(std::span<const char> args, Transport* src, uint8_t slot_id, RfManager* mgr) {
    if (!args.empty() && args[0] == 'H') {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s\r\n", CULFW_HW_REVISION);
        write_prefixed(src, slot_id, buf);
        return;
    }
    if (!args.empty() && args[0] == 'r') {
        auto rd_cfg = [&](uint8_t addr) -> uint8_t {
            uint8_t v = 0xFF; mgr->cc_read_reg(slot_id, addr, v); return v;
        };
        auto rd_st  = [&](uint8_t addr) -> uint8_t {
            uint8_t v = 0xFF; mgr->cc_read_status(slot_id, addr, v); return v;
        };
        uint8_t partnum = rd_st(0x30), version = rd_st(0x31);
        uint8_t lqi     = rd_st(0x33), rssi    = rd_st(0x34);
        uint8_t marcs   = rd_st(0x35);
        uint8_t f2 = rd_cfg(0x0D), f1 = rd_cfg(0x0E), f0 = rd_cfg(0x0F);
        uint8_t m4 = rd_cfg(0x10), m3 = rd_cfg(0x11), m2 = rd_cfg(0x12);
        uint8_t dev = rd_cfg(0x15);
        uint8_t s1 = rd_cfg(0x04), s0 = rd_cfg(0x05);
        uint8_t pkt = rd_cfg(0x08);

        char buf[kMaxOutputLine];
        std::snprintf(buf, sizeof(buf),
            "Vr P=%02X V=%02X MS=%02X FREQ=%02X:%02X:%02X MDM=%02X:%02X:%02X "
            "DEV=%02X SYNC=%02X:%02X PKT=%02X RSSI=%02X LQI=%02X\r\n",
            partnum, version, marcs, f2, f1, f0, m4, m3, m2,
            dev, s1, s0, pkt, rssi, lqi);
        write_prefixed(src, slot_id, buf);
        return;
    }
    if (!args.empty() && args[0] == 'R') {
        // VR — Klartext-Status: Chip, Frequenz, BW, Modulation, Datarate,
        // Deviation, State, aktives Protokoll. Formeln aus CC1101-Datenblatt
        // SWRS061I §16 (Frequenz), §13 (Datarate), §13.1 (BW), §16.1 (Deviation).
        auto rd_cfg = [&](uint8_t a) -> uint8_t {
            uint8_t v = 0xFF; mgr->cc_read_reg(slot_id, a, v); return v;
        };
        auto rd_st  = [&](uint8_t a) -> uint8_t {
            uint8_t v = 0xFF; mgr->cc_read_status(slot_id, a, v); return v;
        };

        // Chip-Detect (CC1101: PARTNUM=0x00 VERSION=0x14 laut SWRS061I).
        // Wenn beide 0xFF → SPI liefert nichts (kein Chip oder falsche Pin-Map).
        // Wir bestimmen das vor allen Frequenz-/Modulations-Decodes, damit auf
        // toter SPI-Leitung keine 6.6-GHz-Phantasiewerte gerendert werden.
        uint8_t partnum = rd_st(0x30), version = rd_st(0x31);
        const char* chip_name = "?";
        bool chip_alive = !(partnum == 0xFF && version == 0xFF);
        if (chip_alive && partnum == 0x00) {
            switch (version) {
                case 0x04: chip_name = "CC1100";       break;  // Vorgaenger
                case 0x14: chip_name = "CC1101";       break;
                case 0x17: chip_name = "CC1101-Q1";    break;
                default:   chip_name = "CC11xx";       break;
            }
        }

        if (!chip_alive) {
            char buf[96];
            std::snprintf(buf, sizeof(buf),
                "VR no-chip (PARTNUM=FF VERSION=FF -- SPI/Pin-Map?)\r\n");
            write_prefixed(src, slot_id, buf);
            return;
        }

        constexpr uint64_t F_XOSC = 26'000'000ULL;

        uint32_t freq_word = (uint32_t(rd_cfg(0x0D)) << 16)
                           | (uint32_t(rd_cfg(0x0E)) << 8)
                           |  uint32_t(rd_cfg(0x0F));
        uint64_t freq_hz = (uint64_t(freq_word) * F_XOSC) >> 16;

        uint8_t mdm4 = rd_cfg(0x10);
        uint8_t mdm3 = rd_cfg(0x11);
        uint8_t mdm2 = rd_cfg(0x12);
        uint8_t devr = rd_cfg(0x15);

        uint8_t chanbw_e = (mdm4 >> 6) & 0x03;
        uint8_t chanbw_m = (mdm4 >> 4) & 0x03;
        uint32_t bw_hz = static_cast<uint32_t>(F_XOSC) /
                         (8u * (4u + chanbw_m) * (1u << chanbw_e));

        uint8_t  drate_e = mdm4 & 0x0F;
        uint8_t  drate_m = mdm3;
        uint64_t drate_bps = ((uint64_t(256u + drate_m) << drate_e) * F_XOSC) >> 28;

        uint8_t mod_format = (mdm2 >> 4) & 0x07;
        const char* mod_name = "?";
        switch (mod_format) {
            case 0: mod_name = "2-FSK"; break;
            case 1: mod_name = "GFSK";  break;
            case 3: mod_name = "OOK";   break;
            case 4: mod_name = "4-FSK"; break;
            case 7: mod_name = "MSK";   break;
        }

        uint8_t  dev_e = (devr >> 4) & 0x07;
        uint8_t  dev_m = devr & 0x07;
        uint32_t dev_hz = static_cast<uint32_t>(((F_XOSC >> 17) * (8u + dev_m)) << dev_e);

        uint8_t marcs = rd_st(0x35) & 0x1F;
        const char* state_str = "?";
        switch (marcs) {
            case 0x01: state_str = "IDLE";  break;
            case 0x0D: state_str = "RX";    break;
            case 0x13: state_str = "TX";    break;
            case 0x11: state_str = "RXOVF"; break;
            case 0x16: state_str = "TXUDF"; break;
        }

        // RSSI byte-genau CC1101-DB SWRS061I §17.3. Nur in RX-State valide;
        // bei IDLE/TX gibt der Chip einen Sentinel (typisch 0x80 → -138 dBm).
        // In RX = aktueller Noisefloor (wenn kein Frame anliegt) bzw. Signal-
        // staerke (wenn gerade ein Frame demoduliert wird).
        uint8_t rssi_raw = rd_st(0x34);
        bool    rssi_valid = (marcs >= 0x0D && marcs <= 0x10);
        int     rssi_dbm = (rssi_raw < 128)
            ? (static_cast<int>(rssi_raw) / 2) - 74
            : ((static_cast<int>(rssi_raw) - 256) / 2) - 74;

        const char* proto_name = "none";
        if (auto p = mgr->active_protocol_on(slot_id)) {
            switch (p->id()) {
                case ProtocolId::AskSin:       proto_name = "AskSin";    break;
                case ProtocolId::Moritz:       proto_name = "Moritz";    break;
                case ProtocolId::SlowRf:       proto_name = "SlowRf";    break;
                case ProtocolId::PulseCapture: proto_name = "PulseCap";  break;
                case ProtocolId::FastRf:       proto_name = "FastRf";    break;
                case ProtocolId::Native:       proto_name = "Native";    break;
                case ProtocolId::Mbus:         proto_name = "wMBus";     break;
                case ProtocolId::Hmip:         proto_name = "HmIP";      break;
                case ProtocolId::Dudette:      proto_name = "Dudette";   break;
            }
        }

        char rssi_str[24];
        if (rssi_valid) std::snprintf(rssi_str, sizeof(rssi_str), "%d dBm (raw 0x%02X)", rssi_dbm, rssi_raw);
        else            std::snprintf(rssi_str, sizeof(rssi_str), "n/a (only valid in RX)");

        char buf[260];
        std::snprintf(buf, sizeof(buf),
            "VR %s(V=%02X) %s %lu.%03lu MHz / %s @ %lu.%02lu kbps / BW %lu.%lu kHz / "
            "dev %lu.%01lu kHz / %s / RSSI %s\r\n",
            chip_name, version,
            proto_name,
            static_cast<unsigned long>(freq_hz / 1'000'000ULL),
            static_cast<unsigned long>((freq_hz / 1000ULL) % 1000ULL),
            mod_name,
            static_cast<unsigned long>(drate_bps / 1000ULL),
            static_cast<unsigned long>((drate_bps % 1000ULL) / 10ULL),
            static_cast<unsigned long>(bw_hz / 1000ULL),
            static_cast<unsigned long>((bw_hz % 1000ULL) / 100ULL),
            static_cast<unsigned long>(dev_hz / 1000ULL),
            static_cast<unsigned long>((dev_hz % 1000ULL) / 100ULL),
            state_str, rssi_str);
        write_prefixed(src, slot_id, buf);
        return;
    }
    // ` Dudette=1` am Ende: Faehigkeitskennung fuer Klienten der D-Befehle
    // (Dudette doc/protocol.md §10.2, Grammatik v1).  Klienten erkennen den
    // Stick daran statt am Firmwarenamen.  Letztes Wort und ohne Punkt:
    // FHEM 00_CUL.pm verlangt nur `^V`, 14_CUL_MAX zerlegt die Zeile gierig am
    // LETZTEN Punkt — ein Punkt im Token verschoebe dessen Versionszahl.
#ifdef CONFIG_CULFW32_BUILD_RF_DUDETTE
    constexpr const char* kDudetteCap = " Dudette=1";
#else
    constexpr const char* kDudetteCap = "";
#endif
    char buf[160];
    std::snprintf(buf, sizeof(buf),
        "V %s %s Build: %s (%s) %s (F-Band: %s)%s\r\n",
        CULFW_VERSION, CULFW_FW_NAME, CULFW_BUILD_NUM, CULFW_BUILD_DATE,
        CULFW_BOARD_ID, CULFW_FBAND, kDudetteCap);
    write_prefixed(src, slot_id, buf);
}

void handle_C(std::span<const char> args, Transport* src, uint8_t slot_id, RfManager* mgr) {
    if (args.empty()) return;

    if (args[0] == 'w') {
        // a-culfw `Cw <aa> <vv>`: addr + value via fromhex, Spaces werden
        // skipped. Seit parse_hex_run a-culfw-byte-genau ist (skipt ' '/':'),
        // reicht der gleiche Pfad — keine Hand-Parsing-Sonderlocke mehr noetig.
        uint8_t pair[2] = {};
        if (parse_hex_run(args.subspan(1), {pair, 2}) != 2) return;
        mgr->cc_write_reg(slot_id, pair[0], pair[1]);
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%02X%02X\r\n", pair[0], pair[1]);
        write_prefixed(src, slot_id, buf);
        return;
    }

    uint8_t hb;
    if (!hex2(args, hb)) return;

    if (hb == 0x99) {
        char buf[64];
        size_t pos = 0;
        for (uint8_t i = 0; i < 0x30; ++i) {
            uint8_t v = 0xFF;
            mgr->cc_read_reg(slot_id, i, v);
            int n = std::snprintf(buf + pos, sizeof(buf) - pos, "%02X", v);
            if (n > 0) pos += static_cast<size_t>(n);
            if ((i & 7) == 7) {
                std::snprintf(buf + pos, sizeof(buf) - pos, "\r\n");
                write_prefixed(src, slot_id, buf);
                pos = 0;
            }
        }
        return;
    }

    uint8_t v = 0xFF;
    mgr->cc_read_reg(slot_id, hb, v);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "C%02X = %02X / %2u\r\n", hb, v, v);
    write_prefixed(src, slot_id, buf);
}

// R-Befehl (read_eeprom). Format aus a-culfw fncollection.c:108
//   R<aa>     -> "Raaaa = vv / ddd\r\n"   (8-bit Adresse, output immer 4 Hex-chars)
//   R<aaaa>   -> "Raaaa = vv / ddd\r\n"   (16-bit Adresse)
// Multi-CC: a-culfw macht slot-spezifisches Adress-Mapping fuer EE_CC1100_CFG-Region.
// In CULFW32 v1 mit nur Slot 0 nicht relevant — bei *R fehlendem Slot: "R NA\r\n".
// a-culfw stringfunc.c fromip(): Dezimalzahlen, getrennt durch '.' oder ' ',
// bis zu `n` Bytes; bricht beim ersten anderen Zeichen ab.
size_t culfw_fromip(std::span<const char> in, uint8_t* out, size_t n) {
    size_t got = 0;
    uint8_t h = 0;
    bool fnd = false;
    for (char c : in) {
        fnd = false;
        if (c >= '0' && c <= '9') {
            h = static_cast<uint8_t>(h * 10 + (c - '0'));
            fnd = true;
        } else {
            if (c != ' ' && c != '.') break;
            if (got < n) {
                out[got++] = h;
                h = 0;
            }
        }
    }
    if (fnd && got < n) out[got++] = h;
    return got;
}

// a-culfw stringfunc.c fromdec(): alle Ziffern der Zeile, 16 bit.
uint16_t culfw_fromdec(std::span<const char> in) {
    uint16_t h = 0;
    for (char c : in) {
        if (c >= '0' && c <= '9') h = static_cast<uint16_t>(h * 10 + (c - '0'));
    }
    return h;
}

// `Wi<x><wert>` — a-culfw fncollection.c write_eeprom(), HAS_ETHERNET-Zweig:
//   Wim<12 hex>  MAC (':' und ' ' erlaubt)   Wid<dez>  DHCP 0/1
//   Wia/Win/Wig/WiN <a.b.c.d>  IP/Netzmaske/Gateway/NTP-Server
//   Wip<dez>     TCP-Port (2 Byte LE)        Wio<hex>  NTP-Offset
// Silent wie a-culfw. Wirksam nach `Ei` bzw. (Port) nach dem naechsten Boot.
// Abweichung: a-culfw schreibt immer die volle Laenge, auch wenn weniger
// geparst wurde (Rest = Stack-Inhalt). Hier wird nur bei vollstaendigem Wert
// geschrieben. `WimFFFFFFFFFFFF` stellt die MAC auf automatisch zurueck.
void handle_Wi(std::span<const char> args, NvsEeprom* ee) {
    if (args.empty()) return;
    const char sub = args[0];
    const auto val = args.subspan(1);
    uint8_t hb[6] = {};
    switch (sub) {
        case 'm':
            if (parse_hex_run(val, {hb, 6}) == 6) ee->write_block(ee::MAC_ADDR, {hb, 6});
            return;
        case 'd':
            ee->write_byte(ee::USE_DHCP, static_cast<uint8_t>(culfw_fromdec(val) & 0xFF));
            return;
        case 'a': case 'n': case 'g': case 'N': {
            const uint16_t addr = sub == 'a' ? ee::IP4_ADDR
                                : sub == 'n' ? ee::IP4_NETMASK
                                : sub == 'g' ? ee::IP4_GATEWAY
                                :              ee::IP4_NTPSERVER;
            if (culfw_fromip(val, hb, 4) == 4) ee->write_block(addr, {hb, 4});
            return;
        }
        case 'p': {
            const uint16_t port = culfw_fromdec(val);
            hb[0] = static_cast<uint8_t>(port & 0xFF);
            hb[1] = static_cast<uint8_t>(port >> 8);
            ee->write_block(ee::IP4_TCPLINK_PORT, {hb, 2});
            return;
        }
        case 'o':
            if (parse_hex_run(val, {hb, 1}) == 1) ee->write_byte(ee::IP4_NTPOFFSET, hb[0]);
            return;
        default:
            return;
    }
}

// `Ri<x>` — a-culfw read_eeprom(), HAS_ETHERNET-Zweig. Ausgabe byte-genau:
//   Rim AA:BB:CC:DD:EE:FF   Rid/Rio zwei Hex-Ziffern   Rip dezimal ohne Padding
//   Ria/Rin/Rig/RiN a.b.c.d
// Rim bei unbeschriebener MAC (FF..FF = automatisch): die wirksame MAC, wie
// sie ein a-culfw-EEPROM nach dem Werksreset enthaelt (busware-MAC aus eFuse
// BLOCK3, sonst die Chip-MAC). Unbekannter Unterbefehl: nur "\r\n" (a-culfw DNL).
void handle_Ri(std::span<const char> args, Transport* src, uint8_t slot_id, NvsEeprom* ee) {
    char buf[40] = "";
    const char sub = args.empty() ? '\0' : args[0];
    auto ip_at = [&](uint16_t a) {
        std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
                      ee->read_byte(a), ee->read_byte(a + 1),
                      ee->read_byte(a + 2), ee->read_byte(a + 3));
    };
    switch (sub) {
        case 'm': {
            uint8_t m[6];
            bool unset = true;
            for (int i = 0; i < 6; ++i) {
                m[i] = ee->read_byte(static_cast<uint16_t>(ee::MAC_ADDR + i));
                unset = unset && m[i] == 0xFF;
            }
            if (unset) EthManager::effective_mac(ee, m);
            std::snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
                          m[0], m[1], m[2], m[3], m[4], m[5]);
            break;
        }
        case 'd': std::snprintf(buf, sizeof(buf), "%02X", ee->read_byte(ee::USE_DHCP));      break;
        case 'o': std::snprintf(buf, sizeof(buf), "%02X", ee->read_byte(ee::IP4_NTPOFFSET)); break;
        case 'a': ip_at(ee::IP4_ADDR);      break;
        case 'n': ip_at(ee::IP4_NETMASK);   break;
        case 'g': ip_at(ee::IP4_GATEWAY);   break;
        case 'N': ip_at(ee::IP4_NTPSERVER); break;
        case 'p':
            std::snprintf(buf, sizeof(buf), "%u",
                          static_cast<unsigned>(ee->read_byte(ee::IP4_TCPLINK_PORT) |
                                                (ee->read_byte(ee::IP4_TCPLINK_PORT + 1) << 8)));
            break;
        default:
            break;
    }
    std::strncat(buf, "\r\n", sizeof(buf) - std::strlen(buf) - 1);
    write_prefixed(src, slot_id, buf);
}

void handle_R(std::span<const char> args, Transport* src, uint8_t slot_id,
              RfManager* mgr, NvsEeprom* ee) {
    if (!ee || args.empty()) return;
    (void)mgr;
    if (args[0] == 'i') {
        handle_Ri(args.subspan(1), src, slot_id, ee);
        return;
    }
    uint8_t parsed[2] = {};
    size_t got = parse_hex_run(args, {parsed, 2});
    if (got == 0) return;
    uint16_t addr = (got >= 2) ? static_cast<uint16_t>((parsed[0] << 8) | parsed[1]) : parsed[0];

    uint8_t v = ee->read_byte(addr);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "R%04X = %02X / %2u\r\n", addr, v, v);
    write_prefixed(src, slot_id, buf);
}

// W-Befehl (write_eeprom). Format aus a-culfw fncollection.c:166
//   W<aa><vv>     -> silent (kein Output, wie a-culfw)
//   W<aaaa><vv>   -> silent
// Wir senden zur Bestaetigung "Waaaa<vv>\r\n" — a-culfw-conform ist der silent-Mode,
// FHEM erwartet keine Antwort. Aber fuer Lab-Debug ist eine Bestaetigung praktisch.
// Loesung: Echo nur wenn `Wv` (W-with-verbose) — oder ganz weglassen?
// Pragmatisch: silent wie a-culfw. Verifizierung erfolgt via R<aa> nach W<aa><vv>.
// Z-Befehl (Moritz / MAX!) — vollstaendiger Sub-Befehl-Satz aus a-culfw moritz_func.
//   Zr        -> activate(Moritz) auf slot_id
//   Zs<hex>   -> send frame mit longPreamble (TODO: Preamble noch nicht honoriert)
//   Zf<hex>   -> send frame ohne longPreamble (== Zs aktuell)
//   Za<hex6>  -> set Auto-Ack-Address (3 Byte hex)
//   Zw<hex6>  -> set Wall-Thermostat-Fake-Address (3 Byte hex)
//   Z<other>  -> off (deactivate)
// Error-Outputs byte-genau a-culfw: LENERR / ZERR1<MS> / ZERR2<MS> / ZERR3<MS>.
void handle_Z(std::span<const char> args, Transport* src, uint8_t slot_id,
              RfManager* mgr, NvsEeprom* eeprom) {
    auto get_moritz = [&]() -> MoritzRF* {
        // RTTI ist aus — `id()` validiert + static_cast.
        auto p = mgr->find_protocol(ProtocolId::Moritz);
        if (!p || p->id() != ProtocolId::Moritz) return nullptr;
        return static_cast<MoritzRF*>(p.get());
    };

    if (args.empty()) {
        // Kein Sub-Tag: deaktivieren wie a-culfw `else`-Zweig.
        if (get_moritz()) mgr->deactivate(slot_id);
        return;
    }

    char sub = args[0];
    auto rest = args.subspan(1);

    if (sub == 'r') {
        if (auto e = mgr->activate(ProtocolId::Moritz, slot_id); e != ESP_OK) {
            char buf[48];
            std::snprintf(buf, sizeof(buf), "Zr ERR %s\r\n", esp_err_to_name(e));
            write_prefixed(src, slot_id, buf);
            return;
        }
        // Erfolg ist stumm wie a-culfw moritz_func(): FHEM schickt beim Init
        // `X21\nZr` und gleich danach `T01` und nimmt die naechste Zeile als
        // FHTID (00_CUL.pm CUL_DoInit, CUL_ReadAnswer ohne Muster).
        return;
    }

    if (sub == 's' || sub == 'f') {
        // Send: hex-bytes parsen, ans MoritzRF::tx via Manager-Hook reichen.
        auto* m = get_moritz();
        if (!m) {
            write_prefixed(src, slot_id, "Z ERR\r\n");
            return;
        }
        MoritzTxFrame frame{};
        frame.long_preamble = (sub == 's');
        // Anzahl moeglicher Bytes (paarweise).
        size_t got = parse_hex_run(rest, {frame.data, MoritzFrame::kMaxPayload});
        if (got < 1) return;
        frame.len = frame.data[0];
        // a-culfw: (hblen-1) != dec[0] -> LENERR.
        if (got != static_cast<size_t>(frame.len) + 1u) {
            write_prefixed(src, slot_id, "LENERR\r\n");
            return;
        }
        if (auto e = mgr->tx(ProtocolId::Moritz, &frame, sizeof(frame)); e != ESP_OK) {
            char buf[32];
            uint8_t ms = m->last_tx_marcstate();
            switch (m->last_tx_error()) {
                case MoritzRF::TxError::NotInRx:
                    std::snprintf(buf, sizeof(buf), "ZERR1%02X\r\n", ms); break;
                case MoritzRF::TxError::NotInTx:
                    std::snprintf(buf, sizeof(buf), "ZERR2%02X\r\n", ms); break;
                case MoritzRF::TxError::NoReturnRx:
                    std::snprintf(buf, sizeof(buf), "ZERR3%02X\r\n", ms); break;
                case MoritzRF::TxError::Lovf:
                    std::snprintf(buf, sizeof(buf), "LOVF\r\n"); break;
                default:
                    std::snprintf(buf, sizeof(buf), "Z ERR\r\n"); break;
            }
            write_prefixed(src, slot_id, buf);
            return;
        }
        // a-culfw silent on success — wir auch.
        return;
    }

    if (sub == 'a' || sub == 'w') {
        auto* m = get_moritz();
        if (!m) return;
        uint8_t addr[3] = {};
        if (parse_hex_run(rest, {addr, 3}) != 3) return;
        if (sub == 'a') m->set_auto_ack_addr({addr, 3});
        else            m->set_wall_thermostat({addr, 3});
        // CULFW32-Extension: Adresse persistieren (a-culfw hat hier keine
        // EE-Persistenz — wir machen das ueber eigene Adressen ab 0x100).
        if (eeprom) {
            eeprom->write_block(sub == 'a' ? ee::MORITZ_AA_ADDR : ee::MORITZ_WT_ADDR,
                                std::span<const uint8_t>{addr, 3});
        }
        // a-culfw silent — wir auch.
        return;
    }

    // Z<other> -> off (a-culfw: alles was nicht r/s/f/a/w ist deaktiviert Moritz).
    if (get_moritz()) mgr->deactivate(slot_id);
}

// A-Befehl (AskSin / HomeMatic) — Sub-Befehl-Satz aus a-culfw asksin_func.
//   Ar        -> activate(AskSin) auf slot_id
//   As<hex>   -> send frame (Crypt + CCA + BURST-Preamble nach Bedarf)
//   A<other>  -> off (deactivate)
// AR (FUP-Mode) ist nicht in v1; siehe asksin_plan.md.
// Error-Outputs: LENERR (analog Moritz, a-culfw hat das auskommentiert),
// ERR:CCA (a-culfw rf_asksin.c:265 byte-genau), AERR1<MS>/AERR3<MS> fuer die
// uebrigen TX-Pfade (ZERR-Stil aus Moritz uebernommen).
void handle_A(std::span<const char> args, Transport* src, uint8_t slot_id, RfManager* mgr) {
    auto get_asksin = [&]() -> AskSinRF* {
        auto p = mgr->find_protocol(ProtocolId::AskSin);
        if (!p || p->id() != ProtocolId::AskSin) return nullptr;
        return static_cast<AskSinRF*>(p.get());
    };

    if (args.empty()) {
        if (get_asksin()) mgr->deactivate(slot_id);
        return;
    }

    char sub = args[0];
    auto rest = args.subspan(1);

    if (sub == 'r' || sub == 'R') {
        // a-culfw rf_asksin.c:322 — `Ar` = Reception normal, `AR` = Reception
        // mit Firmware-Update-Profil (~50 kbps statt ~10 kbps).
        auto* a = get_asksin();
        if (!a) {
            write_prefixed(src, slot_id, "A ERR\r\n");
            return;
        }
        a->set_update_mode(sub == 'R');
        ProtocolId target = ProtocolId::AskSin;
        // KEIN Kurzschluss auf "ist schon aktiv" — er hat den Empfaenger
        // still getoetet.  `Ax` ruft AskSinRF::stop() DIREKT: der Chip geht
        // auf SIDLE und `drv_` wird genullt (beide Klassen tun das).  Der Manager erfaehrt davon
        // nichts und haelt das Protokoll weiter fuer aktiv, also nahm ein
        // spaeteres `Ar` diese Abkuerzung, meldete `ok` und rief start()
        // NIE.  Ergebnis: MARCSTATE bleibt 0x01 (IDLE), es kommt nichts mehr an,
        // und `Vr` zeigt trotzdem eine gueltige Konfiguration — SIDLE fasst die
        // Register nicht an.  Ein Empfaenger, der `ok` sagt und nichts hoert,
        // ist schlimmer als einer, der das Radio einmal zu oft neu anwirft.
        // `Dr` und `Zr` aktivieren seit jeher bedingungslos und sind deshalb
        // nicht betroffen — sie waren die Gegenprobe.
        // Gemessen 07.09.2026 am CUL32 #2 (C35 = MARCSTATE):
        //   AskSin  frisch 0D · nach Ax 01 · nach Ax+Ar 01   <- taub
        //   HmIP    frisch 0D · nach Px 01 · nach Px+Pr 01   <- taub
        //   Dudette frisch 0D · nach Dx 01 · nach Dx+Dr 0D   <- erholt sich
        // Preis: ein wiederholtes `Ar` initialisiert das Radio neu und
        // verwirft einen gerade laufenden Empfang.  Das ist die richtige
        // Richtung: `Ar` heisst "empfange", nicht "melde, dass du es tust".
        if (auto e = mgr->activate(target, slot_id); e != ESP_OK) {
            char buf[48];
            std::snprintf(buf, sizeof(buf), "%c ERR %s\r\n", sub, esp_err_to_name(e));
            write_prefixed(src, slot_id, buf);
            return;
        }
        // Erfolg ist stumm wie a-culfw asksin_func(): FHEM schickt beim Init
        // `X21\nAr` und gleich danach `T01` und nimmt die naechste Zeile als
        // FHTID (00_CUL.pm CUL_DoInit, CUL_ReadAnswer ohne Muster) — mit
        // "Ar ok" schrieb es bei jedem Verbindungsaufbau T01<id>.
        return;
    }

    if (sub == 's') {
        auto* a = get_asksin();
        if (!a) {
            write_prefixed(src, slot_id, "A ERR\r\n");
            return;
        }
        AskSinTxFrame frame{};
        size_t got = parse_hex_run(rest, {frame.data, AskSinFrame::kMaxPayload});
        if (got < 1) return;
        frame.len = frame.data[0];
        if (got != static_cast<size_t>(frame.len) + 1u) {
            write_prefixed(src, slot_id, "LENERR\r\n");
            return;
        }
        if (auto e = mgr->tx(ProtocolId::AskSin, &frame, sizeof(frame)); e != ESP_OK) {
            char buf[32];
            uint8_t ms = a->last_tx_marcstate();
            switch (a->last_tx_error()) {
                case AskSinRF::TxError::NotInRx:
                    std::snprintf(buf, sizeof(buf), "AERR1%02X\r\n", ms); break;
                case AskSinRF::TxError::CcaTimeout:
                    std::snprintf(buf, sizeof(buf), "ERR:CCA\r\n"); break;
                case AskSinRF::TxError::NoReturnRx:
                    std::snprintf(buf, sizeof(buf), "AERR3%02X\r\n", ms); break;
                default:
                    std::snprintf(buf, sizeof(buf), "A ERR\r\n"); break;
            }
            write_prefixed(src, slot_id, buf);
            return;
        }
        // a-culfw silent on success.
        return;
    }

    // A<other> -> off (a-culfw `else`-Zweig in asksin_func).
    if (get_asksin()) mgr->deactivate(slot_id);
}

void handle_W(std::span<const char> args, Transport* src, uint8_t slot_id,
              RfManager* mgr, NvsEeprom* ee) {
    if (!ee || args.empty()) return;
    (void)mgr; (void)slot_id;
    if (args[0] == 'i') {
        handle_Wi(args.subspan(1), ee);
        return;
    }
    // a-culfw fromhex(in+1, hb, 3): liest 2 oder 3 Bytes.
    //   2 Bytes => addr=hb[0] (8-bit), value=hb[1]
    //   3 Bytes => addr=(hb[0]<<8)|hb[1] (16-bit), value=hb[2]
    uint8_t parsed[3] = {};
    size_t got = parse_hex_run(args, {parsed, 3});
    if (got < 2) return;

    uint16_t addr;
    uint8_t  value;
    if (got == 2) {
        addr  = parsed[0];
        value = parsed[1];
    } else {
        addr  = static_cast<uint16_t>((parsed[0] << 8) | parsed[1]);
        value = parsed[2];
    }
    ee->write_byte(addr, value);
    // silent — a-culfw konform.
    (void)src;
}

// SlowRF-Helfer: SlowRf-Klasse holen + auf gewünschtem Slot aktivieren wenn
// nicht schon aktiv. Idempotent — wenn schon SlowRf auf dem Slot läuft,
// kein Re-Init.
static SlowRfRF* ensure_slowrf_active(uint8_t slot_id, RfManager* mgr) {
    auto p = mgr->find_protocol(ProtocolId::SlowRf);
    if (!p || p->id() != ProtocolId::SlowRf) return nullptr;
    auto active = mgr->active_protocol_on(slot_id);
    if (!active || active->id() != ProtocolId::SlowRf) {
        if (mgr->activate(ProtocolId::SlowRf, slot_id) != ESP_OK) return nullptr;
    }
    return static_cast<SlowRfRF*>(p.get());
}

// F-Befehl (FS20 send) — a-culfw rf_send.c:243 fs20send → addParityAndSend(in, 6, 3).
//   F<addr-cmd-hex>     2..4 bytes (housecode? gerätecode + cmd) — wir akzeptieren
//                       beliebige hex-byte-Sequenz und schicken raw mit cksum1(6).
// Repeat = 3 (FS20-Standard), startcs = 6.
void handle_F(std::span<const char> args, Transport* src, uint8_t slot_id, RfManager* mgr) {
    auto* sr = ensure_slowrf_active(slot_id, mgr);
    if (!sr) {
        write_prefixed(src, slot_id, "F ERR\r\n");
        return;
    }
    SlowRfTxFrame frame{};
    frame.kind    = SlowRfTxFrame::Kind::FS20;
    frame.startcs = 6;
    frame.repeat  = 3;
    size_t got = parse_hex_run(args, {frame.data, sizeof(frame.data)});
    if (got < 2) return;
    frame.len = static_cast<uint8_t>(got);
    if (auto e = mgr->tx(ProtocolId::SlowRf, &frame, sizeof(frame)); e != ESP_OK) {
        if (e == ESP_ERR_INVALID_STATE) {
            write_prefixed(src, slot_id, "LOVF\r\n");
        } else {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "F ERR %s\r\n", esp_err_to_name(e));
            write_prefixed(src, slot_id, buf);
        }
        return;
    }
    // a-culfw silent on success.
}

// K-Befehl (KS300 send) — a-culfw rf_send.c:323 `ks_send` byte-genau.
//   K<hex>      Roh-Sensor-Bytes (typisch 7..9), cksum3 wird automatisch angehaengt.
// Repeat = 3 (a-culfw KS-Standard), kein startcs (cksum3 hat keinen Startwert).
void handle_K(std::span<const char> args, Transport* src, uint8_t slot_id, RfManager* mgr) {
    auto* sr = ensure_slowrf_active(slot_id, mgr);
    if (!sr) {
        write_prefixed(src, slot_id, "K ERR\r\n");
        return;
    }
    SlowRfTxFrame frame{};
    frame.kind    = SlowRfTxFrame::Kind::KS300;
    frame.startcs = 0;     // cksum3 ungenutzt
    frame.repeat  = 3;
    size_t got = parse_hex_run(args, {frame.data, sizeof(frame.data)});
    if (got < 1) return;
    frame.len = static_cast<uint8_t>(got);
    if (auto e = mgr->tx(ProtocolId::SlowRf, &frame, sizeof(frame)); e != ESP_OK) {
        if (e == ESP_ERR_INVALID_STATE) {
            write_prefixed(src, slot_id, "LOVF\r\n");
        } else {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "K ERR %s\r\n", esp_err_to_name(e));
            write_prefixed(src, slot_id, buf);
        }
        return;
    }
    // a-culfw silent on success.
}

// FHT-Hauscode (a-culfw fht.c:49 `fht_hc0`/`fht_hc1`). 80b nutzt nur hc0,
// 8v nutzt beide. Beim Boot aus EE_FHTID restauriert (CulfwFrontend::start),
// beim Setter `T01<hh><cc>` aktualisiert + persistent.
//
// Single-Writer-Invariante: geschrieben nur (a) einmalig beim Boot in
// CulfwFrontend::start() vor dem Dispatch-Loop und (b) im T01-Set-Pfad des
// Kommando-Handlers handle_T. Beide laufen im selben Frontend-Kommando-Kontext,
// nie nebenlaeufig — daher kein atomic/Lock noetig. Wird das je aus einem
// zweiten Task geschrieben, muessen diese Scalars std::atomic werden.
uint8_t g_fht_hc0 = 0;
uint8_t g_fht_hc1 = 0;

// T-Befehl (FHT) — a-culfw fht.c:152 `fhtsend()`.
//   T01<hh><cc>   set Hauscode (EE_FHTID = hh,cc) + fht_init
//   T01           query Hauscode → "<hh><cc>\r\n"   (4 hex + CRLF)
//   T02/T03/T10/T11/T12   80b/8v/TF Buffer-Queries (NICHT in v1; stilles "\r\n")
//   T<hex>        l>=4 Bytes: senden via FS20-Encoder (cksum1=12, repeat=2)
//
// a-culfw-Quirk: fhtsend gibt am Ende des Sub-Befehl-Pfades immer DNL() (= \r\n)
// aus — auch fuer leere/nicht-implementierte Subs. Reproduzieren wir.
void handle_T(std::span<const char> args, Transport* src, uint8_t slot_id,
              RfManager* mgr, NvsEeprom* eeprom) {
    // Erst-mal hex-parsen: a-culfw `l = fromhex(in+1, hb, 5)` (max 5 Bytes).
    uint8_t hb[6] = {};
    size_t l = parse_hex_run(args, {hb, 5});

    if (l < 4) {
        // Sub-Befehl-Modus. a-culfw fht.c:160: `if (l < 4)` Block.
        // Set: hb[0]==1 && l==3 → silent return (kein DNL).
        // Sonst: Query/Display + DNL am Ende des Blocks.
        if (hb[0] == 0x01 && l == 3) {
            g_fht_hc0 = hb[1];
            g_fht_hc1 = hb[2];
            if (eeprom) {
                eeprom->write_byte(ee::FHTID,     g_fht_hc0);
                eeprom->write_byte(ee::FHTID + 1, g_fht_hc1);
            }
            return;
        }
        if (hb[0] == 0x01) {
            // Query (hb[0]==1 mit l != 3): 4 hex chars + CRLF wie a-culfw
            // fht.c:172-175. Lokales-CUL zeigt: `T01`/`T011`/`T01XX` treffen
            // alle diesen Pfad.
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%02X%02X\r\n",
                          g_fht_hc0, g_fht_hc1);
            write_prefixed(src, slot_id, buf);
            return;
        }
        // Andere Subs (T02/T03/T10/T11/T12) sind in v1 nicht implementiert —
        // wir geben den a-culfw-DNL ohne Inhalt aus. Lokales-CUL gibt mit
        // HAS_FHT_80b/8v echte Daten zurueck; FHEM-FHT-Modul erwartet die,
        // also folgt das, sobald die FHT-Buffer/8v-Tabellen kommen.
        write_prefixed(src, slot_id, "\r\n");
        return;
    }

    // Send-Modus (>=4 Bytes Hex). FS20-Encoder mit cksum-Start 12, repeat=2
    // (a-culfw `addParityAndSend(in, FHT_CSUM_START=12, 2)`).
    auto* sr = ensure_slowrf_active(slot_id, mgr);
    if (!sr) {
        write_prefixed(src, slot_id, "T ERR\r\n");
        return;
    }
    SlowRfTxFrame frame{};
    frame.kind    = SlowRfTxFrame::Kind::FS20;
    frame.startcs = 12;
    frame.repeat  = 2;
    frame.len     = static_cast<uint8_t>(l);
    std::memcpy(frame.data, hb, l);
    if (auto e = mgr->tx(ProtocolId::SlowRf, &frame, sizeof(frame)); e != ESP_OK) {
        if (e == ESP_ERR_INVALID_STATE) {
            write_prefixed(src, slot_id, "LOVF\r\n");
        } else {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "T ERR %s\r\n", esp_err_to_name(e));
            write_prefixed(src, slot_id, buf);
        }
        return;
    }
}

// X-Befehl (set_txreport) — a-culfw rf_receive.c:160.
//   X            -> Antwort "<TX_REPORT-hex2><credit-decimal-5>\r\n"
//   X<hex2>      -> setze TX_REPORT, RX an wenn != 0
// Bitmaske: REP_KNOWN=01, REP_REPEATED=02, REP_BITS=04, REP_MONITOR=08,
//           REP_BINTIME=10, REP_RSSI=20, REP_FHTPROTO=40, REP_LCDMON=80
void handle_X(std::span<const char> args, Transport* src, uint8_t slot_id, RfManager* mgr) {
    auto p = mgr->find_protocol(ProtocolId::SlowRf);
    auto* slowrf = (p && p->id() == ProtocolId::SlowRf) ? static_cast<SlowRfRF*>(p.get()) : nullptr;
    if (args.empty()) {
        // Query-Mode — Format byte-genau a-culfw rf_receive.c:165 `DH2(TX_REPORT); DU(credit_10ms,5); DNL();`.
        // Reine Abfrage wie in a-culfw set_txreport(): KEIN Moduswechsel. FHEM
        // fragt so `credit10ms` ab; bis 0.1.866 schaltete das nackte X nach
        // SlowRF und machte einen Stick in AskSin/MAX!/wMBus taub, bis wieder
        // Ar/Zr/br kam (gemessen 08.10.2026: `Zf` danach -> "Z ERR").
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%02X%5u\r\n",
                      slowrf ? slowrf->tx_report() : 0,
                      (unsigned)RfStats::instance().credit());
        write_prefixed(src, slot_id, buf);
        return;
    }
    uint8_t mask = 0;
    if (!hex2(args, mask)) return;
    // a-culfw set_txrestore(): solange wMBus empfaengt (mbus_mode != NONE), nur
    // die Maske setzen und den CC1101 nicht anfassen ("that leads to a crash"
    // dort, hier waere wMBus weg). Nach `brx` bleibt das Protokoll eingetragen,
    // empfaengt aber nicht -- dann schaltet X<mask> wie dort auf SlowRF.
#ifdef CONFIG_CULFW32_BUILD_RF_WMBUS
    // wmbus_rf.hpp ist nur mit dem Modul eingebunden (Zeile 16).
    auto active = mgr->active_protocol_on(slot_id);
    if (active && active->id() == ProtocolId::Mbus &&
        static_cast<WmbusRF*>(active.get())->rx_active()) {
        if (slowrf) slowrf->set_tx_report(mask);
        return;
    }
#endif
    // a-culfw set_txrestore(): X<maske != 0> ruft set_ccon() -> ccInitChip(
    // EE_CC1100_CFG), laedt die Register also jedes Mal neu aus dem EEPROM --
    // so uebernimmt FHEM ein per `W` gesetztes bWidth/rAmpl/sens. activate()
    // auf die schon aktive Klasse ist genau das (stop, SRES, start unter der
    // Slot-Sperre). Bis 0.2.0 lud X21 bei aktivem SlowRF nichts.
    SlowRfRF* sr = nullptr;
    if (mask != 0) {
        if (slowrf && mgr->activate(ProtocolId::SlowRf, slot_id) == ESP_OK) sr = slowrf;
    } else {
        sr = ensure_slowrf_active(slot_id, mgr);
    }
    if (sr) sr->set_tx_report(mask);
    // a-culfw silent on success.
}

// `t`: Uptime-Ticks byte-genau a-culfw clock.c `gettime()`.
//   a-culfw zaehlt 1 Tick = 1/125 s (8 ms) im AVR-T2-IRQ. Wir nehmen
//   esp_timer_get_time() (us-monotonic) und konvertieren auf 125-Hz-Ticks,
//   damit FHEM-Module mit der gleichen Skalierung arbeiten koennen.
//   Output: 8 Hex-Chars (uint32 big-endian) + \r\n. Bei Overflow ueber
//   ~9.5 Jahre wrappt es — wie a-culfw.
void handle_t(std::span<const char> /*args*/, Transport* src, uint8_t slot_id) {
    uint64_t us = static_cast<uint64_t>(esp_timer_get_time());
    uint32_t ticks = static_cast<uint32_t>((us / 8000ULL) & 0xFFFFFFFFULL);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02X%02X%02X%02X\r\n",
                  static_cast<uint8_t>((ticks >> 24) & 0xFF),
                  static_cast<uint8_t>((ticks >> 16) & 0xFF),
                  static_cast<uint8_t>((ticks >>  8) & 0xFF),
                  static_cast<uint8_t>( ticks        & 0xFF));
    write_prefixed(src, slot_id, buf);
}

// `B[xx]`: Reset (a-culfw `prepare_boot`). Optional 1-byte hex Argument:
//   B00 (default) → einfacher Reset
//   Bff           → endless-loop (a-culfw "Allow testing", triggert Watchdog)
//   sonst         → reset; AVR-Pfad setzt EE_REQBL=1 und springt zum Bootloader.
//                   Auf ESP32 gibt es keine software-getriggerte Bootloader-
//                   Selektion (Strapping-Pin/UF2-Knopf), trotzdem schreiben wir
//                   EE_REQBL byte-genau a-culfw — Tools/FHEM koennen das lesen
//                   und auf der finalen Hardware kann ein zweiter MCU darauf
//                   reagieren (z.B. die ESP32-C6 Reset-Line passend treiben).
void handle_B(std::span<const char> args, Transport* src, uint8_t slot_id, NvsEeprom* eeprom) {
    uint8_t bl = 0;
    if (!args.empty()) hex2(args, bl);
    if (bl == 0xFF) {
        // a-culfw "Allow testing" — endless loop, triggert Watchdog-Reset.
        for (;;) {}
    }
    if (eeprom) eeprom->write_byte(ee::REQBL, bl ? 1 : 0);
    // Letztes ACK noch rausschreiben bevor der Reset kommt.
    write_prefixed(src, slot_id, "B reset\r\n");
    // Kurz warten damit USB-CDC-/UART-FIFO leer ist.
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
}

// `l[<xx>]`: LED-Mode. byte-genau a-culfw `ledfunc()` als Setter (silent on
// success), inkl. EEPROM-Persistenz (a-culfw `ewb(EE_LED, led_mode)`). Beim
// Boot liest main.cpp EE_LED zurueck und ruft set_mode() — Mode ueberlebt Reset.
// CULFW32-Extension: `l` ohne Argument gibt aktuellen Mode als 2-Hex + CRLF
// (analog `X` query).
//   0=off, 1=on, 2..4=Pattern (siehe led_drv.cpp).
void handle_l(std::span<const char> args, Transport* src, uint8_t slot_id,
              LedDriver* led, NvsEeprom* eeprom) {
    if (!led) return;
    if (args.empty()) {
        char buf[8];
        std::snprintf(buf, sizeof(buf), "%02X\r\n", led->mode());
        write_prefixed(src, slot_id, buf);
        return;
    }
    uint8_t mode = 0;
    if (!hex2(args, mode)) return;
    led->set_mode(mode);
    if (eeprom) eeprom->write_byte(ee::LED, mode);
    // a-culfw silent on success bei Setter-Pfad.
}

// EM-Senden (a-culfw rf_send.c em_send). EM ist strikt 9 Bytes Payload;
// a-culfw selbst hat den `LENERR\r\n`-Output auskommentiert (silent return) —
// wir aktivieren ihn aktiv, damit Tippfehler nicht still verschluckt werden.
// Erreichbar als `E<hex>` (CULFW32-Bestand) und als `M<hex>` (a-culfw CUL.c
// und CUNO.c fntab). `tag` steht nur in den Fehlermeldungen.
void em_send(std::span<const char> args, Transport* src, uint8_t slot_id,
             RfManager* mgr, const char* tag) {
    char buf[40];
    if (!mgr) {
        std::snprintf(buf, sizeof(buf), "%s ERR rf unavail\r\n", tag);
        write_prefixed(src, slot_id, buf);
        return;
    }
    auto* sr = ensure_slowrf_active(slot_id, mgr);
    if (!sr) {
        std::snprintf(buf, sizeof(buf), "%s ERR\r\n", tag);
        write_prefixed(src, slot_id, buf);
        return;
    }
    SlowRfTxFrame frame{};
    frame.kind    = SlowRfTxFrame::Kind::EM;
    frame.startcs = 0;
    frame.repeat  = 3;
    size_t got = parse_hex_run(args, {frame.data, sizeof(frame.data)});
    if (got != 9) {
        // a-culfw-Verbesserung: aktiv LENERR statt silent.
        write_prefixed(src, slot_id, "LENERR\r\n");
        return;
    }
    frame.len = 9;
    if (auto e = mgr->tx(ProtocolId::SlowRf, &frame, sizeof(frame)); e != ESP_OK) {
        if (e == ESP_ERR_INVALID_STATE) {
            write_prefixed(src, slot_id, "LOVF\r\n");
        } else {
            std::snprintf(buf, sizeof(buf), "%s ERR %s\r\n", tag, esp_err_to_name(e));
            write_prefixed(src, slot_id, buf);
        }
        return;
    }
    // a-culfw silent on success.
}

// `M<hex>`: em_send wie in a-culfw (CUL.c:101, CUNO.c:64).
void handle_M(std::span<const char> args, Transport* src, uint8_t slot_id, RfManager* mgr) {
    em_send(args, src, slot_id, mgr, "M");
}

// a-culfw CUNO eth_func() (ethernet.c:168), hier fuer den W5500 an J1:
//   Ei  Netz neu aufsetzen — liest die Wi*-Einstellungen aus dem EEPROM. Silent.
//   Ec  "<ip> <mac>\r\n", IP dezimal, MAC AA:BB:CC:DD:EE:FF (display_ip4/_mac).
//   Ed  Debug-Stufe 0..3 weiterzaehlen, Ausgabe zweistellig hex (DH2). Wirkung
//       hier: Log-Level der Ethernet-Tags, keine Paket-Dumps wie a-culfw.
//   En  NTP — nicht portiert.
// Ohne W5500 (Platine nicht gesteckt): "E ERR eth unavail".
void handle_eth(char sub, Transport* src, uint8_t slot_id, EthManager* eth) {
    auto write_line = [&](const char* s) { write_prefixed(src, slot_id, s); };
    if (!eth || !eth->present()) {
        write_line("E ERR eth unavail\r\n");
        return;
    }
    char buf[48];
    switch (sub) {
        case 'i':
            if (auto e = eth->reinit(); e != ESP_OK) {
                std::snprintf(buf, sizeof(buf), "E ERR %s\r\n", esp_err_to_name(e));
                write_line(buf);
            }
            return;
        case 'c': {
            const uint32_t ip = eth->ip4();
            uint8_t m[6];
            eth->mac(m);
            std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                          (unsigned)(ip & 0xFF), (unsigned)((ip >> 8) & 0xFF),
                          (unsigned)((ip >> 16) & 0xFF), (unsigned)((ip >> 24) & 0xFF),
                          m[0], m[1], m[2], m[3], m[4], m[5]);
            write_line(buf);
            return;
        }
        case 'd':
            std::snprintf(buf, sizeof(buf), "%02X\r\n", eth->bump_debug());
            write_line(buf);
            return;
        case 'n':
            write_line("E ERR ntp unsupported\r\n");
            return;
        default:
            return;
    }
}

// E-Befehl (Ethernet/Network) — culfw `E`-Familie + CULFW32-WiFi-Erweiterung
// `EW*`. a-culfw (CUNO) kennt Ei/Ec/Ed/En, siehe handle_eth(). CULFW32 ergaenzt:
//   EWs<ssid>     set SSID (Rest ist die SSID, bis CR; max 32 byte)
//   EWp<psk>      set PSK (max 63 byte; auch leer = open network)
//   EWm<n>        set Mode (0=off, 1=STA), n als ASCII-Dezimal
//   EWh<host>     set Hostname (mDNS-Name)
//   EW            query (multi-line: SSID, mode, host, state, ip, rssi; PSK zensiert ***)
//   EWS           scan: AP pro Zeile `EWS <bssid> <rssi> ch=<n> auth=<n> <ssid>`
//   EWx           wipe (NVS-Namespace + disconnect)
//   E<18 hex>     EM-Frame senden (wie `M`)
// Beim E ohne sub: einzeiliger WiFi-Status.
void handle_E(std::span<const char> args, Transport* src, uint8_t slot_id,
              WifiManager* wifi, EthManager* eth, RfManager* mgr,
              NvsEeprom* eeprom, TcpServer* tcp) {
    auto write_line = [&](const char* s) {
        write_prefixed(src, slot_id, s);
    };

    // a-culfw CUNO eth_func(): `Ei` / `Ec` / `Ed` / `En` — genau ein Buchstabe.
    // Muss VOR der Hex-Erkennung stehen: 'c' und 'd' sind Hex-Zeichen, und
    // `Ec` landete sonst als EM-Frame im LENERR. EM braucht 18 Hex-Zeichen,
    // die beiden Faelle sind ueber die Laenge eindeutig.
    if (args.size() == 1 &&
        (args[0] == 'i' || args[0] == 'c' || args[0] == 'd' || args[0] == 'n')) {
        handle_eth(args[0], src, slot_id, eth);
        return;
    }

    // `E<hex>` = em_send (CULFW32-Bestand; a-culfw hat em_send auf `M`, das
    // gibt es seit dem W5500 ebenfalls). Detect-Heuristik: args[0] ist Hex.
    auto is_hex = [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
    };
    if (!args.empty() && is_hex(args[0])) {
        em_send(args, src, slot_id, mgr, "E");
        return;
    }

    // Kein Hex-Args → WiFi-Subkommando-Pfad (Bestand).
    if (!wifi) {
        write_line("E ERR wifi unavail\r\n");
        return;
    }

    if (args.empty()) {
        // `E` ohne sub: kompakter Status (eine Zeile, analog a-culfw `Ec`).
        const auto& cfg = wifi->config();
        const char* state = "?";
        switch (wifi->state()) {
            case WifiManager::State::Off:          state = "off";          break;
            case WifiManager::State::Disconnected: state = "disconnected"; break;
            case WifiManager::State::Connecting:   state = "connecting";   break;
            case WifiManager::State::Connected:    state = "connected";    break;
        }
        char buf[160];
        uint32_t ip = wifi->ip4();
        std::snprintf(buf, sizeof(buf),
            "E %s mode=%u ssid=%s ip=%u.%u.%u.%u rssi=%d\r\n",
            state, cfg.mode,
            cfg.ssid.empty() ? "(unset)" : cfg.ssid.c_str(),
            (unsigned)(ip & 0xFF), (unsigned)((ip >> 8) & 0xFF),
            (unsigned)((ip >> 16) & 0xFF), (unsigned)((ip >> 24) & 0xFF),
            wifi->rssi());
        write_line(buf);
        return;
    }

    if (args[0] != 'W') {
        write_line("E ? (use Ec/Ei/Ed/EW*)\r\n");
        return;
    }

    auto rest = args.subspan(1);
    if (rest.empty()) {
        // `EW` query — multi-line.
        const auto& cfg = wifi->config();
        const char* state = "?";
        switch (wifi->state()) {
            case WifiManager::State::Off:          state = "off";          break;
            case WifiManager::State::Disconnected: state = "disconnected"; break;
            case WifiManager::State::Connecting:   state = "connecting";   break;
            case WifiManager::State::Connected:    state = "connected";    break;
        }
        char buf[256];
        std::snprintf(buf, sizeof(buf), "EW ssid=%s\r\n",
                      cfg.ssid.empty() ? "(unset)" : cfg.ssid.c_str());
        write_line(buf);
        std::snprintf(buf, sizeof(buf), "EW psk=%s\r\n",
                      cfg.psk.empty() ? "(unset)" : "***");
        write_line(buf);
        std::snprintf(buf, sizeof(buf), "EW mode=%u\r\n", cfg.mode);
        write_line(buf);
        std::snprintf(buf, sizeof(buf), "EW host=%s\r\n",
                      cfg.hostname.empty() ? wifi->default_hostname() : cfg.hostname.c_str());
        write_line(buf);
        uint32_t ip = wifi->ip4();
        std::snprintf(buf, sizeof(buf), "EW state=%s ip=%u.%u.%u.%u rssi=%d\r\n",
                      state,
                      (unsigned)(ip & 0xFF), (unsigned)((ip >> 8) & 0xFF),
                      (unsigned)((ip >> 16) & 0xFF), (unsigned)((ip >> 24) & 0xFF),
                      wifi->rssi());
        write_line(buf);
        // TCP-Port-Zeile: aktiver Port (vom TcpServer) bzw. EE-konfigurierter Port.
        uint16_t cur_port = tcp ? tcp->port() : 0;
        uint16_t ee_port  = ee::IP4_TCPLINK_PORT_DEFAULT;
        if (eeprom) {
            uint8_t lo = eeprom->read_byte(ee::IP4_TCPLINK_PORT);
            uint8_t hi = eeprom->read_byte(ee::IP4_TCPLINK_PORT + 1);
            uint16_t p = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
            if (p != 0 && p != 0xFFFF) ee_port = p;
        }
        std::snprintf(buf, sizeof(buf), "EW port=%u (ee=%u)\r\n", cur_port, ee_port);
        write_line(buf);
        // TCP-Allowlist-Zeile: aktive Allow-Subnet vom Server.
        if (tcp) {
            uint32_t aip = tcp->allow_ip();
            uint8_t  bts = tcp->allow_bits();
            if (bts == 0) {
                std::snprintf(buf, sizeof(buf), "EW allow=any\r\n");
            } else {
                std::snprintf(buf, sizeof(buf), "EW allow=%u.%u.%u.%u/%u\r\n",
                              (unsigned)(aip & 0xFF), (unsigned)((aip >> 8) & 0xFF),
                              (unsigned)((aip >> 16) & 0xFF), (unsigned)((aip >> 24) & 0xFF),
                              (unsigned)bts);
            }
            write_line(buf);
        }
        return;
    }

    char sub = rest[0];
    auto val = rest.subspan(1);

    // Untrusted Serial/TCP-Input: Laenge VOR der Allokation kappen. Mit
    // -fno-exceptions wuerde ein fehlgeschlagenes std::string-Allocate
    // std::terminate()->abort() ausloesen. Cap deckt das groesste legitime
    // Feld ab (PSK max 63, SSID 32, Hostname 63); laengere Eingaben sind
    // ohnehin nutzlos und werden still abgeschnitten (vgl. Doku oben).
    constexpr size_t kEwMaxField = 63;
    auto str_from = [&](std::span<const char> s) -> std::string {
        size_t n = s.size() < kEwMaxField ? s.size() : kEwMaxField;
        return std::string(s.data(), n);
    };

    if (sub == 's') { wifi->set_ssid    (str_from(val)); return; }
    if (sub == 'p') { wifi->set_psk     (str_from(val)); return; }
    if (sub == 'h') { wifi->set_hostname(str_from(val)); return; }
    if (sub == 'm') {
        // ASCII-decimal mode 0..3
        uint8_t mode = 0;
        for (char c : val) {
            if (c >= '0' && c <= '9') mode = mode * 10 + (c - '0');
            else break;
        }
        wifi->set_mode(mode);
        return;
    }
    if (sub == 'x') { wifi->wipe(); return; }
    if (sub == 'A') {
        // EWA <ip>/<bits>  → IP-Allowlist setzen (z.B. EWA 192.168.1.0/24).
        // EWA any | EWA 0  → Allowlist deaktivieren (jeder darf connecten).
        if (!eeprom) { write_line("EWA ERR ee unavail\r\n"); return; }
        // val ist nach dem 'A'; whitespace skip
        size_t i = 0;
        while (i < val.size() && (val[i] == ' ' || val[i] == '\t')) ++i;
        // Special: "any" oder "0" → Allowlist aus
        bool clear = false;
        if (i < val.size()) {
            if ((val[i] == 'a' || val[i] == 'A') && i + 2 < val.size()
                && val[i+1] == 'n' && val[i+2] == 'y') clear = true;
            else if (val[i] == '0' && (i + 1 >= val.size() || val[i+1] < '0' || val[i+1] > '9')) clear = true;
        } else clear = true;

        if (clear) {
            eeprom->write_byte(ee::TCP_ALLOW_IP,     0);
            eeprom->write_byte(ee::TCP_ALLOW_IP + 1, 0);
            eeprom->write_byte(ee::TCP_ALLOW_IP + 2, 0);
            eeprom->write_byte(ee::TCP_ALLOW_IP + 3, 0);
            eeprom->write_byte(ee::TCP_ALLOW_BITS,   0);
            if (tcp) tcp->set_allow(0, 0);
            write_line("EW allow=any\r\n");
            return;
        }

        // Parse dotted-quad / CIDR-bits
        uint8_t  octets[4] = {0,0,0,0};
        int      oi = 0;
        uint32_t cur = 0;
        bool     in_cidr = false;
        uint8_t  bits = 32;
        bool     have_bits = false;
        bool     in_num = false;
        for (; i < val.size(); ++i) {
            char c = val[i];
            if (c >= '0' && c <= '9') {
                cur = cur * 10 + (c - '0');
                in_num = true;
            } else if (c == '.' && !in_cidr) {
                if (oi >= 4 || cur > 255) { write_line("EWA ERR ip\r\n"); return; }
                octets[oi++] = static_cast<uint8_t>(cur);
                cur = 0; in_num = false;
            } else if (c == '/' && !in_cidr) {
                if (oi >= 4 || cur > 255) { write_line("EWA ERR ip\r\n"); return; }
                octets[oi++] = static_cast<uint8_t>(cur);
                cur = 0; in_cidr = true; in_num = false;
            } else if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
                break;
            } else {
                write_line("EWA ERR syntax\r\n"); return;
            }
        }
        if (in_cidr && in_num) { bits = static_cast<uint8_t>(cur); have_bits = true; }
        else if (!in_cidr && in_num) {
            if (oi >= 4 || cur > 255) { write_line("EWA ERR ip\r\n"); return; }
            octets[oi++] = static_cast<uint8_t>(cur);
        }
        if (oi != 4) { write_line("EWA ERR ip\r\n"); return; }
        if (have_bits && bits > 32) { write_line("EWA ERR cidr\r\n"); return; }
        if (!have_bits) bits = 32;     // single-host default

        eeprom->write_byte(ee::TCP_ALLOW_IP,     octets[0]);
        eeprom->write_byte(ee::TCP_ALLOW_IP + 1, octets[1]);
        eeprom->write_byte(ee::TCP_ALLOW_IP + 2, octets[2]);
        eeprom->write_byte(ee::TCP_ALLOW_IP + 3, octets[3]);
        eeprom->write_byte(ee::TCP_ALLOW_BITS,   bits);

        uint32_t ip_be = static_cast<uint32_t>(octets[0]) |
                         (static_cast<uint32_t>(octets[1]) << 8) |
                         (static_cast<uint32_t>(octets[2]) << 16) |
                         (static_cast<uint32_t>(octets[3]) << 24);
        if (tcp) tcp->set_allow(ip_be, bits);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "EW allow=%u.%u.%u.%u/%u\r\n",
                      octets[0], octets[1], octets[2], octets[3], bits);
        write_line(buf);
        return;
    }
    if (sub == 'P') {
        // EWP<dec>: TCP-Server-Port setzen (a-culfw EE_IP4_TCPLINK_PORT byte-genau).
        if (!eeprom) { write_line("EWP ERR ee unavail\r\n"); return; }
        uint32_t port = 0;
        for (char c : val) {
            if (c >= '0' && c <= '9') port = port * 10 + (c - '0');
            else if (c == ' ' || c == '\t') continue;
            else break;
        }
        if (port < 1 || port > 65535) { write_line("EWP ERR port range\r\n"); return; }
        // EE byte-genau a-culfw little-endian schreiben.
        eeprom->write_byte(ee::IP4_TCPLINK_PORT,     static_cast<uint8_t>(port & 0xFF));
        eeprom->write_byte(ee::IP4_TCPLINK_PORT + 1, static_cast<uint8_t>((port >> 8) & 0xFF));
        // TCP-Server live restarten — bestehende Client-Verbindungen bleiben offen,
        // nur der Listener bindet auf neuen Port.
        if (tcp) {
            if (auto e = tcp->restart(static_cast<uint16_t>(port)); e != ESP_OK) {
                char buf[64];
                std::snprintf(buf, sizeof(buf), "EWP ERR restart %s\r\n", esp_err_to_name(e));
                write_line(buf);
                return;
            }
        }
        char buf[48];
        std::snprintf(buf, sizeof(buf), "EW port=%u\r\n", static_cast<unsigned>(port));
        write_line(buf);
        return;
    }
    if (sub == 'S') {
        std::vector<WifiScanResult> aps;
        if (auto e = wifi->scan(aps); e != ESP_OK) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "EWS ERR %s\r\n", esp_err_to_name(e));
            write_line(buf);
            return;
        }
        for (auto& a : aps) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "EWS %02X:%02X:%02X:%02X:%02X:%02X %d ch=%u auth=%u %s\r\n",
                a.bssid[0], a.bssid[1], a.bssid[2], a.bssid[3], a.bssid[4], a.bssid[5],
                a.rssi, a.channel, a.auth_mode, a.ssid.c_str());
            write_line(buf);
        }
        char buf[32];
        std::snprintf(buf, sizeof(buf), "EWS done %u\r\n",
                      static_cast<unsigned>(aps.size()));
        write_line(buf);
        return;
    }

    write_line("EW ? (use EWs/EWp/EWm/EWh/EWP/EWA/EWS/EWx)\r\n");
}

// f-Befehl (rf_fastrf) — a-culfw fastrf.c:19 `fastrf_func()`.
//   fr            Init + Reception on (CC1101 -> 250 kbps GFSK config)
//   fs<rawbytes>  Send (rohe ASCII-Bytes nach 'fs', NICHT hex-encoded!)
//   f<other>      Reception off
// a-culfw silent on success (kein Antwort-Echo) — wir auch.
// ---------------------------------------------------------------------------
// D — Dudette v2, Funk-Bootloader fuer AVR-Ziele.
//   D    aus          Dr   Empfang ein        Dx   Empfang aus
//   Dl   Bestandsliste                        Dc   Liste leeren
//   Ds<addr>[,<board>]  Sitzung eroeffnen (blind, bis zu 6 s Anlaeufe)
//   Dw<addr>[,<board>]  scharfstellen: SELECT geht raus, sobald das HELLO
//             kommt; Dw zeigt den Stand, Dwx nimmt es zurueck
//   <board> ist die 16-bit-Board-ID aus dem HELLO (Dl: `b=`), vier Hexziffern.
//   Ist sie angegeben und meldet das Geraet etwas anderes, gibt es KEINE
//   Sitzung — Antwort `BOARD b=<ist> want=<soll>`.  Ebenso gibt es keine
//   Sitzung, wenn unter der Adresse mehrere Bausteine gehoert wurden
//   (`KOLLISION n=<k>`, Dl zeigt es als `KOLLISION=`); dann `Dc` und neu
//   lauschen.  Ohne Angabe wird nicht
//   geprueft; ein Intel-HEX traegt keine Kennung, die Erwartung muss also
//   von aussen kommen (Werkzeug, Manifest, Auswahl in der Oberflaeche).
//   De[<page>]  App-Bereich ab Seite loeschen (Vorgabe 0 = alles)
//   Dp/Dd/Dm  Seite fuellen und senden   Dv<from>,<len>  CRC   Dg  starten
//   Db[<addr>] Debug-Kanal an (869.5250), Db- aus   Dk<addr><cmd>[<arg>]
// ACHTUNG: jeder Unterbuchstabe darf genau EINMAL vorkommen.  Der erste
// Treffer gewinnt und kehrt zurueck; ein zweiter Handler mit demselben
// Buchstaben ist toter Code.  Genau das hat einmal 'Dd' (Fragment fuellen)
// verdeckt: das Flashen meldete Erfolg und schrieb 0xFF-Seiten.
// Spezifikation: notes/dudette_v2_protocol_2026-09-04.md
// ---------------------------------------------------------------------------
void handle_D(std::span<const char> args, Transport* src, uint8_t slot_id, RfManager* mgr) {
    auto get_dud = [&]() -> DudetteRF* {
        auto p = mgr->find_protocol(ProtocolId::Dudette);
        if (!p || p->id() != ProtocolId::Dudette) return nullptr;
        return static_cast<DudetteRF*>(p.get());
    };

    if (args.empty()) {
        if (get_dud()) mgr->deactivate(slot_id);
        return;
    }

    const char sub = args[0];

    if (sub == 'r') {
        if (auto e = mgr->activate(ProtocolId::Dudette, slot_id); e != ESP_OK) {
            char buf[48];
            std::snprintf(buf, sizeof(buf), "Dr ERR %s\r\n", esp_err_to_name(e));
            write_prefixed(src, slot_id, buf);
        }
        return;
    }

    if (sub == 'x') {
        if (get_dud()) mgr->deactivate(slot_id);
        return;
    }

    if (sub == 'c') {
        if (auto* d = get_dud()) d->clear_devices();
        return;
    }

    if (sub == 'k') {          // Dk<addr6><zeichen>[<arg2hex>]
        auto* d = get_dud();
        if (!d) { write_prefixed(src, slot_id, "D ERR\r\n"); return; }
        auto a = args.subspan(1);
        if (a.size() < 7) { write_prefixed(src, slot_id, "Dk ERR\r\n"); return; }
        uint32_t to = 0;
        for (size_t i = 0; i < 6; ++i) {
            int v = hexval(a[i]);
            if (v < 0) { write_prefixed(src, slot_id, "Dk ERR\r\n"); return; }
            to = (to << 4) | static_cast<uint32_t>(v);
        }
        uint8_t cmd = static_cast<uint8_t>(a[6]), arg = 0;
        for (size_t i = 7; i + 1 < a.size() + 1 && i < a.size(); ++i) {
            int v = hexval(a[i]);
            if (v >= 0) arg = static_cast<uint8_t>((arg << 4) | v);
        }
        auto e = d->debug_cmd(to, cmd, arg);
        char buf[72];
        std::snprintf(buf, sizeof(buf), "Dk %06lX %c %02X %s\r\n",
                      static_cast<unsigned long>(to), cmd, arg,
                      e == ESP_OK ? "OK" : esp_err_to_name(e));
        write_prefixed(src, slot_id, buf);
        return;
    }

    if (sub == 'b') {                       // Db | Db<addr> | Db-
        auto* d = get_dud();
        if (!d) { write_prefixed(src, slot_id, "D ERR\r\n"); return; }
        if (!d->active()) mgr->activate(ProtocolId::Dudette, slot_id);
        char buf[80];
        auto arg = args.subspan(1);       // `rest` gibt es hier noch nicht
        if (!arg.empty() && arg[0] == '-') {
            d->debug_channel(false);
            write_prefixed(src, slot_id, "Db aus, zurueck auf 868.4998\r\n");
            return;
        }
        uint32_t filt = 0;
        for (char c : arg) {
            int v = hexval(c);
            if (v < 0) { write_prefixed(src, slot_id, "Db ERR\r\n"); return; }
            filt = (filt << 4) | static_cast<uint32_t>(v);
        }
        d->debug_channel(true, filt);
        if (filt) std::snprintf(buf, sizeof(buf),
                    "Db an, 869.5250 MHz, nur %06lX\r\n",
                    static_cast<unsigned long>(filt));
        else      std::snprintf(buf, sizeof(buf),
                    "Db an, 869.5250 MHz, alle Geraete\r\n");
        write_prefixed(src, slot_id, buf);
        // Deutlich sagen, was das kostet — sonst sucht jemand spaeter, warum
        // kein Transfer mehr geht.
        write_prefixed(src, slot_id,
            "Db HINWEIS: solange an, keine Dudette-Sitzung moeglich\r\n");
        return;
    }

    if (sub == 'l') {
        auto* d = get_dud();
        if (!d) { write_prefixed(src, slot_id, "D ERR\r\n"); return; }

        const uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        char buf[160];
        std::snprintf(buf, sizeof(buf), "D %u\r\n",
                      static_cast<unsigned>(d->device_count()));
        write_prefixed(src, slot_id, buf);

        for (size_t i = 0; i < d->device_count(); ++i) {
            const DudetteDevice& e = d->devices()[i];
            // Restzeit des Taster-Fensters steht im oberen Nibble der Flags.
            // Als eigener Anhang HINTEN, damit Parser, die von vorne zaehlen,
            // unveraendert weiterlaufen.
            char rest[40] = "";
            size_t rn = 0;
            if (e.flags & 0x08)
                rn += std::snprintf(rest + rn, sizeof(rest) - rn, " btn=%umin",
                                    static_cast<unsigned>(e.flags >> 4));
            // Zwei Bausteine, ein Name.  Die Adresse ist ein auf 24 bit
            // gekuerzter Hash der Signaturzeile — eindeutig ist sie nur
            // wahrscheinlich.  Ohne diese Marke verschmelzen zwei Geraete
            // still zu einem Eintrag, und ein Transfer trifft das falsche.
            if (e.collisions)
                rn += std::snprintf(rest + rn, sizeof(rest) - rn,
                                    " KOLLISION=%u", e.collisions);
            (void)rn;
            // Alter in Zehntelsekunden — ein Geraet, dessen Fenster zu ist,
            // faellt so sofort auf, ohne dass die Liste selbst altert.
            const uint32_t age_ds = (now - e.last_seen_ms) / 100;
            std::snprintf(buf, sizeof(buf),
                "D %06lX b=%04X s=%02X%02X%02X v=%02X c=%02X id=%02X lot=%s die=%02X%02X%02X%02X "
                "pg=%u app=%lu %c rssi=%d lqi=%u age=%lu.%lus n=%lu%s\r\n",
                static_cast<unsigned long>(e.addr), e.board_id,
                e.sig[0], e.sig[1], e.sig[2],
                e.bl_version, e.caps, static_cast<uint8_t>(e.ident),
                (e.ident == dudette::IdentSource::SignatureRow) ? e.lot : "-",
                e.die[0], e.die[1], e.die[2], e.die[3],
                e.page_size, static_cast<unsigned long>(e.app_size),
                // Dudette v2.2: Bit0 App vorhanden, Bit1 Absturzverdacht,
                // Bit2 Anwendung hatte sich bestaetigt, Bit3 Einstieg ueber den
                // beim Einschalten gehaltenen Taster.  Ein Zeichen bleibt es,
                // damit vorhandene Parser auf ' - ' weiter funktionieren:
                //   '-' keine App · 'B' Taster-Einstieg · 'W' Absturzverdacht
                //   · 'C' bestaetigt · 'A' sonst
                !(e.flags & 0x01) ? '-'
                    : (e.flags & 0x08) ? 'B'
                    : (e.flags & 0x02) ? 'W'
                    : (e.flags & 0x04) ? 'C' : 'A',
                e.rssi_dbm, e.lqi,
                static_cast<unsigned long>(age_ds / 10),
                static_cast<unsigned long>(age_ds % 10),
                static_cast<unsigned long>(e.hello_count), rest);
            write_prefixed(src, slot_id, buf);
            // Diagnoseanhang des Bootloaders — die laufende Sicht darauf, was
            // beim Geraet ankommt, ohne ISP-Clip.  sy=Sync-Flanken,
            // ok=Rahmen mit gueltiger CRC, da=davon DATA, ln=letzte Laenge,
            // bl=Laengenfehler, sp/cs=Timeouts, mc=MCUSR des letzten Starts.
            if (e.has_dbg) {
                std::snprintf(buf, sizeof(buf),
                    "D %06lX rx sy=%u ok=%u da=%u ln=%u ty=%02X bl=%u sp=%u cs=%u mc=%02X\r\n",
                    static_cast<unsigned long>(e.addr),
                    e.dbg[0], e.dbg[1], e.dbg[2], e.dbg[3], e.dbg[13],
                    e.dbg[4], e.dbg[5], e.dbg[6], e.dbg[7]);
                write_prefixed(src, slot_id, buf);
                // Was nach dem Empfang mit dem Rahmen geschah.
                std::snprintf(buf, sizeof(buf),
                    // `cp=` statt `pg=`: die Bestandszeile trennt mit `pg=` die
                    // SEITENGROESSE, hier stand dieselbe Marke fuer die zuletzt
                    // bearbeitete Seite.  Ein Leser, der `pg=` aus der Dl-Ausgabe
                    // zieht, bekam damit 0 statt 128 — und flashte mit falscher
                    // Seitengroesse.
                    "D %06lX ses fr=%u bad=%u cm=%u tx=%u map=%02X rst=%u cp=%u\r\n",
                    static_cast<unsigned long>(e.addr),
                    e.dbg[8], e.dbg[9], e.dbg[10], e.dbg[11], e.dbg[12],
                    e.dbg[14], e.dbg[15]);
                write_prefixed(src, slot_id, buf);
            }
        }
        return;
    }

    // --- Transfer -------------------------------------------------------
    auto hex2bin = [](std::span<const char> in, uint8_t* out, uint8_t max) -> int {
        if (in.size() % 2) return -1;
        size_t n = in.size() / 2;
        if (n > max) return -1;
        for (size_t i = 0; i < n; ++i) {
            int hi = hexval(in[2 * i]), lo = hexval(in[2 * i + 1]);
            if (hi < 0 || lo < 0) return -1;
            out[i] = static_cast<uint8_t>((hi << 4) | lo);
        }
        return static_cast<int>(n);
    };

    auto* d = get_dud();
    if (!d) { write_prefixed(src, slot_id, "D ERR\r\n"); return; }
    auto rest = args.subspan(1);
    char buf[96];

    // `<addr6>[,<board4>]` — die Board-Erwartung ist optional und wird an den
    // Stick durchgereicht.  Sie steht hinter der Adresse statt in einem
    // eigenen Befehl, damit Erwartung und Ziel nicht auseinanderlaufen
    // koennen: ein separat gesetzter Zustand haelt sich sonst ueber die
    // naechste Sitzung hinweg und prueft dann gegen das falsche Abbild.
    auto parse_target = [](std::span<const char> in, uint32_t& addr,
                           uint16_t& board, bool& has_board) -> bool {
        if (in.size() != 6 && in.size() != 11) return false;
        addr = 0;
        for (size_t i = 0; i < 6; ++i) {
            int v = hexval(in[i]);
            if (v < 0) return false;
            addr = (addr << 4) | static_cast<uint32_t>(v);
        }
        board = 0; has_board = false;
        if (in.size() == 6) return true;
        if (in[6] != ',') return false;
        for (size_t i = 7; i < 11; ++i) {
            int v = hexval(in[i]);
            if (v < 0) return false;
            board = static_cast<uint16_t>((board << 4) | v);
        }
        // 0000 ist KEINE Board-ID: expect_board(0) heisst "keine Erwartung".
        // Wer `,0000` anhaengt, haette das Tor also mit dessen eigener Syntax
        // aufgesperrt und dabei ausgesehen, als pruefe er.  Wer nicht pruefen
        // will, laesst das Suffix weg — das ist der dokumentierte Weg.
        if (board == 0) return false;
        has_board = true;
        return true;
    };

    if (sub == 's') {                       // Ds<addr>[,<board>]  Sitzung eroeffnen
        uint32_t addr = 0; uint16_t board = 0; bool has_board = false;
        if (!parse_target(rest, addr, board, has_board)) {
            write_prefixed(src, slot_id, "Ds ERR\r\n"); return;
        }
        // Ohne Angabe wird die Erwartung geloescht, nicht beibehalten — sonst
        // prueft der naechste Transfer still gegen den Wert des vorigen.
        d->expect_board(has_board ? board : 0);
        uint8_t st = 0xFF;
        auto e = d->select(addr, st);
        if (e == ESP_ERR_INVALID_STATE) {
            std::snprintf(buf, sizeof(buf),
                          "Ds %06lX KOLLISION n=%u st=%u — Dc, dann neu lauschen\r\n",
                          static_cast<unsigned long>(addr),
                          d->last_collisions(), st);
            write_prefixed(src, slot_id, buf);
            return;
        }
        if (e == ESP_ERR_INVALID_VERSION) {
            std::snprintf(buf, sizeof(buf),
                          "Ds %06lX BOARD b=%04X want=%04X st=%u\r\n",
                          static_cast<unsigned long>(addr),
                          d->last_board(), board, st);
            write_prefixed(src, slot_id, buf);
            return;
        }
        std::snprintf(buf, sizeof(buf), "Ds %06lX %s st=%u pg=%u\r\n",
                      static_cast<unsigned long>(addr),
                      (e == ESP_OK) ? "OK" : esp_err_to_name(e), st,
                      d->page_size());
        write_prefixed(src, slot_id, buf);
        return;
    }

    if (sub == 'w') {                       // Dw<addr> / Dw / Dwx
        if (rest.empty()) {
            const char* st = "idle";
            switch (d->arm_state()) {
                case DudetteRF::ArmState::Armed: st = "armed"; break;
                case DudetteRF::ArmState::Sent:  st = "sent";  break;
                case DudetteRF::ArmState::Done:  st = (d->arm_status() == 0) ? "OK" : "FAIL"; break;
                default: break;
            }
            // Der Grund gehoert an die Statuszeile: "FAIL st=176" allein
            // schickt jeden auf die Suche nach einem Funkproblem, das es
            // nicht gibt.  "FAIL" bleibt stehen, damit Werkzeuge, die darauf
            // abbrechen, unveraendert weiterlaufen.
            char why[40] = "";
            if (d->arm_state() == DudetteRF::ArmState::Done
                && d->arm_status() == dudette::kStBoardMismatch)
                std::snprintf(why, sizeof(why), " BOARD b=%04X want=%04X",
                              d->last_board(), d->expect_board());
            else if (d->arm_state() == DudetteRF::ArmState::Done
                     && d->arm_status() == dudette::kStAmbiguous)
                std::snprintf(why, sizeof(why), " KOLLISION n=%u",
                              d->last_collisions());
            std::snprintf(buf, sizeof(buf), "Dw %06lX %s st=%u pg=%u%s\r\n",
                          static_cast<unsigned long>(d->arm_addr()), st,
                          d->arm_status(), d->page_size(), why);
            write_prefixed(src, slot_id, buf);
            return;
        }
        if (rest[0] == 'x') { d->disarm(); d->expect_board(0); return; }
        uint32_t addr = 0; uint16_t board = 0; bool has_board = false;
        if (!parse_target(rest, addr, board, has_board)) {
            write_prefixed(src, slot_id, "Dw ERR\r\n"); return;
        }
        if (!d->active()) mgr->activate(ProtocolId::Dudette, slot_id);
        d->expect_board(has_board ? board : 0);
        d->arm(addr);
        if (has_board)
            std::snprintf(buf, sizeof(buf), "Dw %06lX armed want=%04X\r\n",
                          static_cast<unsigned long>(addr), board);
        else
            std::snprintf(buf, sizeof(buf), "Dw %06lX armed\r\n",
                          static_cast<unsigned long>(addr));
        write_prefixed(src, slot_id, buf);
        return;
    }

    if (sub == 'e') {                       // De[<page>]  App-Bereich loeschen
        uint32_t pg = 0;
        for (char c : rest) {
            int v = hexval(c);
            if (v < 0) { write_prefixed(src, slot_id, "De ERR\r\n"); return; }
            pg = (pg << 4) | static_cast<uint32_t>(v);
        }
        uint8_t st = 0xFF;
        auto e = d->erase(static_cast<uint16_t>(pg), st);
        std::snprintf(buf, sizeof(buf), "De %04lX %s st=%u\r\n",
                      static_cast<unsigned long>(pg),
                      (e == ESP_OK) ? "OK" : esp_err_to_name(e), st);
        write_prefixed(src, slot_id, buf);
        return;
    }

    if (sub == 'p') {                       // Dp<page>  Seitenpuffer oeffnen
        uint32_t pg = 0;
        for (char c : rest) {
            int v = hexval(c);
            if (v < 0) { write_prefixed(src, slot_id, "Dp ERR\r\n"); return; }
            pg = (pg << 4) | static_cast<uint32_t>(v);
        }
        d->page_begin(static_cast<uint16_t>(pg));
        return;                             // still, wie a-culfw bei Erfolg
    }

    if (sub == 'd') {                       // Dd<frag>,<hex>  Fragment fuellen
        if (rest.size() < 3 || rest[1] != ',') {
            write_prefixed(src, slot_id, "Dd ERR\r\n"); return;
        }
        int frag = hexval(rest[0]);
        uint8_t tmp[49];
        int n = hex2bin(rest.subspan(2), tmp, sizeof(tmp));
        if (frag < 0 || n <= 0) { write_prefixed(src, slot_id, "Dd ERR\r\n"); return; }
        if (d->page_fill(static_cast<uint8_t>(frag), tmp,
                         static_cast<uint8_t>(n)) != ESP_OK)
            write_prefixed(src, slot_id, "Dd ERR\r\n");
        return;
    }

    if (sub == 'm') {                       // Dm  Seite senden und quittieren
        uint8_t st = 0xFF, bm = 0, rounds = 0;
        // Vier Diagnosebytes des Bootloaders: Sync-Flanken, Rahmen mit
        // gueltiger CRC, davon DATA, zuletzt gesehene Laenge.  0xFF heisst
        // "nicht gemeldet" (aeltere Bootloader).
        uint8_t dbg[24];
        std::memset(dbg, 0xFF, sizeof(dbg));
        uint16_t want = 0;
        // Die Seitennummer VOR dem Senden merken: send_page() gibt den Puffer
        // danach frei (page_no_ = 0xFFFF), und genau diese Nummer ist das
        // Echo, an dem das Werkzeug ein verlorenes `Dp` erkennt.  Ein `Dm`
        // ohne offenen Puffer meldet dementsprechend FFFF.
        const uint16_t pg_sent = d->page_number();
        auto e = d->send_page(st, bm, rounds, dbg, &want);
        std::snprintf(buf, sizeof(buf),
                      "Dm %04X %s st=%u map=%02X r=%u sy=%u ok=%u da=%u ln=%u\r\n",
                      pg_sent,
                      (e == ESP_OK) ? "OK" : esp_err_to_name(e), st, bm, rounds,
                      dbg[0], dbg[1], dbg[2], dbg[3]);
        write_prefixed(src, slot_id, buf);
        // Soll-CRC des Sticks gegen die des Geraets, dazu dessen Pufferraender.
        std::snprintf(buf, sizeof(buf),
                      "Dm crc want=%04X have=%02X%02X buf=%02X%02X..%02X%02X\r\n",
                      want, dbg[5], dbg[4], dbg[6], dbg[7], dbg[8], dbg[9]);
        write_prefixed(src, slot_id, buf);
        // Erstes Byte jedes Fragments im Geraetepuffer.
        std::snprintf(buf, sizeof(buf),
                      "Dm frag %02X %02X %02X %02X %02X %02X\r\n",
                      dbg[10], dbg[11], dbg[12], dbg[13], dbg[14], dbg[15]);
        write_prefixed(src, slot_id, buf);
        // Uebernahmen je Fragment, Puffer-Resets, COMMITs gesamt (ab
        // Bootloader 2.1; 2.0 meldete hier die letzte Seitennummer).
        std::snprintf(buf, sizeof(buf),
                      "Dm n= %u %u %u %u %u %u  rst=%u cm=%u\r\n",
                      dbg[16], dbg[17], dbg[18], dbg[19], dbg[20], dbg[21],
                      dbg[22], dbg[23]);
        write_prefixed(src, slot_id, buf);
        return;
    }

    if (sub == 'v' || sub == 'V') {         // Dv<from>,<len>  CRC ueber Flash,
                                            // DV<from>,<len>  CRC ueber EEPROM
        uint32_t from = 0, n = 0; bool second = false; bool ok = !rest.empty();
        for (char c : rest) {
            if (c == ',') { second = true; continue; }
            int v = hexval(c);
            if (v < 0) { ok = false; break; }
            if (second) n = (n << 4) | static_cast<uint32_t>(v);
            else        from = (from << 4) | static_cast<uint32_t>(v);
        }
        if (!ok || !second) {
            write_prefixed(src, slot_id, sub == 'V' ? "DV ERR\r\n" : "Dv ERR\r\n"); return;
        }
        uint16_t crc = 0; uint8_t st = 0xFF;
        auto e = d->verify(from, n, crc, st, sub == 'V');
        std::snprintf(buf, sizeof(buf), "D%c %04X %s st=%u\r\n", sub, crc,
                      (e == ESP_OK) ? "OK" : esp_err_to_name(e), st);
        write_prefixed(src, slot_id, buf);
        return;
    }

    if (sub == 'g') {                       // Dg  Anwendung starten
        uint8_t st = 0xFF;
        auto e = d->go(st);
        std::snprintf(buf, sizeof(buf), "Dg %s st=%u\r\n",
                      (e == ESP_OK) ? "OK" : esp_err_to_name(e), st);
        write_prefixed(src, slot_id, buf);
        return;
    }

    write_prefixed(src, slot_id, "D ERR\r\n");
}

void handle_f(std::span<const char> args, Transport* src, uint8_t slot_id, RfManager* mgr) {
    auto get_fastrf = [&]() -> FastRfRF* {
        auto p = mgr->find_protocol(ProtocolId::FastRf);
        if (!p || p->id() != ProtocolId::FastRf) return nullptr;
        return static_cast<FastRfRF*>(p.get());
    };

    if (args.empty()) {
        if (get_fastrf()) mgr->deactivate(slot_id);
        return;
    }

    char sub = args[0];
    auto rest = args.subspan(1);

    if (sub == 'r') {
        if (auto e = mgr->activate(ProtocolId::FastRf, slot_id); e != ESP_OK) {
            char buf[48];
            std::snprintf(buf, sizeof(buf), "fr ERR %s\r\n", esp_err_to_name(e));
            write_prefixed(src, slot_id, buf);
            return;
        }
        // a-culfw silent on success.
        return;
    }

    if (sub == 's') {
        auto* f = get_fastrf();
        if (!f) {
            write_prefixed(src, slot_id, "f ERR\r\n");
            return;
        }
        // a-culfw fastrf_func.c:32-39: rohe Bytes ab in[2] direkt ans CC1101
        // schreiben. KEIN Hex-Decode. Wir nehmen die ASCII-Chars aus `rest`
        // und packen sie in den TX-Frame.
        FastRfTxFrame frame{};
        size_t n = rest.size();
        if (n == 0 || n > FastRfFrame::kMaxPayload) return;
        frame.len = static_cast<uint8_t>(n);
        for (size_t i = 0; i < n; ++i) frame.data[i] = static_cast<uint8_t>(rest[i]);
        if (auto e = mgr->tx(ProtocolId::FastRf, &frame, sizeof(frame)); e != ESP_OK) {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "f ERR %s\r\n", esp_err_to_name(e));
            write_prefixed(src, slot_id, buf);
            return;
        }
        return;
    }

    // f<other> -> off
    if (get_fastrf()) mgr->deactivate(slot_id);
}

// N-Befehl (rf_native) — a-culfw rf_native.c:242 `native_func()`.
//   Nr<dec>   Reception on, mode <dec> (1=IT+ 17.241k, 2=IT+ 9.579k, 3=PCA301)
//   Nx        Reception off
//   N<other>  no-op (a-culfw fall-through)
// Antwort: byte-genau a-culfw — nach jedem Nr/Nx wird der aktuelle Mode-Index
// als 2 Hex-Chars + CRLF ausgegeben (`<mode>\r\n`, `00` = aus). Bei ungueltigem
// Mode (`Nr0` oder `Nr5`) gibt a-culfw "specify valid mode number\r\n" + Mode 0.
void handle_N(std::span<const char> args, Transport* src, uint8_t slot_id, RfManager* mgr) {
    auto get_native = [&]() -> NativeRF* {
        auto p = mgr->find_protocol(ProtocolId::Native);
        if (!p || p->id() != ProtocolId::Native) return nullptr;
        return static_cast<NativeRF*>(p.get());
    };

    auto* n = get_native();
    auto echo_mode = [&]() {
        char buf[8];
        std::snprintf(buf, sizeof(buf), "%02X\r\n", n ? n->native_on() : 0);
        write_prefixed(src, slot_id, buf);
    };

    if (args.empty()) {
        echo_mode();
        return;
    }

    char sub = args[0];
    auto rest = args.subspan(1);

    if (sub == 'r') {
        // Mode parsen — a-culfw nutzt `fromdec` (decimal). 1..3 gueltig.
        uint16_t mode = 0;
        for (char c : rest) {
            if (c >= '0' && c <= '9') mode = mode * 10 + (c - '0');
            else break;
        }
        if (!n) {
            write_prefixed(src, slot_id, "N ERR\r\n");
            return;
        }
        if (mode < 1 || mode > 3) {
            // a-culfw rf_native.c:258 returned silent nach dem Error-Text —
            // KEIN Mode-Echo dahinter. Byte-genau reproduzieren.
            write_prefixed(src, slot_id, "specify valid mode number\r\n");
            return;
        }
        n->set_mode(static_cast<uint8_t>(mode));
        if (auto e = mgr->activate(ProtocolId::Native, slot_id); e != ESP_OK) {
            char buf[48];
            std::snprintf(buf, sizeof(buf), "Nr ERR %s\r\n", esp_err_to_name(e));
            write_prefixed(src, slot_id, buf);
            return;
        }
        echo_mode();
        return;
    }

    if (sub == 'x') {
        if (n) mgr->deactivate(slot_id);
        echo_mode();
        return;
    }

    // Unknown sub — a-culfw faellt durch zu MULTICC_PREFIX + DH2(native_on) + DNL.
    echo_mode();
}



#ifdef CONFIG_CULFW32_BUILD_RF_HMIP
// `P`-Befehl — HmIP-Sniffer. Reiner Empfang; a-culfw kennt das nicht.
//
//   P    -> aus
//   Pr   -> Empfang ein (HMIP_CFG, GDO2-IRQ auf CRC-OK)
//   Px   -> Empfang aus
//
// Ausgabe siehe render_hmip: P<len><payload><rssi><lqi> t=<ms> n=<seq>
//
// Bewusst NICHT vorhanden: `Ps` (senden), `Pm` (MAC-Layer mit Quittung) und
// `Pp` (Anlernen). Wer HmIP sprechen statt mithoeren will, nimmt q-culfw —
// die kurze Quittungsfrist ist ueber eine serielle Schnittstelle ohnehin
// nicht zu halten.
//
// Tag 'P' (mnemonic hmIP) statt 'H': 'H' ist in a-culfw fuer HMS-Wireless-
// Sensoren belegt, FHEM 12_HMS.pm wuerde daraus Phantom-Devices anlegen.
// Vereinbarung Dirk 2026-05-03.
void handle_P(std::span<const char> args, Transport* src, uint8_t slot_id,
              RfManager* mgr) {
    auto get_hmip = [&]() -> HmipRF* {
        auto p = mgr->find_protocol(ProtocolId::Hmip);
        if (!p || p->id() != ProtocolId::Hmip) return nullptr;
        return static_cast<HmipRF*>(p.get());
    };

    if (args.empty()) {
        if (get_hmip()) mgr->deactivate(slot_id);
        return;
    }

    const char sub = args[0];

    if (sub == 'r') {
        if (!get_hmip()) {
            write_prefixed(src, slot_id, "P ERR\r\n");
            return;
        }
        // KEIN Kurzschluss auf "ist schon aktiv" — er hat den Empfaenger
        // still getoetet.  `Px` ruft HmipRF::stop() DIREKT: der Chip geht
        // auf SIDLE und `drv_` wird genullt (beide Klassen tun das).  Der Manager erfaehrt davon
        // nichts und haelt das Protokoll weiter fuer aktiv, also nahm ein
        // spaeteres `Pr` diese Abkuerzung, meldete `ok` und rief start()
        // NIE.  Ergebnis: MARCSTATE bleibt 0x01 (IDLE), es kommt nichts mehr an,
        // und `Vr` zeigt trotzdem eine gueltige Konfiguration — SIDLE fasst die
        // Register nicht an.  Ein Empfaenger, der `ok` sagt und nichts hoert,
        // ist schlimmer als einer, der das Radio einmal zu oft neu anwirft.
        // `Dr` und `Zr` aktivieren seit jeher bedingungslos und sind deshalb
        // nicht betroffen — sie waren die Gegenprobe.
        // Gemessen 07.09.2026 am CUL32 #2 (C35 = MARCSTATE):
        //   AskSin  frisch 0D · nach Ax 01 · nach Ax+Ar 01   <- taub
        //   HmIP    frisch 0D · nach Px 01 · nach Px+Pr 01   <- taub
        //   Dudette frisch 0D · nach Dx 01 · nach Dx+Dr 0D   <- erholt sich
        // Preis: ein wiederholtes `Pr` initialisiert das Radio neu und
        // verwirft einen gerade laufenden Empfang.  Das ist die richtige
        // Richtung: `Pr` heisst "empfange", nicht "melde, dass du es tust".
        if (auto e = mgr->activate(ProtocolId::Hmip, slot_id); e != ESP_OK) {
            char buf[48];
            std::snprintf(buf, sizeof(buf), "Pr ERR %s\r\n", esp_err_to_name(e));
            write_prefixed(src, slot_id, buf);
            return;
        }
        write_prefixed(src, slot_id, "Pr ok\r\n");
        return;
    }

    if (sub == 'x') {
        if (get_hmip()) mgr->deactivate(slot_id);
        return;
    }

    // Alles andere (insbesondere das frueher vorhandene `Ps`) gibt es nicht mehr.
    write_prefixed(src, slot_id, "P ERR\r\n");
}
#endif // CONFIG_CULFW32_BUILD_RF_HMIP

#ifdef CONFIG_CULFW32_BUILD_RF_WMBUS
// `b`-Befehl byte-genau a-culfw rf_mbus.c:530 `rf_mbus_func`.
//   b              -> Status (SMODE/TMODE/OFF + \r\n)
//   br             -> Status (in[2]==0, faellt durch zu mbus_status())
//   brs            -> Reception on, S-Mode
//   brt            -> Reception on, T-Mode
//   br<other>      -> Reception off
//   bs<hex>        -> falls in[2]=='s' -> S-Mode-Send;
//                     falls in[2]=='t' -> T-Mode-Send.
//                     Hex (a-culfw fromhex) ergibt MBpacket; packetSize via
//                     L-Feld in pkt[0]. Andere Subletter -> kein Send,
//                     fall-through zu Status.
void handle_b(std::span<const char> args, Transport* src, uint8_t slot_id, RfManager* mgr) {
    auto get_w = [&]() -> WmbusRF* {
        auto p = mgr->find_protocol(ProtocolId::Mbus);
        if (!p || p->id() != ProtocolId::Mbus) return nullptr;
        return static_cast<WmbusRF*>(p.get());
    };
    auto status_line = [&](WmbusRF* w) {
        const char* s = "OFF\r\n";
        if (w && w->rx_active()) {
            switch (w->current_mode()) {
                case wmbus::MODE_S: s = "SMODE\r\n"; break;
                case wmbus::MODE_T: s = "TMODE\r\n"; break;
                default: break;
            }
        }
        write_prefixed(src, slot_id, s);
    };

    auto* w = get_w();
    if (!w) {
        write_prefixed(src, slot_id, "b ERR\r\n");
        return;
    }

    if (args.empty()) { status_line(w); return; }

    // br[s|t|<other>]
    if (args[0] == 'r') {
        if (args.size() < 2) { status_line(w); return; }
        if (auto e = mgr->activate(ProtocolId::Mbus, slot_id); e != ESP_OK) {
            char buf[48];
            std::snprintf(buf, sizeof(buf), "br ERR %s\r\n", esp_err_to_name(e));
            write_prefixed(src, slot_id, buf);
            return;
        }
        switch (args[1]) {
            case 's': w->set_mode_smode(); break;
            case 't': w->set_mode_tmode(); break;
            default:  w->set_off();        break;
        }
        status_line(w);
        return;
    }

    // bs[s|t]<hex>
    if (args[0] == 's') {
        if (args.size() < 3) { status_line(w); return; }
        char mode_c = args[1];
        if (mode_c != 's' && mode_c != 't') { status_line(w); return; }

        WmbusTxFrame tx{};
        tx.mode = (mode_c == 's') ? wmbus::MODE_S : wmbus::MODE_T;

        // a-culfw fromhex: ueberspringt Spaces/Doppelpunkte, bricht bei \r/\n
        // ab. parse_hex_run macht das byte-genau.
        auto hex_args = args.subspan(2);
        size_t got = parse_hex_run(hex_args, {tx.packet, sizeof(tx.packet)});
        if (got == 0 || got > 290) { status_line(w); return; }

        // packet_size aus L-Feld validieren (a-culfw txSendPacket macht das auch:
        // packetSize(MBpacket[0]); wir spiegeln die Plausibilisierung).
        uint16_t computed = wmbus::packet_size(tx.packet[0]);
        if (computed == 0 || computed > sizeof(tx.packet) || computed > got) {
            // Wenn Frontend zu wenige Bytes geschickt hat: ablehnen.
            status_line(w);
            return;
        }
        tx.len = computed;

        // Nur aktivieren, wenn wMBus nicht schon laeuft: der Slot startet die
        // Klasse dabei neu, und tx_send_packet wuesste dann nicht mehr, dass
        // vorher empfangen wurde -- a-culfw txSendPacket kehrt nach dem Senden
        // in den Empfang zurueck (lastMode), CULFW32 blieb bis 0.1.869 auf OFF.
        auto active = mgr->active_protocol_on(slot_id);
        if (!active || active->id() != ProtocolId::Mbus) {
            if (auto e = mgr->activate(ProtocolId::Mbus, slot_id); e != ESP_OK) {
                char buf[48];
                std::snprintf(buf, sizeof(buf), "bs ERR %s\r\n", esp_err_to_name(e));
                write_prefixed(src, slot_id, buf);
                return;
            }
        }
        if (mgr->tx(ProtocolId::Mbus, &tx, sizeof(tx)) != ESP_OK) {
            // a-culfw schreibt im Fehler-Pfad nichts; nur Status. Wir spiegeln.
        }
        status_line(w);
        return;
    }

    status_line(w);
}
#endif

// `q`: tcplink_close — a-culfw clib/tcplink.c:60. Server-seitiger Disconnect
// auf Anforderung des Clients. a-culfw setzt nur `state=CLOSE`; die uIP-Loop
// finalisiert die Verbindung silent. Wir replizieren: src->stop() schliesst
// Socket → rx-Task lauft auf next recv in error → on_closed → detach.
// Wirkt nur auf TCP-Transport (Name "tcp"); auf USB-CDC/UART silent ignoriert
// (sonst wuerde man sich aus dem einzigen verbleibenden Transport aussperren).
void handle_q(std::span<const char> /*args*/, Transport* src, uint8_t /*slot_id*/) {
    if (!src) return;
    const char* n = src->name();
    if (!n || std::strcmp(n, "tcp") != 0) return;
    src->stop();
}

// `e[x]`: factory-reset (a-culfw fncollection.c:268 `eeprom_factory_reset()`).
//   e\n   -> EE auf a-culfw-Defaults setzen + reset (esp_restart)
//   ex\n  -> EE auf a-culfw-Defaults setzen, KEIN reset (a-culfw `in[1]=='x'`)
// Silent on success — a-culfw gibt vor dem reset nichts mehr aus, danach kommt
// der Boot-Banner. Wir reproduzieren das: kein Output ausser dem Boot-V von
// `idf.py monitor` nach dem Restart. Bei `ex` bleibt's still.
void handle_e(std::span<const char> args, Transport* src, uint8_t slot_id,
              NvsEeprom* eeprom) {
    if (!eeprom) return;
    factory_reset(*eeprom);
    // a-culfw: `if(in[1] != 'x') prepare_boot(0);`
    bool no_reset = !args.empty() && args[0] == 'x';
    if (no_reset) {
        // silent
        (void)src; (void)slot_id;
        return;
    }
    // Letztes ACK noch rausschreiben analog `B reset`, damit der Client weiss
    // dass das Kommando akzeptiert wurde — a-culfw selbst ist hier silent, aber
    // bei USB-CDC-/UART-Reconnect verliert man sonst den Boot-Banner. Wir geben
    // das in Klammern aus, damit es nicht als culfw-Antwort missgedeutet wird.
    write_prefixed(src, slot_id, "e reset\r\n");
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
}

// `x[<idx>]`: PA-Tabelle setzen, byte-genau a-culfw cc1100.c:347 `ccsetpa()`.
//   Index 0..4: -10/-5/0/5/10 dBm mit PA-Ramping  (FREND0 = 0x17)
//   Index 5..9: -10/-5/0/5/10 dBm ohne PA-Ramping (FREND0 = 0x11)
//   Index > 9 → auf 8 geclamped (factory-default 5 dBm no ramp)
//   Default ohne Argument: idx=2 (0 dBm mit Ramping) — byte-genau a-culfw,
//   weil `fromhex(in+1, &hb, 1)` bei leerem Input `hb=2` belaesst.
//
// CULFW32-Anpassung gegenueber a-culfw: a-culfw ruft danach `ccInitChip()`,
// das die ganze CC1100_CFG-Tabelle aus EE neu in den Chip schreibt — das
// zerstoert den aktiven RF-Mode (Moritz/AskSin/SlowRf usw.). Wir schreiben
// stattdessen chirurgisch nur PATABLE-Burst (8 Bytes) + FREND0 (Reg 0x22),
// damit der laufende RF-Mode nicht abbricht. Effekt auf TX-Power identisch,
// User-Erfahrung besser.
//
// Caveat: RF-Klassen-Mode-Tabellen schreiben weiterhin `0x3E, 0xC3` als
// Single-Byte-Write beim `start()` — das setzt PATABLE[0] (Idle-Power-Index).
// Bei FREND0=0x11 (no ramp) verwendet die CC1101 PATABLE[1] fuer TX, also
// ueberlebt der `x<idx>`-Wert einen Mode-Switch. Bei FREND0=0x17 (ramping,
// idx 0..4) wird die Ramp-Sequenz von einem Mode-Switch teilweise
// ueberschrieben — bekanntes Limit, dokumentiert.
void handle_x(std::span<const char> args, Transport* src, uint8_t slot_id,
              RfManager* mgr, NvsEeprom* eeprom) {
    if (!eeprom || !mgr) return;
    uint8_t idx = 2;                     // a-culfw default wenn fromhex 0 Bytes
    if (!args.empty()) hex2(args, idx);  // bei Parse-Fehler bleibt idx=2

    // EE schreiben (cc_set_pa clamped idx > 9 auf 8).
    cc_set_pa(*eeprom, idx);

    // PATABLE-Burst aus EE → Chip (alle Slots, weil PATABLE pro CC1101 ist).
    uint8_t pa[ee::CC1100_PA_SIZE];
    eeprom->read_block(ee::CC1100_PA, std::span<uint8_t>(pa, sizeof(pa)));
    const uint8_t frend0 = eeprom->read_byte(ee::CC1100_CFG + 0x22);
    constexpr uint8_t CC_PATABLE = 0x3E;

    for (uint8_t s = 0; s < mgr->slot_count(); ++s) {
        mgr->cc_write_burst(s, CC_PATABLE, std::span<const uint8_t>(pa, sizeof(pa)));
        mgr->cc_write_reg  (s, 0x22, frend0);
    }
    // a-culfw silent on success.
    (void)src; (void)slot_id;
}

// `?`-Befehl: ConfigRegistry-Introspection (CULFW32-Erweiterung).
//   ?              KEIN Registry-Befehl: antwortet wie a-culfw mit
//                  "? (? is unknown) Use one of ..." (dispatch_line). FHEM
//                  liest daraus beim Init die Befehlsliste (CMDS).
//   ??             Liste aller Eintraege als Tabelle "? <key> = <value>"
//   ?<key>         Wert eines Eintrags ausgeben
//   ?<key>=<val>   Wert setzen — alternative zu spezifischen ASCII-Befehlen
//   ?dump          Maschinenlesbar: ein Eintrag pro Zeile als
//                  "<key>\t<type>\t<flags>\t<value>" (für JSON-Parser-Adapter).
// Frontend-Eingang fuer kuenftige UI/MQTT-Komponenten.
void handle_questionmark(std::span<const char> args, Transport* src,
                          uint8_t slot_id, ConfigRegistry* cfg, RfManager* mgr) {
    if (!cfg) {
        write_prefixed(src, slot_id, "? ERR cfg unavail\r\n");
        return;
    }

    auto type_name = [](ConfigEntry::Type t) -> const char* {
        switch (t) {
            case ConfigEntry::Type::Bool:     return "bool";
            case ConfigEntry::Type::U8:       return "u8";
            case ConfigEntry::Type::U16:      return "u16";
            case ConfigEntry::Type::U32:      return "u32";
            case ConfigEntry::Type::Str:      return "str";
            case ConfigEntry::Type::Secret:   return "secret";
            case ConfigEntry::Type::Ipv4Cidr: return "ipv4cidr";
            case ConfigEntry::Type::IntEnum:  return "intenum";
        }
        return "?";
    };

    if (args.size() == 1 && args[0] == '?') {
        // `??` — schoene Tabelle.
        cfg->iter([&](const ConfigEntry& e) {
            char buf[256];
            std::string val = cfg->get(e.key);
            std::snprintf(buf, sizeof(buf), "? %-22s = %s%s%s\r\n",
                          e.key.c_str(), val.c_str(),
                          e.unit.empty() ? "" : " ",
                          e.unit.empty() ? "" : e.unit.c_str());
            write_prefixed(src, slot_id, buf);
        });
        char buf[64];
        std::snprintf(buf, sizeof(buf), "? %u entries\r\n",
                      static_cast<unsigned>(cfg->size()));
        write_prefixed(src, slot_id, buf);
        return;
    }

    // `?stats` — kompakter Tabular-Dump (RfStats::format_text + Stack-Highwater).
    // Zusaetzlich bleiben die einzelnen `stats.*`-Keys ueber `?stats.rf.lovf`
    // etc abrufbar.
    if (args.size() == 5 && args[0] == 's' && args[1] == 't' && args[2] == 'a'
        && args[3] == 't' && args[4] == 's') {
        char buf[2048];
        size_t n = RfStats::instance().format_text(buf, sizeof(buf));
        if (n + 2 < sizeof(buf)) {
            // Sys-Header: Uptime, Boot/Crash-Counter, Reset-Reason.
            n += std::snprintf(buf + n, sizeof(buf) - n,
                "\r\nsys: uptime=%s total=%s boot=%u crash=%u last=%s\r\n",
                SystemMonitor::format_uptime(SystemMonitor::instance().session_uptime_s()).c_str(),
                SystemMonitor::format_uptime(SystemMonitor::instance().uptime_total_s()).c_str(),
                static_cast<unsigned>(SystemMonitor::instance().boot_count()),
                static_cast<unsigned>(SystemMonitor::instance().crash_count()),
                SystemMonitor::reset_reason_short(SystemMonitor::instance().last_reset()));
        }
        // Die-Temperatur des ESP32-C6 (nicht Umgebung), Min/Max seit Boot.
        if (n + 2 < sizeof(buf)) {
            const auto t = SystemMonitor::instance().chip_temp();
            if (t.valid) {
                n += std::snprintf(buf + n, sizeof(buf) - n,
                    "temp: chip=%sC min=%sC max=%sC\r\n",
                    SystemMonitor::format_dc(t.now_dc).c_str(),
                    SystemMonitor::format_dc(t.min_dc).c_str(),
                    SystemMonitor::format_dc(t.max_dc).c_str());
            } else {
                n += std::snprintf(buf + n, sizeof(buf) - n, "temp: chip=n/a\r\n");
            }
        }
        // USB-CDC TX-Queue Telemetrie (Producer-Consumer-Diagnose).
        // Bilanz: enq = sent + evict + drop_gone (+ ggf. in-flight); drop_full
        // zaehlt write()-Aufrufe die nicht mehr in die Queue passten.
        if (auto* cdc = UsbCdcTransport::instance(); cdc && n + 2 < sizeof(buf)) {
            n += std::snprintf(buf + n, sizeof(buf) - n,
                "cdc: enq=%u sent=%u evict=%u drop_full=%u drop_gone=%u\r\n",
                static_cast<unsigned>(cdc->tx_enqueued()),
                static_cast<unsigned>(cdc->tx_sent()),
                static_cast<unsigned>(cdc->tx_evicted_oldest()),
                static_cast<unsigned>(cdc->tx_dropped_full()),
                static_cast<unsigned>(cdc->tx_dropped_host_gone()));
        }
        // CC1101-PLL-Telemetrie pro Slot. lost>0 = der Heartbeat sah den Chip
        // in RX ohne Lock (FSCAL1==0x3F, SWRS061I §22.1 p.58) und hat
        // nachkalibriert; relock>0 = dabei war ein Retry noetig; fail>0 = Lock
        // nicht wiederhergestellt, der Slot ist funktional taub. Alles bleibt
        // sonst unsichtbar, weil ein CC1101 ohne Lock keinen Fehler meldet,
        // sondern einfach schweigt.
        if (mgr && n + 2 < sizeof(buf)) {
            for (uint8_t s = 0; s < mgr->slot_count(); ++s) {
                uint32_t lost = 0, relocks = 0, failures = 0;
                if (mgr->cc_pll_counters(s, lost, relocks, failures) != ESP_OK) continue;
                // RX-FIFO: extra = Folgerahmen, in derselben Runde hinter einem
                // anderen gelesen (Bergungen durch den Heartbeat zaehlen nicht);
                // stale = GDO2 stand bei leerem FIFO; resync = Rahmenkopf passte
                // nicht oder CRC_OK fehlte, FIFO geflusht (Overflow zaehlt nicht).
                uint32_t extra = 0, stale = 0, resync = 0;
                mgr->cc_rx_counters(s, extra, stale, resync);
                if (n + 2 >= sizeof(buf)) break;
                n += std::snprintf(buf + n, sizeof(buf) - n,
                    "cc%u: pll_lost=%u pll_relock=%u pll_fail=%u"
                    " rx_extra=%u rx_stale=%u rx_resync=%u\r\n",
                    static_cast<unsigned>(s),
                    static_cast<unsigned>(lost),
                    static_cast<unsigned>(relocks),
                    static_cast<unsigned>(failures),
                    static_cast<unsigned>(extra),
                    static_cast<unsigned>(stale),
                    static_cast<unsigned>(resync));
            }
        }
        // TcpRxPool-Telemetrie — Saettigung (pending) und Job-Bilanz.
        if (n + 2 < sizeof(buf)) {
            auto& pool = TcpRxPool::instance();
            n += std::snprintf(buf + n, sizeof(buf) - n,
                "tcp-pool: workers=%u pending=%u submitted=%u completed=%u rejected=%u\r\n",
                static_cast<unsigned>(pool.pool_size()),
                static_cast<unsigned>(pool.queue_pending()),
                static_cast<unsigned>(pool.jobs_submitted()),
                static_cast<unsigned>(pool.jobs_completed()),
                static_cast<unsigned>(pool.jobs_rejected()));
        }
        // Netz-Clients ueber die TxQueue (seit 0.1.902), getrennt nach 3 s ohne
        // Abfluss (TCP bzw. WebSocket), weil ihr Puffer voll war (overflow),
        // weil kein Puffer frei war (nobuf: Budget fuer vier gestaute Clients
        // erschoepft, malloc gescheitert oder Online-Update laeuft, Log
        // "shed") oder nach einem Sendefehler
        // (err: Client weg, Netif down). queue_peak = groesster Rueckstand
        // eines Clients seit Boot. Neue Felder nur hinten anhaengen.
        if (n + 2 < sizeof(buf)) {
            n += std::snprintf(buf + n, sizeof(buf) - n,
                "tcp-tx: stall_drop=%u ws_stall_drop=%u overflow_drop=%u queue_peak=%u"
                " nobuf_drop=%u err_drop=%u\r\n",
                static_cast<unsigned>(TcpClientTransport::stall_drops()),
                static_cast<unsigned>(WebSocketTransport::stall_drops()),
                static_cast<unsigned>(TxQueue::overflow_drops()),
                static_cast<unsigned>(TxQueue::peak_bytes()),
                static_cast<unsigned>(TxQueue::nobuf_drops()),
                static_cast<unsigned>(TxQueue::error_drops()));
        }
        if (n + 2 < sizeof(buf)) {
            n += std::snprintf(buf + n, sizeof(buf) - n, "\r\n");
        }
        // Erste Haelfte (rfstats + sys + cdc + tcp-pool) schon jetzt rausgeben —
        // mit wachsendem Task-Inventar (Pool fuegt 8 Worker hinzu) ueberschritt
        // das Ein-Stueck-`write()` TX_SCRATCH=2048 → USB-CDC drop-full, kompletter
        // `?stats`-Reply ging verloren. Split per Sektion umgeht das.
        // Buffer wird wiederverwendet — keine zweite 2-KB-Allocation auf dem
        // Stack der usb-cdc-rx-Task (sonst Stack-Overflow, MCAUSE=0x1b).
        if (n > 0 && src) {
            src->write({reinterpret_cast<const uint8_t*>(buf), n});
        }
        // Zweiter Chunk: Task-Highwater-Tabelle, wiederverwendeter Buffer.
        n = SystemMonitor::instance().format_task_stats(buf, sizeof(buf));
        if (n > 0 && src) {
            src->write({reinterpret_cast<const uint8_t*>(buf), n});
        }
        return;
    }

    // `?heap` — Heap-Detail-Dump (Fragmentation-Diagnose). Liefert pro Cap
    // (DEFAULT/INTERNAL/DMA) free / minimum-free / largest-free-block /
    // allocated-blocks / free-blocks / total-blocks. Diff free vs largest zeigt
    // Fragmentation; largest << free bedeutet zerstueckelter Heap. Pendant zu
    // `?stats` heap-Zeile, aber mit Fragmentation- + Per-Cap-Sicht.
    if (args.size() == 4 && args[0] == 'h' && args[1] == 'e'
        && args[2] == 'a' && args[3] == 'p') {
        char buf[1024];
        size_t n = 0;
        struct CapRow { const char* name; uint32_t caps; };
        static constexpr CapRow rows[] = {
            {"default",  MALLOC_CAP_DEFAULT},
            {"internal", MALLOC_CAP_INTERNAL},
            {"dma",      MALLOC_CAP_DMA},
            {"8bit",     MALLOC_CAP_8BIT},
        };
        n += std::snprintf(buf + n, sizeof(buf) - n,
            "heap  cap          free    min_free  largest  alloc_b  free_b  total_b\r\n");
        for (const auto& r : rows) {
            multi_heap_info_t info{};
            heap_caps_get_info(&info, r.caps);
            size_t largest = heap_caps_get_largest_free_block(r.caps);
            size_t free_b  = heap_caps_get_free_size(r.caps);
            size_t min_b   = heap_caps_get_minimum_free_size(r.caps);
            n += std::snprintf(buf + n, (n < sizeof(buf) ? sizeof(buf) - n : 0),
                "heap  %-10s %7u %9u %8u %8u %7u %8u\r\n",
                r.name,
                static_cast<unsigned>(free_b),
                static_cast<unsigned>(min_b),
                static_cast<unsigned>(largest),
                static_cast<unsigned>(info.allocated_blocks),
                static_cast<unsigned>(info.free_blocks),
                static_cast<unsigned>(info.total_blocks));
        }
        if (n > 0 && src) src->write({reinterpret_cast<const uint8_t*>(buf), n});
        return;
    }

    // `?zombies` — Task-Inventar + Zombie-Detector. Zaehlt Tasks im
    // eDeleted-Zustand (TCB nicht von Idle-Task aufgeraeumt) und tabuliert
    // pro-Name-Counts. Akkumulation z.B. "tcp-rx" > 4 deutet auf zombie-Backlog
    // bzw. Idle-Task-Starvation hin (siehe heap_leak_stress_residual).
    if (args.size() == 7 && args[0] == 'z' && args[1] == 'o' && args[2] == 'm'
        && args[3] == 'b' && args[4] == 'i' && args[5] == 'e' && args[6] == 's') {
        UBaseType_t n_tasks = uxTaskGetNumberOfTasks();
        if (n_tasks == 0) {
            write_prefixed(src, slot_id, "zombies 0 tasks\r\n");
            return;
        }
        auto* arr = reinterpret_cast<TaskStatus_t*>(
            heap_caps_calloc(n_tasks, sizeof(TaskStatus_t), MALLOC_CAP_DEFAULT));
        if (!arr) {
            write_prefixed(src, slot_id, "zombies ERR alloc\r\n");
            return;
        }
        UBaseType_t got = uxTaskGetSystemState(arr, n_tasks, nullptr);

        // Pass 1: counts.
        unsigned total = 0, deleted = 0, blocked = 0, ready = 0, running = 0, suspended = 0;
        for (UBaseType_t i = 0; i < got; ++i) {
            ++total;
            switch (arr[i].eCurrentState) {
                case eDeleted:   ++deleted;   break;
                case eBlocked:   ++blocked;   break;
                case eReady:     ++ready;     break;
                case eRunning:   ++running;   break;
                case eSuspended: ++suspended; break;
                default: break;
            }
        }

        char buf[1024];
        size_t n = std::snprintf(buf, sizeof(buf),
            "zombies total=%u del=%u run=%u rdy=%u blk=%u sus=%u\r\n",
            total, deleted, running, ready, blocked, suspended);

        // Per-Name-Count (catch tcp-rx-Eskalation). Naive O(n^2) ueber visited-
        // Bitmap — pcTaskName zeigt in lebende TCBs, also NICHT mutieren.
        auto* visited = reinterpret_cast<uint8_t*>(
            heap_caps_calloc(got, 1, MALLOC_CAP_DEFAULT));
        if (visited) {
            for (UBaseType_t i = 0; i < got && n < sizeof(buf); ++i) {
                if (visited[i]) continue;
                if (arr[i].pcTaskName == nullptr || arr[i].pcTaskName[0] == '\0') continue;
                unsigned cnt = 1;
                visited[i] = 1;
                const char* nm = arr[i].pcTaskName;
                for (UBaseType_t j = i + 1; j < got; ++j) {
                    if (visited[j]) continue;
                    if (arr[j].pcTaskName && std::strcmp(arr[j].pcTaskName, nm) == 0) {
                        visited[j] = 1;
                        ++cnt;
                    }
                }
                if (cnt > 1) {
                    n += std::snprintf(buf + n, (n < sizeof(buf) ? sizeof(buf) - n : 0),
                        "zombies   %-18s x%u\r\n", nm, cnt);
                }
            }
            heap_caps_free(visited);
        }

        // Liste der eDeleted-Tasks (sollte leer sein wenn Idle gesund).
        if (deleted > 0) {
            for (UBaseType_t i = 0; i < got && n < sizeof(buf); ++i) {
                if (arr[i].eCurrentState != eDeleted) continue;
                n += std::snprintf(buf + n, (n < sizeof(buf) ? sizeof(buf) - n : 0),
                    "zombies   DEL %s prio=%u\r\n",
                    arr[i].pcTaskName ? arr[i].pcTaskName : "(?)",
                    static_cast<unsigned>(arr[i].uxCurrentPriority));
            }
        }

        heap_caps_free(arr);
        if (n > 0 && src) src->write({reinterpret_cast<const uint8_t*>(buf), n});
        return;
    }

    // `?dump` — maschinenlesbar.
    if (args.size() == 4 && args[0] == 'd' && args[1] == 'u'
        && args[2] == 'm' && args[3] == 'p') {
        cfg->iter([&](const ConfigEntry& e) {
            char buf[320];
            std::string val = cfg->get(e.key);
            std::snprintf(buf, sizeof(buf), "?d %s\t%s\t%s%s%s\t%s\r\n",
                          e.key.c_str(), type_name(e.type),
                          e.persist()   ? "P" : "-",
                          e.readonly()  ? "R" : "-",
                          e.sensitive() ? "S" : "-",
                          val.c_str());
            write_prefixed(src, slot_id, buf);
        });
        return;
    }

    // `?<key>` oder `?<key>=<val>`.
    size_t eq = 0;
    while (eq < args.size() && args[eq] != '=') ++eq;
    auto key  = args.subspan(0, eq);
    std::string key_s(key.data(), key.size());

    if (eq < args.size()) {
        auto val = args.subspan(eq + 1);
        std::string val_s(val.data(), val.size());
        if (cfg->set(key_s, val_s)) {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "? %s = %s\r\n",
                          key_s.c_str(), cfg->get(key_s).c_str());
            write_prefixed(src, slot_id, buf);
        } else {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "? %s ERR (unknown/readonly/parse)\r\n",
                          key_s.c_str());
            write_prefixed(src, slot_id, buf);
        }
        return;
    }

    auto* e = cfg->find(key_s);
    if (!e) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "? %s unknown\r\n", key_s.c_str());
        write_prefixed(src, slot_id, buf);
        return;
    }
    char buf[256];
    std::string val = cfg->get(key_s);
    std::snprintf(buf, sizeof(buf), "? %s = %s%s%s  (%s%s%s%s)\r\n",
                  key_s.c_str(), val.c_str(),
                  e->unit.empty() ? "" : " ",
                  e->unit.empty() ? "" : e->unit.c_str(),
                  type_name(e->type),
                  e->persist()   ? ", persist"  : "",
                  e->readonly()  ? ", ro"       : "",
                  e->sensitive() ? ", sensitive": "");
    write_prefixed(src, slot_id, buf);
    if (!e->help.empty()) {
        std::snprintf(buf, sizeof(buf), "?    %s\r\n", e->help.c_str());
        write_prefixed(src, slot_id, buf);
    }
}

// Tag-Liste fuer "Use one of": genau die Tags aus dem switch in
// dispatch_line(), in a-culfw-fntab-Reihenfolge (Buchstabe fuer Buchstabe,
// Gross vor Klein), jeder als " <tag>" wie callfn(0). FHEM baut daraus CMDS
// (00_CUL.pm CUL_DoInit) und gibt rfmode danach frei: HomeMatic braucht `A`,
// MAX `Z`, WMBus_* `b`; `Ax`/`Zx`/`brx` schickt es nur fuer gelistete Tags.
// Das nackte `?` fehlt bewusst — es liefert selbst diese Zeile.
#ifdef CONFIG_CULFW32_BUILD_RF_WMBUS
#define CULFW32_TAG_b " b"
#else
#define CULFW32_TAG_b ""
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_HMIP
#define CULFW32_TAG_P " P"
#else
#define CULFW32_TAG_P ""
#endif
constexpr const char* kKnownTags =
    " A B" CULFW32_TAG_b " C D E e F f K l M N" CULFW32_TAG_P " q R T t V W X x Z";
#undef CULFW32_TAG_b
#undef CULFW32_TAG_P

// Legacy-Format byte-genau a-culfw `analyze_ttydata` (ttydata.c:92-98) wenn
// callfn() den Befehl nicht dispatchen kann. Bei Multi-CC: gibt's das Tag
// `*` in fntab, callfn returns 0 wenn has_CC(slot+1)==false → derselbe Pfad.
//
//   ? (<original-input> is unknown) Use one of <tag-list>\r\n
//
// KEIN `*`-Prefix hier (a-culfw gibt das auch ohne MULTICC_PREFIX aus, weil
// CC1101.instance noch 0 ist wenn callfn returns 0). Den Tag `*` listet
// a-culfw nur, wenn es einen weiteren CC1101 gibt (callfn() mit
// USE_HW_AUTODETECT), bei MapleCUN am Ende der fntab.
// Das gilt nur fuer den callfn-Pfad (Slot 0, oder `*<cmd>` auf einen Slot, den
// es nicht gibt). Fuer `*<cmd>` auf einen EXISTIERENDEN weiteren Slot druckt
// a-culfw multiCC_func() den Prefix, die Tag-Liste dieses Slots (fntab1) und
// `*` nur bei noch einem CC1101 (multi_CC.c:67-73); hier kaeme dieselbe Zeile
// wie fuer Slot 0. Auf CUL32 (ein Slot) nicht erreichbar -- erst fuer eine
// Mehrfunk-Variante nachziehen.
void write_unknown(Transport* t, std::span<const char> original, const RfManager* mgr) {
    const bool star = mgr && mgr->has_slot(1);
    char buf[256];
    int n = std::snprintf(buf, sizeof(buf), "? (%.*s is unknown) Use one of%s%s\r\n",
                          static_cast<int>(original.size()), original.data(), kKnownTags,
                          star ? " *" : "");
    if (n <= 0) return;
    // snprintf liefert die Soll-Laenge; abgeschnitten nie hinter buf lesen
    // (heute hoechstens 204 Byte bei kLineBufSize 128).
    t->write({reinterpret_cast<const uint8_t*>(buf),
              std::min(static_cast<size_t>(n), sizeof(buf) - 1)});
}

void dispatch_line(std::span<const char> line, Transport* src, const FrontendServices& s) {
    if (line.empty() || !s.rf_manager) return;
    auto original = line;     // fuer "? (...) is unknown" Output
    uint8_t slot_id = 0;
    while (!line.empty() && line[0] == '*') { ++slot_id; line = line.subspan(1); }
    if (line.empty()) return;

    // Multi-CC: `*<cmd>` auf nicht-existentem Slot → Legacy-Fehlermeldung byte-genau
    // a-culfw. Vorher hatten wir `<tag> NA\r\n` was FHEM als gueltige Antwort
    // missverstehen koennte.
    if (slot_id > 0 && !s.rf_manager->has_slot(slot_id)) {
        write_unknown(src, original, s.rf_manager);
        return;
    }

    char tag = line[0];
    auto args = line.subspan(1);
    switch (tag) {
        case 'V': handle_V(args, src, slot_id, s.rf_manager); break;
        case 'C': handle_C(args, src, slot_id, s.rf_manager); break;
        case 'R': handle_R(args, src, slot_id, s.rf_manager, s.eeprom); break;
        case 'W': handle_W(args, src, slot_id, s.rf_manager, s.eeprom); break;
        case 'Z': handle_Z(args, src, slot_id, s.rf_manager, s.eeprom); break;
        case 'A': handle_A(args, src, slot_id, s.rf_manager); break;
        case 'F': handle_F(args, src, slot_id, s.rf_manager); break;
        case 'K': handle_K(args, src, slot_id, s.rf_manager); break;
        case 'T': handle_T(args, src, slot_id, s.rf_manager, s.eeprom); break;
        case 'X': handle_X(args, src, slot_id, s.rf_manager); break;
        case 'N': handle_N(args, src, slot_id, s.rf_manager); break;
#ifdef CONFIG_CULFW32_BUILD_RF_WMBUS
        case 'b': handle_b(args, src, slot_id, s.rf_manager); break;
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_HMIP
        case 'P': handle_P(args, src, slot_id, s.rf_manager); break;
#endif
        case 'f': handle_f(args, src, slot_id, s.rf_manager); break;
        case 'D': handle_D(args, src, slot_id, s.rf_manager); break;
        case 'E': handle_E(args, src, slot_id, s.wifi, s.eth, s.rf_manager,
                           s.eeprom, s.tcp_server); break;
        case 'M': handle_M(args, src, slot_id, s.rf_manager); break;
        case 't': handle_t(args, src, slot_id); break;
        case 'B': handle_B(args, src, slot_id, s.eeprom); break;
        case 'e': handle_e(args, src, slot_id, s.eeprom); break;
        case 'q': handle_q(args, src, slot_id); break;
        case 'l': handle_l(args, src, slot_id, s.led, s.eeprom); break;
        case 'x': handle_x(args, src, slot_id, s.rf_manager, s.eeprom); break;
        case '?':
            // Nacktes `?` wie a-culfw: FHEM schickt es bei jeder Anmeldung
            // (`get cmds`) und wartet 3 s auf "Use one of" — ohne die Zeile
            // trennt es die Verbindung. Die Registry-Tabelle liegt unter `??`.
            if (args.empty()) write_unknown(src, original, s.rf_manager);
            else handle_questionmark(args, src, slot_id, s.config, s.rf_manager);
            break;
        default:
            // Unbekanntes Tag → Legacy-Fehlermeldung byte-genau a-culfw.
            write_unknown(src, original, s.rf_manager);
            break;
    }
}

struct LineBuf {
    std::array<char, kLineBufSize> data{};
    size_t len = 0;
};

// Pro Transport ein eigener LineBuf — auch fuer dynamisch hinzukommende
// TCP-Clients. Map ist hash-frei (linear search ueber wenige Eintraege),
// reicht dicke fuer typische Kunden-Setups (USB+UART+1-2 TCP).
struct TransportLine {
    Transport* t;
    LineBuf    buf;
};
constexpr size_t kMaxTransports = 16;
std::array<TransportLine, kMaxTransports> g_lines{};
size_t                                    g_lines_count = 0;
SemaphoreHandle_t                         g_lines_mtx   = nullptr;

// LineBuf fuer src holen (oder neu anlegen). Mutex-geschuetzt.
// release_line_for: Member-Sichtbar via forward-decl ganz oben.
void release_line_for(Transport* t) {
    if (!g_lines_mtx) return;
    xSemaphoreTake(g_lines_mtx, portMAX_DELAY);
    for (size_t i = 0; i < g_lines_count; ++i) {
        if (g_lines[i].t == t) {
            g_lines[i] = g_lines[--g_lines_count];
            g_lines[g_lines_count].t = nullptr;
            break;
        }
    }
    xSemaphoreGive(g_lines_mtx);
}

LineBuf* line_for_locked(Transport* t) {
    if (!g_lines_mtx) g_lines_mtx = xSemaphoreCreateMutex();
    xSemaphoreTake(g_lines_mtx, portMAX_DELAY);
    for (size_t i = 0; i < g_lines_count; ++i) {
        if (g_lines[i].t == t) {
            xSemaphoreGive(g_lines_mtx);
            return &g_lines[i].buf;
        }
    }
    if (g_lines_count >= g_lines.size()) {
        xSemaphoreGive(g_lines_mtx);
        return nullptr;
    }
    g_lines[g_lines_count].t   = t;
    g_lines[g_lines_count].buf = LineBuf{};
    LineBuf* p = &g_lines[g_lines_count].buf;
    g_lines_count++;
    xSemaphoreGive(g_lines_mtx);
    return p;
}

} // namespace

void CulfwFrontend::handle_rx(std::span<const uint8_t> data, Transport* src) {
    auto* lb = line_for_locked(src);
    if (!lb) return;
    for (uint8_t b : data) {
        if (b == '\r' || b == '\n') {
            if (lb->len > 0) {
                dispatch_line({lb->data.data(), lb->len}, src, services_);
                lb->len = 0;
            }
        } else if (lb->len < lb->data.size()) {
            lb->data[lb->len++] = static_cast<char>(b);
        }
        // Volle Zeile: weitere Zeichen bis CR/LF verwerfen, die gekuerzte Zeile
        // laeuft am Zeilenende wie in a-culfw analyze_ttydata (TTY_BUFSIZE 128).
        // Bis 0.1.871 begann hier eine NEUE Zeile, und der Rest einer
        // ueberlangen Zeile lief als eigener Befehl -- gemessen 08.10.2026:
        // der Ueberhang "B05" eines 131 Zeichen langen bst-Befehls startete
        // das Board neu; je nach Daten waeren auch e (Werksreset), F oder W
        // getroffen worden.
    }
}

esp_err_t CulfwFrontend::start() {
    if (!services_.rf_manager) return ESP_ERR_INVALID_STATE;
    // FHT-Hauscode aus EE restaurieren (a-culfw fht.c:125 `fht_init()` —
    // wird bei jedem Boot gerufen). Uninitialisiert (0xFF) → 0x00, sonst
    // wuerden FHEM-Module FF:FF als gueltigen Hauscode interpretieren.
    if (services_.eeprom) {
        // Byte-genau a-culfw fht.c:127: raw aus EE laden, kein 0xFF-Mapping.
        // Lokales-CUL nach Erstboot zeigt `T01` -> `FFFF\r\n`; reproduzieren.
        g_fht_hc0 = services_.eeprom->read_byte(ee::FHTID);
        g_fht_hc1 = services_.eeprom->read_byte(ee::FHTID + 1);
    }
    register_codecs();
    for (auto& t : transports_) {
        Transport* raw = t.get();
        t->on_rx([this, raw](std::span<const uint8_t> d) { handle_rx(d, raw); });
        if (auto err = t->start(); err != ESP_OK) return err;
    }
    ESP_LOGI(TAG, "ready, %u transport(s)", static_cast<unsigned>(transports_.size()));
    return ESP_OK;
}

esp_err_t CulfwFrontend::stop() {
    for (auto& t : transports_) t->stop();
    return ESP_OK;
}

} // namespace culfw32
