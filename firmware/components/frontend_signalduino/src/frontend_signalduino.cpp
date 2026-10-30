// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
//
// Contains code ported from SIGNALduino (https://github.com/RFD-FHEM/SIGNALDuino):
//   send.h  no individual notice; the SIGNALDuino repository is licensed under
//           the GNU GPL version 3.
// Modified for CULFW32 (ESP-IDF port), Dirk Tostmann, 2026.

#include "frontend_signalduino.hpp"
#include "rf_manager.hpp"
#include "rf_protocol.hpp"
#include "signalduino_rf.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG = "fe-sd";

// Wir signalisieren PulseCapture (= SignalduinoRF) als kompatibel.
// Andere Protokolle akzeptieren wir nicht — die kommen ueber andere Frontends.
constexpr ProtocolId kCompatible[] = { ProtocolId::PulseCapture };

// FHEM-SIGNALduino-Modul akzeptiert Versions-Strings die das Regex
//   V\s.*SIGNAL(?:duino|ESP|STM).*\s\d\d:\d\d:\d\d
// matchen UND Mindestversion 3.2 erfuellen. Wir geben CULFW32-Build-Datum
// als HH:MM:SS-Suffix (statt Echtzeit) zurueck — das genuegt dem Regex.
// Build-Date kommt via culfw_version.hpp aus dem Build-System.
constexpr const char* kVersionBanner =
    "V 3.4.0 SIGNALduino-CULFW32 cc1101 00:00:00\r\n";

// Kleiner Helper: Vergleich case-insensitive (FHEM-SD-Modul ist tolerant).
bool starts_with_i(std::span<const char> line, const char* p) {
    size_t n = 0;
    while (p[n]) ++n;
    if (line.size() < n) return false;
    for (size_t i = 0; i < n; ++i) {
        char a = line[i], b = p[i];
        if (a >= 'A' && a <= 'Z') a = static_cast<char>(a + 32);
        if (b >= 'A' && b <= 'Z') b = static_cast<char>(b + 32);
        if (a != b) return false;
    }
    return true;
}

} // anon

std::span<const ProtocolId> SignalduinoFrontend::compatible_protocols() const {
    return std::span<const ProtocolId>(kCompatible, std::size(kCompatible));
}

void SignalduinoFrontend::attach(const FrontendServices& services) {
    services_ = services;
    if (!mtx_) mtx_ = xSemaphoreCreateMutex();
}

void SignalduinoFrontend::bind_transport(std::shared_ptr<Transport> t) {
    if (!t || !mtx_) return;
    xSemaphoreTake(mtx_, portMAX_DELAY);
    transports_.push_back(std::move(t));
    xSemaphoreGive(mtx_);
}

void SignalduinoFrontend::attach_dynamic_transport(std::shared_ptr<Transport> t) {
    if (!t) return;
    bind_transport(t);
    Transport* raw = t.get();
    t->on_rx([this, raw](std::span<const uint8_t> bytes) {
        on_rx_data_(bytes, raw);
    });
    if (started_) {
        // Fehlerpfad wie im JSON-Frontend: ohne on_closed kein detach.
        if (t->start() != ESP_OK) {
            detach_dynamic_transport(raw);
            return;
        }
        // SignalDuino-Frontend MUSS einen FSK-RF-Mode (HmDual/AskSin/Moritz/...)
        // respektieren der schon laeuft.  Vorherige Logik unconditional auf
        // PulseCapture umzuschalten kickte HmDual aus Slot 0, sobald FHEM
        // sich (re)verbindet → AskSin/HmIP-RX "freeze" verifiziert
        // 2026-05-11/12 (siehe notes/decoder_freeze_investigation*.md).
        // Jetzt: nur PulseCapture aktivieren wenn Slot 0 leer ODER schon
        // PulseCapture.  Ein FSK-Mode der HmDual/AskSin/HmIP bedient bleibt
        // unangetastet; der SignalDuino-Client hat dann eben keine
        // Pulse-Events bis der User explizit `Xz`/SignalDuino-Restart
        // anfragt (Future-Work falls echter Pulse-Sniff-Bedarf besteht).
        if (services_.rf_manager) {
            auto active = services_.rf_manager->active_protocol_on(0);
            if (!active) {
                services_.rf_manager->activate(ProtocolId::PulseCapture, 0);
            } else if (active->id() != ProtocolId::PulseCapture) {
                ESP_LOGI(TAG, "client connect: slot busy (%d) — leaving as-is",
                         (int)active->id());
            }
        }
    }
}

void SignalduinoFrontend::detach_dynamic_transport(Transport* t) {
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
            line_buf_per_transport_[i].len = 0;
        }
    }
    xSemaphoreGive(mtx_);
}

esp_err_t SignalduinoFrontend::start() {
    if (started_) return ESP_OK;
    started_ = true;

    if (services_.rf_manager) {
        services_.rf_manager->subscribe(ProtocolId::PulseCapture, [this](const RxEvent& ev) {
            on_rx_event_(ev);
        });
        // PulseCapture sofort aktivieren, damit Pulse ab jetzt klassifiziert
        // werden — die emit-Pfade sind no-op solange transports_ leer ist.
        if (auto e = services_.rf_manager->activate(ProtocolId::PulseCapture, 0); e != ESP_OK) {
            ESP_LOGW(TAG, "activate PulseCapture: %s", esp_err_to_name(e));
        }
    }

    for (auto& t : transports_) {
        Transport* raw = t.get();
        t->on_rx([this, raw](std::span<const uint8_t> bytes) {
            on_rx_data_(bytes, raw);
        });
        t->start();
    }

    ESP_LOGI(TAG, "ready, %u transport(s)", static_cast<unsigned>(transports_.size()));
    return ESP_OK;
}

esp_err_t SignalduinoFrontend::stop() {
    started_ = false;
    return ESP_OK;
}

void SignalduinoFrontend::emit_to_(Transport* t, std::string_view text) {
    if (!t) return;
    t->write(std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(text.data()),
                                      text.size()));
}

void SignalduinoFrontend::broadcast_(std::string_view text) {
    if (!mtx_) return;
    xSemaphoreTake(mtx_, portMAX_DELAY);
    for (auto& t : transports_) {
        if (t) t->write(std::span<const uint8_t>(
            reinterpret_cast<const uint8_t*>(text.data()), text.size()));
    }
    xSemaphoreGive(mtx_);
}

void SignalduinoFrontend::on_rx_data_(std::span<const uint8_t> bytes, Transport* src) {
    if (!mtx_) return;
    xSemaphoreTake(mtx_, portMAX_DELAY);

    // LineBuf fuer diesen Transport finden bzw. allokieren.
    LineBuf* lb = nullptr;
    for (size_t i = 0; i < kMaxTransports; ++i) {
        if (lb_owner_[i] == src) { lb = &line_buf_per_transport_[i]; break; }
    }
    if (!lb) {
        for (size_t i = 0; i < kMaxTransports; ++i) {
            if (lb_owner_[i] == nullptr) {
                lb_owner_[i] = src;
                lb = &line_buf_per_transport_[i];
                lb->len = 0;
                break;
            }
        }
    }
    if (!lb) {
        xSemaphoreGive(mtx_);
        return;     // alle Slots belegt — ignorieren
    }

    for (auto b : bytes) {
        if (b == '\r') continue;
        if (b == '\n') {
            process_line_(std::span<const char>(lb->data.data(), lb->len), src);
            lb->len = 0;
            continue;
        }
        if (lb->len < lb->data.size()) {
            lb->data[lb->len++] = static_cast<char>(b);
        } else {
            // Zeile zu lang → reset, kein partial-dispatch.
            lb->len = 0;
        }
    }
    xSemaphoreGive(mtx_);
}

void SignalduinoFrontend::process_line_(std::span<const char> line, Transport* src) {
    if (line.empty()) return;

    if (starts_with_i(line, "V")) {
        // FHEM erwartet die Banner-Zeile als Antwort auf 'V'. Mehr Inhalt
        // wuerde es nicht stoeren, aber das Regex matcht so zuverlaessig.
        emit_to_(src, kVersionBanner);
        return;
    }
    if (starts_with_i(line, "XE")) {
        // Receiver-Enable. PulseCapture laeuft in unserem Modell schon ab
        // start(), deswegen reicht eine kurze Bestaetigung — manche FHEM-
        // Versionen ignorieren das, andere wollen einen leeren Echo.
        emit_to_(src, "XE\r\n");
        return;
    }
    if (starts_with_i(line, "XQ")) {
        emit_to_(src, "XQ\r\n");
        return;
    }
    if (starts_with_i(line, "?")) {
        emit_to_(src, "? (use V/XE/XQ)\r\n");
        return;
    }
    // SR-Send-Befehl byte-genau Real-SD-Firmware send.h:101..272.
    // Format: SR;R=<n>;P0=<us>;P1=<us>;...;P7=<us>;D=<index-string>;
    // Pro D-Char EIN Halbpuls (Vorzeichen aus buckets[idx], Dauer = abs).
    // FHEM-Modul nutzt SR fuer Raw-TX; wir parsen, bauen SignalduinoTxFrame
    // und reichen es ueber den RfManager an SignalduinoRF::tx weiter.
    if (starts_with_i(line, "SR;") && services_.rf_manager) {
        SignalduinoTxFrame f{};
        f.repeats = 1;
        // Pattern-Slots auf 0 lassen — nur die explizit gesetzten Pn= belegen
        // ihre Werte; "npat" ist der hoechste belegte Index + 1.
        std::string s(line.data(), line.size());
        size_t pos = 3;          // hinter "SR;"
        int max_pat_seen = -1;
        while (pos < s.size()) {
            size_t end = s.find(';', pos);
            if (end == std::string::npos) end = s.size();
            std::string_view tok(s.data() + pos, end - pos);
            if (tok.size() >= 3 && tok[0] == 'P' && tok[2] == '=' &&
                tok[1] >= '0' && tok[1] <= '7') {
                int idx = tok[1] - '0';
                int v = std::atoi(s.c_str() + pos + 3);
                if (v < -32767) v = -32767;
                if (v > 32767)  v = 32767;
                f.buckets[idx] = static_cast<int16_t>(v);
                if (idx > max_pat_seen) max_pat_seen = idx;
            } else if (tok.size() >= 2 && tok[0] == 'R' && tok[1] == '=') {
                int r = std::atoi(s.c_str() + pos + 2);
                if (r < 1) r = 1;
                if (r > 32) r = 32;
                f.repeats = static_cast<uint8_t>(r);
            } else if (tok.size() >= 2 && tok[0] == 'D' && tok[1] == '=') {
                size_t dn = tok.size() - 2;
                if (dn > sizeof(f.data)) dn = sizeof(f.data);
                std::memcpy(f.data, tok.data() + 2, dn);
                f.dlen = static_cast<uint16_t>(dn);
            }
            pos = end + 1;
        }
        f.npat = static_cast<uint8_t>(max_pat_seen + 1);
        if (f.dlen == 0 || f.npat == 0) {
            emit_to_(src, "SR ERR: missing D or P\r\n");
            return;
        }
        auto e = services_.rf_manager->tx(ProtocolId::PulseCapture, &f, sizeof(f));
        if (e == ESP_OK) {
            // Real-SD-FW echot die ersten chars der Kommandozeile zurueck —
            // wir nehmen das ganze Frame als Bestaetigung. Manche FHEM-Wege
            // erwarten kein ack; ein simpler Echo-Loop tut hier nicht weh.
            char buf[256];
            int n = snprintf(buf, sizeof(buf), "SR;ok;R=%u;dlen=%u;\r\n",
                             f.repeats, static_cast<unsigned>(f.dlen));
            emit_to_(src, std::string_view(buf, n));
        } else {
            char buf[64];
            int n = snprintf(buf, sizeof(buf), "SR ERR: %s\r\n", esp_err_to_name(e));
            emit_to_(src, std::string_view(buf, n));
        }
        return;
    }
    if (starts_with_i(line, "b")) {
        emit_to_(src, "b 0\r\n");
        return;
    }
    // CC1101-Reg-Read byte-genau SIGNALduino-Firmware cc1101.cpp:179..
    //   C<reg>             → "C<reg> = <hex>\r\n"
    //   C<reg>n<count>     → "C<reg>n<count+2>=<hex0><hex1>...\r\n"  (config 0..0x2E)
    //   C3E                → "C3E =<patable[0..7]>\r\n"             (PATABLE-Burst)
    //   C99                → ccreg 00: <hex0..0F> ccreg 10: <...> ccreg 20: <...>
    // FHEM ruft typischerweise C0DnF beim Connect ab (ccconf-readback).
    if ((line[0] == 'C' || line[0] == 'c') && line.size() >= 3 && services_.rf_manager) {
        char buf[256];
        auto hexnib = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };
        int h1 = hexnib(line[1]), h2 = hexnib(line[2]);
        if (h1 < 0 || h2 < 0) return;
        uint8_t reg = static_cast<uint8_t>((h1 << 4) | h2);

        // C99 → kompletter ccreg-Dump (0x00..0x2E) in 3 Zeilen `ccreg <base>: <16 bytes>`
        if (reg == 0x99) {
            for (uint8_t base = 0; base < 0x30; base += 0x10) {
                int n = snprintf(buf, sizeof(buf), "ccreg %02X: ", base);
                for (uint8_t i = 0; i < 0x10 && (base + i) < 0x30; ++i) {
                    uint8_t v = 0xFF;
                    services_.rf_manager->cc_read_reg(0, base + i, v);
                    n += snprintf(buf + n, sizeof(buf) - n, "%02X ", v);
                }
                n += snprintf(buf + n, sizeof(buf) - n, "\r\n");
                emit_to_(src, std::string_view(buf, n));
            }
            return;
        }
        // C3E → 8-byte PATABLE-Hex. PATABLE-Index inkrementiert NICHT bei
        // Single-Reads (8× read_reg(0x3E) liefert 8× PATABLE[0]); nur der
        // Burst-Read (Header 0xFE = READ|BURST|0x3E) laeuft die 8 Eintraege
        // sequentiell ab (SWRS061I §10.5 / Tabelle PATABLE-Access).
        if (reg == 0x3E) {
            uint8_t pa[8] = {};
            services_.rf_manager->cc_read_burst(0, 0x3E, std::span<uint8_t>(pa, sizeof(pa)));
            int n = snprintf(buf, sizeof(buf), "C3E =");
            for (int i = 0; i < 8; ++i)
                n += snprintf(buf + n, sizeof(buf) - n, "%02X", pa[i]);
            n += snprintf(buf + n, sizeof(buf) - n, "\r\n");
            emit_to_(src, std::string_view(buf, n));
            return;
        }
        // C<reg>n<count> → mehrere konsekutive Register
        if (line.size() >= 5 && (line[3] == 'n' || line[3] == 'N')) {
            int hc = hexnib(line[4]);
            if (hc < 0) return;
            int nh2 = (line.size() >= 6) ? hexnib(line[5]) : -1;
            uint8_t cnt = static_cast<uint8_t>(nh2 >= 0 ? ((hc << 4) | nh2) : hc);
            cnt += 2;     // SD-Firmware-Quirk: cnt ist + 2 in der Antwort
            if (reg < 0x2F) {
                int n = snprintf(buf, sizeof(buf), "C%02Xn%02X=", reg, cnt);
                for (uint8_t i = 0; i < cnt && (reg + i) < 0x30; ++i) {
                    uint8_t v = 0xFF;
                    services_.rf_manager->cc_read_reg(0, reg + i, v);
                    n += snprintf(buf + n, sizeof(buf) - n, "%02X", v);
                }
                n += snprintf(buf + n, sizeof(buf) - n, "\r\n");
                emit_to_(src, std::string_view(buf, n));
            }
            return;
        }
        // C<reg> → einzelnes Config- oder Status-Register
        if (reg < 0x3E) {
            uint8_t v = 0xFF;
            if (reg < 0x2F) services_.rf_manager->cc_read_reg(0, reg, v);
            else services_.rf_manager->cc_read_status(0, reg, v);
            int n = snprintf(buf, sizeof(buf), "C%02X = %02X\r\n", reg, v);
            emit_to_(src, std::string_view(buf, n));
        }
        return;
    }
    // Andere Befehle (e/CG/C0DnF/...) silently ignorieren — FHEM schickt
    // gelegentlich solche, behandelt aber das Ausbleiben einer Antwort als ok.
}

void SignalduinoFrontend::on_rx_event_(const RxEvent& ev) {
    if (ev.proto != ProtocolId::PulseCapture) return;
    auto* f = static_cast<const SignalduinoFrame*>(ev.payload);
    if (!f || f->len == 0) return;

    // FHEM-SIGNALduino-Modul (00_SIGNALduino.pm:3006) parst nur Frames die
    // mit STX (0x02) starten und ETX (0x03) enden:
    //     if (!($rmsg =~ s/^\002(M.;.*;)\003/$1/)) ... noMsg ...
    // Real SD-Firmware (signalDecoder.cpp:400/503) wrappt ihre MS/MU/MC
    // Frames mit MSG_START=0x02 vorne und MSG_END=0x03 + char(0xA) hinten.
    // DBG-Frames (mit '#'-Praefix) wrappen wir nicht — die sind unsere
    // Diagnose, keine SD-Frames.
    char out[800];
    int n = 0;
    bool wrap = (f->len >= 3 && f->ascii[0] == 'M' && f->ascii[2] == ';');
    if (wrap) out[n++] = '\x02';
    int copy_n = static_cast<int>(f->len);
    if (copy_n > static_cast<int>(sizeof(out)) - 5) copy_n = sizeof(out) - 5;
    std::memcpy(out + n, f->ascii, copy_n);
    n += copy_n;
    if (wrap) out[n++] = '\x03';
    out[n++] = '\r';
    out[n++] = '\n';
    broadcast_(std::string_view(out, n));
}

} // namespace culfw32
