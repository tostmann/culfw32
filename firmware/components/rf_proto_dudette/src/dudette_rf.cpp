// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "dudette_rf.hpp"
#include "rf_stats.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG = "dudette";

constexpr uint8_t CC_RXFIFO   = 0x3F;
constexpr uint8_t REG_MARC    = 0x35;

constexpr uint8_t CC_TXFIFO = 0x3F;
constexpr uint8_t REG_TXBYTES = 0x3A;
constexpr uint8_t STX   = 0x35;
constexpr uint8_t SFTX  = 0x3B;
constexpr uint8_t SRX   = 0x34;
constexpr uint8_t SIDLE = 0x36;
constexpr uint8_t SFRX  = 0x3A;

constexpr uint8_t MARC_IDLE            = 0x01;
constexpr uint8_t MARC_RX              = 0x0D;
constexpr uint8_t MARC_RXFIFO_OVERFLOW = 0x11;

// Registertabelle — identisch zur AVR-Seite in avr/dudette2/cc1101_regs.h.
// 868,4998 MHz, 99,976 kBaud GFSK, 50,78 kHz Hub, 325 kHz RX-Filter,
// Whitening an, kein Manchester, Sync 0x0D3B, variable Laenge + CRC.
// FSCTRL0 fehlt hier absichtlich: die Frequenzablage gehoert dem Board und
// wird vom Treiber nach dem SRES nachgezogen (eFuse/NVS).
const uint8_t DUDETTE_CFG[] = {
    0x00, 0x07,  0x02, 0x09,  0x03, 0x0D,  0x04, 0x0D,  0x05, 0x3B,
    0x06, 0x3A,  0x07, 0x0C,  0x08, 0x45,  0x09, 0x00,  0x0A, 0x00,
    0x0B, 0x0C,  0x0D, 0x21,  0x0E, 0x67,  0x0F, 0x62,  0x10, 0x5B,
    0x11, 0xF8,  0x12, 0x13,  0x13, 0x22,  0x14, 0xF8,  0x15, 0x50,
    0x17, 0x3F,  0x18, 0x18,  0x19, 0x1D,  0x1A, 0x1C,  0x1B, 0xC7,
    0x1C, 0x00,  0x1D, 0xB0,  0x21, 0xB6,  0x22, 0x10,  0x23, 0xEA,
    0x24, 0x2A,  0x25, 0x00,  0x26, 0x1F,
    0xFF
};

uint8_t marc_state(CC1101Driver& d) {
    uint8_t v = 0xFF;
    d.read_status(REG_MARC, v);
    return v & 0x1F;
}

// SWRS061I §17.3 / TI DN505: das RSSI-Byte ist ein Zweierkomplement in
// halben dB, davon der bandabhaengige Offset (868 MHz: 74 dB).
// ACHTUNG: nicht von rf_proto_fastrf abschreiben — dort steht fuer positive
// Rohwerte ein zusaetzliches "+64" in der Formel, das dort schon falsch ist
// und hier zu positiven dBm-Werten fuehrte.
int8_t rssi_to_dbm(uint8_t raw) {
    int16_t dbm = static_cast<int16_t>(static_cast<int8_t>(raw)) / 2 - 74;
    if (dbm < -128) dbm = -128;
    if (dbm >  127) dbm =  127;
    return static_cast<int8_t>(dbm);
}

// CRC-16/CCITT-FALSE — dieselbe Rechnung wie im Bootloader.
uint16_t crc16_ccitt(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= static_cast<uint16_t>(*d++) << 8;
        for (int i = 0; i < 8; ++i)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

// Los-Code fuer `Dl lot=` (Dudette doc/protocol.md §10.4): immer sechs Zeichen,
// jedes Byte ausserhalb 0x21..0x7E als '.'.  Bis 0.1.857 kuerzte ein NUL das
// Feld, und ein Leerzeichen ging durch und zerlegte die Zeile bei jedem
// Klienten, der nach Leerzeichen trennt.
void lot_printable(char out[7], const uint8_t* raw) {
    for (int i = 0; i < 6; ++i)
        out[i] = (raw[i] > 0x20 && raw[i] < 0x7F) ? static_cast<char>(raw[i]) : '.';
    out[6] = '\0';
}

uint16_t le16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t le32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
} // namespace

const CC1101Profile& DudetteRF::profile() const {
    static const CC1101Profile p = {
        .freq_hz      = 868'499'817,
        .datarate_bps = 99'976,
        .deviation_hz = 50'781,
        .mod          = CC1101Profile::Modulation::GFSK,
        .sync_word    = {0x0D, 0x3B, 0, 0},
        .sync_len     = 16,
        .pa_table     = {0xC0, 0, 0, 0, 0, 0, 0, 0},
        .raw_regs     = nullptr,
    };
    return p;
}

void IRAM_ATTR DudetteRF::gdo_isr(void* arg) {
    auto* self = static_cast<DudetteRF*>(arg);
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(self->task_handle_), &hpw);
    portYIELD_FROM_ISR(hpw);
}

void DudetteRF::rx_task_trampoline(void* arg) {
    static_cast<DudetteRF*>(arg)->rx_task_loop();
    // Eternal-Task, kein vTaskDelete — wie AskSin/Moritz/SlowRf/FastRf.
}

DudetteDevice* DudetteRF::upsert(const uint8_t* f, uint8_t flen, int8_t rssi_dbm,
                                 uint8_t lqi, bool& is_new)
{
    const uint32_t addr = (static_cast<uint32_t>(f[2]) << 16)
                        | (static_cast<uint32_t>(f[3]) << 8)
                        |  static_cast<uint32_t>(f[4]);

    DudetteDevice* d = nullptr;
    for (size_t i = 0; i < dev_count_; ++i) {
        if (devs_[i].addr == addr) { d = &devs_[i]; break; }
    }
    is_new = (d == nullptr);
    if (!d) {
        if (dev_count_ >= kMaxDevices) {
            // Liste voll: den am laengsten nicht gehoerten Eintrag verdraengen,
            // damit ein frisch gestartetes Geraet nicht ausgesperrt bleibt.
            size_t oldest = 0;
            for (size_t i = 1; i < dev_count_; ++i)
                if (devs_[i].last_seen_ms < devs_[oldest].last_seen_ms) oldest = i;
            d = &devs_[oldest];
            *d = DudetteDevice{};
        } else {
            d = &devs_[dev_count_++];
        }
        d->addr = addr;
    }

    // Vor dem Ueberschreiben pruefen, ob sich die Silizium-Kennung geaendert
    // hat.  Tut sie das unter derselben Adresse, sind es ZWEI Bausteine mit
    // demselben abgeleiteten Namen — sonst still, hier laut.
    if (!is_new) {
        char lot_neu[7];
        lot_printable(lot_neu, &f[9]);
        if (std::memcmp(d->lot, lot_neu, 6) != 0
            || std::memcmp(d->die, &f[15], 4) != 0) {
            if (d->collisions < 255) ++d->collisions;
        }
    }

    d->bl_version = f[6];
    d->caps       = f[7];
    d->ident      = static_cast<dudette::IdentSource>(f[8]);
    lot_printable(d->lot, &f[9]);
    std::memcpy(d->die, &f[15], 4);
    std::memcpy(d->sig, &f[19], 3);
    d->page_size = le16(&f[22]);
    d->app_size  = le32(&f[24]);
    d->board_id  = static_cast<uint16_t>((f[28] << 8) | f[29]);
    d->flags     = f[30];
    d->rssi_dbm  = rssi_dbm;
    d->lqi       = lqi;
    if (flen >= dudette::kHelloLen + dudette::kHelloDbgLen) {
        std::memcpy(d->dbg, &f[dudette::kHelloLen], dudette::kHelloDbgLen);
        d->has_dbg = true;
    }
    d->last_seen_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    d->hello_count++;
    return d;
}

void DudetteRF::rx_task_loop() {
    task_running_ = true;
    uint8_t buf[dudette::kMaxFrame];

    while (true) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0) {
            if (!active_) continue;
            // Ein uebergelaufenes RX-FIFO bleibt sonst stumm stehen.
            if (marc_state(*drv_) == MARC_RXFIFO_OVERFLOW) {
                drv_->strobe(SIDLE);
                drv_->strobe(SFRX);
                drv_->strobe(SRX);
            }
            continue;
        }
        if (!active_) continue;

        uint8_t len = 0;
        if (drv_->read_reg(CC_RXFIFO, len) != ESP_OK) continue;
        if (len == 0 || len > dudette::kMaxFrame) {
            drv_->strobe(SIDLE); drv_->flush_rx(); drv_->strobe(SRX);
            continue;
        }
        if (drv_->read_burst(CC_RXFIFO, {buf, len}) != ESP_OK) {
            drv_->strobe(SIDLE); drv_->flush_rx(); drv_->strobe(SRX);
            continue;
        }

        // PKTCTRL1 = 0x0C: APPEND_STATUS ist AN, der Chip haengt RSSI und
        // LQI/CRC an — anders als bei FastRF, dessen Tabelle 0x08 setzt.
        uint8_t rssi_raw = 0, lqi_crc = 0;
        drv_->read_reg(CC_RXFIFO, rssi_raw);
        drv_->read_reg(CC_RXFIFO, lqi_crc);

        const int8_t rssi_dbm = rssi_to_dbm(rssi_raw);
        if (!(lqi_crc & 0x80)) {
            RfStats::instance().on_crc_error(ProtocolId::Dudette);
            drv_->strobe(SRX);
            continue;
        }
        RfStats::instance().on_rx(ProtocolId::Dudette, len, rssi_dbm);

        if (buf[0] != dudette::kMagicVer) {          // fremdes Paket auf dem Sync
            drv_->strobe(SRX);
            continue;
        }
        // Antwort auf eine laufende Anfrage?  Dann weiterreichen, bevor die
        // HELLO-Auswertung greift — Antworten sind nie HELLOs.
        // Typ ALLEIN reicht nicht: eine verspaetete Antwort auf eine laengst
        // abgelaufene Anfrage traegt denselben Typ und wuerde als Antwort auf
        // die laufende durchgehen — mit ihrem alten Ergebnis und ihrer alten
        // Bitmap.  Der Bootloader spiegelt die Sequenznummer zurueck (Bit 7 ist
        // das Antwort-Flag), also hier dagegen pruefen.
        // Antwort auf ein scharfgestelltes SELECT (siehe arm()).
        if (arm_state_ == ArmState::Sent && buf[1] == dudette::kTypeStatus
            && (buf[5] & 0x7F) == (arm_seq_ & 0x7F)) {
            arm_status_ = (len > dudette::kHeaderLen) ? buf[dudette::kHeaderLen] : 0xFF;
            arm_state_  = ArmState::Done;
            drv_->strobe(SRX);
            continue;
        }
        if (await_type_ && buf[1] == await_type_ && reply_sem_
            && (buf[5] & 0x7F) == (seq_ & 0x7F)) {
            std::memcpy(reply_, buf, len);
            reply_len_ = len;
            // ERST den Chip zurueck in RX, DANN den Anfrager wecken.  In der
            // alten Reihenfolge lief der Anfrager (hoehere Prioritaet) sofort
            // in sein naechstes send_frame(), und das SRX von hier fiel ihm
            // zwischen SIDLE/SFTX und STX — ein STX aus RX heraus haengt am
            // CCA-Gate, das unmittelbar nach dem eigenen Empfang zu ist, und
            // das erste Fragment der naechsten Seite ging verloren.  Das ist
            // der Kandidat fuer das "r=2 bei nur einer Datenrunde am Geraet".
            drv_->strobe(SRX);
            xSemaphoreGive(static_cast<SemaphoreHandle_t>(reply_sem_));
            continue;
        }

        // Debug-Zeile einer Anwendung.  Kommt nur an, wenn der Debug-Kanal
        // eingeschaltet ist — auf dem normalen Kanal traegt sie ein anderes
        // Syncwort und wird schon von der Hardware verworfen.
        if (buf[1] == dudette::kTypePrint && len > dudette::kHeaderLen) {
            const uint32_t from = ((uint32_t)buf[2] << 16)
                                | ((uint32_t)buf[3] << 8) | buf[4];
            if (!dbg_filter_ || dbg_filter_ == from) {
                char txt[dudette::kMaxFrame];
                uint8_t n = static_cast<uint8_t>(len - dudette::kHeaderLen);
                if (n > sizeof(txt) - 1) n = sizeof(txt) - 1;
                for (uint8_t i = 0; i < n; ++i) {
                    uint8_t c = buf[dudette::kHeaderLen + i];
                    // Nur Druckbares durchlassen: eine verstuemmelte Zeile
                    // darf die Konsole nicht mit Steuerzeichen fluten.
                    txt[i] = (c >= 0x20 && c < 0x7F) ? static_cast<char>(c) : '.';
                }
                txt[n] = '\0';
                if (rx_cb_) {
                    DudetteRxEvent payload{nullptr, false, txt, from};
                    RxEvent ev{
                        .proto        = ProtocolId::Dudette,
                        .slot_id      = slot_id_,
                        .rssi_dbm     = rssi_dbm,
                        .timestamp_us = static_cast<uint32_t>(esp_timer_get_time()),
                        .payload      = &payload,
                    };
                    rx_cb_(ev);
                }
            }
            // SWRS061I 19.4, S. 52: ein SRX waehrend einer laufenden Sendung
            // beendet sie.  Der Rueckruf kann blockieren (TCP bis 0,5 s, bis
            // 0.1.893 unbegrenzt); bis er zurueckkehrt, kann der Slot eine
            // andere Klasse tragen.
            // `continue` statt `return`: Eternal-Task ohne vTaskDelete.
            if (!active_) continue;
            drv_->strobe(SRX);
            continue;
        }

        if (buf[1] == dudette::kTypeHello && len >= dudette::kHelloLen) {
            bool is_new = false;
            DudetteDevice* d = upsert(buf, len, rssi_dbm, lqi_crc & 0x7F, is_new);
            if (is_new) {
                ESP_LOGI(TAG, "neu: %06lX board=%04X lot=%s rssi=%d",
                         static_cast<unsigned long>(d->addr), d->board_id,
                         d->lot, rssi_dbm);
            }
            // Ein NEUES HELLO vom scharfgestellten Geraet beweist, dass das
            // vorige SELECT nicht angekommen ist: ein Geraet, das eine Sitzung
            // angenommen hat, beacont nicht mehr.  Also zurueck auf Armed und
            // noch einmal.  Ohne diesen Rueckfall war Armed -> Sent -> Done
            // eine Einbahnstrasse: ein einziger verlorener SELECT (oder eine
            // verlorene STATUS-Antwort) fror den Zustand fuer immer auf Sent
            // ein, jedes weitere HELLO lief in die Bedingung unten und wurde
            // verworfen.  Das Werkzeug meldete dann nach 180 s "kein HELLO
            // gehoert", waehrend das Ziel mit ~9 HELLOs/s sendete — am
            // 06.09.2026 am SCC so gemessen.  Ging die STATUS-Antwort verloren,
            // sitzt das Geraet noch in seiner Sitzung und schweigt; es beacont
            // nach deren Timeout wieder, und dann greift derselbe Pfad.
            if (arm_state_ == ArmState::Sent && d->addr == arm_addr_) {
                const uint32_t now_ms =
                    static_cast<uint32_t>(esp_timer_get_time() / 1000);
                if (now_ms - arm_sent_ms_ >= dudette::kArmRetryMs) {
                    ESP_LOGW(TAG, "SELECT an %06lX unbeantwortet — neuer Versuch",
                             static_cast<unsigned long>(d->addr));
                    arm_state_ = ArmState::Armed;
                }
            }
            // Dieselben zwei Tore wie in select(), nur hier auf das HELLO hin.
            if (arm_state_ == ArmState::Armed && d->addr == arm_addr_
                && d->collisions) {
                ESP_LOGE(TAG, "SELECT an %06lX verweigert: %u fremde Signaturzeilen",
                         static_cast<unsigned long>(d->addr),
                         static_cast<unsigned>(d->collisions));
                last_collisions_ = d->collisions;
                arm_status_ = dudette::kStAmbiguous;
                arm_state_  = ArmState::Done;
            }
            // Board-Erwartung: das HELLO, auf das hin wir gleich selektieren
            // wuerden, meldet den Typ der Platine.  Hier abzubrechen ist der
            // letzte Punkt, an dem es nichts gekostet hat — das naechste, was
            // der Stick sonst schickt, ist ERASE, und danach ist die
            // Anwendung des Geraets weg.
            if (arm_state_ == ArmState::Armed && d->addr == arm_addr_
                && expect_board_ && d->board_id != expect_board_) {
                ESP_LOGE(TAG, "SELECT an %06lX verweigert: Board %04X, erwartet %04X",
                         static_cast<unsigned long>(d->addr),
                         d->board_id, expect_board_);
                last_board_ = d->board_id;
                arm_status_ = dudette::kStBoardMismatch;
                arm_state_  = ArmState::Done;
            }
            if (arm_state_ == ArmState::Armed && d->addr == arm_addr_) {
                // Das Geraet lauscht jetzt ~90 ms.  send_frame() aus dieser
                // Task heraus ist unkritisch: es wartet selbst, bis der Rahmen
                // durch ist, und TXOFF_MODE = RX bringt den Chip zurueck.
                session_addr_ = arm_addr_;
                if (d->page_size) page_size_ = d->page_size;
                ++seq_;
                arm_seq_ = seq_;
                const uint8_t body[3] = {seq_, 10, 0x00};
                if (send_frame(dudette::kTypeSelect, body, 3) == ESP_OK) {
                    arm_state_   = ArmState::Sent;
                    // NACH dem Senden stempeln: send_frame() wartet, bis der
                    // Rahmen durch ist, und genau ab da laeuft die Antwortfrist.
                    arm_sent_ms_ =
                        static_cast<uint32_t>(esp_timer_get_time() / 1000);
                    ESP_LOGI(TAG, "SELECT auf HELLO von %06lX geschickt",
                             static_cast<unsigned long>(d->addr));
                }
            }
            if (rx_cb_) {
                DudetteRxEvent payload{d, is_new};
                RxEvent ev{
                    .proto        = ProtocolId::Dudette,
                    .slot_id      = slot_id_,
                    .rssi_dbm     = rssi_dbm,
                    .timestamp_us = static_cast<uint32_t>(esp_timer_get_time()),
                    .payload      = &payload,
                };
                rx_cb_(ev);
            }
        }

        // SWRS061I 19.4, S. 52: ein SRX waehrend einer laufenden Sendung
        // beendet sie.  Der Rueckruf kann blockieren (TCP bis 0,5 s, bis
        // 0.1.893 unbegrenzt); bis er zurueckkehrt, kann der Slot eine
        // andere Klasse tragen.
        // `continue` statt `return`: Eternal-Task ohne vTaskDelete.
        if (!active_) continue;
        drv_->strobe(SRX);
    }
}

esp_err_t DudetteRF::start(uint8_t slot_id, CC1101Driver& drv) {
    slot_id_ = slot_id;
    drv_     = &drv;

    for (size_t i = 0; DUDETTE_CFG[i] <= 0x40; i += 2) {
        if (auto e = drv.write_reg(DUDETTE_CFG[i], DUDETTE_CFG[i + 1]); e != ESP_OK)
            return e;
    }
    if (auto e = drv.write_reg(0x3E, profile().pa_table[0]); e != ESP_OK) return e;

    if (!lock_)      lock_      = xSemaphoreCreateMutex();
    if (!reply_sem_) reply_sem_ = xSemaphoreCreateBinary();
    if (!lock_ || !reply_sem_) return ESP_ERR_NO_MEM;

    if (!task_running_) {
        TaskHandle_t h = nullptr;
        if (xTaskCreate(rx_task_trampoline, "dudette-rx", 3072, this, 5, &h) != pdPASS)
            return ESP_ERR_NO_MEM;
        task_handle_ = h;
        // Kurz warten, bis die Task laeuft — sonst kann die erste ISR auf ein
        // noch nicht gesetztes Handle treffen.
        for (int i = 0; i < 50 && !task_running_; ++i) vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (auto e = drv.attach_gdo2_irq(&DudetteRF::gdo_isr, this, true); e != ESP_OK)
        return e;

    drv.strobe(SIDLE);
    drv.flush_rx();
    drv.strobe(SRX);
    active_ = true;
    ESP_LOGI(TAG, "RX auf 868.4998 MHz / 100 kBaud, Sync 0D3B");
    return ESP_OK;
}

esp_err_t DudetteRF::stop() {
    active_ = false;
    if (drv_) {
        drv_->set_gdo2_irq_enabled(false);
        drv_->strobe(SIDLE);
    }
    return ESP_OK;
}

esp_err_t DudetteRF::tx(const void*, size_t) {
    // M2 hoert nur zu.  Der Sendepfad (SELECT/DATA/COMMIT/GO) kommt mit M3;
    // bis dahin soll ein versehentlicher Sendeversuch klar scheitern statt
    // stumm nichts zu tun.
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t DudetteRF::send_frame(uint8_t type, const uint8_t* body, uint8_t blen) {
    if (!active_ || blen > dudette::kMaxFrame - dudette::kHeaderLen)
        return ESP_ERR_INVALID_ARG;

    uint8_t f[dudette::kMaxFrame];
    f[0] = dudette::kMagicVer;
    f[1] = type;
    f[2] = static_cast<uint8_t>(session_addr_ >> 16);
    f[3] = static_cast<uint8_t>(session_addr_ >> 8);
    f[4] = static_cast<uint8_t>(session_addr_);
    f[5] = seq_ & 0x7F;
    if (blen) std::memcpy(&f[6], body, blen);
    const uint8_t len = static_cast<uint8_t>(dudette::kHeaderLen + blen);

    // Waehrend des Sendens darf die RX-Task nicht dazwischen SRX strobe'n.
    drv_->set_gdo2_irq_enabled(false);

    // SIDLE wirkt nicht sofort, und im RX-Zustand ist das Folgende zweifach
    // wirkungslos: SFTX ist nur in IDLE bzw. TXFIFO_UNDERFLOW zulaessig
    // (SWRS061I S. 33, dieselbe Regel, die flush_rx() im Treiber begruendet) —
    // aus RX heraus verpufft es und der FIFO behaelt seinen Inhalt.  Und ein
    // STX aus RX heraus unterliegt dem CCA-Gate: SWRS061I §17.5 (S. 48) "the TX
    // ... state is only entered if the clear channel requirements are
    // fulfilled.  Otherwise, the chip will remain in RX", mit MCSM1 = 0x3F sind
    // das RSSI unter der Schwelle UND kein laufender Empfang.  Unmittelbar nach
    // dem eigenen Rahmen ist die AGC noch nicht eingeschwungen — deshalb kam
    // ein Einzelrahmen (SELECT, mit 40 Anlaeufen) durch und eine dichte
    // DATA-Folge nicht.  Aus IDLE heraus gibt es kein CCA-Gate.
    // Die AVR-Seite macht das seit dem ersten Bringup so (cc_wait_idle); hier
    // war die Lehre nicht nachgezogen.
    drv_->strobe(SIDLE);
    for (int i = 0; i < 100; ++i) {              // max ~10 ms
        if (drv_->marc_state() == MARC_IDLE) break;
        ets_delay_us(100);
    }
    drv_->strobe(SFTX);

    const uint8_t hdr[1] = {len};
    esp_err_t e = drv_->write_burst(CC_TXFIFO, {hdr, 1});
    if (e == ESP_OK) e = drv_->write_burst(CC_TXFIFO, {f, len});
    if (e != ESP_OK) { drv_->set_gdo2_irq_enabled(true); return e; }

    drv_->strobe(STX);

    // Nach dem STX wird der Chip NICHT mehr angefasst: MCSM1.TXOFF_MODE = RX
    // bringt ihn selbst zurueck in den Empfang, und zwar sofort am Rahmenende.
    // Ein SIDLE+SRX hier kostet mehrere Millisekunden Umschaltzeit — genau die,
    // in der die Antwort des Geraets eintrifft; SELECT lief deshalb in einen
    // Timeout.  Warten, bis der Rahmen durch ist: (4 Praeambel + 4 Sync + 1
    // Laenge + len + 2 CRC) * 8 / 100 kBaud.  Beim laengsten Rahmen (58 Byte)
    // sind das 69 Byte = 5,5 ms, dazu die PLL-Kalibrierung vor dem Senden.
    // Mit 6 ms wurden aufeinanderfolgende DATA-Rahmen vom naechsten SIDLE
    // abgeschnitten — beim Ziel kam kein einziges Fragment an, nur das COMMIT.
    vTaskDelay(pdMS_TO_TICKS(10));
    drv_->set_gdo2_irq_enabled(true);

    // Der GDO2-Interrupt ist FLANKENgesteuert (GPIO_INTR_POSEDGE) und war die
    // letzten 10 ms abgeschaltet.  Antwortet die Gegenstelle in diesem Fenster
    // — und genau das tut sie, ein Bootloader antwortet in wenigen Millisekunden
    // — dann hebt ihr Rahmen GDO2 an, waehrend niemand hinsieht: die Flanke ist
    // verloren, der Pegel bleibt oben, und es kommt nie wieder eine.  Das Paket
    // liegt vollstaendig im FIFO und verhungert, bis der naechste Rahmen den
    // Ueberlauf ausloest.  Deshalb hier nachsehen und die RX-Task selbst
    // anstossen.  (Erklaert das bisherige Bild vollstaendig: der COMMIT_ACK kam
    // nur durch, wenn das Geraet die Seite wirklich geschrieben hat — die rund
    // 5 ms Flash-Write schoben die Antwort aus dem Fenster heraus.  Bei
    // falscher CRC gibt es keinen Write, die Antwort kommt frueher und war
    // jedes Mal weg.)
    if (drv_->gdo2_level() > 0 && task_handle_)
        xTaskNotifyGive(static_cast<TaskHandle_t>(task_handle_));
    // Sendezeit in 10-ms-Einheiten: (Praeambel 4 + Sync 4 + Laenge 1 + len +
    // CRC 2) Byte bei 99,976 kBaud.  Ein Rahmen liegt bei ~5 ms, also 0 oder 1.
    const uint16_t air10 = static_cast<uint16_t>(((len + 11u) * 8u * 100u)
                                                 / 99976u / 10u);
    RfStats::instance().on_tx(ProtocolId::Dudette, len, air10);
    return ESP_OK;
}

esp_err_t DudetteRF::request(uint8_t type, const uint8_t* body, uint8_t blen,
                             uint8_t expect, uint8_t* reply, uint8_t& rlen,
                             uint32_t timeout_ms)
{
    if (!lock_ || !reply_sem_) return ESP_ERR_INVALID_STATE;
    auto lk = static_cast<SemaphoreHandle_t>(lock_);
    if (xSemaphoreTake(lk, pdMS_TO_TICKS(2000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    // Altlasten aus einer abgelaufenen Anfrage wegraeumen.
    xSemaphoreTake(static_cast<SemaphoreHandle_t>(reply_sem_), 0);
    reply_len_  = 0;
    await_type_ = expect;
    ++seq_;

    esp_err_t e = send_frame(type, body, blen);
    if (e == ESP_OK) {
        if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(reply_sem_),
                           pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
            rlen = reply_len_;
            std::memcpy(reply, reply_, rlen);
        } else {
            e = ESP_ERR_TIMEOUT;
        }
    }
    await_type_ = 0;
    xSemaphoreGive(lk);
    return e;
}

void DudetteRF::arm(uint32_t addr) {
    arm_addr_    = addr;
    arm_status_  = 0xFF;
    arm_sent_ms_ = 0;
    arm_state_   = ArmState::Armed;
}

void DudetteRF::disarm() {
    arm_state_ = ArmState::Idle;
    arm_addr_  = 0;
}

esp_err_t DudetteRF::erase(uint16_t from_page, uint8_t& status) {
    const uint8_t body[2] = {static_cast<uint8_t>(from_page),
                             static_cast<uint8_t>(from_page >> 8)};
    uint8_t rep[dudette::kMaxFrame]; uint8_t rlen = 0;
    // 496 Seiten zu ~4,5 ms beim 1284P: der Bootloader antwortet nach gut 2 s.
    //
    // Wiederholen ist hier gefahrlos: erase_app() loescht einfach noch einmal,
    // der Zustand danach ist derselbe.  Frueher stand hier nur ein Versuch, und
    // ein verlorenes ERASE oder dessen STATUS-Antwort hat den ganzen Transfer
    // getoetet (am 06.09.2026 dreimal reproduziert).  Waehrend das Geraet die
    // gut 2 s loescht, hoert es nicht zu — deshalb erst nach dem vollen Timeout
    // nachfassen, nicht frueher.
    // GENAU ZWEI Versuche, und die Zahl ist nicht frei waehlbar — sie ist von
    // zwei Fristen eingeklemmt, die beide bei 10 s liegen:
    //   * Die Sitzung auf dem Geraet laeuft 10 s ab dem letzten empfangenen
    //     Rahmen (SELECT-Body traegt die 10, siehe select()).  Versuch 2 geht
    //     bei t=6 s raus, kommt also noch in der Sitzung an; ein dritter waere
    //     bei t=12 s dran — da hat das Geraet laengst per WDT neu gestartet und
    //     beacont, der Versuch liefe ins Leere.
    //   * dudette_flash.py wartet auf `De` nur 10 s.  Versuch 2 antwortet nach
    //     6 s Timeout + gut 2 s Loeschzeit, also bei ~8,2 s — gerade noch
    //     drin.  Ein dritter Versuch wuerde das Werkzeug in sein eigenes
    //     Timeout laufen lassen, waehrend der Stick noch arbeitet.
    // Wer mehr Versuche will, muss BEIDE Fristen mit anheben.
    esp_err_t e = ESP_ERR_TIMEOUT;
    for (uint8_t attempt = 1; attempt <= 2; ++attempt) {
        e = request(dudette::kTypeErase, body, 2, dudette::kTypeStatus,
                    rep, rlen, 6000);
        if (e != ESP_ERR_TIMEOUT) break;
        ESP_LOGW(TAG, "ERASE ab Seite %u ohne Antwort (Versuch %u von 2)",
                 static_cast<unsigned>(from_page), static_cast<unsigned>(attempt));
    }
    if (e != ESP_OK) return e;
    status = (rlen > dudette::kHeaderLen) ? rep[dudette::kHeaderLen] : 0xFF;
    return ESP_OK;
}

esp_err_t DudetteRF::select(uint32_t addr, uint8_t& status) {
    // Zwei Tore vor jeder Abkuerzung: wer ist das da, und ist es das richtige
    // Geraet?  Beide muessen VOR dem SELECT liegen — danach ist das naechste,
    // was der Stick schickt, ERASE.
    const DudetteDevice* known = nullptr;
    for (size_t i = 0; i < dev_count_; ++i)
        if (devs_[i].addr == addr) { known = &devs_[i]; break; }
    last_collisions_ = known ? known->collisions : 0;

    // Tor 1: unter dieser Adresse haben sich mehrere Bausteine gemeldet.  Die
    // Adresse ist ein auf 24 bit gekuerzter Hash der Signaturzeile — unter
    // 1000 Geraeten kollidieren rund 3 %.  e837b95 hat die Marke gesetzt und
    // bewusst nur angezeigt; ausgewertet hat sie niemand, also ging der
    // Transfer bisher trotzdem los und traf einen der beiden.  Welchen, ist
    // nicht bestimmbar.  Erholung: `Dc` (Liste leeren), das stoerende Geraet
    // ausser Reichweite, neu lauschen.
    if (known && known->collisions) {
        ESP_LOGE(TAG, "SELECT an %06lX verweigert: %u fremde Signaturzeilen",
                 static_cast<unsigned long>(addr),
                 static_cast<unsigned>(known->collisions));
        status = dudette::kStAmbiguous;
        return ESP_ERR_INVALID_STATE;
    }

    // Tor 2: Board-Erwartung.  Geprueft wird gegen das, was das Geraet selbst
    // im HELLO gemeldet hat.  Steht es nicht in der Liste, ist die Pruefung
    // nicht durchfuehrbar — und dann ist Ablehnen die richtige Antwort, nicht
    // Durchwinken: "ungeprueft" darf nicht wie "geprueft und in Ordnung"
    // aussehen.
    if (expect_board_) {
        last_board_ = known ? known->board_id : 0;
        if (!known || known->board_id != expect_board_) {
            ESP_LOGE(TAG, "SELECT an %06lX verweigert: Board %04X, erwartet %04X",
                     static_cast<unsigned long>(addr), last_board_, expect_board_);
            status = dudette::kStBoardMismatch;
            return ESP_ERR_INVALID_VERSION;
        }
    }
    if (arm_state_ == ArmState::Done && arm_addr_ == addr && arm_status_ == 0) {
        // Sitzung steht schon — das SELECT ging auf das HELLO hinaus.
        session_addr_ = addr;
        status = 0;
        arm_state_ = ArmState::Idle;
        return ESP_OK;
    }
    session_addr_ = addr;
    // BODY: Sitzungsnummer, Timeout in Sekunden, Flags
    const uint8_t body[3] = {seq_, 10, 0x00};
    uint8_t rep[dudette::kMaxFrame]; uint8_t rlen = 0;
    // Das Geraet lauscht nur im Beacon-Fenster; mehrere Anlaeufe decken den
    // Fall ab, dass gerade gesendet wird.
    esp_err_t e = ESP_ERR_TIMEOUT;
    for (int i = 0; i < 40 && e != ESP_OK; ++i)
        e = request(dudette::kTypeSelect, body, 3, dudette::kTypeStatus,
                    rep, rlen, 150);
    if (e != ESP_OK) return e;
    status = (rlen > dudette::kHeaderLen) ? rep[dudette::kHeaderLen] : 0xFF;

    for (size_t i = 0; i < dev_count_; ++i)
        if (devs_[i].addr == addr && devs_[i].page_size)
            page_size_ = devs_[i].page_size;
    return ESP_OK;
}

void DudetteRF::page_begin(uint16_t page) {
    page_no_  = page;
    page_map_ = 0;
    std::memset(page_, 0xFF, sizeof(page_));
}

esp_err_t DudetteRF::page_fill(uint8_t frag, const uint8_t* data, uint8_t len) {
    if (frag >= dudette::kMaxFrags || len > dudette::kFragData)
        return ESP_ERR_INVALID_ARG;
    const uint16_t off = static_cast<uint16_t>(frag) * dudette::kFragData;
    if (off >= page_size_) return ESP_ERR_INVALID_SIZE;
    // Ein Fragment ist entweder voll oder das letzte der Seite — dazwischen
    // gibt es nichts.  Ohne diese Pruefung gilt ein 20-Byte-Fragment als
    // vollstaendig: page_map_ bekommt sein Bit, send_page() sieht ein volles
    // Bitmap und schickt eine Seite, deren Rest 0xFF geblieben ist.  Das
    // Geraet rechnet ueber genau das dieselbe CRC und quittiert ST_OK.
    // Dieselbe Fehlerklasse wie der ungefuellte Puffer in send_page(): eine
    // Quittung, die ueber die falsche Nutzlast rechnet.
    const uint16_t rest = static_cast<uint16_t>(page_size_ - off);
    const uint8_t  want = (rest > dudette::kFragData)
                              ? dudette::kFragData : static_cast<uint8_t>(rest);
    if (len != want) return ESP_ERR_INVALID_SIZE;
    std::memcpy(&page_[off], data, len);
    page_map_ |= static_cast<uint8_t>(1u << frag);
    return ESP_OK;
}

esp_err_t DudetteRF::send_page(uint8_t& status, uint8_t& bitmap, uint8_t& rounds,
                               uint8_t* dbg, uint16_t* want_out) {
    if (page_no_ == 0xFFFF) return ESP_ERR_INVALID_STATE;
    const uint8_t nfrag = static_cast<uint8_t>((page_size_ + dudette::kFragData - 1)
                                              / dudette::kFragData);
    // Ein ungefuellter Puffer ist 0xFF und damit gueltige Nutzlast: das Geraet
    // rechnet dieselbe CRC, quittiert ST_OK und schreibt eine leere Seite.  Der
    // Transfer meldet dann Erfolg und hat nichts geflasht — genau so hat ein
    // verdecktes `Dd` einmal 20 Seiten still verschluckt.  Wer eine Seite
    // wirklich leeren will, loescht sie (De); hierher gehoert sie nicht.
    if (page_map_ != ((nfrag >= 8) ? 0xFF : ((1u << nfrag) - 1))) {
        bitmap = page_map_; rounds = 0; status = 0xFF;
        return ESP_ERR_INVALID_STATE;
    }
    // Ab hier ist der Puffer VERBRAUCHT, wie es der Bootloader mit seinem
    // `cur_page = 0xFFFF` nach dem Schreiben auch haelt.  Ohne das blieben
    // page_no_ und page_map_ stehen, und weil `Dp` bei Erfolg still ist,
    // faellt ein verlorenes oder verstuemmeltes `Dp<N>` nirgends auf: die
    // folgenden `Dd` fuellen die Fragmente der NEUEN Seite in den ALTEN,
    // schon vollstaendigen Puffer, und `Dm` schreibt die VORSEITE ein zweites
    // Mal — mit den Daten von N.  Seite N bleibt 0xFF, und auffallen wuerde
    // das erst bei der Gesamt-CRC am Ende des Transfers.
    struct Spent {
        DudetteRF* s;
        ~Spent() { s->page_no_ = 0xFFFF; s->page_map_ = 0; }
    } spent{this};

    const uint16_t want_crc = crc16_ccitt(page_, page_size_);
    if (want_out) *want_out = want_crc;
    const uint8_t all  = static_cast<uint8_t>((nfrag >= 8) ? 0xFF
                                                           : ((1u << nfrag) - 1));
    uint8_t todo         = all;
    bool    got_reply    = false;
    bool    last_timeout = false;

    for (rounds = 1; rounds <= 4; ++rounds) {
        for (uint8_t i = 0; i < nfrag; ++i) {
            if (!(todo & (1u << i))) continue;
            uint8_t body[3 + dudette::kFragData];
            const uint16_t off = static_cast<uint16_t>(i) * dudette::kFragData;
            // In 16 bit rechnen und ERST DANN begrenzen: beim ersten Fragment
            // ist der Rest die volle Seite, und 256 wird als uint8_t zu 0 —
            // der Vergleich gegen kFragData greift dann nicht mehr, und das
            // Fragment geht als leerer Rahmen auf die Reise.  Das Ziel setzt
            // brav sein Bitmap-Bit und schreibt nichts: Bitmap voll, Seite
            // vorne leer, CRC falsch.  Bei 128-Byte-Seiten (ATmega328P) waere
            // das nie aufgefallen.
            const uint16_t rest = static_cast<uint16_t>(page_size_ - off);
            const uint8_t n = (rest > dudette::kFragData)
                                  ? dudette::kFragData
                                  : static_cast<uint8_t>(rest);
            body[0] = static_cast<uint8_t>(page_no_);
            body[1] = static_cast<uint8_t>(page_no_ >> 8);
            body[2] = i;
            std::memcpy(&body[3], &page_[off], n);
            ++seq_;
            if (auto e = send_frame(dudette::kTypeData, body,
                                    static_cast<uint8_t>(3 + n)); e != ESP_OK)
                return e;
        }

        const uint8_t cbody[4] = {
            static_cast<uint8_t>(page_no_), static_cast<uint8_t>(page_no_ >> 8),
            static_cast<uint8_t>(want_crc), static_cast<uint8_t>(want_crc >> 8)};
        uint8_t rep[dudette::kMaxFrame]; uint8_t rlen = 0;
        // Das Geraet schreibt die Seite, BEVOR es den COMMIT quittiert.  Im
        // Flash sind das ~5 ms; eine EEPROM-Seite (Bit 15 der Seitennummer,
        // Bootloader v2.3+) braucht bis 3,4 ms je geaendertem Byte — eine
        // komplett neue 256er-Seite gut 0,9 s.  Mit 500 ms lief der COMMIT
        // dann in den Timeout, die Runde wurde wiederholt, und erst die
        // vierte kam durch (SCC, 12.09.2026).  Deshalb je Seitenart warten.
        const uint32_t commit_to = (page_no_ & dudette::kPageEeprom) ? 1500 : 500;
        auto e = request(dudette::kTypeCommit, cbody, 4, dudette::kTypeCommitAck,
                         rep, rlen, commit_to);
        if (e == ESP_ERR_TIMEOUT) {
            // Frueher stand hier `return e` — ein einziger verlorener COMMIT
            // oder COMMIT_ACK hat damit den ganzen Transfer getoetet (am
            // 06.09.2026 bei Seite 3 von 101 gemessen).
            //
            // Die Anfrage BLIND zu wiederholen waere falsch: der Bootloader
            // setzt nach dem Schreiben `cur_page = 0xFFFF` ("Puffer ist
            // verbraucht"), ein zweites COMMIT auf dieselbe Seite liefert also
            // ST_BAD_PAGE — aus einem verlorenen ACK wuerde ein gemeldeter
            // Fehler.  Stattdessen die ganze RUNDE wiederholen: die Fragmente
            // setzen `cur_page` neu und fuellen den Puffer, und write_page()
            // loescht die Seite ohnehin vor dem Schreiben, ein zweites
            // Schreiben derselben Seite ist also unschaedlich.
            ESP_LOGW(TAG, "COMMIT Seite %u ohne Antwort — Runde %u wiederholen",
                     static_cast<unsigned>(page_no_), static_cast<unsigned>(rounds));
            todo         = all;
            last_timeout = true;
            continue;
        }
        if (e != ESP_OK) return e;
        if (rlen < dudette::kHeaderLen + 4) return ESP_ERR_INVALID_RESPONSE;
        got_reply    = true;
        last_timeout = false;

        bitmap = rep[dudette::kHeaderLen + 2];
        status = rep[dudette::kHeaderLen + 3];
        if (dbg) {
            const uint8_t have = static_cast<uint8_t>(rlen - dudette::kHeaderLen - 4);
            std::memcpy(dbg, &rep[dudette::kHeaderLen + 4],
                        (have > 24) ? 24 : have);
        }
        if (status == 0) return ESP_OK;

        // Nur die Fragmente nachschicken, die das Geraet nicht hat.  Meldet es
        // alle als angekommen und die CRC stimmt trotzdem nicht, hilft
        // Nachschicken nicht — dann alles noch einmal.
        todo = static_cast<uint8_t>(all & ~bitmap);
        if (todo == 0) todo = all;
    }
    // `status` ist nur dann belastbar, wenn die LETZTE Runde tatsaechlich
    // beantwortet wurde.  Sonst stammt er aus einer aelteren Runde und wuerde
    // einen Transportfehler als Geraetefehler ausgeben — z.B. Runde 1 meldet
    // ST_BAD_CRC, Runden 2..4 laufen in den Timeout: ohne diese Pruefung kaeme
    // ESP_OK mit status=ST_BAD_CRC heraus, und der Bediener suchte den Fehler
    // im Geraet statt auf der Strecke.
    if (!got_reply || last_timeout) return ESP_ERR_TIMEOUT;
    return ESP_OK;   // status traegt den letzten Fehlercode
}

esp_err_t DudetteRF::verify(uint32_t from, uint32_t len, uint16_t& crc,
                            uint8_t& status, bool ee)
{
    const uint8_t body[8] = {
        static_cast<uint8_t>(from), static_cast<uint8_t>(from >> 8),
        static_cast<uint8_t>(from >> 16), static_cast<uint8_t>(from >> 24),
        static_cast<uint8_t>(len), static_cast<uint8_t>(len >> 8),
        static_cast<uint8_t>(len >> 16), static_cast<uint8_t>(len >> 24)};
    uint8_t rep[dudette::kMaxFrame]; uint8_t rlen = 0;
    // Der Bootloader liest den Flash byteweise; grosse Bereiche dauern.
    //
    // Wiederholen ist gefahrlos: VERIFY ist eine reine Lesefunktion, es
    // veraendert am Geraet nichts.  Ohne Wiederholung war das die unangenehmste
    // Fehlerart ueberhaupt — ein vollstaendig geglueckter Transfer endet an
    // einem einzigen verlorenen Rahmen als "VERIFY weicht ab", das GO wird nie
    // gesendet, und der Bediener sucht den Fehler im Image statt auf der
    // Strecke.
    //
    // ZWEI Versuche, aus demselben Grund wie bei erase(): dudette_flash.py
    // wartet auf `Dv` nur 8 s.  Versuch 2 antwortet nach 4 s Timeout plus der
    // CRC-Zeit (gut 150 ms ueber den App-Bereich) bei ~4,2 s — sicher drin.
    // Ein dritter waere erst bei ~8,2 s fertig und liefe dem Werkzeug in sein
    // eigenes Timeout.  Mehr Versuche verlangen ein groesseres Fenster dort.
    esp_err_t e = ESP_ERR_TIMEOUT;
    for (uint8_t attempt = 1; attempt <= 2; ++attempt) {
        e = request(ee ? dudette::kTypeEeVerify : dudette::kTypeVerify, body, 8,
                    dudette::kTypeVerifyAck, rep, rlen, 4000);
        if (e != ESP_ERR_TIMEOUT) break;
        ESP_LOGW(TAG, "VERIFY ohne Antwort (Versuch %u von 2)",
                 static_cast<unsigned>(attempt));
    }
    if (e != ESP_OK) return e;
    if (rlen < dudette::kHeaderLen + 3) return ESP_ERR_INVALID_RESPONSE;
    crc    = static_cast<uint16_t>(rep[dudette::kHeaderLen]
             | (rep[dudette::kHeaderLen + 1] << 8));
    status = rep[dudette::kHeaderLen + 2];
    return ESP_OK;
}

esp_err_t DudetteRF::go(uint8_t& status) {
    uint8_t rep[dudette::kMaxFrame]; uint8_t rlen = 0;
    // Zwei Verlustfaelle, und die Wiederholung hilft nur beim ersten:
    //   * GO verloren  -> das Geraet ist noch in der Sitzung und springt beim
    //                     naechsten Versuch.  Genau dafuer ist sie da.
    //   * ACK verloren -> der Bootloader quittiert VOR dem Sprung (`case T_GO`
    //                     in dudette.c: reply_status, dann start_app), das
    //                     Geraet ist also schon weg.  Der Versuch laeuft ins
    //                     Leere und die Funktion meldet Timeout — unschoen,
    //                     aber nicht falsch, und schaedlich ist er nicht: ein
    //                     GO ausserhalb einer Sitzung wird im Beacon-Zustand
    //                     gar nicht behandelt.
    // Ohne Wiederholung heilte sich ein verlorenes GO erst ueber den
    // Sitzungsverfall und das 3-s-Fenster danach, also nach gut 13 s.
    // 3 x 500 ms sind hier gefahrlos: das Werkzeug wartet auf `Dg` 8 s
    // (Default in Stick.cmd), die Sitzung laeuft 10 s.
    esp_err_t e = ESP_ERR_TIMEOUT;
    for (uint8_t attempt = 1; attempt <= 3; ++attempt) {
        e = request(dudette::kTypeGo, nullptr, 0, dudette::kTypeStatus,
                    rep, rlen, 500);
        if (e != ESP_ERR_TIMEOUT) break;
        ESP_LOGW(TAG, "GO ohne Antwort (Versuch %u von 3)",
                 static_cast<unsigned>(attempt));
    }
    if (e != ESP_OK) return e;
    status = (rlen > dudette::kHeaderLen) ? rep[dudette::kHeaderLen] : 0xFF;
    return ESP_OK;
}

void DudetteRF::debug_channel(bool on, uint32_t filter) {
    // Ein Radio, ein Kanal: solange der Debug-Kanal an ist, hoert der Stick
    // die Dudette-Beacons NICHT.  Das ist Absicht und kein Mangel — 869,525
    // und 868,4998 sind 1 MHz auseinander.
    if (!active_) return;
    dbg_filter_ = filter;
    dbg_on_     = on;
    if (on) {
        drv_->write_reg(0x04, dudette::kDbgSync1);
        drv_->write_reg(0x05, dudette::kDbgSync0);
        drv_->write_reg(0x0D, dudette::kDbgFreq2);
        drv_->write_reg(0x0E, dudette::kDbgFreq1);
        drv_->write_reg(0x0F, dudette::kDbgFreq0);
    } else {
        // Zurueck auf die Werte des Profils.  Sie stehen in der Tabelle, die
        // start() geschrieben hat — hier die beiden Bloecke von Hand, damit
        // kein kompletter Neustart des Protokolls noetig ist.
        drv_->write_reg(0x04, 0x0D);
        drv_->write_reg(0x05, 0x3B);
        drv_->write_reg(0x0D, 0x21);
        drv_->write_reg(0x0E, 0x67);
        drv_->write_reg(0x0F, 0x62);
    }
    drv_->strobe(SIDLE);
    drv_->strobe(SFRX);
    drv_->strobe(SRX);
}

esp_err_t DudetteRF::debug_cmd(uint32_t addr, uint8_t cmd, uint8_t arg) {
    if (!dbg_on_) return ESP_ERR_INVALID_STATE;
    // Kein request(): die Anwendung pollt in ihrem eigenen Takt und schickt
    // keine formgebundene Antwort — sie meldet sich hoechstens mit einer
    // Textzeile zurueck.  Also einfach senden und dem Bediener die Antwort
    // in der Konsole ueberlassen.
    // send_frame() nimmt die Zieladresse aus session_addr_.  Auf dem
    // Debug-Kanal kann keine Sitzung laufen (andere Frequenz, anderes
    // Syncwort), also darf sie hier gesetzt werden.
    session_addr_ = addr;
    const uint8_t body[2] = {cmd, arg};
    ++seq_;
    return send_frame(dudette::kTypeCmd, body, 2);
}

void DudetteRF::clear_devices() {
    devs_.fill(DudetteDevice{});
    dev_count_ = 0;
}

} // namespace culfw32
