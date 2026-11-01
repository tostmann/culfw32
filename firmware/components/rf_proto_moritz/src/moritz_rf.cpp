// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
// Portions derived from a-culfw (https://github.com/tostmann/a-culfw),
// GNU GPL version 2 or (at your option) any later version.

#include "moritz_rf.hpp"
#include "moritz_cfg.hpp"
#include "rf_stats.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG = "moritz";

constexpr uint8_t CC_TXFIFO  = 0x3F;          // Adresse wie RX-FIFO, Schreibzugriff waehlt TX
constexpr uint8_t REG_MARC   = 0x35;

constexpr uint8_t SRES  = 0x30;
constexpr uint8_t SRX   = 0x34;
constexpr uint8_t STX   = 0x35;
constexpr uint8_t SIDLE = 0x36;
constexpr uint8_t SFRX  = 0x3A;
constexpr uint8_t SFTX  = 0x3B;

constexpr uint8_t MARC_IDLE              = 0x01;
constexpr uint8_t MARC_RX                = 0x0D;
constexpr uint8_t MARC_TX                = 0x13;
constexpr uint8_t MARC_RXFIFO_OVERFLOW   = 0x11;
constexpr uint8_t MARC_TXFIFO_UNDERFLOW  = 0x16;

uint8_t marc_state(CC1101Driver& d) {
    uint8_t v = 0xFF;
    d.read_status(REG_MARC, v);
    return v & 0x1F;
}

// Benachrichtigungsbits des RX-Tasks.
constexpr uint32_t kNotifyPacket = 1u << 0;   // GDO2: Paket mit CRC_OK fertig
constexpr uint32_t kNotifySync   = 1u << 1;   // GDO0: Sync erkannt, Paket laeuft
// Ein MAX!-Rahmen dauert hoechstens ~30 Byte x 0,8 ms. Ein aelterer gemerkter
// FREQEST-Wert gehoert nicht mehr zum gerade gelesenen Rahmen.
constexpr int64_t kFreqEstMaxAgeUs = 100'000;
}

const CC1101Profile& MoritzRF::profile() const {
    static const CC1101Profile p = {
        .freq_hz        = 868'299'820,
        .datarate_bps   = 9'993,
        .deviation_hz   = 19'042,
        .mod            = CC1101Profile::Modulation::FSK_2,
        .sync_word      = {0xC6, 0x26, 0, 0},
        .sync_len       = 16,
        .pa_table       = {0xC3, 0, 0, 0, 0, 0, 0, 0},
        .raw_regs       = nullptr,
    };
    return p;
}

void IRAM_ATTR MoritzRF::gdo2_isr(void* arg) {
    auto* self = static_cast<MoritzRF*>(arg);
    BaseType_t hpw = pdFALSE;
    xTaskNotifyFromISR(static_cast<TaskHandle_t>(self->task_handle_), kNotifyPacket,
                       eSetBits, &hpw);
    portYIELD_FROM_ISR(hpw);
}

void IRAM_ATTR MoritzRF::gdo0_isr(void* arg) {
    auto* self = static_cast<MoritzRF*>(arg);
    BaseType_t hpw = pdFALSE;
    xTaskNotifyFromISR(static_cast<TaskHandle_t>(self->task_handle_), kNotifySync,
                       eSetBits, &hpw);
    portYIELD_FROM_ISR(hpw);
}

void MoritzRF::latch_freqest_() {
    // GDO0 = IOCFG0 0x46: "Asserts when sync word has been sent / received,
    // and de-asserts at the end of the packet" (SWRS061I Tab. 41, S. 62),
    // invertiert -- also low vom Sync bis zum Paketende. Nur so lange gehoert
    // FREQEST zu diesem Paket: danach bleibt der Chip in RX (MCSM1 RXOFF = RX),
    // und der Kompensator laeuft weiter (SWRS061I §14.1, ohne FOC_BS_CS_GATE).
    // Gemessen 19.10.2026 mit dem Lesen nach dem FIFO (bis 0.1.908): Median
    // 0 bzw. -1 bei -8..+8 LSB Verstimmung statt der Steigung -1 -- der Wert
    // war wertlos, `?rf.freqcal=apply` damit nach `Zr` auch.
    if (drv_->gdo0_level() != 0) return;          // Paket schon zu Ende
    if (marc_state(*drv_) != MARC_RX) return;     // eigene Sendung (Sync gesendet)
    int8_t v = 0;
    if (!drv_->read_freqest(v)) return;
    pending_fe_       = v;
    pending_fe_us_    = esp_timer_get_time();
    pending_fe_valid_ = true;
}

void MoritzRF::rx_task_trampoline(void* arg) {
    static_cast<MoritzRF*>(arg)->rx_task_loop();
    vTaskDelete(nullptr);
}

void MoritzRF::drain_fifo() {
    if (!enabled_) return;
    // Alle fertigen Rahmen lesen, nicht nur den gemeldeten: MCSM1 = 0x3F laesst
    // den Chip nach jedem Paket in RX, mehrere Rahmen koennen im FIFO stehen,
    // und GDO2 meldet nur einen davon (siehe CC1101Driver::read_rx_packet).
    for (int i = 0; i < CC1101Driver::kRxFramesPerDrain; ++i) {
        // Ein blockierter Callback kann zurueckkehren, waehrend ein anderer Task
        // sendet (tx() ohne Sperre gegen diesen Task); Lesen, Flushen oder SRX
        // wuerden die Sendung abbrechen. Nur in RX/IDLE/Overflow weitermachen.
        if (!drv_->rx_state_safe()) break;
        MoritzFrame frame{};
        uint8_t rssi = 0, lqi = 0;
        const auto r = drv_->read_rx_packet({frame.data, MoritzFrame::kMaxPayload},
                                            rssi, lqi, i == 0);
        if (r == CC1101Driver::RxPacket::Overflow) { drv_->flush_rx(); break; }
        if (r != CC1101Driver::RxPacket::Frame)    break;   // Empty/Busy/Error; Resync ist geflusht

        const uint8_t len = frame.data[0];
        frame.len      = len;
        frame.rssi_raw = rssi;
        frame.lqi_raw  = lqi;
        drv_->note_rx_frame();
        // FREQEST kommt aus latch_freqest_(): jetzt, nach dem Paket, gehoert der
        // Registerwert nicht mehr zu diesem Rahmen (s. dort). Ein Wert je Sync.
        if (pending_fe_valid_ && esp_timer_get_time() - pending_fe_us_ < kFreqEstMaxAgeUs) {
            drv_->push_freqest(pending_fe_);
        }
        pending_fe_valid_ = false;

        // RSSI dBm-Konversion (CC1101 Datenblatt §17.3, byte-genau a-culfw cc1100.c).
        int16_t rssi_dbm = cc1101_rssi_dbm(rssi);
        if (rssi_dbm < -128) rssi_dbm = -128;
        if (rssi_dbm > 127)  rssi_dbm =  127;
        // CRC-Fehler kommen hier nicht an: CRC_AUTOFLUSH verwirft solche
        // Pakete im Chip, einen Rahmen ohne CRC_OK verwirft read_rx_packet
        // (zaehlt als rx_resync).
        RfStats::instance().on_rx(ProtocolId::Moritz, len, static_cast<int8_t>(rssi_dbm));

        // Auto-Ack VOR Renderer-Callback — sonst sieht das Frontend den ACK
        // moeglicherweise vor dem ausloesenden Frame (Reihenfolge wichtig fuer
        // Lab-Vergleich gegen a-culfw). Der Sendepfad leert nur den TX-FIFO;
        // weitere Rahmen im RX-FIFO bleiben fuer die naechste Runde stehen.
        handle_auto_ack(frame);

        if (rx_cb_) {
            RxEvent ev{
                .proto        = ProtocolId::Moritz,
                .slot_id      = slot_id_,
                .rssi_dbm     = static_cast<int8_t>(rssi),
                .timestamp_us = static_cast<uint32_t>(esp_timer_get_time()),
                .payload      = &frame,
            };
            rx_cb_(ev);
        }

        // Gate NACH dem Callback erneut pruefen.  `rx_cb_` fuehrt in den Renderer
        // und von dort in TcpClientTransport::write -- seit 0.1.902 wartet der Task
        // dort nicht mehr am Socket (bis 0.1.893 unbegrenzt).  Lief waehrenddessen
        // ein Modewechsel durch, gehoert das Radio einer anderen Klasse, und
        // weiteres Lesen oder ein Strobe waeren nicht folgenlos:
        //
        //   SWRS061I 19.4, S. 52: "If the radio controller is currently in transmit
        //   and the SRX strobe is used, the current transmission will be ended and
        //   the transition to RX will be done."
        //
        // Ein verspaeteter SRX bricht also eine LAUFENDE Sendung ab -- still,
        // mitten im Rahmen.  Deshalb prueft die Schleife oben den Zustand und
        // strobt SRX nur aus IDLE.  (Bis 0.1.848 stand hier die unbelegte
        // Annahme, der Strobe sei harmlos.  Er ist es nicht.)
        if (!enabled_) return;
    }
    if (!enabled_) return;
    // Nach einem Flush (Resync, Overflow, CRC_OK ohne Rahmen) steht der Chip in
    // IDLE und braucht das SRX. Sonst steht er in RX (RXOFF = RX) oder sendet
    // fuer einen anderen Task — dann bleibt er unberuehrt.
    drv_->restart_rx_if_idle();
}

void MoritzRF::rx_task_loop() {
    // Eternal-Task-Pattern (siehe AskSinRF::rx_task_loop fuer Begruendung).
    // Task ueberlebt jeden stop()/start()-Zyklus; drv_==nullptr = idle.
    while (true) {
        uint32_t bits = 0;
        const bool notified =
            xTaskNotifyWait(0, UINT32_MAX, &bits, pdMS_TO_TICKS(1000)) == pdTRUE;
        if (!enabled_) continue;                         // deaktiviert — idle wait
        if (!notified) {
            // Heartbeat (a-culfw rf_moritz.c-Aequivalent: GDO2-Level pollen
            // statt rising-edge vertrauen, dazu state-Recovery).
            uint8_t ms = marc_state(*drv_);
            if (ms == MARC_RXFIFO_OVERFLOW) drv_->strobe(SFRX);
            // Auch ohne GDO2: liegt ein fertiger Rahmen im FIFO, den keine
            // Flanke gemeldet hat, holt ihn spaetestens der Heartbeat.
            // drain_fifo() fasst den Chip nur in IDLE/RX/RX_END/RX_RST/Overflow
            // an (rx_state_safe).
            if (drv_->gdo2_level() || drv_->rx_frame_waiting()) {
                drain_fifo();
            }
            // Recovery NUR bei IDLE/Overflow — laufende TX/RX/Calibrate nicht
            // anfassen, sonst kill der Heartbeat die Zs-longPreamble-TX.
            ms = marc_state(*drv_);
            if (ms == MARC_IDLE || ms == MARC_RXFIFO_OVERFLOW) {
                drv_->strobe(SIDLE); drv_->strobe(SRX);
            } else if (ms == CC1101Driver::MARC_RX && !drv_->pll_locked()) {
                // In RX ohne PLL-Lock empfaengt der Chip still nichts
                // (SWRS061I §22.1 p.58). MARC_TX bleibt unberuehrt, die
                // Zs-longPreamble-TX wird also nicht gekillt.
                drv_->note_lock_lost();
                drv_->strobe(SIDLE);
                drv_->calibrate();
                drv_->strobe(SRX);
            }
            continue;
        }
        // Erst den fertigen Rahmen (nimmt seinen gemerkten Wert mit), dann den
        // Sync eines schon laufenden Folgepakets merken (Review 0909 F3).
        if (bits & kNotifyPacket) drain_fifo();
        if (bits & kNotifySync)   latch_freqest_();
    }
}

// Single-Writer-Invariante: auto_ack_addr_/wall_thermo_ werden ausschliesslich
// zur Config-Zeit (Za/Zw aus dem Frontend bzw. NVS-Restore beim Boot) ueber
// set_auto_ack_addr()/set_wall_thermostat() geschrieben — nicht parallel zum
// RX-Pfad. handle_auto_ack() (rx_task-Kontext) liest sie nur. Kein Lock noetig;
// 3-Byte-Felder werden ohnehin per-Byte ohne Tearing geschrieben, ein worst-
// case-Mid-Update verpasst hoechstens ein einzelnes AutoAck. a-culfw macht's
// genauso ohne Sync.
void MoritzRF::handle_auto_ack(const MoritzFrame& f) {
    auto match_addr = [&](const uint8_t* a) {
        return a[0] == f.data[7] && a[1] == f.data[8] && a[2] == f.data[9];
    };

    bool any_aa = (auto_ack_addr_[0] | auto_ack_addr_[1] | auto_ack_addr_[2]) != 0;
    bool any_wt = (wall_thermo_[0]   | wall_thermo_[1]   | wall_thermo_[2])   != 0;

    // a-culfw rf_moritz.c:153ff: Auto-Ack auf bestimmte Frame-Types wenn dest matched.
    bool ack_for_aa = any_aa
                   && f.len >= 10
                   && (f.data[3] == 0x30   // ShutterContactState
                    || f.data[3] == 0x40   // SetTemperature
                    || f.data[3] == 0x50)  // PushButtonState
                   && match_addr(auto_ack_addr_);

    // Wall-Thermo-Fake antwortet nur auf SetTemperature mit dest==WT-Addr.
    bool ack_for_wt = any_wt
                   && f.len == 11
                   && f.data[3] == 0x40
                   && match_addr(wall_thermo_);

    if (ack_for_aa || ack_for_wt) send_ack(f);
}

esp_err_t MoritzRF::send_ack(const MoritzFrame& orig) {
    // ACK-Frame analog a-culfw rf_moritz.c:374ff (moritz_sendAck):
    //   ackPacket[0]  = 11 (len)
    //   ackPacket[1]  = orig msgcnt
    //   ackPacket[2]  = 0 (flag)
    //   ackPacket[3]  = 2 (type = Ack)
    //   ackPacket[4..6] = orig dest (= unsere src)
    //   ackPacket[7..9] = orig src  (= dst)
    //   ackPacket[10] = 0 (groupid)
    //   ackPacket[11] = 0 (payload)
    MoritzTxFrame ack{};
    ack.long_preamble = false;
    ack.len           = 11;
    ack.data[0]       = 11;
    ack.data[1]       = orig.data[1];                // msgcnt
    ack.data[2]       = 0;
    ack.data[3]       = 2;
    ack.data[4]       = orig.data[7];
    ack.data[5]       = orig.data[8];
    ack.data[6]       = orig.data[9];
    ack.data[7]       = orig.data[4];
    ack.data[8]       = orig.data[5];
    ack.data[9]       = orig.data[6];
    ack.data[10]      = 0;
    ack.data[11]      = 0;

    // a-culfw wartet 20 ms vor ACK (siehe Kommentar im Original „by experiments").
    vTaskDelay(pdMS_TO_TICKS(20));
    return tx(&ack, sizeof(ack));
}

esp_err_t MoritzRF::tx(const void* typed_payload, size_t payload_size) {
    if (!enabled_ || !typed_payload || payload_size < sizeof(MoritzTxFrame)) {
        return ESP_ERR_INVALID_ARG;
    }
    auto* f = static_cast<const MoritzTxFrame*>(typed_payload);
    if (f->len == 0 || f->len + 1u > MoritzFrame::kMaxPayload) return ESP_ERR_INVALID_ARG;
    if (f->data[0] != f->len) return ESP_ERR_INVALID_ARG;

    last_tx_err_ = TxError::None;

    // 0) LOVF-Check byte-genau a-culfw rf_moritz.c:261:
    //   sum = (longPreamble ? 100 : 0) + (hblen*8)/10
    // 1 kbit/s = 1 bit/ms, hblen = dec[0]+1 = f->len+1 (incl Length-Byte).
    // Bei longPreamble zusätzlich 100*10ms = 1 s Preamble eingerechnet.
    uint16_t hblen = (uint16_t)f->len + 1;
    uint16_t need  = (f->long_preamble ? 100u : 0u) + (uint16_t)((hblen * 8u) / 10u);
    if (!RfStats::instance().try_consume(need)) {
        RfStats::instance().on_tx_error(ProtocolId::Moritz);
        last_tx_err_ = TxError::Lovf;
        return ESP_ERR_INVALID_STATE;
    }

    // 1) Pre-flight: muessen im RX-State sein (a-culfw ZERR1).
    uint8_t ms = marc_state(*drv_);
    if (ms != MARC_RX) {
        last_tx_err_  = TxError::NotInRx;
        last_tx_marc_ = ms;
        // Credit war schon konsumiert — refund nicht ganz exakt, aber wir geben
        // wenigstens den Großteil zurück damit nächster Versuch noch geht.
        RfStats::instance().grant(need);
        RfStats::instance().on_tx_error(ProtocolId::Moritz);
        return ESP_ERR_INVALID_STATE;
    }

    // 2) IDLE + TXFIFO flush (defensiv, falls Reste).
    drv_->strobe(SIDLE);
    drv_->strobe(SFTX);

    auto strobe_until_tx = [&]() -> bool {
        uint8_t c = 0xFF;
        while (c--) {
            drv_->strobe(STX);
            ms = marc_state(*drv_);
            if (ms == MARC_TX) return true;
            ets_delay_us(10);
        }
        return false;
    };

    if (f->long_preamble) {
        // longPreamble-Pfad byte-genau a-culfw moritz_sendraw(longPreamble=1)
        // (rf_moritz.c:319). Reihenfolge: STX VOR TXFIFO, 1s Preamble-
        // Continuous, dann Frame. CC1101 sendet bei leerem TXFIFO im TX-State
        // weiterhin Preamble (TI-E2E-Forum-Empirie); UNDERFLOW kommt erst
        // wenn der Preamble-Burst im Hardware-FIFO leerlaeuft. 1s reicht fuer
        // schlafende MAX!-Empfaenger zum Wakeup.
        if (!strobe_until_tx()) {
            last_tx_err_  = TxError::NotInTx;
            last_tx_marc_ = ms;
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Wenn CC1101 zwischenzeitlich UNDERFLOW gesehen hat: recovery
        // + Frame-Send-Pfad fortfahren ist nicht moeglich — der Wakeup-
        // Cycle ist bereits gelaufen, wir senden den Frame trotzdem im
        // gleichen Calibration-Pfad wie die Zf-Variante.
        ms = marc_state(*drv_);
        if (ms == MARC_TXFIFO_UNDERFLOW) {
            drv_->strobe(SFTX);
            drv_->strobe(SIDLE);
            ets_delay_us(100);
            drv_->strobe(SFTX);
            // erneut in TX gehen
            if (!strobe_until_tx()) {
                last_tx_err_  = TxError::NotInTx;
                last_tx_marc_ = ms;
                return ESP_ERR_TIMEOUT;
            }
        }
        if (auto e = drv_->write_burst(CC_TXFIFO,
                {f->data, static_cast<size_t>(f->len + 1)}); e != ESP_OK) {
            last_tx_err_ = TxError::FifoNotEmpty;
            return e;
        }
    } else {
        // Zf-Pfad (long_preamble=false) — bisher byte-genau gegen Lokales+
        // Remote CUL868 verifiziert. TXFIFO first, dann STX. Reihenfolge ist
        // hier nicht-a-culfw-konform (a-culfw macht STX dann TXFIFO via
        // CS-Assert), aber funktional aequivalent fuer Frames mit Length-
        // Header < TXFIFO-Tiefe (64 bytes), und hat sich bewaehrt.
        if (auto e = drv_->write_burst(CC_TXFIFO,
                {f->data, static_cast<size_t>(f->len + 1)}); e != ESP_OK) {
            last_tx_err_ = TxError::FifoNotEmpty;
            return e;
        }
        if (!strobe_until_tx()) {
            last_tx_err_  = TxError::NotInTx;
            last_tx_marc_ = ms;
            return ESP_ERR_TIMEOUT;
        }
    }

    // Warten bis Frame gesendet + Auto-RX-Restart durch (MCSM1.TXOFF=RX).
    // 12-Byte-Frame bei ~10 kbit/s = ~16 ms TX-Zeit, + Auto-Cal (~1 ms),
    // + Settling (~2 ms) = ~20 ms minimum. SPI-Polls waehrend TX sind laut
    // CC1101-Errata unzuverlaessig (a-culfw umgeht das mit Doppel-Read; bei uns
    // reicht das nicht — wir warten erst hart, dann pollen).
    vTaskDelay(pdMS_TO_TICKS(30));
    for (int i = 0; i < 250; ++i) {
        ms = marc_state(*drv_);
        if (ms == MARC_RX) {
            last_tx_marc_ = ms;
            RfStats::instance().on_tx(ProtocolId::Moritz, hblen, need);
            return ESP_OK;
        }
        if (ms == MARC_TXFIFO_UNDERFLOW) {
            // Recovery-Versuch.
            drv_->strobe(SFTX);
            drv_->strobe(SIDLE);
            drv_->strobe(SRX);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        // Nach jedem Schlaf neu pruefen: die Schleife kann ueber einen
        // Modewechsel hinweg laufen, und dann traefen SFTX/SIDLE/SRX oben den
        // Chip der Nachfolgeklasse (SWRS061I 19.4 S. 52 -- SRX beendet eine
        // laufende Sendung).
        if (!enabled_) return ESP_ERR_INVALID_STATE;
    }
    last_tx_err_  = TxError::NoReturnRx;
    last_tx_marc_ = ms;
    RfStats::instance().on_tx_error(ProtocolId::Moritz);
    return ESP_ERR_TIMEOUT;
}

void MoritzRF::set_auto_ack_addr(std::span<const uint8_t> addr) {
    if (addr.size() < 3) return;
    auto_ack_addr_[0] = addr[0]; auto_ack_addr_[1] = addr[1]; auto_ack_addr_[2] = addr[2];
}

void MoritzRF::set_wall_thermostat(std::span<const uint8_t> addr) {
    if (addr.size() < 3) return;
    wall_thermo_[0] = addr[0]; wall_thermo_[1] = addr[1]; wall_thermo_[2] = addr[2];
}

esp_err_t MoritzRF::start(uint8_t slot_id, CC1101Driver& drv) {
    drv_     = &drv;
    slot_id_ = slot_id;

    drv.strobe(SIDLE);
    ets_delay_us(100);

    // Software-Reset bei jedem Mode-Switch (siehe AskSinRF::start fuer
    // ausfuehrliche Begruendung — leftover Reg-State von vorigem Modus, z.B.
    // SignalDuino's OOK-Direct-Mode-PKTCTRL0).
    drv.strobe(SRES);
    ets_delay_us(100);

    for (size_t i = 0; i < sizeof(MORITZ_CFG); i += 2) {
        uint8_t addr = MORITZ_CFG[i];
        if (addr > 0x40) break;
        if (auto e = drv.write_reg(addr, MORITZ_CFG[i + 1]); e != ESP_OK) {
            ESP_LOGE(TAG, "write_reg(0x%02x): %s", addr, esp_err_to_name(e));
            return e;
        }
    }

    // SCAL mit Lock-Pruefung (SWRS061I §22.1 p.58: FSCAL1 != 0x3F == Lock).
    if (drv.calibrate() != ESP_OK) {
        ESP_LOGE(TAG, "PLL ohne Lock — RX/TX wird ausfallen");
    }

    uint8_t cnt = 0xFF;
    while (cnt--) {
        uint8_t st = 0;   // Status race-frei unter demselben Bus-Lock erfassen
        if (drv.strobe(SRX, st) == ESP_OK && ((st >> 4) & 0x07) == 0x01) break;
        ets_delay_us(10);
    }

    if (!task_handle_) {
        TaskHandle_t h = nullptr;
        if (xTaskCreate(&MoritzRF::rx_task_trampoline, "moritz-rx", 4096, this, 8, &h) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
        task_handle_ = h;
    }

    if (auto e = drv.attach_gdo2_irq(&MoritzRF::gdo2_isr, this, /*rising=*/true); e != ESP_OK) {
        ESP_LOGW(TAG, "GDO2 IRQ attach: %s", esp_err_to_name(e));
    }
    // Sync-Flanke fuer FREQEST waehrend des Pakets (latch_freqest_). Faellt der
    // Interrupt aus, fehlen nur die FREQEST-Werte, der Empfang laeuft ueber GDO2.
    pending_fe_valid_ = false;
    if (auto e = drv.attach_gdo0_irq(&MoritzRF::gdo0_isr, this, /*rising=*/false); e != ESP_OK) {
        ESP_LOGW(TAG, "GDO0 IRQ attach: %s", esp_err_to_name(e));
    }

    ESP_LOGI(TAG, "started on slot %u (status=0x%02x)", slot_id, drv.last_status());

    enabled_ = true;   // ab hier darf der RX-Task ans Radio
    return ESP_OK;
}

esp_err_t MoritzRF::stop() {
    // Eternal-Task-Pattern — Task NICHT killen, nur das Flag loeschen.
    // Der Zeiger wird NICHT mehr genullt: der CC1101Driver ist ein WERT-Member
    // des Slots (cc1101_slot.hpp) und lebt, solange der Manager lebt -- er wird
    // also nie ungueltig.  Ihn zu loeschen brachte keine Sicherheit, sondern
    // verwandelte "Task fasst ein stillgelegtes Radio an" in einen Null-Zugriff:
    // der RX-Task laeuft hoeher priorisiert (8) als die Frontend-Tasks (5/6) und
    // kann in drain_fifo() warten (SPI-Mutex; Renderer ->
    // TcpClientTransport::write, seit 0.1.902 ohne Warten am Socket; bis 0.1.893 unbegrenzt).
    // Kehrte er zurueck, waehrend das Frontend zwischen stop() und start() stand,
    // lief ein strobe() auf einen geloeschten Zeiger.  Gegated wird ueber das Flag.
    enabled_ = false;
    if (drv_) drv_->strobe(SIDLE);
    return ESP_OK;
}

} // namespace culfw32
