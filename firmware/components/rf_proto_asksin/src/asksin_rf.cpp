// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
// Portions derived from a-culfw (https://github.com/tostmann/a-culfw),
// GNU GPL version 2 or (at your option) any later version.

#include "asksin_rf.hpp"
#include "asksin_cfg.hpp"
#include "rf_stats.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

namespace culfw32 {

std::atomic<uint32_t> AskSinRF::s_last_tx_src_hmid_ {0};
std::atomic<uint32_t> AskSinRF::s_last_tx_dst_hmid_ {0};
std::atomic<uint8_t>  AskSinRF::s_last_tx_cnt_      {0};
std::atomic<bool>     AskSinRF::s_last_tx_valid_    {false};

namespace {
constexpr const char* TAG = "asksin";

constexpr uint8_t CC_TXFIFO  = 0x3F;          // Adresse wie RX-FIFO, Schreibzugriff waehlt TX
constexpr uint8_t REG_MARC   = 0x35;

constexpr uint8_t SRES  = 0x30;
constexpr uint8_t SRX   = 0x34;
constexpr uint8_t STX   = 0x35;
constexpr uint8_t SIDLE = 0x36;
constexpr uint8_t SNOP  = 0x3D;
constexpr uint8_t SFRX  = 0x3A;
constexpr uint8_t SFTX  = 0x3B;

constexpr uint8_t MARC_IDLE              = 0x01;
constexpr uint8_t MARC_RX                = 0x0D;
constexpr uint8_t MARC_TX                = 0x13;
constexpr uint8_t MARC_RXFIFO_OVERFLOW   = 0x11;
constexpr uint8_t MARC_TXFIFO_UNDERFLOW  = 0x16;

// a-culfw ASKSIN_WAIT_TICKS_CCA = 188 @ 125 Hz Tick = ~1.504 s.
constexpr uint32_t CCA_TIMEOUT_MS = 1500;

uint8_t marc_state(CC1101Driver& d) {
    uint8_t v = 0xFF;
    d.read_status(REG_MARC, v);
    return v & 0x1F;
}
}

// Crypt-Decoder. Byte-genau a-culfw rf_asksin.c:175..185 (RX-Pfad).
// In-place: msg muss msg[0]+1 Bytes lang sein.
void AskSinRF::decrypt(uint8_t* msg) {
    uint8_t this_enc, last_enc;
    last_enc = msg[1];
    msg[1] = (~msg[1]) ^ 0x89;
    uint8_t l;
    for (l = 2; l < msg[0]; l++) {
        this_enc = msg[l];
        msg[l] = (last_enc + 0xdc) ^ msg[l];
        last_enc = this_enc;
    }
    msg[l] = msg[l] ^ msg[2];
}

// Crypt-Encoder. Byte-genau a-culfw rf_asksin.c:248..254 (TX-Pfad).
// Achtung: ctl muss VOR dieser Funktion gesichert werden, falls fuer BURST-Check
// gebraucht — Encrypt schreibt msg[2] ueber.
void AskSinRF::encrypt(uint8_t* msg) {
    uint8_t ctl = msg[2];
    msg[1] = (~msg[1]) ^ 0x89;
    uint8_t l;
    for (l = 2; l < msg[0]; l++)
        msg[l] = (msg[l-1] + 0xdc) ^ msg[l];
    msg[l] = msg[l] ^ ctl;
}

const CC1101Profile& AskSinRF::profile() const {
    static const CC1101Profile p = {
        .freq_hz        = 868'300'000,
        .datarate_bps   = 9'993,
        .deviation_hz   = 19'042,
        .mod            = CC1101Profile::Modulation::FSK_2,
        .sync_word      = {0xE9, 0xCA, 0, 0},
        .sync_len       = 16,
        .pa_table       = {0xC3, 0, 0, 0, 0, 0, 0, 0},
        .raw_regs       = nullptr,
    };
    return p;
}

void IRAM_ATTR AskSinRF::gdo2_isr(void* arg) {
    auto* self = static_cast<AskSinRF*>(arg);
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(self->task_handle_), &hpw);
    portYIELD_FROM_ISR(hpw);
}

void AskSinRF::rx_task_trampoline(void* arg) {
    static_cast<AskSinRF*>(arg)->rx_task_loop();
    vTaskDelete(nullptr);
}

void AskSinRF::drain_fifo() {
    if (!enabled_) return;
    // MCSM1 = 0x33: nach einem Paket geht der Chip in IDLE, mehr als ein Rahmen
    // steht hier praktisch nie im FIFO. Gelesen wird trotzdem ueber denselben
    // Weg wie bei Moritz/HmIP — er prueft vor dem Lesen, ob ueberhaupt Daten da
    // sind, und gibt kein CRC_OK-Signal ohne Rahmen als Geisterrahmen aus
    // (siehe CC1101Driver::read_rx_packet).
    for (int i = 0; i < CC1101Driver::kRxFramesPerDrain; ++i) {
        // Ein blockierter Callback kann zurueckkehren, waehrend ein anderer Task
        // sendet (tx() ohne Sperre gegen diesen Task); Lesen, Flushen oder SRX
        // wuerden die Sendung abbrechen. Nur in RX/IDLE/Overflow weitermachen.
        if (!drv_->rx_state_safe()) break;
        AskSinFrame frame{};
        uint8_t rssi = 0, lqi = 0;
        const auto r = drv_->read_rx_packet({frame.data, AskSinFrame::kMaxPayload},
                                            rssi, lqi, i == 0);
        if (r == CC1101Driver::RxPacket::Overflow) { drv_->flush_rx(); break; }
        if (r != CC1101Driver::RxPacket::Frame)    break;   // Empty/Busy/Error; Resync ist geflusht

        const uint8_t len = frame.data[0];
        frame.len      = len;
        frame.rssi_raw = rssi;
        frame.lqi_raw  = lqi;
        // Frequenzoffset-Schaetzung mitnehmen — nur frame-synchron belastbar
        // (SWRS061I §14.1). Speist `?rf.freqest` / `?rf.freqcal=apply`. Muss VOR
        // dem SRX unten stehen: der neue RX-Start setzt den Schaetzer zurueck.
        drv_->sample_freqest();

        // RX sofort wieder scharf schalten, VOR Decrypt und Callback — wie
        // a-culfw (rf_asksin.c: SRX direkt nach dem FIFO-Lesen). MCSM1 = 0x33
        // laesst den Chip nach jedem Paket in IDLE; bis 0.1.894 kam das SRX erst
        // nach dem Callback, und solange der Renderer stand (TCP bis 0.1.901
        // bis zu 3 s), war der Empfaenger taub — gemessen 13.10.2026: jeder
        // Rahmen im Stau-Fenster verloren. Jetzt faengt der Chip waehrenddessen
        // das naechste Paket auf; die Schleife liest es nach dem Callback.
        if (!enabled_) return;
        drv_->restart_rx_if_idle();

        // RSSI dBm (CC1101 §17.3 / a-culfw cc1100.c).
        int8_t rs = static_cast<int8_t>(rssi);
        int16_t rssi_dbm = (rs < 0) ? (rs / 2 - 74) : (rs / 2 + 64 - 74);
        if (rssi_dbm < -128) rssi_dbm = -128;
        if (rssi_dbm > 127)  rssi_dbm =  127;
        // CRC-Fehler kommen hier nicht an: CRC_AUTOFLUSH verwirft solche
        // Pakete im Chip, einen Rahmen ohne CRC_OK verwirft read_rx_packet
        // (zaehlt als rx_resync).
        RfStats::instance().on_rx(ProtocolId::AskSin, len, static_cast<int8_t>(rssi_dbm));

        // Decrypt in-place. Frontend bekommt Klartext.
        decrypt(frame.data);

        if (rx_cb_) {
            RxEvent ev{
                .proto        = ProtocolId::AskSin,
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
        // mitten im Rahmen, der Nachfolgeklasse wie der eigenen aus einem anderen
        // Task (tx() laeuft ohne Sperre gegen diesen Task).  Deshalb prueft die
        // Schleife oben den Zustand, und SRX kommt nur aus IDLE.  (Bis 0.1.848
        // stand hier die unbelegte Annahme, der Strobe sei harmlos.  Er ist es
        // nicht.)
        if (!enabled_) return;
    }
    if (!enabled_) return;
    // Nach einem Flush (Resync, Overflow, CRC_OK ohne Rahmen) steht der Chip in
    // IDLE und braucht das SRX; nach einem Rahmen steht er schon in RX (oben
    // re-armt) oder sendet fuer einen anderen Task — dann bleibt er unberuehrt.
    drv_->restart_rx_if_idle();
}

void AskSinRF::rx_task_loop() {
    // Eternal-Task-Pattern: Task lebt bis zum Reboot, ueberlebt jeden
    // stop()/start()-Zyklus. drv_==nullptr signalisiert "deaktiviert"
    // (Mode-Switch lief auf andere RF-Klasse) — Task pollt dann nur leer
    // weiter ohne Chip-Zugriff. Versuch im Build 0.1.115..0.1.135 mit
    // task_running_-Flag + vTaskDelete fuehrte zur Regression: stop()
    // killte den Task waehrend task_handle_ stale blieb, sodass beim
    // naechsten start() KEIN neuer Task erzeugt wurde (xTaskCreate-Guard
    // `if (!task_handle_)` traf zu) und der gdo2_isr Notify in eine tote
    // TaskHandle schickte. Resultat: AskSin RX komplett broken nach jedem
    // Mode-Switch (PulseCapture→AskSin via Ar). Mirror 0.1.102-Pattern.
    while (true) {
        bool notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) > 0;
        if (!enabled_) continue;                         // deaktiviert — idle wait
        if (!notified) {
            // Heartbeat (a-culfw rf_asksin.c:206ff: RXFIFO_OVERFLOW oder
            // IDLE -> SRX). Zusaetzlich GDO2-Level pollen falls Edge-IRQ
            // verpasst wurde (a-culfw rf_asksin.c:147 macht es genauso).
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
            // anfassen, sonst killt der Heartbeat eine Moritz-Zs-1s-Preamble-
            // TX (verifiziert 2026-04-30 Build 0.1.136: CULFW32→CUL868 Zs
            // verschwand wenn Heartbeat-Timeout in die TX-Phase fiel).
            ms = marc_state(*drv_);
            if (ms == MARC_IDLE || ms == MARC_RXFIFO_OVERFLOW) {
                drv_->strobe(SIDLE); drv_->strobe(SNOP); drv_->strobe(SRX);
            } else if (ms == CC1101Driver::MARC_RX && !drv_->pll_locked()) {
                // Der Chip kann in RX stehen OHNE PLL-Lock — dann empfaengt er
                // STILL nichts: kein Fehlerbit, keine Meldung, nur ausbleibende
                // Frames. SWRS061I §22.1 p.58 nennt FSCAL1!=0x3F als Lock-Beleg
                // und verlangt Nachkalibrieren. Was der Check faengt: eine
                // gescheiterte (Auto-)Kalibrierung beim RX-Eintritt (MCSM0
                // FS_AUTOCAL=01 kalibriert bei jedem IDLE->RX; genau den Fall
                // sah tsculfw als "stuck in RX without PLL lock"). Was er NICHT
                // faengt: Lock-Verlust durch Drift bei stehendem RX — FSCAL1 ist
                // ein Kalibrier-Ergebnis, kein Live-Detektor. Bewiesen 08-17:
                // FSCAL1:=0x3F -> Recal <1 s; FREQ2:=0x00 -> pll_fail zaehlt.
                // Nur hier im Heartbeat und nur aus RX heraus — laufende TX/
                // Calibrate-Phasen bleiben unangetastet (Regression 0.1.136).
                drv_->note_lock_lost();
                drv_->strobe(SIDLE);
                drv_->calibrate();
                drv_->strobe(SRX);
            }
            continue;
        }
        drain_fifo();
    }
}

esp_err_t AskSinRF::tx(const void* typed_payload, size_t payload_size) {
    if (!enabled_ || !typed_payload || payload_size < sizeof(AskSinTxFrame)) {
        return ESP_ERR_INVALID_ARG;
    }
    auto* f = static_cast<const AskSinTxFrame*>(typed_payload);
    if (f->len == 0 || f->len + 1u > AskSinFrame::kMaxPayload) return ESP_ERR_INVALID_ARG;
    if (f->data[0] != f->len) return ESP_ERR_INVALID_ARG;

    last_tx_err_ = TxError::None;

    // Lokale Kopie + Encrypt + BURST-ctl-Sicherung.
    uint8_t msg[AskSinFrame::kMaxPayload];
    std::memcpy(msg, f->data, f->len + 1u);
    uint8_t ctl_clear = msg[2];

    // Diagnostik: on-air-Header VOR encrypt erfassen. Beantwortet "ist
    // APP_SET_HMID(xxxxxx) wirklich aktiv?" via `?stats`/JSON ohne Sniffer.
    // Frame-Layout (cleartext): [0]=LEN [1]=CNT [2]=FLAG [3]=TYPE [4..6]=SRC
    // [7..9]=DST [10..]=payload.  AskSin-Crypt laesst Header 0..9 plaintext.
    if (f->len >= 9) {
        uint32_t src = (uint32_t(msg[4]) << 16) | (uint32_t(msg[5]) << 8) | msg[6];
        uint32_t dst = (uint32_t(msg[7]) << 16) | (uint32_t(msg[8]) << 8) | msg[9];
        s_last_tx_src_hmid_.store(src, std::memory_order_relaxed);
        s_last_tx_dst_hmid_.store(dst, std::memory_order_relaxed);
        s_last_tx_cnt_.store(msg[1], std::memory_order_relaxed);
        s_last_tx_valid_.store(true, std::memory_order_relaxed);
    }

    encrypt(msg);

    // 1) CCA-Loop: STX strobe bis MARCSTATE==TX, Timeout 1.5 s.
    // a-culfw asksin_send macht hier KEINEN Pre-Flight-Check (chip kann in
    // IDLE/RX/sonstigem stecken). CCA-Mode=11 sorgt dafuer dass STX aus RX
    // nur dann nach TX wechselt wenn Kanal frei; aus IDLE geht STX laut
    // Datenblatt direkt nach TX. Beide Faelle abgedeckt.
    uint8_t ms;
    int64_t t0 = esp_timer_get_time();
    while (true) {
        drv_->strobe(STX);
        ms = marc_state(*drv_);
        if (ms == MARC_TX) break;
        int64_t elapsed = (esp_timer_get_time() - t0) / 1000;
        if (elapsed > CCA_TIMEOUT_MS) {
            last_tx_err_  = TxError::CcaTimeout;
            last_tx_marc_ = ms;
            // Aufraeumen: zurueck nach RX.
            drv_->strobe(SIDLE); drv_->strobe(SRX);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(8));    // a-culfw pollt mit ~8ms Tick-Rate
    }

    // 3) BURST-Preamble. a-culfw rf_asksin.c:271ff:
    //   - ctl & 0x10 (Bit 4): 360 ms Preamble (3 * 120 ms — uint8 Loop-Limit).
    //   - sonst:               10 ms.
    // Waehrend dieser Zeit ist Chip in TX und sendet Preamble + Sync continuously.
    if (ctl_clear & 0x10) {
        vTaskDelay(pdMS_TO_TICKS(360));
    } else {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 4) FIFO-Write: encrypted msg, len+1 bytes (Length-Byte mit gesendet).
    if (auto e = drv_->write_burst(CC_TXFIFO, {msg, static_cast<size_t>(f->len + 1)});
        e != ESP_OK) {
        last_tx_err_ = TxError::FifoNotEmpty;
        return e;
    }

    // 5) Hard-Wait nach FIFO-Write (CC1101-SPI-Errata, siehe Moritz). Frame-TX
    // bei 30 Bytes / 10 kbit/s = ~25 ms. Wir warten 30 ms hart, dann pollen.
    vTaskDelay(pdMS_TO_TICKS(30));
    for (int i = 0; i < 250; ++i) {
        ms = marc_state(*drv_);
        if (ms == MARC_RX) {
            last_tx_marc_ = ms;
            // a-culfw checkt KEIN credit_10ms in AskSin (siehe rf_asksin.c) — nur
            // Stats-Counter erfassen, kein LOVF. AskSin-Frames sind ueblicherweise
            // BURST-getriggert von Endgeraeten (Reaktion), Duty-Cycle wird durch
            // Anwendungslogik begrenzt.
            uint16_t airtime = (uint16_t)((((uint16_t)f->len + 1) * 8u) / 10u);
            RfStats::instance().on_tx(ProtocolId::AskSin, f->len, airtime);
            return ESP_OK;
        }
        if (ms == MARC_TXFIFO_UNDERFLOW) {
            // a-culfw rf_asksin.c:294ff: SFTX, SIDLE, SNOP — dann Restore-RX.
            drv_->strobe(SFTX);
            drv_->strobe(SIDLE);
            drv_->strobe(SNOP);
            drv_->strobe(SRX);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    last_tx_err_  = TxError::NoReturnRx;
    last_tx_marc_ = ms;
    RfStats::instance().on_tx_error(ProtocolId::AskSin);
    // Defensiver Restore wenn TX in odd state stecken bleibt.
    drv_->strobe(SIDLE); drv_->strobe(SRX);
    return ESP_ERR_TIMEOUT;
}

esp_err_t AskSinRF::start(uint8_t slot_id, CC1101Driver& drv) {
    drv_     = &drv;
    slot_id_ = slot_id;

    drv.strobe(SIDLE);
    ets_delay_us(100);

    // Software-Reset (SRES) bevor wir ASKSIN_CFG laden. Notwendig weil bei
    // Mode-Switch (z.B. SignalDuino-RF → AskSin) der vorherige Modus Register
    // hinterlaesst, die ASKSIN_CFG nicht ueberschreibt — verifiziert 2026-04-30:
    // SignalDuino setzt PKTCTRL0=0x32 (OOK Async Direct-Mode) und AGCCTRL1/0
    // auf OOK-tuned Werte; AskSin's CFG laesst diese stehen, Frame-TX bleibt
    // dann in MARC_TX hangen (AERR313). a-culfw rf_asksin.c:95 macht denselben
    // SRES vor Cfg-Load — Pattern uebernommen.
    drv.strobe(SRES);
    ets_delay_us(100);

    for (size_t i = 0; i < sizeof(ASKSIN_CFG); i += 2) {
        uint8_t addr = ASKSIN_CFG[i];
        if (addr > 0x40) break;
        if (auto e = drv.write_reg(addr, ASKSIN_CFG[i + 1]); e != ESP_OK) {
            ESP_LOGE(TAG, "write_reg(0x%02x): %s", addr, esp_err_to_name(e));
            return e;
        }
    }

    // FUP-Mode-Overlay (a-culfw rf_asksin.c:104-110): zusaetzliche Reg-Werte
    // fuer hoehere Datarate (~50 kbps statt ~10 kbps), gefolgt von Re-Cal.
    if (update_mode_) {
        for (size_t i = 0; i < sizeof(ASKSIN_UPDATE_CFG); i += 2) {
            uint8_t addr = ASKSIN_UPDATE_CFG[i];
            if (addr > 0x40) break;
            if (auto e = drv.write_reg(addr, ASKSIN_UPDATE_CFG[i + 1]); e != ESP_OK) {
                ESP_LOGE(TAG, "update_cfg write_reg(0x%02x): %s", addr, esp_err_to_name(e));
                return e;
            }
        }
        ESP_LOGI(TAG, "FUP-Mode (AR) aktiv — ~50 kbps Update-Profil");
    }

    // SCAL mit Lock-Pruefung statt blind 4 ms zu warten: SWRS061I §22.1 p.58 —
    // "The PLL is in lock if the register content [FSCAL1] is different from
    // 0x3F", und es ist neu zu kalibrieren, bis Lock steht. Ohne Lock arbeitet
    // der Chip STILL nicht (kein Fehlerbit, nur ausbleibende Frames).
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
        if (xTaskCreate(&AskSinRF::rx_task_trampoline, "asksin-rx", 4096, this, 8, &h) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
        task_handle_ = h;
    }

    if (auto e = drv.attach_gdo2_irq(&AskSinRF::gdo2_isr, this, /*rising=*/true); e != ESP_OK) {
        ESP_LOGW(TAG, "GDO2 IRQ attach: %s", esp_err_to_name(e));
    }

    ESP_LOGI(TAG, "started on slot %u (status=0x%02x)", slot_id, drv.last_status());

    enabled_ = true;   // ab hier darf der RX-Task ans Radio
    return ESP_OK;
}

esp_err_t AskSinRF::stop() {
    // Task NICHT killen — Eternal-Task-Pattern, siehe rx_task_loop.  Das Flag
    // auf false zu setzen reicht: der Task pollt dann leer weiter und greift
    // den naechsten start() ohne xTaskCreate-Race auf.
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
