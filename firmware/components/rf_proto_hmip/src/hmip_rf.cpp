// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "hmip_rf.hpp"
#include "asksin_rf.hpp"   // geteiltes AskSin/BidCoS-Whitening (HmIP nutzt dasselbe)
#include "hmip_cfg.hpp"
#include "rf_stats.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG = "hmip";

constexpr uint8_t CC_PATABLE = 0x3E;
constexpr uint8_t REG_MARC   = 0x35;
constexpr uint8_t REG_TXBYTES = 0x3A;   // Status-Read (Burst-Bit); als Strobe waere 0x3A = SFRX

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

uint8_t marc_state(CC1101Driver& d) {
    uint8_t v = 0xFF;
    d.read_status(REG_MARC, v);
    return v & 0x1F;
}
} // anon

const CC1101Profile& HmipRF::profile() const {
    static const CC1101Profile p = {
        .freq_hz        = 868'305'000,
        .datarate_bps   = 9'993,
        .deviation_hz   = 19'042,
        .mod            = CC1101Profile::Modulation::FSK_2,
        .sync_word      = {0xE9, 0xCA, 0, 0},
        .sync_len       = 16,
        .pa_table       = {HMIP_PA_DEFAULT, 0, 0, 0, 0, 0, 0, 0},
        .raw_regs       = nullptr,
    };
    return p;
}

void IRAM_ATTR HmipRF::gdo2_isr(void* arg) {
    auto* self = static_cast<HmipRF*>(arg);
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(self->task_handle_), &hpw);
    portYIELD_FROM_ISR(hpw);
}

void HmipRF::rx_task_trampoline(void* arg) {
    static_cast<HmipRF*>(arg)->rx_task_loop();
    vTaskDelete(nullptr);
}

void HmipRF::drain_fifo() {
    if (!enabled_) return;
    // Alle fertigen Rahmen lesen, nicht nur den gemeldeten: MCSM1 = 0x0F laesst
    // den Chip nach jedem Paket in RX, und HmIP-Quittungen folgen dem Rahmen nach
    // wenigen Millisekunden. GDO2 meldet nur einen Rahmen je Flanke (siehe
    // CC1101Driver::read_rx_packet).
    for (int i = 0; i < CC1101Driver::kRxFramesPerDrain; ++i) {
        // Ein blockierter Callback kann zurueckkehren, waehrend ein anderer Task
        // sendet (tx() ohne Sperre gegen diesen Task); Lesen, Flushen oder SRX
        // wuerden die Sendung abbrechen. Nur in RX/IDLE/Overflow weitermachen.
        if (!drv_->rx_state_safe()) break;
        HmipFrame frame{};
        uint8_t rssi = 0, lqi = 0;
        const auto r = drv_->read_rx_packet({frame.data, HmipFrame::kMaxPayload},
                                            rssi, lqi, i == 0);
        if (r == CC1101Driver::RxPacket::Overflow) { drv_->flush_rx(); break; }
        if (r != CC1101Driver::RxPacket::Frame)    break;   // Empty/Busy/Error; Resync ist geflusht

        const uint8_t len = frame.data[0];
        frame.len = len;

        // HmIP teilt sich PHY *und* Software-Whitening mit BidCoS/AskSin (Sync 0xE9CA,
        // gleiches CC1101-Profil). Das CC1101-PN9-Whitening (PKTCTRL0=0x45) nimmt der
        // Chip selbst raus; DIESE Schicht hier ist die zusaetzliche a-culfw-AskSin-
        // Verschleierung (0x89 / +0xdc, rf_asksin.c:175..185). Ohne sie bleibt jeder
        // HmIP-Frame "opaque" — genau der Grund, warum die Inclusion-Frames (sec=3/ct=7)
        // jahrelang als unbekanntes "UDP-compressed"-Format galten.
        // Nachgewiesen 2026-08-10 am Luftmitschnitt: dewhiten(Luft-Request) legt SGTIN +
        // DeviceInclusionRequest-Layout im Klartext frei; whiten(NM-Accept-Body) == der
        // mitgeschnittene Luftframe, byte-identisch.
        AskSinRF::decrypt(frame.data);

        // PKTCTRL1=0x0C (APPEND_STATUS=1) → 2 Status-Bytes nach payload: RSSI, LQI/CRC_OK.
        frame.rssi_raw = rssi;
        frame.lqi_raw  = lqi;

        // RSSI dBm (CC1101 §17.3 / a-culfw cc1100.c).
        int8_t rs = static_cast<int8_t>(rssi);
        int16_t rssi_dbm = (rs < 0) ? (rs / 2 - 74) : (rs / 2 + 64 - 74);
        if (rssi_dbm < -128) rssi_dbm = -128;
        if (rssi_dbm > 127)  rssi_dbm =  127;
        // CRC-Fehler kommen hier nicht an: CRC_AUTOFLUSH verwirft solche
        // Pakete im Chip, einen Rahmen ohne CRC_OK verwirft read_rx_packet
        // (zaehlt als rx_resync).
        RfStats::instance().on_rx(ProtocolId::Hmip, len, static_cast<int8_t>(rssi_dbm));

        if (rx_cb_) {
            RxEvent ev{
                .proto        = ProtocolId::Hmip,
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

void HmipRF::rx_task_loop() {
    // Eternal-Task-Pattern (siehe AskSinRF::rx_task_loop). drv_=nullptr
    // signalisiert "deaktiviert".
    while (true) {
        bool notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) > 0;
        if (!enabled_) continue;
        if (!notified) {
            uint8_t ms = marc_state(*drv_);
            if (ms == MARC_RXFIFO_OVERFLOW) drv_->strobe(SFRX);
            // Auch ohne GDO2: liegt ein fertiger Rahmen im FIFO, den keine
            // Flanke gemeldet hat, holt ihn spaetestens der Heartbeat.
            // drain_fifo() fasst den Chip nur in IDLE/RX/RX_END/RX_RST/Overflow
            // an (rx_state_safe).
            if (drv_->gdo2_level() || drv_->rx_frame_waiting()) {
                drain_fifo();
            }
            ms = marc_state(*drv_);
            if (ms == MARC_IDLE || ms == MARC_RXFIFO_OVERFLOW) {
                drv_->strobe(SIDLE); drv_->strobe(SNOP); drv_->strobe(SRX);
            } else if (ms == CC1101Driver::MARC_RX && !drv_->pll_locked()) {
                // In RX ohne PLL-Lock empfaengt der Chip still nichts
                // (SWRS061I §22.1 p.58). Nur im Heartbeat, nur aus RX heraus.
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


// Reiner Mithoerer: kein Sendepfad. Das Interface verlangt tx(), der Manager
// routet wegen supports_tx()==false nie hierher. HmIP-Senden ist q-culfw-Sache.
esp_err_t HmipRF::tx(const void*, size_t) {
    return ESP_ERR_NOT_SUPPORTED;
}


esp_err_t HmipRF::start(uint8_t slot_id, CC1101Driver& drv) {
    drv_     = &drv;
    slot_id_ = slot_id;

    drv.strobe(SIDLE);
    ets_delay_us(100);

    // SRES vor Cfg-Load (gleicher Grund wie AskSin: Mode-Switch koennte
    // PKTCTRL0/AGCCTRL stehen lassen und Frame-TX in MARC_TX haengenlassen).
    drv.strobe(SRES);
    ets_delay_us(100);

    for (size_t i = 0; i < sizeof(HMIP_CFG); i += 2) {
        uint8_t addr = HMIP_CFG[i];
        if (addr > 0x40) break;
        if (auto e = drv.write_reg(addr, HMIP_CFG[i + 1]); e != ESP_OK) {
            ESP_LOGE(TAG, "write_reg(0x%02x): %s", addr, esp_err_to_name(e));
            return e;
        }
    }

    // PATABLE[0] = +10 dBm (HmIP-RFUSB-Default).
    if (auto e = drv.write_reg(CC_PATABLE, HMIP_PA_DEFAULT); e != ESP_OK) {
        ESP_LOGW(TAG, "PATABLE write: %s", esp_err_to_name(e));
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
        if (xTaskCreate(&HmipRF::rx_task_trampoline, "hmip-rx", 4096, this, 8, &h) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
        task_handle_ = h;
    }

    if (auto e = drv.attach_gdo2_irq(&HmipRF::gdo2_isr, this, /*rising=*/true); e != ESP_OK) {
        ESP_LOGW(TAG, "GDO2 IRQ attach: %s", esp_err_to_name(e));
    }

    ESP_LOGI(TAG, "started on slot %u (status=0x%02x)", slot_id, drv.last_status());

    enabled_ = true;   // ab hier darf der RX-Task ans Radio
    return ESP_OK;
}

esp_err_t HmipRF::stop() {
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
