// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
// Portions derived from a-culfw (https://github.com/tostmann/a-culfw),
// GNU GPL version 2 or (at your option) any later version.

#include "fastrf_rf.hpp"
#include "fastrf_cfg.hpp"
#include "rf_stats.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace culfw32 {

namespace {
constexpr const char* TAG = "fastrf";

constexpr uint8_t CC_RXFIFO  = 0x3F;
constexpr uint8_t CC_TXFIFO  = 0x3F;
constexpr uint8_t REG_MARC    = 0x35;
constexpr uint8_t REG_TXBYTES = 0x3A;

constexpr uint8_t SRX   = 0x34;
constexpr uint8_t STX   = 0x35;
constexpr uint8_t SIDLE = 0x36;
constexpr uint8_t SFRX  = 0x3A;
constexpr uint8_t SFTX  = 0x3B;

constexpr uint8_t MARC_IDLE              = 0x01;
constexpr uint8_t MARC_RX                = 0x0D;
constexpr uint8_t MARC_TX                = 0x13;
constexpr uint8_t MARC_RXFIFO_OVERFLOW   = 0x11;

uint8_t marc_state(CC1101Driver& d) {
    uint8_t v = 0xFF;
    d.read_status(REG_MARC, v);
    return v & 0x1F;
}

esp_err_t push_cfg(CC1101Driver& drv, const uint8_t* cfg, size_t max_pairs) {
    for (size_t i = 0; i < max_pairs * 2; i += 2) {
        uint8_t addr = cfg[i];
        if (addr > 0x40) break;
        if (auto e = drv.write_reg(addr, cfg[i + 1]); e != ESP_OK) return e;
    }
    return ESP_OK;
}
}

const CC1101Profile& FastRfRF::profile() const {
    static const CC1101Profile p = {
        .freq_hz        = 868'299'820,
        .datarate_bps   = 250'000,
        .deviation_hz   = 65'000,   // Profile-Feld ist uint16; tatsaechlich 127 kHz (DEVIATN=0x62)
        .mod            = CC1101Profile::Modulation::GFSK,
        .sync_word      = {0x91, 0xD3, 0, 0},
        .sync_len       = 16,
        .pa_table       = {0xC3, 0, 0, 0, 0, 0, 0, 0},
        .raw_regs       = nullptr,
    };
    return p;
}

void IRAM_ATTR FastRfRF::gdo_isr(void* arg) {
    auto* self = static_cast<FastRfRF*>(arg);
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(self->task_handle_), &hpw);
    portYIELD_FROM_ISR(hpw);
}

void FastRfRF::rx_task_trampoline(void* arg) {
    static_cast<FastRfRF*>(arg)->rx_task_loop();
    // Eternal-Task — kein vTaskDelete (wie AskSin/Moritz/SlowRf).
}

void FastRfRF::rx_task_loop() {
    task_running_ = true;
    FastRfFrame frame{};

    while (true) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0) {
            // Watchdog: a-culfw FastRF_Task:65 prueft periodisch RXFIFO_OVERFLOW.
            if (!active_) continue;
            if (marc_state(*drv_) == MARC_RXFIFO_OVERFLOW) {
                drv_->strobe(SFRX);
                drv_->strobe(SRX);
            }
            continue;
        }
        if (!active_) continue;

        // a-culfw fastrf.c:74: erst single-byte read = len, dann burst-read len.
        // PKTCTRL0=0x05 (variable mode) -> first byte ist length.
        uint8_t len = 0;
        if (drv_->read_reg(CC_RXFIFO, len) != ESP_OK) continue;
        if (len == 0 || len >= FastRfFrame::kMaxPayload) {
            // SWRS061I p.33: SFRX nur in IDLE/TXFIFO_UNDERFLOW/RXFIFO_OVERFLOW —
            // hier steht der Chip in RX, flush_rx() geht erst nach IDLE.
            drv_->flush_rx(); drv_->strobe(SRX);
            continue;
        }
        if (drv_->read_burst(CC_RXFIFO, {frame.data, len}) != ESP_OK) {
            drv_->flush_rx(); drv_->strobe(SRX);
            continue;
        }
        // ACHTUNG: hier stand "PKTCTRL1.APPEND_STATUS=1 -> 2 status bytes im FIFO".
        // Das ist falsch. PKTCTRL1 ist 0x08 = CRC_AUTOFLUSH, APPEND_STATUS ist AUS
        // (SWRS061I S.73: Bit 2 = APPEND_STATUS, Bit 3 = CRC_AUTOFLUSH) — der Chip
        // haengt keine Statusbytes an. Die beiden Reads unten holen daher NICHT die
        // Statusbytes: rssi_raw/lqi_raw sind unbrauchbar, die CRC-Auswertung
        // (lqi_crc & 0x80) traegt nicht, und steht schon ein Folge-Frame im FIFO,
        // fressen sie dessen erste zwei Bytes. Wert bleibt trotzdem 0x08, weil die
        // Registertabelle byte-genau a-culfw FASTRF_CFG entspricht (34/34) — der
        // Fehler ist von dort geerbt. Aufraeumen ist eine eigene Entscheidung,
        // siehe Followup.md. Bis dahin: den Werten hier nicht trauen.
        uint8_t rssi = 0, lqi_crc = 0;
        drv_->read_reg(CC_RXFIFO, rssi);
        drv_->read_reg(CC_RXFIFO, lqi_crc);
        frame.len      = len;
        frame.rssi_raw = rssi;
        frame.lqi_raw  = lqi_crc;

        int8_t rs = static_cast<int8_t>(rssi);
        int16_t rssi_dbm = (rs < 0) ? (rs / 2 - 74) : (rs / 2 + 64 - 74);
        if (rssi_dbm < -128) rssi_dbm = -128;
        if (rssi_dbm > 127)  rssi_dbm =  127;
        if (!(lqi_crc & 0x80)) RfStats::instance().on_crc_error(ProtocolId::FastRf);
        RfStats::instance().on_rx(ProtocolId::FastRf, len, static_cast<int8_t>(rssi_dbm));

        if (rx_cb_) {
            RxEvent ev{
                .proto        = ProtocolId::FastRf,
                .slot_id      = slot_id_,
                .rssi_dbm     = static_cast<int8_t>(rssi),
                .timestamp_us = static_cast<uint32_t>(esp_timer_get_time()),
                .payload      = &frame,
            };
            rx_cb_(ev);
        }

        // SWRS061I 19.4, S. 52: "If the radio controller is currently in transmit
        // and the SRX strobe is used, the current transmission will be ended and
        // the transition to RX will be done."  Der Rueckruf kann blockieren
        // (TCP-Schreiben bis 0,5 s, bis 0.1.893 unbegrenzt); in dieser Zeit
        // kann der Slot laengst eine andere Klasse tragen, die gerade sendet.
        // `continue`, nicht `return`: das hier ist ein Eternal-Task ohne
        // vTaskDelete — ein Ruecksprung aus rx_task_loop() waere fatal.
        if (!active_) continue;
        // a-culfw setzt MCSM1.RXOFF_MODE = RX_AFTER_RX (TXOFF=RX im 0x3F),
        // also auto-restart. Defensiv strobe SRX.
        drv_->strobe(SRX);
    }
}

esp_err_t FastRfRF::tx(const void* typed_payload, size_t payload_size) {
    if (!active_ || !typed_payload || payload_size < sizeof(FastRfTxFrame)) {
        return ESP_ERR_INVALID_ARG;
    }
    auto* f = static_cast<const FastRfTxFrame*>(typed_payload);
    if (f->len == 0 || f->len > FastRfFrame::kMaxPayload) return ESP_ERR_INVALID_ARG;

    // a-culfw fastrf_func.c:32-43 Pattern:
    //   1. write TXFIFO: length-byte + data (variable length packet)
    //   2. ccTX (SIDLE + STX-strobe-loop)
    //   3. wait while TXBYTES > 0
    //   4. ccRX (SIDLE + SRX-strobe-loop) — MCSM1 reicht laut Kommentar nicht.
    drv_->strobe(SIDLE);
    drv_->strobe(SFTX);

    uint8_t hdr[1] = {f->len};
    if (auto e = drv_->write_burst(CC_TXFIFO, {hdr, 1}); e != ESP_OK) return e;
    if (auto e = drv_->write_burst(CC_TXFIFO, {f->data, f->len}); e != ESP_OK) return e;

    uint8_t cnt = 0xFF;
    uint8_t ms = 0;
    while (cnt--) {
        drv_->strobe(STX);
        ms = marc_state(*drv_);
        if (ms == MARC_TX) break;
        ets_delay_us(10);
    }
    if (ms != MARC_TX) return ESP_ERR_TIMEOUT;

    // Wait for TX-FIFO to drain (a-culfw: while TXBYTES & 0x7F).
    for (int i = 0; i < 200; ++i) {
        uint8_t txbytes = 0xFF;
        if (drv_->read_status(REG_TXBYTES, txbytes) == ESP_OK && (txbytes & 0x7F) == 0) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Force back to RX (a-culfw fastrf.c:43 "MCSM1 does not work").
    drv_->strobe(SIDLE);
    cnt = 0xFF;
    while (cnt--) {
        drv_->strobe(SRX);
        if (marc_state(*drv_) == MARC_RX) {
            // FastRf 250 kbps -> Frame-Time = (len+1)*8/250000 s.
            // 10ms-Units: (len+1)*8 / 2500 -> meist 0 für kurze Frames.
            uint16_t airtime = (uint16_t)((((uint16_t)f->len + 1) * 8u + 2499u) / 2500u);
            RfStats::instance().on_tx(ProtocolId::FastRf, f->len, airtime);
            return ESP_OK;
        }
        ets_delay_us(10);
    }
    RfStats::instance().on_tx_error(ProtocolId::FastRf);
    return ESP_ERR_TIMEOUT;
}

esp_err_t FastRfRF::start(uint8_t slot_id, CC1101Driver& drv) {
    drv_     = &drv;
    slot_id_ = slot_id;

    drv.strobe(SIDLE);
    ets_delay_us(100);

    if (auto e = push_cfg(drv, FASTRF_CFG, sizeof(FASTRF_CFG) / 2); e != ESP_OK) {
        ESP_LOGE(TAG, "FASTRF_CFG push failed: %s", esp_err_to_name(e));
        return e;
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
        if (xTaskCreate(&FastRfRF::rx_task_trampoline, "fastrf-rx", 4096, this, 8, &h) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
        task_handle_ = h;
    }

    // IOCFG2=0x07 mappt auf den GDO2-Pin (CC1101-Reg-Layout — IOCFGn konfiguriert
    // Pin GDOn). "asserts when packet received with CRC OK, deasserts at end of
    // packet" — wir triggern auf rising edge (Frame-Start mit CRC OK angekuendigt;
    // FIFO ist bis dahin schon befuellt von der CC1101-RX-Pipeline).
    if (auto e = drv.attach_gdo2_irq(&FastRfRF::gdo_isr, this, /*rising=*/true); e != ESP_OK) {
        ESP_LOGW(TAG, "GDO2 IRQ attach: %s", esp_err_to_name(e));
    }

    active_ = true;
    ESP_LOGI(TAG, "started on slot %u (status=0x%02x)", slot_id, drv.last_status());
    return ESP_OK;
}

esp_err_t FastRfRF::stop() {
    // Eternal-Task-Pattern (analog NativeRF/SlowRfRF): Task lebt bis Reboot,
    // active_=false signalisiert deaktiviert.
    active_ = false;
    // Der Zeiger wird NICHT mehr genullt: der CC1101Driver ist ein WERT-Member
    // des Slots (cc1101_slot.hpp) und lebt, solange der Manager lebt -- er wird
    // also nie ungueltig.  Ihn zu loeschen brachte keine Sicherheit, sondern
    // verwandelte "Task fasst ein stillgelegtes Radio an" in einen Null-Zugriff:
    // der RX-Task laeuft hoeher priorisiert (8) als die Frontend-Tasks (5/6) und
    // kann in drain_one_frame() blockieren (SPI-Mutex; Renderer ->
    // TcpClientTransport::write, seit 0.1.902 ohne Warten am Socket; bis 0.1.893 unbegrenzt).
    // Kehrte er zurueck, waehrend das Frontend zwischen stop() und start() stand,
    // lief ein strobe() auf einen geloeschten Zeiger.  Gegated wird ueber das Flag.
    active_ = false;
    if (drv_) drv_->strobe(SIDLE);
    return ESP_OK;
}

} // namespace culfw32
