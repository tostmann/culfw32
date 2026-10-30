// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "native_rf.hpp"
#include "native_cfg.hpp"
#include "rf_stats.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace culfw32 {

namespace {
constexpr const char* TAG = "native";

constexpr uint8_t CC_RXFIFO  = 0x3F;
constexpr uint8_t REG_MARC   = 0x35;
constexpr uint8_t REG_RXBYTES= 0x3B;

constexpr uint8_t SRX   = 0x34;
constexpr uint8_t SIDLE = 0x36;
constexpr uint8_t SFRX  = 0x3A;
constexpr uint8_t SNOP  = 0x3D;

constexpr uint8_t MARC_IDLE              = 0x01;
constexpr uint8_t MARC_RX                = 0x0D;
constexpr uint8_t MARC_RXFIFO_OVERFLOW   = 0x11;

uint8_t marc_state(CC1101Driver& d) {
    uint8_t v = 0xFF;
    d.read_status(REG_MARC, v);
    return v & 0x1F;
}

// Konfig-Tabelle in den CC1101 schreiben — Sentinel-terminiert (addr > 0x40).
esp_err_t push_cfg(CC1101Driver& drv, const uint8_t* cfg, size_t max_pairs) {
    for (size_t i = 0; i < max_pairs * 2; i += 2) {
        uint8_t addr = cfg[i];
        if (addr > 0x40) break;
        if (auto e = drv.write_reg(addr, cfg[i + 1]); e != ESP_OK) return e;
    }
    return ESP_OK;
}
}

const CC1101Profile& NativeRF::profile() const {
    // Profile ist nur deklarativ — die echten Reg-Werte kommen aus NATIVE_CFG +
    // MODE_CFG. Hier reportet das Profile die "typische" Konfig fuer Mode 1.
    static const CC1101Profile p = {
        .freq_hz        = 868'299'820,
        .datarate_bps   = 17'241,
        .deviation_hz   = 19'042,
        .mod            = CC1101Profile::Modulation::FSK_2,
        .sync_word      = {0x2D, 0xD4, 0, 0},
        .sync_len       = 16,
        .pa_table       = {0xC3, 0, 0, 0, 0, 0, 0, 0},
        .raw_regs       = nullptr,
    };
    return p;
}

void IRAM_ATTR NativeRF::gdo2_isr(void* arg) {
    auto* self = static_cast<NativeRF*>(arg);
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(self->task_handle_), &hpw);
    portYIELD_FROM_ISR(hpw);
}

void NativeRF::rx_task_trampoline(void* arg) {
    static_cast<NativeRF*>(arg)->rx_task_loop();
    // Eternal-Task — kein vTaskDelete (wie AskSin/Moritz/SlowRf).
}

void NativeRF::rx_task_loop() {
    task_running_ = true;
    NativeFrame frame{};

    while (true) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0) {
            // Watchdog auf RXFIFO_OVERFLOW / IDLE — a-culfw native_task:226-238.
            if (!active_) continue;
            uint8_t ms = marc_state(*drv_);
            if (ms == MARC_RXFIFO_OVERFLOW || ms == MARC_IDLE) {
                drv_->strobe(SFRX);
                drv_->strobe(SIDLE);
                drv_->strobe(SNOP);
                drv_->strobe(SRX);
            }
            continue;
        }
        if (!active_) continue;

        // a-culfw Pattern (rf_native.c:182): SIDLE first, dann RXBYTES lesen.
        drv_->strobe(SIDLE);

        uint8_t rxbytes = 0;
        drv_->read_status(REG_RXBYTES, rxbytes);
        uint8_t len = rxbytes & 0x7F;
        if (len == 0 || len >= NativeFrame::kMaxPayload) {
            drv_->strobe(SFRX);
            drv_->strobe(SRX);
            continue;
        }

        if (drv_->read_burst(CC_RXFIFO, {frame.data, len}) != ESP_OK) {
            drv_->strobe(SFRX); drv_->strobe(SRX);
            continue;
        }
        frame.mode     = mode_;
        frame.len      = len;
        frame.rssi_raw = 0;       // PKTCTRL1=0x00 -> kein RSSI/LQI append in PKTCTRL1
        frame.lqi_raw  = 0;

        RfStats::instance().on_rx(ProtocolId::Native, len, 0);

        if (rx_cb_) {
            RxEvent ev{
                .proto        = ProtocolId::Native,
                .slot_id      = slot_id_,
                .rssi_dbm     = 0,
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
        // Re-arm for next frame: flush + restart RX.
        drv_->strobe(SFRX);
        drv_->strobe(SRX);
    }
}

esp_err_t NativeRF::start(uint8_t slot_id, CC1101Driver& drv) {
    drv_     = &drv;
    slot_id_ = slot_id;

    drv.strobe(SIDLE);
    ets_delay_us(100);

    // Default Cfg (NATIVE_CFG) zuerst.
    if (auto e = push_cfg(drv, NATIVE_CFG, sizeof(NATIVE_CFG) / 2); e != ESP_OK) {
        ESP_LOGE(TAG, "NATIVE_CFG push failed: %s", esp_err_to_name(e));
        return e;
    }

    // Mode-spezifischer Overlay.
    const uint8_t* mode_cfg = nullptr;
    size_t         mode_cfg_pairs = 0;
    switch (mode_) {
        case 1: mode_cfg = NATIVE_MODE1_CFG; mode_cfg_pairs = sizeof(NATIVE_MODE1_CFG) / 2; break;
        case 2: mode_cfg = NATIVE_MODE2_CFG; mode_cfg_pairs = sizeof(NATIVE_MODE2_CFG) / 2; break;
        case 3: mode_cfg = NATIVE_MODE3_CFG; mode_cfg_pairs = sizeof(NATIVE_MODE3_CFG) / 2; break;
        default: return ESP_ERR_INVALID_ARG;
    }
    if (auto e = push_cfg(drv, mode_cfg, mode_cfg_pairs); e != ESP_OK) {
        ESP_LOGE(TAG, "MODE%u_CFG push failed: %s", mode_, esp_err_to_name(e));
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
        if (xTaskCreate(&NativeRF::rx_task_trampoline, "native-rx", 4096, this, 8, &h) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
        task_handle_ = h;
    }

    if (auto e = drv.attach_gdo2_irq(&NativeRF::gdo2_isr, this, /*rising=*/true); e != ESP_OK) {
        ESP_LOGW(TAG, "GDO2 IRQ attach: %s", esp_err_to_name(e));
    }

    active_ = true;
    ESP_LOGI(TAG, "started on slot %u, mode %u (status=0x%02x)",
             slot_id, mode_, drv.last_status());
    return ESP_OK;
}

esp_err_t NativeRF::stop() {
    // Eternal-Task-Pattern: Task lebt bis Reboot, active_=false signalisiert
    // deaktiviert. task_handle_ + task_running_ bleiben unangetastet, sonst
    // werden Restart-Aufrufe und ggf. zwischendurch feuernde GDO2-IRQs auf
    // stale TCB-Pointer korrupt (siehe asksin/moritz/slowrf-Pattern).
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
