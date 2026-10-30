// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "freq_cal.hpp"
#include "nvs_eeprom.hpp"
#include "culfw_ee_addr.hpp"
#include "esp_efuse.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cstring>

namespace culfw32::freqcal {

static const char* TAG = "freqcal";

const char* source_name(Source s) {
    switch (s) {
        case Source::Efuse: return "efuse";
        case Source::Nvs:   return "nvs";
        default:            return "none";
    }
}

bool efuse_read(int8_t& out) {
    uint8_t blk[32] = {};
    // BLK3 = EFUSE_BLK_USER_DATA. Wird vom Produktionstest in EINEM Vorgang
    // gebrannt (RS-Coding, in der Fertigung); ein leerer Block
    // liest sich als lauter Nullen.
    if (esp_efuse_read_block(EFUSE_BLK_USER_DATA, blk, 0, sizeof(blk) * 8) != ESP_OK) {
        return false;
    }
    if (blk[0] != 'B' || blk[1] != 'W') return false;      // unprovisioniert
    if (blk[2] < 2)                     return false;      // Layout v1: kein RF-Cal-Feld
    if (blk[kBlk3MarkerOff] != kBlk3RfCalMarker) return false;
    out = static_cast<int8_t>(blk[kBlk3ValueOff]);
    return true;
}

bool nvs_read(NvsEeprom& ee, int8_t& out) {
    if (ee.read_byte(ee::RF_FREQOFF_VALID) != ee::RF_FREQOFF_MAGIC) return false;
    out = static_cast<int8_t>(ee.read_byte(ee::RF_FREQOFF));
    return true;
}

void nvs_write(NvsEeprom& ee, int8_t value) {
    ee.write_byte(ee::RF_FREQOFF, static_cast<uint8_t>(value));
    ee.write_byte(ee::RF_FREQOFF_VALID, ee::RF_FREQOFF_MAGIC);
}

void nvs_clear(NvsEeprom& ee) {
    ee.write_byte(ee::RF_FREQOFF_VALID, 0x00);
    ee.write_byte(ee::RF_FREQOFF, 0x00);
}

int8_t effective(NvsEeprom* ee, Source& src) {
    int8_t v = 0;
    if (ee && nvs_read(*ee, v)) { src = Source::Nvs;   return v; }
    if (efuse_read(v))          { src = Source::Efuse; return v; }
    src = Source::None;
    return 0;
}

} // namespace culfw32::freqcal

// ---------------------------------------------------------------------------
// Suchlauf (siehe Header).
// ---------------------------------------------------------------------------

namespace culfw32::freqcal {

namespace {

ScanState        g_scan{};
portMUX_TYPE     g_scan_mux = portMUX_INITIALIZER_UNLOCKED;
CC1101Driver*    g_scan_drv = nullptr;
uint32_t         g_scan_dwell_ms = 1200;

// Schrittraster: 8 LSB = 12,7 kHz. Bei 101 kHz RX-Bandbreite ueberlappen die
// Fenster deutlich, ein Traeger wird also auf mehreren Kandidaten gesehen —
// gewollt, der Gewinner ist dann der mit den meisten Treffern.
constexpr int8_t kSteps[] = {
    0, -8, 8, -16, 16, -24, 24, -32, 32, -40, 40, -48, 48, -56, 56, -64, 64,
};
// RSSI-Rohwert -> dBm (SWRS061I §17.3, offset 74 dB bei 868 MHz).
int8_t rssi_dbm_from_raw(uint8_t raw) {
    int v = static_cast<int8_t>(raw);
    return static_cast<int8_t>((v / 2) - 74);
}

constexpr int8_t kCarrierThresholdDbm = -90;   // darueber gilt "da ist was"

void scan_task(void*) {
    CC1101Driver& drv = *g_scan_drv;
    const int8_t base = drv.freq_offset();
    drv.clear_scan_abort();
    bool abgebrochen = false;

    // Kriterium ist die Zahl DEKODIERTER Frames je Kandidat, nicht die Zahl
    // gesehener Traeger. Ein starker Sender wird wegen der 101-kHz-Bandbreite
    // in fast jedem Fenster als Traeger gesehen — die Trefferzahl darauf sagt
    // ueber die Frequenzmitte fast nichts (drei Laeufe am Bench: -33, -29, -10
    // LSB, davon einer am Rand des nutzbaren Fensters). Ob ein Frame durch
    // Sync und CRC kommt, sagt es dagegen sehr genau: die FSK-RX-Pfade zaehlen
    // jeden dekodierten Frame (note_rx_frame), auch wenn kein FREQEST-Wert dazu
    // vorliegt (MAX!: nur waehrend des Pakets gelesen).
    int8_t best_step = base, best_median = 0;
    size_t best_n = 0;
    int8_t strongest = -128;
    // Fallback, falls NIRGENDS ein Frame dekodiert wird (keine Gegenstelle in
    // Reichweite, oder Offset groesser als das Suchraster): dann wenigstens den
    // Kandidaten mit dem staerksten Traeger melden, damit die Ausgabe zwischen
    // "keine Quelle" und "Quelle da, aber nichts dekodierbar" unterscheidet.
    int8_t carrier_step = base;
    size_t carrier_hits = 0;

    for (int8_t step : kSteps) {
        if (drv.scan_abort_requested()) { abgebrochen = true; break; }
        int sum = base + step;
        if (sum < -128 || sum > 127) continue;
        drv.set_freq_offset(static_cast<int8_t>(sum));
        drv.apply_freq_offset();
        drv.freqest_reset();
        // Der Empfaenger braucht einen Moment, bis die neue Frequenz steht.
        vTaskDelay(pdMS_TO_TICKS(30));

        size_t hits = 0;
        TickType_t until = xTaskGetTickCount() + pdMS_TO_TICKS(g_scan_dwell_ms);
        while (xTaskGetTickCount() < until) {
            uint8_t rssi_raw = 0;
            if (drv.read_status(0x34 /* RSSI */, rssi_raw) == ESP_OK) {
                int8_t dbm = rssi_dbm_from_raw(rssi_raw);
                if (dbm > strongest) strongest = dbm;
                if (dbm > kCarrierThresholdDbm) ++hits;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            if (drv.scan_abort_requested()) { abgebrochen = true; break; }
        }
        if (abgebrochen) break;
        if (hits > carrier_hits) { carrier_hits = hits; carrier_step = static_cast<int8_t>(sum); }

        int8_t med = 0;
        size_t samples = 0;
        const bool have_med = drv.freqest_median(med, samples);
        const size_t frames = drv.rx_frames();
        if (frames > best_n) {
            best_n      = frames;
            best_median = have_med ? med : 0;
            best_step   = static_cast<int8_t>(sum);
        }
    }
    if (best_n == 0) {
        best_step   = carrier_step;   // nur Traeger gesehen, nichts dekodiert
        best_median = 0;
    }

    // Ausgangswert wiederherstellen; uebernommen wird nur auf Ansage.  Beim
    // Abbruch ist das der eigentliche Zweck: die neue RF-Klasse soll nicht mit
    // einem gesweepten Offset weiterlaufen.
    drv.set_freq_offset(base);
    drv.apply_freq_offset();
    drv.clear_scan_abort();
    if (abgebrochen) {
        ESP_LOGW(TAG, "freq scan abgebrochen — RF-Klasse gewechselt, Offset auf %d zurueck",
                 static_cast<int>(base));
        portENTER_CRITICAL(&g_scan_mux);
        g_scan.running  = false;
        g_scan.finished = true;
        g_scan.ok       = false;
        portEXIT_CRITICAL(&g_scan_mux);
        g_scan_drv = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    int proposal = best_step + best_median;
    if (proposal < -128) proposal = -128;
    if (proposal > 127)  proposal = 127;

    portENTER_CRITICAL(&g_scan_mux);
    g_scan.running   = false;
    g_scan.finished  = true;
    g_scan.ok        = (best_n >= 2);   // mind. 2 dekodierte Frames im besten Fenster
    g_scan.result    = static_cast<int8_t>(proposal);
    g_scan.best_step = best_step;
    g_scan.median    = best_median;
    g_scan.samples   = best_n;
    g_scan.rssi_dbm  = strongest;
    portEXIT_CRITICAL(&g_scan_mux);

    g_scan_drv = nullptr;
    vTaskDelete(nullptr);
}

} // namespace

esp_err_t scan_start(CC1101Driver& drv, uint32_t dwell_ms) {
    portENTER_CRITICAL(&g_scan_mux);
    bool busy = g_scan.running;
    if (!busy) {
        g_scan = ScanState{};
        g_scan.running = true;
    }
    portEXIT_CRITICAL(&g_scan_mux);
    if (busy) return ESP_ERR_INVALID_STATE;

    g_scan_drv      = &drv;
    g_scan_dwell_ms = dwell_ms;
    if (xTaskCreate(scan_task, "freqcal", 3072, nullptr, 4, nullptr) != pdPASS) {
        portENTER_CRITICAL(&g_scan_mux);
        g_scan.running = false;
        portEXIT_CRITICAL(&g_scan_mux);
        g_scan_drv = nullptr;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "freq scan started (dwell %u ms)", static_cast<unsigned>(dwell_ms));
    return ESP_OK;
}

ScanState scan_state() {
    portENTER_CRITICAL(&g_scan_mux);
    ScanState s = g_scan;
    portEXIT_CRITICAL(&g_scan_mux);
    return s;
}

} // namespace culfw32::freqcal
