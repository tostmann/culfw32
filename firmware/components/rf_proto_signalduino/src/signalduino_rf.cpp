// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
//
// Contains code ported from SIGNALduino (https://github.com/RFD-FHEM/SIGNALDuino):
//   signalDecoder.h/.cpp  Pattern Decoder Library V3
//                         2014-2015 N.Butzek, S.Butzek; 2015-2018 S.Butzek;
//                         2018-2020 S.Butzek, HomeAutoUser
// GNU GPL version 3 or (at your option) any later version.
// Modified for CULFW32 (ESP-IDF port), Dirk Tostmann, 2026.

#include "signalduino_rf.hpp"
#include "rf_stats.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <algorithm>

// Diese Klasse portiert die `SignalDetectorClass` aus
// SIGNALDuino src/arduino-ide/SIGNALDuino/signalDecoder.cpp
// nach unserem RMT-Batch-Modell. Die Konstanten und Algorithmen sind
// byte-genau aus signalDecoder.h:117..125 / signalDecoder.cpp:148..1059.
//
// Unterschied zum Original: SD-Firmware decodiert pulse-by-pulse aus einem
// Interrupt-Handler, wir verarbeiten einen ganzen RMT-Batch (= ein Burst). Der
// Algorithmus ist sonst identisch:
//   1) Per-Puls findpatt() / addPattern() / addData()
//   2) Bei Long-Idle (< -maxPulse) oder Buffer-voll: processMessage()
//   3) processMessage(): compress_pattern() → getClock() → getSync()
//   4) Bei sync_found: emit MS;..;CP=..;SP=..;
//      sonst: emit MU;..;CP=..;
//
// Frame-Format byte-genau gegen Beispiele in signalDecoder.cpp:482, 715.

namespace culfw32 {

namespace {
constexpr const char* TAG = "sdrf";

// CC1101 Strobe-Adressen.
constexpr uint8_t SRX   = 0x34;
constexpr uint8_t STX   = 0x35;
constexpr uint8_t SIDLE = 0x36;
constexpr uint8_t REG_RSSI = 0x34;

// RAII fuer rx_mtx_ (wie SlowRfRF).  Null-Mutex (vor dem ersten start()) ->
// ungesperrt; dann laeuft auch noch kein RX-Task.  Reihenfolge immer
// rx_mtx_ -> SPI-Mutex (arm_rx_ strobt SRX); das Frontend fasst unter rx_mtx_
// kein SPI an.
struct RxChanLock {
    explicit RxChanLock(SemaphoreHandle_t m) : m_(m) {
        if (m_) xSemaphoreTake(m_, portMAX_DELAY);
    }
    ~RxChanLock() { if (m_) xSemaphoreGive(m_); }
    RxChanLock(const RxChanLock&) = delete;
    RxChanLock& operator=(const RxChanLock&) = delete;
    SemaphoreHandle_t m_;
};

// 868.300 MHz OOK / async serial — Reg-Tabelle byte-genau a-culfw cc1100.c:55..107
// (= unsere SLOWRF_CFG). Lokal kopiert, weil rf_proto_slowrf eine Schwester-
// Komponente ist — wir wollen kein REQUIRES dorthin (Layering).
constexpr uint8_t SD_CFG[] = {
    0x00, 0x0D, 0x01, 0x2E, 0x02, 0x2D, 0x03, 0x07, 0x04, 0xD3, 0x05, 0x91,
    0x06, 0x3D, 0x07, 0x04, 0x08, 0x32, 0x09, 0x00, 0x0A, 0x00, 0x0B, 0x06,
    0x0C, 0x00, 0x0D, 0x21, 0x0E, 0x65, 0x0F, 0x6A, 0x10, 0x57, 0x11, 0xC4,
    0x12, 0x30, 0x13, 0x23, 0x14, 0xB9, 0x15, 0x00, 0x16, 0x07, 0x17, 0x00,
    0x18, 0x18, 0x19, 0x14, 0x1A, 0x6C, 0x1B, 0x07, 0x1C, 0x00, 0x1D, 0x90,
    0x1E, 0x87, 0x1F, 0x6B, 0x20, 0xF8, 0x21, 0x56, 0x22, 0x11, 0x23, 0xE9,
    0x24, 0x2A, 0x25, 0x00, 0x26, 0x1F, 0x27, 0x41, 0x28, 0x00, 0x3E, 0xC2,
    0xFF
};

// Konstanten byte-genau signalDecoder.h:117..125
constexpr int      kMaxPatterns     = 8;       // maxNumPattern
constexpr int      kMaxDataLen      = 254;     // maxMsgSize
constexpr int      kMinMessageLen   = 40;      // minMessageLen
constexpr int32_t  kMaxPulseUs      = 32001;   // maxPulse — Frame-Boundary
constexpr int      kSyncMinFact     = 6;       // syncMinFact
constexpr int      kSyncMaxFact     = 44;      // syncMaxFact
constexpr int      kSyncMaxMicros   = 17000;   // syncMaxMicros
constexpr int      kSyncSearchLen   = 125;     // syncLenMax in getSync()

// Toleranzen.
// Real SD nutzt findpatt 20 % / compress 25 % auf einer per-pulse-IRQ-Sample-Pipeline
// mit Hardware-Edge-Sharpening (Demod liefert sauber-debouncede Edges). Unser RMT
// sieht jeden µs-genauen Edge inklusive Drift — beobachtet sind ±10 % Variation in
// realer FS20-RX-Welle (Pattern P0=423 vs P2=325 wo a-culfw "FS20 0-Bit-HIGH 400 µs"
// dasteht). Mit 25 %-Compress entgehen die uns als getrennte Slots und blockieren
// die FHEM-Demodulation. 30 % deckt die Drift gerade noch ab ohne FS20-zero (400)
// vs FS20-one (600) zu kollidieren (Diff 200, avg 500, tol 30 % = 150 < 200).
constexpr int      kFindTolPctNum   = 25;      // findpatt: ±25 % (vs SD's 20 %)
constexpr int      kCompTolPctNum   = 30;      // compress_pattern: 30 % (vs SD's 25 %)
constexpr int      kTolFloorUs      = 50;      // unter 50 µs greift der relative Faktor zu eng

// Real SD-Firmware functions.h:34: #define pulseMin 90 — Pulse < 90 µs werden
// im IRQ-Handler verworfen BEVOR sie in die FIFO landen (functions.h:62).
// Das heisst: alternation-check sieht den noise nie, und der naechste echte
// Pulse vergleicht sich gegen den letzten echten — alternation bleibt heil.
// Wir haben via RMT-Capture sub-100-µs-Glitches die wir genauso filtern muessen.
constexpr int      kMinNoisePulseUs = 90;

using Pattern = SignalduinoRF::Pattern;

// findpatt — signalDecoder.cpp:912..923
inline int findpatt(const Pattern* pats, int npat, int32_t v) {
    int abs_v = v < 0 ? -v : v;
    int tol = (abs_v * kFindTolPctNum) / 100;
    if (tol < kTolFloorUs) tol = kTolFloorUs;
    for (int i = 0; i < npat; ++i) {
        if (pats[i].value_us == 0 && pats[i].count == 0) continue;
        // Pattern muss gleichvorzeichig sein — pulse signs MUST match
        if ((pats[i].value_us < 0) != (v < 0)) continue;
        int diff = pats[i].value_us - v;
        if (diff < 0) diff = -diff;
        if (diff <= tol) return i;
    }
    return -1;
}

// updPattern — signalDecoder.cpp:152..155 (moving avg, 50/50)
inline void upd_pattern(Pattern& p, int32_t v) {
    p.value_us = (static_cast<long long>(p.value_us) + v) / 2;
    if (p.count < 0xFFFF) p.count++;
}

// compress_pattern — signalDecoder.cpp:264..315
// Reduziert die Pattern-Tabelle indem aehnliche Slots zusammengefasst werden.
// Pflicht fuer FHEM-Auto-Decode: ohne das haben wir bei realem RMT-Capture
// 7-8 Slots wo 3-4 ausreichen — die FS20-/IT-Decoder dispatchen nicht weil
// die Pattern-Werte zu fein-granular sind.
void compress_patterns(Pattern* pats, int npat, int8_t* data, int dlen) {
    for (int idx = 0; idx < npat - 1; ++idx) {
        if (pats[idx].count == 0) continue;
        for (int idx2 = idx + 1; idx2 < npat; ++idx2) {
            if (pats[idx2].count == 0) continue;
            // Gleiches Vorzeichen Pflicht — sonst keine Verschmelzung
            if ((pats[idx2].value_us ^ pats[idx].value_us) < 0) continue;

            int abs1 = pats[idx].value_us < 0 ? -pats[idx].value_us : pats[idx].value_us;
            int abs2 = pats[idx2].value_us < 0 ? -pats[idx2].value_us : pats[idx2].value_us;
            int tol = ((abs1 * kCompTolPctNum) + (abs2 * kCompTolPctNum)) / 200;  // ±25 % avg
            if (tol < kTolFloorUs) tol = kTolFloorUs;
            int diff = pats[idx].value_us - pats[idx2].value_us;
            if (diff < 0) diff = -diff;
            if (diff > tol) continue;

            // idx2 → idx mergen. Alle Daten-References auf idx2 → idx umbiegen.
            for (int i = 0; i < dlen; ++i) {
                if (data[i] == idx2) data[i] = idx;
            }
            // Gewichteter Mittelwert + Histogramm zusammen.
            int sum = pats[idx].count + pats[idx2].count;
            if (sum > 0) {
                pats[idx].value_us = (static_cast<long long>(pats[idx].value_us) * pats[idx].count
                                    + static_cast<long long>(pats[idx2].value_us) * pats[idx2].count) / sum;
            }
            pats[idx].count = static_cast<uint16_t>(sum);
            pats[idx2].value_us = 0;
            pats[idx2].count = 0;
        }
    }
}

// getClock — signalDecoder.cpp:983..1005
// Findet den positiven Pattern-Slot mit:
//   - histo[i] > 13 % der Gesamtmessages
//   - kuerzester Pulse-Laenge unter den qualifizierten
// Returns -1 wenn nichts qualifiziert.
//
// CULFW32-Anpassung: zusaetzliche Min-Pulse-Schwelle 100 µs.
// Real SD-Firmware hat sie nicht weil ihre Hardware-Demod (CC1101 GDO) keine
// Sub-100-µs-Pulse erzeugt. Wir bekommen via RMT-Capture aber Glitches < 100 µs
// (kurzes Zucken im Demod-Output bei Rauschen). Ohne Filter pickt getClock
// einen 50-µs-Glitch als Clock weil "kuerzester positiver Pulse mit hoher
// Histogramm-Frequenz". Die Schwelle 100 µs ist konservativ — die kuerzesten
// realen culfw-OOK-Symbol-Halbphasen sind FS20 zero=400 µs, IT+ ~250 µs.
constexpr int kMinClockUs = 100;
int get_clock(const Pattern* pats, int npat, int valcount) {
    int tstclock = -1;
    int threshold = (valcount * 13) / 100;     // 13 %
    for (int i = 0; i < npat; ++i) {
        if (pats[i].count == 0) continue;
        if (pats[i].value_us <= 0) continue;          // nur positive
        if (pats[i].value_us < kMinClockUs) continue; // Glitch-Filter
        if (static_cast<int>(pats[i].count) <= threshold) continue;
        if (tstclock == -1) { tstclock = i; continue; }
        if (pats[i].value_us < pats[tstclock].value_us) tstclock = i;
    }
    return tstclock;
}

// getSync — signalDecoder.cpp:1009..1059
// Findet ein negatives Pattern das die Sync-Bedingungen erfuellt UND in der
// Datensequenz mindestens einmal direkt nach einem clock-Pulse vorkommt.
// Returns Pattern-Index + setzt mstart_out auf die erste gefundene Position.
int get_sync(const Pattern* pats, int npat, int clock_idx,
             const int8_t* data, int dlen, int& mstart_out) {
    if (clock_idx < 0 || pats[clock_idx].value_us <= 0) return -1;
    int clock_us = pats[clock_idx].value_us;
    int valcount = dlen;
    int max_search = kSyncSearchLen;
    if (max_search > valcount - kMinMessageLen) max_search = valcount - kMinMessageLen;
    if (max_search <= 0) return -1;
    int rare_threshold = (valcount * 8) / 100;     // 8 %

    // Real-SD iteriert von hoechstem Index nach unten — uns egal, aber
    // konsistent zur Referenz lassen.
    for (int p = npat - 1; p >= 0; --p) {
        if (pats[p].count == 0) continue;
        if (pats[p].value_us >= 0) continue;
        int syncabs = -pats[p].value_us;
        if (syncabs >= kSyncMaxMicros) continue;
        if (syncabs / clock_us > kSyncMaxFact) continue;
        if (syncabs <= kSyncMinFact * clock_us) continue;
        if (static_cast<int>(pats[p].count) >= rare_threshold) continue;
        if (pats[p].count <= 1) continue;

        // Erste Stelle suchen wo [clock, p] direkt aufeinander folgen.
        for (int c = 0; c + 1 < max_search; ++c) {
            if (data[c] == clock_idx && data[c + 1] == p) {
                mstart_out = c;
                return p;
            }
        }
    }
    return -1;
}

} // anon namespace

// ============================================================================
// Profile / RMT setup (unveraendert)
// ============================================================================

const CC1101Profile& SignalduinoRF::profile() const {
    static const CC1101Profile p = {
        .freq_hz       = 868'300'000,
        .datarate_bps  = 5'603,
        .deviation_hz  = 0,
        .mod           = CC1101Profile::Modulation::OOK_ASK,
        .sync_word     = {0, 0, 0, 0},
        .sync_len      = 0,
        .pa_table      = {0xC2, 0, 0, 0, 0, 0, 0, 0},
        .raw_regs      = nullptr,
    };
    return p;
}

esp_err_t SignalduinoRF::setup_rmt_rx_() {
    rmt_rx_channel_config_t cfg = {};
    cfg.gpio_num     = static_cast<gpio_num_t>(drv_->pin_gdo2());
    cfg.clk_src      = RMT_CLK_SRC_DEFAULT;
    cfg.resolution_hz = 1'000'000;            // 1 µs Aufloesung
    cfg.mem_block_symbols = 48;
    cfg.flags.with_dma    = false;

    RxChanLock lk(rx_mtx_);
    // Erst lokal anlegen und aktivieren, dann veroeffentlichen (wie SlowRfRF).
    rmt_channel_handle_t chan = nullptr;
    if (auto e = rmt_new_rx_channel(&cfg, &chan); e != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_rx_channel: %s", esp_err_to_name(e));
        return e;
    }
    rmt_rx_event_callbacks_t cbs = { .on_recv_done = &SignalduinoRF::rmt_rx_done_cb };
    if (auto e = rmt_rx_register_event_callbacks(chan, &cbs, this); e != ESP_OK) {
        ESP_LOGE(TAG, "rmt_rx_register_callbacks: %s", esp_err_to_name(e));
        rmt_del_channel(chan);
        return e;
    }
    if (auto e = rmt_enable(chan); e != ESP_OK) {
        ESP_LOGE(TAG, "rmt_rx enable: %s", esp_err_to_name(e));
        rmt_del_channel(chan);
        return e;
    }
    uint32_t gen = rx_gen_.load(std::memory_order_relaxed) + 1;
    if (gen == 0) gen = 1;                    // 0 heisst "nur Wecken"
    rx_gen_.store(gen, std::memory_order_relaxed);
    rmt_rx_chan_ = chan;
    return ESP_OK;
}

esp_err_t SignalduinoRF::teardown_rmt_rx_() {
    RxChanLock lk(rx_mtx_);
    if (rmt_rx_chan_) {
        rmt_disable(rmt_rx_chan_);
        rmt_del_channel(rmt_rx_chan_);
        rmt_rx_chan_ = nullptr;
    }
    return ESP_OK;
}

// Einziger Aufrufer von rmt_receive() ist der RX-Task (Begruendung bei
// SlowRfRF::arm_rx_: bis 0.1.910 armierte tx() selbst, der Task armierte
// danach ein zweites Mal -> "channel not in enable state").
esp_err_t SignalduinoRF::arm_rx_(uint32_t& armed_gen) {
    RxChanLock lk(rx_mtx_);
    if (!enabled_ || !rmt_rx_chan_) return ESP_ERR_NOT_FOUND;   // TX/stop() laeuft
    const uint32_t gen = rx_gen_.load(std::memory_order_relaxed);
    if (armed_gen == gen) return ESP_OK;                        // laeuft schon
    drv_->strobe(SRX);
    rmt_receive_config_t rxc = {};
    rxc.signal_range_min_ns = 1'000;
    rxc.signal_range_max_ns = 32 * 1000'000;     // 32 ms idle = Frame-Ende-Trigger
    esp_err_t e = rmt_receive(rmt_rx_chan_, rx_buffer_, sizeof(rx_buffer_), &rxc);
    if (e == ESP_OK) armed_gen = gen;
    return e;
}

void SignalduinoRF::wake_rx_task_() {
    RxBatch wake{};                           // gen 0: kein Empfang, nur Wecken
    xQueueSend(rx_queue_, &wake, 0);
}

// Nach dem Senden: Kanal neu anlegen, der RX-Task strobt SRX und armiert.
void SignalduinoRF::resume_rx_() {
    if (setup_rmt_rx_() == ESP_OK) wake_rx_task_();
}

bool IRAM_ATTR SignalduinoRF::rmt_rx_done_cb(rmt_channel_handle_t,
                                              const rmt_rx_done_event_data_t* edata,
                                              void* user) {
    auto* self = static_cast<SignalduinoRF*>(user);
    if (!self || !self->rx_queue_) return false;
    BaseType_t hpw = pdFALSE;
    RxBatch batch{ .n_syms = edata->num_symbols,
                   .gen    = self->rx_gen_.load(std::memory_order_relaxed) };
    xQueueSendFromISR(self->rx_queue_, &batch, &hpw);
    return hpw == pdTRUE;
}

void SignalduinoRF::rx_task_trampoline(void* arg) {
    static_cast<SignalduinoRF*>(arg)->rx_task_loop();
    // Eternal-Task-Pattern — kein vTaskDelete, Task lebt bis zum Reboot.
}

void SignalduinoRF::rx_task_loop() {
    // Eternal-Task-Pattern (siehe SlowRfRF / AskSinRF / MoritzRF / HmDualRF):
    // Loop laeuft bis zum Reboot. enabled_==false signalisiert
    // "deaktiviert" (Slot-Transition lief auf andere RF-Klasse) — Task pollt
    // dann leer weiter. So vermeiden wir vQueueDelete waehrend Task in
    // xQueueReceive blockiert (Load Fault in xTaskRemoveFromEventList — gleicher
    // Bug-Class wie SlowRf Build 0.1.770).
    task_running_ = true;
    // Generation des Kanals, auf dem unser rmt_receive() laeuft; 0 = keiner.
    uint32_t armed_gen = 0;

    while (true) {
        RxBatch batch{};
        bool got = xQueueReceive(rx_queue_, &batch, pdMS_TO_TICKS(1000)) == pdTRUE;

        if (!enabled_) {
            armed_gen = 0;
            continue;
        }

        // RX-done (gen != 0; gen 0 ist der Weck-Batch von start()/tx()).
        if (got && batch.gen != 0) {
            process_symbols_(rx_buffer_, batch.n_syms);
            if (batch.gen == armed_gen) armed_gen = 0;   // Empfang ist zu Ende
        }

        if (armed_gen != rx_gen_.load(std::memory_order_relaxed)) {
            if (auto e = arm_rx_(armed_gen); e != ESP_OK && e != ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG, "rmt_receive arm: %s", esp_err_to_name(e));
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
}

// ============================================================================
// process_symbols_ — Per-Pulse-Decode + processMessage am Frame-Ende.
// Mirrored auf SignalDetectorClass::decode() + processMessage() byte-genau.
// ============================================================================

void SignalduinoRF::process_symbols_(const rmt_symbol_word_t* syms, size_t n) {
    if (n < 2 || !rx_cb_) return;

    Pattern pats[kMaxPatterns] = {};
    int     npat = 0;
    int8_t  data[kMaxDataLen + 4];     // Pattern-Indices (-1 = invalid)
    int     dlen = 0;
    int32_t prev_pulse = 0;            // last-Pulse-Vergleich fuer sign-alternation

    auto add_pulse = [&](int32_t v) {
        // Validity check byte-genau signalDecoder.cpp:163..165:
        //   valid = (valcount==0 || (*first ^ *last) < 0)   // alternierendes Vorzeichen
        // Bei !valid ruft real SD processMessage() + reset() — dann faellt
        // der pulse durch zum addPattern in fresh frame. Mirror genauso.
        if (dlen > 0 && prev_pulse != 0) {
            bool same_sign = (prev_pulse < 0) == (v < 0);
            if (same_sign) {
                // Frame-Boundary durch alternation-mismatch.
                if (dlen >= kMinMessageLen) {
                    finalize_frame_(pats, npat, data, dlen);
                }
                for (auto& p : pats) p = {};
                npat = 0; dlen = 0; prev_pulse = 0;
            }
        }

        int idx = findpatt(pats, npat, v);
        if (idx >= 0) {
            upd_pattern(pats[idx], v);
        } else {
            if (npat >= kMaxPatterns) {
                // Pattern-Tabelle voll. Real-SD evict via Histogramm; wir
                // simplify: Pulse droppen, Daten-Rest weiter sammeln.
                // Compress_pattern wird das beim Output ggf. aufraeumen.
                return;
            }
            pats[npat] = { v, 1 };
            idx = npat++;
        }
        if (dlen < kMaxDataLen) {
            data[dlen++] = static_cast<int8_t>(idx);
            prev_pulse = v;
        }
    };

    for (size_t i = 0; i < n; ++i) {
        uint32_t d0 = syms[i].duration0, l0 = syms[i].level0;
        uint32_t d1 = syms[i].duration1, l1 = syms[i].level1;

        auto handle_pulse = [&](uint32_t dur, uint32_t lvl) {
            if (dur == 0) return true;                            // RMT padding
            if (dur < kMinNoisePulseUs) return true;              // < 90 µs = real SD pulseMin trash
            int32_t v = (lvl == 1) ? static_cast<int32_t>(dur)
                                   : -static_cast<int32_t>(dur);
            // Frame-Boundary byte-genau signalDecoder.cpp:165: *first > -maxPulse
            // → bei *first <= -maxPulse: invalid → processMessage + reset.
            if (v <= -kMaxPulseUs) {
                // Output current frame (if any), reset for next.
                if (dlen > 0) {
                    finalize_frame_(pats, npat, data, dlen);
                    npat = 0; dlen = 0; prev_pulse = 0;
                    for (auto& p : pats) p = {};
                }
                return true;
            }
            add_pulse(v);
            return true;
        };

        handle_pulse(d0, l0);
        if (d1 != 0) handle_pulse(d1, l1);
    }

    // Letzter Frame nach Schleifenende.
    if (dlen >= kMinMessageLen) {
        finalize_frame_(pats, npat, data, dlen);
    }
}

// processMessage byte-genau signalDecoder.cpp:318..520 (MS-Pfad) bzw.
// 706..731 (MU-Pfad). Wir reduzieren auf: compress, getClock, getSync,
// emit MS oder MU. MC (Manchester) lassen wir erstmal weg.
void SignalduinoRF::finalize_frame_(Pattern* pats, int npat,
                                     int8_t* data, int dlen) {
    if (dlen < kMinMessageLen) return;

    // 1. compress_pattern
    compress_patterns(pats, npat, data, dlen);

    // 2. getClock
    int clock_idx = get_clock(pats, npat, dlen);
    if (clock_idx < 0) return;        // Kein Clock-Pulse identifizierbar — drop

    // 3. getSync — wenn gefunden: MS, sonst MU
    int mstart = 0;
    int sync_idx = get_sync(pats, npat, clock_idx, data, dlen, mstart);

    // 4. RSSI-Snapshot
    uint8_t rssi_raw = 0;
    if (drv_) drv_->read_status(REG_RSSI, rssi_raw);
    int8_t raw_signed = static_cast<int8_t>(rssi_raw);
    int rssi_dbm = (raw_signed < 0) ? (raw_signed / 2 - 74) : (raw_signed / 2 + 64 - 74);
    if (rssi_dbm < -128) rssi_dbm = -128;
    if (rssi_dbm > 127)  rssi_dbm =  127;

    // 5. Output rendern in frame_.ascii (Class-Member, kein Stack-Frame).
    char* w = frame_.ascii;
    int   cap = sizeof(frame_.ascii);
    int   used = 0;

    if (sync_idx >= 0) {
        // mend bestimmen: ab mstart so weit wie moeglich, aber bei naechstem
        // [clock, sync]-Paar abschneiden (separate Repeats trennen) — byte-genau
        // signalDecoder.cpp:363..376.
        int mend = mstart + 2;
        bool syncend = false;
        while (mend < dlen - 1) {
            if (!syncend && data[mend + 1] != sync_idx) {
                syncend = true;
            } else if (syncend && data[mend + 1] == sync_idx && data[mend] == clock_idx) {
                mend -= 1;
                break;
            }
            mend += 2;
        }
        if (mend > dlen - 1) mend = dlen - 1;
        if (mend - mstart < kMinMessageLen) {
            // Zu wenig zwischen den Syncs — fallback auf MU
            sync_idx = -1;
        } else {
            used += snprintf(w + used, cap - used, "MS;");
            for (int idx = 0; idx < npat && used < cap - 32; ++idx) {
                if (pats[idx].count == 0 && pats[idx].value_us == 0) continue;
                used += snprintf(w + used, cap - used, "P%d=%d;", idx,
                                 static_cast<int>(pats[idx].value_us));
            }
            if (used < cap - 8) {
                used += snprintf(w + used, cap - used, "D=");
                int copy_n = mend - mstart + 1;
                if (copy_n > cap - used - 32) copy_n = cap - used - 32;
                for (int i = 0; i < copy_n; ++i) {
                    w[used++] = static_cast<char>('0' + data[mstart + i]);
                }
            }
            used += snprintf(w + used, cap - used, ";CP=%d;SP=%d;R=%d;",
                             clock_idx, sync_idx,
                             rssi_dbm < 0 ? -rssi_dbm : rssi_dbm);
        }
    }

    if (sync_idx < 0) {
        // MU-Pfad
        used += snprintf(w + used, cap - used, "MU;");
        for (int idx = 0; idx < npat && used < cap - 32; ++idx) {
            if (pats[idx].count == 0 && pats[idx].value_us == 0) continue;
            used += snprintf(w + used, cap - used, "P%d=%d;", idx,
                             static_cast<int>(pats[idx].value_us));
        }
        if (used < cap - 8) {
            used += snprintf(w + used, cap - used, "D=");
            int copy_n = dlen;
            if (copy_n > cap - used - 24) copy_n = cap - used - 24;
            for (int i = 0; i < copy_n; ++i) {
                w[used++] = static_cast<char>('0' + data[i]);
            }
        }
        used += snprintf(w + used, cap - used, ";CP=%d;R=%d;",
                         clock_idx, rssi_dbm < 0 ? -rssi_dbm : rssi_dbm);
    }

    if (used >= cap) used = cap - 1;
    frame_.ascii[used] = '\0';
    frame_.len = static_cast<uint16_t>(used);
    frame_.rssi_dbm = static_cast<int8_t>(rssi_dbm);

    RfStats::instance().on_rx(ProtocolId::PulseCapture, dlen / 2,
                              static_cast<int8_t>(rssi_dbm));

    RxEvent ev{
        .proto        = ProtocolId::PulseCapture,
        .slot_id      = slot_id_,
        .rssi_dbm     = static_cast<int8_t>(rssi_dbm),
        .timestamp_us = static_cast<uint32_t>(esp_timer_get_time()),
        .payload      = &frame_,
    };
    rx_cb_(ev);
}

// ============================================================================
// start / stop — wie SlowRfRF
// ============================================================================

esp_err_t SignalduinoRF::start(uint8_t slot_id, CC1101Driver& drv) {
    drv_     = &drv;
    slot_id_ = slot_id;

    drv.strobe(SIDLE);
    ets_delay_us(100);

    for (size_t i = 0; SD_CFG[i] != 0xFF && SD_CFG[i] <= 0x40; i += 2) {
        if (auto e = drv.write_reg(SD_CFG[i], SD_CFG[i + 1]); e != ESP_OK) {
            ESP_LOGE(TAG, "write_reg(0x%02x): %s", SD_CFG[i], esp_err_to_name(e));
            return e;
        }
    }
    // PATABLE + FREND0 aus dem EEPROM (a-culfw ccInitChip) — Begruendung und
    // Messung siehe SlowRfRF::start(). Fallback ohne Provider: 0 dBm, kein Ramping.
    {
        uint8_t pa[8]  = {0x00, 0x50, 0, 0, 0, 0, 0, 0};
        uint8_t frend0 = 0x11;
        size_t  n      = 2;
        if (pa_provider_ && pa_provider_(std::span<uint8_t, 8>(pa), frend0)) n = 8;
        drv.write_burst(0x3E, {pa, n});
        drv.write_reg(0x22, frend0);
    }

    // SCAL mit Lock-Pruefung (SWRS061I §22.1 p.58: FSCAL1 != 0x3F == Lock).
    if (drv.calibrate() != ESP_OK) {
        ESP_LOGE(TAG, "PLL ohne Lock — RX/TX wird ausfallen");
    }

    static constexpr uint8_t REWRITE_REGS[][2] = {
        {0x0D, 0x21}, {0x0E, 0x65}, {0x0F, 0x6A},
        {0x08, 0x32},
        {0x10, 0x57}, {0x11, 0xC4}, {0x12, 0x30},
        {0x18, 0x18},
    };
    for (auto& rr : REWRITE_REGS) {
        drv.write_reg(rr[0], rr[1]);
    }

    if (!rx_queue_) rx_queue_ = xQueueCreate(4, sizeof(RxBatch));
    if (!rx_queue_) return ESP_ERR_NO_MEM;
    if (!rx_mtx_) rx_mtx_ = xSemaphoreCreateMutex();
    if (!rx_mtx_) return ESP_ERR_NO_MEM;

    if (auto e = setup_rmt_rx_(); e != ESP_OK) return e;

    // Flag VOR dem Wecken/Erzeugen des Tasks.  Er laeuft mit Prio 7, die
    // Frontend-Tasks mit 5/6 -- xQueueSend bzw. xTaskCreate verdraengt also
    // sofort.  Stand das Flag erst danach, fiel der Task durchs Gate, legte
    // sich fuer den 1000-ms-Timeout von xQueueReceive hin, und da `start()`
    // selbst kein SRX strobt (das tut nur arm_rx_() aus dem Task), war der
    // Empfaenger bis zu einer Sekunde taub -- MARCSTATE blieb 0x01.  Gefunden
    // in der Gegenpruefung 07.09.2026; `drv_` und rmt_rx_chan_ stehen hier
    // bereits, die Invariante Flag => Zeiger gueltig bleibt gewahrt.
    enabled_ = true;

    if (!task_handle_) {
        TaskHandle_t h = nullptr;
        if (xTaskCreate(&SignalduinoRF::rx_task_trampoline, "sd-rx",
                        4096, this, 7, &h) != pdPASS) {
            // Der Slot geht auf Error, ohne stop() zu rufen -- Kanal sonst verwaist.
            enabled_ = false;
            teardown_rmt_rx_();
            return ESP_ERR_NO_MEM;
        }
        task_handle_ = h;
    }

    // Weck-Batch fuer BEIDE Pfade, nicht nur fuer den schon lebenden Task.
    // Die Schleife beginnt mit xQueueReceive ueber 1000 ms und strobt SRX erst
    // danach (arm_rx_); ein frisch erzeugter Task fand ohne diesen Batch
    // nichts vor und liess den Empfaenger die erste Sekunde nach dem Boot
    // blind.  Gemessen 07.09.2026: erster Start 1010 ms, danach 61 ms.
    wake_rx_task_();

    ESP_LOGI(TAG, "started on slot %u (gdo2=%d)", slot_id, drv.pin_gdo2());

    return ESP_OK;
}

// ============================================================================
// TX-Pfad — `SR;...`-Befehl byte-genau Real-SD-Firmware send.h:17..43.
//
// Pro D-Char wird ein Halbpuls geschaltet:
//   index = '0'..'7' → buckets[index]
//   isLow = (buckets[index] < 0)
//   dur   = |buckets[index]|
// Wir bauen RMT-Symbole, jedes packt 2 Halbpulse — ungerade D-Laengen lassen
// das letzte Symbol mit dur1=0 (einphasig).
//
// CC1101 in async-serial-mode (PKT_FORMAT=3): GDO0 ist TX-Daten-EINGANG, der
// Chip moduliert die OOK-Welle entsprechend. RMT-TX bindet daher GDO0 (nicht
// GDO2 — das ist RX).
// ============================================================================

esp_err_t SignalduinoRF::setup_rmt_tx_() {
    rmt_tx_channel_config_t cfg = {};
    cfg.gpio_num         = static_cast<gpio_num_t>(drv_->pin_gdo0());
    cfg.clk_src          = RMT_CLK_SRC_DEFAULT;
    cfg.resolution_hz    = 1'000'000;
    cfg.mem_block_symbols = 48;
    cfg.trans_queue_depth = 2;
    cfg.flags.with_dma   = false;
    if (auto e = rmt_new_tx_channel(&cfg, &rmt_tx_chan_); e != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel: %s", esp_err_to_name(e));
        return e;
    }
    rmt_copy_encoder_config_t ecfg = {};
    if (auto e = rmt_new_copy_encoder(&ecfg, &rmt_tx_enc_); e != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_copy_encoder: %s", esp_err_to_name(e));
        return e;
    }
    return rmt_enable(rmt_tx_chan_);
}

esp_err_t SignalduinoRF::teardown_rmt_tx_() {
    if (rmt_tx_enc_)  { rmt_del_encoder(rmt_tx_enc_); rmt_tx_enc_ = nullptr; }
    if (rmt_tx_chan_) {
        rmt_disable(rmt_tx_chan_);
        rmt_del_channel(rmt_tx_chan_);
        rmt_tx_chan_ = nullptr;
    }
    return ESP_OK;
}

esp_err_t SignalduinoRF::tx(const void* typed_payload, size_t payload_size) {
    if (!enabled_ || !typed_payload || payload_size < sizeof(SignalduinoTxFrame)) {
        return ESP_ERR_INVALID_ARG;
    }
    auto* f = static_cast<const SignalduinoTxFrame*>(typed_payload);
    if (f->repeats == 0 || f->repeats > 32) return ESP_ERR_INVALID_ARG;
    if (f->dlen == 0 || f->dlen > sizeof(f->data)) return ESP_ERR_INVALID_ARG;
    if (f->npat == 0 || f->npat > 8) return ESP_ERR_INVALID_ARG;

    // RX pausieren — RMT-Pin-Konflikt vermeiden (GDO0 vs GDO2 sind verschieden,
    // aber wir teardown beide Channels um sauberen TX-Slot zu kriegen).
    // resume_rx_() legt den Kanal danach in jedem Fall neu an (heilt ein
    // fehlgeschlagenes RX-Setup beim naechsten Sendevorgang, wie SlowRfRF).
    teardown_rmt_rx_();
    drv_->strobe(SIDLE);
    ets_delay_us(200);

    if (auto e = setup_rmt_tx_(); e != ESP_OK) {
        teardown_rmt_tx_();                   // halb angelegten TX-Kanal abraeumen
        resume_rx_();
        return e;
    }

    // Symbol-Buffer aus D-string + buckets bauen. Jeder D-Char produziert
    // einen Halbpuls (Vorzeichen aus buckets, Dauer = abs). Zwei aufeinander
    // folgende Halbpulse passen in ein rmt_symbol_word_t. Bei ungerader
    // D-Laenge bleibt der letzte Halbpuls als d0 ohne d1.
    size_t out_n = 0;
    for (size_t i = 0; i < f->dlen && out_n < kTxBufferSyms; ) {
        unsigned idx0 = f->data[i] - '0';
        if (idx0 >= 8) { i++; continue; }
        int16_t v0 = f->buckets[idx0];
        uint16_t d0 = static_cast<uint16_t>(v0 < 0 ? -v0 : v0);
        // RMT duration ist 15-bit (0..32767 µs). Lange Pausen clippen wir; das
        // SR-Format kann lange negative Werte haben (z.B. -10000 fuer Sync), das
        // passt noch rein. Sehr lange Pausen > 32 ms muessten in mehrere RMT-
        // Symbole gesplittet werden — fuer FS20/IT-typische Werte tritt das nicht auf.
        if (d0 > 32767) d0 = 32767;
        uint8_t l0 = (v0 < 0) ? 0 : 1;
        uint16_t d1 = 0;
        uint8_t  l1 = 0;
        i++;
        if (i < f->dlen) {
            unsigned idx1 = f->data[i] - '0';
            if (idx1 < 8) {
                int16_t v1 = f->buckets[idx1];
                d1 = static_cast<uint16_t>(v1 < 0 ? -v1 : v1);
                if (d1 > 32767) d1 = 32767;
                l1 = (v1 < 0) ? 0 : 1;
                i++;
            }
        }
        tx_buffer_[out_n].duration0 = d0;
        tx_buffer_[out_n].level0    = l0;
        tx_buffer_[out_n].duration1 = d1;
        tx_buffer_[out_n].level1    = l1;
        out_n++;
    }
    if (out_n == 0) {
        teardown_rmt_tx_();
        resume_rx_();
        return ESP_ERR_NO_MEM;
    }

    rmt_transmit_config_t tcfg = {};
    tcfg.loop_count = 0;
    for (uint8_t r = 0; r < f->repeats; ++r) {
        // Pro Repeat STX strobe + 500 µs settle (wie SlowRfRF::tx). Ohne den
        // strobe faellt der CC1101 nach dem ersten Burst zurueck in IDLE
        // (MCSM1 TXOFF=IDLE) und der zweite rmt_transmit feuert ins Leere.
        drv_->strobe(STX);
        ets_delay_us(500);

        if (auto e = rmt_transmit(rmt_tx_chan_, rmt_tx_enc_, tx_buffer_,
                                  out_n * sizeof(rmt_symbol_word_t), &tcfg);
            e != ESP_OK) {
            ESP_LOGE(TAG, "rmt_transmit %u: %s", r, esp_err_to_name(e));
            break;
        }
        rmt_tx_wait_all_done(rmt_tx_chan_, 1000);
        if (r + 1 < f->repeats) vTaskDelay(pdMS_TO_TICKS(10));
    }

    drv_->strobe(SIDLE);
    teardown_rmt_tx_();

    resume_rx_();

    // Stats — Bytes-Aequivalent ist hier dlen/2 (≈ Bits), Airtime in 10 ms-Units
    // berechnen wir aus der Summe der bucket-abs-Dauern, gewichtet mit repeats.
    uint32_t airtime_us = 0;
    for (size_t i = 0; i < f->dlen; ++i) {
        unsigned idx = f->data[i] - '0';
        if (idx < 8) {
            int16_t v = f->buckets[idx];
            airtime_us += (v < 0 ? -v : v);
        }
    }
    airtime_us *= f->repeats;
    RfStats::instance().on_tx(ProtocolId::PulseCapture, f->dlen / 2,
                              airtime_us / 10000);
    return ESP_OK;
}

esp_err_t SignalduinoRF::stop() {
    // Eternal-Task-Pattern — siehe rx_task_loop. Task + Queue bleiben persistent,
    // nur RMT-Channels werden teardown'd. Fix Build 0.1.771 (gleicher Bug-Class
    // wie SlowRf): vQueueDelete waehrend Task in xQueueReceive blockiert ->
    // Load Fault in xTaskRemoveFromEventList.
    teardown_rmt_rx_();
    teardown_rmt_tx_();
    // Der Zeiger wird NICHT mehr genullt: der CC1101Driver ist ein WERT-Member
    // des Slots (cc1101_slot.hpp) und lebt, solange der Manager lebt -- er wird
    // also nie ungueltig.  Ihn zu loeschen brachte keine Sicherheit, sondern
    // verwandelte "Task fasst ein stillgelegtes Radio an" in einen Null-Zugriff:
    // der RX-Task laeuft hoeher priorisiert (7) als die Frontend-Tasks (5/6) und
    // kann bei der Ausgabe eines Rahmens blockieren (SPI-Mutex; Renderer ->
    // TcpClientTransport::write, seit 0.1.902 ohne Warten am Socket; bis 0.1.893 unbegrenzt).
    // Kehrte er zurueck, waehrend das Frontend zwischen stop() und start() stand,
    // lief ein strobe() auf einen geloeschten Zeiger.  Gegated wird ueber das Flag.
    enabled_ = false;
    if (drv_) drv_->strobe(SIDLE);
    return ESP_OK;
}

} // namespace culfw32
