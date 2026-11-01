// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 Dirk Tostmann
//
// Contains code ported from a-culfw (https://github.com/tostmann/a-culfw):
//   clib/rf_receive.c         Copyright by R.Koenig, Copyright 2015 B. Hempel,
//                             inspired by code from Dirk Tostmann
//   clib/rf_receive_bucket.c  Copyright (C) 2015 B. Hempel
// Both under the GNU GPL version 2 or (at your option) any later version.
// The HMS frame check (analyze_hms) is written anew from the frame format.
// Modified for CULFW32 (ESP-IDF port), Dirk Tostmann, 2026.

#include "slowrf_rf.hpp"
#include "slowrf_cfg.hpp"
#include "rf_stats.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include <cstring>
#include <algorithm>

namespace culfw32 {

namespace {
constexpr const char* TAG = "slowrf";

// CC1101-Strobe-Adressen.
constexpr uint8_t SRX   = 0x34;
constexpr uint8_t STX   = 0x35;
constexpr uint8_t SIDLE = 0x36;
constexpr uint8_t SFRX  = 0x3A;
constexpr uint8_t SFTX  = 0x3B;

constexpr uint8_t REG_RSSI = 0x34;   // status reg

// a-culfw rf_receive_bucket.h: TSCALE(x) = x/16. Wir teilen RMT-Roh-µs durch 16
// um auf das identische uint8_t-Zeitraster der a-culfw-Bucket-Logik zu kommen.
// TDIFF=12 (=200µs Toleranz), SILENCE=4000µs Idle-Threshold (=Frame-Ende).
constexpr uint16_t TSCALE_DIV = 16;
constexpr uint8_t  TDIFF      = 12;
constexpr uint32_t SILENCE_US = 4000;
constexpr uint32_t IDLE_THRESHOLD_US = 5000;     // RMT-Idle (>SILENCE)

// RAII fuer rx_mtx_.  Null-Mutex (vor dem ersten start()) -> ungesperrt; dann
// laeuft auch noch kein RX-Task.  Reihenfolge immer rx_mtx_ -> SPI-Mutex
// (arm_rx_ strobt SRX); das Frontend fasst unter rx_mtx_ kein SPI an.
struct RxChanLock {
    explicit RxChanLock(SemaphoreHandle_t m) : m_(m) {
        if (m_) xSemaphoreTake(m_, portMAX_DELAY);
    }
    ~RxChanLock() { if (m_) xSemaphoreGive(m_); }
    RxChanLock(const RxChanLock&) = delete;
    RxChanLock& operator=(const RxChanLock&) = delete;
    SemaphoreHandle_t m_;
};

// Maximaler Pulse-Counter (8-bit). Längere Pulse als 4080µs werden geclippt
// — passieren in unserer 868-MHz-Familie nicht im Mittel-Frame.
constexpr uint16_t TSCALE_CLIP = 0xFF;

inline uint8_t tscale(uint32_t us) {
    uint32_t v = us / TSCALE_DIV;
    return v > TSCALE_CLIP ? TSCALE_CLIP : static_cast<uint8_t>(v);
}

// makeavg aus a-culfw rf_receive_bucket.c:8 — gleitender Mittelwert.
inline uint8_t makeavg(uint8_t i, uint8_t j) {
    return (3 * (uint16_t)i + (uint16_t)j) >> 2;
}

// ============================================================================
// Bucket-State-Machine — Subset aus a-culfw rf_receive.c.
// 868-MHz-Branch only (FS20/FHT/EM/KS300/HMS). Manchester/IT/TCM/REVOLT/ESA gestrichen.
// ============================================================================

constexpr uint8_t STATE_RESET   = 0;
constexpr uint8_t STATE_INIT    = 1;
constexpr uint8_t STATE_SYNC    = 2;
constexpr uint8_t STATE_COLLECT = 3;
constexpr uint8_t STATE_HMS     = 4;   // a-culfw STATE_HMS: Flanken statt Wellen
constexpr uint8_t STATE_DONE    = 5;   // Frame fertig, weitere Waves ignorieren

// HMS (a-culfw rf_receive.c:828 + :1099; Rahmenpruefung: analyze_hms). Nach
// mindestens 12 gleichen Sync-Wellen mit mehr als 1600 µs Periode zaehlt
// a-culfw einzelne Flanken: frueher als TSCALE(750) nach der letzten gezaehlten
// wird eine Flanke uebergangen, spaeter als TSCALE(1250) bricht der Empfang ab;
// fallend = 1, steigend = 0.  Das Ende setzt der SILENCE-Timer, gemessen ab der
// letzten gezaehlten Flanke.
constexpr uint8_t HMS_SYNC_MIN = 12;
constexpr uint8_t HMS_EDGE_MIN = 750 / TSCALE_DIV;    // 46
constexpr uint8_t HMS_EDGE_MAX = 1250 / TSCALE_DIV;   // 78
constexpr uint8_t HMS_BITS     = 69;   // 6 × (8 Daten + Paritaet + Stopp) + 8 + Paritaet

constexpr uint8_t TYPE_FS20  = 'F';
constexpr uint8_t TYPE_FHT   = 'T';
constexpr uint8_t TYPE_EM    = 'E';
constexpr uint8_t TYPE_KS300 = 'K';

constexpr size_t MAXMSG = 12;        // a-culfw EM-/FHT-Größe

struct wave_t { uint8_t hightime, lowtime; };

struct bucket_t {
    uint8_t state, byteidx, sync, bitidx;
    uint8_t data[MAXMSG];
    uint8_t valCount;
    wave_t  zero, one, two;
    // KS300-Last-Bit-Patch (a-culfw rf_receive.c:434): letzter beobachteter
    // hightime des invalid-wave, der COLLECT terminierte. KS300-Frames enden mit
    // einem nicht-paaren high-Pulse (kein nachfolgender low fuer das letzte Bit),
    // identify_frame hat damit ein Bit zu wenig — wir holen es aus dem letzten
    // wave-htime auf b->one.lowtime-Match nach.
    uint8_t last_htime;
    // HMS: µs seit der letzten gezaehlten Flanke (in a-culfw TCNT1).
    uint32_t hms_us;
};

inline bool wave_equals(const wave_t& a, uint8_t htime, uint8_t ltime) {
    int16_t dlow      = (int16_t)a.lowtime  - ltime;
    int16_t dhigh     = (int16_t)a.hightime - htime;
    int16_t dcomplete = (int16_t)(a.lowtime + a.hightime) - (int16_t)(ltime + htime);
    return dlow > -TDIFF && dlow < TDIFF
        && dhigh > -TDIFF && dhigh < TDIFF
        && dcomplete > -TDIFF && dcomplete < TDIFF;
}

inline void addbit(bucket_t* b, uint8_t bit) {
    if (b->byteidx >= MAXMSG) return;
    if (bit) b->data[b->byteidx] |= (1u << b->bitidx);
    if (b->bitidx-- == 0) {
        b->bitidx = 7;
        if (++b->byteidx < MAXMSG) b->data[b->byteidx] = 0;
    }
}

inline void delbit(bucket_t* b) {
    if (b->bitidx++ == 7) {
        b->bitidx = 0;
        if (b->byteidx > 0) b->byteidx--;
    }
}

// Edge-Verarbeitung — a-culfw CC1100_INTVECT, 868-MHz-Branch.
// Aufgerufen pro hightime/lowtime-Paar (= 1 wave).
static void feed_wave(bucket_t* b, uint8_t htime, uint8_t ltime) {
    switch (b->state) {
        case STATE_RESET: {
            // Erste Welle: Referenz für 0-Bit.
            if (htime > tscale(1600) || ltime > tscale(1600)) return;  // Junk
            b->zero.hightime = htime;
            b->zero.lowtime  = ltime;
            b->one  = {0, 0};
            b->two  = {0, 0};
            b->byteidx = 0; b->bitidx = 7; b->data[0] = 0;
            b->sync = 1;
            b->valCount = 0;
            b->state = STATE_SYNC;
            break;
        }
        case STATE_SYNC: {
            if (wave_equals(b->zero, htime, ltime)) {
                b->zero.hightime = makeavg(b->zero.hightime, htime);
                b->zero.lowtime  = makeavg(b->zero.lowtime,  ltime);
                b->sync++;
            } else if (b->sync >= 4) {
                // Wave davon abweichend → 1-Bit-Referenz, Übergang COLLECT —
                // bzw. HMS nach langem, langsamem Sync (a-culfw prueft das vor
                // COLLECT).  Ab dieser steigenden Flanke laeuft die HMS-Uhr.
                b->one.hightime = htime;
                b->one.lowtime  = ltime;
                b->byteidx = 0; b->bitidx = 7; b->data[0] = 0;
                if (b->sync >= HMS_SYNC_MIN &&
                    b->zero.hightime + b->zero.lowtime > tscale(1600)) {
                    b->hms_us = 0;
                    b->state  = STATE_HMS;
                } else {
                    b->state = STATE_COLLECT;
                }
            } else {
                // Zu wenig syncs → reset, retry mit dieser Welle als neue zero-Ref.
                b->state = STATE_RESET;
                feed_wave(b, htime, ltime);
            }
            break;
        }
        case STATE_COLLECT: {
            if (wave_equals(b->one, htime, ltime)) {
                addbit(b, 1);
                b->one.hightime = makeavg(b->one.hightime, htime);
                b->one.lowtime  = makeavg(b->one.lowtime,  ltime);
            } else if (wave_equals(b->zero, htime, ltime)) {
                addbit(b, 0);
                b->zero.hightime = makeavg(b->zero.hightime, htime);
                b->zero.lowtime  = makeavg(b->zero.lowtime,  ltime);
            } else {
                // Ungültiges Wave während COLLECT → Frame-Ende. a-culfw faengt
                // das ueber den SILENCE-Timer ab, der das Bucket finalisiert
                // BEVOR Junk reinkommt. Wir haben keinen Inter-Wave-Timer (nur
                // RMT-idle pro Batch), also muessen wir hier die Collection
                // einfrieren statt das schon validierte Frame wegzuwerfen.
                // Letzten htime fuer KS300-Last-Bit-Patch merken.
                b->last_htime = htime;
                b->state = STATE_DONE;
            }
            break;
        }
        case STATE_DONE:
            // Frame ist fertig — weitere Waves im selben Batch ignorieren.
            break;
        default:
            b->state = STATE_RESET;
            break;
    }
}

enum class HmsEdge : uint8_t { Ignored, Counted, Aborted, Silence };

// Eine Flanke im HMS-Zustand; `dur_us` = Dauer des Pegels davor, `falling` =
// der Pegel war high.  Silence: der SILENCE-Timer waere vor dieser Flanke
// abgelaufen, der Rahmen ist davor zu Ende.
static HmsEdge feed_hms_edge(bucket_t* b, uint32_t dur_us, bool falling) {
    b->hms_us += dur_us;
    if (b->hms_us >= SILENCE_US) return HmsEdge::Silence;
    uint32_t c = b->hms_us / TSCALE_DIV;
    if (c < HMS_EDGE_MIN) return HmsEdge::Ignored;
    if (c > HMS_EDGE_MAX) return HmsEdge::Aborted;
    addbit(b, falling ? 1 : 0);
    b->hms_us = 0;
    return HmsEdge::Counted;
}

// ============================================================================
// Decoder-Layer: parity-stripping + cksum-Validierung pro Frame-Typ.
// Aus a-culfw rf_receive.c:217 (analyze) + Cksum-Helfer.
// ============================================================================

// Even-parity-bit (a-culfw util/parity.h). Branchless, 32→1 reduction.
inline uint8_t parity_even_bit(uint8_t v) {
    v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;
    return v & 1;
}

inline uint8_t cksum1(uint8_t s, const uint8_t* buf, uint8_t len) {
    while (len) s += buf[--len];
    return s;
}
inline uint8_t cksum2(const uint8_t* buf, uint8_t len) {
    uint8_t s = 0; while (len) s ^= buf[--len]; return s;
}
inline uint8_t cksum3(const uint8_t* buf, uint8_t len, bool& nibble_out) {
    uint8_t x = 0, y = 5, cnt = 0;
    bool nibble = false;
    while (len) {
        uint8_t d = buf[--len];
        x ^= (d >> 4);
        y += (d >> 4);
        if (!nibble || cnt) {
            x ^= (d & 0xf);
            y += (d & 0xf);
        }
        cnt++;
    }
    y += x;
    nibble_out = nibble;
    return (uint8_t)((y << 4) | x);
}

// Port a-culfw analyze() — extrahiert obuf aus bucket b, abhängig vom Type.
// Returns true wenn obuf+oby gefüllt wurden; obuf hat MAXMSG Bytes Reserve.
static bool analyze(const bucket_t* b, uint8_t t, uint8_t* obuf, uint8_t* oby_out,
                    bool* nibble_out) {
    uint8_t i = 0, cnt = 0, max, iby = 0;
    int8_t  ibi = 7, obi = 7;
    bool    nibble = false;

    max = (uint8_t)(b->byteidx * 8u + (7u - b->bitidx));
    obuf[0] = 0;

    while (cnt++ < max) {
        uint8_t bit = (b->data[iby] & (1u << ibi)) ? 1 : 0;
        if (ibi-- == 0) { iby++; ibi = 7; }

        if (t == TYPE_KS300 && obi == 3) {
            if (!nibble) {
                if (!bit) return false;
                nibble = true;
                continue;
            }
            nibble = false;
        }

        if (obi == -1) {
            if (t == TYPE_FS20) {
                if (parity_even_bit(obuf[i]) != bit) return false;
            }
            if (t == TYPE_EM || t == TYPE_KS300) {
                if (!bit) return false;
            }
            if (++i >= MAXMSG) return false;
            obuf[i] = 0;
            obi = 7;
        } else {
            if (bit) {
                if (t == TYPE_FS20)
                    obuf[i] |= (1u << obi);
                if (t == TYPE_EM || t == TYPE_KS300)
                    obuf[i] |= (1u << (7 - obi));
            }
            obi--;
        }
    }

    if (cnt <= max) return false;
    if (t == TYPE_EM && obi == -1) i++;
    else if (nibble)               i++;

    *oby_out    = i;
    *nibble_out = nibble;
    return i != 0;
}

// Frame-Identifikation: probiert FS20/FHT/EM/KS300, return SlowRfFrame oder nichts.
static bool identify_frame(const bucket_t* b, SlowRfFrame& out) {
    uint8_t obuf[MAXMSG] = {};
    uint8_t oby = 0;
    bool    nibble = false;

    // FS20 (Even-Parity, cksum1 startwert=6) ODER FHT (cksum1 startwert=12).
    if (analyze(b, TYPE_FS20, obuf, &oby, &nibble)) {
        if (oby >= 5) {
            uint8_t fs_ck = cksum1(6, obuf, oby - 1);
            if (fs_ck == obuf[oby - 1]) {
                out.kind = SlowRfFrame::Kind::FS20;
                out.len  = oby - 1;
                std::memcpy(out.data, obuf, out.len);
                return true;
            }
            if (fs_ck + 1 == obuf[oby - 1]) {
                out.kind = SlowRfFrame::Kind::FS20;
                out.len  = oby - 1;
                std::memcpy(out.data, obuf, out.len);
                out.data[oby - 1] = fs_ck;   // Repeater-Indikator unterdrücken
                return true;
            }
            uint8_t ft_ck = cksum1(12, obuf, oby - 1);
            if (ft_ck == obuf[oby - 1]) {
                out.kind = SlowRfFrame::Kind::FHT;
                out.len  = oby - 1;
                std::memcpy(out.data, obuf, out.len);
                return true;
            }
        }
    }

    // EM (always 9 bytes payload + 1 ck = 10 → oby=10 vor Strip).
    // a-culfw rf_receive.c:425 versucht EM-analyze direkt; CULFW32 reproduziert
    // das, plus zusätzlich eine Last-Bit-Variante analog KS300 (same observation:
    // RMT-Capture trimmt manchmal das letzte stopbit des Frames bevor SILENCE-
    // Detect das Bucket finalisiert). Wir versuchen beide Varianten — die erste
    // matchende gewinnt.
    // EM-RX zwei-Wege-Versuch: einmal direkt (a-culfw rf_receive.c:425), dann
    // mit Last-Bit-Patch analog KS300. Bei Lab-Bedingungen (permanenter Lab-
    // Beacon F66666600) konnten wir EM-RX nicht empirisch gegen Lokales
    // verifizieren — die EM-Frames kollidieren mit dem Beacon. Code-Pfad
    // strukturell korrekt (siehe slowrf_status.md), Verifikation braucht
    // sauberes EM-Test-Setup ohne Lab-Beacon.
    auto try_em = [&](const bucket_t* bk) -> bool {
        std::memset(obuf, 0, sizeof(obuf));
        oby = 0; nibble = false;
        if (!analyze(bk, TYPE_EM, obuf, &oby, &nibble) || oby < 1) return false;
        uint8_t payload_len = oby - 1;
        if (payload_len == 9 && cksum2(obuf, payload_len) == obuf[payload_len]) {
            out.kind = SlowRfFrame::Kind::EM;
            out.len  = payload_len;
            std::memcpy(out.data, obuf, out.len);
            return true;
        }
        return false;
    };
    if (try_em(b)) return true;
    if (b->last_htime != 0) {
        bucket_t bc = *b;
        uint8_t bit = wave_equals(bc.one, bc.last_htime, bc.one.lowtime) ? 1u : 0u;
        addbit(&bc, bit);
        if (try_em(&bc)) return true;
    }

    // KS300 — Last-Bit-Patch byte-genau a-culfw rf_receive.c:432-441:
    //   addbit(b, wave_equals(&b->one, hightime, b->one.lowtime, b->state));
    //   if (analyze(...TYPE_KS300...)) cksum3-check; if not, delbit.
    // Wir mutieren nicht den Caller-Bucket; arbeiten auf einer Kopie.
    {
        bucket_t bc = *b;
        if (bc.last_htime != 0) {
            // wave_equals auf nur htime+ltime: ltime wird mit b->one.lowtime
            // verglichen — das ist die a-culfw-Logik. last_htime entstammt dem
            // ungueltigen wave, der COLLECT terminierte.
            uint8_t bit = wave_equals(bc.one, bc.last_htime, bc.one.lowtime) ? 1u : 0u;
            addbit(&bc, bit);
        }
        std::memset(obuf, 0, sizeof(obuf));
        oby = 0;
        nibble = false;
        if (analyze(&bc, TYPE_KS300, obuf, &oby, &nibble) && oby >= 2) {
            uint8_t payload_len = oby - 1;
            bool dummy = false;
            // a-culfw-Quirk: bei nibble-half wird der cksum-Read auf [oby-1] (mit
            // -nibble) beruecksichtigt — siehe rf_receive.c:437.
            uint8_t ck = cksum3(obuf, payload_len - (nibble ? 1u : 0u), dummy);
            if (ck == obuf[payload_len]) {
                out.kind = SlowRfFrame::Kind::KS300;
                out.len  = payload_len;
                std::memcpy(out.data, obuf, out.len);
                out.nibble_half = nibble;
                return true;
            }
        }
    }

    return false;
}

// HMS (HMS100TF und Verwandte, nur Empfang): sechs Datenbytes, jedes mit acht
// Datenbits (das niederwertigste zuerst), einem Paritaetsbit fuer gerade
// Paritaet und einem Stoppbit 0; danach die Pruefsumme (XOR der sechs Bytes)
// mit acht Bits und Paritaetsbit -- zusammen HMS_BITS. feed_hms_edge legt die
// Bits hoechstwertig zuerst im Bucket ab (fallende Flanke = 1); Bits nach dem
// letzten Pruefsummenbit bleiben unbeachtet.
static bool analyze_hms(const bucket_t* b, SlowRfFrame& out) {
    constexpr uint8_t kBytes = 6;
    if (b->byteidx * 8 + (7 - b->bitidx) < HMS_BITS) return false;

    unsigned pos = 0;
    auto next_bit = [&]() -> uint8_t {
        const uint8_t v = static_cast<uint8_t>((b->data[pos >> 3] >> (7 - (pos & 7))) & 1u);
        ++pos;
        return v;
    };
    // Acht Bits, niederwertigstes zuerst, und das folgende Paritaetsbit.
    auto read_byte = [&](uint8_t& v) -> bool {
        v = 0;
        for (uint8_t i = 0; i < 8; ++i) v = static_cast<uint8_t>(v | (next_bit() << i));
        return next_bit() == parity_even_bit(v);
    };

    uint8_t sum = 0;
    for (uint8_t i = 0; i < kBytes; ++i) {
        if (!read_byte(out.data[i]) || next_bit() != 0) return false;
        sum ^= out.data[i];
    }
    uint8_t ck = 0;
    if (!read_byte(ck) || ck != sum) return false;

    out.kind = SlowRfFrame::Kind::HMS;
    out.len  = kBytes;
    return true;
}

} // anon namespace

// ============================================================================
// Profile / start / stop
// ============================================================================

const CC1101Profile& SlowRfRF::profile() const {
    static const CC1101Profile p = {
        .freq_hz        = 868'300'000,
        .datarate_bps   = 5'603,
        .deviation_hz   = 0,
        .mod            = CC1101Profile::Modulation::OOK_ASK,
        .sync_word      = {0, 0, 0, 0},
        .sync_len       = 0,
        .pa_table       = {0xC2, 0, 0, 0, 0, 0, 0, 0},
        .raw_regs       = nullptr,
    };
    return p;
}

esp_err_t SlowRfRF::setup_rmt_rx_() {
    rmt_rx_channel_config_t cfg = {};
    // CC1101 async serial mode (PKT_FORMAT=3): IOCFG2=0x0D treibt die demodulierten
    // RX-Bits auf GDO2. GDO0 = TX-Daten-EINGANG vom MCU (IOCFG0=0x2D = HW0, chip
    // gibt den Pin frei). Daher: RX bindet RMT an GDO2, TX (siehe setup_rmt_tx_)
    // bindet RMT an GDO0.
    cfg.gpio_num     = static_cast<gpio_num_t>(drv_->pin_gdo2());
    cfg.clk_src      = RMT_CLK_SRC_DEFAULT;
    cfg.resolution_hz = 1'000'000;            // 1 MHz → 1 µs Auflösung
    cfg.mem_block_symbols = 48;               // ESP32-C6 ohne DMA: max = SOC_RMT_MEM_WORDS_PER_CHANNEL
    cfg.flags.with_dma    = false;

    RxChanLock lk(rx_mtx_);
    // Erst lokal anlegen und aktivieren, dann veroeffentlichen: der RX-Task
    // sieht nie einen angelegten, aber noch nicht aktivierten Kanal, und ein
    // Fehlschlag hinterlaesst keinen halben Kanal in rmt_rx_chan_.
    rmt_channel_handle_t chan = nullptr;
    if (auto e = rmt_new_rx_channel(&cfg, &chan); e != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_rx_channel: %s", esp_err_to_name(e));
        return e;
    }

    rmt_rx_event_callbacks_t cbs = { .on_recv_done = &SlowRfRF::rmt_rx_done_cb };
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

esp_err_t SlowRfRF::teardown_rmt_rx_() {
    RxChanLock lk(rx_mtx_);
    if (rmt_rx_chan_) {
        rmt_disable(rmt_rx_chan_);
        rmt_del_channel(rmt_rx_chan_);
        rmt_rx_chan_ = nullptr;
    }
    return ESP_OK;
}

// Armiert den Empfang auf dem aktuellen Kanal.  Einziger Aufrufer von
// rmt_receive() ist der RX-Task; start() und tx() legen den Kanal nur an und
// wecken ihn.  Bis 0.1.910 armierte tx() nach dem Senden selbst, waehrend der
// Task seinen Stand in einem eigenen `armed` fuehrte.  Wachte er waehrend des
// Sendens auf (Kanal weg -> armed=false), rief er danach bei jedem Timeout ein
// zweites rmt_receive() auf den laufenden Empfang: IDF "channel not in enable
// state", im 1010-ms-Takt bis zum naechsten RX-done (Dudette-Bench 28.10.2026).
esp_err_t SlowRfRF::arm_rx_(uint32_t& armed_gen) {
    RxChanLock lk(rx_mtx_);
    if (!enabled_ || !rmt_rx_chan_) return ESP_ERR_NOT_FOUND;   // TX/stop() laeuft
    const uint32_t gen = rx_gen_.load(std::memory_order_relaxed);
    if (armed_gen == gen) return ESP_OK;                        // laeuft schon
    drv_->strobe(SRX);
    rmt_receive_config_t rxc = {};
    // ESP32-C6: signal_range_min_ns laeuft mit dem APB-Clock-Filter (~80 MHz),
    // nicht mit der Kanal-Resolution. Das Filter-Register ist 8-bit; 50 µs
    // wuerde > 255 ticks erzeugen → INVALID_ARG. 1 µs entspricht 80 ticks.
    rxc.signal_range_min_ns = 1'000;
    rxc.signal_range_max_ns = IDLE_THRESHOLD_US * 1000;
    esp_err_t e = rmt_receive(rmt_rx_chan_, rx_buffer_, sizeof(rx_buffer_), &rxc);
    if (e == ESP_OK) armed_gen = gen;
    return e;
}

void SlowRfRF::wake_rx_task_() {
    RxBatch wake{};                           // gen 0: kein Empfang, nur Wecken
    xQueueSend(rx_queue_, &wake, 0);
}

// Nach dem Senden: Kanal neu anlegen und den RX-Task wecken, der SRX strobt
// und armiert.  Er laeuft hoeher priorisiert als die Frontends und verdraengt
// sie beim xQueueSend sofort -- die Empfangsluecke bleibt wie bisher kurz.
void SlowRfRF::resume_rx_() {
    if (setup_rmt_rx_() == ESP_OK) wake_rx_task_();
}

bool IRAM_ATTR SlowRfRF::rmt_rx_done_cb(rmt_channel_handle_t,
                                        const rmt_rx_done_event_data_t* edata,
                                        void* user) {
    auto* self = static_cast<SlowRfRF*>(user);
    if (!self || !self->rx_queue_) return false;

    // Symbole liegen im rx_buffer_ — wir geben Anzahl an den Decoder-Task weiter.
    // Buffer wird vom Task konsumiert, dann re-armiert (kein Race weil RMT erst
    // nach explizitem rmt_receive() wieder schreibt, und das ruft nur der Task).
    BaseType_t hpw = pdFALSE;
    RxBatch batch{ .n_syms = edata->num_symbols,
                   .gen    = self->rx_gen_.load(std::memory_order_relaxed) };
    xQueueSendFromISR(self->rx_queue_, &batch, &hpw);
    return hpw == pdTRUE;
}

void SlowRfRF::rx_task_trampoline(void* arg) {
    static_cast<SlowRfRF*>(arg)->rx_task_loop();
    // Eternal-Task-Pattern — kein vTaskDelete, Task lebt bis zum Reboot.
}

void SlowRfRF::rx_task_loop() {
    task_running_ = true;
    // Generation des Kanals, auf dem unser rmt_receive() laeuft; 0 = keiner.
    uint32_t armed_gen = 0;

    while (true) {
        RxBatch batch{};
        bool got = xQueueReceive(rx_queue_, &batch, pdMS_TO_TICKS(1000)) == pdTRUE;

        // Wenn deaktiviert (Slot-Transition lief auf andere RF-Klasse) — leer
        // pollen.  Den Kanal prueft arm_rx_() unter rx_mtx_.
        if (!enabled_) {
            armed_gen = 0;
            continue;
        }

        // RX-done (gen != 0; gen 0 ist der Weck-Batch von start()/tx()).  Der
        // Puffer bleibt bis zum naechsten rmt_receive() unberuehrt, und das
        // ruft nur dieser Task -- also erst nach dem Dekodieren.
        if (got && batch.gen != 0) {
            process_symbols_(rx_buffer_, batch.n_syms);
            if (batch.gen == armed_gen) armed_gen = 0;   // Empfang ist zu Ende
        }

        // (Re-)arm, wenn kein Empfang laeuft oder tx()/start() den Kanal neu
        // angelegt haben.  CC1101 braucht keine SIDLE/SRX-Reset — async serial
        // liefert permanent demoduliertes Signal solange MARCSTATE=RX bleibt.
        if (armed_gen != rx_gen_.load(std::memory_order_relaxed)) {
            if (auto e = arm_rx_(armed_gen); e != ESP_OK && e != ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG, "rmt_receive arm: %s", esp_err_to_name(e));
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
}

void SlowRfRF::process_symbols_(const rmt_symbol_word_t* syms, size_t n) {
    // Eine RMT-Burst-Sequenz = ein bucket. Wir verarbeiten alle Symbole, dann
    // analyze() am Ende.  Ausnahme HMS: dort beendet wie in a-culfw schon der
    // SILENCE-Timer den Rahmen, ein weiterer kann im selben Batch folgen.
    // Umgekehrt nicht: nach COLLECT steht der Bucket auf STATE_DONE und der
    // Rest des Batches verfaellt -- auch ein HMS-Rahmen, der weniger als die
    // RMT-Idle-Zeit nach einem FS20-/FHT-/EM-/KS300-Rahmen kommt.  a-culfw
    // nimmt ihn an (SILENCE-Timer nach 4000 µs, frischer Bucket).
    bucket_t b{};
    b.state = STATE_RESET;

    auto finish = [this](const bucket_t& bk) {
        SlowRfFrame frame{};
        bool ok = false;
        if (bk.state == STATE_HMS)
            ok = analyze_hms(&bk, frame);
        else if ((bk.state == STATE_COLLECT || bk.state == STATE_DONE) && bk.byteidx >= 2)
            ok = identify_frame(&bk, frame);
        if (!ok) return;
        uint8_t rssi = 0;
        if (drv_) drv_->read_status(REG_RSSI, rssi);
        frame.rssi_raw = rssi;
        emit_frame_(frame);
    };

    for (size_t i = 0; i < n; ++i) {
        // Symbol = {duration0:level0, duration1:level1}. Bei OOK-Bursts liefert
        // CC1101 GDO0 = 1 wenn modulator-output active. duration0 = high-time
        // wenn level0=1, sonst low-time.
        uint16_t d0 = syms[i].duration0;
        uint8_t  l0 = syms[i].level0;
        uint16_t d1 = syms[i].duration1;
        uint8_t  l1 = syms[i].level1;

        if (b.state == STATE_HMS) {
            // Jeder Pegel endet mit einer Flanke, Dauer 0 = keine Flanke (Ende
            // der Aufnahme).  Nach Abbruch oder Rahmenende verfaellt der Rest
            // des Symbols, das naechste beginnt die Sync-Suche neu.
            const uint16_t dur[2] = {d0, d1};
            const uint8_t  lvl[2] = {l0, l1};
            for (int k = 0; k < 2 && b.state == STATE_HMS && dur[k]; ++k) {
                switch (feed_hms_edge(&b, dur[k], lvl[k] == 1)) {
                    case HmsEdge::Silence: finish(b); b = bucket_t{}; break;
                    case HmsEdge::Aborted: b = bucket_t{}; break;
                    default: break;
                }
            }
            continue;
        }

        // Wir wollen wave_t = {hightime, lowtime}. Pulses kommen alternierend an.
        // Erstes Symbol: ggf. low-pulse-only (vor erstem rising edge) — skippen.
        if (l0 == 1 && l1 == 0) {
            feed_wave(&b, tscale(d0), tscale(d1));
        } else if (l0 == 0 && l1 == 1) {
            // Trailing low + leading high; trailing low wird im nächsten Symbol
            // mit dessen falling edge ergänzt — vereinfacht: behandeln wir als
            // einzelne Wave mit halt halbe Information. Für 1-Bit-Genauigkeit
            // reicht's nicht — wir skippen (genauso wie a-culfw beim FE/RE-Mismatch).
            continue;
        } else {
            // Sehr unwahrscheinlich (level0==level1) — RMT fasst gleichlevel nie.
            continue;
        }
    }

    // Frame komplett im Bucket — analyze + emit.
    finish(b);
}

void SlowRfRF::emit_frame_(const SlowRfFrame& f) {
    // CC1101 RSSI (a-culfw cc1100.c:rxrssi): raw → dBm via 2's-complement /2 - 74.
    int16_t rssi_dbm = cc1101_rssi_dbm(f.rssi_raw);
    if (rssi_dbm < -128) rssi_dbm = -128;
    if (rssi_dbm > 127)  rssi_dbm =  127;
    RfStats::instance().on_rx(ProtocolId::SlowRf, f.len, static_cast<int8_t>(rssi_dbm));

    // Repeat-Filter byte-genau a-culfw rf_receive.c:461..477. Kind=FS20/KS300/EM/HMS
    // werden alle gefiltert; FHT-Repeats werden in a-culfw separat (REP_FHTPROTO)
    // behandelt — fuer FHT lassen wir den Filter weg, weil die FHT-Logik in
    // 8b/8v-Tabellen Repeat-Erkennung macht und nicht ueber REPTIME laeuft.
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    bool filter_active = !(tx_report_ & 0x02);  // REP_REPEATED bit
    bool is_dup = false;
    if (filter_active && f.kind != SlowRfFrame::Kind::FHT) {
        if (f.kind == last_kind_ && f.len == last_len_ && f.len <= 16 &&
            std::memcmp(f.data, last_buf_, f.len) == 0 &&
            (now_ms - last_emit_ms_) < kRepWindowMs) {
            is_dup = true;
        }
    }
    // Last-Frame-Snapshot IMMER aktualisieren (a-culfw setzt reptime
    // unconditional; egal ob Match oder nicht).
    last_kind_    = f.kind;
    last_len_     = f.len <= 16 ? f.len : 16;
    if (last_len_) std::memcpy(last_buf_, f.data, last_len_);
    last_emit_ms_ = now_ms;

    if (is_dup) return;     // Frame als Repeat unterdrueckt — kein rx_cb_!

    if (!rx_cb_) return;
    RxEvent ev{
        .proto        = ProtocolId::SlowRf,
        .slot_id      = slot_id_,
        .rssi_dbm     = static_cast<int8_t>(f.rssi_raw),
        .timestamp_us = static_cast<uint32_t>(esp_timer_get_time()),
        .payload      = &f,
    };
    rx_cb_(ev);
}

esp_err_t SlowRfRF::start(uint8_t slot_id, CC1101Driver& drv) {
    drv_     = &drv;
    slot_id_ = slot_id;

    drv.strobe(SIDLE);
    ets_delay_us(100);

    // Register 0x00..0x28 wie a-culfw ccInitChip(EE_CC1100_CFG) aus dem
    // EEPROM; FHEM stellt Frequenz, bWidth, rAmpl und sens per `W` dort ein und
    // laedt mit X21 neu. Bis 0.2.0 kam hier immer SLOWRF_CFG, `W` wirkte nie
    // (gemeldet 30.10.2026, bekannt seit 23.08.). SLOWRF_CFG bleibt Rueckfall
    // ohne EEPROM; seine Werte 0x00..0x28 sind die EEPROM-Defaults.
    uint8_t regs[kCfgRegs] = {};
    for (size_t i = 0; SLOWRF_CFG[i] != 0xFF && SLOWRF_CFG[i] <= 0x40; i += 2) {
        if (SLOWRF_CFG[i] < kCfgRegs) regs[SLOWRF_CFG[i]] = SLOWRF_CFG[i + 1];
    }
    if (cfg_provider_) cfg_provider_(std::span<uint8_t, kCfgRegs>(regs));
    for (uint8_t addr = 0; addr < kCfgRegs; ++addr) {
        if (auto e = drv.write_reg(addr, regs[addr]); e != ESP_OK) {
            ESP_LOGE(TAG, "write_reg(0x%02x): %s", addr, esp_err_to_name(e));
            return e;
        }
    }
    // PATABLE + FREND0 aus dem EEPROM — wie a-culfw ccInitChip(), das bei jedem
    // Mode-Start EE_CC1100_PA laedt. Damit gilt `x<idx>` auch nach Reboot und
    // Mode-Wechsel (vorher stand hier fest {0x00, 0xC2} = +10 dBm, und der
    // per `x` gewaehlte Pegel galt nur, solange kein Mode-Start folgte —
    // gemessen 02.09.2026, HackRF: nach Reset +5,4 dB lauter als `x02`).
    // Fuer OOK zaehlen [0]=carrier-off und der Eintrag unter FREND0.PA_POWER;
    // bei einem Ramping-Profil (PA_POWER=7) laeuft der Chip durch alle 8.
    // Fallback ohne Provider: 0 dBm ohne Ramping.
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

    // SCAL ueberschreibt 0x23..0x25 (FSCAL3/2/1) mit Hardware-kalibrierten Werten
    // — das ist gewollt. Andere Register sollten unveraendert bleiben. Trotzdem
    // hat sich auf der bisherigen Hardware FREQ1 (0x0E) als unzuverlaessig
    // erwiesen: der Bulk-Write klappt, aber zum Lese-Zeitpunkt steht der Reset-
    // Default 0xC4 statt 0x65 drin. Workaround: kritische Register (FREQ2/1/0,
    // PKTCTRL0, MDMCFG2/4, MCSM0) explizit NACH SCAL nochmal schreiben. Das ist
    // billiger als ein Read-Verify pro Register und deckt den Fail-Modus ab.
    // Mit den geladenen Werten, nicht mit Konstanten: bis 0.2.0 standen hier
    // feste Werte und haetten ein per `W` gesetztes bWidth/Frequenz ueberschrieben.
    static constexpr uint8_t REWRITE_REGS[] = {
        0x0D, 0x0E, 0x0F,     // FREQ
        0x08,                 // PKTCTRL0 (PKT_FORMAT=3)
        0x10, 0x11, 0x12,     // MDMCFG4/3/2 (BW + DRATE + OOK)
        0x18,                 // MCSM0 (FS_AUTOCAL)
    };
    for (uint8_t addr : REWRITE_REGS) {
        drv.write_reg(addr, regs[addr]);
    }

    // Final-Verify (nur Diagnose).
    for (uint8_t addr = 0; addr < kCfgRegs; ++addr) {
        const uint8_t want = regs[addr];
        // Diese sind nach SCAL erwartet hardware-spezifisch und nicht user-defined.
        if (addr == 0x23 || addr == 0x24 || addr == 0x25) continue;
        // 0x22 FREND0 kommt mit der PA-Tabelle (PaTableProvider), 0x0C FSCTRL0
        // aus der Frequenzkalibrierung.
        if (addr == 0x22 || addr == 0x0C) continue;
        uint8_t got = 0xFF;
        drv.read_reg(addr, got);
        if (got != want) {
            ESP_LOGE(TAG, "reg 0x%02x final mismatch: want 0x%02x got 0x%02x",
                     addr, want, got);
        }
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
        if (xTaskCreate(&SlowRfRF::rx_task_trampoline, "slowrf-rx",
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

    ESP_LOGI(TAG, "started on slot %u (gdo0=%d gdo2=%d)",
             slot_id, drv.pin_gdo0(), drv.pin_gdo2());

    return ESP_OK;
}

esp_err_t SlowRfRF::stop() {
    // Eternal-Task-Pattern (wie AskSin/Moritz/HmDual): rx_task lebt bis zum
    // Reboot, ueberlebt jeden stop()/start()-Zyklus. enabled_==false signalisiert
    // "deaktiviert" — Task pollt dann leer weiter.
    //
    // Frueheres Pattern (Build <= 0.1.770) tat task_running_=false +
    // vQueueDelete(rx_queue_) waehrend die Task noch in xQueueReceive(rx_queue_)
    // blockierte. Bei Slot-Transition (z.B. HmDual→SlowRF→wMBus im Smoke-Test
    // ueber jedes reset_mode-X00-Tripel) fuehrte das zu Load Fault in
    // xTaskRemoveFromEventList (FreeRTOS tasks.c:3990, MTVAL=0x1f, MEPC=
    // 0x4080ecc0) — die TCB-Event-List zeigt nach vQueueDelete auf freed memory.
    // Fix Build 0.1.771: rx_queue_ + task bleiben persistent, nur die RMT-
    // Channels werden teardown'd. Verifiziert via Smoke `rf` 17/17 PASS.
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

// ============================================================================
// TX-Pfad — RMT-TX für FS20 (PKT_FORMAT=3 async serial: GDO0 als TX-Daten-Eingang).
// ============================================================================

esp_err_t SlowRfRF::setup_rmt_tx_() {
    rmt_tx_channel_config_t cfg = {};
    cfg.gpio_num         = static_cast<gpio_num_t>(drv_->pin_gdo0());
    cfg.clk_src          = RMT_CLK_SRC_DEFAULT;
    cfg.resolution_hz    = 1'000'000;             // 1 µs ticks
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
    if (auto e = rmt_enable(rmt_tx_chan_); e != ESP_OK) {
        ESP_LOGE(TAG, "rmt_tx enable: %s", esp_err_to_name(e));
        return e;
    }
    return ESP_OK;
}

esp_err_t SlowRfRF::teardown_rmt_tx_() {
    if (rmt_tx_enc_) {
        rmt_del_encoder(rmt_tx_enc_);
        rmt_tx_enc_ = nullptr;
    }
    if (rmt_tx_chan_) {
        rmt_disable(rmt_tx_chan_);
        rmt_del_channel(rmt_tx_chan_);
        rmt_tx_chan_ = nullptr;
    }
    return ESP_OK;
}

namespace {

// Encode FS20-Frame in RMT-Symbole. Sync = 12 zeros + 1 one, dann jedes byte
// MSB-first 8 bits + 1 even-parity-bit, dann trailing 0-bit.
// Ein FS20-Bit = HIGH(t) + LOW(t) mit t=400µs (zero) oder 600µs (one).
struct PulseBuilder {
    rmt_symbol_word_t* buf;
    size_t cap;
    size_t pos;

    void push(uint16_t high_us, uint16_t low_us) {
        if (pos >= cap) return;
        buf[pos].duration0 = high_us; buf[pos].level0 = 1;
        buf[pos].duration1 = low_us;  buf[pos].level1 = 0;
        pos++;
    }
};

constexpr uint16_t FS20_ZERO_US = 400;
constexpr uint16_t FS20_ONE_US  = 600;

// KS300-Bit-Timing byte-genau a-culfw rf_send.c:338-341 (`ks_send`):
//   zerohigh = 854 µs, zerolow = 366 µs
//   onehigh  = 366 µs, onelow  = 854 µs
// Pulse fuer "0": HIGH(854) + LOW(366)
// Pulse fuer "1": HIGH(366) + LOW(854)
constexpr uint16_t KS300_ZERO_HIGH_US = 854;
constexpr uint16_t KS300_ZERO_LOW_US  = 366;
constexpr uint16_t KS300_ONE_HIGH_US  = 366;
constexpr uint16_t KS300_ONE_LOW_US   = 854;

// EM-Bit-Timing byte-genau a-culfw rf_send.c:297-298 (`em_send`):
//   onehigh = zerohigh = zerolow = EM_ZERO = 400 µs
//   onelow  = EM_ONE = 800 µs
// Pulse fuer "0": HIGH(400) + LOW(400)
// Pulse fuer "1": HIGH(400) + LOW(800)
constexpr uint16_t EM_ZERO_HIGH_US = 400;
constexpr uint16_t EM_ZERO_LOW_US  = 400;
constexpr uint16_t EM_ONE_HIGH_US  = 400;
constexpr uint16_t EM_ONE_LOW_US   = 800;

static size_t encode_fs20_frame(const uint8_t* hb, uint8_t hblen, uint8_t startcs,
                                 rmt_symbol_word_t* out, size_t out_cap) {
    PulseBuilder pb{out, out_cap, 0};

    // Checksum anhängen.
    uint8_t buf[16];
    if (hblen + 1 > sizeof(buf)) return 0;
    std::memcpy(buf, hb, hblen);
    buf[hblen] = cksum1(startcs, buf, hblen);
    uint8_t total = hblen + 1;

    // Sync: 12 zeroes.
    for (int i = 0; i < 12; ++i) pb.push(FS20_ZERO_US, FS20_ZERO_US);
    // Sync trailer 1.
    pb.push(FS20_ONE_US, FS20_ONE_US);

    // Bytes mit even-parity-bit.
    for (uint8_t i = 0; i < total; ++i) {
        uint8_t b = buf[i];
        for (int k = 7; k >= 0; --k) {
            uint8_t bit = (b >> k) & 1;
            uint16_t t = bit ? FS20_ONE_US : FS20_ZERO_US;
            pb.push(t, t);
        }
        uint8_t pb_bit = parity_even_bit(b);
        uint16_t t = pb_bit ? FS20_ONE_US : FS20_ZERO_US;
        pb.push(t, t);
    }
    // Trailing 0.
    pb.push(FS20_ZERO_US, FS20_ZERO_US);
    return pb.pos;
}

// KS300-Encoder byte-genau a-culfw rf_send.c:323 (`ks_send`).
// Pro Datenbyte werden 10 Bits gesendet, alle LSB-first-organisiert:
//   bit 0..3 (untere Halbnibble), 1-Stopbit, bit 4..7 (obere Halbnibble), 1-Stopbit
// Sync = 10 Zero-Bits + 1 One-Bit (sendraw mit sync=10, addH=addL=0).
// Checksum: cksum3 (XOR/add ueber alle Halbnibbles, byte-genau a-culfw rf_receive.c:200).
// `nibble`-Flag (RX-Decoder-Feature) ist beim TX immer false → cksum3-Pfad
// `if (!nibble || cnt)` ist immer true; voller Roundtrip ueber alle Halbnibbles.
static size_t encode_ks300_frame(const uint8_t* hb, uint8_t hblen,
                                  rmt_symbol_word_t* out, size_t out_cap) {
    PulseBuilder pb{out, out_cap, 0};

    auto push_bit = [&](uint8_t bit) {
        if (bit)
            pb.push(KS300_ONE_HIGH_US,  KS300_ONE_LOW_US);
        else
            pb.push(KS300_ZERO_HIGH_US, KS300_ZERO_LOW_US);
    };

    // Checksum anhaengen.
    uint8_t buf[16];
    if (hblen + 1 > sizeof(buf)) return 0;
    std::memcpy(buf, hb, hblen);
    bool dummy = false;
    buf[hblen] = cksum3(buf, hblen, dummy);
    uint8_t total = hblen + 1;

    // Sync: 10 Zeros + 1 One (a-culfw sendraw sync=10 → 10 zero-bits, dann ein one-bit).
    for (int i = 0; i < 10; ++i) push_bit(0);
    push_bit(1);

    // Daten-Bytes: 4 LSB + 1-stopbit + 4 MSB + 1-stopbit, jedes Halbnibble LSB-first.
    for (uint8_t i = 0; i < total; ++i) {
        uint8_t b = buf[i];
        for (int k = 0; k < 4; ++k) push_bit((b >> k) & 1);
        push_bit(1);
        for (int k = 4; k < 8; ++k) push_bit((b >> k) & 1);
        push_bit(1);
    }
    return pb.pos;
}

// EM-Encoder byte-genau a-culfw rf_send.c:283 (`em_send`).
// EM ist immer 9 Bytes payload + 1 Byte cksum2 = 10 Bytes total.
// Encoding pro Byte: 8 Daten-Bits LSB-first, dann 1-stopbit (= 9 bits per byte).
// Sync = 12 Zero-Bits + 1 One-Bit (sendraw mit sync=12).
static size_t encode_em_frame(const uint8_t* hb, uint8_t hblen,
                               rmt_symbol_word_t* out, size_t out_cap) {
    if (hblen != 9) return 0;       // EM ist strikt 9 bytes — Caller validiert vorher
    PulseBuilder pb{out, out_cap, 0};

    auto push_bit = [&](uint8_t bit) {
        if (bit)
            pb.push(EM_ONE_HIGH_US,  EM_ONE_LOW_US);
        else
            pb.push(EM_ZERO_HIGH_US, EM_ZERO_LOW_US);
    };

    // Checksum (cksum2 = XOR ueber alle Bytes) anhaengen.
    uint8_t buf[16];
    std::memcpy(buf, hb, hblen);
    uint8_t cs = 0;
    for (uint8_t i = 0; i < hblen; ++i) cs ^= hb[i];
    buf[hblen] = cs;
    uint8_t total = hblen + 1;

    // Sync: 12 Zeros + 1 One (sendraw sync=12).
    for (int i = 0; i < 12; ++i) push_bit(0);
    push_bit(1);

    // Daten-Bytes: 8 Bits LSB-first + 1-stopbit pro Byte.
    for (uint8_t i = 0; i < total; ++i) {
        uint8_t b = buf[i];
        for (int k = 0; k < 8; ++k) push_bit((b >> k) & 1);
        push_bit(1);
    }
    return pb.pos;
}

} // anon namespace

esp_err_t SlowRfRF::tx(const void* typed_payload, size_t payload_size) {
    if (!enabled_ || !typed_payload || payload_size < sizeof(SlowRfTxFrame)) {
        return ESP_ERR_INVALID_ARG;
    }
    auto* f = static_cast<const SlowRfTxFrame*>(typed_payload);
    if (f->kind != SlowRfTxFrame::Kind::FS20 &&
        f->kind != SlowRfTxFrame::Kind::KS300 &&
        f->kind != SlowRfTxFrame::Kind::EM) return ESP_ERR_NOT_SUPPORTED;
    // EM ist strikt 9 Bytes; KS300 erlaubt 1..12; FS20 1..8.
    if (f->kind == SlowRfTxFrame::Kind::EM) {
        if (f->len != 9) return ESP_ERR_INVALID_SIZE;
    } else {
        const uint8_t max_len = (f->kind == SlowRfTxFrame::Kind::KS300) ? 12 : 8;
        if (f->len == 0 || f->len > max_len) return ESP_ERR_INVALID_ARG;
    }

    // Credit-Check zentral via RfStats (a-culfw rf_send.c:104 ff,
    // sum=(nbyte+2)*repeat+addH+addL; wir haben addH=addL=0 für FS20/EM/KS).
    // Refill +1/s passiert im RfStats-1Hz-Timer; init=MAX_CREDIT/2=450.
    uint16_t need = (uint16_t)(f->len + 2) * f->repeat;
    if (!RfStats::instance().try_consume(need)) {
        RfStats::instance().on_tx_error(ProtocolId::SlowRf);
        return ESP_ERR_INVALID_STATE;   // → LOVF im Frontend
    }

    // RX kurz pausieren — RMT-Channel-Pin-Konflikt vermeiden.  Danach legt
    // resume_rx_() den Kanal in jedem Fall neu an, auch wenn keiner da war:
    // so heilt der naechste Sendevorgang ein fehlgeschlagenes RX-Setup.
    teardown_rmt_rx_();
    drv_->strobe(SIDLE);
    ets_delay_us(200);

    if (auto e = setup_rmt_tx_(); e != ESP_OK) {
        teardown_rmt_tx_();                   // halb angelegten TX-Kanal abraeumen
        resume_rx_();
        return e;
    }

    // Frame encodieren — class-member-buffer, NICHT auf Stack.
    size_t n;
    switch (f->kind) {
        case SlowRfTxFrame::Kind::KS300:
            n = encode_ks300_frame(f->data, f->len, tx_buffer_, kTxBufferSyms);
            break;
        case SlowRfTxFrame::Kind::EM:
            n = encode_em_frame(f->data, f->len, tx_buffer_, kTxBufferSyms);
            break;
        default:
            n = encode_fs20_frame(f->data, f->len, f->startcs, tx_buffer_, kTxBufferSyms);
            break;
    }
    if (n == 0) {
        teardown_rmt_tx_();
        resume_rx_();
        return ESP_ERR_NO_MEM;
    }

    // STX strobe — Chip geht IDLE → SETTLING → TX, ~500 µs Settling reicht.
    drv_->strobe(STX);
    ets_delay_us(500);

    rmt_transmit_config_t tcfg = {};
    tcfg.loop_count = 0;

    for (uint8_t r = 0; r < f->repeat; ++r) {
        if (auto e = rmt_transmit(rmt_tx_chan_, rmt_tx_enc_, tx_buffer_,
                                  n * sizeof(rmt_symbol_word_t), &tcfg);
            e != ESP_OK) {
            ESP_LOGE(TAG, "rmt_transmit %u: %s", r, esp_err_to_name(e));
            break;
        }
        rmt_tx_wait_all_done(rmt_tx_chan_, 1000);
        if (r + 1 < f->repeat) vTaskDelay(pdMS_TO_TICKS(10));
    }

    drv_->strobe(SIDLE);
    teardown_rmt_tx_();

    resume_rx_();
    RfStats::instance().on_tx(ProtocolId::SlowRf, f->len, need);
    return ESP_OK;
}

} // namespace culfw32
