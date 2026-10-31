// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
// Portions derived from a-culfw (https://github.com/tostmann/a-culfw),
// GNU GPL version 2 or (at your option) any later version.

#pragma once

#include <atomic>
#include <cstdint>
#include "esp_err.h"
#include "esp_timer.h"
#include "rf_protocol.hpp"

namespace culfw32 {

// Zentrale RF-/System-Telemetrie. Singleton, lock-free wo moeglich (atomics).
//
// LOVF/Credit byte-genau a-culfw rf_send.c/clock.c:
//   uint16_t credit_10ms;       // current
//   #define MAX_CREDIT 900      // 9s burst budget
//   init: credit_10ms = MAX_CREDIT/2 = 450 (a-culfw rf_receive.c:125)
//   refill: clock-tick 1Hz: if (credit_10ms < MAX_CREDIT) credit_10ms += 1
//   consume: if (credit_10ms < sum) emit "LOVF\r\n"; else credit_10ms -= sum;
//
// Zusaetzlich: Per-Protocol-Counters (rx_msgs/tx_msgs/rx_bytes/tx_bytes/last_rssi/
// last_rx_ts) plus globale tx_total/rx_total/lovf_count plus heap-min/free.
//
// Alle Werte werden per ConfigRegistry FLAG_READONLY exponiert → automatisches
// Live-Push 5s zur Web-UI/JSON-Frontend.
class RfStats {
public:
    static constexpr uint16_t kMaxCredit  = 900;   // a-culfw MAX_CREDIT
    static constexpr uint16_t kInitCredit = 450;   // a-culfw MAX_CREDIT/2

    // Anzahl distinct Protocols, fuer per-proto Slots.
    static constexpr size_t kProtoCount = 8;       // AskSin/Moritz/SlowRf/PulseCapture/FastRf/Native/Mbus/Hmip

    static RfStats& instance();

    // Lifecycle: refill-timer starten/stoppen.
    esp_err_t start();
    esp_err_t stop();

    // Credit-API.
    uint16_t credit() const          { return credit_10ms_.load(std::memory_order_relaxed); }
    uint16_t credit_max() const      { return kMaxCredit; }
    uint8_t  credit_pct() const      { return (uint8_t)((100u * credit()) / kMaxCredit); }

    // Versuche `units` 10ms-Einheiten zu konsumieren. Liefert true bei Erfolg.
    // Bei false → caller muss "LOVF\r\n" emit. Lovf-Counter wird hier gebumpt.
    bool try_consume(uint16_t units);

    // Bucht `units` ab, ohne je abzulehnen: reicht der Kredit nicht, steht er
    // danach auf 0. Fuer Sendungen, die nicht ausfallen duerfen, deren
    // Funklast `X` aber zeigen soll (AskSin). Kein LOVF.
    void consume_floor(uint16_t units);

    // Refill-Bonus (z.B. FHT 8v rules in a-culfw fht.c:324).
    void grant(uint16_t units);

    // Globale RF-Counter.
    uint32_t lovf_count() const      { return lovf_count_.load(std::memory_order_relaxed); }
    uint32_t tx_total() const        { return tx_total_.load(std::memory_order_relaxed); }
    uint32_t rx_total() const        { return rx_total_.load(std::memory_order_relaxed); }

    // Per-Protocol Hooks. ProtocolId::AskSin..Native akzeptiert; Mbus/PulseCapture
    // werden auf Slot 3/6 gemappt. Callable aus Task-Kontext (atomic increments).
    void on_rx(ProtocolId p, uint16_t bytes, int8_t rssi_dbm);
    // tx_airtime_10ms = wie viele 10-ms-Einheiten der Frame on-air war
    // (= consumed credit). Wir tracken das global UND pro Protokoll, damit man
    // CE/FCC-Duty-Cycle pro Stunde visualisieren kann (1% = 360 Units/h).
    void on_tx(ProtocolId p, uint16_t bytes, uint16_t tx_airtime_10ms);

    // Error-Counter pro Protokoll. Generic — pro Klasse selber dokumentiert,
    // was der Counter zaehlt:
    //   rx_err: CRC-Fehler, LENERR-RX, FIFO-Overflow waehrend RX, gegen Sync-
    //           Word fehlgeschlagene Frames.
    //   tx_err: ZERR1 (nicht in RX), STX-Timeout, TXFIFO-Underflow ohne
    //           Recovery, ERR:CCA bei AskSin.
    //   crc_err ist Sub-Set von rx_err und wird zusaetzlich gezaehlt fuer
    //           Protokolle mit On-Air-CRC, die Fehlrahmen selbst sehen
    //           (FastRf, wMBus, Dudette). AskSin, Moritz und HmIP laufen mit
    //           CRC_AUTOFLUSH: der CC1101 verwirft Pakete mit falscher CRC,
    //           einen Rahmen ohne CRC_OK verwirft der Treiber (rx_resync in
    //           ?stats); ihr crc_err bleibt 0.
    void on_rx_error(ProtocolId p);
    void on_tx_error(ProtocolId p);
    void on_crc_error(ProtocolId p);

    // Globaler TX-Airtime-Counter (Summe ueber alle Protokolle).
    uint32_t tx_airtime_10ms_total() const {
        return tx_airtime_total_.load(std::memory_order_relaxed);
    }

    // Per-Protocol Getter (read-side, ConfigRegistry).
    struct Snapshot {
        uint32_t rx_msgs;
        uint32_t tx_msgs;
        uint32_t rx_bytes;
        uint32_t tx_bytes;
        uint32_t tx_airtime_10ms;
        uint32_t rx_err;
        uint32_t tx_err;
        uint32_t crc_err;
        int32_t  last_rssi_dbm;
        uint32_t last_rx_uptime_ms; // 0 = noch nichts empfangen
    };
    Snapshot snapshot(ProtocolId p) const;

    // Sekunden seit letztem RX, oder UINT32_MAX wenn nie.
    uint32_t last_rx_ago_s(ProtocolId p) const;

    // System.
    uint32_t heap_free() const;
    uint32_t heap_min() const;

    // Dump-Helper fuer ASCII-Frontend (compact tabular). Caller-buffer.
    // Schreibt bis to_size-1 Bytes; null-terminiert.
    size_t format_text(char* out, size_t out_size) const;

private:
    RfStats() = default;
    RfStats(const RfStats&) = delete;
    RfStats& operator=(const RfStats&) = delete;

    static int proto_idx(ProtocolId p);

    static void on_refill_tick(void* arg);

    std::atomic<uint16_t> credit_10ms_      {kInitCredit};
    std::atomic<uint32_t> lovf_count_       {0};
    std::atomic<uint32_t> tx_total_         {0};
    std::atomic<uint32_t> rx_total_         {0};
    std::atomic<uint32_t> tx_airtime_total_ {0};

    struct ProtoSlot {
        std::atomic<uint32_t> rx_msgs         {0};
        std::atomic<uint32_t> tx_msgs         {0};
        std::atomic<uint32_t> rx_bytes        {0};
        std::atomic<uint32_t> tx_bytes        {0};
        std::atomic<uint32_t> tx_airtime_10ms {0};
        std::atomic<uint32_t> rx_err          {0};
        std::atomic<uint32_t> tx_err          {0};
        std::atomic<uint32_t> crc_err         {0};
        std::atomic<int32_t>  last_rssi_dbm   {0};
        std::atomic<uint32_t> last_rx_us      {0};   // esp_timer_get_time() snapshot ms
    };
    ProtoSlot slots_[kProtoCount];

    esp_timer_handle_t refill_timer_ = nullptr;
};

} // namespace culfw32
