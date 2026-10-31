// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
// Portions derived from a-culfw (https://github.com/tostmann/a-culfw),
// GNU GPL version 2 or (at your option) any later version.

#include "rf_stats.hpp"
#include "esp_heap_caps.h"
#include <cstdio>
#include <cstring>

namespace culfw32 {

namespace {
const char* proto_name(ProtocolId p) {
    switch (p) {
        case ProtocolId::AskSin:       return "asksin";
        case ProtocolId::Moritz:       return "moritz";
        case ProtocolId::SlowRf:       return "slowrf";
        case ProtocolId::PulseCapture: return "pulse";
        case ProtocolId::FastRf:       return "fastrf";
        case ProtocolId::Native:       return "native";
        case ProtocolId::Mbus:         return "wmbus";
        case ProtocolId::Hmip:         return "hmip";
        default:                       return "?";
    }
}
} // anon

RfStats& RfStats::instance() {
    static RfStats s_inst;
    return s_inst;
}

int RfStats::proto_idx(ProtocolId p) {
    switch (p) {
        case ProtocolId::AskSin:       return 0;
        case ProtocolId::Moritz:       return 1;
        case ProtocolId::SlowRf:       return 2;
        case ProtocolId::PulseCapture: return 3;
        case ProtocolId::FastRf:       return 4;
        case ProtocolId::Native:       return 5;
        case ProtocolId::Mbus:         return 6;
        case ProtocolId::Hmip:         return 7;
        default:                       return -1;
    }
}

void RfStats::on_refill_tick(void* /*arg*/) {
    auto& s = RfStats::instance();
    // a-culfw clock.c:203 — atomar +1 nur wenn < MAX_CREDIT.
    uint16_t cur = s.credit_10ms_.load(std::memory_order_relaxed);
    while (cur < kMaxCredit) {
        if (s.credit_10ms_.compare_exchange_weak(cur, cur + 1,
                                                  std::memory_order_relaxed)) {
            break;
        }
    }
}

esp_err_t RfStats::start() {
    if (refill_timer_) return ESP_OK;
    esp_timer_create_args_t cfg = {};
    cfg.callback = &RfStats::on_refill_tick;
    cfg.arg      = this;
    cfg.dispatch_method = ESP_TIMER_TASK;
    cfg.name     = "rf_credit";
    if (auto e = esp_timer_create(&cfg, &refill_timer_); e != ESP_OK) return e;
    return esp_timer_start_periodic(refill_timer_, 1'000'000);  // 1Hz
}

esp_err_t RfStats::stop() {
    if (!refill_timer_) return ESP_OK;
    esp_timer_stop(refill_timer_);
    esp_timer_delete(refill_timer_);
    refill_timer_ = nullptr;
    return ESP_OK;
}

bool RfStats::try_consume(uint16_t units) {
    if (units == 0) return true;
    uint16_t cur = credit_10ms_.load(std::memory_order_relaxed);
    while (cur >= units) {
        if (credit_10ms_.compare_exchange_weak(cur, cur - units,
                                                std::memory_order_relaxed)) {
            return true;
        }
    }
    lovf_count_.fetch_add(1, std::memory_order_relaxed);
    return false;
}

void RfStats::consume_floor(uint16_t units) {
    if (units == 0) return;
    uint16_t cur = credit_10ms_.load(std::memory_order_relaxed);
    while (!credit_10ms_.compare_exchange_weak(cur, cur > units ? (uint16_t)(cur - units) : 0,
                                               std::memory_order_relaxed)) {
    }
}

void RfStats::grant(uint16_t units) {
    if (units == 0) return;
    uint16_t cur = credit_10ms_.load(std::memory_order_relaxed);
    while (true) {
        uint32_t want = (uint32_t)cur + units;
        if (want > kMaxCredit) want = kMaxCredit;
        if (credit_10ms_.compare_exchange_weak(cur, (uint16_t)want,
                                                std::memory_order_relaxed)) {
            return;
        }
    }
}

void RfStats::on_rx(ProtocolId p, uint16_t bytes, int8_t rssi_dbm) {
    int i = proto_idx(p);
    if (i < 0) return;
    auto& s = slots_[i];
    s.rx_msgs.fetch_add(1, std::memory_order_relaxed);
    s.rx_bytes.fetch_add(bytes, std::memory_order_relaxed);
    s.last_rssi_dbm.store((int32_t)rssi_dbm, std::memory_order_relaxed);
    int64_t us = esp_timer_get_time();
    s.last_rx_us.store((uint32_t)(us / 1000), std::memory_order_relaxed); // ms
    rx_total_.fetch_add(1, std::memory_order_relaxed);
}

void RfStats::on_tx(ProtocolId p, uint16_t bytes, uint16_t tx_airtime_10ms) {
    int i = proto_idx(p);
    if (i < 0) return;
    auto& s = slots_[i];
    s.tx_msgs.fetch_add(1, std::memory_order_relaxed);
    s.tx_bytes.fetch_add(bytes, std::memory_order_relaxed);
    s.tx_airtime_10ms.fetch_add(tx_airtime_10ms, std::memory_order_relaxed);
    tx_total_.fetch_add(1, std::memory_order_relaxed);
    tx_airtime_total_.fetch_add(tx_airtime_10ms, std::memory_order_relaxed);
}

void RfStats::on_rx_error(ProtocolId p) {
    int i = proto_idx(p);
    if (i < 0) return;
    slots_[i].rx_err.fetch_add(1, std::memory_order_relaxed);
}

void RfStats::on_tx_error(ProtocolId p) {
    int i = proto_idx(p);
    if (i < 0) return;
    slots_[i].tx_err.fetch_add(1, std::memory_order_relaxed);
}

void RfStats::on_crc_error(ProtocolId p) {
    int i = proto_idx(p);
    if (i < 0) return;
    slots_[i].rx_err.fetch_add(1, std::memory_order_relaxed);
    slots_[i].crc_err.fetch_add(1, std::memory_order_relaxed);
}

RfStats::Snapshot RfStats::snapshot(ProtocolId p) const {
    Snapshot snap = {};
    int i = proto_idx(p);
    if (i < 0) return snap;
    const auto& s = slots_[i];
    snap.rx_msgs           = s.rx_msgs.load(std::memory_order_relaxed);
    snap.tx_msgs           = s.tx_msgs.load(std::memory_order_relaxed);
    snap.rx_bytes          = s.rx_bytes.load(std::memory_order_relaxed);
    snap.tx_bytes          = s.tx_bytes.load(std::memory_order_relaxed);
    snap.tx_airtime_10ms   = s.tx_airtime_10ms.load(std::memory_order_relaxed);
    snap.rx_err            = s.rx_err.load(std::memory_order_relaxed);
    snap.tx_err            = s.tx_err.load(std::memory_order_relaxed);
    snap.crc_err           = s.crc_err.load(std::memory_order_relaxed);
    snap.last_rssi_dbm     = s.last_rssi_dbm.load(std::memory_order_relaxed);
    snap.last_rx_uptime_ms = s.last_rx_us.load(std::memory_order_relaxed);
    return snap;
}

uint32_t RfStats::last_rx_ago_s(ProtocolId p) const {
    int i = proto_idx(p);
    if (i < 0) return UINT32_MAX;
    uint32_t t_ms = slots_[i].last_rx_us.load(std::memory_order_relaxed);
    if (t_ms == 0) return UINT32_MAX;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (now_ms < t_ms) return 0;
    return (now_ms - t_ms) / 1000;
}

uint32_t RfStats::heap_free() const {
    return (uint32_t)heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
}

uint32_t RfStats::heap_min() const {
    return (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
}

size_t RfStats::format_text(char* out, size_t out_size) const {
    if (!out || out_size < 32) return 0;
    int n = 0;
    n += snprintf(out + n, out_size - n,
                  "credit %u/%u (%u%%)  lovf %lu  rx_total %lu  tx_total %lu\r\n"
                  "heap free %lu  min %lu\r\n",
                  (unsigned)credit(), (unsigned)kMaxCredit, (unsigned)credit_pct(),
                  (unsigned long)lovf_count(),
                  (unsigned long)rx_total(), (unsigned long)tx_total(),
                  (unsigned long)heap_free(), (unsigned long)heap_min());
    if (n < 0 || (size_t)n >= out_size) { out[out_size - 1] = 0; return out_size - 1; }

    static const ProtocolId kAll[] = {
        ProtocolId::Moritz, ProtocolId::AskSin, ProtocolId::SlowRf,
        ProtocolId::Native, ProtocolId::FastRf, ProtocolId::PulseCapture,
        ProtocolId::Mbus,   ProtocolId::Hmip,
    };
    for (ProtocolId p : kAll) {
        auto s = snapshot(p);
        if (s.rx_msgs == 0 && s.tx_msgs == 0 && s.rx_err == 0 && s.tx_err == 0) continue;
        int rc = snprintf(out + n, out_size - n,
            "%-7s rx %lu/%lu (%lu B)  tx %lu/%lu (%lu B,%lu*10ms)  rssi %ld  ago %lus\r\n",
            proto_name(p),
            (unsigned long)s.rx_msgs, (unsigned long)s.rx_err,
            (unsigned long)s.rx_bytes,
            (unsigned long)s.tx_msgs, (unsigned long)s.tx_err,
            (unsigned long)s.tx_bytes, (unsigned long)s.tx_airtime_10ms,
            (long)s.last_rssi_dbm, (unsigned long)last_rx_ago_s(p));
        if (rc < 0) break;
        n += rc;
        if ((size_t)n >= out_size) { out[out_size - 1] = 0; return out_size - 1; }
    }
    return (size_t)n;
}

} // namespace culfw32
