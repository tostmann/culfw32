// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace culfw32 {

class TcpClientTransport;

// TcpRxPool — fester Pool langlebiger rx-Worker-Tasks. Ersetzt das
// pro-Connect-xTaskCreate/vTaskDelete-Muster im TcpClientTransport, das unter
// Stress Idle-Task-Starvation aufwies: jeder vTaskDelete laesst TCB+Stack auf
// Idle-Cleanup warten; bei dichtem Smoke (5 Frontends x ~10 Connects in 14 s)
// akkumulieren Zombies schneller als Idle sie freigibt → Heap-OOM.
// Siehe heap_leak_root_cause_confirmed_2026-05-12.md.
//
// Worker-Tasks leben bis Reboot. Jeder Job = shared_ptr<TcpClientTransport>:
// Worker dequeued, ruft run_rx_loop() bis EOF/error, ruft invoke_on_closed(),
// gibt shared_ptr frei → Transport-Dtor schliesst Socket, Worker zurueck in
// xQueueReceive(portMAX_DELAY).
class TcpRxPool {
public:
    static TcpRxPool& instance();

    // Idempotent. Default 8 Worker x 6 KB Stack = 48 KB fix statt
    // pro-Connect-Growth (peak ~66 KB im Smoke beobachtet).
    esp_err_t init(size_t pool_size = 8, uint32_t worker_stack = 6144);

    // Non-blocking submit. Bei voller Queue (alle Worker busy + Queue voll)
    // gibt ESP_ERR_NO_MEM zurueck → Caller (TcpClientTransport::start) muss
    // sauber abbauen.
    esp_err_t submit(std::shared_ptr<TcpClientTransport> job);

    size_t pool_size() const { return pool_size_; }

    // Diagnose-Snapshot fuer ?stats.
    size_t queue_pending() const;
    size_t jobs_submitted() const { return jobs_submitted_; }
    size_t jobs_completed() const { return jobs_completed_; }
    size_t jobs_rejected()  const { return jobs_rejected_;  }

private:
    static void worker_trampoline(void* arg);
    void        worker_loop();

    QueueHandle_t queue_     = nullptr;
    size_t        pool_size_ = 0;
    bool          init_done_ = false;

    // Telemetrie (best-effort, kein Lock noetig — single-writer pro Feld).
    std::atomic<uint32_t> jobs_submitted_{0};
    std::atomic<uint32_t> jobs_completed_{0};
    std::atomic<uint32_t> jobs_rejected_{0};
};

} // namespace culfw32
