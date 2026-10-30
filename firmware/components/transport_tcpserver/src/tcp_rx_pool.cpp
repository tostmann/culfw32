// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "tcp_rx_pool.hpp"
#include "tcp_client_transport.hpp"
#include "freertos/task.h"
#include "esp_log.h"
#include <cstdio>
#include <new>

namespace culfw32 {

namespace {
constexpr const char* TAG = "tcp-pool";

// Job-Wrapper auf dem Heap. FreeRTOS-Queue kopiert per memcpy, das vertraegt
// sich nicht mit shared_ptr-Refcount; deshalb tunneln wir den Pointer.
// Heap-Alloc ist ~16 B/job vs ~6 KB Task-Stack vorher → klare Verbesserung.
struct PooledJob {
    std::shared_ptr<TcpClientTransport> tx;
};
}

TcpRxPool& TcpRxPool::instance() {
    static TcpRxPool inst;
    return inst;
}

esp_err_t TcpRxPool::init(size_t pool_size, uint32_t worker_stack) {
    if (init_done_) return ESP_OK;
    if (pool_size == 0) pool_size = 1;
    // Queue-Tiefe = pool_size + Puffer fuer kurze Bursts ohne Reject.
    const size_t queue_depth = pool_size + pool_size; // 2x pool_size
    queue_ = xQueueCreate(queue_depth, sizeof(PooledJob*));
    if (!queue_) {
        ESP_LOGE(TAG, "queue create failed");
        return ESP_ERR_NO_MEM;
    }
    pool_size_ = pool_size;
    for (size_t i = 0; i < pool_size_; ++i) {
        // configMAX_TASK_NAME_LEN ist typischerweise 16 — i ist immer < pool_size,
        // wir clampen auf 2-stellig fuer Format-Truncation-Compliance.
        char name[16];
        std::snprintf(name, sizeof(name), "tcp-rx-%02u",
                      static_cast<unsigned>(i) % 100);
        TaskHandle_t h = nullptr;
        if (xTaskCreate(&TcpRxPool::worker_trampoline, name, worker_stack,
                        this, 6, &h) != pdPASS) {
            ESP_LOGE(TAG, "worker %u create failed", static_cast<unsigned>(i));
            // Partial-Init ist OK — was schon laeuft, laeuft. pool_size_
            // bleibt nominal, queue-Tiefe ist nicht-blockierend, also faellt
            // nur die Latenz/Saettigung etwas frueher an.
            pool_size_ = i;
            break;
        }
    }
    init_done_ = true;
    ESP_LOGI(TAG, "pool ready: %u workers x %u B stack",
             static_cast<unsigned>(pool_size_), static_cast<unsigned>(worker_stack));
    return ESP_OK;
}

esp_err_t TcpRxPool::submit(std::shared_ptr<TcpClientTransport> job) {
    if (!init_done_ || !queue_) return ESP_ERR_INVALID_STATE;
    if (!job) return ESP_ERR_INVALID_ARG;

    auto* j = new (std::nothrow) PooledJob{std::move(job)};
    if (!j) {
        jobs_rejected_.fetch_add(1, std::memory_order_relaxed);
        return ESP_ERR_NO_MEM;
    }
    // Non-blocking send — wenn Queue voll, Reject.
    if (xQueueSend(queue_, &j, 0) != pdPASS) {
        ESP_LOGW(TAG, "queue full, rejecting job");
        delete j; // shared_ptr-Dtor schliesst Socket
        jobs_rejected_.fetch_add(1, std::memory_order_relaxed);
        return ESP_ERR_NO_MEM;
    }
    jobs_submitted_.fetch_add(1, std::memory_order_relaxed);
    return ESP_OK;
}

size_t TcpRxPool::queue_pending() const {
    return queue_ ? uxQueueMessagesWaiting(queue_) : 0;
}

void TcpRxPool::worker_trampoline(void* arg) {
    static_cast<TcpRxPool*>(arg)->worker_loop();
    // Eternal-Task. Sollte nie zurueckkehren.
    vTaskDelete(nullptr);
}

void TcpRxPool::worker_loop() {
    for (;;) {
        PooledJob* j = nullptr;
        if (xQueueReceive(queue_, &j, portMAX_DELAY) != pdPASS) continue;
        if (!j) continue;

        auto tx = std::move(j->tx);
        delete j;

        if (tx) {
            tx->run_rx_loop_public();
            tx->invoke_on_closed_public();
        }
        // tx (shared_ptr) faellt aus dem Scope → Refcount sinkt, dtor laeuft
        // wenn niemand mehr eine Referenz haelt (typisch nach detach aus
        // Frontend-Liste in on_closed). Worker zurueck in xQueueReceive.
        jobs_completed_.fetch_add(1, std::memory_order_relaxed);
    }
}

} // namespace culfw32
