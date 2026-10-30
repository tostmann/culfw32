// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "transport_usbcdc.hpp"
#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/message_buffer.h"
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG = "tx-usbcdc";
// 4 KB statt 256: ein Dudette-Seitentransfer sind sieben Zeilen zu ~104
// Zeichen (Dp, 6x Dd, Dm), die ein Browser per Web Serial ohne Luecke
// schickt — ueber 600 Byte in unter einer Millisekunde. Was der Treiber bis
// dahin nicht in den Ringpuffer bekommt, ist weg; am 12.09.2026 verlor der
// Stick so bei Seite 66 von 105 das Dm. 4 KB fassen mehrere Seiten, auch
// wenn die RX-Task einmal ein paar Millisekunden nicht drankommt.
constexpr int RX_BUF_SIZE  = 4096;
constexpr int TX_BUF_SIZE  = 1024;     // USB-Serial-JTAG-Driver hardware-side TX FIFO

// App-Level TX-Queue (Producer-Consumer mit drop-oldest-on-overflow):
//   - write() (Frontend-Task-Kontext) enqueued NON-BLOCKING in tx_buf_.
//     Bei voller Queue evict-oldest in kurzer Schleife (max kMaxEvictsPerSend).
//   - tx_task drained BLOCKING aufs USB-CDC-Endpoint (is_connected-Gate).
// 8 KiB Queue ≈ 30-40 typische A/P/Z-Frames + Headroom fuer `??`-Burst-Output
// (streaming `dump_begin/item/end`-Items je ~200-400 B; Stand 0.1.717+).
// Single-Frame-Max ist TX_SCRATCH = 2048 — frontend_culfw `?stats`-Handler
// hat schon `buf[2048]` als Producer-Buffer, jetzt symmetrisch. 1024 war zu
// klein nachdem `cdc:`-Counter-Zeile dazukam (Build 0.1.729): 1267-B-Reply
// wurde silent gedroppt (siehe notes/phase5_decoder_freeze_2026-05-11.md).
constexpr size_t TX_QUEUE_BYTES        = 8192;
// 4 KB Stack — scratch[2048] auf dem Stack frisst die Haelfte, dazu kommt
// usb_serial_jtag_write_bytes-Driver-Frames + FreeRTOS. 3 KB war mit
// SCRATCH=2048 zu eng (`?stats` Build 0.1.750 zeigte stack-min=740 B).
constexpr int    TX_TASK_STACK         = 4096;
constexpr int    TX_TASK_PRIO          = 5;
constexpr size_t TX_SCRATCH            = 2048;
constexpr int    MAX_EVICTS_PER_SEND   = 16;
}

std::atomic<UsbCdcTransport*> UsbCdcTransport::s_instance_{nullptr};

esp_err_t UsbCdcTransport::start() {
    if (running_) return ESP_OK;

    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = RX_BUF_SIZE;
    cfg.tx_buffer_size = TX_BUF_SIZE;
    if (auto err = usb_serial_jtag_driver_install(&cfg); err != ESP_OK) {
        ESP_LOGE(TAG, "driver_install failed: %d", err);
        return err;
    }

    tx_buf_ = xMessageBufferCreate(TX_QUEUE_BYTES);
    if (!tx_buf_) {
        ESP_LOGE(TAG, "xMessageBufferCreate(%zu) failed", TX_QUEUE_BYTES);
        usb_serial_jtag_driver_uninstall();
        return ESP_ERR_NO_MEM;
    }

    running_ = true;
    TaskHandle_t hrx = nullptr;
    // 6 KB Stack: Frontend-Handler erzeugen mit char buf[256] + std::string-Return-
    // Werte aus ConfigRegistry-Iter spuerbar Stack-Last. 3 KB war zu knapp fuer
    // `??`-Full-Table-Output (Stack-Overflow im Iter-Path beobachtet).
    // +512 seit dem 256-Byte-Lesepuffer in rx_task (vorher 64).
    if (xTaskCreate(&UsbCdcTransport::rx_task, "usb-cdc-rx", 6656, this, 5, &hrx) != pdPASS) {
        running_ = false;
        vMessageBufferDelete(static_cast<MessageBufferHandle_t>(tx_buf_));
        tx_buf_ = nullptr;
        usb_serial_jtag_driver_uninstall();
        return ESP_FAIL;
    }
    task_handle_ = hrx;

    TaskHandle_t htx = nullptr;
    if (xTaskCreate(&UsbCdcTransport::tx_task, "usb-cdc-tx",
                    TX_TASK_STACK, this, TX_TASK_PRIO, &htx) != pdPASS) {
        ESP_LOGE(TAG, "tx_task spawn failed");
        running_ = false;
        // rx_task self-deletes on next loop-check.
        return ESP_FAIL;
    }
    tx_task_handle_ = htx;

    s_instance_.store(this, std::memory_order_release);
    ESP_LOGI(TAG, "started (tx-queue %zu bytes, evict-oldest backpressure)",
             TX_QUEUE_BYTES);
    return ESP_OK;
}

esp_err_t UsbCdcTransport::stop() {
    if (!running_) return ESP_OK;
    s_instance_.store(nullptr, std::memory_order_release);
    running_ = false;
    // Beide Tasks (rx + tx) checken running_ und vTaskDelete sich selbst.
    // tx_buf_-Cleanup machen wir hier NICHT — die Tasks koennten noch
    // mid-Receive sein.  In der Praxis wird stop() nur beim Shutdown
    // gerufen, dann ist System-Reset.  Falls echter clean-stop noetig
    // wird: Task-Join via Event + then vMessageBufferDelete.
    return ESP_OK;
}

esp_err_t UsbCdcTransport::write(std::span<const uint8_t> data) {
    if (!running_ || data.empty()) return ESP_OK;
    if (!tx_buf_) return ESP_ERR_INVALID_STATE;

    if (data.size() > TX_SCRATCH) {
        // Sollte mit den heutigen Renderer-Sub-1KB-Outputs nicht vorkommen.
        // Faengt aber eine zukuenftige Regression: Frame waere nicht
        // empfangbar (scratch zu klein) und wuerde tx_task ewig spinnen
        // (xMessageBufferReceive returnt 0, Message bleibt im Buffer).
        ESP_LOGW(TAG, "write size=%zu exceeds TX_SCRATCH=%zu — dropped",
                 data.size(), TX_SCRATCH);
        tx_dropped_full_.fetch_add(1, std::memory_order_relaxed);
        return ESP_OK;
    }

    auto buf = static_cast<MessageBufferHandle_t>(tx_buf_);
    // Producer: NON-BLOCKING send, mit evict-oldest-Loop falls voll.
    // xMessageBufferSend mit timeout=0 returnt 0 wenn nicht genug Platz
    // — Message wird NICHT teilweise aufgenommen.
    for (int i = 0; i < MAX_EVICTS_PER_SEND; ++i) {
        size_t sent = xMessageBufferSend(buf, data.data(), data.size(), 0);
        if (sent == data.size()) {
            tx_enqueued_.fetch_add(1, std::memory_order_relaxed);
            return ESP_OK;
        }
        // Voll → aelteste Message rauswerfen.
        uint8_t scratch[TX_SCRATCH];
        size_t r = xMessageBufferReceive(buf, scratch, sizeof(scratch), 0);
        if (r == 0) break;   // Queue leer aber Send schlaegt fehl → Frame zu gross
        tx_evicted_oldest_.fetch_add(1, std::memory_order_relaxed);
    }
    tx_dropped_full_.fetch_add(1, std::memory_order_relaxed);
    return ESP_OK;
}

void UsbCdcTransport::tx_task(void* arg) {
    auto* self = static_cast<UsbCdcTransport*>(arg);
    auto buf = static_cast<MessageBufferHandle_t>(self->tx_buf_);
    uint8_t scratch[TX_SCRATCH];
    while (self->running_) {
        // BLOCKING-receive — wartet auf naechste enqueued Message oder Timeout.
        size_t n = xMessageBufferReceive(buf, scratch, sizeof(scratch),
                                         pdMS_TO_TICKS(100));
        if (n == 0) continue;

        if (!usb_serial_jtag_is_connected()) {
            // Host weg → Frame ist soeben aus der Queue gezogen aber kommt
            // nirgendwo an.  Konsistent zum alten Verhalten "is_connected-
            // gate droppt silent" — aber jetzt erst NACHDEM die Queue voll
            // wurde.  D.h. wenn der Host innerhalb der Queue-Tiefe (~8 KB
            // ≈ 30-40 frames) zurueckkommt sehen wir alles; danach
            // graceful loss in Tail-Order.
            self->tx_dropped_host_gone_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        // BLOCKING-write — sicher entkoppelt vom Frontend-Task.  Default
        // 500 ms timeout deckt USB-Buffer-Drain ab; bei laenger blockierten
        // Hosts wuerde write_bytes hier returnen (loss), tx_dropped_full
        // bleibt 0 weil enqueue erfolgreich war — Loss erscheint dann als
        // tx_enqueued > tx_sent + tx_dropped_host_gone Diff.
        usb_serial_jtag_write_bytes(scratch, n, pdMS_TO_TICKS(500));
        self->tx_sent_.fetch_add(1, std::memory_order_relaxed);
    }
    vTaskDelete(nullptr);
}

void UsbCdcTransport::on_rx(RxCallback cb) {
    rx_cb_ = std::move(cb);
}

void UsbCdcTransport::set_rx_filter(RxFilter fn, void* user) {
    rx_filter_user_.store(user, std::memory_order_relaxed);
    rx_filter_.store(fn, std::memory_order_release);
}

void UsbCdcTransport::emit_to_frontend_(const uint8_t* d, size_t n, void* self_v) {
    auto* self = static_cast<UsbCdcTransport*>(self_v);
    if (self->rx_cb_ && n > 0) {
        self->rx_cb_({d, n});
    }
}

void UsbCdcTransport::rx_task(void* arg) {
    auto* self = static_cast<UsbCdcTransport*>(arg);
    // 256 statt 64 je Lesevorgang: mit dem groesseren Ringpuffer soll die
    // Task ihn auch in wenigen Schritten leeren koennen.
    uint8_t buf[256];
    while (self->running_) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(50));
        if (n > 0) {
            if (auto fn = self->rx_filter_.load(std::memory_order_acquire); fn) {
                // Filter entscheidet was ans Frontend weitergereicht wird.
                fn(buf, static_cast<size_t>(n),
                   self->rx_filter_user_.load(std::memory_order_relaxed),
                   &UsbCdcTransport::emit_to_frontend_, self);
            } else if (self->rx_cb_) {
                self->rx_cb_({buf, static_cast<size_t>(n)});
            }
        }
    }
    vTaskDelete(nullptr);
}

} // namespace culfw32
