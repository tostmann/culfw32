// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "transport_uart.hpp"
#include "esp_log.h"

namespace culfw32 {

namespace {
constexpr const char* TAG = "tx-uart";
}

esp_err_t UartTransport::start() {
    if (running_) return ESP_OK;

    uart_config_t uc = {
        .baud_rate  = cfg_.baud_rate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags      = {},
    };

    if (auto e = uart_driver_install(cfg_.port, cfg_.rx_buf_sz, cfg_.tx_buf_sz,
                                     0, nullptr, 0); e != ESP_OK) {
        ESP_LOGE(TAG, "driver_install failed: %d", e);
        return e;
    }
    if (auto e = uart_param_config(cfg_.port, &uc); e != ESP_OK) {
        ESP_LOGE(TAG, "param_config failed: %d", e);
        uart_driver_delete(cfg_.port);
        return e;
    }
    if (cfg_.pin_tx >= 0 || cfg_.pin_rx >= 0) {
        if (auto e = uart_set_pin(cfg_.port, cfg_.pin_tx, cfg_.pin_rx,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            e != ESP_OK) {
            ESP_LOGE(TAG, "set_pin failed: %d", e);
            uart_driver_delete(cfg_.port);
            return e;
        }
    }

    running_ = true;
    // 6 KB analog usb-cdc-rx — ConfigRegistry-Iter mit char buf + std::string
    // braucht den Headroom; 3 KB triggerte Stack-Protection-Fault.
    if (xTaskCreate(&UartTransport::rx_task, "uart-rx", 6144, this, 5,
                    &task_handle_) != pdPASS) {
        running_ = false;
        uart_driver_delete(cfg_.port);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "started on UART%d @ %d 8N1 (TX=%d RX=%d)",
             cfg_.port, cfg_.baud_rate, cfg_.pin_tx, cfg_.pin_rx);
    return ESP_OK;
}

esp_err_t UartTransport::stop() {
    if (!running_) return ESP_OK;
    running_ = false;
    return ESP_OK;
}

esp_err_t UartTransport::write(std::span<const uint8_t> data) {
    if (!running_ || data.empty()) return ESP_OK;
    int sent = uart_write_bytes(cfg_.port, data.data(), data.size());
    return (sent < 0) ? ESP_FAIL : ESP_OK;
}

void UartTransport::on_rx(RxCallback cb) {
    rx_cb_ = std::move(cb);
}

void UartTransport::rx_task(void* arg) {
    auto* self = static_cast<UartTransport*>(arg);
    uint8_t buf[64];
    while (self->running_) {
        int n = uart_read_bytes(self->cfg_.port, buf, sizeof(buf), pdMS_TO_TICKS(50));
        if (n > 0 && self->rx_cb_) {
            self->rx_cb_({buf, static_cast<size_t>(n)});
        }
    }
    vTaskDelete(nullptr);
}

} // namespace culfw32
