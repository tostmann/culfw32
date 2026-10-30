// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "transport.hpp"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace culfw32 {

// UART-Transport (default UART0, 38400 8N1).
//
// Use-Case CULFW32-Hardware: USB-C-SBU1/SBU2-Pins fuehren UART0 nach aussen,
// damit ein zweiter Mikrocontroller die culfw-Daten direkt seriell mitlesen kann
// — parallel zum USB-CDC-Console-Path. Frontend bindet *beide* Transports und
// echoed RX-Frames auf alle gebundenen Senken.
//
// UART0 ist frei nutzbar weil ESP_CONSOLE_USB_SERIAL_JTAG primary console ist
// (sdkconfig.defaults). ESP_LOG geht ueber USB-CDC, nicht ueber UART0 — der
// SBU-Stream sieht ausschliesslich culfw-Frames.
struct UartTransportConfig {
    uart_port_t port      = UART_NUM_0;
    int         baud_rate = 38400;
    int         pin_tx    = -1;     // -1 = default
    int         pin_rx    = -1;
    int         rx_buf_sz = 1024;
    int         tx_buf_sz = 1024;
};

class UartTransport final : public Transport {
public:
    explicit UartTransport(const UartTransportConfig& cfg) : cfg_(cfg) {}

    esp_err_t   start() override;
    esp_err_t   stop()  override;
    esp_err_t   write(std::span<const uint8_t> data) override;
    void        on_rx(RxCallback cb) override;
    const char* name() const override { return "uart"; }

private:
    static void rx_task(void* arg);

    UartTransportConfig cfg_;
    RxCallback          rx_cb_;
    TaskHandle_t        task_handle_ = nullptr;
    bool                running_     = false;
};

} // namespace culfw32
