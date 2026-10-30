// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "transport.hpp"
#include <atomic>
#include <cstdint>

namespace culfw32 {

class HbrfethListener;

// Virtual Transport Wrapper fuer eine HB-RF-ETH-UDP-Connection. Mappt das
// Transport-Interface (start/stop/write/on_rx) auf den UDP-Tunnel:
//   - write(bytes)   → Listener packt in Type-7-UDP-Paket
//   - on_rx(cb)      → Listener ruft Callback bei eingehendem Type-7-Paket
class HbrfethVirtualTransport final : public Transport {
public:
    HbrfethVirtualTransport(HbrfethListener* listener, uint32_t ip, uint16_t port);

    esp_err_t start() override { return ESP_OK; }
    esp_err_t stop()  override { return ESP_OK; }
    esp_err_t write(std::span<const uint8_t> data) override;
    void      on_rx(RxCallback cb) override { rx_cb_ = std::move(cb); }
    const char* name() const override { return "hbrfeth-udp"; }

    uint32_t client_ip()   const { return ip_; }
    uint16_t client_port() const { return port_; }

    // Vom HbrfethListener aufgerufen bei Type-7-Frame.
    void deliver_rx(std::span<const uint8_t> bytes);

private:
    HbrfethListener* listener_ = nullptr;
    uint32_t         ip_   = 0;
    uint16_t         port_ = 0;
    RxCallback       rx_cb_;
};

} // namespace culfw32
