// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "hbrfeth_virtual_transport.hpp"
#include "hbrfeth_listener.hpp"

namespace culfw32 {

HbrfethVirtualTransport::HbrfethVirtualTransport(HbrfethListener* listener, uint32_t ip, uint16_t port)
    : listener_(listener), ip_(ip), port_(port) {}

esp_err_t HbrfethVirtualTransport::write(std::span<const uint8_t> data) {
    if (!listener_) return ESP_ERR_INVALID_STATE;
    listener_->send_frame(ip_, port_, data.data(), data.size());
    return ESP_OK;
}

void HbrfethVirtualTransport::deliver_rx(std::span<const uint8_t> bytes) {
    if (rx_cb_) rx_cb_(bytes);
}

} // namespace culfw32
