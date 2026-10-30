// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "rf_protocol.hpp"
#include "cc1101_driver.hpp"
#include <span>

namespace culfw32 {

// FastRF-RX-Frame: variable length payload (PKTCTRL0=0x05 variable mode).
// Status-Bytes (RSSI/LQI) werden vom CC1101 appended (PKTCTRL1.APPEND_STATUS=1).
struct FastRfFrame {
    static constexpr size_t kMaxPayload = 255;
    uint8_t  len;
    uint8_t  data[kMaxPayload];
    uint8_t  rssi_raw;
    uint8_t  lqi_raw;
};

// TX-Payload: rohe bytes ohne Hex-Encoding (a-culfw fastrf_func liest in[2..len]
// direkt als Daten). Length-byte wird beim FIFO-Write vorne angefuegt
// (CC1101 variable-length packet mode).
struct FastRfTxFrame {
    uint8_t len;
    uint8_t data[FastRfFrame::kMaxPayload];
};

class FastRfRF final : public RfProtocol {
public:
    ProtocolId           id() const override { return ProtocolId::FastRf; }
    const CC1101Profile& profile() const override;

    esp_err_t start(uint8_t slot_id, CC1101Driver& drv) override;
    esp_err_t stop()  override;
    esp_err_t tx(const void* typed_payload, size_t payload_size) override;
    void      on_rx(RxCallback cb) override { rx_cb_ = std::move(cb); }

    // a-culfw `fastrf_on` Flag: 0 = aus, 1 = aktiv. Frontend braucht das fuer
    // eventuelle Mode-Echos (analog `native_on`).
    bool active() const { return active_; }

private:
    static void rx_task_trampoline(void* arg);
    void        rx_task_loop();
    static void IRAM_ATTR gdo_isr(void* arg);

    CC1101Driver* drv_         = nullptr;
    uint8_t       slot_id_     = 0;
    RxCallback    rx_cb_;
    void*         task_handle_ = nullptr;
    volatile bool task_running_ = false;
    volatile bool          active_      = false;
};

} // namespace culfw32
