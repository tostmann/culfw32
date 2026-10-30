// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "rf_protocol.hpp"
#include "cc1101_driver.hpp"

namespace culfw32 {

// rf_native: RX-only Sniffer für RFM12-basierte Protokolle (IT+, PCA301).
// Frame ist roher RXFIFO-Inhalt (FIFOTHR-Bytes; 12 Mode 1/2, 32 Mode 3).
// Kein Length-Byte, kein CRC — PKTCTRL0=0x02 (infinite packet length).
struct NativeFrame {
    static constexpr size_t kMaxPayload = 64;
    uint8_t  mode;                 // 1..3 (Empfangs-Mode zur Zeit des Frames)
    uint8_t  len;                  // tatsaechliche Bytezahl im FIFO (= FIFOTHR)
    uint8_t  data[kMaxPayload];
    uint8_t  rssi_raw;
    uint8_t  lqi_raw;
};

class NativeRF final : public RfProtocol {
public:
    ProtocolId           id() const override { return ProtocolId::Native; }
    const CC1101Profile& profile() const override;

    esp_err_t start(uint8_t slot_id, CC1101Driver& drv) override;
    esp_err_t stop()  override;

    // RX-only — TX existiert in a-culfw nicht. Returns INVALID_STATE.
    esp_err_t tx(const void* /*payload*/, size_t /*size*/) override {
        return ESP_ERR_INVALID_STATE;
    }
    void on_rx(RxCallback cb) override { rx_cb_ = std::move(cb); }

    // Mode 1..3 vor start() setzen — wird beim Init in den Chip gepushed.
    // Default: Mode 1 (IT+ 17.241 kbps).
    void    set_mode(uint8_t mode) { mode_ = (mode >= 1 && mode <= 3) ? mode : 1; }
    uint8_t mode() const           { return mode_; }

    // a-culfw `native_on` Flag: 0 wenn Reception aus, sonst Mode-Index. Frontend
    // braucht das fuer den `N` (leer) Query-Pfad — byte-genau a-culfw rf_native.c:285.
    uint8_t native_on() const      { return active_ ? mode_ : 0; }

private:
    static void rx_task_trampoline(void* arg);
    void        rx_task_loop();
    static void IRAM_ATTR gdo2_isr(void* arg);

    CC1101Driver* drv_         = nullptr;
    uint8_t       slot_id_     = 0;
    uint8_t       mode_        = 1;
    RxCallback    rx_cb_;
    void*         task_handle_ = nullptr;
    volatile bool task_running_ = false;
    volatile bool          active_      = false;
};

} // namespace culfw32
