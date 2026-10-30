// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "rf_protocol.hpp"
#include "cc1101_driver.hpp"

namespace culfw32 {

// HmIP-RX-Frame (PHY-only). Inhalt = was der CC1101 nach Whitening-Decode +
// CRC-Check liefert: <length-byte><payload[length]>. Status-Bytes (RSSI, LQI)
// kommen separat.
//
// Frame-Format (= was *innerhalb* der payload steht — Header-/Crypto-Layer)
// ist in v1 NICHT dekodiert. Sniffer-Mode: Hex-Dump zur Analyse.
struct HmipFrame {
    static constexpr size_t kMaxPayload = 64;  // PKTLEN=61 + Reserve
    uint8_t  len;                              // payload length (= byte 0 vom CC1101)
    uint8_t  data[kMaxPayload];                // [0]=len, [1..len]=payload
    uint8_t  rssi_raw;                         // CC1101 RSSI status byte
    uint8_t  lqi_raw;                          // CC1101 LQI/CRC_OK status byte
};


class HmipRF final : public RfProtocol {
public:
    ProtocolId           id() const override { return ProtocolId::Hmip; }
    const CC1101Profile& profile() const override;

    esp_err_t start(uint8_t slot_id, CC1101Driver& drv) override;
    esp_err_t stop()  override;
    esp_err_t tx(const void* typed_payload, size_t payload_size) override;
    void      on_rx(RxCallback cb) override { rx_cb_ = std::move(cb); }

    // Diese Klasse ist ein reiner Mithoerer. tx() liefert ESP_ERR_NOT_SUPPORTED,
    // und der Manager routet wegen supports_tx()==false nie hierher. HmIP-Senden
    // ist Sache von q-culfw.
    bool supports_tx(ProtocolId) const override { return false; }

private:
    static void rx_task_trampoline(void* arg);
    void        rx_task_loop();
    void        drain_fifo();        // alle fertigen Rahmen: read + decrypt + callback, dann SRX
    static void IRAM_ATTR gdo2_isr(void* arg);

    CC1101Driver* drv_         = nullptr;

    // true zwischen start() und stop().  Der Treiberzeiger bleibt dabei

    // immer gueltig (Wert-Member des Slots); gegated wird ueber dieses Flag.

    // volatile: gesetzt vom Frontend-Task, gelesen vom RX-Task (Prio 8).

    volatile bool enabled_ = false;
    uint8_t       slot_id_     = 0;
    RxCallback    rx_cb_;
    void*         task_handle_ = nullptr;
};

} // namespace culfw32
