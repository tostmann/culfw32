// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
//
// wMBus T1/S1 (Receive + Transmit) auf CC1101.
// Vendor-Pattern aus a-culfw culfw/clib/rf_mbus.c. RX-State-Machine
// (RXinfo state 0..3) + TX-FIFO-Refill-Loop, 1:1 portiert auf FreeRTOS.

#pragma once

#include "rf_protocol.hpp"
#include "cc1101_driver.hpp"
#include "wmbus_codec.hpp"

namespace culfw32 {

// Roh-Frame-Buffer-Groessen — a-culfw MBpacket[291] / MBbytes[584].
struct WmbusFrame {
    static constexpr size_t kMaxPacket = 291;
    uint8_t  mode;                 // 1=S, 2=T
    uint16_t len;                  // = packet_size(data[0])
    uint8_t  data[kMaxPacket];     // data[0]=L-Field, data[1..]=C/M/A/CI/...
    uint8_t  rssi_raw;
    uint8_t  lqi_raw;
};

struct WmbusTxFrame {
    uint8_t  mode;                 // 1=S, 2=T
    uint16_t len;                  // = packet_size(packet[0])
    uint8_t  packet[WmbusFrame::kMaxPacket];
};

class WmbusRF final : public RfProtocol {
public:
    ProtocolId           id() const override { return ProtocolId::Mbus; }
    const CC1101Profile& profile() const override;

    esp_err_t start(uint8_t slot_id, CC1101Driver& drv) override;
    esp_err_t stop()  override;
    esp_err_t tx(const void* typed_payload, size_t payload_size) override;
    void      on_rx(RxCallback cb) override { rx_cb_ = std::move(cb); }

    // a-culfw rf_mbus.c `rf_mbus_func` Sub-Befehle:
    //   br_off()      -> WMBUS_NONE, RADIO_MODE_NONE  (br / br<other>)
    //   set_mode_*()  -> WMBUS_SMODE/TMODE, RADIO_MODE_RX  (brs / brt)
    void set_mode_smode();
    void set_mode_tmode();
    void set_off();

    // Aktueller Mode fuer Status-Output (`b` ohne Args):
    //   0=OFF, 1=SMODE, 2=TMODE.
    uint8_t current_mode() const { return mbus_mode_; }
    bool    rx_active()    const { return radio_mode_ == RadioMode::Rx; }

private:
    enum class RadioMode : uint8_t { None = 0, Tx = 1, Rx = 2 };

    static void rx_task_trampoline(void* arg);
    static void tick_timer_cb(void* arg);          // taktet rx_task_loop per Notify
    void        rx_task_loop();
    void        rx_step();                         // RXinfo state-machine ein Tick

    // a-culfw rf_mbus.c:124 rf_mbus_init — schreibt Cfg-Tabelle, schaltet
    // Mode an, ggf. TX-spezifische Overrides.
    esp_err_t init_radio(uint8_t mbus_mode, RadioMode rmode);
    bool      enter_rx(bool force);                // rf_mbus_on
    esp_err_t tx_send_packet(const uint8_t* pkt, uint16_t pkt_size, uint8_t mbus_mode);

    CC1101Driver* drv_           = nullptr;

    volatile bool enabled_ = false;   // siehe stop()
    uint8_t       slot_id_       = 0;
    RxCallback    rx_cb_;
    void*         task_handle_   = nullptr;
    // esp_timer (µs-Aufloesung, unabhaengig von CONFIG_FREERTOS_HZ) — gibt dem
    // rx-Task seine 2-ms-Kadenz. Laeuft nur, solange wMBus die aktive Klasse
    // ist (start()/stop()). Begruendung im Kommentar an rx_task_loop().
    void*         tick_timer_    = nullptr;
    uint8_t       mbus_mode_     = 0;        // 0=NONE, 1=S, 2=T
    RadioMode     radio_mode_    = RadioMode::None;
    // Sperre Frontend <-> RX-Task (SemaphoreHandle_t, Anlage in start()).
    // Begruendung an OpLock in wmbus_rf.cpp.
    void*         op_mtx_        = nullptr;
    // RX-Takt.  T-Mode: Sync -> L-Feld (3 Byte, 240 us) -> PKTLEN muss
    // geschrieben sein, bevor der Rahmen endet (kurzer Rahmen ~2 ms nach dem
    // Sync).  500 us halten das ein; 2 ms taten es nicht (0/20 im T-Mode).
    static constexpr uint32_t kTickUs = 500;
    // Hoechstens so lange wartet case 2 im selben Takt auf die ersten 3 Byte.
    static constexpr uint16_t kL3WaitUs = 400;
    // Takte seit der letzten MARCSTATE-Pruefung im Warten auf Sync (case 1).
    static constexpr uint8_t kIdleCheckTicks = 40;   // 40 x 500 us = 20 ms
    uint8_t       idle_check_    = 0;
    // Fertiger Rahmen fuer rx_cb_, ausgegeben ausserhalb der Sperre.
    WmbusFrame    out_frame_{};
    int8_t        out_rssi_dbm_  = 0;
    uint32_t      out_ts_us_     = 0;
    bool          frame_ready_   = false;

    // RX-Statemachine (a-culfw RXinfoDescr).
    struct RxInfo {
        uint8_t  l_field   = 0;
        uint16_t length    = 0;
        uint16_t bytes_left = 0;
        uint8_t* p_byte    = nullptr;
        uint8_t  format    = 0;     // 0=infinite, 1=fixed
        bool     start     = true;
        bool     complete  = false;
        uint8_t  state     = 0;
    } rxi_{};

    // Encoded byte buffer (raw on-air fragments) und Roh-Paket-Buffer.
    static constexpr size_t kEncBufSize = 584;     // a-culfw MBbytes
    uint8_t enc_buf_[kEncBufSize] = {};
    uint8_t pkt_buf_[WmbusFrame::kMaxPacket] = {};
};

} // namespace culfw32
