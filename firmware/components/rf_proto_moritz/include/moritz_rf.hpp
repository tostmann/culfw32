// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "rf_protocol.hpp"
#include "cc1101_driver.hpp"
#include <span>

namespace culfw32 {

// Typed Moritz-RX-Frame. Lebensdauer = RxCallback-Aufruf. Wer ihn ueber den
// Aufruf hinaus braucht, kopiert.
struct MoritzFrame {
    static constexpr size_t kMaxPayload = 64;  // a-culfw MAX_MORITZ_MSG = 30, mit Reserve
    uint8_t  len;                              // payload length (= byte 0 des Frames)
    uint8_t  data[kMaxPayload];                // [0]=len, [1..len]=payload
    uint8_t  rssi_raw;                         // CC1101 RSSI status byte (roh, hex)
    uint8_t  lqi_raw;                          // CC1101 LQI status byte
};

// TX-Payload (was ans `tx(...)` reingereicht wird).
// `data[0]` ist das Length-Byte (= Anzahl Payload-Bytes excl. Length selbst),
// `data[1..len]` ist Payload. Insgesamt schreibt MoritzRF len+1 Bytes ins TXFIFO.
struct MoritzTxFrame {
    bool    long_preamble;                     // Zs=true, Zf=false (TODO: noch nicht honoriert)
    uint8_t len;
    uint8_t data[MoritzFrame::kMaxPayload];    // [0]=len, [1..len]=payload
};

class MoritzRF final : public RfProtocol {
public:
    ProtocolId           id() const override { return ProtocolId::Moritz; }
    const CC1101Profile& profile() const override;

    esp_err_t start(uint8_t slot_id, CC1101Driver& drv) override;
    esp_err_t stop()  override;
    esp_err_t tx(const void* typed_payload, size_t payload_size) override;
    void      on_rx(RxCallback cb) override { rx_cb_ = std::move(cb); }

    // Pair-Konfig (RAM-only, wie a-culfw — Persistenz ist Frontend-Aufgabe via NVS).
    // Erwartet exakt 3 Bytes; weniger wird ignoriert.
    void set_auto_ack_addr     (std::span<const uint8_t> addr);
    void set_wall_thermostat   (std::span<const uint8_t> addr);

    // Letzter TX-Error-Code fuer Frontend-Output (analog a-culfw ZERR1/2/3).
    enum class TxError : uint8_t { None, NotInRx, NotInTx, NoReturnRx, FifoNotEmpty, Lovf };
    TxError last_tx_error()    const { return last_tx_err_; }
    uint8_t last_tx_marcstate() const { return last_tx_marc_; }

private:
    static void rx_task_trampoline(void* arg);
    void        rx_task_loop();
    void        drain_fifo();        // alle fertigen Rahmen: read + auto-ack + callback, dann SRX
    static void IRAM_ATTR gdo2_isr(void* arg);
    static void IRAM_ATTR gdo0_isr(void* arg);   // Sync erkannt (IOCFG0 0x46, fallende Flanke)
    void        latch_freqest_();                 // FREQEST waehrend des Pakets lesen

    void        handle_auto_ack(const MoritzFrame& f);
    esp_err_t   send_ack(const MoritzFrame& original);

    CC1101Driver* drv_         = nullptr;

    // true zwischen start() und stop().  Der Treiberzeiger bleibt dabei

    // immer gueltig (Wert-Member des Slots); gegated wird ueber dieses Flag.

    // volatile: gesetzt vom Frontend-Task, gelesen vom RX-Task (Prio 8).

    volatile bool enabled_ = false;
    uint8_t       slot_id_     = 0;
    RxCallback    rx_cb_;
    void*         task_handle_ = nullptr;
    volatile bool task_running_ = false;

    // FREQEST des laufenden Pakets, gelesen beim Sync (latch_freqest_), abgelegt
    // nach dem CRC (drain_fifo). Nur der RX-Task liest und schreibt.
    int8_t  pending_fe_       = 0;
    int64_t pending_fe_us_    = 0;
    bool    pending_fe_valid_ = false;

    uint8_t auto_ack_addr_[3] = {0, 0, 0};
    uint8_t wall_thermo_[3]   = {0, 0, 0};

    TxError last_tx_err_ = TxError::None;
    uint8_t last_tx_marc_ = 0;
};

} // namespace culfw32
