// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "rf_protocol.hpp"
#include "cc1101_driver.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic>
#include <span>

namespace culfw32 {

// Typed AskSin-RX-Frame. Lebensdauer = RxCallback-Aufruf. Wer ihn ueber den
// Aufruf hinaus braucht, kopiert. Inhalt ist bereits DECRYPTED (Crypt-Decoder
// im RX-Task angewandt, byte-genau a-culfw rf_asksin.c:175ff).
struct AskSinFrame {
    static constexpr size_t kMaxPayload = 64;  // a-culfw MAX_ASKSIN_MSG=30 (50 mit FUP), mit Reserve
    uint8_t  len;                              // payload length (= byte 0 des Frames, decrypted)
    uint8_t  data[kMaxPayload];                // [0]=len, [1..len]=decrypted payload
    uint8_t  rssi_raw;                         // CC1101 RSSI status byte
    uint8_t  lqi_raw;                          // CC1101 LQI status byte
};

// TX-Payload (Klartext). MoritzTxFrame-Aequivalent. AskSin-Crypt wird in tx()
// VOR dem FIFO-Write angewandt.
struct AskSinTxFrame {
    uint8_t len;
    uint8_t data[AskSinFrame::kMaxPayload];    // [0]=len, [1..len]=Klartext-Payload
};

class AskSinRF final : public RfProtocol {
public:
    ProtocolId           id() const override { return ProtocolId::AskSin; }
    const CC1101Profile& profile() const override;

    esp_err_t start(uint8_t slot_id, CC1101Driver& drv) override;
    esp_err_t stop()  override;
    esp_err_t tx(const void* typed_payload, size_t payload_size) override;
    void      on_rx(RxCallback cb) override { rx_cb_ = std::move(cb); }

    // Letzter TX-Error-Code fuer Frontend-Output. CCA-Timeout ist a-culfw-konform
    // `ERR:CCA\r\n`; die anderen Pfade lehnen sich an Moritz' ZERR1/2/3 an.
    enum class TxError : uint8_t { None, NotInRx, CcaTimeout, NoReturnRx, FifoNotEmpty };
    TxError last_tx_error()    const { return last_tx_err_; }
    uint8_t last_tx_marcstate() const { return last_tx_marc_; }

    // Letzter on-air TX-Header — Klartext-Bytes des Frames der zuletzt an den
    // CC1101 uebergeben wurde (vor encrypt). Antwortet "ist APP_SET_HMID
    // wirklich aktiv?" und aehnliche Fragen ohne RF-Sniffer.
    struct LastTx {
        uint32_t src_hmid;   // Bytes 4..6, 24-bit. 0 = noch keine TX.
        uint32_t dst_hmid;   // Bytes 7..9, 24-bit.
        uint8_t  cnt;        // Byte 1, on-air message counter.
        bool     valid;      // false bevor erstmals ge-tx't.
    };
    static LastTx last_tx() {
        LastTx l{};
        l.src_hmid = s_last_tx_src_hmid_.load(std::memory_order_relaxed);
        l.dst_hmid = s_last_tx_dst_hmid_.load(std::memory_order_relaxed);
        l.cnt      = s_last_tx_cnt_.load(std::memory_order_relaxed);
        l.valid    = s_last_tx_valid_.load(std::memory_order_relaxed);
        return l;
    }

    // Crypt/Decrypt-Helfer (oeffentlich fuer Tests). Modifizieren in-place.
    // Byte-genau a-culfw `asksin_send` (encrypt) und `rf_asksin_task` (decrypt).
    static void encrypt(uint8_t* msg);
    static void decrypt(uint8_t* msg);

    // FUP-Mode (Firmware-Update via OTA fuer HomeMatic-Geraete) — `AR` statt `Ar`.
    // Bei aktivem Mode wird beim naechsten start() zusaetzlich ASKSIN_UPDATE_CFG
    // ueber das normale ASKSIN_CFG drueber geschrieben (hoehere Datarate ~50 kbps,
    // andere AGC-/IF-Werte). Byte-genau a-culfw rf_asksin.c:104-110.
    void set_update_mode(bool on) { update_mode_ = on; }
    bool update_mode() const      { return update_mode_; }

    // TX-only-Bind: setzt drv_/slot_id_ ohne Slot-Init und ohne RX-Task. Wird
    // von HmDualRF benutzt, das den CC1101 selbst initialisiert (HMIP_CFG) und
    // nur die AskSin-TX-Logik (encrypt + CCA + Burst-Preamble) leihen will.
    void bind_for_tx(uint8_t slot_id, CC1101Driver& drv) {
        slot_id_ = slot_id;
        drv_     = &drv;
        // `enabled_` MUSS hier mit, sonst weist tx() den geliehenen Sendepfad
        // ab: seit 0.1.842 gatet tx() ueber das Flag statt ueber den Zeiger.
        // Heute ruft das niemand (der Kommentar oben nennt HmDualRF, die Stelle
        // existiert nicht) — genau deshalb waere es beim ersten Aufrufer ein
        // stiller Fehlschlag geworden statt eines Compilerfehlers.
        enabled_ = true;
    }

private:
    static void rx_task_trampoline(void* arg);
    void        rx_task_loop();
    void        drain_fifo();        // alle fertigen Rahmen: read, SRX, decrypt + callback
    void        recover_rx_(bool heartbeat);   // Chip aus IDLE/Overflow zurueck nach RX
    void        rearm_rx_();         // SRX aus IDLE, aber nicht waehrend tx()
    static void IRAM_ATTR gdo2_isr(void* arg);

    // Haelt tx() fuer die ganze Sendung; der RX-Task liest, flusht (SIDLE/SFRX)
    // und strobt SRX nur, wenn er ihn ohne Warten bekommt, und gibt ihn vor dem
    // Callback frei. Ein SIDLE oder SRX nach dem STX beendet die Sendung
    // (SWRS061I 19.4, S. 52). Angelegt in start(); ohne (bind_for_tx) ungesperrt.
    SemaphoreHandle_t chip_mtx_ = nullptr;

    CC1101Driver* drv_         = nullptr;

    // true zwischen start() und stop().  Der Treiberzeiger bleibt dabei

    // immer gueltig (Wert-Member des Slots); gegated wird ueber dieses Flag.

    // volatile: gesetzt vom Frontend-Task, gelesen vom RX-Task (Prio 8).

    volatile bool enabled_ = false;
    uint8_t       slot_id_     = 0;
    RxCallback    rx_cb_;
    void*         task_handle_ = nullptr;
    volatile bool task_running_ = false;

    TxError last_tx_err_ = TxError::None;
    uint8_t last_tx_marc_ = 0;
    bool    update_mode_ = false;

    // Diagnostik-State ist process-global: HmDualRF haelt eine zweite
    // AskSinRF-Instanz als TX-Helper, beide tx-Pfade landen am selben CC1101.
    // Statische Atomics → "letzte on-air TX" ist eindeutig, egal welche Instanz.
    static std::atomic<uint32_t> s_last_tx_src_hmid_;
    static std::atomic<uint32_t> s_last_tx_dst_hmid_;
    static std::atomic<uint8_t>  s_last_tx_cnt_;
    static std::atomic<bool>     s_last_tx_valid_;
};

} // namespace culfw32
