// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
// Portions derived from a-culfw (https://github.com/tostmann/a-culfw),
// GNU GPL version 2 or (at your option) any later version.

#pragma once

#include "rf_protocol.hpp"
#include "cc1101_driver.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/rmt_types.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include <atomic>
#include <span>

namespace culfw32 {

// SlowRF-Subprotokolle: 868-MHz-OOK-Familie (FS20, FHT, EM, KS300, HMS).
// In Schicht 4 als EIN RfProtocol gefasst — alle Decoder teilen sich GDO0/RMT-Backbone.
// Pro RX-Event wird ein typisierter Frame mit `kind`-Tag durchgereicht; Renderer
// dispatched dann auf 'F'/'T'/'K'/'E'/'H'-Output.  HMS ist nur Empfang (wie a-culfw).
struct SlowRfFrame {
    enum class Kind : uint8_t { FS20, FHT, KS300, EM, HMS };
    Kind     kind;
    uint8_t  len;             // payload length (= bytes in `data`)
    uint8_t  data[16];        // ausreichend für FS20/FHT/KS300/EM/HMS (max ~9 bytes + ck)
    uint8_t  rssi_raw;        // CC1101 RSSI register snapshot zum Frame-Ende
    bool     nibble_half;     // KS300: letztes Halb-Nibble; rendern mit nur 1 Hex
};

// TX-Payload (was an `tx(...)` reingereicht wird).
struct SlowRfTxFrame {
    enum class Kind : uint8_t { FS20, KS300, EM };
    Kind     kind;
    uint8_t  len;             // Anzahl Roh-Bytes (vor Parity/Checksum); EM erfordert exakt 9
    uint8_t  data[16];
    uint8_t  startcs;         // FS20=6, FHT=12 (cksum1 startwert); KS300/EM ungenutzt
    uint8_t  repeat;          // FS20/KS300/EM=3, FHT=2 — Anzahl Wiederholungen
};

class SlowRfRF final : public RfProtocol {
public:
    ProtocolId           id() const override { return ProtocolId::SlowRf; }
    const CC1101Profile& profile() const override;

    esp_err_t start(uint8_t slot_id, CC1101Driver& drv) override;
    esp_err_t stop()  override;
    esp_err_t tx(const void* typed_payload, size_t payload_size) override;
    void      on_rx(RxCallback cb) override { rx_cb_ = std::move(cb); }

    // tx_report-Bitmaske wie a-culfw (REP_KNOWN=1, REP_REPEATED=2, REP_BITS=4,
    // REP_MONITOR=8, REP_RSSI=0x20, REP_FHTPROTO=0x40). Default 0 = stille RX.
    void    set_tx_report(uint8_t mask) { tx_report_ = mask; }
    uint8_t tx_report() const { return tx_report_; }

    // Sendepegel aus dem EEPROM (a-culfw: ccInitChip laedt EE_CC1100_PA bei
    // jedem Mode-Start). Ohne Provider: fester Fallback 0 dBm ohne Ramping.
    void set_pa_provider(PaTableProvider p) { pa_provider_ = std::move(p); }

private:
    // RX-Pipeline — RMT-Edge-Capture + Bucket-Decoder.
    static bool IRAM_ATTR rmt_rx_done_cb(rmt_channel_handle_t,
                                         const rmt_rx_done_event_data_t*, void*);
    static void           rx_task_trampoline(void* arg);
    void                  rx_task_loop();

    esp_err_t setup_rmt_rx_();      // RMT-RX-Channel anlegen + aktivieren (rx_mtx_)
    esp_err_t teardown_rmt_rx_();   // (rx_mtx_)
    esp_err_t setup_rmt_tx_();      // RMT-TX-Channel — bei jedem TX neu erstellt
    esp_err_t teardown_rmt_tx_();
    // Nur der RX-Task: CC1101 in RX strobe + RMT-Receive armieren (rx_mtx_).
    esp_err_t arm_rx_(uint32_t& armed_gen);
    void      resume_rx_();         // nach TX: Kanal neu anlegen + RX-Task wecken
    void      wake_rx_task_();      // Weck-Batch (gen 0) an den RX-Task

    void process_symbols_(const rmt_symbol_word_t* syms, size_t n);
    void emit_frame_(const SlowRfFrame& f);

    CC1101Driver* drv_         = nullptr;

    // true zwischen start() und stop().  Der Treiberzeiger bleibt dabei

    // immer gueltig (Wert-Member des Slots); gegated wird ueber dieses Flag.

    // volatile: gesetzt vom Frontend-Task, gelesen vom RX-Task (Prio 7).

    volatile bool enabled_ = false;
    uint8_t       slot_id_     = 0;
    RxCallback    rx_cb_;

    rmt_channel_handle_t rmt_rx_chan_ = nullptr;
    rmt_channel_handle_t rmt_tx_chan_ = nullptr;
    rmt_encoder_handle_t rmt_tx_enc_  = nullptr;

    // RMT-Symbol-Buffer für die laufende rmt_receive(). Entkoppelt RMT-Done-IRQ
    // vom Decoder-Task: Producer (RMT-Done-CB) postet Anzahl + Kanal-Generation
    // an die Queue, Consumer (rx_task) verarbeitet und armiert RMT erneut.
    static constexpr size_t kRxBufferSyms = 256;          // 256 syms = 512 Pulses
    rmt_symbol_word_t       rx_buffer_[kRxBufferSyms]{};
    // TX-Symbol-Buffer als Member (nicht Stack) — der Caller (USB-CDC-RX-Task)
    // hat nur ~4 KB Stack, ein 1 KB sym-Buffer plus Call-Frames overflowt.
    static constexpr size_t kTxBufferSyms = 128;          // 128 syms reicht für FS20/FHT-Frames
    rmt_symbol_word_t       tx_buffer_[kTxBufferSyms]{};

    struct RxBatch {
        size_t   n_syms;      // Anzahl gültiger Symbole im rx_buffer_
        uint32_t gen;         // Kanal-Generation des Empfangs; 0 = nur Wecken (start/tx)
    };
    // Schuetzt den RX-Kanal: Anlegen/Loeschen (Frontend: start/stop/tx) gegen
    // das Armieren im RX-Task.  Laut IDF-Doku (RMT, "Thread Safety") sind nur
    // die Factory-Funktionen thread-sicher -- rmt_receive/rmt_disable/
    // rmt_del_channel auf demselben Kanal duerfen nicht aus zwei Tasks laufen.
    SemaphoreHandle_t rx_mtx_ = nullptr;
    // +1 bei jedem Anlegen des RX-Kanals (unter rx_mtx_), nie 0.  Der RX-Task
    // merkt sich, fuer welche Generation sein rmt_receive() laeuft, und armiert
    // neu, sobald sie wechselt oder der Empfang fertig ist.  Liest auch der
    // RX-done-Callback (ISR) fuer RxBatch::gen.
    std::atomic<uint32_t> rx_gen_{0};
    QueueHandle_t rx_queue_   = nullptr;
    TaskHandle_t  task_handle_ = nullptr;
    std::atomic<bool> task_running_{false};

    uint8_t  tx_report_   = 0;       // tx_report-Bitmaske
    PaTableProvider pa_provider_;    // EEPROM-PA-Tabelle, gesetzt von main

    // Repeat-Filter (a-culfw rf_receive.c:461..477). Default-X-Mask hat
    // REP_REPEATED=0 → wir suchen Duplikate innerhalb 304 ms und droppen sie,
    // damit `F12345600`-Triplets als 1 Zeile rauskommen statt 3. Bei REP_REPEATED=1
    // (X23 etc.) ist Filter aus, alle Repeats werden durchgereicht.
    static constexpr uint32_t kRepWindowMs = 304;   // a-culfw REPTIME=38 @125 Hz
    SlowRfFrame::Kind last_kind_      = SlowRfFrame::Kind::FS20;
    uint8_t           last_len_       = 0;
    uint8_t           last_buf_[16]   = {};
    uint32_t          last_emit_ms_   = 0;
};

} // namespace culfw32
