// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
//
// Contains code ported from SIGNALduino (https://github.com/RFD-FHEM/SIGNALDuino):
//   signalDecoder.h/.cpp  Pattern Decoder Library V3
//                         2014-2015 N.Butzek, S.Butzek; 2015-2018 S.Butzek;
//                         2018-2020 S.Butzek, HomeAutoUser
// GNU GPL version 3 or (at your option) any later version.
// Modified for CULFW32 (ESP-IDF port), Dirk Tostmann, 2026.

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

namespace culfw32 {

// SIGNALduino-Wire-Format-Frame, ready zum Senden an alle Clients.
// Ein RMT-Burst → ein Frame (MS/MU/MC). Der Frontend-Renderer haengt nur
// noch \r\n an und broadcastet.
struct SignalduinoFrame {
    char     ascii[768];   // genug fuer 8 Pattern-Slots + 512 Daten-Chars + Header
    uint16_t len;
    int8_t   rssi_dbm;
};

// TX-Payload fuer SignalduinoRF::tx — entspricht einem geparsten SR-Frame.
// Das Frontend baut diese Struct aus dem `SR;R=...;P0=...;...;D=...;`-Befehl
// und reicht sie an `rf_manager->tx()` weiter. send_raw byte-genau Real-SD-FW
// send.h:17..43: pro D-Char einen Halbpuls (Vorzeichen aus buckets[idx],
// Dauer = abs(buckets[idx])); kein bit-pair sondern Edge-Sequenz.
struct SignalduinoTxFrame {
    int16_t  buckets[8];   // signed µs, [0..7] entspricht D-Char '0'..'7'
    uint8_t  npat;         // wieviele Slots gefuellt sind (rest = 0 reserved)
    uint8_t  repeats;      // R=<n>; mind. 1
    uint16_t dlen;         // Anzahl Halbpulse in `data`
    char     data[512];    // D=<index-string>, '0'..'7'
};

// SIGNALduino-Pulse-Capture: zweite OOK-Klasse parallel zu SlowRfRF, exklusiv
// auf demselben CC1101-Slot (Mode-Switch). Liest GDO2-Demod-Output via RMT-RX
// (1 µs Aufloesung), klassifiziert Pulse in Pattern-Slots, emittiert
// MU-Frames (v1) bzw. MS/MC (Folge-Iteration).
class SignalduinoRF final : public RfProtocol {
public:
    ProtocolId           id() const override { return ProtocolId::PulseCapture; }
    const CC1101Profile& profile() const override;

    esp_err_t start(uint8_t slot_id, CC1101Driver& drv) override;
    esp_err_t stop()  override;
    esp_err_t tx(const void* typed_payload, size_t payload_size) override;
    void      on_rx(RxCallback cb) override { rx_cb_ = std::move(cb); }

    // Sendepegel aus dem EEPROM, siehe SlowRfRF::set_pa_provider (gleicher OOK-Pfad).
    void set_pa_provider(PaTableProvider p) { pa_provider_ = std::move(p); }

private:
    // Pattern-Slot fuer den Pulse-Klassifikator. Mirror SignalDetectorClass.pattern[]
    // + .histo[] aus signalDecoder.h. Public weil Pattern in finalize_frame_'s
    // Parameter-Liste auftaucht — alles andere bleibt private logic in der .cpp.
public:
    struct Pattern {
        int32_t  value_us;   // signed: positiv = HIGH, negativ = LOW
        uint16_t count;      // histo[i]: Anzahl Treffer
    };

private:
    static bool IRAM_ATTR rmt_rx_done_cb(rmt_channel_handle_t,
                                         const rmt_rx_done_event_data_t*, void*);
    static void rx_task_trampoline(void* arg);
    void        rx_task_loop();

    esp_err_t setup_rmt_rx_();      // RMT-RX-Channel anlegen + aktivieren (rx_mtx_)
    esp_err_t teardown_rmt_rx_();   // (rx_mtx_)
    // Nur der RX-Task: CC1101 in RX strobe + RMT-Receive armieren (rx_mtx_).
    esp_err_t arm_rx_(uint32_t& armed_gen);
    void      resume_rx_();         // nach TX: Kanal neu anlegen + RX-Task wecken
    void      wake_rx_task_();      // Weck-Batch (gen 0) an den RX-Task
    esp_err_t setup_rmt_tx_();
    esp_err_t teardown_rmt_tx_();

    void process_symbols_(const rmt_symbol_word_t* syms, size_t n);
    void finalize_frame_(Pattern* pats, int npat, int8_t* data, int dlen);

    CC1101Driver* drv_     = nullptr;

    // true zwischen start() und stop().  Der Treiberzeiger bleibt dabei

    // immer gueltig (Wert-Member des Slots); gegated wird ueber dieses Flag.

    // volatile: gesetzt vom Frontend-Task, gelesen vom RX-Task (Prio 7).

    volatile bool enabled_ = false;
    uint8_t       slot_id_ = 0;
    RxCallback    rx_cb_;
    PaTableProvider pa_provider_;   // EEPROM-PA-Tabelle, gesetzt von main

    rmt_channel_handle_t rmt_rx_chan_ = nullptr;
    rmt_channel_handle_t rmt_tx_chan_ = nullptr;
    rmt_encoder_handle_t rmt_tx_enc_  = nullptr;
    static constexpr size_t kRxBufferSyms = 256;
    rmt_symbol_word_t       rx_buffer_[kRxBufferSyms]{};
    static constexpr size_t kTxBufferSyms = 256;     // 256 syms = 512 Halbpulse
    rmt_symbol_word_t       tx_buffer_[kTxBufferSyms]{};

    struct RxBatch {
        size_t   n_syms;      // Anzahl gültiger Symbole im rx_buffer_
        uint32_t gen;         // Kanal-Generation des Empfangs; 0 = nur Wecken (start/tx)
    };
    // Wie SlowRfRF: schuetzt Anlegen/Loeschen des RX-Kanals (Frontend) gegen
    // das Armieren im RX-Task -- RMT-Kanalfunktionen sind laut IDF-Doku nicht
    // thread-sicher.  rx_gen_ zaehlt die Kanal-Generationen (nie 0).
    SemaphoreHandle_t rx_mtx_ = nullptr;
    std::atomic<uint32_t> rx_gen_{0};
    QueueHandle_t rx_queue_   = nullptr;
    TaskHandle_t  task_handle_ = nullptr;
    std::atomic<bool> task_running_{false};

    // Frame-Member statt Stack — der RX-Task hat ~4 KB Stack, ein 768-byte
    // ascii-Buffer plus Pattern-Tabelle und Call-Frames wird sonst eng.
    SignalduinoFrame frame_{};
};

} // namespace culfw32
