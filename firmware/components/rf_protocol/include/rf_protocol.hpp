// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include <cstdint>
#include <cstddef>
#include <span>
#include <functional>
#include "esp_err.h"

namespace culfw32 {

class CC1101Driver;  // forward — Driver wird beim start() reingereicht

// Liefert die persistente PA-Tabelle (8 Bytes, a-culfw EE_CC1100_PA) und das
// zugehoerige FREND0 (EE_CC1100_CFG+0x22). RF-Klassen, die wie a-culfw
// `ccInitChip()` den Sendepegel aus dem EEPROM nehmen sollen (OOK: SlowRF,
// PulseCapture), rufen den Provider in start(). Rueckgabe false = keine gueltige
// Tabelle hinterlegt -> Klasse nimmt ihren eingebauten Fallback.
// AskSin/Moritz bleiben absichtlich draussen: a-culfw setzt dort einen festen
// PA-Wert, `x<idx>` gilt fuer FSK-Modes auch im Original nicht.
using PaTableProvider = std::function<bool(std::span<uint8_t, 8> pa, uint8_t& frend0)>;

// Identifiziert ein konkretes Protokoll. Manager benutzt das für Slot-Aktivierung.
enum class ProtocolId : uint8_t {
    AskSin = 1,    // HomeMatic / BidCos — vendor: rf_asksin.c
    Moritz,        // MAX! — vendor: rf_moritz.c
    SlowRf,        // FS20/HMS/KS300/EM/FHT culfw OOK-Decoder — vendor: rf_receive*.c
    PulseCapture,  // Signalduino-Style Bucket-Capture — vendor: rf_receive_bucket.c
    FastRf,        // CC1101-P2P high rate — vendor: fastrf.c
    Native,        // generische FSK-Brücke — vendor: rf_native.c
    Mbus,          // wMBus
    Hmip,          // HmIP raw RX (PHY-only Sniffer, RX-only; TX/Crypto sind q-culfw-Sache)
    Dudette,       // AVR-Funk-Bootloader (Dudette v2) — Bestandsliste + Transfer
    // ...
};

// CC1101-Register-Profil. RF-Klasse deklariert ihres statisch.
// Manager liest beim Mode-Wechsel und reconfiguriert den belegten Slot.
struct CC1101Profile {
    uint32_t freq_hz;        // Trägerfrequenz
    uint32_t datarate_bps;   // Symbolrate
    uint16_t deviation_hz;   // FSK-Hub (0 für OOK)
    enum class Modulation : uint8_t { OOK_ASK, FSK_2, FSK_4, MSK, GFSK } mod;
    uint8_t  sync_word[4];   // bis zu 32 bit
    uint8_t  sync_len;       // 0/16/32
    uint8_t  pa_table[8];    // PATABLE
    // Pointer auf Roh-Register-Tabelle für Sonderfälle (PulseCapture braucht GDO-Routing).
    const uint8_t (*raw_regs)[2]; // Array of {addr, value} pairs, 0xFF-terminated
};

// Typisiertes RX-Event. Konkrete Subklassen (z.B. AskSinFrame, HmsReading)
// liegen in der jeweiligen RF-Komponente — Frontend/Renderer bekommt
// einen ProtocolId + Pointer auf den protokoll-spezifischen Payload.
struct RxEvent {
    ProtocolId  proto;
    uint8_t     slot_id;       // welcher CC1101-Slot
    int8_t      rssi_dbm;
    uint32_t    timestamp_us;
    // Cast-by-ProtocolId. payload ist ein Pointer auf einen Stack-/Transient-
    // Puffer, gueltig NUR fuer die Dauer des synchronen Dispatch-Callbacks; wer
    // ihn ueber den Callback hinaus braucht, MUSS kopieren.
    const void* payload;
};

// Schicht 4: Protokoll-Engine. Wrapper um vendored a-culfw-C-Core.
// Singleton-pro-Protokoll (file-scope-State im C-Modul).
class RfProtocol {
public:
    using RxCallback = std::function<void(const RxEvent&)>;

    virtual ~RfProtocol() = default;

    virtual ProtocolId           id() const = 0;
    virtual const CC1101Profile& profile() const = 0;

    // Wird vom Manager gerufen, wenn der Slot dieser RF-Klasse zugewiesen ist.
    // RF-Klasse merkt sich slot_id (fuer RX-Event-Tagging) und Driver-Referenz
    // (fuer alle SPI-Zugriffe — Reg-Schreiben, Strobes, FIFO-Burst).
    virtual esp_err_t start(uint8_t slot_id, CC1101Driver& drv) = 0;
    virtual esp_err_t stop()  = 0;

    // TX. Payload-Typ ist protokoll-spezifisch — Frontend baut ihn via Parser zusammen.
    virtual esp_err_t tx(const void* typed_payload, size_t payload_size) = 0;

    virtual void on_rx(RxCallback cb) = 0;

    // Multi-Protocol-Klassen (z.B. HmDualRF) koennen TX fuer mehrere ProtocolIds
    // anbieten. Default: nur eigene id(). Manager nutzt das im aktiv-Slot-Pfad,
    // bevor er auf find_protocol(proto) faellt.
    virtual bool supports_tx(ProtocolId p) const { return p == id(); }
    virtual esp_err_t tx_for(ProtocolId, const void* payload, size_t size) {
        return tx(payload, size);
    }
};

} // namespace culfw32
