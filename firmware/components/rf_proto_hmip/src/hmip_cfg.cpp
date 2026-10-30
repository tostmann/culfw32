// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "hmip_cfg.hpp"

namespace culfw32 {

// Byte-genau aus dem SPI-Mitschnitt am HmIP-RFUSB (eigene Messung, 01.05.2026).
// Burst-Schreibung 0x00..0x2E. Hier als {addr, value}-Liste mit Sentinel 0xFF
// (Sentinel-Konvention der anderen RF-Klassen — write_reg pro Paar reicht,
// keine Burst-Performance-Optimierung in dieser PHY-only Bringup-Stufe).
const uint8_t HMIP_CFG[96] = {
    0x00, 0x07,  // IOCFG2  — CULFW32-OVERRIDE: Asserts on RX with CRC OK
                 //           (GDO2-IRQ-Pin, identisch zu AskSin/Moritz).
                 //           HmIP-RFUSB Original: 0x0E (Carrier Sense) —
                 //           air-verhalten unveraendert, IOCFG ist reines
                 //           GPIO-Routing.
    0x01, 0x2E,  // IOCFG1  — CULFW32-OVERRIDE: 3-state (Chip-Default).
                 //           HmIP-RFUSB Original: 0x0E (Carrier Sense).
                 //           GDO1 IST der SO-Pin; jeder Wert ausser 0x2E
                 //           treibt ihn auch bei CSn high (SWRS061I §11.2)
                 //           und damit MISO gegen den W5500 an J1.
                 //           Gelesen wird GDO1 hier nirgends.
    0x02, 0x06,  // IOCFG0  — Sync-Word-Detect (HmIP-RFUSB nutzt das,
                 //           wir pollen GDO2-CRC-OK). Dokumentiert.
    0x03, 0x0D,  // FIFOTHR
    0x04, 0xE9,  // SYNC1
    0x05, 0xCA,  // SYNC0   --> 16-bit Sync 0xE9CA (BidCoS-identisch)
    0x06, 0x3D,  // PKTLEN  --> max 61 Byte
    0x07, 0x0C,  // PKTCTRL1 — APPEND_STATUS=1, kein Adress-Filter
    0x08, 0x45,  // PKTCTRL0 — Whitening ON, var-len, CRC ON
    0x09, 0x00,  // ADDR
    0x0A, 0x00,  // CHANNR
    0x0B, 0x06,  // FSCTRL1
    0x0C, 0x00,  // FSCTRL0
    0x0D, 0x21,  // FREQ2
    0x0E, 0x65,  // FREQ1
    0x0F, 0x6A,  // FREQ0   --> 868.305 MHz
    0x10, 0xC8,  // MDMCFG4
    0x11, 0x93,  // MDMCFG3 --> ~10 kbps
    0x12, 0x03,  // MDMCFG2 --> 2-FSK, SYNC_MODE=3: Sync-Wort DOPPELT (4 B)
    0x13, 0x22,  // MDMCFG1 --> NUM_PREAMBLE=010: 4 B Praeambel
    0x14, 0xF8,  // MDMCFG0
    0x15, 0x34,  // DEVIATN --> ~19 kHz
    0x16, 0x07,  // MCSM2
    // MCSM1 = 0x0F: CCA_MODE(5:4)=0 "Always", RXOFF(3:2)=3, TXOFF(1:0)=3.
    // ⚠️ CCA_MODE 0 = KEINE Kanalpruefung, wir senden immer los (Datenblatt
    // SWRS061I S. 81). Wert aus dem Original-HmIP-RFUSB; AskSin nutzt 0x33
    // (CCA_MODE 3 = mit Pruefung). Die Sendezeit begrenzt deshalb das Konto
    // in hmip_rf.cpp, nicht der Funkchip.
    0x17, 0x0F,  // MCSM1   --> OHNE CCA, TXOFF=RX (HmIP-RFUSB Original)
    0x18, 0x18,  // MCSM0
    0x19, 0x16,  // FOCCFG
    0x1A, 0x6C,  // BSCFG
    0x1B, 0x43,  // AGCCTRL2
    0x1C, 0x40,  // AGCCTRL1
    0x1D, 0x91,  // AGCCTRL0
    0x1E, 0x8E,  // WOREVT1
    0x1F, 0x2F,  // WOREVT0
    0x20, 0x78,  // WORCTRL
    0x21, 0x56,  // FREND1
    0x22, 0x10,  // FREND0
    0x23, 0xE9,  // FSCAL3
    0x24, 0x2A,  // FSCAL2
    0x25, 0x00,  // FSCAL1
    0x26, 0x1F,  // FSCAL0
    0x27, 0x41,  // RCCTRL1
    0x28, 0x00,  // RCCTRL0
    0x29, 0x59,  // FSTEST
    0x2A, 0x7F,  // PTEST
    0x2B, 0x3F,  // AGCTEST
    0x2C, 0x81,  // TEST2
    0x2D, 0x35,  // TEST1
    0x2E, 0x09,  // TEST0
    0xFF, 0x00,  // Sentinel
};

} // namespace culfw32
