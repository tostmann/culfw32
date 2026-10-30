// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include <cstdint>

namespace culfw32 {

// CC1101-Register-Konfig fuer HmIP-Funk-Layer (eq3 HomeMatic IP, 868.305 MHz,
// 2-FSK ~10 kbps, Whitening on, var-len + CRC).
//
// Quelle: SPI-Mitschnitt am echten HmIP-RFUSB-Stick (EFM32 ↔ CC1101),
// eigene Messung vom 01.05.2026.
// Verifiziert (kein Modell-Wissen). Komplettes 47-Byte-Burst-Init,
// Adressen 0x00..0x2E.
//
// PHY-Eigenschaften (gegen CC1101 SWRS061I gegengerechnet):
//   Carrier      868.305 MHz (FREQ=21:65:6A)
//   Modulation   2-FSK       (MDMCFG2[6:4]=000)
//   Datenrate    9.99 kbps   (MDMCFG4=0xC8, MDMCFG3=0x93)
//   Deviation    19.04 kHz   (DEVIATN=0x34)
//   Sync         0xE9CA      (SYNC1:SYNC0)  -- byte-IDENTISCH zu BidCoS!
//   Packet       PKTCTRL0=0x45 = whitening ON, var-len, CRC ON
//                PKTCTRL1=0x0C = APPEND_STATUS=1, kein Adress-Filter
//   PATABLE[0]   0xC0        (~+10 dBm TX)
//
// Format der Tabelle: Reihen von {addr, value} Paaren, sentinel 0xFF.
// IOCFG2 ist gegenueber dem Stick-Original (0x0E = Carrier-Sense) auf
// 0x07 (Asserts on packet RX with CRC OK) umgesetzt — unser GDO2-Pin ist
// die IRQ-Quelle, identisch zur AskSin-Klasse. Air-Verhalten unveraendert
// (IOCFG-Pins sind reine GPIO-Routing-Bits, nicht modulierungsrelevant).
// IOCFG0 bleibt 0x06 (Sync-Detect), unbenutzt aber dokumentiert.
extern const uint8_t HMIP_CFG[96];

// PATABLE[0] = +10 dBm TX. Burst nach Reg-Init, vor SCAL/SRX.
constexpr uint8_t HMIP_PA_DEFAULT = 0xC0;

} // namespace culfw32
