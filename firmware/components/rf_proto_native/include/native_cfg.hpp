// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include <cstdint>

namespace culfw32 {

// CC1101-Konfig für rf_native (generischer FSK-RX-Sniffer fuer IT+/PCA301).
// Quelle: a-culfw `culfw/clib/rf_native.c:42` (NATIVE_CFG[]). Byte-genau.
// PKTCTRL0 = 0x02 = infinite packet length (kein CRC, kein length byte).
// Sentinel: erstes Reg-byte > 0x40 beendet die Tabelle (analog Moritz/AskSin).
extern const uint8_t NATIVE_CFG[46];

// Mode-Overrides ueber das Default-Cfg (Mode-Index 1..3).
// Quelle: a-culfw rf_native.c:80 (MODE_CFG[3][20]).
//   Mode 1: IT+ 17.241 kbps
//   Mode 2: IT+ 9.579 kbps
//   Mode 3: PCA 301 — 868.95 MHz, 6.631 kbps
extern const uint8_t NATIVE_MODE1_CFG[4];
extern const uint8_t NATIVE_MODE2_CFG[8];
extern const uint8_t NATIVE_MODE3_CFG[16];

// FIFOTHR-Schwellen pro Mode (in RX-FIFO-Bytes), aus a-culfw rf_native.c
// (Datenblatt SWRS061I §29 RXFIFO Threshold Tabelle 64).
//   FIFOTHR=2  -> 12 bytes  (Mode 1, 2)
//   FIFOTHR=7  -> 32 bytes  (Mode 3)
constexpr uint8_t NATIVE_FIFO_BYTES_MODE1 = 12;
constexpr uint8_t NATIVE_FIFO_BYTES_MODE2 = 12;
constexpr uint8_t NATIVE_FIFO_BYTES_MODE3 = 32;

} // namespace culfw32
