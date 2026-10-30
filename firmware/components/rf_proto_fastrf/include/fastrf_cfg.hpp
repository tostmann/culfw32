// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 Dirk Tostmann
// Register tables taken from a-culfw (https://github.com/tostmann/a-culfw) clib/cc1100.c,
// GNU GPL version 2 or (at your option) any later version; the original
// file carries no individual copyright notice.
// Modified for CULFW32 (ESP-IDF port), Dirk Tostmann, 2026.

#pragma once

#include <cstdint>

namespace culfw32 {

// CC1101-Konfig fuer FastRF (CC1101-P2P high-rate, 250 kbps GFSK).
// Quelle: a-culfw `culfw/clib/cc1100.c:165` (FASTRF_CFG[]). Byte-genau.
// 868.3 MHz, GFSK, 250 kbps, BW 203 kHz, dev 127 kHz, CRC autoflush,
// PKTCTRL1=0x08 (APPEND_STATUS=1, kein ADDR check), PKTCTRL0=0x05
// (variable len, CRC enable, packet mode).
//
// Sentinel: erstes Reg-byte > 0x40 beendet die Tabelle. Das letzte Reg ist
// 0x28 (RCCTRL0). Wir terminieren nach 0x28 mit 0xFF.
extern const uint8_t FASTRF_CFG[80];

} // namespace culfw32
