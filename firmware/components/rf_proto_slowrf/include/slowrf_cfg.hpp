// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 Dirk Tostmann
// Register tables taken from a-culfw (https://github.com/tostmann/a-culfw) clib/cc1100.c,
// GNU GPL version 2 or (at your option) any later version; the original
// file carries no individual copyright notice.
// Modified for CULFW32 (ESP-IDF port), Dirk Tostmann, 2026.

#pragma once

#include <cstdint>

namespace culfw32 {

// CC1101-Register-Konfig für SlowRF (OOK/ASK 868 MHz async serial).
//
// Quelle: a-culfw `culfw/clib/cc1100.c:55..107` (CC1100_CFG[EE_CC1100_CFG_SIZE]).
// Byte-genau übernommen — Format {addr, value} Paare, Sentinel 0xFF.
//
// PKT_FORMAT=3 (Async serial). Das heisst: TX/RX laufen via GDO0 als rohe
// Bit-Stream-Leitung; PKTLEN/CRC/sync sind irrelevant (CC1101 demoduliert nur).
// Der Decoder sitzt in Software (Bucket-State-Machine) — wie a-culfw.
extern const uint8_t SLOWRF_CFG[];

// Anzahl Register-Paare (ohne Sentinel). Compile-time konstant für Iteration.
constexpr unsigned SLOWRF_CFG_PAIRS = 41;

} // namespace culfw32
