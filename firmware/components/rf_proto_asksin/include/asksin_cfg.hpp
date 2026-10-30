// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include <cstdint>

namespace culfw32 {

// CC1101-Register-Konfig fuer AskSin (HomeMatic/BidCos, eq3 868.3 MHz, FSK ~10 kbps).
//
// Quelle: a-culfw `culfw/clib/rf_asksin.c:28` (ASKSIN_CFG[]). Byte-genau uebernommen.
// Sentinel: erstes Byte > 0x40 beendet die Tabelle (analog Moritz; a-culfw verwendet
// ein C-Array mit sizeof, wir terminieren explizit mit 0xFF).
//
// Format: Reihen von {register_addr, value} Paaren.
extern const uint8_t ASKSIN_CFG[52];

// FUP-Mode (Firmware-Update, hoehere Datarate ~50 kbps). Nicht in v1 verwendet,
// dokumentiert fuer spaetere `AR`-Befehl-Implementation.
// Quelle: a-culfw rf_asksin.c:57 (ASKSIN_UPDATE_CFG[]).
extern const uint8_t ASKSIN_UPDATE_CFG[24];

} // namespace culfw32
