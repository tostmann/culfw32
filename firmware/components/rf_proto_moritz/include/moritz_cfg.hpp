// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include <cstdint>

namespace culfw32 {

// CC1101-Register-Konfig fuer Moritz/MAX! (eq3 868.299 MHz, FSK ~10 kbps).
//
// Quelle: a-culfw `culfw/clib/rf_moritz.c:48` (MORITZ_CFG[60]). Byte-genau uebernommen.
// Sentinel: erstes Byte > 0x40 beendet die Tabelle (a-culfw verwendet 0xFF).
//
// Format: Reihen von {register_addr, value} Paaren. RF-Klasse iteriert ueber
// das Array und ruft pro Paar `cc1101.write_reg(addr, value)`.
//
// PROGMEM aus AVR ist auf ESP32 irrelevant — `const` reicht.
extern const uint8_t MORITZ_CFG[60];

} // namespace culfw32
