// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
//
// CC1101-Konfiguration fuer wMBus Modus S und T (Empfang; die Abweichungen
// fuers Senden setzt wmbus_rf.cpp) und die FIFO-Konstanten des Funkpfads.
// Herleitung der Registerwerte: wmbus_cfg.cpp.

#pragma once

#include <cstdint>

namespace culfw32::wmbus {

// Paare {Adresse, Wert}; ein Adressbyte > 0x40 beendet die Tabelle (push_cfg
// in wmbus_rf.cpp). Vor dem Laden setzt wmbus_rf den Chip per SRES zurueck,
// nicht genannte Register behalten also ihren Reset-Wert.
extern const uint8_t WMBUS_S_CFG[];
extern const uint8_t WMBUS_T_CFG[];

// FIFO-Schwellen und Paketsteuerung des Funkpfads (a-culfw rf_mbus.h).
constexpr uint8_t  RX_FIFO_THRESHOLD       = 0x07;  // 32 bytes free / 32 bytes used
constexpr uint8_t  RX_FIFO_START_THRESHOLD = 0x00;  // 4 bytes free / 60 bytes used (init)
constexpr uint8_t  RX_AVAILABLE_FIFO       = 32;
constexpr uint8_t  TX_FIFO_THRESHOLD       = 0x07;
constexpr uint8_t  TX_AVAILABLE_FIFO       = 32;
constexpr uint8_t  TX_FIFO_SIZE            = 64;
constexpr uint16_t MAX_FIXED_LENGTH        = 256;
constexpr uint8_t  PKTCTRL0_FIXED          = 0x00;
constexpr uint8_t  PKTCTRL0_INFINITE       = 0x02;

}  // namespace culfw32::wmbus
