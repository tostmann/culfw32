// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 Dirk Tostmann
// Register tables taken from a-culfw (https://github.com/tostmann/a-culfw) clib/cc1100.c,
// GNU GPL version 2 or (at your option) any later version; the original
// file carries no individual copyright notice.
// Modified for CULFW32 (ESP-IDF port), Dirk Tostmann, 2026.

#include "fastrf_cfg.hpp"

namespace culfw32 {

// Byte-genau a-culfw cc1100.c:165 (FASTRF_CFG[]). Format = sequenzieller
// Reg-Index 0x00..0x28 (alle 41 Werte), KEINE Adress-Paare wie unsere
// anderen Tabellen (das ist der ccInitChip-Pfad in a-culfw — er schreibt
// linear ab Reg 0x00).
//
// Wir konvertieren in unser {addr,val}-Pair-Format fuer push_cfg, damit
// das Pattern konsistent zu Moritz/AskSin/Native bleibt.
const uint8_t FASTRF_CFG[80] = {
    0x00, 0x07,  // IOCFG2     INT when packet RX with CRC OK
    0x01, 0x2E,  // IOCFG1     3-State
    0x02, 0x05,  // IOCFG0     Interrupt in TX underflow
    0x03, 0x0D,  // FIFOTHR    TX:9 / RX:56
    0x04, 0xD3,  // SYNC1
    0x05, 0x91,  // SYNC0
    0x06, 0xFF,  // PKTLEN     infinite-ish (variable mode)
    // PKTCTRL1: byte-genau a-culfw FASTRF_CFG (cc1100.c:174) — Wert bleibt 0x08.
    // ACHTUNG, der a-culfw-Kommentar dort ist falsch und war es hier auch:
    // SWRS061I S.73 hat Bit 3 = CRC_AUTOFLUSH, Bit 2 = APPEND_STATUS. 0x08 heisst
    // also CRC_AUTOFLUSH=1 und APPEND_STATUS=0 (der Reset-Default 1 wird ueberschrieben),
    // nicht umgekehrt. Der Chip haengt hier KEINE RSSI/LQI-Statusbytes an — siehe die
    // Notiz im RX-Pfad (fastrf_rf.cpp). Aendern wuerde die Byte-Genauigkeit brechen.
    0x07, 0x08,  // PKTCTRL1   CRC_AUTOFLUSH=1, no ADDR check, APPEND_STATUS=0
    0x08, 0x05,  // PKTCTRL0   CRC enable, variable len, packet mode
    0x09, 0x00,  // ADDR
    0x0A, 0x00,  // CHANNR
    0x0B, 0x0C,  // FSCTRL1    305 kHz IF
    0x0C, 0x00,  // FSCTRL0
    0x0D, 0x21,  // FREQ2      868.3 MHz
    0x0E, 0x65,  // FREQ1
    0x0F, 0x6A,  // FREQ0
    0x10, 0x2D,  // MDMCFG4    BW 203 kHz
    0x11, 0x3B,  // MDMCFG3    DataRate 250 kbps
    0x12, 0x13,  // MDMCFG2    GFSK, Sync 30/32
    0x13, 0x22,  // MDMCFG1    Preamble 4 bytes
    0x14, 0xF8,  // MDMCFG0    Channel 200 kHz
    0x15, 0x62,  // DEVIATN    127 kHz
    0x16, 0x07,  // MCSM2
    0x17, 0x3F,  // MCSM1      CCA: RSSI&RX, RX->RX, TX->RX
    0x18, 0x18,  // MCSM0      auto-cal IDLE->TX/RX
    0x19, 0x1D,  // FOCCFG
    0x1A, 0x1C,  // BSCFG
    0x1B, 0xC7,  // AGCCTRL2
    0x1C, 0x00,  // AGCCTRL1
    0x1D, 0xB0,  // AGCCTRL0
    0x21, 0xB6,  // FREND1
    0x22, 0x11,  // FREND0
    0x23, 0xEA,  // FSCAL3
    0x24, 0x2A,  // FSCAL2
    0x25, 0x00,  // FSCAL1
    0x26, 0x1F,  // FSCAL0
    0x27, 0x41,  // RCCTRL1
    0x28, 0x00,  // RCCTRL0
    0x3E, 0xC3,  // PATABLE (CULFW32-Konvention; a-culfw setzt das via ccsetpa)
    0xFF, 0x00,  // Sentinel + Padding
};

} // namespace culfw32
