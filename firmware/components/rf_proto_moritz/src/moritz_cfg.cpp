// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "moritz_cfg.hpp"

namespace culfw32 {

// Byte-genau uebernommen aus a-culfw rf_moritz.c:48..82.
// Kommentare auf Original-Quelle bezogen — bei Aenderungen mit Datenblatt SWRS061I
// und a-culfw mainline gegenpruefen.
const uint8_t MORITZ_CFG[60] = {
    0x00, 0x07,  // IOCFG2: GDO2_CFG=7 (Asserts on packet RX with CRC OK)
    0x02, 0x46,  // IOCFG0
    0x04, 0xC6,  // SYNC1
    0x05, 0x26,  // SYNC0
    0x0B, 0x06,  // FSCTRL1
    0x10, 0xC8,  // MDMCFG4 DRATE_E=8
    0x11, 0x93,  // MDMCFG3 DRATE_M=147 -> ~10 kbit/s
    0x12, 0x03,  // MDMCFG2 SYNC_MODE=3 (30/32), MOD_FORMAT=0 (2-FSK)
    0x13, 0x22,  // MDMCFG1 CHANSPC_E=2, NUM_PREAMBLE=2 (4 bytes)
    0x15, 0x34,  // DEVIATN
    0x17, 0x3F,  // MCSM1: TXOFF=RX, RXOFF=RX, CCA_MODE=3
    0x18, 0x28,  // MCSM0: PO_TIMEOUT=64, FS_AUTOCAL=2 (idle->RX/TX)
    0x19, 0x16,  // FOCCFG
    0x1B, 0x43,  // AGCTRL2
    0x21, 0x56,  // FREND1
    0x25, 0x00,  // FSCAL1
    0x26, 0x11,  // FSCAL0
    0x0D, 0x21,  // FREQ2
    0x0E, 0x65,  // FREQ1   -> ~868.299 MHz
    0x0F, 0x6A,  // FREQ0
    0x07, 0x0C,  // PKTCTRL1
    0x16, 0x07,  // MCSM2 RX_TIME=7 (timeout = end of packet)
    0x20, 0xF8,  // WORCTRL
    0x1E, 0x87,  // WOREVT1 (EVENT0 high)
    0x1F, 0x6B,  // WOREVT0 (EVENT0 low)
    0x29, 0x59,  // FSTEST
    0x2C, 0x81,  // TEST2
    0x2D, 0x35,  // TEST1
    0x3E, 0xC3,  // PATABLE-ish — laut a-culfw-Kommentar; mit Dirk verifizieren bei Hardware-Spin
    0xFF,        // Sentinel
};

} // namespace culfw32
