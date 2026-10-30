// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "asksin_cfg.hpp"

namespace culfw32 {

// Byte-genau uebernommen aus a-culfw rf_asksin.c:28..54.
// Kommentare auf Original-Quelle bezogen — bei Aenderungen mit Datenblatt
// SWRS061I und a-culfw mainline gegenpruefen.
const uint8_t ASKSIN_CFG[52] = {
    0x00, 0x07,  // IOCFG2: GDO2_CFG=7 (Asserts on packet RX with CRC OK)
    0x02, 0x2E,  // IOCFG0: tristate (nicht benutzt)
    0x03, 0x0D,  // FIFOTHR
    0x04, 0xE9,  // SYNC1
    0x05, 0xCA,  // SYNC0
    0x07, 0x0C,  // PKTCTRL1: append RSSI/LQI status, no addr check, CRC_AUTOFLUSH on
    0x0B, 0x06,  // FSCTRL1
    0x0D, 0x21,  // FREQ2
    0x0E, 0x65,  // FREQ1   -> ~868.3 MHz
    0x0F, 0x6A,  // FREQ0
    0x10, 0xC8,  // MDMCFG4 DRATE_E=8
    0x11, 0x93,  // MDMCFG3 DRATE_M=147 -> ~10 kbit/s
    0x12, 0x03,  // MDMCFG2 SYNC_MODE=3 (30/32), MOD_FORMAT=0 (2-FSK)
    0x15, 0x34,  // DEVIATN
    0x17, 0x33,  // MCSM1: TXOFF=RX (auto-restart), CCA_MODE=11 (RSSI<thresh && !RX)
                 // a-culfw-Kommentar: "go into RX after TX, CCA; EQ3 uses 0x03"
    0x18, 0x18,  // MCSM0: PO_TIMEOUT=64, FS_AUTOCAL=2 (idle->RX/TX)
    0x19, 0x16,  // FOCCFG
    0x1B, 0x43,  // AGCTRL2
    0x21, 0x56,  // FREND1
    0x25, 0x00,  // FSCAL1
    0x26, 0x11,  // FSCAL0
    0x29, 0x59,  // FSTEST
    0x2C, 0x81,  // TEST2
    0x2D, 0x35,  // TEST1
    0x3E, 0xC3,  // PATABLE-ish — laut a-culfw-Kommentar; mit Dirk verifizieren bei Hardware-Spin
    0xFF,        // Sentinel
    0x00,        // Padding (Array-Size symmetrisch)
};

// FUP-Mode (Firmware-Update Bootloader-Pfad). Wird ueber ASKSIN_CFG drueber-
// geschrieben wenn `AR` aktiviert. Nicht in v1 verwendet.
// Quelle: a-culfw rf_asksin.c:57.
const uint8_t ASKSIN_UPDATE_CFG[24] = {
    0x0B, 0x08,
    0x10, 0x5B,
    0x11, 0xF8,
    0x15, 0x47,
    0x19, 0x1D,
    0x1A, 0x1C,
    0x1B, 0xC7,
    0x1C, 0x00,
    0x1D, 0xB2,
    0x21, 0xB6,
    0x23, 0xEA,
    0xFF, 0x00,  // Sentinel + Padding
};

} // namespace culfw32
