// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "native_cfg.hpp"

namespace culfw32 {

// Byte-genau a-culfw rf_native.c:42 (NATIVE_CFG[]).
// CC1100_FIFOTHR = 2 (Mode 1 default = 12 bytes RX threshold).
// PKTCTRL0 = 0x02 = infinite packet length, kein CRC, kein length byte.
// IOCFG2 = 0x01: RX FIFO at FIFOTHR bytes (= GDO2-IRQ-Pin trigger).
const uint8_t NATIVE_CFG[46] = {
    0x0B, 0x06,  // FSCTRL1
    0x0D, 0x21,  // FREQ2     868.3 MHz
    0x0E, 0x65,  // FREQ1
    0x0F, 0x6A,  // FREQ0
    0x17, 0x00,  // MCSM1     always go into IDLE
    0x18, 0x18,  // MCSM0
    0x19, 0x16,  // FOCCFG
    0x1B, 0x43,  // AGCCTRL2
    0x1C, 0x68,  // AGCCTRL1
    0x25, 0x00,  // FSCAL1
    0x26, 0x11,  // FSCAL0
    0x00, 0x01,  // IOCFG2    RX FIFO at FIFOTHR bytes
    0x02, 0x46,  // IOCFG0    not used
    0x05, 0xD4,  // SYNC0
    0x04, 0x2D,  // SYNC1
    0x03, 0x02,  // FIFOTHR   12 bytes RX (Mode 1 default)
    0x07, 0x00,  // PKTCTRL1
    0x08, 0x02,  // PKTCTRL0  infinite packet length
    0x10, 0x89,  // MDMCFG4
    0x11, 0x5C,  // MDMCFG3
    0x12, 0x06,  // MDMCFG2   2-FSK
    0x15, 0x56,  // DEVIATN
    0xFF,        // Sentinel
    0x00,        // Padding
};

// Mode 1: IT+ 17.241 kbps — only sets FIFOTHR (selbe wie Default).
// Wir lassen die Tabelle drin damit MODE_CFG[mode-1] symmetrisch loopt.
const uint8_t NATIVE_MODE1_CFG[4] = {
    0x03, 0x02,  // FIFOTHR=2 (12 bytes)
    0xFF, 0x00,
};

// Mode 2: IT+ 9.579 kbps — slow data rate.
const uint8_t NATIVE_MODE2_CFG[8] = {
    0x03, 0x02,  // FIFOTHR=2 (12 bytes)
    0x10, 0x88,  // MDMCFG4
    0x11, 0x82,  // MDMCFG3
    0xFF, 0x00,
};

// Mode 3: PCA 301 — 868.95 MHz, 6.631 kbps, FIFOTHR=7 (32 bytes).
const uint8_t NATIVE_MODE3_CFG[16] = {
    0x03, 0x07,  // FIFOTHR=7 (32 bytes)
    0x10, 0x88,  // MDMCFG4
    0x11, 0x0B,  // MDMCFG3
    0x0D, 0x21,  // FREQ2
    0x0E, 0x6B,  // FREQ1     868.95 MHz
    0x0F, 0xD0,  // FREQ0
    0x15, 0x53,  // DEVIATN
    0xFF, 0x00,
};

} // namespace culfw32
