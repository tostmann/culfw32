// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 Dirk Tostmann
// Register tables taken from a-culfw (https://github.com/tostmann/a-culfw) clib/cc1100.c,
// GNU GPL version 2 or (at your option) any later version; the original
// file carries no individual copyright notice.
// Modified for CULFW32 (ESP-IDF port), Dirk Tostmann, 2026.

#include "slowrf_cfg.hpp"

namespace culfw32 {

// Byte-genau aus a-culfw cc1100.c:55..107 (CC1100_CFG[]).
// 868.30 MHz, ASK/OOK, async-serial; alle Decoder/Modulator in Software.
const uint8_t SLOWRF_CFG[] = {
    0x00, 0x0D,  // IOCFG2: GDO2_CFG=0x0D (Serial Data Output, async demod)
    0x01, 0x2E,  // IOCFG1: tri-state
    0x02, 0x2D,  // IOCFG0: 0x2D = HW0 (für TX-Direction-Workaround in async PKT_FORMAT=3)
    0x03, 0x07,  // FIFOTHR
    0x04, 0xD3,  // SYNC1 (irrelevant in async)
    0x05, 0x91,  // SYNC0
    0x06, 0x3D,  // PKTLEN
    0x07, 0x04,  // PKTCTRL1
    0x08, 0x32,  // PKTCTRL0: PKT_FORMAT=3 (async serial), no CRC, no whiten
    0x09, 0x00,  // ADDR
    0x0A, 0x00,  // CHANNR
    0x0B, 0x06,  // FSCTRL1: IF=152 kHz
    0x0C, 0x00,  // FSCTRL0
    0x0D, 0x21,  // FREQ2 (high)  -- 868.30 MHz
    0x0E, 0x65,  // FREQ1 (mid)   -- 868.30 MHz
    0x0F, 0x6A,  // FREQ0 (low)   -- 868.30 MHz
    0x10, 0x57,  // MDMCFG4: BW≈325 kHz
    0x11, 0xC4,  // MDMCFG3: DRATE_M -> ~5.6 kBaud
    0x12, 0x30,  // MDMCFG2: MOD_FORMAT=3 (ASK/OOK), no preamble/sync detect
    0x13, 0x23,  // MDMCFG1
    0x14, 0xB9,  // MDMCFG0: ChannelSpace 350 kHz
    0x15, 0x00,  // DEVIATN: irrelevant für OOK
    0x16, 0x07,  // MCSM2
    0x17, 0x00,  // MCSM1: TXOFF=IDLE, RXOFF=IDLE — a-culfw default
    0x18, 0x18,  // MCSM0: PO_TIMEOUT=64, FS_AUTOCAL=2 (idle->RX/TX)
    0x19, 0x14,  // FOCCFG
    0x1A, 0x6C,  // BSCFG
    0x1B, 0x07,  // AGCCTRL2: 42 dB statt 33 dB
    0x1C, 0x00,  // AGCCTRL1
    0x1D, 0x90,  // AGCCTRL0: 4 dB decision boundary
    0x1E, 0x87,  // WOREVT1
    0x1F, 0x6B,  // WOREVT0
    0x20, 0xF8,  // WORCTRL
    0x21, 0x56,  // FREND1
    0x22, 0x11,  // FREND0: kein PA-Ramping (saubere ASK-Flanken)
    0x23, 0xE9,  // FSCAL3
    0x24, 0x2A,  // FSCAL2
    0x25, 0x00,  // FSCAL1
    0x26, 0x1F,  // FSCAL0
    0x27, 0x41,  // RCCTRL1
    0x28, 0x00,  // RCCTRL0
    0x3E, 0xC2,  // PATABLE: 10 dBm (a-culfw `default_PA[8]`, idx 8 = 10 dBm)
    0xFF,        // Sentinel
};

} // namespace culfw32
