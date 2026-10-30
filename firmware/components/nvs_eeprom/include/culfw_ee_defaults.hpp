// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 Dirk Tostmann
// Register tables taken from a-culfw (https://github.com/tostmann/a-culfw) clib/cc1100.c,
// GNU GPL version 2 or (at your option) any later version; the original
// file carries no individual copyright notice.
// Modified for CULFW32 (ESP-IDF port), Dirk Tostmann, 2026.

#pragma once

#include <array>
#include <cstdint>

namespace culfw32::ee {

// Byte-genaue Spiegelung von a-culfw `cc1100.c::CC1100_CFG[]` (868 MHz Default,
// SlowRF/OOK 5.6 kbps BW 325 kHz). Wird beim factory_reset und beim Erst-Boot
// (Magic-Mismatch) in EE_CC1100_CFG geschrieben — analog zu a-culfw
// `cc_factory_reset()`. RF-Klassen (MoritzRF/AskSinRF/SlowRfRF) lesen diese
// Tabelle NICHT zur Laufzeit; sie nutzen ihre eigenen Mode-Tabellen aus
// rf_moritz/asksin/slowrf. Trotzdem persistieren wir die Tabelle byte-genau,
// damit `R<aa>` / `C99` / Bestandskunden-Tools das erwartete Snapshot sehen.
inline constexpr std::array<uint8_t, CC1100_CFG_SIZE> CC1100_CFG_DEFAULTS = {
    // CULFW   IDX NAME      COMMENT (a-culfw cc1100.c:55)
    0x0D, // 00 IOCFG2     GDO2 as serial output
    0x2E, // 01 IOCFG1     Tri-State
    0x2D, // 02 IOCFG0     GDO0 for input
    0x07, // 03 FIFOTHR
    0xD3, // 04 SYNC1
    0x91, // 05 SYNC0
    0x3D, // 06 PKTLEN
    0x04, // 07 PKTCTRL1
    0x32, // 08 PKTCTRL0
    0x00, // 09 ADDR
    0x00, // 0A CHANNR
    0x06, // 0B FSCTRL1    152 kHz IF Frequency
    0x00, // 0C FSCTRL0
    0x21, // 0D FREQ2      868.3 MHz
    0x65, // 0E FREQ1
    0x6A, // 0F FREQ0
    0x57, // 10 MDMCFG4    BW 325 kHz
    0xC4, // 11 MDMCFG3    DataRate ~5604 baud
    0x30, // 12 MDMCFG2    Modulation: ASK/OOK
    0x23, // 13 MDMCFG1
    0xB9, // 14 MDMCFG0    ChannelSpace 350 kHz
    0x00, // 15 DEVIATN
    0x07, // 16 MCSM2
    0x00, // 17 MCSM1
    0x18, // 18 MCSM0      Calibration: RX/TX->IDLE
    0x14, // 19 FOCCFG
    0x6C, // 1A BSCFG
    0x07, // 1B AGCCTRL2   42 dB instead of 33 dB
    0x00, // 1C AGCCTRL1
    0x90, // 1D AGCCTRL0   4 dB decision boundary
    0x87, // 1E WOREVT1
    0x6B, // 1F WOREVT0
    0xF8, // 20 WORCTRL
    0x56, // 21 FREND1
    0x11, // 22 FREND0     0x11 for no PA ramping
    0xE9, // 23 FSCAL3
    0x2A, // 24 FSCAL2
    0x00, // 25 FSCAL1
    0x1F, // 26 FSCAL0
    0x41, // 27 RCCTRL1
    0x00, // 28 RCCTRL0
};

// Spiegelung von a-culfw `cc1100.c::CC1100_PA[]` mit FULL_CC1100_PA-Define
// (CUL/board.h:36 — Lokales+Remote bauen mit FULL).
// 10 PA-Profile a 8 Bytes = 80 Bytes:
//   idx 0..4: -10/-5/0/5/10 dBm mit PA-Ramping (FREND0=0x17)
//   idx 5..9: -10/-5/0/5/10 dBm ohne PA-Ramping (FREND0=0x11)
//   idx > 9   in cc_set_pa() auf 8 geclamped (=5 dBm no ramp, factory-default)
//
// Die Ramping-Profile 0..4 bleiben allein aus a-culfw-Kompatibilitaet in der
// Tabelle (wer `x02` setzt, soll dasselbe bekommen wie am CUL). Fuer OOK sind
// sie nicht sinnvoll: SWRS061I §24 sieht die Rampe fuer FSK-Paketraender und
// "ASK modulation shaping" vor; bei OOK laeuft sie an JEDER Bitflanke. On-air
// gemessen 02.09.2026 (HackRF, FS20, CUL32): ohne Ramping 4 us Anstieg
// (Filtergrenze), mit Ramping ~28 us, ein FS20-Handsender 7 us. Der
// Werksdefault (idx 8) bleibt ohne Ramping.
//
// Bewusste Abweichung von a-culfw (02.09.2026): der -5-dBm-Endpegel ist 0x2E
// statt 0x67. a-culfw fuehrt 0x67 ("yes it is 67, checked 3 times!") — DN013
// Tabelle 3 listet es tatsaechlich mit -5,0 dBm, aber es liegt im Bereich
// 0x61..0x6F, den SWRS061I §24 und DN013 §3 ausschliessen (ohne Begruendung;
// die Tabelle zeigt fuer alle 0x6x eine ~5..6 dB schlechtere 2. Harmonische).
// 0x2E liefert -4,9 dBm bei -51,8 dBm 2. Harm. statt -46,0 (DN013, 868 MHz).
// Betrifft idx 1 und idx 6 (Endpegel) sowie den 5. Schritt in idx 2
// (0x67 -> 0x2E; Rampe -30,2 / -20,0 / -12,5 / -4,9 / -1,5 / -0,5 / -0,3 dBm
// nach DN013). 0x60 im 7. Schritt bleibt a-culfw: der Ausschluss lautet in
// SWRS061I §24 wie in DN013 §3 "0x61 to 0x6F", 0x60 ist nicht dabei; on-air
// 03.09.2026 sind 0x2E/0x60 und 0x2E/0x51 nicht unterscheidbar (27,5 vs
// 27,8 us Anstieg, 6,6 Schwellendurchgaenge je Flanke). 0x67 dagegen bringt
// beim Umschalten einen Phasensprung von ~100 Grad (Kerbe in der Flanke,
// Abfall 72 us); sein Pegel stimmt mit DN013 (-4,5 dB rel. 0x50).
//
// OFFEN (03.09.2026, Pegelleiter CUL32 #2, HackRF relativ zu 0x50): die
// Eintraege 0x25/0x27/0x2E/0x39 liegen +5,4..+6,5 dB UEBER den DN013-
// Abstaenden, 0x40/0x60/0x67/0x50 stimmen, 0x81 +1,2 dB. Folge: diese Rampe
// ist auf der Hardware NICHT monoton (0x2E = +1,2 dB ueber dem 0x50-Endwert
// -> "Ohren" an jeder Flanke), und die x-Profile 5/6/7/8
// liegen bei -3,1/+1,2/0/+6,5 dB statt -10/-5/0/+5 nominal. Monotoner
// Kandidat on-air verifiziert: 0x27 statt 0x2E (Treppe 0,6/0,7/0,8/1,0,
// 69 us Anstieg). Entscheidung nach Absolutmessung (Leistungsmesser).
inline constexpr std::array<uint8_t, 10 * CC1100_PA_SIZE> CC1100_PA_DEFAULTS = {
    // idx 0: -10 dBm mit PA-Ramping
    0x00, 0x03, 0x0F, 0x1E, 0x26, 0x27, 0x00, 0x00,
    // idx 1:  -5 dBm  (Endpegel 0x2E statt a-culfw 0x67, s.o.)
    0x00, 0x03, 0x0F, 0x1E, 0x25, 0x36, 0x38, 0x2E,
    // idx 2:   0 dBm  (5. Schritt 0x2E statt a-culfw 0x67, s.o.)
    0x00, 0x03, 0x0F, 0x25, 0x2E, 0x40, 0x60, 0x50,
    // idx 3:   5 dBm
    0x00, 0x03, 0x0F, 0x27, 0x51, 0x88, 0x83, 0x81,
    // idx 4:  10 dBm
    0x00, 0x03, 0x0F, 0x27, 0x50, 0xC8, 0xC3, 0xC2,
    // idx 5: -10 dBm ohne PA-Ramping
    0x00, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // idx 6:  -5 dBm  (0x2E statt a-culfw 0x67, s.o.)
    0x00, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // idx 7:   0 dBm
    0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // idx 8:   5 dBm  (factory-default)
    0x00, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // idx 9:  10 dBm
    0x00, 0xC2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

} // namespace culfw32::ee
