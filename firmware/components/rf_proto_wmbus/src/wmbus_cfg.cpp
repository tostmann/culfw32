// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
//
// CC1101-Registersaetze fuer den wMBus-Empfang, Modus S und T.
//
// Normgroessen (prEN 13757-4:2003) mit den Formeln des CC1101-Datenblatts
// (SWRS061I; f_xosc = 26 MHz): Frequenz (Abschnitt 21), Datenrate (12),
// Kanalfilter (13), Hub (16), Sync-Wort aus den Chipfolgen 4.4.1.2 und
// 5.4.1.2.
//
// Abstimmwerte (ZF, Kanalfilter, Sync-Modus, Hub und T-Datenrate im
// Empfang, FREND1, FOCCFG, BSCFG, AGCCTRL, FSCAL, TEST) sind keine
// Normgroessen; das Datenblatt verweist fuer sie auf SmartRF Studio. Es sind
// die Werte, die a-culfw fuer den CC1101 nutzt (Herkunft Texas Instruments);
// mit ihnen ist CULFW32 am 08.10.2026 on-air gegen a-culfw gemessen.
//
// FSTEST bleibt auf dem Reset-Wert (Datenblatt: "For test only. Do not write
// to this register.").

#include "wmbus_cfg.hpp"

namespace culfw32::wmbus {

namespace {

// CC1101-Konfigurationsregister (Datenblatt Abschnitt 29).
constexpr uint8_t IOCFG2   = 0x00;
constexpr uint8_t IOCFG0   = 0x02;
constexpr uint8_t SYNC1    = 0x04;
constexpr uint8_t SYNC0    = 0x05;
constexpr uint8_t PKTLEN   = 0x06;
constexpr uint8_t PKTCTRL1 = 0x07;
constexpr uint8_t PKTCTRL0 = 0x08;
constexpr uint8_t ADDR     = 0x09;
constexpr uint8_t CHANNR   = 0x0A;
constexpr uint8_t FSCTRL1  = 0x0B;
constexpr uint8_t FSCTRL0  = 0x0C;
constexpr uint8_t FREQ2    = 0x0D;
constexpr uint8_t FREQ1    = 0x0E;
constexpr uint8_t FREQ0    = 0x0F;
constexpr uint8_t MDMCFG4  = 0x10;
constexpr uint8_t MDMCFG3  = 0x11;
constexpr uint8_t MDMCFG2  = 0x12;
constexpr uint8_t MDMCFG1  = 0x13;
constexpr uint8_t MDMCFG0  = 0x14;
constexpr uint8_t DEVIATN  = 0x15;
constexpr uint8_t MCSM1    = 0x17;
constexpr uint8_t MCSM0    = 0x18;
constexpr uint8_t FOCCFG   = 0x19;
constexpr uint8_t BSCFG    = 0x1A;
constexpr uint8_t AGCCTRL2 = 0x1B;
constexpr uint8_t AGCCTRL1 = 0x1C;
constexpr uint8_t AGCCTRL0 = 0x1D;
constexpr uint8_t FREND1   = 0x21;
constexpr uint8_t FREND0   = 0x22;
constexpr uint8_t FSCAL3   = 0x23;
constexpr uint8_t FSCAL2   = 0x24;
constexpr uint8_t FSCAL1   = 0x25;
constexpr uint8_t FSCAL0   = 0x26;
constexpr uint8_t TEST2    = 0x2C;
constexpr uint8_t TEST1    = 0x2D;
constexpr uint8_t TEST0    = 0x2E;
constexpr uint8_t PATABLE  = 0x3E;
constexpr uint8_t END      = 0xFF;

}  // namespace

// Modus S: 868,30 MHz, 32,768 kcps Manchester, Hub +-50 kHz (Tabelle 4).
const uint8_t WMBUS_S_CFG[] = {
    SYNC1,    0x76,   // Sync-Wort = letzte 16 Chips von n*(01) 0001110110 10010110
    SYNC0,    0x96,   //   -> 0111011010010110 (4.4.1.2)
    MCSM1,    0x00,   // nach Empfang/Senden IDLE, wmbus_rf armiert neu; CCA aus
    IOCFG2,   0x06,   // GDO2: Sync-Wort empfangen bis Paketende
    IOCFG0,   0x00,   // GDO0: RX-FIFO ueber der Schwelle in FIFOTHR
    FSCTRL1,  0x08,   // ZF 203 kHz -- Abstimmwert
    FSCTRL0,  0x00,   // Frequenzkorrektur; der Treiber setzt den kalibrierten Wert
    FREQ2,    0x21,   // 0x21656A * 26 MHz / 2^16 = 868,2999 MHz
    FREQ1,    0x65,
    FREQ0,    0x6A,
    MDMCFG4,  0x6A,   // Kanalfilter 270 kHz (E=1, M=2) -- Abstimmwert; DRATE_E = 10
    MDMCFG3,  0x4A,   // DRATE_M = 74: 32,73 kBd
    MDMCFG2,  0x06,   // 2-FSK, Manchester in Software, Sync 16/16 + Carrier Sense
    MDMCFG1,  0x22,   // 4 Byte Praeambel; CHANSPC_E = 2
    MDMCFG0,  0xF8,   // Kanalabstand (Reset-Wert, CHANNR = 0)
    CHANNR,   0x00,
    DEVIATN,  0x47,   // erwarteter Hub 47,6 kHz -- Abstimmwert
    FREND1,   0xB6,   // Abstimmwert
    FREND0,   0x10,   // PA-Index 0 -> PATABLE[0]
    MCSM0,    0x18,   // Kalibrierung beim Wechsel IDLE -> RX/TX
    FOCCFG,   0x2E,   // Abstimmwert
    BSCFG,    0x6D,   // Abstimmwert
    AGCCTRL2, 0x04,   // Abstimmwert
    AGCCTRL1, 0x09,   // Abstimmwert
    AGCCTRL0, 0xB2,   // Abstimmwert
    FSCAL3,   0xEA,   // CHP_CURR_CAL_EN = 10b wie SmartRF Studio (19.6.2)
    FSCAL2,   0x2A,   // Startwerte der Kalibrierung
    FSCAL1,   0x00,
    FSCAL0,   0x1F,
    TEST2,    0x81,   // 0x81/0x35: Werte fuer Kanalfilter unter 325 kHz
    TEST1,    0x35,   //   (Datenblatt, FIFOTHR.ADC_RETENTION)
    TEST0,    0x09,   // 0x09 oder 0x0B je nach Band (19.6.2)
    PKTCTRL1, 0x04,   // RSSI/LQI anhaengen (liest wmbus_rf), keine Adresspruefung
    PKTCTRL0, 0x00,   // feste Laenge, ohne HW-CRC/Whitening; wmbus_rf schaltet um
    ADDR,     0x00,
    PKTLEN,   0xFF,
    PATABLE,  0xC2,   // PATABLE[0]: Sendepegel
    END,      0x00,
};

// Modus T: 868,95 MHz, 100 kcps "3 aus 6", Hub +-50 kHz (Tabelle 11); der
// Empfaenger muss 88..112 kcps annehmen (Tabelle 12).
const uint8_t WMBUS_T_CFG[] = {
    SYNC1,    0x54,   // Sync-Wort = letzte 16 Chips von n*(01) 0000111101
    SYNC0,    0x3D,   //   -> 0101010000111101 (5.4.1.2)
    MCSM1,    0x00,   // nach Empfang/Senden IDLE, wmbus_rf armiert neu; CCA aus
    IOCFG2,   0x06,   // GDO2: Sync-Wort empfangen bis Paketende
    IOCFG0,   0x00,   // GDO0: RX-FIFO ueber der Schwelle in FIFOTHR
    FSCTRL1,  0x08,   // ZF 203 kHz -- Abstimmwert
    FSCTRL0,  0x00,   // Frequenzkorrektur; der Treiber setzt den kalibrierten Wert
    FREQ2,    0x21,   // 0x216BD0 * 26 MHz / 2^16 = 868,9497 MHz (gerundet waere
    FREQ1,    0x6B,   //   0xD1 = 868,9501 MHz; 0xD0 ist so on-air gemessen)
    FREQ0,    0xD0,
    MDMCFG4,  0x5C,   // Kanalfilter 325 kHz (E=1, M=1) -- Abstimmwert; DRATE_E = 12
    MDMCFG3,  0x04,   // DRATE_M = 4: 103,15 kBd im Empfang -- Abstimmwert (Senden: 99,98 kBd)
    MDMCFG2,  0x06,   // 2-FSK, Dekodierung in Software, Sync 16/16 + Carrier Sense
    MDMCFG1,  0x22,   // 4 Byte Praeambel: mit dem Sync-Wort 38 Chips (01), n >= 19
    MDMCFG0,  0xF8,   // Kanalabstand (Reset-Wert, CHANNR = 0)
    CHANNR,   0x00,
    DEVIATN,  0x44,   // erwarteter Hub 38,1 kHz -- Abstimmwert (Senden: 50,8 kHz)
    FREND1,   0xB6,   // Abstimmwert
    FREND0,   0x10,   // PA-Index 0 -> PATABLE[0]
    MCSM0,    0x18,   // Kalibrierung beim Wechsel IDLE -> RX/TX
    FOCCFG,   0x2E,   // Abstimmwert
    BSCFG,    0xBF,   // Abstimmwert
    AGCCTRL2, 0x43,   // Abstimmwert
    AGCCTRL1, 0x09,   // Abstimmwert
    AGCCTRL0, 0xB5,   // Abstimmwert
    FSCAL3,   0xEA,   // CHP_CURR_CAL_EN = 10b wie SmartRF Studio (19.6.2)
    FSCAL2,   0x2A,   // Startwerte der Kalibrierung
    FSCAL1,   0x00,
    FSCAL0,   0x1F,
    TEST2,    0x81,   // Abstimmwert (Datenblatt nennt 0x81/0x35 fuer Kanalfilter
    TEST1,    0x35,   //   unter 325 kHz)
    TEST0,    0x09,   // 0x09 oder 0x0B je nach Band (19.6.2)
    PKTCTRL1, 0x04,   // RSSI/LQI anhaengen (liest wmbus_rf), keine Adresspruefung
    PKTCTRL0, 0x00,   // feste Laenge, ohne HW-CRC/Whitening; wmbus_rf schaltet um
    ADDR,     0x00,
    PKTLEN,   0xFF,
    PATABLE,  0xC2,   // PATABLE[0]: Sendepegel
    END,      0x00,
};

}  // namespace culfw32::wmbus
