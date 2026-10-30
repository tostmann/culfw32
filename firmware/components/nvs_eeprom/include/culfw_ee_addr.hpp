// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include <cstdint>

namespace culfw32::ee {

// Byte-genaue Spiegelung der a-culfw EEPROM-Map (fncollection.h).
// Eintraege so anlegen wie sie auftauchen — der NVS-Layer haelt das als
// flachen Adressraum, einzelne Bytes sind ueber `R<aaaa>`/`W<aaaa><vv>`
// direkt erreichbar.

constexpr uint16_t MAGIC_OFFSET    = 0;        // 2 bytes Magic-Cookie
constexpr uint16_t CC1100_CFG      = 2;        // 41 bytes Reg-Snapshot
constexpr uint16_t CC1100_CFG_SIZE = 0x29;     // 41
constexpr uint16_t CC1100_PA       = 43;       // 8 bytes PATABLE
constexpr uint16_t CC1100_PA_SIZE  = 8;
constexpr uint16_t REQBL           = 51;       // 1 byte Bootloader-Request
constexpr uint16_t LED             = 52;       // 1 byte LED-Mode (a-culfw EE_LED)
constexpr uint16_t FHTID           = 53;       // 2 bytes FHT Hauscode
constexpr uint16_t FASTRF_CFG      = 55;       // 41 bytes (a-culfw HAS_FASTRF/HAS_RF_ROUTER)
constexpr uint16_t RF_ROUTER_ID    = 96;       // 1 byte
constexpr uint16_t RF_ROUTER_ROUTER= 97;       // 1 byte

// a-culfw HAS_ETHERNET-Region (fncollection.h:45..52, CUNO-Layout). Byte-genau,
// damit `Wi*`/`Ri*` wie auf dem CUNO lesen und schreiben. Genutzt vom W5500
// an J1 (eth_drv); den TCP-Port liest auch der WiFi-Pfad. a-culfw legt OHNE
// Ethernet ab 98 die zweite CC1101-Konfiguration ab (EE_CC1100_CFG1) — die
// gibt es in CULFW32 nicht, der Block gehoert hier immer dem Netz.
constexpr uint16_t MAC_ADDR         = 98;      // 6 bytes; FF..FF = automatisch (eth_drv)
constexpr uint16_t USE_DHCP         = 104;     // 1 byte; != 0 (auch 0xFF) = DHCP
constexpr uint16_t IP4_ADDR         = 105;     // 4 bytes a.b.c.d
constexpr uint16_t IP4_NETMASK      = 109;     // 4 bytes
constexpr uint16_t IP4_GATEWAY      = 113;     // 4 bytes
constexpr uint16_t IP4_NTPSERVER    = 117;     // 4 bytes (gespeichert, NTP nicht portiert)
constexpr uint16_t IP4_TCPLINK_PORT = 121;     // 2 bytes, Little-Endian wie a-culfw fromdec
constexpr uint16_t IP4_TCPLINK_PORT_DEFAULT = 2323;
constexpr uint16_t IP4_NTPOFFSET    = 123;     // 1 byte (gespeichert, NTP nicht portiert)

// CULFW32-Erweiterung: TCP-Allow-Subnet im 0x100-Bereich (CULFW32-Extension).
// 4 Bytes IPv4 + 1 Byte CIDR-Bits. Default 0.0.0.0/0 = jeder darf connecten.
// `accept()` im TcpServer prueft client-IP & mask gegen erlaubte IP & mask.
constexpr uint16_t TCP_ALLOW_IP   = 0x110;     // 4 bytes (IPv4 in network-byte-order)
constexpr uint16_t TCP_ALLOW_BITS = 0x114;     // 1 byte (CIDR 0..32)

// JSON-Frontend TCP-Port (CULFW32-Extension). 2 Bytes Little-Endian wie
// IP4_TCPLINK_PORT. Default kommt aus Kconfig (typisch 2324).
constexpr uint16_t TCP_JSON_PORT  = 0x115;     // 2 bytes
constexpr uint16_t TCP_JSON_PORT_DEFAULT = 2324;

// SIGNALduino-Frontend TCP-Port (CULFW32-Extension). 2 Bytes Little-Endian.
// Default 2326 (frei waehlbar, neben 2324/2325 fuer json/hmuartlgw).
constexpr uint16_t TCP_SD_PORT    = 0x117;     // 2 bytes
constexpr uint16_t TCP_SD_PORT_DEFAULT = 2326;

// a-culfw version.h: VERSION_1=1, VERSION_2=26 — Magic-Cookie fuer EE-Init-Detect.
// Bei Boot-Mismatch (Erstbetrieb/NVS-Wipe) wird factory_reset_silent ausgeloest.
constexpr uint8_t  MAGIC_V1        = 1;
constexpr uint8_t  MAGIC_V2        = 26;

// CULFW32-Erweiterungen ab 0x100 — sicher hinter allen a-culfw EE-Bytes (incl.
// Ethernet/LCD/Multi-CC). Ab 0x100 hat **kein** legacy a-culfw je etwas
// gespeichert, also kein Adress-Konflikt fuer Bestandskunden-Migration.
constexpr uint16_t MORITZ_AA_ADDR  = 0x100;    // 3 bytes Auto-Ack-Adresse (Za)
constexpr uint16_t MORITZ_WT_ADDR  = 0x103;    // 3 bytes Wall-Thermostat-Fake (Zw)

// HMUARTLGW-Frontend (CULFW32-Extension, 2026-04-28). HMID = 3-byte BidCos-
// HomeMatic-Adresse (entspricht den 3 src-Bytes im AskSin-Frame). Wird via
// FHEM-Befehl APP_SET_HMID gesetzt und persistiert. Default 0xFFFFFF =
// "noch nicht provisioned" — Frontend liefert dann 0xFFFFFF aus, FHEM
// setzt seine attr-hmId via APP_SET_HMID.
constexpr uint16_t HMU_HMID          = 0x120;  // 3 bytes
constexpr uint16_t HMU_PORT          = 0x123;  // 2 bytes LE (Default 2325)
constexpr uint16_t HMU_PORT_DEFAULT  = 2325;
// AES-Key-Slots — 17 Byte je (1 Byte index + 16 Byte AES-Key). In v1 nur
// gespeichert, in v1.x+1 fuer mbedtls/aes verwendet.
constexpr uint16_t HMU_KEY_CURRENT   = 0x125;  // 17 bytes
constexpr uint16_t HMU_KEY_PREVIOUS  = 0x136;  // 17 bytes
constexpr uint16_t HMU_KEY_TEMP      = 0x147;  // 17 bytes

// HMUARTLGW Module-Identitaet. 0=HM-MOD-RPI-PCB (Legacy CoPro, BidCoS-only;
// FHEM/Homegear-Default), 1=RPI-RF-MOD (DualCoPro fuer debmatic/RaspberryMatic).
// debmatic-multimacd weigert sich mit Legacy-Modulen zu starten; im RPI-RF-MOD-
// Mode emuliert CULFW32 die DualCoPro-Init-Sequenz auf HM_DST_COMMON/TRX/HMIP/
// LLMAC. Default 0xFF → 0 (HM-MOD-RPI-PCB). Siehe hmuartlgw_module_modes.md.
constexpr uint16_t HMU_MODULE_MODE   = 0x158;  // 1 byte enum
constexpr uint8_t  HMU_MODULE_LEGACY = 0;      // HM-MOD-RPI-PCB
constexpr uint8_t  HMU_MODULE_DUAL   = 1;      // RPI-RF-MOD (DualCoPro)

// CC1101-Frequenzoffset (FSCTRL0), Feldkalibrierung. 1 Byte Wert (int8, als
// uint8 abgelegt) + 1 Byte Gueltigkeitsmarke. Ohne Marke gilt der eFuse-Wert
// aus der Produktion bzw. 0. Siehe components/freq_cal.
constexpr uint16_t RF_FREQOFF        = 0x159;  // 1 byte, int8 als uint8
constexpr uint16_t RF_FREQOFF_VALID  = 0x15A;  // 1 byte, 0x5A = gueltig
constexpr uint8_t  RF_FREQOFF_MAGIC  = 0x5A;

// Online-Update (components/ota_update): 1 = gefundenes Update selbst
// installieren, sonst (0, 0xFF unbeschrieben) nur pruefen und melden.
constexpr uint16_t OTA_AUTO          = 0x15B;  // 1 byte

// CULFW32-Erweiterungen reichen bis 0x15B; der Rest bis 0x3FF ist frei.

// Default-Wert fuer uninitialisierte EE-Bytes (wie nach Power-On AVR-EEPROM).
constexpr uint8_t  DEFAULT         = 0xFF;

} // namespace culfw32::ee
