// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
//
// wMBus-Codec (EN 13757-4), Modus S, T und C: Manchester, "3 aus 6", CRC und
// die Blockformate A und B. Eigenstaendig geschrieben nach prEN 13757-4:2003:
//   Manchester (S; T anderer -> Zaehler)   4.4.1, 5.4.2
//   "3 aus 6" (T, Zaehler sendet)          5.4.1, Tabelle 13
//   Bloecke und L-Feld                     4.5.1, 4.5.2.1, 5.5.1
//   CRC (FT3 nach IEC 60870-5-1)           4.5.3, 5.5.3
//   Postamble                              Tabelle 4 Anm. 3, Tabelle 11 Anm. 2
//   Rechenbeispiel                         Anhang C
// Modus C kennt der Entwurf von 2003 noch nicht; eine neuere Ausgabe der Norm
// liegt hier nicht vor. Grundlage dafuer:
//   Sync, Signalbyte, keine Kodierung      TI SWRA522E (2018) 2.1, 2.2
//   Kennbyte Rahmenformat A / B            TI TIDU582 (2014) Tabelle 2
//   Bloecke und CRCs im Rahmenformat B     wie wmbusmeters 3.1.1
//                                          (src/wmbus.cc, trimCRCsFrameFormatB),
//                                          gegen das dieser Empfang laeuft
// Reines C++, kein Funkzugriff.

#pragma once

#include <cstddef>
#include <cstdint>

namespace culfw32::wmbus {

// x^16 + x^13 + x^12 + x^11 + x^10 + x^8 + x^6 + x^5 + x^2 + 1
constexpr uint16_t CRC_POLYNOM = 0x3D65;

constexpr uint8_t  MODE_S = 1;
constexpr uint8_t  MODE_T = 2;
// Nur Empfang: Rahmen in Modus C (Format A und B) und in Modus T.
constexpr uint8_t  MODE_C = 3;

// Modus C: Auf das 16-Chip-Sync-Wort 0x543D -- es ist zugleich das Ende von
// Praeambel und Sync im Modus T -- folgen das Signalbyte 01010100 und das
// Kennbyte des Rahmenformats. Das Signalbyte kommt in "3 aus 6" nie vor; ein
// Empfaenger nimmt daran beide Modi auseinander (SWRA522E 2.1). Danach steht
// der Rahmen ohne Kodierung.
constexpr uint8_t  C_SIGNAL_BYTE = 0x54;
constexpr uint8_t  C_FRAME_A     = 0xCD;
constexpr uint8_t  C_FRAME_B     = 0x3D;

constexpr uint16_t PACKET_OK            = 0;
constexpr uint16_t PACKET_CODING_ERROR  = 1;
constexpr uint16_t PACKET_CRC_ERROR     = 2;

constexpr uint8_t  DECODING_3OUTOF6_OK    = 0;
constexpr uint8_t  DECODING_3OUTOF6_ERROR = 1;

constexpr uint8_t  MAN_DECODING_OK    = 0;
constexpr uint8_t  MAN_DECODING_ERROR = 1;

// Ein Byte in das CRC-Register (Startwert 0, MSb zuerst, ohne Invertierung).
// Im Rahmen steht ~crc, High-Byte zuerst.
uint16_t crc_calc(uint16_t crc_reg, uint8_t crc_data);

// "3 aus 6", Modus T (Zaehler sendet).
//   last_byte = 0: uncoded[0..1] -> encoded[0..2]
//   last_byte = 1: uncoded[0]    -> encoded[0..1]; die unteren vier Chips von
//                  encoded[1] sind die Postamble.
void    encode_3outof6(const uint8_t* uncoded, uint8_t* encoded, uint8_t last_byte);
// Rueckweg; DECODING_3OUTOF6_ERROR, wenn ein 6-Chip-Wort nicht in Tabelle 13
// steht.
uint8_t decode_3outof6(const uint8_t* encoded, uint8_t* decoded, uint8_t last_byte);

// Manchester, ein Byte <-> zwei Bytes ("01" = Eins, "10" = Null, MSb zuerst).
void    manch_encode(const uint8_t* uncoded, uint8_t* encoded);
uint8_t manch_decode(const uint8_t* encoded, uint8_t* decoded);

// Rahmenlaenge in Bytes zum L-Feld: L-Feld + L Nutzbytes + je Block zwei
// CRC-Bytes (erster Block 10 Bytes samt L-Feld, danach je 16).
uint16_t packet_size(uint8_t l_field);

// Rahmenformat B: das L-Feld zaehlt alle folgenden Bytes samt CRCs.
uint16_t packet_size_b(uint8_t l_field);

// Bytes auf der Luft fuer pkt_size Rahmenbytes; s_mode = 1 Manchester,
// 0 "3 aus 6"; transmit = 1 mit Postamble.
uint16_t byte_size(uint8_t s_mode, uint8_t transmit, uint16_t pkt_size);

// Rahmen -> Bytes auf der Luft, samt Postamble (Praeambel und Sync-Wort
// erzeugt der Funkchip; im Modus S beginnt die Ausgabe mit dem Sync-Rest 0x96).
void encode_tx_bytes_smode(uint8_t* out_bytes, const uint8_t* packet, uint16_t pkt_size);
void encode_tx_bytes_tmode(uint8_t* out_bytes, const uint8_t* packet, uint16_t pkt_size);

// Bytes auf der Luft -> Rahmen (alle pkt_size Bytes samt CRCs) und CRC-
// Pruefung jedes Blocks: PACKET_OK / PACKET_CODING_ERROR / PACKET_CRC_ERROR.
uint16_t decode_rx_bytes_smode(const uint8_t* in_bytes, uint8_t* out_packet, uint16_t pkt_size);
uint16_t decode_rx_bytes_tmode(const uint8_t* in_bytes, uint8_t* out_packet, uint16_t pkt_size);

// Modus C: in_bytes beginnt mit dem L-Feld (Signal- und Kennbyte davor sind
// schon abgetrennt) und ist nicht kodiert. Kopiert pkt_size Bytes samt CRCs
// nach out_packet und prueft die CRCs: Format A wie Modus T (Bloecke 10/16),
// Format B ein CRC ueber die Bytes 0..125 und ein zweiter ueber den Rest ab
// Byte 128, bei hoechstens 128 Bytes nur einer am Rahmenende.
uint16_t decode_rx_bytes_cmode(const uint8_t* in_bytes, uint8_t* out_packet,
                               uint16_t pkt_size, bool frame_b);

}  // namespace culfw32::wmbus
