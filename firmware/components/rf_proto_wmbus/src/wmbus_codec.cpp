// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
//
// Quellen und Abschnitte der Norm: siehe wmbus_codec.hpp.

#include "wmbus_codec.hpp"

namespace culfw32::wmbus {

namespace {

// Tabelle 13: Nibble -> 6-Chip-Wort (je drei Nullen und Einsen, mindestens
// zwei Wechsel).
constexpr uint8_t kCode3of6[16] = {
    0x16, 0x0D, 0x0E, 0x0B, 0x1C, 0x19, 0x1A, 0x13,
    0x2C, 0x25, 0x26, 0x23, 0x34, 0x31, 0x32, 0x29,
};

// Rueckweg 6-Chip-Wort -> Nibble; 0xFF = kein gueltiges Wort.
struct Decode3of6 {
    uint8_t nibble[64];
    constexpr Decode3of6() : nibble{} {
        for (auto& n : nibble) n = 0xFF;
        for (uint8_t n = 0; n < 16; ++n) nibble[kCode3of6[n]] = n;
    }
};
constexpr Decode3of6 kDecode3of6{};

// Postamble im Modus T: Wechselchips 0101..., 2 bis 8 Chips (Tabelle 11).
// Anm. 2 empfiehlt, gegenlaeufig zum letzten Datenchip zu beginnen; gesendet
// wird wie bei a-culfw immer 0101... -- kein Empfaenger wertet die Postamble
// aus, und die Sendung bleibt bytegleich zur on-air gemessenen Fassung.
constexpr uint8_t kPostambleT = 0x55;

// Erster Block: L-Feld + C + M + A = 10 Bytes, danach je 16 Bytes; jeder
// Block traegt ~CRC, High-Byte zuerst (4.5.1, 4.5.3).
bool blocks_ok(const uint8_t* p, uint16_t n) {
    uint16_t pos = 0;
    uint16_t len = 10;
    while (pos < n) {
        if (n - pos < 3) return false;                 // kein Platz fuer Daten + CRC
        if (pos + len + 2 > n) len = static_cast<uint16_t>(n - pos - 2);
        uint16_t crc = 0;
        for (uint16_t i = 0; i < len; ++i) crc = crc_calc(crc, p[pos + i]);
        crc = static_cast<uint16_t>(~crc);
        if (p[pos + len] != static_cast<uint8_t>(crc >> 8) ||
            p[pos + len + 1] != static_cast<uint8_t>(crc)) {
            return false;
        }
        pos = static_cast<uint16_t>(pos + len + 2);
        len = 16;
    }
    return true;
}

}  // namespace

uint16_t crc_calc(uint16_t crc_reg, uint8_t crc_data) {
    crc_reg ^= static_cast<uint16_t>(crc_data) << 8;
    for (int i = 0; i < 8; ++i) {
        crc_reg = (crc_reg & 0x8000)
            ? static_cast<uint16_t>((crc_reg << 1) ^ CRC_POLYNOM)
            : static_cast<uint16_t>(crc_reg << 1);
    }
    return crc_reg;
}

void encode_3outof6(const uint8_t* uncoded, uint8_t* encoded, uint8_t last_byte) {
    const uint32_t hi = kCode3of6[uncoded[0] >> 4];
    const uint32_t lo = kCode3of6[uncoded[0] & 0x0F];
    if (last_byte) {
        // 12 Datenchips + 4 Chips Postamble
        const uint32_t w = (hi << 10) | (lo << 4) | (kPostambleT >> 4);
        encoded[0] = static_cast<uint8_t>(w >> 8);
        encoded[1] = static_cast<uint8_t>(w);
        return;
    }
    const uint32_t w = (hi << 18) | (lo << 12)
                     | (static_cast<uint32_t>(kCode3of6[uncoded[1] >> 4]) << 6)
                     | kCode3of6[uncoded[1] & 0x0F];
    encoded[0] = static_cast<uint8_t>(w >> 16);
    encoded[1] = static_cast<uint8_t>(w >> 8);
    encoded[2] = static_cast<uint8_t>(w);
}

uint8_t decode_3outof6(const uint8_t* encoded, uint8_t* decoded, uint8_t last_byte) {
    const uint32_t w = last_byte
        ? (static_cast<uint32_t>(encoded[0]) << 16) | (static_cast<uint32_t>(encoded[1]) << 8)
        : (static_cast<uint32_t>(encoded[0]) << 16) | (static_cast<uint32_t>(encoded[1]) << 8)
          | encoded[2];
    const unsigned words = last_byte ? 2 : 4;
    uint8_t n[4] = {};
    for (unsigned i = 0; i < words; ++i) {
        n[i] = kDecode3of6.nibble[(w >> (18 - 6 * i)) & 0x3F];
        if (n[i] == 0xFF) return DECODING_3OUTOF6_ERROR;
    }
    decoded[0] = static_cast<uint8_t>((n[0] << 4) | n[1]);
    if (!last_byte) decoded[1] = static_cast<uint8_t>((n[2] << 4) | n[3]);
    return DECODING_3OUTOF6_OK;
}

void manch_encode(const uint8_t* uncoded, uint8_t* encoded) {
    uint16_t chips = 0;
    for (int bit = 7; bit >= 0; --bit) {
        chips = static_cast<uint16_t>(chips << 2);
        chips |= ((uncoded[0] >> bit) & 1) ? 0x1 : 0x2;   // "01" = Eins, "10" = Null
    }
    encoded[0] = static_cast<uint8_t>(chips >> 8);
    encoded[1] = static_cast<uint8_t>(chips);
}

uint8_t manch_decode(const uint8_t* encoded, uint8_t* decoded) {
    const uint16_t chips = static_cast<uint16_t>((encoded[0] << 8) | encoded[1]);
    uint8_t value = 0;
    for (int pair = 7; pair >= 0; --pair) {
        const unsigned c = (chips >> (2 * pair)) & 0x3;
        if (c != 0x1 && c != 0x2) return MAN_DECODING_ERROR;
        value = static_cast<uint8_t>((value << 1) | (c == 0x1));
    }
    decoded[0] = value;
    return MAN_DECODING_OK;
}

uint16_t packet_size(uint8_t l_field) {
    const uint16_t rest = (l_field > 9) ? static_cast<uint16_t>(l_field - 9) : 0;
    const uint16_t blocks = static_cast<uint16_t>(1 + (rest + 15) / 16);
    return static_cast<uint16_t>(l_field + 1 + 2 * blocks);
}

uint16_t byte_size(uint8_t s_mode, uint8_t transmit, uint16_t pkt_size) {
    // Modus S, Senden: Sync-Rest 0x96 + Daten + Postamble (encode_tx_bytes_smode).
    if (s_mode) return static_cast<uint16_t>(2 * pkt_size + (transmit ? 2 : 0));
    // 12 Chips je Byte; bei ungerader Laenge fuellt die Postamble das letzte
    // Halbbyte, bei gerader braucht sie zum Senden ein eigenes Byte.
    const uint16_t n = static_cast<uint16_t>((3 * pkt_size + 1) / 2);
    return static_cast<uint16_t>(n + ((transmit && !(pkt_size & 1)) ? 1 : 0));
}

void encode_tx_bytes_smode(uint8_t* out_bytes, const uint8_t* packet, uint16_t pkt_size) {
    // Sync-Folge im Modus S: n*(01) 0001110110 10010110 (4.4.1.2). Das Sync-
    // Wort des Funkchips (beim Senden 0x54 0x76) traegt Praeambelreste und die
    // ersten zehn Chips; die letzten acht Chips gehen als Datenbyte 0x96 hinaus.
    out_bytes[0] = 0x96;
    for (uint16_t i = 0; i < pkt_size; ++i) manch_encode(&packet[i], &out_bytes[1 + 2 * i]);
    out_bytes[1 + 2 * pkt_size] = 0x55;                // Postamble: vier "Einsen" (Tabelle 4, Anm. 3)
}

void encode_tx_bytes_tmode(uint8_t* out_bytes, const uint8_t* packet, uint16_t pkt_size) {
    uint16_t i = 0;
    uint16_t k = 0;
    for (; i + 1 < pkt_size; i = static_cast<uint16_t>(i + 2), k = static_cast<uint16_t>(k + 3)) {
        encode_3outof6(&packet[i], &out_bytes[k], 0);
    }
    if (i < pkt_size) {
        encode_3outof6(&packet[i], &out_bytes[k], 1);
    } else {
        out_bytes[k] = kPostambleT;
    }
}

uint16_t decode_rx_bytes_smode(const uint8_t* in_bytes, uint8_t* out_packet, uint16_t pkt_size) {
    for (uint16_t i = 0; i < pkt_size; ++i) {
        if (manch_decode(&in_bytes[2 * i], &out_packet[i]) != MAN_DECODING_OK) {
            return PACKET_CODING_ERROR;
        }
    }
    return blocks_ok(out_packet, pkt_size) ? PACKET_OK : PACKET_CRC_ERROR;
}

uint16_t decode_rx_bytes_tmode(const uint8_t* in_bytes, uint8_t* out_packet, uint16_t pkt_size) {
    uint16_t i = 0;
    uint16_t k = 0;
    for (; i + 1 < pkt_size; i = static_cast<uint16_t>(i + 2), k = static_cast<uint16_t>(k + 3)) {
        if (decode_3outof6(&in_bytes[k], &out_packet[i], 0) != DECODING_3OUTOF6_OK) {
            return PACKET_CODING_ERROR;
        }
    }
    if (i < pkt_size && decode_3outof6(&in_bytes[k], &out_packet[i], 1) != DECODING_3OUTOF6_OK) {
        return PACKET_CODING_ERROR;
    }
    return blocks_ok(out_packet, pkt_size) ? PACKET_OK : PACKET_CRC_ERROR;
}

}  // namespace culfw32::wmbus
