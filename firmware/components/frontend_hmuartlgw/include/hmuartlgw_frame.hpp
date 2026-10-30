// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

// HMUARTLGW Wire-Frame-Layer.
//
// Wire-Format (verifiziert gegen FHEM 00_HMUARTLGW.pm + Live-Trace 2026-04-28):
//
//     0xfd <len_hi> <len_lo> <dst> <cnt> <payload...> <crc_hi> <crc_lo>
//
// length  = dst + cnt + payload bytes (CRC NICHT eingerechnet)
// escape  = im Stream (nach 0xfd) werden 0xfd und 0xfc durch 0xfc + (b ^ 0x80)
//           ersetzt. Length-Bytes UND Body UND CRC durchlaufen die Escape-Stufe.
// crc16   = poly 0x8005, init 0xd77f, byte-weise XOR ins Hi-Byte, 8x rotieren,
//           MSB→XOR mit Polynom. Berechnet ueber [0xfd, len_hi, len_lo, dst,
//           cnt, payload...] (alles BEVOR escape).
//
// Header-only-API; Implementation in hmuartlgw_frame.cpp.

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace culfw32 {

inline constexpr uint8_t kHmuStart  = 0xFD;
inline constexpr uint8_t kHmuEsc    = 0xFC;
// HM-Layer-Adressen byte-genau pivccu detect_radio_module/hmframe.h:41-45.
// Naming: Os=HMSYSTEM (Legacy), App=TRX (auch HmIP-TRX-Layer), Hmip=HMIP-Layer
// (DualCoPro), Llmac=Low-Level-MAC (BidCoS-Default-RF-Address bei DualCoPro),
// Common=COMMON (DualCoPro Identifikations-Layer).
inline constexpr uint8_t kHmuDstOs       = 0x00;   // HMSYSTEM
inline constexpr uint8_t kHmuDstApp      = 0x01;   // TRX (BidCoS App-Layer)
inline constexpr uint8_t kHmuDstHmip     = 0x02;   // HMIP (DualCoPro)
inline constexpr uint8_t kHmuDstLlmac    = 0x03;   // LLMAC (DualCoPro BidCoS-MAC)
inline constexpr uint8_t kHmuDstCommon   = 0xFE;   // COMMON (DualCoPro identify)
inline constexpr uint8_t kHmuDstDualErr  = 0xFF;
// Backwards-compat-Alias — bisheriger Code nennt 0xFE "kHmuDstDual".
inline constexpr uint8_t kHmuDstDual     = kHmuDstCommon;

// CRC-16 Polynom 0x8005 mit Init-Wert 0xd77f (byte-weise gegen FHEM verifiziert).
uint16_t hmu_crc16(std::span<const uint8_t> data);

// Escape-Encode: input → output mit 0xfd/0xfc-Substitutionen.
void hmu_escape_into(std::span<const uint8_t> in, std::vector<uint8_t>& out);

struct HmuFrame {
    uint8_t  dst = 0;
    uint8_t  cnt = 0;
    std::vector<uint8_t> payload;          // ohne dst/cnt

    [[nodiscard]] uint8_t cmd() const { return payload.empty() ? 0xFF : payload[0]; }

    // Vollstaendige Wire-Bytes inklusive Start/Length/Escape/CRC.
    std::vector<uint8_t> encode() const;
};

// Stream-Decoder fuer eine einzelne TCP/UART-Verbindung. Bytes mit feed()
// reinpushen, danach try_pop() bis leer fuer alle vollstaendigen Frames.
class HmuFrameDecoder {
public:
    void feed(std::span<const uint8_t> bytes);

    // Versucht einen vollstaendigen Frame zu extrahieren. Liefert true wenn
    // out gefuellt wurde. CRC-fehlerhafte Frames werden silent verworfen
    // (resync auf naechstes 0xfd).
    bool try_pop(HmuFrame& out);

    void reset() { buf_.clear(); }
    size_t buffered() const { return buf_.size(); }

private:
    // _try_parse_one liest aus buf_ einen Frame. Returns:
    //   1  → ein Frame wurde nach `out` geschrieben, Buffer-Bytes konsumiert
    //   0  → noch nicht genug Bytes, nichts konsumiert
    //  -1  → CRC-Fehler oder Garbage; ein Byte am Anfang wurde verworfen,
    //        Caller soll erneut versuchen
    int _try_parse_one(HmuFrame& out);

    std::vector<uint8_t> buf_;
};

} // namespace culfw32
