// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "hmuartlgw_frame.hpp"

#include <cstring>

#include "esp_log.h"

namespace culfw32 {

namespace {
// Harte Obergrenze fuer den RX-Akkumulator. Single-Frame ist auf length<=1024
// gedeckelt (_try_parse_one); diese Grenze faengt das ANDERE Wachstumsmuster:
// ein flutender/Muell-emittierender Client (TCP :2325/:2327 oder UDP-Tunnel)
// der nie eine valide 0xfd-Frame-Sequenz bildet, sodass der 1-Byte-Resync nie
// genug konsumiert. Ohne Cap waechst buf_ unbegrenzt → OOM → mit
// -fno-exceptions abort(). Bei Ueberlauf: buf_ verwerfen (ein um Kilobytes
// desynchronisierter Puffer enthaelt keinen brauchbaren Frame-Kopf mehr).
constexpr size_t kHmuMaxBuf = 8192;
} // anon

uint16_t hmu_crc16(std::span<const uint8_t> data) {
    uint16_t crc = 0xD77F;
    for (uint8_t b : data) {
        crc ^= static_cast<uint16_t>(b) << 8;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000) crc = static_cast<uint16_t>((crc << 1) ^ 0x8005);
            else              crc = static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

void hmu_escape_into(std::span<const uint8_t> in, std::vector<uint8_t>& out) {
    out.reserve(out.size() + in.size());
    for (uint8_t b : in) {
        if (b == kHmuStart || b == kHmuEsc) {
            out.push_back(kHmuEsc);
            out.push_back(static_cast<uint8_t>(b ^ 0x80));
        } else {
            out.push_back(b);
        }
    }
}

std::vector<uint8_t> HmuFrame::encode() const {
    // Body: dst + cnt + payload. CRC ueber [START, len_hi, len_lo, body].
    const uint16_t length = static_cast<uint16_t>(2 + payload.size());

    std::vector<uint8_t> crc_input;
    crc_input.reserve(3 + 2 + payload.size());
    crc_input.push_back(kHmuStart);
    crc_input.push_back(static_cast<uint8_t>(length >> 8));
    crc_input.push_back(static_cast<uint8_t>(length & 0xFF));
    crc_input.push_back(dst);
    crc_input.push_back(cnt);
    for (uint8_t b : payload) crc_input.push_back(b);

    const uint16_t crc = hmu_crc16(crc_input);

    // Wire: Start ist NICHT escaped. Alles dahinter (length+body+crc) waere escaped.
    // FHEM-Verifikation 2026-04-28: in 14 Init-Steps kam KEIN ESC-Sequenz vor —
    // Length-Field wird laut Source allerdings escape-aware gelesen. Wir
    // escapen daher konsistent alles nach dem Start-Byte.
    std::vector<uint8_t> after_start;
    after_start.reserve(crc_input.size() - 1 + 2);
    for (size_t i = 1; i < crc_input.size(); ++i) after_start.push_back(crc_input[i]);
    after_start.push_back(static_cast<uint8_t>(crc >> 8));
    after_start.push_back(static_cast<uint8_t>(crc & 0xFF));

    std::vector<uint8_t> out;
    out.reserve(1 + after_start.size());
    out.push_back(kHmuStart);
    hmu_escape_into(after_start, out);
    return out;
}

void HmuFrameDecoder::feed(std::span<const uint8_t> bytes) {
    // Overflow-Schutz VOR dem insert — Allokation bounden, nicht Fehler fangen
    // (-fno-exceptions: ein fehlgeschlagenes vector-Wachstum ruft abort()).
    if (buf_.size() + bytes.size() > kHmuMaxBuf) {
        ESP_LOGW("fe-hmu", "decoder buf overflow (%u+%u > %u), resync",
                 static_cast<unsigned>(buf_.size()),
                 static_cast<unsigned>(bytes.size()),
                 static_cast<unsigned>(kHmuMaxBuf));
        buf_.clear();
        // Neuer Block kann selbst groesser als das Cap sein (Riesen-Burst in
        // einem read()) — dann nur den letzten kMaxBuf-Schwanz behalten, der
        // den juengsten potentiellen 0xfd-Kopf enthaelt.
        if (bytes.size() > kHmuMaxBuf) {
            bytes = bytes.subspan(bytes.size() - kHmuMaxBuf);
        }
    }
    buf_.insert(buf_.end(), bytes.begin(), bytes.end());
}

bool HmuFrameDecoder::try_pop(HmuFrame& out) {
    while (true) {
        const int r = _try_parse_one(out);
        if (r == 1)  return true;
        if (r == 0)  return false;
        // r == -1: weiter resync
    }
}

namespace {

// Unescape n_unesc Bytes aus `in` (alles nach dem Start-Byte). Liefert die
// unescape-bytes UND die Anzahl raw-bytes die konsumiert wurden. Bei
// dangling ESC am Ende: returns false (Caller muss mehr Bytes sammeln).
bool unescape_n(std::span<const uint8_t> in, size_t n_unesc,
                std::vector<uint8_t>& out, size_t& raw_consumed) {
    out.clear();
    out.reserve(n_unesc);
    size_t i = 0;
    while (out.size() < n_unesc) {
        if (i >= in.size()) return false;       // unvollstaendig
        if (in[i] == kHmuEsc) {
            if (i + 1 >= in.size()) return false;
            out.push_back(static_cast<uint8_t>(in[i + 1] ^ 0x80));
            i += 2;
        } else {
            out.push_back(in[i]);
            i += 1;
        }
    }
    raw_consumed = i;
    return true;
}

} // anon

int HmuFrameDecoder::_try_parse_one(HmuFrame& out) {
    // Resync auf 0xfd
    while (!buf_.empty() && buf_[0] != kHmuStart) {
        buf_.erase(buf_.begin());
    }
    if (buf_.size() < 3) return 0;             // mindestens Start + 2 Length-Bytes

    // Wir muessen length vor allem anderen unescape-aware lesen.
    std::span<const uint8_t> after_start(buf_.data() + 1, buf_.size() - 1);
    std::vector<uint8_t> unesc;
    size_t raw_consumed = 0;
    if (!unescape_n(after_start, 2, unesc, raw_consumed)) return 0;

    const uint16_t length = static_cast<uint16_t>((unesc[0] << 8) | unesc[1]);
    // length ist dst+cnt+payload — sollte mindestens 2 (dst+cnt) sein, max sane.
    if (length < 2 || length > 1024) {
        buf_.erase(buf_.begin());
        return -1;
    }

    // Brauchen length + 2 CRC bytes (alle escape-pflichtig).
    const size_t n_total_unesc = 2 + length + 2;
    if (!unescape_n(after_start, n_total_unesc, unesc, raw_consumed)) return 0;

    // CRC ueber [START + length_field + body] — also unesc[0..2+length-1] mit
    // vorangestelltem 0xfd.
    std::vector<uint8_t> crc_input;
    crc_input.reserve(1 + 2 + length);
    crc_input.push_back(kHmuStart);
    for (size_t i = 0; i < 2 + length; ++i) crc_input.push_back(unesc[i]);
    const uint16_t calc = hmu_crc16(crc_input);
    const uint16_t recv = static_cast<uint16_t>((unesc[2 + length] << 8) | unesc[2 + length + 1]);
    if (calc != recv) {
        buf_.erase(buf_.begin());
        return -1;
    }

    out.dst = unesc[2];
    out.cnt = unesc[3];
    out.payload.assign(unesc.begin() + 4, unesc.begin() + 2 + length);

    // Konsumiere die raw-bytes plus das Start-Byte.
    buf_.erase(buf_.begin(), buf_.begin() + 1 + raw_consumed);
    return 1;
}

} // namespace culfw32
