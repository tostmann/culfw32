# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""HMUARTLGW frame parser/builder — Reference implementation.

Wire-Format (aus FHEM 00_HMUARTLGW.pm extrahiert):

    0xfd <len_hi> <len_lo> <dst> <cnt> <payload...> <crc_hi> <crc_lo>

length  = dst + cnt + payload bytes (CRC nicht eingerechnet)
escape  = im Payload werden 0xfd und 0xfc ersetzt durch 0xfc + (byte ^ 0x80)
crc16   = poly 0x8005, init 0xd77f, byte-weise XOR ins Hi-Byte, 8x rotieren,
          MSB→XOR mit Polynom

Diese Datei ist Spec-Referenz fuer die spaetere C++-Implementation in
firmware/components/frontend_hmuartlgw/.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

START = 0xFD
ESC = 0xFC

DST_OS = 0x00
DST_APP = 0x01
DST_DUAL = 0xFE
DST_DUAL_ERR = 0xFF

OS_CMDS = {
    0x00: "OS_GET_APP",
    0x02: "OS_GET_FIRMWARE",
    0x03: "OS_CHANGE_APP",
    0x04: "OS_ACK",
    0x05: "OS_UPDATE_FIRMWARE",
    0x06: "OS_NORMAL_MODE",
    0x07: "OS_UPDATE_MODE",
    0x08: "OS_GET_CREDITS",
    0x09: "OS_ENABLE_CREDITS",
    0x0A: "OS_ENABLE_CSMACA",
    0x0B: "OS_GET_SERIAL",
    0x0E: "OS_SET_TIME",
}

APP_CMDS = {
    0x00: "APP_SET_HMID",
    0x01: "APP_GET_HMID",
    0x02: "APP_SEND",
    0x03: "APP_SET_CURRENT_KEY",
    0x04: "APP_ACK",
    0x05: "APP_RECV",
    0x06: "APP_ADD_PEER",
    0x07: "APP_REMOVE_PEER",
    0x08: "APP_GET_PEERS",
    0x09: "APP_PEER_ADD_AES",
    0x0A: "APP_PEER_REMOVE_AES",
    0x0B: "APP_SET_TEMP_KEY",
    0x0F: "APP_SET_PREVIOUS_KEY",
    0x10: "APP_DEFAULT_HMID",
}

ACK_CODES = {
    0x00: "NACK",
    0x01: "ACK",
    0x02: "ACK_INFO",
    0x03: "ACK_WITH_RESPONSE",
    0x04: "ACK_EUNKNOWN",
    0x05: "ACK_ENOCREDITS",
    0x06: "ACK_ECSMACA",
    0x07: "ACK_WITH_MULTIPART_DATA",
    0x08: "ACK_EINPROGRESS",
    0x0C: "ACK_WITH_RESPONSE_AES_OK",
    0x0D: "ACK_WITH_RESPONSE_AES_KO",
}


def crc16(data: bytes) -> int:
    """CRC-16 wie HMUARTLGW: poly 0x8005, init 0xd77f."""
    crc = 0xD77F
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8005) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def escape(payload: bytes) -> bytes:
    """0xfd und 0xfc als 0xfc + (byte ^ 0x80) ersetzen."""
    out = bytearray()
    for b in payload:
        if b == START or b == ESC:
            out.append(ESC)
            out.append(b ^ 0x80)
        else:
            out.append(b)
    return bytes(out)


def unescape(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        if data[i] == ESC and i + 1 < len(data):
            out.append(data[i + 1] ^ 0x80)
            i += 2
        else:
            out.append(data[i])
            i += 1
    return bytes(out)


@dataclass
class Frame:
    dst: int
    cnt: int
    payload: bytes  # ohne dst/cnt-bytes

    def cmd_byte(self) -> int:
        return self.payload[0] if self.payload else -1

    def cmd_name(self) -> str:
        c = self.cmd_byte()
        if c < 0:
            return "<empty>"
        if self.dst == DST_OS:
            return OS_CMDS.get(c, f"OS_UNK_{c:02x}")
        if self.dst == DST_APP:
            return APP_CMDS.get(c, f"APP_UNK_{c:02x}")
        return f"DST{self.dst}_CMD_{c:02x}"

    def pretty(self) -> str:
        body = self.payload[1:].hex() if len(self.payload) > 1 else ""
        return f"dst={self.dst:02x}({_dst_name(self.dst)}) cnt={self.cnt:02x} {self.cmd_name()} body={body}"

    def encode(self) -> bytes:
        """Raw bytes inkl. Start-Byte, len, escape und CRC."""
        body = bytes([self.dst, self.cnt]) + self.payload
        length = len(body)
        crc = crc16(bytes([START, (length >> 8) & 0xFF, length & 0xFF]) + body)
        crc_bytes = bytes([(crc >> 8) & 0xFF, crc & 0xFF])
        return bytes([START]) + escape(
            bytes([(length >> 8) & 0xFF, length & 0xFF]) + body + crc_bytes
        )


def _dst_name(d: int) -> str:
    return {DST_OS: "OS", DST_APP: "APP", DST_DUAL: "DUAL", DST_DUAL_ERR: "DUAL_ERR"}.get(
        d, f"DST_{d:02x}"
    )


class FrameDecoder:
    """Stream-Decoder: feed() Bytes rein, frames() liefert komplette Frames raus.

    Hinweis zur Escape-Stelle: FHEM-Source eskapiert NUR den Body (inkl. CRC),
    NICHT das Length-Feld. Wir muessen daher zuerst length unescape-frei lesen,
    dann den Rest unescapen.
    """

    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes) -> None:
        self.buf.extend(data)

    def frames(self):
        while True:
            f = self._try_parse_one()
            if f is None:
                return
            yield f

    def _try_parse_one(self) -> Optional[Frame]:
        # Resync auf 0xfd
        while self.buf and self.buf[0] != START:
            self.buf.pop(0)
        if len(self.buf) < 3:
            return None

        # length sind die 2 bytes NACH 0xfd. Diese werden auch escaped — also
        # holen wir uns alle bytes nach 0xfd, unescapen den Stream stueckweise
        # bis wir wissen wie lang der gesamte Frame sein muss.
        unesc = unescape_stream(self.buf[1:])
        if unesc is None:
            return None
        unesc_bytes, raw_consumed = unesc
        if len(unesc_bytes) < 2:
            return None

        length = (unesc_bytes[0] << 8) | unesc_bytes[1]
        # length = dst + cnt + payload  (CRC noch separat 2 byte)
        need_unesc = 2 + length + 2  # length-field + body + crc
        if len(unesc_bytes) < need_unesc:
            return None

        body = unesc_bytes[2 : 2 + length]
        crc_recv = (unesc_bytes[2 + length] << 8) | unesc_bytes[2 + length + 1]
        crc_calc = crc16(bytes([START]) + unesc_bytes[: 2 + length])
        if crc_recv != crc_calc:
            # CRC-Fehler — verwerfe das 0xfd-Byte und resync
            self.buf.pop(0)
            return None

        # Wir muessen wissen wie viele RAW (escaped) Bytes wir konsumiert haben.
        # Dafuer encoden wir das Resultat zurueck und vergleichen mit dem Buffer.
        # Trick: wir wissen `need_unesc` unescape-bytes -> berechne raw-bytes
        # durch erneutes Escape vom decoded portion.
        raw_consumed_now = _measure_raw_bytes(self.buf[1:], need_unesc)
        if raw_consumed_now is None:
            return None
        del self.buf[: 1 + raw_consumed_now]

        return Frame(dst=body[0], cnt=body[1], payload=bytes(body[2:]))


def unescape_stream(data: bytes):
    """Unescape; bei dangling ESC am Ende: None zurueck (mehr Bytes brauchen)."""
    out = bytearray()
    i = 0
    while i < len(data):
        if data[i] == ESC:
            if i + 1 >= len(data):
                return None
            out.append(data[i + 1] ^ 0x80)
            i += 2
        else:
            out.append(data[i])
            i += 1
    return bytes(out), i


def _measure_raw_bytes(data: bytes, n_unesc: int):
    """Wieviele RAW-bytes braucht man um n_unesc unescape-bytes zu konsumieren?"""
    i = 0
    consumed = 0
    while consumed < n_unesc and i < len(data):
        if data[i] == ESC:
            if i + 1 >= len(data):
                return None
            i += 2
        else:
            i += 1
        consumed += 1
    if consumed < n_unesc:
        return None
    return i


# Selbsttest mit dem Forum-Beispiel "Co_CPU_BL?" (HM-MOD-RPI-PCB Boot-Probe).
if __name__ == "__main__":
    sample = bytes.fromhex("FD000D000004024370 5F4350 555F424C3F".replace(" ", ""))
    print("sample:", sample.hex())
    dec = FrameDecoder()
    dec.feed(sample)
    for f in dec.frames():
        print("decoded:", f.pretty())
    # Wir erwarten dass das Sample evtl. ohne CRC ist (Forum-Quote war nicht
    # vollstaendig). Das Selbsttest-Ergebnis dient nur als visueller Sanity-Check.
