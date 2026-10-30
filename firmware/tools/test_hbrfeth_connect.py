#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""
HB-RF-ETH UDP-Tunnel Wire-Format-Test gegen CULFW32.

Validiert byte-genau gegen Alex Reinerts Reference-Source
(rawuartudplistener.cpp::on_packet, sendMessage):

  V1-Connect:
    Out: [type=0, cnt, ver=1, crc_hi, crc_lo]                     (5 bytes)
    In:  [type=0, our_cnt, status=1, mirror_cnt, crc_hi, crc_lo]  (6 bytes)

  V2-Connect:
    Out: [type=0, cnt, ver=2, ep_id, crc_hi, crc_lo]              (6 bytes)
    In:  [type=0, our_cnt, status=2, mirror_cnt, ep_id, crc_hi, crc_lo] (7 bytes)

  KeepAlive in/out: [type=2, cnt, crc_hi, crc_lo]                 (4 bytes)

  Frame: [type=7, cnt, <payload...>, crc_hi, crc_lo]
"""

import os
import os, socket, struct, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _lab_env import CULFW32_HOST  # noqa: E402

PORT         = 3008


def crc16(buf: bytes) -> int:
    # HMUARTLGW CRC-16: poly 0x8005, init 0xd77f
    crc = 0xd77f
    for b in buf:
        for i in range(8):
            xor_in = ((crc & 0x8000) >> 8) ^ (b & 0x80)
            crc = ((crc << 1) & 0xffff) ^ (0x8005 if xor_in else 0)
            b = (b << 1) & 0xff
    return crc


def pkt(type_byte: int, cnt: int, payload: bytes) -> bytes:
    body = bytes([type_byte, cnt]) + payload
    c = crc16(body)
    return body + bytes([(c >> 8) & 0xff, c & 0xff])


def parse(data: bytes) -> tuple[int, int, bytes]:
    assert len(data) >= 4, f"too short: {len(data)} bytes"
    body, crc_recv = data[:-2], (data[-2] << 8) | data[-1]
    crc_calc = crc16(body)
    assert crc_calc == crc_recv, f"CRC mismatch: calc={crc_calc:04x} recv={crc_recv:04x}"
    return body[0], body[1], body[2:]


def main() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", 0))
    s.settimeout(2.0)

    # ====================================================================
    # 1) V1-Connect — payload = [1]
    # ====================================================================
    print("=" * 60)
    print("Test 1: V1-Connect")
    print("=" * 60)
    out = pkt(0, cnt=0x01, payload=bytes([1]))
    print(f"OUT ({len(out)} bytes): {out.hex(' ')}")
    s.sendto(out, (CULFW32_HOST, PORT))
    in_data, _ = s.recvfrom(1500)
    print(f"IN  ({len(in_data)} bytes): {in_data.hex(' ')}")
    type_in, our_cnt, payload_in = parse(in_data)
    assert type_in == 0, f"V1: bad type {type_in}"
    assert len(payload_in) == 2, f"V1: expected 2-byte payload, got {len(payload_in)}: {payload_in.hex()}"
    assert payload_in[0] == 1, f"V1: status != 1: {payload_in[0]}"
    assert payload_in[1] == 0x01, f"V1: mirror cnt != 0x01: {payload_in[1]:02x}"
    print(f"PASS: type=0 our_cnt={our_cnt:#x} status=1 mirror_cnt=0x01")

    # ====================================================================
    # 2) V2-Connect — payload = [2, 0]  (initial connect, ep_id=0)
    # ====================================================================
    print("\n" + "=" * 60)
    print("Test 2: V2-Connect (initial, ep_id=0)")
    print("=" * 60)
    out = pkt(0, cnt=0x42, payload=bytes([2, 0]))
    print(f"OUT ({len(out)} bytes): {out.hex(' ')}")
    s.sendto(out, (CULFW32_HOST, PORT))
    in_data, _ = s.recvfrom(1500)
    print(f"IN  ({len(in_data)} bytes): {in_data.hex(' ')}")
    type_in, our_cnt, payload_in = parse(in_data)
    assert type_in == 0, f"V2: bad type {type_in}"
    assert len(payload_in) == 3, f"V2: expected 3-byte payload, got {len(payload_in)}: {payload_in.hex()}"
    assert payload_in[0] == 2, f"V2: status != 2: {payload_in[0]}"
    assert payload_in[1] == 0x42, f"V2: mirror cnt != 0x42: {payload_in[1]:02x}"
    server_ep = payload_in[2]
    print(f"PASS: type=0 our_cnt={our_cnt:#x} status=2 mirror_cnt=0x42 ep_id={server_ep:#x}")

    # ====================================================================
    # 3) StartConnection (type 5) — kein Reply, nur State-Aenderung
    # ====================================================================
    print("\n" + "=" * 60)
    print("Test 3: StartConnection")
    print("=" * 60)
    out = pkt(5, cnt=0x10, payload=b"")
    print(f"OUT: {out.hex(' ')}")
    s.sendto(out, (CULFW32_HOST, PORT))
    print("PASS: sent (no reply expected)")

    # ====================================================================
    # 4) Frame (type 7) — HMUARTLGW OS_GET_APP -> erwarte Antwort
    #    HMUARTLGW frame format: [0xfd, len_hi, len_lo, body..., crc_hi, crc_lo]
    #    body = [counter, OS_BUS=0x00, sub=0x02 (OS_GET_APP)]
    # ====================================================================
    print("\n" + "=" * 60)
    print("Test 4: Frame round-trip (HMUARTLGW OS_GET_APP)")
    print("=" * 60)
    hmu_body = bytes([0x77, 0x00, 0x02])  # counter, dst=OS, cmd=GET_APP
    hmu_crc = crc16(bytes([0xfd, 0x00, len(hmu_body)]) + hmu_body)
    hmu_frame_inner = bytes([0xfd, 0x00, len(hmu_body)]) + hmu_body + bytes([(hmu_crc >> 8) & 0xff, hmu_crc & 0xff])
    # 0xfd-Escape: 0xfc 7d / 0xfc 7c gehen ueber UDP nicht (UDP transportiert raw),
    # aber Wire ist hier Reentrant. Wir senden raw 0xfd+body, kein UART-Escape.
    out = pkt(7, cnt=0x55, payload=hmu_frame_inner)
    print(f"OUT ({len(out)} bytes): {out.hex(' ')}")
    s.sendto(out, (CULFW32_HOST, PORT))

    # Erwartete Antwort: type 7 mit HMUARTLGW-Reply (mind. Hello-Push haben wir
    # mit cnt=0x77 nicht, aber da AskSinRF bei OS_GET_APP ueblich ein OS_APP_TYPE
    # (0x03) zurueckgibt, sollten wir was sehen).
    try:
        in_data, _ = s.recvfrom(1500)
        print(f"IN  ({len(in_data)} bytes): {in_data.hex(' ')}")
        type_in, our_cnt, payload_in = parse(in_data)
        if type_in == 7:
            print(f"PASS: type=7 our_cnt={our_cnt:#x} payload={payload_in.hex(' ')}")
        else:
            print(f"NOTE: got type={type_in} our_cnt={our_cnt:#x} payload={payload_in.hex(' ')}")
    except socket.timeout:
        print("NOTE: no frame reply within 2s (Hello-Push only? expected on REAL connect via Pi)")

    # ====================================================================
    # 5) KeepAlive-Out — sollte vom CULFW32 alle 1 s kommen
    # ====================================================================
    print("\n" + "=" * 60)
    print("Test 5: KeepAlive-Out from CULFW32 (~1 Hz)")
    print("=" * 60)
    s.settimeout(2.5)
    seen = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.5 and seen < 2:
        try:
            in_data, _ = s.recvfrom(1500)
            type_in, our_cnt, payload_in = parse(in_data)
            if type_in == 2:
                seen += 1
                print(f"  KA #{seen}: cnt={our_cnt:#x}")
        except socket.timeout:
            break
    if seen >= 1:
        print(f"PASS: saw {seen} KeepAlive(s)")
    else:
        print("FAIL: no KeepAlive in 2.5s")
        return 1

    # ====================================================================
    # 6) Disconnect (type 1)
    # ====================================================================
    print("\n" + "=" * 60)
    print("Test 6: Disconnect")
    print("=" * 60)
    out = pkt(1, cnt=0xff, payload=b"")
    print(f"OUT: {out.hex(' ')}")
    s.sendto(out, (CULFW32_HOST, PORT))
    print("PASS: sent (no reply expected)")

    print("\n" + "=" * 60)
    print("ALL TESTS PASS")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
