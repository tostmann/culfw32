#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""Send synthetic AskSin frames via local CUL868 (/dev/ttyACM2).

Uses tty.setraw mandatory (siehe Memory feedback_tty_setraw_for_cul.md).
"""
from __future__ import annotations

import argparse
import os
import sys
import termios
import time
import tty


def open_raw(path: str) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    # 38400 8N1 (standard CUL baudrate)
    attrs = termios.tcgetattr(fd)
    # cflag indices: [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
    speed = termios.B38400
    attrs[4] = speed
    attrs[5] = speed
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def write(fd: int, s: str):
    os.write(fd, s.encode("ascii"))


def drain(fd: int, timeout: float = 0.5) -> str:
    import select
    out = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        r, _, _ = select.select([fd], [], [], min(0.1, deadline - time.time()))
        if not r:
            continue
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            continue
        if not chunk:
            continue
        out.extend(chunk)
        deadline = time.time() + 0.15  # extend a bit while data flowing
    return out.decode("ascii", errors="replace")


def send_frame(fd: int, hexframe: str, label: str = ""):
    cmd = f"As{hexframe}\r\n"
    print(f"\n[CUL868 TX] {label}: {cmd.strip()}")
    write(fd, cmd)
    reply = drain(fd, 0.6)
    if reply:
        print(f"[CUL868 reply] {reply!r}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/ttyACM2")
    ap.add_argument("--repeat", type=int, default=1, help="repeat each frame N times with 200ms delay")
    args = ap.parse_args()

    fd = open_raw(args.device)
    try:
        # 1) Get version
        write(fd, "V\r\n")
        v = drain(fd, 0.3)
        print(f"[CUL868 V] {v!r}")

        # 2) Activate Ar mode (AskSin RX/TX)
        write(fd, "Ar\r\n")
        ar = drain(fd, 0.3)
        print(f"[CUL868 Ar] {ar!r}")
        time.sleep(0.1)

        # 3) Compose three test frames.
        # Standard AskSin frame layout:
        #   LEN | MSG_CNT | MSG_FLAG | MSG_TYPE | SRC(3) | DST(3) | PAYLOAD...
        #   LEN = total bytes after LEN itself (NOT incl. LEN)
        #
        # Frame A: DEVICE_INFO pair-announce (broadcast, mtype=0x00).
        #   src=ABCDEF (fictitious), dst=000000 (broadcast)
        #   payload = [FW=24, magic=0001, serial="JEQ0123456", dev-type=001A, p=01, info=00]
        #   serial 10 bytes ASCII = 4A45513031323334353600
        #   Body: 18 0001 4A45513031323334353600 0001 00
        #   size of body = 17 bytes
        #   LEN = 1+1+1+3+3+17 = 26 = 0x1A
        body_a = bytes.fromhex("18000150455131323334353600001A0001")
        # rebuild with correct serial bytes:
        # "PEQ1234567" = 50 45 51 31 32 33 34 35 36 37
        # FW=0x18=24 (.), magic 0x00 0x01, serial(10), dev-type-hi/lo 0x00 0x1A, two trailing 0x00 0x01
        # That's 1+2+10+2+2 = 17. Let me hand-pack:
        body_a = bytes([0x18, 0x00, 0x01]) + b"PEQ1234567" + bytes([0x00, 0x1A, 0x00, 0x01])
        assert len(body_a) == 17
        msg_a = bytes([0xA4, 0x84, 0x00]) + bytes.fromhex("ABCDEF") + bytes.fromhex("000000") + body_a
        len_a = len(msg_a)  # = 26
        frame_a = bytes([len_a]) + msg_a
        send_frame(fd, frame_a.hex().upper(), label="DEVICE_INFO pair-announce src=ABCDEF dst=000000")
        time.sleep(0.3)

        # Frame B: simple INFO_ACTUATOR_STATUS from a known source to F11234 (our CCU).
        #   mtype=0x10 (INFO), payload = [06=ACTUATOR_STATUS, ch=01, level=C8, status=00, RSSI-byte=00]
        #   LEN = 9 + 5 = 14 = 0x0E
        body_b = bytes([0x06, 0x01, 0xC8, 0x00, 0x00])
        msg_b = bytes([0x55, 0xA4, 0x10]) + bytes.fromhex("AABBCC") + bytes.fromhex("F11234") + body_b
        frame_b = bytes([len(msg_b)]) + msg_b
        send_frame(fd, frame_b.hex().upper(), label="INFO_ACTUATOR_STATUS src=AABBCC dst=F11234")
        time.sleep(0.3)

        # Frame C: ACK from device to F11234.
        #   mtype=0x02 (ACK), payload = [00=ACK_OK]
        #   LEN = 9 + 1 = 10 = 0x0A
        body_c = bytes([0x00])
        msg_c = bytes([0x77, 0x80, 0x02]) + bytes.fromhex("AABBCC") + bytes.fromhex("F11234") + body_c
        frame_c = bytes([len(msg_c)]) + msg_c
        send_frame(fd, frame_c.hex().upper(), label="ACK src=AABBCC dst=F11234")

    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
