#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""KS300-RX-Test: Lokales-CUL868 sendet K, CULFW32 empfaengt."""
import os, sys, time, tty
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _lab_env import CULFW32_DEV as CULFW32, CUL868_DEV as LOKAL  # noqa: E402

def open_raw(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    return fd

def cmd(fd, line, wait=0.3):
    os.write(fd, (line + "\r\n").encode())
    time.sleep(wait)
    out = b""
    while True:
        try:
            os.set_blocking(fd, False)
            chunk = os.read(fd, 4096)
            if not chunk: break
            out += chunk
        except BlockingIOError:
            break
    return out.decode("ascii", errors="replace").replace("\r", "").strip()

def drain(fd, dur=2.0):
    end = time.time() + dur
    out = b""
    os.set_blocking(fd, False)
    while time.time() < end:
        try:
            chunk = os.read(fd, 4096)
            if chunk: out += chunk
            else: time.sleep(0.05)
        except BlockingIOError: time.sleep(0.05)
    return out.decode("ascii", errors="replace").replace("\r", "").strip()

cul = open_raw(CULFW32)
lok = open_raw(LOKAL)

# Reset both to RX with all reports
cmd(cul, "X21")
cmd(lok, "X21")
time.sleep(0.5)
drain(cul, 0.5); drain(lok, 0.5)

TESTS = [
    "K12345678",
    "KAABBCCDDEE",
    "K11223344556677",
]

for hexcmd in TESTS:
    print(f"\n=== Lokales sendet: {hexcmd} ===")
    drain(cul, 0.2)
    cmd(lok, hexcmd, wait=0.0)
    rx = drain(cul, 1.5)
    print(f"  CULFW32 RX: {rx if rx else '<keine>'}")

cmd(cul, "X00"); cmd(lok, "X00")
os.close(cul); os.close(lok)
