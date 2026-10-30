#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""E<hex> EM-Send Roundtrip CULFW32 ↔ Lokales-CUL868."""
import os, sys, time, tty
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _lab_env import CULFW32_DEV as CULFW32, CUL868_DEV as LOKAL  # noqa: E402

def open_raw(p):
    fd = os.open(p, os.O_RDWR | os.O_NOCTTY); tty.setraw(fd); return fd

def cmd(fd, line, wait=0.3):
    os.write(fd, (line + "\r\n").encode()); time.sleep(wait)
    out = b""
    while True:
        try:
            os.set_blocking(fd, False)
            chunk = os.read(fd, 4096)
            if not chunk: break
            out += chunk
        except BlockingIOError: break
    return out.decode("ascii", errors="replace").replace("\r","").strip()

def drain(fd, dur=2.0):
    end = time.time()+dur; out = b""
    os.set_blocking(fd, False)
    while time.time() < end:
        try:
            chunk = os.read(fd, 4096)
            if chunk: out += chunk
            else: time.sleep(0.05)
        except BlockingIOError: time.sleep(0.05)
    return out.decode("ascii", errors="replace").replace("\r","").strip()

cul = open_raw(CULFW32); lok = open_raw(LOKAL)

print("=== Setup ===")
print("CULFW32 V :", cmd(cul, "V"))
print("Lokales V :", cmd(lok, "V"))

# RX-Mode: X21 = REP_KNOWN | REP_RSSI
cmd(cul, "X21"); cmd(lok, "X21"); time.sleep(0.5)
drain(cul, 0.5); drain(lok, 0.5)

# EM ist strikt 9 Datenbytes (a-culfw rf_send.c:292). Test mit a-culfw-Beispiel
# `E0205E7000000000000` (aus Source-Comment) plus weitere Probe-Frames.
TESTS_OK = [
    "E0205E7000000000000",     # a-culfw Beispiel rf_send.c:281
    "E010203040506070809",     # incrementing
    "EAABBCCDDEEFF0011223",
]

for hexcmd in TESTS_OK:
    print(f"\n=== CULFW32 → Lokales: {hexcmd} ===")
    drain(lok, 0.2)
    cmd(cul, hexcmd, wait=0.0)
    rx = drain(lok, 1.5)
    print(f"  Lokales RX: {rx if rx else '<keine>'}")

# LENERR-Test: <9 byte → LENERR (a-culfw silent → CULFW32 aktiv)
print(f"\n=== LENERR-Test (CULFW32 erwartet aktive LENERR-Antwort) ===")
print("E12345678 (4 bytes):", repr(cmd(cul, "E12345678")))
print("Lokales x12345678 (silent):", repr(cmd(lok, "E12345678")))

# Reverse: Lokales → CULFW32
print(f"\n=== Lokales → CULFW32: E0205E7000000000000 ===")
drain(cul, 0.2)
cmd(lok, "E0205E7000000000000", wait=0.0)
print(f"  CULFW32 RX: {drain(cul, 1.5) or '<keine>'}")

cmd(cul, "X00"); cmd(lok, "X00")
os.close(cul); os.close(lok)
