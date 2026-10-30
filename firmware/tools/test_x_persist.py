#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""Reboot-Persistenz: x09 setzen, Reset via B, prüfe x09-Werte sind erhalten."""
import os, sys, time, tty
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _lab_env import CULFW32_DEV as CULFW32  # noqa: E402

def open_raw(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    return fd

def cmd(fd, line, wait=0.2):
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

def read_pa(fd):
    bytes_ = []
    for addr in range(0x2B, 0x33):
        out = cmd(fd, f"R{addr:04X}", wait=0.1)
        try:
            v = out.split("=")[1].strip().split("/")[0].strip()
            bytes_.append(int(v, 16))
        except (IndexError, ValueError):
            bytes_.append(None)
    return bytes_

def read_frend0(fd):
    out = cmd(fd, "R0024", wait=0.1)  # FREND0 = EE_CC1100_CFG (2) + 0x22 = 0x24
    try:
        return int(out.split("=")[1].strip().split("/")[0].strip(), 16)
    except (IndexError, ValueError):
        return None

print("Schritt 1: x09 setzen (10 dBm no ramping)")
cul = open_raw(CULFW32)
cmd(cul, "x09", wait=0.3)
pa1     = read_pa(cul)
frend01 = read_frend0(cul)
print(f"  PATABLE: {' '.join(f'{b:02X}' for b in pa1)}")
print(f"  FREND0 : {frend01:02X}")
print()

print("Schritt 2: Reset via B")
cmd(cul, "B", wait=0.0)
os.close(cul)
time.sleep(3)  # warte auf Reboot

print("Schritt 3: PATABLE-Werte nach Reboot lesen")
cul     = open_raw(CULFW32)
v       = cmd(cul, "V", wait=0.3)
print(f"  Boot-V: {v}")
pa2     = read_pa(cul)
frend02 = read_frend0(cul)
print(f"  PATABLE: {' '.join(f'{b:02X}' for b in pa2)}")
print(f"  FREND0 : {frend02:02X}")
print()

if pa1 == pa2 and frend01 == frend02:
    print("PERSISTENZ OK: PATABLE + FREND0 ueberleben Reboot byte-genau.")
else:
    print(f"PERSISTENZ DIFF:")
    print(f"  PATABLE  : {pa1} -> {pa2}")
    print(f"  FREND0   : {frend01} -> {frend02}")

os.close(cul)
