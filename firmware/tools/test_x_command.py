#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""Test x<idx> command (ccsetpa) byte-genau gegen Lokales-CUL868 + EE-Persistenz."""
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

def reg_read(fd, addr):
    """Read register via Cw addr-style. Use 'C<aa>' returns 'C<aa> = <vv> / <dd>\\n'."""
    return cmd(fd, f"C{addr:02X}", wait=0.2)

def patable_dump_via_C99(fd):
    """C99 returns 41-byte cfg dump in 4 lines, then PATABLE on 5th."""
    return cmd(fd, "C99", wait=0.5)

def main():
    cul = open_raw(CULFW32)
    lok = open_raw(LOKAL)

    print("=== CULFW32 baseline ===")
    print(cmd(cul, "V"))
    print()
    print("CULFW32 C99 (vor x):")
    pre = patable_dump_via_C99(cul)
    print(pre)
    print()
    print("Lokales C99 (Referenz):")
    lref = patable_dump_via_C99(lok)
    print(lref)
    print()

    print("=== Test 1: x08 (factory default 5 dBm no ramp) ===")
    print("CULFW32 x08:", repr(cmd(cul, "x08")))
    print("Lokales x08:", repr(cmd(lok, "x08")))
    print()
    print("CULFW32 C99 (nach x08):")
    print(patable_dump_via_C99(cul))
    print()
    print("Lokales C99 (nach x08):")
    print(patable_dump_via_C99(lok))
    print()

    print("=== Test 2: x06 (-5 dBm no ramp) ===")
    print("CULFW32 x06:", repr(cmd(cul, "x06")))
    print("Lokales x06:", repr(cmd(lok, "x06")))
    print()
    print("CULFW32 C99 (nach x06):")
    print(patable_dump_via_C99(cul))
    print()
    print("Lokales C99 (nach x06):")
    print(patable_dump_via_C99(lok))
    print()

    print("=== Test 3: x04 (10 dBm mit Ramping, FREND0=0x17) ===")
    print("CULFW32 x04:", repr(cmd(cul, "x04")))
    print("Lokales x04:", repr(cmd(lok, "x04")))
    print()
    print("CULFW32 C99 (nach x04 — FREND0 sollte 0x17 sein):")
    print(patable_dump_via_C99(cul))
    print()
    print("Lokales C99 (nach x04):")
    print(patable_dump_via_C99(lok))
    print()

    print("=== Test 4: x ohne Argument (default idx=2 = 0 dBm mit Ramping) ===")
    print("CULFW32 x:", repr(cmd(cul, "x")))
    print("Lokales x:", repr(cmd(lok, "x")))
    print()
    print("CULFW32 C99:")
    print(patable_dump_via_C99(cul))
    print()
    print("Lokales C99:")
    print(patable_dump_via_C99(lok))
    print()

    print("=== Test 5: x99 (out-of-range → clamped 8) ===")
    print("CULFW32 x99:", repr(cmd(cul, "x99")))
    print("Lokales x99:", repr(cmd(lok, "x99")))
    print()
    print("CULFW32 C99 (sollte gleich x08 sein):")
    print(patable_dump_via_C99(cul))
    print()
    print("Lokales C99:")
    print(patable_dump_via_C99(lok))

    os.close(cul); os.close(lok)

if __name__ == "__main__":
    main()
