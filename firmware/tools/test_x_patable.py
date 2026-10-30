#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""Verifiziere EE_CC1100_PA (8 Bytes ab 0x2B) byte-genau gegen Lokales-CUL868."""
import os, sys, time, tty
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _lab_env import CULFW32_DEV as CULFW32, CUL868_DEV as LOKAL  # noqa: E402

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
    """Read 8 EE bytes from EE_CC1100_PA = 0x2B..0x32."""
    bytes_ = []
    for addr in range(0x2B, 0x33):
        # Format: 'R002B = vv / ddd'
        out = cmd(fd, f"R{addr:04X}", wait=0.1)
        # Find vv
        try:
            v = out.split("=")[1].strip().split("/")[0].strip()
            bytes_.append(int(v, 16))
        except (IndexError, ValueError):
            bytes_.append(None)
    return bytes_

# Erwartete PA-Profile byte-genau a-culfw cc1100.c:28 FULL_CC1100_PA
PROFILES = {
    0x00: "00 03 0F 1E 26 27 00 00",  # -10 dBm with PA ramping
    0x01: "00 03 0F 1E 25 36 38 67",  # -5
    0x02: "00 03 0F 25 67 40 60 50",  #  0 dBm with ramp
    0x03: "00 03 0F 27 51 88 83 81",  # +5
    0x04: "00 03 0F 27 50 C8 C3 C2",  # +10
    0x05: "00 27 00 00 00 00 00 00",  # -10 no ramp
    0x06: "00 67 00 00 00 00 00 00",  # -5
    0x07: "00 50 00 00 00 00 00 00",  #  0
    0x08: "00 81 00 00 00 00 00 00",  # +5 (factory)
    0x09: "00 C2 00 00 00 00 00 00",  # +10
}

def main():
    cul = open_raw(CULFW32)
    lok = open_raw(LOKAL)

    # Test alle relevanten Profile
    for idx in [0x00, 0x02, 0x04, 0x06, 0x08, 0x09]:
        cul_resp = cmd(cul, f"x{idx:02X}", wait=0.2)
        lok_resp = cmd(lok, f"x{idx:02X}", wait=0.2)
        cul_pa   = read_pa(cul)
        lok_pa   = read_pa(lok)
        cul_str  = " ".join(f"{b:02X}" if b is not None else "??" for b in cul_pa)
        lok_str  = " ".join(f"{b:02X}" if b is not None else "??" for b in lok_pa)
        exp      = PROFILES[idx]
        cul_ok   = "OK" if cul_str == exp else "DIFF"
        lok_ok   = "OK" if lok_str == exp else "DIFF"
        match    = "MATCH" if cul_str == lok_str else "DIFF"
        print(f"x{idx:02X} (idx={idx}):")
        print(f"  CULFW32: {cul_str}  [{cul_ok}]")
        print(f"  Lokales: {lok_str}  [{lok_ok}]")
        print(f"  Spec   : {exp}")
        print(f"  CUL/Lokales: {match}")
        print()

    os.close(cul); os.close(lok)

if __name__ == "__main__":
    main()
