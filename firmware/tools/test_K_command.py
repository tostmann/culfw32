#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""K-Befehl (KS300-TX) Roundtrip CULFW32 → Lokales-CUL868."""
import os, sys, time, tty, threading
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
    """Read everything that comes in for dur seconds."""
    end = time.time() + dur
    out = b""
    os.set_blocking(fd, False)
    while time.time() < end:
        try:
            chunk = os.read(fd, 4096)
            if chunk:
                out += chunk
            else:
                time.sleep(0.05)
        except BlockingIOError:
            time.sleep(0.05)
    return out.decode("ascii", errors="replace").replace("\r", "").strip()

cul = open_raw(CULFW32)
lok = open_raw(LOKAL)

print("=== Setup ===")
print("CULFW32 V :", cmd(cul, "V"))
print("Lokales V :", cmd(lok, "V"))

# RX-Mode: Xff aktiviert alle Reports auf beiden Seiten (REP_KNOWN+REP_REPEATED+
# REP_BITS+REP_MONITOR+REP_BINTIME+REP_RSSI+REP_FHTPROTO+REP_LCDMON).
# Damit liefert RX auch unbekannte Frames als raw bytes (REP_MONITOR).
print("CULFW32 X21:", cmd(cul, "X21"))
print("Lokales X21:", cmd(lok, "X21"))
time.sleep(0.5)

# Drain initial noise
drain(cul, 0.5)
drain(lok, 0.5)

# Test-Frames byte-genau a-culfw KS-Send-Format. Typische KS300-Frames sind
# 7-9 Bytes Sensor-Payload; cksum3 wird automatisch angehaengt.
TESTS = [
    ("K12345678",      "8-byte test"),
    ("KAABBCCDDEE",    "5-byte test"),
    ("K11223344556677",  "7-byte typischer KS300-Frame"),
]

for hexcmd, desc in TESTS:
    print(f"\n=== Test: {hexcmd}  ({desc}) ===")
    drain(cul, 0.2); drain(lok, 0.2)
    cmd(cul, hexcmd, wait=0.0)
    # KS-Frame braucht lange wegen 854µs Pulse + 3 repeats + 10s pause
    # zwischen repeats. Aber a-culfw `pause`-Param ist 10*10ms = 100ms (bug?)
    # — sendraw nutzt my_delay_ms(pause) mit pause=FS20_PAUSE=10. Also 10ms.
    # 1 Frame ca. (10 sync + 10*nbyte) bits * ~1.2ms = ~120ms. * 3 repeats ~400ms.
    rx_local = drain(lok, 1.5)
    rx_cul   = drain(cul, 0.2)
    print(f"  CULFW32 self-RX (Echo): {rx_cul if rx_cul else '<keine>'}")
    print(f"  Lokales RX            : {rx_local if rx_local else '<keine>'}")

# Cleanup: TX_REPORT auf 0
cmd(cul, "X00", wait=0.1)
cmd(lok, "X00", wait=0.1)

os.close(cul); os.close(lok)
