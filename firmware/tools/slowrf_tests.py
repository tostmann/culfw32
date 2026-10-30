#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""SlowRF FS20 Hardware-Tests gegen Lokales und Remote CUL868.

Setup:
- CULFW32 ESP32-C6 Board am USB-CDC (default-Pfad in DEV_TARGET).
- Lokales busware CUL868 am USB-Serial (DEV_LOCAL).
- Remote CUL868 via TCP (REMOTE_HOST:REMOTE_PORT) mit identischem a-culfw-Build.

Aufruf:
    python3 slowrf_tests.py smoke       # Boot-Smoke (V/Vr/VR/X/C99)
    python3 slowrf_tests.py local-tx    # CULFW32 sendet → Lokales empfängt
    python3 slowrf_tests.py local-rx    # Lokales sendet → CULFW32 empfängt
    python3 slowrf_tests.py remote-tx   # CULFW32 sendet → Remote empfängt
    python3 slowrf_tests.py remote-rx   # Remote sendet → CULFW32 empfängt
    python3 slowrf_tests.py pong        # Ping F12345600, erwarte Pong F12345611
    python3 slowrf_tests.py all         # local-tx + local-rx + remote-tx + remote-rx

Test-Pattern: 4 Daten-Bytes (oberhalb der a-culfw FS20-Min-Length-Heuristik
von 4 Bytes; 3 Bytes wuerden vom Decoder als FS20 abgelehnt).

ACHTUNG:
NIEMALS direkt mit `>` in /dev/serial/by-id/... schreiben — bei disconnect wird
sonst eine Junk-Datei angelegt. Wir nutzen os.open mit O_RDWR (kein O_CREAT),
das errort sauber wenn der char-dev fehlt.
"""

import os
import socket
import stat
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _lab_env import (  # noqa: E402
    CULFW32_DEV as DEV_TARGET,
    CUL868_DEV as DEV_LOCAL,
    CUL868_REMOTE_HOST as REMOTE_HOST,
    CUL868_REMOTE_PORT as REMOTE_PORT,
)

# X21 = REP_KNOWN | REP_RSSI — RSSI-Suffix in F-Output.
X_MASK = b"X21\r\n"


def open_serial(dev: str) -> int:
    real = os.path.realpath(dev)
    if not os.path.exists(real):
        sys.exit(f"ERROR: {dev} -> {real} does not exist")
    if not stat.S_ISCHR(os.stat(real).st_mode):
        sys.exit(f"ERROR: {real} is not a char device — refusing")
    return os.open(dev, os.O_RDWR | os.O_NONBLOCK)


def open_remote() -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2.0)
    s.connect((REMOTE_HOST, REMOTE_PORT))
    s.setblocking(False)
    return s


def drain_fd(fd: int, dur: float = 0.2):
    end = time.time() + dur
    while time.time() < end:
        try: os.read(fd, 4096)
        except (BlockingIOError, OSError): time.sleep(0.02)


def drain_sock(s: socket.socket, dur: float = 0.2):
    end = time.time() + dur
    while time.time() < end:
        try: s.recv(4096)
        except (BlockingIOError, OSError): time.sleep(0.02)


def collect_fd(fd: int, dur: float, label: str = "", filter_fn=None) -> list:
    """Sammle Output bis dur Sekunden vorbei. Filter optional."""
    out = []
    end = time.time() + dur
    while time.time() < end:
        try:
            chunk = os.read(fd, 4096)
            if chunk:
                for ln in chunk.decode("ascii", errors="replace").splitlines():
                    ln = ln.strip()
                    if not ln: continue
                    if filter_fn and not filter_fn(ln): continue
                    out.append(ln)
                    print(f"{label}: {ln}", flush=True)
        except (BlockingIOError, OSError): pass
        time.sleep(0.03)
    return out


def collect_sock(s: socket.socket, dur: float, label: str = "", filter_fn=None) -> list:
    out = []
    end = time.time() + dur
    while time.time() < end:
        try:
            chunk = s.recv(4096)
            if chunk:
                for ln in chunk.decode("ascii", errors="replace").splitlines():
                    ln = ln.strip()
                    if not ln: continue
                    if filter_fn and not filter_fn(ln): continue
                    out.append(ln)
                    print(f"{label}: {ln}", flush=True)
        except (BlockingIOError, OSError): pass
        time.sleep(0.03)
    return out


# ---------------------------------------------------------------------------

def smoke():
    """Boot-Smoke: Version, Vr, VR, X, C99."""
    fd = open_serial(DEV_TARGET)
    drain_fd(fd)
    for cmd in [b"V\r\n", b"X\r\n", b"X01\r\n", b"X\r\n", b"Vr\r\n", b"VR\r\n", b"C99\r\n"]:
        os.write(fd, cmd)
        time.sleep(0.3)
        collect_fd(fd, 0.3, label=f"<< {cmd.strip().decode()}")
    os.close(fd)


def _flood_test(fd_tx, send_cmd_fn, fd_rx, rx_label: str,
                rx_collect_fn, dur: float = 5.0, period: float = 0.6,
                expect_prefix: str = "F"):
    """Generischer Flood-Test: tx-Seite sendet, rx-Seite collected."""
    drain_fd(fd_tx) if isinstance(fd_tx, int) else drain_sock(fd_tx)
    if isinstance(fd_rx, int):
        drain_fd(fd_rx)
    else:
        drain_sock(fd_rx)

    stop = [False]
    def sender():
        i = 0
        while not stop[0]:
            send_cmd_fn(i)
            i += 1
            time.sleep(period)
    t = threading.Thread(target=sender)
    t.start()

    def filt(ln): return ln.startswith(expect_prefix)
    out = rx_collect_fn(fd_rx, dur, label=rx_label, filter_fn=filt)
    stop[0] = True; t.join()
    return out


def local_tx():
    """CULFW32 sendet F0A0B0CXX → Lokales empfaengt."""
    fd_tgt = open_serial(DEV_TARGET)
    fd_loc = open_serial(DEV_LOCAL)
    drain_fd(fd_tgt); drain_fd(fd_loc)
    os.write(fd_tgt, X_MASK); os.write(fd_loc, X_MASK)
    time.sleep(0.5)
    drain_fd(fd_tgt); drain_fd(fd_loc)
    print("=== local-tx: CULFW32 → Lokales ===", flush=True)
    _flood_test(
        fd_tgt,
        lambda i: os.write(fd_tgt, b"F0A0B0C%02X\r\n" % (i & 0xFF)),
        fd_loc, "lcl", collect_fd, dur=5.0, period=0.6, expect_prefix="F0A0B0C")
    os.close(fd_tgt); os.close(fd_loc)


def local_rx():
    """Lokales sendet F0102030XX → CULFW32 empfaengt."""
    fd_tgt = open_serial(DEV_TARGET)
    fd_loc = open_serial(DEV_LOCAL)
    drain_fd(fd_tgt); drain_fd(fd_loc)
    os.write(fd_tgt, X_MASK); os.write(fd_loc, X_MASK)
    time.sleep(0.5)
    drain_fd(fd_tgt); drain_fd(fd_loc)
    print("=== local-rx: Lokales → CULFW32 ===", flush=True)
    _flood_test(
        fd_loc,
        lambda i: os.write(fd_loc, b"F010203%02X\r\n" % (i & 0xFF)),
        fd_tgt, "tgt", collect_fd, dur=5.0, period=0.6, expect_prefix="F010203")
    os.close(fd_tgt); os.close(fd_loc)


def remote_tx():
    """CULFW32 sendet F0A0B0CXX → Remote empfaengt."""
    fd_tgt = open_serial(DEV_TARGET)
    s_rem = open_remote()
    drain_fd(fd_tgt); drain_sock(s_rem)
    os.write(fd_tgt, X_MASK); s_rem.send(X_MASK)
    time.sleep(0.5)
    drain_fd(fd_tgt); drain_sock(s_rem)
    print(f"=== remote-tx: CULFW32 → Remote ({REMOTE_HOST}:{REMOTE_PORT}) ===", flush=True)
    stop = [False]
    def sender():
        i = 0
        while not stop[0]:
            os.write(fd_tgt, b"F0A0B0C%02X\r\n" % (i & 0xFF))
            i += 1
            time.sleep(0.6)
    t = threading.Thread(target=sender); t.start()
    collect_sock(s_rem, 6.0, "rem", filter_fn=lambda ln: ln.startswith("F0A0B0C"))
    stop[0] = True; t.join()
    s_rem.close(); os.close(fd_tgt)


def remote_rx():
    """Remote sendet F0102030XX → CULFW32 empfaengt."""
    fd_tgt = open_serial(DEV_TARGET)
    s_rem = open_remote()
    drain_fd(fd_tgt); drain_sock(s_rem)
    os.write(fd_tgt, X_MASK); s_rem.send(X_MASK)
    time.sleep(0.5)
    drain_fd(fd_tgt); drain_sock(s_rem)
    print(f"=== remote-rx: Remote ({REMOTE_HOST}:{REMOTE_PORT}) → CULFW32 ===", flush=True)
    stop = [False]
    def sender():
        i = 0
        while not stop[0]:
            s_rem.send(b"F010203%02X\r\n" % (i & 0xFF))
            i += 1
            time.sleep(0.6)
    t = threading.Thread(target=sender); t.start()
    collect_fd(fd_tgt, 6.0, "tgt", filter_fn=lambda ln: ln.startswith("F010203"))
    stop[0] = True; t.join()
    s_rem.close(); os.close(fd_tgt)


def pong():
    """Ping/Pong-Test gegen Lab-Pong-Sender. Sende F12345600, erwarte F12345611."""
    fd_tgt = open_serial(DEV_TARGET)
    drain_fd(fd_tgt)
    os.write(fd_tgt, X_MASK)
    time.sleep(0.5)
    drain_fd(fd_tgt)
    print("=== pong: F12345600 → erwarte F12345611 ===", flush=True)
    for i in range(8):
        print(f"-- ping {i} --", flush=True)
        os.write(fd_tgt, b"F12345600\r\n")
        out = collect_fd(fd_tgt, 1.5, label=f"rx{i}",
                         filter_fn=lambda ln: ln.startswith("F12345611"))
        if out:
            print(f"   PONG empfangen ({len(out)} repeats)", flush=True)
        time.sleep(0.3)
    os.close(fd_tgt)


# ---------------------------------------------------------------------------

COMMANDS = {
    "smoke":     smoke,
    "local-tx":  local_tx,
    "local-rx":  local_rx,
    "remote-tx": remote_tx,
    "remote-rx": remote_rx,
    "pong":      pong,
}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in COMMANDS and sys.argv[1] != "all":
        print(f"usage: {sys.argv[0]} <{ '|'.join(list(COMMANDS) + ['all']) }>")
        sys.exit(2)

    if sys.argv[1] == "all":
        for k in ("local-tx", "local-rx", "remote-tx", "remote-rx"):
            print(f"\n>>> {k}\n", flush=True)
            COMMANDS[k]()
            time.sleep(0.5)
    else:
        COMMANDS[sys.argv[1]]()


if __name__ == "__main__":
    main()
