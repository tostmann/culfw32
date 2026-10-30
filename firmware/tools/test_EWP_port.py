#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""EWP-Test: Port live aendern + Reboot-Persistenz."""
import os, sys, time, tty, socket
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _lab_env import CULFW32_DEV as CULFW32, CULFW32_HOST as IP  # noqa: E402

def open_raw(p):
    fd = os.open(p, os.O_RDWR | os.O_NOCTTY); tty.setraw(fd); return fd
def cmd(fd, line, wait=0.5):
    os.write(fd, (line+"\r\n").encode()); time.sleep(wait)
    out=b""
    while True:
        try:
            os.set_blocking(fd, False)
            chunk = os.read(fd, 4096)
            if not chunk: break
            out += chunk
        except BlockingIOError: break
    return out.decode("ascii", errors="replace").replace("\r","").strip()

def tcp_probe(ip, port, send_cmd="V\n"):
    """Versuch eine TCP-Verbindung; sende send_cmd, lies bis 1s timeout."""
    try:
        s = socket.create_connection((ip, port), timeout=2.0)
        s.sendall(send_cmd.encode())
        s.settimeout(1.0)
        data = b""
        try:
            while True:
                chunk = s.recv(256)
                if not chunk: break
                data += chunk
                if len(data) > 80: break
        except socket.timeout:
            pass
        s.close()
        return data.decode("ascii", errors="replace").strip()
    except Exception as e:
        return f"ERR: {e}"

cul = open_raw(CULFW32)

print("=== Step 1: Default-Status ===")
print("EW:")
print(cmd(cul, "EW"))
print()

print("=== Step 2: Probe TCP auf 2323 (Default) ===")
print(f"nc {IP}:2323 → V →", repr(tcp_probe(IP, 2323, "V\n")))
print()

print("=== Step 3: Port auf 8080 setzen ===")
print("EWP 8080:", repr(cmd(cul, "EWP 8080")))
time.sleep(1.0)
print("EW:")
print(cmd(cul, "EW"))
print()

print("=== Step 4: Probe alter (sollte fehlschlagen) + neuer Port ===")
print(f"nc {IP}:2323 →", repr(tcp_probe(IP, 2323, "V\n")))
print(f"nc {IP}:8080 →", repr(tcp_probe(IP, 8080, "V\n")))
print()

print("=== Step 5: Reboot via B, PORT-Persistenz ===")
cmd(cul, "B", wait=0.0)
os.close(cul); time.sleep(4)
cul = open_raw(CULFW32)
print("EW nach Reboot:")
print(cmd(cul, "EW"))
print()
print("=== Step 6: TCP auf 8080 (sollte funktionieren) ===")
time.sleep(2)
print(f"nc {IP}:8080 →", repr(tcp_probe(IP, 8080, "V\n")))
print()

print("=== Step 7: Zurueck auf 2323 ===")
print("EWP 2323:", repr(cmd(cul, "EWP 2323")))
time.sleep(1.0)
print(f"nc {IP}:2323 →", repr(tcp_probe(IP, 2323, "V\n")))

os.close(cul)
