#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""EWA-Test: IP-Allowlist live + Reboot-Persistenz."""
import os, sys, time, tty, socket
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _lab_env import (  # noqa: E402
    CULFW32_DEV as CULFW32,
    CULFW32_HOST as IP,
    LAB_HOST,
    LAB_SUBNET,
)

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

def tcp_probe(ip, port, send=b"V\n"):
    try:
        s = socket.create_connection((ip, port), timeout=2.0)
        s.sendall(send); s.settimeout(1.5)
        data = b""
        try:
            while True:
                chunk = s.recv(256)
                if not chunk: break
                data += chunk
                if len(data) > 80: break
        except socket.timeout: pass
        s.close()
        return data.decode("ascii", errors="replace").strip()
    except Exception as e:
        return f"ERR: {type(e).__name__}: {e}"

cul = open_raw(CULFW32)

print("=== Step 1: Default (allow=any) ===")
print(cmd(cul, "EW"))
print()

print("=== Step 2: Probe (sollte funktionieren) ===")
print("nc:", repr(tcp_probe(IP, 2323)))
print()

print("=== Step 3: EWA 99.99.99.99/32 (lockt uns aus) ===")
print(cmd(cul, "EWA 99.99.99.99/32"))
time.sleep(0.5)
print(cmd(cul, "EW"))
print()

print("=== Step 4: Probe (sollte refused/timeout sein) ===")
print("nc:", repr(tcp_probe(IP, 2323)))
print()

print("=== Step 5: Reboot via B, EE-Persistenz ===")
cmd(cul, "B", wait=0.0)
os.close(cul); time.sleep(4)
cul = open_raw(CULFW32)
time.sleep(2)
print(cmd(cul, "EW"))
print("nc:", repr(tcp_probe(IP, 2323)))
print()

print("=== Step 6: EWA any (Allowlist aus) ===")
print(cmd(cul, "EWA any"))
time.sleep(0.5)
print(cmd(cul, "EW"))
print("nc:", repr(tcp_probe(IP, 2323)))
print()

# LAB_HOST = IP des Hosts, der dieses Skript ausfuehrt — single-host-Allow
# erlaubt nur diesen Host. CULFW32 muss erreichbar bleiben.
print(f"=== Step 7: EWA {LAB_HOST}/32 (allow nur Test-Host) ===")
print(cmd(cul, f"EWA {LAB_HOST}/32"))
time.sleep(0.5)
print(cmd(cul, "EW"))
print(f"nc von {LAB_HOST}:", repr(tcp_probe(IP, 2323)))   # sollte funktionieren wenn Test auf LAB_HOST laeuft
print()

print(f"=== Step 8: EWA {LAB_SUBNET} (subnet allow) ===")
print(cmd(cul, f"EWA {LAB_SUBNET}"))
time.sleep(0.5)
print(cmd(cul, "EW"))
print("nc:", repr(tcp_probe(IP, 2323)))
print()

print("=== Cleanup: zurück auf any ===")
cmd(cul, "EWA any")
print(cmd(cul, "EW"))
os.close(cul)
