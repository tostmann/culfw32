#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""JSON-Frontend Test gegen CULFW32 auf TCP Port 2324."""
import json, os, socket, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _lab_env import CULFW32_HOST as IP  # noqa: E402

PORT = 2324

def open_conn():
    s = socket.create_connection((IP, PORT), timeout=3.0)
    s.settimeout(2.0)
    return s

def recv_lines(s, timeout=1.5, max_lines=10):
    end = time.time() + timeout
    buf = b""
    out = []
    s.settimeout(0.5)
    while time.time() < end and len(out) < max_lines:
        try:
            chunk = s.recv(4096)
            if not chunk: break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if line: out.append(line.decode("utf-8", errors="replace"))
        except socket.timeout:
            if out: break
    return out

def send(s, obj):
    s.sendall((json.dumps(obj) + "\n").encode())

def cmd(s, obj, wait=0.4):
    send(s, obj); time.sleep(wait)
    return recv_lines(s, timeout=1.0)

print("=== Connect ===")
s = open_conn()
hello = recv_lines(s, timeout=1.5)
print("Hello-Frame :", hello)
print()

print("=== ping ===")
print(cmd(s, {"cmd": "ping"}))
print()

print("=== get wifi.state ===")
print(cmd(s, {"cmd": "get", "key": "wifi.state"}))
print()

print("=== get wifi.ssid ===")
print(cmd(s, {"cmd": "get", "key": "wifi.ssid"}))
print()

print("=== get wifi.psk (sensitive — sollte *** sein) ===")
print(cmd(s, {"cmd": "get", "key": "wifi.psk"}))
print()

print("=== set led.mode=3 ===")
print(cmd(s, {"cmd": "set", "key": "led.mode", "value": "3"}))
print()

print("=== set led.mode=1 (zurueck) ===")
print(cmd(s, {"cmd": "set", "key": "led.mode", "value": "1"}))
print()

print("=== set unknown.key (sollte ok=false) ===")
print(cmd(s, {"cmd": "set", "key": "no.such.key", "value": "x"}))
print()

print("=== dump (alle Eintraege) ===")
res = cmd(s, {"cmd": "dump"}, wait=0.6)
for line in res:
    parsed = json.loads(line)
    if parsed.get("event") == "dump":
        print(f"dump.items count = {len(parsed.get('items', []))}")
        for it in parsed["items"][:5]:   # nur die ersten 5 zur Sichtprobe
            print("  ", it)
        print("  ... (truncated)")
    else:
        print(line)
print()

print("=== invalid json ===")
s.sendall(b"this-is-not-json\n"); time.sleep(0.5)
print(recv_lines(s, timeout=1.0))
print()

print("=== unknown cmd ===")
print(cmd(s, {"cmd": "fly-to-mars"}))

s.close()
