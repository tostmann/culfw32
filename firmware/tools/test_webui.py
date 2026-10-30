#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""Web-UI WebSocket-Smoke-Test."""
import json, os, sys, time, websocket
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _lab_env import CULFW32_HOST  # noqa: E402

URL_JSON = f"ws://{CULFW32_HOST}/ws/json"
URL_TTY  = f"ws://{CULFW32_HOST}/ws/tty"

def recv_until(ws, timeout=1.0, max_msgs=10):
    out = []
    end = time.time() + timeout
    ws.settimeout(0.4)
    while time.time() < end and len(out) < max_msgs:
        try:
            msg = ws.recv()
            if msg: out.append(msg)
        except (websocket.WebSocketTimeoutException, TimeoutError):
            if out: break
    return out

print("=== /ws/json ===")
ws = websocket.create_connection(URL_JSON, timeout=3)
print("Hello:", recv_until(ws, 0.5))
ws.send(json.dumps({"cmd": "ping"}))
print("Ping :", recv_until(ws, 0.5))
ws.send(json.dumps({"cmd": "get", "key": "wifi.state"}))
print("Get  :", recv_until(ws, 0.5))
ws.send(json.dumps({"cmd": "set", "key": "led.mode", "value": "3"}))
print("Set  :", recv_until(ws, 0.5))
ws.send(json.dumps({"cmd": "set", "key": "led.mode", "value": "1"}))
print("Set  :", recv_until(ws, 0.5))
ws.send(json.dumps({"cmd": "dump"}))
dump = recv_until(ws, 1.0)
if dump:
    parsed = json.loads(dump[0])
    print(f"Dump : items={len(parsed.get('items', []))}")
ws.close()
print()

print("=== /ws/tty ===")
ws = websocket.create_connection(URL_TTY, timeout=3)
print("Initial (V):", recv_until(ws, 1.0))
ws.send("V\r\n")
print("V       :", recv_until(ws, 0.5))
ws.send("?wifi.state\r\n")
print("?wifi.. :", recv_until(ws, 0.5))
ws.send("t\r\n")
print("t       :", recv_until(ws, 0.5))
ws.close()
