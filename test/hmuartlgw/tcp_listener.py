#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""HMUARTLGW-TCP-Listener gegen CULFW32-Hardware (Port 2325).

Twin von uart_listener.py. Schickt die FHEM-Init-Sequenz und logged was
zurueckkommt — so koennen wir die CULFW32-HMUARTLGW-Frontend-Antworten
byte-genau gegen die echte HM-MOD-RPI-PCB-Reference vergleichen.
"""
from __future__ import annotations

import argparse
import os
import select
import socket
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame_parser import (
    Frame, FrameDecoder,
    DST_OS, DST_APP, DST_DUAL,
)


def hexdump(b: bytes) -> str:
    return " ".join(f"{x:02x}" for x in b)


def ts() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


class TcpLink:
    def __init__(self, host: str, port: int, log_fp):
        self.sock = socket.create_connection((host, port), timeout=5.0)
        self.sock.setblocking(False)
        self.cnt = 0
        self.dec = FrameDecoder()
        self.log_fp = log_fp

    def close(self):
        try: self.sock.close()
        except OSError: pass

    def _next_cnt(self) -> int:
        self.cnt = (self.cnt + 1) & 0xFF
        if self.cnt == 0:
            self.cnt = 1
        return self.cnt

    def send(self, dst: int, payload: bytes, label: str = "") -> int:
        cnt = self._next_cnt()
        f = Frame(dst=dst, cnt=cnt, payload=payload)
        raw = f.encode()
        msg = f"[TX] {ts()} dst={dst:02x} cnt={cnt:02x} body={payload.hex()}"
        if label:
            msg += f"  ({label})"
        msg += f"\n      raw: {hexdump(raw)}"
        print(msg, flush=True)
        if self.log_fp: self.log_fp.write(msg + "\n"); self.log_fp.flush()
        self.sock.sendall(raw)
        return cnt

    def wait_for(self, predicate, timeout: float = 2.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            r, _, _ = select.select([self.sock], [], [], min(0.2, deadline - time.time()))
            if not r: continue
            try:
                chunk = self.sock.recv(4096)
            except (BlockingIOError, ConnectionResetError):
                continue
            if not chunk: continue
            self.dec.feed(chunk)
            for fr in self.dec.frames():
                self._log_rx(fr)
                if predicate(fr):
                    return fr
        return None

    def listen_forever(self, duration: float):
        end = time.time() + duration
        print(f"--- {ts()} listener active for {duration:.0f}s ---", flush=True)
        while time.time() < end:
            r, _, _ = select.select([self.sock], [], [], 1.0)
            if not r: continue
            try:
                chunk = self.sock.recv(4096)
            except (BlockingIOError, ConnectionResetError):
                continue
            if not chunk: continue
            self.dec.feed(chunk)
            for fr in self.dec.frames():
                self._log_rx(fr)

    def _log_rx(self, fr: Frame):
        cmd = fr.cmd_byte()
        body = fr.payload[1:].hex() if len(fr.payload) > 1 else ""
        first = f"[RX] {ts()} dst={fr.dst:02x} cnt={fr.cnt:02x} cmd={cmd:02x}({fr.cmd_name()}) body={body}"
        decoded_extra = ""
        if fr.dst == DST_APP and cmd == 0x05 and len(fr.payload) >= 5:
            status = fr.payload[1]; info = fr.payload[2]; rssi = fr.payload[3]
            askin = fr.payload[4:]
            decoded_extra = (
                f"\n      APP_RECV: status={status:02x} info={info:02x} "
                f"rssi=0x{rssi:02x} askin_len={len(askin)} askin={askin.hex()}"
            )
        msg = first + decoded_extra
        print(msg, flush=True)
        if self.log_fp: self.log_fp.write(msg + "\n"); self.log_fp.flush()


def init_sequence(link: TcpLink, hmid: bytes):
    link.send(DST_APP, b"\x01", label="APP_GET_HMID (probe)")
    link.wait_for(lambda f: f.dst == DST_APP and f.cmd_byte() == 0x04, timeout=1.5)
    link.send(DST_APP, b"\x00" + hmid, label="APP_SET_HMID")
    link.wait_for(lambda f: f.dst == DST_APP and f.cmd_byte() == 0x04, timeout=1.5)
    link.send(DST_OS, b"\x02", label="OS_GET_FIRMWARE")
    link.wait_for(lambda f: f.dst == DST_OS and f.cmd_byte() == 0x04, timeout=1.5)
    link.send(DST_OS, b"\x0b", label="OS_GET_SERIAL")
    link.wait_for(lambda f: f.dst == DST_OS and f.cmd_byte() == 0x04, timeout=1.5)
    link.send(DST_OS, b"\x06", label="OS_NORMAL_MODE")
    link.wait_for(lambda f: f.dst == DST_OS and f.cmd_byte() == 0x04, timeout=1.5)
    link.send(DST_OS, b"\x0a\x00", label="OS_ENABLE_CSMACA off")
    link.wait_for(lambda f: f.dst == DST_OS and f.cmd_byte() == 0x04, timeout=1.5)
    link.send(DST_OS, b"\x09\x01", label="OS_ENABLE_CREDITS on")
    link.wait_for(lambda f: f.dst == DST_OS and f.cmd_byte() == 0x04, timeout=1.5)
    link.send(DST_OS, b"\x08", label="OS_GET_CREDITS")
    link.wait_for(lambda f: f.dst == DST_OS and f.cmd_byte() == 0x04, timeout=1.5)
    z17 = b"\x00" * 17
    for cmd, name in [(b"\x03","APP_SET_CURRENT_KEY"),(b"\x0f","APP_SET_PREVIOUS_KEY"),(b"\x0b","APP_SET_TEMP_KEY")]:
        link.send(DST_APP, cmd + z17, label=name)
        link.wait_for(lambda f: f.dst == DST_APP and f.cmd_byte() == 0x04, timeout=1.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("host", nargs="?", default=os.environ.get("CULFW32_HOST", "192.168.1.42"))
    ap.add_argument("--port", type=int, default=2325)
    ap.add_argument("--hmid", default="F11234")
    ap.add_argument("--log", default=None)
    ap.add_argument("--listen", type=float, default=20.0, help="seconds to listen after init")
    args = ap.parse_args()

    hmid = bytes.fromhex(args.hmid)
    log_fp = open(args.log, "w") if args.log else None
    link = TcpLink(args.host, args.port, log_fp)
    try:
        print(f"=== {ts()} init start (host={args.host}:{args.port} hmid={args.hmid}) ===", flush=True)
        init_sequence(link, hmid)
        print(f"=== {ts()} init done — listening for {args.listen}s ===", flush=True)
        link.listen_forever(args.listen)
    except KeyboardInterrupt:
        print(f"\n=== {ts()} stopped by user ===")
    finally:
        link.close()
        if log_fp: log_fp.close()


if __name__ == "__main__":
    main()
