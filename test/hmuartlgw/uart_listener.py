#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""HMUARTLGW UART-Listener fuer echte HM-MOD-RPI-PCB / RPI-RF-MOD am /dev/raw-uart.

Zweck: byte-genaue Referenz der echten Hardware.

Schickt die FHEM-Init-Sequenz an die echte Hardware und logged danach alle
empfangenen Frames. So sehen wir byte-genau wie die echte Hardware
- die OS-/APP-Layer-Replies aufbaut (vergleich mit unserem CULFW32-Frontend)
- AskSin-RX-Frames an den Host signalisiert (cmd-/dst-/payload-Layout)
- TX-/Pair-Roundtrips abwickelt

Aufruf:
    sudo python3 uart_listener.py /dev/raw-uart [--hmid F11234] [--log out.log]

Beendet mit Ctrl-C.
"""
from __future__ import annotations

import argparse
import os
import select
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame_parser import (
    Frame, FrameDecoder,
    DST_OS, DST_APP, DST_DUAL,
    OS_CMDS, APP_CMDS, ACK_CODES,
)


def hexdump(b: bytes) -> str:
    return " ".join(f"{x:02x}" for x in b)


def ts() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


class UartLink:
    """Wraps /dev/raw-uart as binary file. Counter-Management included."""

    def __init__(self, path: str, log_fp):
        # O_RDWR ohne O_CREAT — sicheres Verhalten gegen device-disconnect
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        self.cnt = 0
        self.dec = FrameDecoder()
        self.log_fp = log_fp

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass

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
        os.write(self.fd, raw)
        return cnt

    def wait_for(self, predicate, timeout: float = 2.0):
        """Read until predicate(frame) returns True or timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            r, _, _ = select.select([self.fd], [], [], min(0.2, deadline - time.time()))
            if not r:
                continue
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                continue
            if not chunk:
                continue
            self.dec.feed(chunk)
            for fr in self.dec.frames():
                self._log_rx(fr)
                if predicate(fr):
                    return fr
        return None

    def listen_forever(self):
        print(f"--- {ts()} listener active. Ctrl-C to stop. ---", flush=True)
        while True:
            r, _, _ = select.select([self.fd], [], [], 1.0)
            if not r:
                continue
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                continue
            if not chunk:
                continue
            self.dec.feed(chunk)
            for fr in self.dec.frames():
                self._log_rx(fr)

    def _log_rx(self, fr: Frame):
        cmd = fr.cmd_byte()
        cmd_name = fr.cmd_name()
        body = fr.payload[1:].hex() if len(fr.payload) > 1 else ""
        first = f"[RX] {ts()} dst={fr.dst:02x} cnt={fr.cnt:02x} cmd={cmd:02x}({cmd_name}) body={body}"
        # APP_RECV-Frame zerlegen falls erkennbar
        decoded_extra = ""
        if fr.dst == DST_APP and cmd == 0x05 and len(fr.payload) >= 5:
            # Erwartet: [05, status, info, rssi, askin_body...]
            status = fr.payload[1]
            info = fr.payload[2]
            rssi = fr.payload[3]
            askin = fr.payload[4:]
            decoded_extra = (
                f"\n      APP_RECV: status={status:02x} info={info:02x} "
                f"rssi=0x{rssi:02x} askin_len={len(askin)} askin={askin.hex()}"
            )
        # vielleicht ist dst etwas anderes (DualCoPro?)
        elif fr.dst not in (DST_OS, DST_APP, DST_DUAL):
            decoded_extra = f"\n      *** UNUSUAL dst={fr.dst:02x} ***"
        msg = first + decoded_extra
        print(msg, flush=True)
        if self.log_fp: self.log_fp.write(msg + "\n"); self.log_fp.flush()


def init_sequence(link: UartLink, hmid: bytes):
    """Vollstaendige FHEM-aequivalente Init. Setzt HMID, aktiviert NORMAL_MODE, Keys all-zero."""

    # Schritt 1: APP_GET_HMID — pollen ob App schon laeuft.
    link.send(DST_APP, b"\x01", label="APP_GET_HMID (probe)")
    fr = link.wait_for(lambda f: f.dst == DST_APP and f.cmd_byte() == 0x04)
    if fr is None:
        print("(probe ohne reply, evtl. ist Modul im BL — versuche change_app)")
        link.send(DST_OS, b"\x03", label="OS_CHANGE_APP")
        link.wait_for(lambda f: f.dst == DST_OS and f.payload == b"\x00" + b"Co_CPU_App", timeout=3.0)

    # Schritt 2: SET_HMID
    link.send(DST_APP, b"\x00" + hmid, label="APP_SET_HMID")
    link.wait_for(lambda f: f.dst == DST_APP and f.cmd_byte() == 0x04, timeout=2.0)

    # Schritt 3: GET_FIRMWARE
    link.send(DST_OS, b"\x02", label="OS_GET_FIRMWARE")
    link.wait_for(lambda f: f.dst == DST_OS and f.cmd_byte() == 0x04, timeout=2.0)

    # Schritt 4: GET_SERIAL
    link.send(DST_OS, b"\x0b", label="OS_GET_SERIAL")
    link.wait_for(lambda f: f.dst == DST_OS and f.cmd_byte() == 0x04, timeout=2.0)

    # Schritt 5: NORMAL_MODE — AKTIVIERT RX
    link.send(DST_OS, b"\x06", label="OS_NORMAL_MODE")
    link.wait_for(lambda f: f.dst == DST_OS and f.cmd_byte() == 0x04, timeout=2.0)

    # Schritt 6: ENABLE_CSMACA
    link.send(DST_OS, b"\x0a\x00", label="OS_ENABLE_CSMACA off")
    link.wait_for(lambda f: f.dst == DST_OS and f.cmd_byte() == 0x04, timeout=2.0)

    # Schritt 7: ENABLE_CREDITS dutyCycle on
    link.send(DST_OS, b"\x09\x01", label="OS_ENABLE_CREDITS on")
    link.wait_for(lambda f: f.dst == DST_OS and f.cmd_byte() == 0x04, timeout=2.0)

    # Schritt 8: GET_CREDITS
    link.send(DST_OS, b"\x08", label="OS_GET_CREDITS")
    link.wait_for(lambda f: f.dst == DST_OS and f.cmd_byte() == 0x04, timeout=2.0)

    # Schritt 9: keys all-zero
    z17 = b"\x00" * 17
    for cmd, name in [(b"\x03", "APP_SET_CURRENT_KEY"),
                      (b"\x0f", "APP_SET_PREVIOUS_KEY"),
                      (b"\x0b", "APP_SET_TEMP_KEY")]:
        link.send(DST_APP, cmd + z17, label=name)
        link.wait_for(lambda f: f.dst == DST_APP and f.cmd_byte() == 0x04, timeout=2.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("device", help="/dev/raw-uart oder Tunnel-aequivalent")
    ap.add_argument("--hmid", default="F11234", help="HMID hex 6 chars (default F11234)")
    ap.add_argument("--log", default=None, help="optional log file path")
    args = ap.parse_args()

    if len(args.hmid) != 6:
        ap.error("hmid must be 6 hex chars")
    hmid = bytes.fromhex(args.hmid)

    log_fp = open(args.log, "w") if args.log else None
    link = UartLink(args.device, log_fp)

    try:
        print(f"=== {ts()} init sequence start (hmid={args.hmid}) ===", flush=True)
        init_sequence(link, hmid)
        print(f"=== {ts()} init complete, normal mode active ===", flush=True)
        link.listen_forever()
    except KeyboardInterrupt:
        print(f"\n=== {ts()} listener stopped by user ===")
    finally:
        link.close()
        if log_fp:
            log_fp.close()


if __name__ == "__main__":
    main()
