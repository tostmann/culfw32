#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""HM-PB-2-WM Pair-Sequenz-Simulation via lokales CUL868.

Bypassed das RX-Sensitivity-Problem auf dem Test-Board: anstelle echter
Wandtaster-Funkframes spielt das lokale CUL868 (/dev/ttyACM2) selbst die
HM-PB-2-WM-Rolle. Wir verifizieren beide Wege byte-genau:

    Phase A — Device → Central (RX-Pfad):
        CUL868 TX `As<frame>` (synthetisches HM-PB-2-WM-PAIR_ANNOUNCE)
            → CULFW32 CC1101 RX
            → CULFW32 HMUARTLGW APP_RECV
            → unser TCP-Listener (FHEM-Surrogat)
        Verify: APP_RECV.askin == gesendetes Frame ab CNT (ohne LEN-Byte)

    Phase B — Central → Device (TX-Pfad):
        TCP-Listener APP_SEND mit CONFIG_PAIRING_REQUEST
            → CULFW32 HMUARTLGW
            → CULFW32 CC1101 TX
            → CUL868 RX als A<frame>
        Verify: CUL868-A-Line == APP_SEND payload ab CNT (mit LEN am Anfang)

Reference-Frames stammen aus echtem FHEM-Pair-Log gegen HM-MOD-RPI-PCB
(FHEM-Log eines Anlernvorgangs vom 28.04.2026, nicht im Repository).
"""
from __future__ import annotations

import argparse
import os
import select
import socket
import sys
import termios
import time
import tty
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame_parser import (
    Frame, FrameDecoder,
    DST_OS, DST_APP,
)


def ts() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def hexdump(b: bytes) -> str:
    return " ".join(f"{x:02x}" for x in b)


# ---------------------------------------------------------------------------
# CUL868 lokal (Ar-Mode)
# ---------------------------------------------------------------------------

class Cul:
    def __init__(self, path: str):
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        attrs[4] = termios.B38400
        attrs[5] = termios.B38400
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self._inbuf = bytearray()

    def close(self):
        try: os.close(self.fd)
        except OSError: pass

    def write_line(self, s: str):
        os.write(self.fd, (s + "\r\n").encode("ascii"))

    def drain_for(self, seconds: float) -> str:
        out = bytearray()
        deadline = time.time() + seconds
        while time.time() < deadline:
            r, _, _ = select.select([self.fd], [], [], min(0.1, max(0.01, deadline - time.time())))
            if not r: continue
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                continue
            if chunk:
                out.extend(chunk)
        return out.decode("ascii", errors="replace")

    def feed(self):
        """Return any new lines ending in \\n collected from the device."""
        while True:
            r, _, _ = select.select([self.fd], [], [], 0.0)
            if not r: break
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                break
            if not chunk: break
            self._inbuf.extend(chunk)
        lines = []
        while True:
            try:
                idx = self._inbuf.index(b"\n")
            except ValueError:
                break
            line = bytes(self._inbuf[:idx]).decode("ascii", errors="replace").strip()
            del self._inbuf[: idx + 1]
            if line:
                lines.append(line)
        return lines

    def wait_for_a_line(self, timeout: float = 2.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            for line in self.feed():
                if line.startswith("A") and not line.startswith("Ar") and not line.startswith("As"):
                    return line
            time.sleep(0.02)
        return None


# ---------------------------------------------------------------------------
# HMUARTLGW TCP-Frontend (FHEM-Surrogat)
# ---------------------------------------------------------------------------

class TcpLink:
    def __init__(self, host: str, port: int):
        self.sock = socket.create_connection((host, port), timeout=5.0)
        self.sock.setblocking(False)
        self.cnt = 0
        self.dec = FrameDecoder()
        self._rx_q: list[Frame] = []

    def close(self):
        try: self.sock.close()
        except OSError: pass

    def _next_cnt(self) -> int:
        self.cnt = (self.cnt + 1) & 0xFF
        if self.cnt == 0:
            self.cnt = 1
        return self.cnt

    def send(self, dst: int, payload: bytes, label: str = "") -> Frame:
        cnt = self._next_cnt()
        f = Frame(dst=dst, cnt=cnt, payload=payload)
        raw = f.encode()
        body_hex = payload.hex()
        print(f"[TCP-TX] {ts()} dst={dst:02x} cnt={cnt:02x} body={body_hex}"
              + (f"  ({label})" if label else ""), flush=True)
        self.sock.sendall(raw)
        return f

    def _ingest(self, timeout: float = 0.0):
        deadline = time.time() + max(0.0, timeout)
        while True:
            now = time.time()
            r, _, _ = select.select([self.sock], [], [], max(0.0, deadline - now) if timeout else 0.0)
            if not r: break
            try:
                chunk = self.sock.recv(4096)
            except (BlockingIOError, ConnectionResetError):
                break
            if not chunk: break
            self.dec.feed(chunk)
            for fr in self.dec.frames():
                self._log_rx(fr)
                self._rx_q.append(fr)
            if not timeout: break

    def _log_rx(self, fr: Frame):
        cmd = fr.cmd_byte()
        body = fr.payload[1:].hex() if len(fr.payload) > 1 else ""
        msg = f"[TCP-RX] {ts()} dst={fr.dst:02x} cnt={fr.cnt:02x} cmd={cmd:02x}({fr.cmd_name()}) body={body}"
        if fr.dst == DST_APP and cmd == 0x05 and len(fr.payload) >= 5:
            status = fr.payload[1]; info = fr.payload[2]; rssi = fr.payload[3]
            askin = fr.payload[4:]
            msg += (f"\n         APP_RECV: status={status:02x} info={info:02x} "
                    f"rssi=0x{rssi:02x}(-{rssi}dBm) askin_len={len(askin)} askin={askin.hex().upper()}")
        print(msg, flush=True)

    def wait_for(self, predicate, timeout: float = 2.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._ingest(timeout=min(0.2, deadline - time.time()))
            for i, fr in enumerate(list(self._rx_q)):
                if predicate(fr):
                    self._rx_q.pop(i)
                    return fr
        return None


def init_sequence(link: TcpLink, hmid: bytes):
    """FHEM-style HMUARTLGW init."""
    print(f"=== {ts()} init start (hmid={hmid.hex()}) ===", flush=True)
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
    for cmd, name in [(b"\x03","APP_SET_CURRENT_KEY"),
                      (b"\x0f","APP_SET_PREVIOUS_KEY"),
                      (b"\x0b","APP_SET_TEMP_KEY")]:
        link.send(DST_APP, cmd + z17, label=name)
        link.wait_for(lambda f: f.dst == DST_APP and f.cmd_byte() == 0x04, timeout=1.5)
    print(f"=== {ts()} init done ===", flush=True)


# ---------------------------------------------------------------------------
# HM-PB-2-WM Pair-Reference (aus FHEM 2026-04-28 23:26:47)
# ---------------------------------------------------------------------------
# AskSin: LEN=1A CNT=8F FLAG=84 TYPE=00 SRC=19B5DF DST=000000
#         FW=12 DEVTYPE=0036(HM-PB-2-WM) SERIAL="JEQ0064205" CLASS=40 SUBTYPE=02 0000
DEVICE_HMID  = bytes.fromhex("19B5DF")
CENTRAL_HMID = bytes.fromhex("F11234")  # FHEM/HMUARTLGW HMID
DEVICE_TYPE  = bytes.fromhex("0036")
DEVICE_SERIAL = b"JEQ0064205"
DEVICE_FW    = 0x12  # 1.2

def build_pair_announce(cnt: int = 0x8F) -> bytes:
    """LEN + CNT + FLAG + TYPE + SRC + DST + FW + DEVTYPE + SERIAL + CLASS + SUBTYPE + 0000."""
    body = (bytes([cnt, 0x84, 0x00]) + DEVICE_HMID + b"\x00\x00\x00"
            + bytes([DEVICE_FW]) + DEVICE_TYPE + DEVICE_SERIAL
            + bytes([0x40, 0x02, 0x00, 0x00]))
    frame = bytes([len(body)]) + body
    assert len(frame) == 0x1B  # LEN-Byte + 26 body bytes
    return frame


def build_config_pairing_req(cnt: int) -> bytes:
    """Central (F11234) → Device (19B5DF), CONFIG_PAIRING (mtype=01 sub=05).

    AskSin: LEN=10 CNT=<x> FLAG=A0 TYPE=01 SRC=F11234 DST=19B5DF
            00 05 00 00 00 00 00 00
        00      = channel 0 (central config)
        05      = subcmd PAIRING
        00 00   = peer-channel 0
        00 00 00 00 = padding
    """
    body = (bytes([cnt, 0xA0, 0x01]) + CENTRAL_HMID + DEVICE_HMID
            + bytes([0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    frame = bytes([len(body)]) + body
    return frame


def build_device_ack(cnt: int) -> bytes:
    """Device (19B5DF) → Central (F11234), ACK (mtype=02 payload=00)."""
    body = (bytes([cnt, 0x80, 0x02]) + DEVICE_HMID + CENTRAL_HMID
            + bytes([0x00]))
    return bytes([len(body)]) + body


def build_config_end(cnt: int) -> bytes:
    """Central (F11234) → Device (19B5DF), CONFIG_END (mtype=01 sub=06)."""
    body = (bytes([cnt, 0xA0, 0x01]) + CENTRAL_HMID + DEVICE_HMID
            + bytes([0x00, 0x06]))
    return bytes([len(body)]) + body


def build_param_response(cnt: int) -> bytes:
    """Device (19B5DF) → Central (F11234), INFO_PARAM_RESPONSE (mtype=10).

    Typical payload: 02 <chan> <peer> <list-bytes>
    """
    body = (bytes([cnt, 0x80, 0x10]) + DEVICE_HMID + CENTRAL_HMID
            + bytes([0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00]))
    return bytes([len(body)]) + body


# ---------------------------------------------------------------------------
# Phasen
# ---------------------------------------------------------------------------

def device_to_central(cul: Cul, link: TcpLink, askin_with_len: bytes, label: str) -> bool:
    """CUL868 sendet AskSin-Frame, wir erwarten APP_RECV mit identischer payload."""
    hex_str = askin_with_len.hex().upper()
    expect = askin_with_len[1:]
    print(f"\n[Step ↑ {label}] {ts()}", flush=True)
    print(f"  CUL868-TX: As{hex_str}", flush=True)
    print(f"    LEN={askin_with_len[0]:02x} CNT={askin_with_len[1]:02x} FLAG={askin_with_len[2]:02x}"
          f" TYPE={askin_with_len[3]:02x} SRC={askin_with_len[4:7].hex()}"
          f" DST={askin_with_len[7:10].hex()} payload={askin_with_len[10:].hex()}", flush=True)
    cul.write_line(f"As{hex_str}")
    rx = link.wait_for(
        lambda f: (f.dst == DST_APP and f.cmd_byte() == 0x05
                   and len(f.payload) >= 5 and f.payload[4:] == expect),
        timeout=3.0,
    )
    if rx is None:
        print(f"  [FAIL] kein matching APP_RECV", flush=True)
        return False
    print(f"  [PASS] APP_RECV.askin byte-genau == TX-Frame ab CNT", flush=True)
    return True


def central_to_device(cul: Cul, link: TcpLink, askin_with_len: bytes, burst: int, label: str) -> bool:
    """TCP APP_SEND, wir erwarten dass CUL868 das Frame on-air empfaengt."""
    askin_no_len = askin_with_len[1:]
    send_body = bytes([0x02, 0x00, 0x00, burst]) + askin_no_len
    print(f"\n[Step ↓ {label}] {ts()}", flush=True)
    cul.feed()  # drain stale
    link.send(DST_APP, send_body, label=f"APP_SEND {label}")
    link.wait_for(lambda f: f.dst == DST_APP and f.cmd_byte() == 0x04, timeout=2.0)
    line = cul.wait_for_a_line(timeout=2.5)
    if line is None:
        print(f"  [FAIL] CUL868 hat keinen on-air-Frame gesehen", flush=True)
        return False
    print(f"  CUL868-RX: {line}", flush=True)
    rx_hex = line[1:].split()[0]
    rx_bytes = bytes.fromhex(rx_hex)
    if not rx_bytes.startswith(askin_with_len):
        print(f"  [FAIL] byte-mismatch", flush=True)
        print(f"    expect: {askin_with_len.hex().upper()}", flush=True)
        print(f"    got:    {rx_bytes.hex().upper()}", flush=True)
        return False
    rssi = rx_bytes[len(askin_with_len):].hex() or "(none)"
    print(f"  [PASS] CUL868-RX byte-genau == APP_SEND-Frame, trailing={rssi}", flush=True)
    return True


def run_pair_dialog(cul: Cul, link: TcpLink) -> dict:
    """Multi-step HM-PB-2-WM Pair-Dialog (5 Frames, 2 Richtungen)."""
    print(f"\n=== {ts()} Pair-Dialog HM-PB-2-WM ↔ FHEM-Surrogat ===", flush=True)
    results = {}
    results["1: PAIR_ANNOUNCE (Dev→Central)"] = device_to_central(
        cul, link, build_pair_announce(cnt=0x8F),
        label="DEVICE_INFO PAIR_ANNOUNCE",
    )
    time.sleep(0.4)
    results["2: CONFIG_PAIRING (Central→Dev)"] = central_to_device(
        cul, link, build_config_pairing_req(cnt=0x06), burst=0x00,
        label="CONFIG_PAIRING (mtype=01 sub=05)",
    )
    time.sleep(0.4)
    results["3: ACK (Dev→Central)"] = device_to_central(
        cul, link, build_device_ack(cnt=0x90),
        label="ACK (mtype=02)",
    )
    time.sleep(0.4)
    results["4: INFO_PARAM_RESPONSE (Dev→Central)"] = device_to_central(
        cul, link, build_param_response(cnt=0x91),
        label="INFO_PARAM_RESPONSE (mtype=10)",
    )
    time.sleep(0.4)
    results["5: CONFIG_END (Central→Dev)"] = central_to_device(
        cul, link, build_config_end(cnt=0x07), burst=0x00,
        label="CONFIG_END (mtype=01 sub=06)",
    )
    return results


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cul", default=os.environ.get("CUL868_DEV", "/dev/ttyACM2"))
    ap.add_argument("--host", default=os.environ.get("CULFW32_HOST", "192.168.1.42"))
    ap.add_argument("--port", type=int, default=2325)
    ap.add_argument("--hmid", default="F11234")
    args = ap.parse_args()

    hmid = bytes.fromhex(args.hmid)

    cul = Cul(args.cul)
    try:
        cul.write_line("V")
        v = cul.drain_for(0.3)
        print(f"[CUL868] V → {v.strip()!r}", flush=True)
        cul.write_line("Ar")
        ar = cul.drain_for(0.3)
        print(f"[CUL868] Ar → {ar.strip()!r}", flush=True)
        time.sleep(0.1)
        cul.feed()  # drain

        link = TcpLink(args.host, args.port)
        try:
            init_sequence(link, hmid)
            time.sleep(0.2)

            results = run_pair_dialog(cul, link)

            print(f"\n=== {ts()} RESULT ===", flush=True)
            n_pass = sum(1 for v in results.values() if v)
            for label, ok in results.items():
                print(f"  {'PASS' if ok else 'FAIL'}  {label}", flush=True)
            print(f"\n  → {n_pass}/{len(results)} Frames byte-genau", flush=True)
            sys.exit(0 if n_pass == len(results) else 1)
        finally:
            link.close()
    finally:
        cul.close()


if __name__ == "__main__":
    main()
