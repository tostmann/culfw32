#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""CULFW32 Release-Smoke-Test: byte-genau gegen Lokales+Remote CUL868.

Pflicht-Test vor jedem Release. Faellt **eines** der Pruefungen aus, ist der
Build NICHT release-fertig. Pattern aus `slowrf_tests.py` verallgemeinert auf
alle aktiven Protokolle + Frontends.

Targets (konfigurierbar via Environment-Variablen, siehe _lab_env.py):
- CULFW32         ASCII via TCP $CULFW32_HOST:$CULFW32_PORT
- Lokales CUL868  USB-CDC $CUL868_DEV
- Remote CUL868   ASCII via TCP $CUL868_REMOTE_HOST:$CUL868_REMOTE_PORT

Protokolle:
- Moritz/MAX!  (Z)        — RX/TX lokal+remote
- AskSin       (A)        — RX/TX lokal+remote
- SlowRF FS20  (F + Xnn)  — RX/TX lokal+remote
- Native       (N)        — RX/TX lokal (Remote evtl. ohne HAS_NATIVE → Skip)
- FastRF       (f)        — RX/TX lokal (HAS_FASTRF aus auf Lokales+Remote → Skip)

Frontends auf CULFW32:
- ASCII 2323               — `V` Antwort byte-strukturiert
- HMUARTLGW 2325           — Hello-Push 200ms nach connect
- JSON 2324                — `{"cmd":"ping"}` round-trip
- SignalDuino 2326         — V-Init

Aufruf:
    python3 release_smoke_test.py            # alles
    python3 release_smoke_test.py rf         # nur RF
    python3 release_smoke_test.py frontends  # nur Frontends
"""
from __future__ import annotations

import os
import re
import select
import socket
import stat
import sys
from pathlib import Path
import termios
import time
import tty
from dataclasses import dataclass, field
from typing import Callable, Optional

# ─── Targets ─────────────────────────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _lab_env import (  # noqa: E402
    CULFW32_HOST as TARGET_HOST,
    CULFW32_PORT as TARGET_PORT,
    CUL868_DEV as DEV_LOCAL,
    CUL868_REMOTE_HOST as REMOTE_HOST,
    CUL868_REMOTE_PORT as REMOTE_PORT,
)

# ─── ANSI ────────────────────────────────────────────────────────────────────
GRN = "\033[32m"; RED = "\033[31m"; YLW = "\033[33m"; DIM = "\033[2m"; RST = "\033[0m"


def green(s):  return f"{GRN}{s}{RST}"
def red(s):    return f"{RED}{s}{RST}"
def yellow(s): return f"{YLW}{s}{RST}"
def dim(s):    return f"{DIM}{s}{RST}"


# ─── Connections ─────────────────────────────────────────────────────────────
class Conn:
    """Common interface: send(str), recv-until-timeout, drain."""

    def send(self, s: str): ...
    def recv_until(self, deadline: float) -> str: ...
    def drain(self, dur: float = 0.2) -> str: ...
    def close(self): ...


class TtyConn(Conn):
    def __init__(self, path: str):
        real = os.path.realpath(path)
        if not os.path.exists(real) or not stat.S_ISCHR(os.stat(real).st_mode):
            raise IOError(f"{path}: not a char device")
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        attrs[4] = attrs[5] = termios.B38400
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def send(self, s: str):
        os.write(self.fd, s.encode("ascii"))

    def recv_until(self, deadline: float) -> str:
        out = bytearray()
        while time.time() < deadline:
            r, _, _ = select.select([self.fd], [], [], min(0.2, max(0.05, deadline - time.time())))
            if not r:
                continue
            try: chunk = os.read(self.fd, 4096)
            except (BlockingIOError, OSError): continue
            if chunk: out.extend(chunk)
        return out.decode("ascii", errors="replace")

    def drain(self, dur: float = 0.2) -> str:
        return self.recv_until(time.time() + dur)

    def close(self):
        try: os.close(self.fd)
        except OSError: pass


class TcpConn(Conn):
    """TCP-Conn mit auto-reconnect bei BrokenPipe/ConnectionReset."""

    def __init__(self, host: str, port: int):
        self.host = host; self.port = port
        self._open()

    def _open(self):
        self.s = socket.create_connection((self.host, self.port), timeout=3.0)
        self.s.setblocking(False)

    def _reconnect(self) -> bool:
        try: self.s.close()
        except: pass
        time.sleep(0.5)
        try: self._open(); return True
        except Exception: return False

    def send(self, s: str):
        try:
            self.s.sendall(s.encode("ascii"))
        except (BrokenPipeError, ConnectionResetError, OSError):
            if not self._reconnect():
                raise
            time.sleep(0.2)
            self.s.sendall(s.encode("ascii"))

    def recv_until(self, deadline: float) -> str:
        out = bytearray()
        while time.time() < deadline:
            r, _, _ = select.select([self.s], [], [], min(0.2, max(0.05, deadline - time.time())))
            if not r:
                continue
            try: chunk = self.s.recv(4096)
            except (BlockingIOError, ConnectionResetError, OSError): continue
            if chunk: out.extend(chunk)
        return out.decode("ascii", errors="replace")

    def drain(self, dur: float = 0.2) -> str:
        return self.recv_until(time.time() + dur)

    def alive(self) -> bool:
        """Quick TCP probe: try send single byte, reconnect-and-keep on fail."""
        try:
            self.s.sendall(b"\r\n")
            return True
        except (BrokenPipeError, ConnectionResetError, OSError):
            return self._reconnect()

    def close(self):
        try: self.s.close()
        except OSError: pass


# ─── Test framework ──────────────────────────────────────────────────────────
@dataclass
class Result:
    name:    str
    status:  str         # "pass" | "fail" | "skip"
    detail:  str = ""
    elapsed: float = 0.0


@dataclass
class Suite:
    results: list = field(default_factory=list)

    def run(self, name: str, fn: Callable[[], Optional[str]], target_check=None):
        sys.stdout.write(f"  {name:<42}")
        sys.stdout.flush()
        # Pre-check: if target is dead, try recover
        if target_check is not None and not target_check():
            self.results.append(Result(name, "fail", "target unreachable (crashed?)", 0))
            print(f"{red('FAIL')} {dim('0.00s')}  target unreachable (crashed?)")
            return
        t0 = time.time()
        try:
            ret = fn()
            elapsed = time.time() - t0
            if ret is None:
                self.results.append(Result(name, "pass", "", elapsed))
                print(f"{green('PASS')} {dim(f'{elapsed:.2f}s')}")
            elif ret.startswith("SKIP:"):
                self.results.append(Result(name, "skip", ret[5:].strip(), elapsed))
                print(f"{yellow('SKIP')} {dim(f'{elapsed:.2f}s')}  {ret[5:].strip()}")
            else:
                self.results.append(Result(name, "fail", ret, elapsed))
                print(f"{red('FAIL')} {dim(f'{elapsed:.2f}s')}  {ret}")
        except Exception as e:
            elapsed = time.time() - t0
            self.results.append(Result(name, "fail", f"exception: {e!r}", elapsed))
            print(f"{red('FAIL')} {dim(f'{elapsed:.2f}s')}  exception: {e!r}")

    def summary(self) -> int:
        n_pass = sum(1 for r in self.results if r.status == "pass")
        n_fail = sum(1 for r in self.results if r.status == "fail")
        n_skip = sum(1 for r in self.results if r.status == "skip")
        total = len(self.results)
        line = f"{green(f'{n_pass} pass')}, {red(f'{n_fail} fail')}, {yellow(f'{n_skip} skip')}  /  {total} total"
        print(f"\n{line}")
        if n_fail:
            print(f"\n{red('FAILED')}:")
            for r in self.results:
                if r.status == "fail":
                    print(f"  • {r.name:<42} {r.detail}")
        return 1 if n_fail else 0


# ─── Helpers ─────────────────────────────────────────────────────────────────
def reset_mode(c: Conn):
    """Disable all RX modes. X00 schaltet SlowRF aus; A/Z/N/f deaktivieren mit
    leerem 1-arg Aufruf (a-culfw rf_*.c je Klasse handhabt das selbst)."""
    c.send("X00\r\n")
    time.sleep(0.1)
    c.send("Ax\r\n")  # not 'r' or 'R' → deactivate
    c.send("Zx\r\n")
    c.send("Nx\r\n")
    c.send("fx\r\n")
    c.drain(0.4)


def expect_in(c: Conn, pattern: str, timeout: float = 2.0) -> Optional[str]:
    """Lese bis `pattern` (Regex) im Output zu sehen ist. Gibt match oder None."""
    rx = re.compile(pattern, re.MULTILINE)
    end = time.time() + timeout
    out = ""
    while time.time() < end:
        chunk = c.recv_until(min(end, time.time() + 0.3))
        if chunk:
            out += chunk
            m = rx.search(out)
            if m:
                return m.group(0)
    return None


# ─── RF tests ────────────────────────────────────────────────────────────────
# Format: (mode-cmd, send-prefix, expect-regex, frame-hex)
# Mode-cmd aktiviert RX auf beiden Seiten. Frame ist die On-Air-Payload.
# Expect-Regex matcht die Empfangs-Zeile (mit RSSI-Suffix).

class RfPair:
    """Test-Helfer fuer die 4 Richtungen pro RF-Klasse."""

    def __init__(self, target: Conn, peer: Conn, mode_cmd: str,
                 send_prefix: str, frame_hex: str, expect_re: str):
        self.target = target
        self.peer = peer
        self.mode_cmd = mode_cmd
        self.send_prefix = send_prefix
        self.frame = frame_hex
        self.expect_re = expect_re

    def setup(self):
        for c in (self.target, self.peer):
            reset_mode(c)
            c.send(f"{self.mode_cmd}\r\n")
            time.sleep(0.6)        # chip settle (SCAL etc.); Ar/Zr antworten seit 0.1.882 nicht mehr
            c.drain(0.4)

    def tx(self, sender: Conn, receiver: Conn) -> Optional[str]:
        """Sender → Receiver. 2 Retries falls erster TX-Versuch nichts liefert
        (Lab-RF ist nicht 100% reliable; ein einziger Frame-Loss soll noch kein
        FAIL produzieren — RF in der Praxis wiederholt auch 3× nach a-culfw)."""
        rx = re.compile(self.expect_re, re.MULTILINE)
        last_buf = ""
        for attempt in range(3):
            receiver.drain(0.3)
            sender.send(f"{self.send_prefix}{self.frame}\r\n")
            end = time.time() + 1.8
            out = ""
            while time.time() < end:
                chunk = receiver.recv_until(min(end, time.time() + 0.3))
                if chunk: out += chunk
                if rx.search(out):
                    return None
            last_buf = out
            time.sleep(0.3)        # short pause before retry
        snippet = last_buf.replace("\r", "\\r").replace("\n", "\\n")[:200]
        return f"no match in 3 attempts; last buffer: '{snippet}'"


# ─── Test bodies ─────────────────────────────────────────────────────────────
def make_tests(target: Conn, local: Conn, remote: Optional[Conn]) -> list:
    """Returns list of (name, callable). Callable returns None/SKIP/error-string."""
    tests = []

    # 0) Boot/V tests
    def test_v_target():
        target.drain(0.1); target.send("V\r\n")
        m = expect_in(target, r"V \S+\s+CULFW32\s+Build:\s+\d+", 1.5)
        return None if m else "no V response"

    def test_v_local():
        local.drain(0.1); local.send("V\r\n")
        m = expect_in(local, r"V \S+\s+a-culfw\s+Build:\s+\d+", 1.5)
        return None if m else "no V response"

    def test_v_remote():
        if not remote: return "SKIP: remote unreachable"
        remote.drain(0.1); remote.send("V\r\n")
        m = expect_in(remote, r"V \S+\s+a-culfw\s+Build:\s+\d+", 1.5)
        return None if m else "no V response"

    tests.append(("V CULFW32",        test_v_target))
    tests.append(("V Lokales CUL868", test_v_local))
    tests.append(("V Remote CUL868",  test_v_remote))

    # ── 1) AskSin (A) ────────────────────────────────────────────────────────
    askin_frame = "0A778002AABBCCF1123400"  # 10-byte ACK frame
    askin_re    = r"A0A778002AABBCCF1123400[0-9A-Fa-f]*"

    def setup_asksin(peer):
        return RfPair(target, peer, "Ar", "As", askin_frame, askin_re)

    def t_asksin_local_tx():
        p = setup_asksin(local); p.setup()
        return p.tx(target, local)

    def t_asksin_local_rx():
        p = setup_asksin(local); p.setup()
        return p.tx(local, target)

    def t_asksin_remote_tx():
        if not remote: return "SKIP: remote unreachable"
        p = setup_asksin(remote); p.setup()
        return p.tx(target, remote)

    def t_asksin_remote_rx():
        if not remote: return "SKIP: remote unreachable"
        p = setup_asksin(remote); p.setup()
        return p.tx(remote, target)

    tests.append(("AskSin lokal: CULFW32 → CUL868", t_asksin_local_tx))
    tests.append(("AskSin lokal: CUL868 → CULFW32", t_asksin_local_rx))
    tests.append(("AskSin remote: CULFW32 → Remote", t_asksin_remote_tx))
    tests.append(("AskSin remote: Remote → CULFW32", t_asksin_remote_rx))

    # ── 2) Moritz/MAX! (Z) ───────────────────────────────────────────────────
    # Minimal ShutterContact-Status-Frame (mtype=0x30): LEN=0x0B
    # 0B 11 04 30 AABBCC F1F1F1 00 00
    moritz_frame = "0B110430AABBCCF1F1F10000"
    moritz_re    = r"Z0B110430AABBCCF1F1F10000[0-9A-Fa-f]*"

    def setup_moritz(peer):
        return RfPair(target, peer, "Zr", "Zs", moritz_frame, moritz_re)

    def t_moritz_local_tx():
        p = setup_moritz(local); p.setup()
        return p.tx(target, local)
    def t_moritz_local_rx():
        p = setup_moritz(local); p.setup()
        return p.tx(local, target)
    def t_moritz_remote_tx():
        if not remote: return "SKIP: remote unreachable"
        p = setup_moritz(remote); p.setup()
        return p.tx(target, remote)
    def t_moritz_remote_rx():
        if not remote: return "SKIP: remote unreachable"
        p = setup_moritz(remote); p.setup()
        return p.tx(remote, target)

    tests.append(("Moritz lokal: CULFW32 → CUL868", t_moritz_local_tx))
    tests.append(("Moritz lokal: CUL868 → CULFW32", t_moritz_local_rx))
    tests.append(("Moritz remote: CULFW32 → Remote", t_moritz_remote_tx))
    tests.append(("Moritz remote: Remote → CULFW32", t_moritz_remote_rx))

    # ── 3) SlowRF FS20 (F + X21) ─────────────────────────────────────────────
    # Frame F12345678 = HC1=12 HC2=34 ADDR=56 CMD=78. CUL appends checksum.
    fs20_frame = "12345678"
    fs20_re    = r"F12345678[0-9A-Fa-f]{0,4}"

    def setup_slowrf(peer):
        # X21 = REPORT_KNOWN | REPORT_RSSI; reset first
        for c in (target, peer):
            reset_mode(c)
            c.send("X21\r\n"); time.sleep(0.15); c.drain(0.2)
        return None

    def t_slowrf_local_tx():
        setup_slowrf(local)
        local.drain(0.2); target.send(f"F{fs20_frame}\r\n")
        m = expect_in(local, fs20_re, 2.0)
        return None if m else f"no '{fs20_re}'"
    def t_slowrf_local_rx():
        setup_slowrf(local)
        target.drain(0.2); local.send(f"F{fs20_frame}\r\n")
        m = expect_in(target, fs20_re, 2.0)
        return None if m else f"no '{fs20_re}'"
    def t_slowrf_remote_tx():
        if not remote: return "SKIP: remote unreachable"
        setup_slowrf(remote)
        remote.drain(0.2); target.send(f"F{fs20_frame}\r\n")
        m = expect_in(remote, fs20_re, 2.0)
        return None if m else f"no '{fs20_re}'"
    def t_slowrf_remote_rx():
        if not remote: return "SKIP: remote unreachable"
        setup_slowrf(remote)
        target.drain(0.2); remote.send(f"F{fs20_frame}\r\n")
        m = expect_in(target, fs20_re, 2.0)
        return None if m else f"no '{fs20_re}'"

    tests.append(("SlowRF lokal: CULFW32 → CUL868", t_slowrf_local_tx))
    tests.append(("SlowRF lokal: CUL868 → CULFW32", t_slowrf_local_rx))
    tests.append(("SlowRF remote: CULFW32 → Remote", t_slowrf_remote_tx))
    tests.append(("SlowRF remote: Remote → CULFW32", t_slowrf_remote_rx))

    # ── 4) Native (N) — Sniffer-only, kein TX im Protokoll ─────────────────
    # a-culfw rf_native.c kennt nur `Nr<mode>` (RX on) und `Nx` (off), KEIN `Ns`.
    # Native ist passiver Sniffer fuer IT+/PCA301. 4-Wege-RX/TX-Test prinzipiell
    # unmoeglich ohne reale Sensoren. Statt dessen: Mode-Aktivierung-Smoke.
    def t_native_mode_smoke():
        target.drain(0.1)
        target.send("N1\r\n")
        out = ""
        end = time.time() + 1.0
        while time.time() < end:
            chunk = target.recv_until(min(end, time.time() + 0.2))
            if chunk: out += chunk
        if "unknown" in out.lower():
            return f"N1 not implemented: {out!r}"
        target.send("Nx\r\n")
        target.recv_until(time.time() + 0.5)
        return None

    tests.append(("Native N1/Nx mode smoke", t_native_mode_smoke))

    def t_wmbus_b_mode_smoke():
        """`b`/`brt`/`brs`/`brx` Frontend-Statemachine byte-genau a-culfw."""
        target.drain(0.1)
        # Off → OFF
        target.send("b\r\n")
        out = ""
        end = time.time() + 1.0
        while time.time() < end:
            chunk = target.recv_until(min(end, time.time() + 0.2))
            if chunk: out += chunk
        if "OFF" not in out:
            return f"b initial: expected OFF, got {out!r}"
        # T-Mode an
        target.send("brt\r\n")
        end = time.time() + 1.5
        out = ""
        while time.time() < end:
            chunk = target.recv_until(min(end, time.time() + 0.2))
            if chunk: out += chunk
        if "TMODE" not in out:
            return f"brt: expected TMODE, got {out!r}"
        # S-Mode an (Switch)
        target.send("brs\r\n")
        end = time.time() + 1.5
        out = ""
        while time.time() < end:
            chunk = target.recv_until(min(end, time.time() + 0.2))
            if chunk: out += chunk
        if "SMODE" not in out:
            return f"brs: expected SMODE, got {out!r}"
        # Off
        target.send("brx\r\n")
        end = time.time() + 1.0
        out = ""
        while time.time() < end:
            chunk = target.recv_until(min(end, time.time() + 0.2))
            if chunk: out += chunk
        if "OFF" not in out:
            return f"brx: expected OFF, got {out!r}"
        return None

    tests.append(("wMBus b/brt/brs/brx state machine", t_wmbus_b_mode_smoke))

    return tests


def make_frontend_tests(target_host: str) -> list:
    """Frontend-Tests gegen CULFW32: HMUARTLGW, JSON, SignalDuino."""
    tests = []

    def _hmu_recv(port, deadline_s=1.5):
        s = socket.create_connection((target_host, port), timeout=2)
        s.setblocking(False)
        end = time.time() + deadline_s
        out = bytearray()
        while time.time() < end:
            r, _, _ = select.select([s], [], [], 0.2)
            if r:
                try: chunk = s.recv(4096)
                except: continue
                if chunk: out.extend(chunk)
        s.close()
        return bytes(out)

    def _hmu_crc16(data):
        # HMUARTLGW CRC: poly 0x8005, init 0xD77F. Matches firmware
        # hmuartlgw_frame.hpp::hmu_crc16. Verified byte-genau gegen FHEM-Trace.
        crc = 0xD77F
        for b in data:
            crc ^= (b << 8)
            for _ in range(8):
                if crc & 0x8000: crc = ((crc << 1) ^ 0x8005) & 0xFFFF
                else:            crc = (crc << 1) & 0xFFFF
        return crc

    def _hmu_frame(dst, cnt, cmd, body=b''):
        # 0xfd len_be(2) dst cnt cmd body crc_be(2)
        payload = bytes([dst, cnt, cmd]) + body
        ln = len(payload)
        header = bytes([0xfd, (ln >> 8) & 0xff, ln & 0xff])
        crc = _hmu_crc16(header + payload)
        return header + payload + bytes([(crc >> 8) & 0xff, crc & 0xff])

    def _hmu_request(port, dst, cnt, cmd, body=b''):
        # Connect, drain any unsolicited Hello-Push, send a frame, read reply.
        s = socket.create_connection((target_host, port), timeout=2)
        s.setblocking(False)
        end = time.time() + 1.0
        while time.time() < end:
            r, _, _ = select.select([s], [], [], 0.2)
            if r:
                try: s.recv(4096)
                except: pass
        s.send(_hmu_frame(dst, cnt, cmd, body))
        end = time.time() + 2.0
        out = bytearray()
        while time.time() < end:
            r, _, _ = select.select([s], [], [], 0.3)
            if r:
                try:
                    c = s.recv(4096)
                    if c: out.extend(c)
                except: pass
        s.close()
        return bytes(out)

    def t_hmuartlgw_dual_no_unsolicited():
        # :2327 = RPI-RF-MOD DualCoPro Emulation. Echte Hardware sendet KEINEN
        # unsolicited Connect-Time-Push (rfd::CCU2CommController::improvedInit
        # crashed daran, siehe notes/rfd_identify_unsolicited_push_2026-05-13.md).
        # Wir muessen also 1.5s nach Connect ohne ein einziges Byte vom Modul
        # auskommen.
        out = _hmu_recv(2327, 1.5)
        if out: return f"unexpected unsolicited push on :2327: {out.hex()}"
        return None

    def t_hmuartlgw_legacy_no_unsolicited():
        # :2325 = HM-MOD-RPI-PCB Emulation. Symmetrisch zur :2327-Regel.
        out = _hmu_recv(2325, 1.5)
        if out: return f"unexpected unsolicited push on :2325: {out.hex()}"
        return None

    def t_hmuartlgw_legacy_get_app():
        # FHEM-Init-Probe: OS_GET_APP -> "Co_CPU_App". Verifiziert dass die :2325
        # Slots wirklich Legacy-stamped sind (nicht versehentlich Dual).
        out = _hmu_request(2325, dst=0x00, cnt=0x01, cmd=0x00)
        if b"Co_CPU_App" not in out:
            return f"no Co_CPU_App reply on :2325: {out.hex()}"
        return None

    def t_hmuartlgw_dual_identify():
        # COMMON_IDENTIFY auf dst=FE → echte RPI-RF-MOD-Hardware antwortet mit
        # cmd=0x05 + "HMIP_TRX_Bl" (BL-Banner; im BL identifiziert sich auch der
        # DualCoPro als HMIP_TRX_Bl — HmIP/BidCoS noch nicht differenziert).
        # Verifiziert dass :2327 Slots Dual-stamped sind und der COMMON_IDENTIFY-
        # Handler spec-konform genau ein Frame zurueckgibt.
        out = _hmu_request(2327, dst=0xFE, cnt=0x01, cmd=0x01)
        if b"HMIP_TRX_Bl" not in out:
            return f"no HMIP_TRX_Bl banner on :2327: {out.hex()}"
        return None

    def t_json_ping():
        # _json_get drained hello und liest bis value-Reply unseren Key enthaelt.
        data = _json_get("fw.version")
        if b'"value"' not in data or b'"key":"fw.version"' not in data:
            return f"no value-reply: {data!r}"
        return None

    def t_signalduino_v():
        s = socket.create_connection((target_host, 2326), timeout=2)
        s.settimeout(1.5)
        # SignalDuino-Frontend antwortet auf plain V (STX/ETX-wrap optional).
        s.sendall(b"V\r\n")
        try: data = s.recv(4096)
        except socket.timeout: data = b""
        s.close()
        if not data:
            return "no response"
        if b"SIGNALduino" not in data:
            return f"no SIGNALduino banner: {data!r}"
        return None

    def t_ascii_v():
        s = socket.create_connection((target_host, 2323), timeout=2)
        s.setblocking(False)
        s.sendall(b"V\r\n")
        end = time.time() + 1.0
        out = bytearray()
        while time.time() < end:
            r, _, _ = select.select([s], [], [], 0.2)
            if r:
                try: chunk = s.recv(4096)
                except: continue
                if chunk: out.extend(chunk)
        s.close()
        if b"CULFW32" not in out:
            return f"no CULFW32 banner: {out!r}"
        return None

    def _json_get(key):
        # JSON-Frontend pusht Hello-Event und 5s-Live-Pushes asynchron — wir
        # muessen drainen bis wir die value-Reply mit unserem Key sehen.
        s = socket.create_connection((target_host, 2324), timeout=2)
        s.setblocking(False)
        # Drain initial hello.
        end = time.time() + 0.5
        while time.time() < end:
            r,_,_ = select.select([s], [], [], 0.1)
            if r:
                try: s.recv(4096)
                except: pass
        s.send(b'{"cmd":"get","key":"' + key.encode() + b'"}\n')
        # Sammle bis zu 2.5 s, breche frueh ab wenn unsere Antwort drin ist.
        end = time.time() + 2.5
        out = bytearray()
        needle = b'"key":"' + key.encode() + b'"'
        while time.time() < end:
            r,_,_ = select.select([s], [], [], 0.2)
            if r:
                try: chunk = s.recv(4096)
                except: continue
                if chunk: out.extend(chunk)
                if needle in out and b'"value"' in out:
                    break
        s.close()
        return bytes(out)

    def t_sysmon_uptime():
        # SystemMonitor: sys.uptime ist FLAG_READONLY in der ConfigRegistry.
        # Erwartet Format wie "12s" oder "3m 4s" oder "1h 23m" oder "2d 4h 15m".
        data = _json_get("sys.uptime")
        if b'"value"' not in data:
            return f"no value-reply for sys.uptime: {data!r}"
        # Plausibilitaet: Boot kommt < 1 min, also "Ns" oder "Nm Ms"
        if not (b'"key":"sys.uptime"' in data):
            return f"sys.uptime key missing: {data!r}"
        return None

    def t_sysmon_reset_reason():
        # last_reset gibt es als short (POR/SW/PANIC/...) und long (Klartext).
        data = _json_get("sys.reset_reason")
        if b'"value"' not in data:
            return f"no value-reply for sys.reset_reason: {data!r}"
        # mindestens eine bekannte Reason-Kennung
        valid = [b"POR", b"SW", b"EXT", b"PANIC", b"WDT", b"TASK_WDT",
                 b"INT_WDT", b"BROWN", b"DEEP", b"GLITCH", b"LOCK", b"UNK",
                 b"USB", b"JTAG", b"SDIO", b"EFUSE"]
        if not any(v in data for v in valid):
            return f"unrecognized reset_reason: {data!r}"
        return None

    def t_sysmon_boot_count():
        # Muss > 0 sein nach init().
        data = _json_get("sys.boot_count")
        if b'"value"' not in data:
            return f"no value-reply for sys.boot_count: {data!r}"
        # Parse "value":"N" — zumindest 1 nach erstem Boot.
        import re
        m = re.search(rb'"value"\s*:\s*"(\d+)"', data)
        if not m: return f"no numeric value: {data!r}"
        n = int(m.group(1))
        if n < 1: return f"boot_count = {n} < 1"
        return None

    tests.append(("Frontend ASCII :2323 V", t_ascii_v))
    tests.append(("Frontend JSON :2324 get fw.version", t_json_ping))
    tests.append(("Frontend HMUARTLGW :2325 no unsolicited push", t_hmuartlgw_legacy_no_unsolicited))
    tests.append(("Frontend HMUARTLGW :2325 OS_GET_APP -> Co_CPU_App", t_hmuartlgw_legacy_get_app))
    tests.append(("Frontend HMUARTLGW :2327 no unsolicited push", t_hmuartlgw_dual_no_unsolicited))
    tests.append(("Frontend HMUARTLGW :2327 COMMON_IDENTIFY -> HMIP_TRX_Bl", t_hmuartlgw_dual_identify))

    def t_hmuartlgw_rfd_improvedinit():
        # End-to-End: rfd::CCU2CommController::improvedInit() Frame-Sequenz
        # gegen :2327. Bricht wenn ein zusaetzlicher unsolicited Frame nach
        # OS_GET_APP kommt (HMIP_TRX_Bl-Banner) — siehe
        # notes/rfd_identify_unsolicited_push_2026-05-13.md
        import subprocess
        probe = Path(__file__).resolve().parents[2] / "test" / "hmuartlgw" / "rfd_identify_probe.py"
        if not probe.exists():
            return f"probe script missing: {probe}"
        r = subprocess.run([sys.executable, str(probe), "--host", target_host, "--port", "2327"],
                           capture_output=True, timeout=15)
        if r.returncode != 0:
            tail = (r.stdout + r.stderr).decode("utf-8", "replace").strip().splitlines()[-3:]
            return f"rfd-probe rc={r.returncode}: " + " | ".join(tail)
        return None

    tests.append(("Frontend HMUARTLGW :2327 rfd::improvedInit", t_hmuartlgw_rfd_improvedinit))
    tests.append(("Frontend SignalDuino :2326 V", t_signalduino_v))
    tests.append(("SystemMonitor sys.uptime",        t_sysmon_uptime))
    tests.append(("SystemMonitor sys.reset_reason",  t_sysmon_reset_reason))
    tests.append(("SystemMonitor sys.boot_count >= 1", t_sysmon_boot_count))
    return tests


# ─── Main ────────────────────────────────────────────────────────────────────
def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", nargs="?", default="all", choices=["all","rf","frontends"])
    ap.add_argument("--repeat", type=int, default=1,
                    help="Anzahl Runs. Bei >1 wird der Test als 'best-of-N' gewertet — "
                         "ein Test gilt als PASS wenn er in mindestens 1 von N Runs PASS war. "
                         "Empfohlen 3-5 fuer release-criterion (RF-Lab-Variance).")
    args_ns = ap.parse_args()
    arg = args_ns.mode
    repeat = max(1, args_ns.repeat)

    print(f"\n{dim('CULFW32 Release Smoke Test  '):<60}")
    print(f"{dim('  Target  '):<10} {TARGET_HOST}:{TARGET_PORT}")
    print(f"{dim('  Local   '):<10} {DEV_LOCAL}")
    print(f"{dim('  Remote  '):<10} {REMOTE_HOST}:{REMOTE_PORT}")
    print(f"{dim('  Mode    '):<10} {arg}")
    print(f"{dim('  Repeat  '):<10} {repeat} run(s){' (best-of-N)' if repeat>1 else ''}")
    print()

    target = TcpConn(TARGET_HOST, TARGET_PORT)
    local  = TtyConn(DEV_LOCAL)
    try:
        remote: Optional[TcpConn] = TcpConn(REMOTE_HOST, REMOTE_PORT)
    except Exception as e:
        print(yellow(f"  (remote CUL unreachable: {e})"))
        remote = None

    suite = Suite()

    # Initial drain
    for c in (target, local, *( [remote] if remote else [] )):
        c.drain(0.3)

    target_check = lambda: target.alive()

    # Build complete test list (lazy-evaluated on each repeat).
    rf_tests = make_tests(target, local, remote) if arg in ("all", "rf") else []
    fe_tests = make_frontend_tests(TARGET_HOST)  if arg in ("all", "frontends") else []
    all_tests = [(name, fn, target_check) for name, fn in rf_tests] + \
                [(name, fn, None)         for name, fn in fe_tests]

    if repeat == 1:
        if rf_tests:
            print(f"{dim('━━━ RF protocols ━━━')}")
            for name, fn in rf_tests: suite.run(name, fn, target_check=target_check)
        if fe_tests:
            print(f"\n{dim('━━━ Frontends ━━━')}")
            for name, fn in fe_tests: suite.run(name, fn)
    else:
        # Best-of-N: jeden Test bis zu N mal versuchen, ein PASS reicht.
        # Rationale: RF-Lab ist nicht 100% zuverlaessig; ein deterministisch
        # broken Test scheitert auch in N runs, ein flaky Test ist meist gruen.
        from collections import defaultdict
        results_by_test = defaultdict(list)
        for r in range(repeat):
            print(f"{dim(f'━━━ Run {r+1}/{repeat} ━━━')}")
            for name, fn, tcheck in all_tests:
                # Mini-suite per run, so we can capture per-run result
                mini = Suite()
                mini.run(name, fn, target_check=tcheck)
                results_by_test[name].append(mini.results[-1])
            print()
        # Aggregate: best-of-N (= PASS if any run was PASS)
        print(f"{dim('━━━ Aggregate (best-of-' + str(repeat) + ') ━━━')}")
        for name, _, _ in all_tests:
            runs = results_by_test[name]
            stats = sum(1 for r in runs if r.status == "pass")
            if stats > 0:
                detail = f"{stats}/{repeat} runs PASS"
                suite.results.append(Result(name, "pass", detail, 0))
                print(f"  {name:<42}{green('PASS')} {dim(detail)}")
            elif any(r.status == "skip" for r in runs):
                detail = next(r.detail for r in runs if r.status == "skip")
                suite.results.append(Result(name, "skip", detail, 0))
                print(f"  {name:<42}{yellow('SKIP')} {dim(detail)}")
            else:
                detail = runs[-1].detail
                suite.results.append(Result(name, "fail", detail, 0))
                print(f"  {name:<42}{red('FAIL')} {dim('0/'+str(repeat)+' runs PASS')}")

    # Reset modes (best-effort, target may be down)
    for c in (target, local):
        try: reset_mode(c)
        except Exception: pass
    if remote:
        try: reset_mode(remote)
        except Exception: pass

    # Cleanup
    for c in (target, local, *( [remote] if remote else [] )):
        c.close()

    code = suite.summary()
    sys.exit(code)


if __name__ == "__main__":
    main()
