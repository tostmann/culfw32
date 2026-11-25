#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""
HB-RF-ETH resume test against CULFW32 (UDP port 3008).

Plays the connection handling of hb_rf_eth.ko (piVCCU kernel/hb_rf_eth.c):

- Every connection attempt opens a fresh UDP socket, so it comes from a new
  source port, and sends CONNECT V2 with the endpoint id of the last
  successful connect (0 on the very first one). The kernel waits 50 ms, but
  each receive blocks up to 100 ms, so an answer within about 100 ms counts;
  otherwise the attempt is repeated after 500 ms (about every 0.6 s).
- After 5 s without a packet from the stick the kernel drops its socket and
  resumes that way.
- StartConn (type 5) is sent only on the first open() of /dev/raw-uart,
  never after a resume.
- Every packet carries the kernel's running message counter; the CONNECT
  answer mirrors it.

Cases:
  session  CONNECT with endpoint id 0, StartConn, HMUARTLGW OS_GET_APP
           answered
  resume   7 s without packets (the stick times out after 5 s), resume from
           a new port with the stored endpoint id and without StartConn:
           OS_GET_APP answered, keepalives at the new port, none at the old
  storm    20 resumes from new ports in a row, then OS_GET_APP answered
  stopconn StopConn, 7 s without packets, resume: still no answer to
           OS_GET_APP until a new StartConn
  forget   DISCONNECT, then a resume with the old endpoint id (the stick no
           longer knows the session): it takes the id over and answers
           OS_GET_APP without StartConn
  reboot   (--reboot) restart the stick with `B00` on its culfw console
           (TCP), resume with the stored endpoint id until it answers:
           OS_GET_APP answered without StartConn
  v1       CONNECT V1: answer [1, counter], OS_GET_APP answered only after
           StartConn
  newsess  CONNECT with endpoint id 0 from a new port: new endpoint id,
           OS_GET_APP not answered before StartConn, answered after it

Before and after the run the tool reads sys.boot_count and sys.crash_count
through the culfw console on TCP (CULFW32_PORT, default 2323) and reports
restarts that were not asked for.

    CULFW32_HOST=<stick ip> python3 firmware/tools/test_hbrfeth_resume.py [--reboot]

The stick keeps one session per host IP. Do not run the tool on a host
whose hb_rf_eth.ko is connected to the same stick: the test takes over that
host's session and ends it with DISCONNECT.
"""

import argparse
import os
import re
import select
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _lab_env import CULFW32_HOST, CULFW32_PORT  # noqa: E402

UDP_PORT = 3008
T_CONNECT, T_DISCONNECT, T_KEEPALIVE, T_STARTCONN, T_STOPCONN, T_FRAME = 0, 1, 2, 5, 6, 7
HMU_START, HMU_ESC = 0xFD, 0xFC
HMU_DST_OS, OS_GET_APP = 0x00, 0x00


def crc16(buf: bytes) -> int:
    # HMUARTLGW CRC-16: poly 0x8005, init 0xd77f
    crc = 0xd77f
    for b in buf:
        for _ in range(8):
            xor_in = ((crc & 0x8000) >> 8) ^ (b & 0x80)
            crc = ((crc << 1) & 0xffff) ^ (0x8005 if xor_in else 0)
            b = (b << 1) & 0xff
    return crc


def parse(data: bytes):
    """(type, payload) of an HB-RF-ETH packet, None on a bad CRC."""
    if len(data) < 4 or crc16(data[:-2]) != ((data[-2] << 8) | data[-1]):
        return None
    return data[0], data[2:-2]


def hmu_frame(dst: int, cnt: int, payload: bytes) -> bytes:
    length = 2 + len(payload)
    body = bytes([length >> 8, length & 0xff, dst, cnt]) + payload
    c = crc16(bytes([HMU_START]) + body)
    out = bytearray([HMU_START])
    for b in body + bytes([c >> 8, c & 0xff]):
        out += bytes([HMU_ESC, b ^ 0x80]) if b in (HMU_START, HMU_ESC) else bytes([b])
    return bytes(out)


def hmu_parse(data: bytes):
    """(dst, cnt, payload) of one HMUARTLGW frame, None if it does not decode."""
    if not data or data[0] != HMU_START:
        return None
    raw, esc = bytearray(), False
    for b in data[1:]:
        if esc:
            raw.append(b ^ 0x80)
            esc = False
        elif b == HMU_ESC:
            esc = True
        else:
            raw.append(b)
    if len(raw) < 6:
        return None
    length = (raw[0] << 8) | raw[1]
    if len(raw) != 2 + length + 2:
        return None
    if crc16(bytes([HMU_START]) + bytes(raw[:2 + length])) != ((raw[-2] << 8) | raw[-1]):
        return None
    return raw[2], raw[3], bytes(raw[4:2 + length])


class Kernel:
    """Connection handling of hb_rf_eth.ko, one socket per attempt."""

    def __init__(self, host: str):
        self.host = host
        self.msg_cnt = 0
        self.ep = 0
        self.sock = None
        self.old_socks = []
        self.hmu_cnt = 0x40
        self.last_ka_out = 0.0

    def _next(self) -> int:
        self.msg_cnt = (self.msg_cnt + 1) & 0xff
        return self.msg_cnt

    def _pkt(self, type_byte: int, payload: bytes = b"") -> bytes:
        body = bytes([type_byte, self._next()]) + payload
        c = crc16(body)
        return body + bytes([c >> 8, c & 0xff])

    def send(self, type_byte: int, payload: bytes = b""):
        self.sock.sendto(self._pkt(type_byte, payload), (self.host, UDP_PORT))

    def try_connect(self, ep: int, wait: float = 0.1, version: int = 2):
        """One attempt like hb_rf_eth_try_connect(): fresh socket, CONNECT,
        answer within `wait` seconds. Returns the endpoint id (V2), 0 (V1)
        or None."""
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("0.0.0.0", 0))
        out = self._pkt(T_CONNECT, bytes([2, ep]) if version == 2 else bytes([1]))
        mirror = out[1]
        s.sendto(out, (self.host, UDP_PORT))
        deadline = time.monotonic() + wait
        while True:
            left = deadline - time.monotonic()
            if left <= 0 or not select.select([s], [], [], left)[0]:
                break
            data = s.recv(1500)
            p = parse(data)
            ok_v2 = version == 2 and len(p[1]) == 3 if p else False
            ok_v1 = version == 1 and len(p[1]) == 2 if p else False
            if p and p[0] == T_CONNECT and (ok_v1 or ok_v2) and p[1][0] == version and p[1][1] == mirror:
                if self.sock:
                    self.old_socks.append(self.sock)
                self.sock = s
                self.ep = p[1][2] if version == 2 else 0
                return self.ep
        # The kernel releases the socket; keep it open to see late answers.
        self.old_socks.append(s)
        return None

    def resume(self, max_attempts: int = 40, pause: float = 0.5):
        """Reconnect loop of hb_rf_eth_recv_threadproc(). Attempts used or None."""
        for n in range(1, max_attempts + 1):
            if self.try_connect(self.ep) is not None:
                return n
            time.sleep(pause)
        return None

    def pump(self, seconds: float, keepalive: bool = True):
        """Run `seconds`: keepalive every second, collect what arrives.
        Returns ([(t, type, payload)] on the current socket, {sock: count} on older ones)."""
        got, stray = [], {}
        t_end = time.monotonic() + seconds
        while True:
            now = time.monotonic()
            if now >= t_end:
                return got, stray
            if keepalive and self.sock and now - self.last_ka_out >= 1.0:
                self.send(T_KEEPALIVE)
                self.last_ka_out = now
            socks = ([self.sock] if self.sock else []) + self.old_socks
            for s in select.select(socks, [], [], min(0.1, t_end - now))[0]:
                p = parse(s.recv(1500))
                if not p:
                    continue
                if s is self.sock:
                    got.append((time.monotonic(), p[0], p[1]))
                else:
                    stray[s] = stray.get(s, 0) + 1

    def os_get_app(self, timeout: float = 1.5):
        """HMUARTLGW OS_GET_APP in a type-7 frame; the answer frame or None."""
        self.hmu_cnt = (self.hmu_cnt + 1) & 0xff
        cnt = self.hmu_cnt
        self.send(T_FRAME, hmu_frame(HMU_DST_OS, cnt, bytes([OS_GET_APP])))
        got, _ = self.pump(timeout)
        for _, t, payload in got:
            h = hmu_parse(payload) if t == T_FRAME else None
            if h and h[0] == HMU_DST_OS and h[1] == cnt:
                return h
        return None


def console(cmds, wait: float = 0.8) -> str:
    with socket.create_connection((CULFW32_HOST, CULFW32_PORT), timeout=3) as c:
        out = b""
        for cmd in cmds:
            c.sendall((cmd + "\n").encode())
            t_end = time.monotonic() + wait
            while time.monotonic() < t_end:
                if select.select([c], [], [], 0.1)[0]:
                    chunk = c.recv(4096)
                    if not chunk:
                        break
                    out += chunk
        return out.decode(errors="replace")


def counters(tries: int = 20):
    """(boot_count, crash_count) from the culfw console; retries while the stick restarts."""
    for _ in range(tries):
        try:
            txt = console(["?sys.boot_count", "?sys.crash_count"])
            b = re.search(r"sys\.boot_count\s*=\s*(\d+)", txt)
            c = re.search(r"sys\.crash_count\s*=\s*(\d+)", txt)
            if b and c:
                return int(b.group(1)), int(c.group(1))
        except OSError:
            pass
        time.sleep(1.5)
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--reboot", action="store_true",
                    help="also restart the stick (B00) and test the resume after it")
    ap.add_argument("--storm", type=int, default=20, help="resumes in a row (default 20)")
    a = ap.parse_args()

    results = []

    def check(name, ok, detail=""):
        results.append(ok)
        print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}".rstrip(), flush=True)
        return ok

    before = counters()
    print(f"stick {CULFW32_HOST}: boot_count/crash_count before = {before}", flush=True)
    k = Kernel(CULFW32_HOST)

    # session -----------------------------------------------------------
    ep = None
    for n in range(1, 11):
        ep = k.try_connect(0)
        if ep is not None:
            break
        time.sleep(0.5)
    if not check("session: CONNECT ep 0 answered", ep is not None, f"ep={ep} attempts={n}"):
        return 1
    k.send(T_STARTCONN)
    k.pump(0.3)
    check("session: OS_GET_APP answered after StartConn", k.os_get_app() is not None)

    # resume ------------------------------------------------------------
    k.pump(2.0)
    old = k.sock
    t0 = time.monotonic()
    got, _ = k.pump(7.0, keepalive=False)
    late_ka = [t - t0 for t, typ, _ in got if typ == T_KEEPALIVE and t - t0 > 6.2]
    check("resume: stick stops keepalives after its 5 s timeout", not late_ka,
          f"late keepalives at {['%.1f s' % x for x in late_ka]}" if late_ka else "")
    n = k.resume()
    if check("resume: CONNECT with the stored ep answered", n is not None,
             f"ep={k.ep} attempts={n}"):
        check("resume: OS_GET_APP answered without new StartConn", k.os_get_app() is not None)
        got, stray = k.pump(2.5)
        ka = sum(1 for _, typ, _ in got if typ == T_KEEPALIVE)
        check("resume: keepalives at the new port", ka >= 2, f"{ka} in 2.5 s")
        check("resume: nothing at the old port", stray.get(old, 0) == 0,
              f"{stray.get(old, 0)} packets")

        # storm ---------------------------------------------------------
        # 300 ms per attempt: this case is about the slots, not the timing.
        ok = sum(1 for _ in range(a.storm) if k.try_connect(k.ep, wait=0.3) is not None)
        check(f"storm: {a.storm} resumes from new ports answered", ok == a.storm, f"{ok}/{a.storm}")
        check("storm: OS_GET_APP answered afterwards", k.os_get_app() is not None)

        # stopconn ------------------------------------------------------
        k.send(T_STOPCONN)
        k.pump(0.3)
        check("stopconn: OS_GET_APP not answered after StopConn", k.os_get_app(timeout=1.0) is None)
        k.pump(7.0, keepalive=False)
        n = k.resume()
        if check("stopconn: resume answered", n is not None, f"attempts={n}"):
            check("stopconn: still no answer after the resume", k.os_get_app(timeout=1.0) is None)
            k.send(T_STARTCONN)
            k.pump(0.3)
            check("stopconn: answered after a new StartConn", k.os_get_app() is not None)

        # forget --------------------------------------------------------
        ep_old = k.ep
        k.send(T_DISCONNECT)
        k.pump(0.5, keepalive=False)
        n = k.resume()
        if check("forget: resume with the old ep after DISCONNECT answered",
                 n is not None and k.ep == ep_old, f"attempts={n} ep={ep_old}->{k.ep}"):
            check("forget: OS_GET_APP answered without StartConn", k.os_get_app() is not None)

        # reboot --------------------------------------------------------
        if a.reboot:
            try:
                console(["B00"], wait=0.3)
            except OSError as e:
                print(f"B00 failed: {e}")
            t0 = time.monotonic()
            k.pump(5.5)   # the kernel notices the stick only after 5 s of silence
            n = k.resume(max_attempts=60)
            if check("reboot: resume with the stored ep answered", n is not None,
                     f"{time.monotonic() - t0:.1f} s after B00, attempts={n}, ep={k.ep}"):
                check("reboot: OS_GET_APP answered without StartConn", k.os_get_app() is not None)

        # v1 ------------------------------------------------------------
        r = None
        for n in range(1, 11):
            r = k.try_connect(0, version=1)
            if r is not None:
                break
            time.sleep(0.5)
        if check("v1: CONNECT V1 answered with [1, counter]", r is not None, f"attempts={n}"):
            check("v1: OS_GET_APP not answered before StartConn", k.os_get_app(timeout=1.0) is None)
            k.send(T_STARTCONN)
            k.pump(0.3)
            check("v1: OS_GET_APP answered after StartConn", k.os_get_app() is not None)

        # newsess -------------------------------------------------------
        ep_before = k.ep
        ep = None
        for n in range(1, 11):
            ep = k.try_connect(0)
            if ep is not None:
                break
            time.sleep(0.5)
        check("newsess: CONNECT ep 0 gives a new endpoint id",
              ep is not None and ep not in (0, ep_before), f"{ep_before} -> {ep}")
        if ep is not None:
            check("newsess: OS_GET_APP not answered before StartConn",
                  k.os_get_app(timeout=1.0) is None)
            k.send(T_STARTCONN)
            k.pump(0.3)
            check("newsess: OS_GET_APP answered after StartConn", k.os_get_app() is not None)

    if k.sock:
        k.send(T_DISCONNECT)
    time.sleep(0.5)
    after = counters()
    exp_boot = 1 if a.reboot else 0
    ok = (before is not None and after is not None and after[1] == before[1]
          and after[0] - before[0] == exp_boot)
    check("no restart that was not asked for", ok, f"before={before} after={after}")
    print(f"\n{sum(results)}/{len(results)} checks passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
