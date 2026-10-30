#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""HMUARTLGW Trace-Listener.

Phase-(-1)-Werkzeug fuer das CULFW32-HMUARTLGW-Projekt: bindet auf TCP-Port
2325, akzeptiert eine Verbindung von FHEM (`HMUARTLGW DNS:host:2325`), und
loggt jedes empfangene Byte sowohl roh als auch — wenn es ein gueltiger
0xfd-Frame ist — geparst mit Befehlsname und Payload.

Modi:
  --silent   (default) antwortet NICHTS; FHEM laeuft in Reconnect-Loop, wir
             sehen aber jeden Init-Versuch sauber von Anfang an.
  --stub-ack antwortet auf jeden Frame mit OS_ACK / APP_ACK (status=0x01) plus
             Counter-Mirror; reicht fuer FHEM um die Init-Sequenz weiter zu
             treiben (mit canned firmware-version + HMID).

Logs gehen nach stdout und parallel in trace_<timestamp>.log.
"""

from __future__ import annotations

import argparse
import datetime as dt
import socket
import sys
import time
from pathlib import Path

from frame_parser import (
    APP_CMDS,
    DST_APP,
    DST_OS,
    Frame,
    FrameDecoder,
    OS_CMDS,
)

DEFAULT_PORT = 2325


def hexdump(data: bytes, prefix: str = "") -> str:
    out_lines = []
    for offset in range(0, len(data), 16):
        chunk = data[offset : offset + 16]
        hex_part = " ".join(f"{b:02x}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        out_lines.append(f"{prefix}{offset:04x}  {hex_part:<48}  {ascii_part}")
    return "\n".join(out_lines)


def make_response(req: Frame, fw_pad: bytes, fw_version: bytes, hmid: bytes, serial: bytes) -> bytes:
    """Baut eine FHEM-byte-genaue Antwort fuer jeden Request-Frame.

    Antwort-Formate aus FHEM 00_HMUARTLGW.pm rekonstruiert:
    - OS_GET_APP   -> payload = [0x00] + "Co_CPU_App"          (regex ^00..)
    - OS_GET_FW    -> payload = [0x04, 0x02] + 4 fw bytes       (ACK_INFO)
    - OS_GET_SER   -> payload = [0x04, 0x02] + 10 ASCII serial  (ACK_INFO)
    - OS_GET_CREDS -> payload = [0x04, 0x02, credit]            (ACK_INFO)
    - APP_GET_HMID -> payload = [0x04, 0x07] + 3 hmid bytes     (ACK_WITH_MULTIPART_DATA)
    - APP_DEF_HMID -> payload = [0x04, 0x07] + 3 hmid bytes     (ACK_WITH_MULTIPART_DATA)
    - alle anderen -> payload = [0x04, 0x01]                    (plain ACK)
    """
    cmd = req.cmd_byte()
    ACK = 0x04
    ACK_OK = 0x01
    ACK_INFO = 0x02
    ACK_MULTIPART = 0x07

    if req.dst == DST_OS:
        if cmd == 0x00:  # OS_GET_APP
            payload = bytes([0x00]) + b"Co_CPU_App"
        elif cmd == 0x02:  # OS_GET_FIRMWARE
            # FHEM liest substr(10,2)/(12,2)/(14,2) = bytes 5/6/7 des payloads
            payload = bytes([ACK, ACK_INFO]) + fw_pad + fw_version
        elif cmd == 0x0B:  # OS_GET_SERIAL
            payload = bytes([ACK, ACK_INFO]) + serial
        elif cmd == 0x08:  # OS_GET_CREDITS
            # Byte ist Load% (0=idle, 100=fully loaded, >100 → ERROR-Overload).
            payload = bytes([ACK, ACK_INFO, 0x00])
        else:
            payload = bytes([ACK, ACK_OK])
        return Frame(dst=DST_OS, cnt=req.cnt, payload=payload).encode()

    if req.dst == DST_APP:
        if cmd == 0x01:  # APP_GET_HMID
            payload = bytes([ACK, ACK_MULTIPART]) + hmid
        elif cmd == 0x10:  # APP_DEFAULT_HMID
            payload = bytes([ACK, ACK_MULTIPART]) + hmid
        else:
            payload = bytes([ACK, ACK_OK])
        return Frame(dst=DST_APP, cnt=req.cnt, payload=payload).encode()

    return b""


def serve(port: int, mode: str, log_path: Path) -> None:
    # FHEM liest Firmware aus byte-offsets 5/6/7 des $msg-Strings (chars 10/12/14
    # nach "04 02"). Die ersten 3 Daten-Bytes nach ACK_INFO sind Padding/Info-
    # Felder die wir mit 0 fuellen; danach kommt M/m/p der Versionsnummer.
    fw_pad = bytes([0x00, 0x00, 0x00])
    fw_version = bytes([0x01, 0x04, 0x01])  # "1.4.1" → nicht outdated
    hmid = bytes([0xCA, 0xFE, 0x42])
    serial = b"CULFW32001"  # 10 ASCII

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)

    log_fp = log_path.open("a", buffering=1)

    def log(msg: str) -> None:
        ts = dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"{ts}  {msg}"
        print(line, flush=True)
        log_fp.write(line + "\n")

    log(f"--- listener startet auf :{port}, mode={mode}, log={log_path}")

    while True:
        log(f"warte auf Verbindung ...")
        conn, addr = srv.accept()
        log(f"=== verbunden von {addr[0]}:{addr[1]}")
        decoder = FrameDecoder()
        try:
            while True:
                data = conn.recv(4096)
                if not data:
                    log("=== Peer hat zu (EOF)")
                    break
                log(f"<- RAW {len(data)} bytes\n" + hexdump(data, prefix="    "))
                decoder.feed(data)
                for frame in decoder.frames():
                    log(f"<- FRAME  {frame.pretty()}")
                    if mode == "stub-ack":
                        resp = make_response(frame, fw_pad, fw_version, hmid, serial)
                        if resp:
                            conn.sendall(resp)
                            log(f"-> RESP {len(resp)} bytes\n" + hexdump(resp, prefix="    "))
        except (ConnectionResetError, BrokenPipeError) as exc:
            log(f"=== connection error: {exc}")
        finally:
            conn.close()
            log(f"=== Verbindung zu {addr[0]} geschlossen")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--mode", choices=["silent", "stub-ack"], default="silent")
    p.add_argument("--log-dir", default=str(Path(__file__).parent / "traces"))
    args = p.parse_args()

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"trace_{args.mode}_{stamp}.log"

    try:
        serve(args.port, args.mode, log_path)
    except KeyboardInterrupt:
        print("\n--- abgebrochen", file=sys.stderr)


if __name__ == "__main__":
    main()
