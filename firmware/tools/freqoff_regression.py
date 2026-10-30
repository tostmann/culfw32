#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""Prueft, ob der Board-Frequenzoffset (FSCTRL0) JEDE Protokoll-CFG erreicht.

Warum es diesen Test gibt
-------------------------
Der kalibrierte Ziehfehler des Boards liegt im Treiber und wird beim Laden
einer CFG-Tabelle ueber deren FSCTRL0-Byte gelegt (`write_reg`-Hook).  Dieser
Hook kann nur greifen, wenn die Tabelle das Register ueberhaupt anfasst — und
die aus a-culfw uebernommenen Tabellen von AskSin, Moritz und native tun das
nicht (a-culfw kennt keinen Boardoffset).  Bis Build 0.1.813 verpuffte die
Kalibrierung deshalb in genau den FSK-Protokollen, fuer die sie gebaut wurde:
gemessen am 01.09.2026 blieb FSCTRL0 nach `Ar`/`Zr`/`Nr1`/`Pr` auf 0x00, waehrend
`X`/`b`/`f` den Wert trugen.  Seit 0.1.814 zieht `strobe(SRES)` den Wert nach.

Der Test faellt also nicht nur bei einer Regression im Treiber an, sondern auch,
wenn eine NEUE Protokollklasse dazukommt, die den Wert wegschreibt.

Ablauf
------
    1. Aktuellen Offset merken, markanten Testwert setzen (Default -20 = 0xEC;
       kommt in keiner CFG-Tabelle als Default vor).
    2. Jeden Mode laden und Register 0x0C (`C0C`) lesen.
    3. Zur Kontrolle `Vr` mitlesen — aendert sich die CFG-Signatur nicht, hat
       der Mode gar nicht neu geladen und die Zeile ist wertlos.
    4. Originalwert zuruecksetzen.

Aufruf:

    python3 freqoff_regression.py [/dev/serial/by-id/...] [--value -20] [--skip-wmbus]

`--skip-wmbus` laesst `brt`/`brs` aus: solange der bekannte IDLE0-Starvation-Bug
im wmbus-rx-Task offen ist, hinterlaesst ein wMBus-Mode einen Stick, der alle
5 s TWDT-Dumps schreibt — fuer nachfolgende Timing-Messungen unbrauchbar.

Exit 0 = alle Modi tragen den Boardwert, 1 = mindestens einer nicht.
"""
import argparse
import os
import re
import select
import sys
import time
import tty

MODES = [
    ("SlowRF (FS20/OOK)", "X21",  False),
    ("AskSin  (BidCoS)",  "Ar",   False),
    ("Moritz  (MAX!)",    "Zr",   False),
    ("native  IT+ 17k",   "Nr1",  False),
    ("native  PCA301",    "Nr3",  False),
    ("wMBus   T-Mode",    "brt",  True),
    ("wMBus   S-Mode",    "brs",  True),
    ("fastrf  250k GFSK", "fr",   False),
    ("HmIP    (P-Pfad)",  "Pr",   False),
]


class Dut:
    def __init__(self, dev):
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)

    def read(self, t=1.0):
        end = time.time() + t
        b = b""
        while time.time() < end:
            r, _, _ = select.select([self.fd], [], [], 0.2)
            if r:
                b += os.read(self.fd, 8192)
        return [x.strip() for x in b.decode("utf-8", "replace").splitlines() if x.strip()]

    def send(self, cmd, wait=0.4):
        os.write(self.fd, (cmd + "\r\n").encode())
        time.sleep(wait)

    def ask(self, cmd, t=1.0):
        self.read(0.1)
        self.send(cmd, 0.1)
        return self.read(t)

    def close(self):
        os.close(self.fd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("--value", type=int, default=-20,
                    help="Testwert in LSB (Default -20 = 0xEC)")
    ap.add_argument("--skip-wmbus", action="store_true",
                    help="brt/brs auslassen (offener TWDT-Bug im wmbus-rx-Task)")
    a = ap.parse_args()

    want = "%02X" % (a.value & 0xFF)
    d = Dut(a.port)
    # Port-Open ist ein Reset — erst hochlaufen lassen, sonst geht das erste
    # Kommando im Boot-Log unter.
    time.sleep(2.5)
    d.read(1.0)

    before = None
    for l in d.ask("?rf.freqoff", 1.2):
        m = re.search(r"rf\.freqoff = (-?\d+)", l)
        if m:
            before = int(m.group(1))
    if before is None:
        print("FEHLER: ?rf.freqoff nicht lesbar — falscher Port oder alte Firmware?")
        d.close()
        return 2
    print("Offset vorher: %d LSB   Testwert: %d (0x%s)\n" % (before, a.value, want))

    d.send("?rf.freqoff=%d" % a.value, 1.2)
    d.read(0.4)

    print("%-20s %-9s %-9s %s" % ("Mode", "Kommando", "FSCTRL0", "CFG-Signatur"))
    print("-" * 74)
    rows = []
    for name, cmd, is_wmbus in MODES:
        if is_wmbus and a.skip_wmbus:
            print("%-20s %-9s %-9s uebersprungen (--skip-wmbus)" % (name, cmd, "-"))
            continue
        d.read(0.2)
        d.send(cmd, 1.2)
        vr = [l for l in d.ask("Vr", 1.5) if l.startswith("Vr")]
        c0c = [l for l in d.ask("C0C", 1.2) if l.startswith("C0C")]
        reg = c0c[0].split("=")[1].split("/")[0].strip() if c0c else "??"
        sig = ""
        if vr:
            m = re.search(r"(FREQ=\S+).*?(MDM=\S+).*?(DEV=\S+)", vr[0])
            sig = " ".join(m.groups()) if m else vr[0][:46]
        ok = reg.upper() == want
        print("%-20s %-9s %-9s %s  %s" % (name, cmd, reg, sig, "OK" if ok else "FAIL"))
        rows.append((name, cmd, reg, sig, ok))

    d.send("?rf.freqoff=%d" % before, 1.2)
    d.read(0.4)
    print("\nOffset zurueckgesetzt auf %d LSB" % before)

    bad = [r for r in rows if not r[4]]
    sigs = {r[3] for r in rows}
    if len(sigs) < len(rows):
        print("Hinweis: %d Modi teilen sich %d CFG-Signaturen — AskSin/HmIP sind "
              "PHY-identisch, das allein ist kein Fehler." % (len(rows), len(sigs)))
    if bad:
        print("\nFAIL: Boardoffset fehlt in %s"
              % ", ".join("%s (0x%s)" % (r[0].strip(), r[2]) for r in bad))
    else:
        print("\nPASS: alle %d geprueften Modi tragen den Boardoffset." % len(rows))
    d.close()
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
