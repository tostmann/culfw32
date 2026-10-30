#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""CULFW32-Protokollsuite gegen einen entfernten CUL868 (a-culfw) ueber TCP.

Prueft je Protokoll beide Richtungen. Der Pruefling haengt seriell am Host,
die Gegenstelle ueber die socat-Bruecke.  Kein lokaler CUL noetig.

Fallen, die hier beruecksichtigt sind:
  * Vor FSK-Tests am Remote `X00` — solange tx_report (X21) gesetzt ist, faellt
    a-culfw in den SlowRF-Empfang zurueck und Ar/Zr wirken scheinbar nicht.
  * Ein Mode-Wechsel am Remote braucht ZEIT. Mit 1 s Wartezeit kam der erste
    FS20-Block als 0/5 zurueck, mit Vorlauf 8/8 — der Stick war schlicht noch
    nicht im SlowRF-Empfang. Deshalb settle() nach jedem Moduswechsel.
  * Moritz `Zs` sendet in a-culfw IMMER mit LongPreamble (rf_moritz.c:252) und
    kostet damit `100 + (hblen*8)/10` = 109 Kredit-Einheiten pro Frame. Nach
    ~8 Frames am Stueck ist das 900er-Budget leer und der Rest wird mit LOVF
    verworfen — korrektes Verhalten, kein Fehler. Deshalb vor dem Moritz-Block
    den Kredit lesen und die Frame-Zahl daran ausrichten.
  * `As`/`Zs` mit falscher Laenge verwirft a-culfw STILL (LENERR ist dort
    auskommentiert) — deshalb feste 12-Byte-Frames (len 0x0B + 11).
  * Sendekredit: culfw drosselt nach ~450 10ms-Einheiten (LOVF). Zwischen den
    Bloecken Pause, Nutzlast variieren (Duplikatfilter REP_KNOWN, 304 ms).
"""
import os, re, socket, sys, time, tty, select

DEV = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
REF = (os.environ.get("CUL868_REMOTE_HOST", "192.0.2.150"), 2323)  # wie _lab_env.py
results = []

fd = os.open(DEV, os.O_RDWR | os.O_NOCTTY); tty.setraw(fd)
rm = socket.create_connection(REF, 5); rm.setblocking(False)

def rd(t):
    end=time.time()+t; b=b""
    while time.time()<end:
        r,_,_=select.select([fd],[],[],0.2)
        if r: b+=os.read(fd,8192)
    return [x.strip() for x in b.decode("utf-8","replace").splitlines() if x.strip()]
def rs(t):
    end=time.time()+t; b=b""
    while time.time()<end:
        try: b+=rm.recv(8192)
        except Exception: time.sleep(0.03)
    return [x.strip() for x in b.decode("utf-8","replace").splitlines() if x.strip()]
def dut(cmd, wait=0.5):
    os.write(fd,(cmd+"\r\n").encode()); time.sleep(wait)
def ref(cmd, wait=0.5):
    rm.send((cmd+"\r\n").encode()); time.sleep(wait)
def settle(sec=2.0):
    """Nach einem Moduswechsel: warten UND die Puffer beider Seiten leeren."""
    time.sleep(sec); rd(0.4); rs(0.4)
def credit():
    dut("?stats", 0.8)
    for l in rd(1.5):
        m=re.match(r"credit (\d+)/900", l)
        if m: return int(m.group(1))
    return -1
def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("  PASS  " if ok else "  FAIL  ")+name+("   "+detail if detail else ""), flush=True)

print("== Identitaet ==")
# Das Oeffnen des Ports IST ein Reset: das erste Kommando geht sonst im
# Boot-Log unter und `V` faellt grundlos rot aus (beobachtet 01.09.2026).
# Deshalb warten, Puffer leeren und `V` bis zu dreimal fragen.
time.sleep(2.5); rd(1.5); rs(0.5)
v = []
for _ in range(3):
    dut("V", 0.8)
    v = [l for l in rd(1.2) if l.startswith("V ")]
    if v: break
dut("VH",0.8);  vh = [l for l in rd(1.2) if l.startswith("CULFW32")]
dut("?rf.freqoff",0.8); fo=[l for l in rd(1.2) if l.startswith("? rf.freqoff ")]
check("V", bool(v), v[0] if v else "")
check("VH", bool(vh), vh[0] if vh else "")
check("rf.freqoff gesetzt", bool(fo), fo[0].split("(")[0].strip() if fo else "")

print("== SlowRF / FS20 (OOK) ==")
ref("X21"); dut("X21"); settle(2.5)
n=0
for i in range(5):
    dut("F1A2B3C%02X" % (0x10+i), 0.8)
got=[l for l in rs(3.0) if l.startswith("F1A2B3C")]
check("FS20 TX  DUT -> Remote", len(got)>=4, "%d/5" % len(got))
rd(0.5)
for i in range(5):
    ref("F4D5E6F%02X" % (0x20+i), 0.8)
got=[l for l in rd(3.0) if l.startswith("F4D5E6F")]
rssi=sorted({int(l[-2:],16) for l in got}) if got else []
check("FS20 RX  Remote -> DUT", len(got)>=4,
      "%d/5  RSSI raw %s" % (len(got), rssi[:3]))

print("== AskSin / BidCoS (FSK) ==")
ref("X00", 1.0); rs(0.5)
dut("Ar"); ref("Ar"); settle(2.5)
for i in range(5):
    dut("As0B%02X0000ABCD01ABCD020000" % (0x30+i), 0.8)
got=[l for l in rs(3.0) if "ABCD01" in l.upper()]
check("AskSin TX  DUT -> Remote", len(got)>=4, "%d/5" % len(got))
rd(0.5)
for i in range(5):
    ref("As0B%02X0000ABCD03ABCD040000" % (0x40+i), 0.8)
got=[l for l in rd(3.0) if "ABCD03" in l.upper()]
check("AskSin RX  Remote -> DUT", len(got)>=4,
      "%d/5  %s" % (len(got), got[0] if got else ""))
dut("?rf.freqest",0.8); fe=[l for l in rd(1.2) if l.startswith("? rf.freqest")]
check("Restoffset ~0 nach Kalibrierung",
      bool(fe) and abs(int(fe[0].split("=")[1].strip().split()[0]))<=2,
      fe[0].split("(")[0].strip() if fe else "")

print("== Moritz / MAX! (FSK) ==")
dut("Zr"); ref("Zr"); settle(2.5)
# LongPreamble: 109 Einheiten je Frame. Nur so viele senden, wie das Budget traegt.
c=credit(); n_moritz=min(5, max(0, (c-20)//109 if c>0 else 0))
print("     Kredit %d -> %d Moritz-Frames" % (c, n_moritz))
if n_moritz == 0:
    # Kredit leer (LongPreamble kostet 109 je Frame, Regeneration +1/s). Das ist
    # kein Fehlschlag des Boards — nur nichts messbar. ~2 min warten hilft.
    print("  SKIP  Moritz TX  DUT -> Remote   Sendekredit erschoepft (%d/900)" % c)
else:
    for i in range(n_moritz):
        dut("Zs0B%02X0402ABCD01ABCD020000" % (0x50+i), 2.5)
    got=[l for l in rs(3.0) if "ABCD01" in l.upper()]
    check("Moritz TX  DUT -> Remote", len(got)>=n_moritz, "%d/%d" % (len(got), n_moritz))
rd(0.5)
for i in range(5):
    ref("Zs0B%02X0402ABCD03ABCD040000" % (0x60+i), 1.2)
got=[l for l in rd(3.0) if "ABCD03" in l.upper()]
# Auch die Gegenstelle hat ein Kreditkonto — 5 LongPreamble-Frames sind dort
# 545 Einheiten. Deshalb hier 3 als Schwelle, nicht 4.
check("Moritz RX  Remote -> DUT", len(got)>=3,
      "%d/5  %s" % (len(got), got[0] if got else ""))

print("== Telemetrie ==")
dut("?stats", 1.0)
st=[l for l in rd(2.0) if l.startswith(("credit","asksin","moritz","slowrf","cc0","heap"))]
for l in st[:8]: print("     " + l)
lovf = [l for l in st if l.startswith("credit")]
# LOVF zaehlt kumulativ ueber die Laufzeit — nach mehreren Moritz-Bloecken ist
# ein Zaehlerstand > 0 normal und kein Defekt. Gemeldet wird nur der Stand.
print("     LOVF-Zaehler (kumulativ, kein Fehlerkriterium): %s"
      % (lovf[0] if lovf else "-"))

# Ausgangszustand
ref("X21", 0.6); dut("Ar", 0.6)
rm.close(); os.close(fd)
ok = sum(1 for _,o,_ in results if o)
print("\n== %d/%d PASS ==" % (ok, len(results)))
for n_,o,d in results:
    if not o: print("   offen: %s  %s" % (n_, d))
sys.exit(0 if ok == len(results) else 1)
