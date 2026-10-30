#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""CC1101-Registerrechner (SWRS061I).

Rechnet Traegerfrequenz, Symbolrate, FSK-Hub und RX-Filterbandbreite in die
Registerwerte um und zurueck, damit jede Angabe in einer CFG-Tabelle gegen die
Datenblattformel belegbar ist statt aus SmartRF Studio abgeschrieben.

Formeln (26-MHz-Quarz, SWRS061I):
  f_carrier = (f_XOSC / 2^16) * FREQ                       Abschnitt 21
  R_DATA    = ((256 + DRATE_M) * 2^DRATE_E / 2^28) * f_XOSC  Abschnitt 12
  f_dev     = (f_XOSC / 2^17) * (8 + DEVIATION_M) * 2^DEVIATION_E  Abschnitt 16.1
  BW_chan   = f_XOSC / (8 * (4 + CHANBW_M) * 2^CHANBW_E)   Abschnitt 13

Aufruf:  python3 cc1101_regcalc.py [--freq HZ] [--rate BAUD] [--dev HZ] [--bw HZ]
"""

import argparse

XOSC = 26_000_000


def freq_regs(f_hz):
    """FREQ2/1/0 fuer die gewuenschte Traegerfrequenz."""
    word = round(f_hz / (XOSC / 2**16))
    actual = word * (XOSC / 2**16)
    return word, (word >> 16) & 0xFF, (word >> 8) & 0xFF, word & 0xFF, actual


def drate_regs(rate_baud):
    """DRATE_E/M mit der kleinsten Abweichung zur Wunsch-Symbolrate."""
    best = None
    for e in range(16):
        m = round(rate_baud * 2**28 / (2**e * XOSC)) - 256
        if not 0 <= m <= 255:
            continue
        actual = (256 + m) * 2**e / 2**28 * XOSC
        err = abs(actual - rate_baud)
        if best is None or err < best[0]:
            best = (err, e, m, actual)
    return best[1], best[2], best[3]


def deviatn_regs(dev_hz):
    """DEVIATION_E/M (je 3 bit) mit der kleinsten Abweichung zum Wunsch-Hub."""
    best = None
    for e in range(8):
        for m in range(8):
            actual = XOSC / 2**17 * (8 + m) * 2**e
            err = abs(actual - dev_hz)
            if best is None or err < best[0]:
                best = (err, e, m, actual)
    _, e, m, actual = best
    return e, m, (e << 4) | m, actual


def chanbw_regs(bw_hz):
    """CHANBW_E/M — kleinste Bandbreite, die den Wunschwert noch erreicht."""
    best = None
    for e in range(4):
        for m in range(4):
            actual = XOSC / (8 * (4 + m) * 2**e)
            err = abs(actual - bw_hz)
            if best is None or err < best[0]:
                best = (err, e, m, actual)
    _, e, m, actual = best
    return e, m, actual


def sync_autocorr_worst(word16):
    """Groesster Nebenmaximum-Betrag der 32-bit-Folge SYNC1:SYNC0:SYNC1:SYNC0.

    Der CC1101 vergleicht im Modus 30/32 gegen das zweifach wiederholte
    16-bit-Wort. Ein Sync mit hohen Nebenmaxima laesst sich von verschobenen
    Kopien seiner selbst nicht unterscheiden -> Fehlsynchronisation.
    """
    bits = [(word16 >> (15 - i)) & 1 for i in range(16)] * 2
    pm = [1 if b else -1 for b in bits]
    n = len(pm)
    worst = 0
    for shift in range(1, n):
        # Verschiebungen um ein Vielfaches von 16 korrelieren bei einem
        # verdoppelten Wort zwangslaeufig voll — das sagt nichts ueber die
        # Guete des Wortes aus und wuerde jede Bewertung einebnen.
        if shift % 16 == 0:
            continue
        acc = sum(pm[i] * pm[i + shift] for i in range(n - shift))
        worst = max(worst, abs(acc))
    return worst


def pick_sync(exclude):
    """Sync-Wort mit dem kleinsten Nebenmaximum, ausgenommen belegte Woerter."""
    cands = []
    for w in range(0x0100, 0x10000):
        if w in exclude:
            continue
        b = bin(w).count("1")
        if not 6 <= b <= 10:          # ausgewogen, damit kein DC-Versatz entsteht
            continue
        if (w & 0xFF) == (w >> 8):    # halbperiodisch -> schlechte Autokorrelation
            continue
        cands.append((sync_autocorr_worst(w), w))
    cands.sort()
    return cands[:8]


def airtime(payload_bytes, rate_baud, per_pkt_payload, overhead_bytes=10):
    """Reine Sendezeit fuer eine Nutzlast, in Sekunden.

    overhead_bytes = Praeambel(4) + Sync(4) + Laenge(1) + CRC(2) minus Rundung;
    die Gegenrichtung (ACK) und die Umschaltzeiten sind NICHT enthalten.
    """
    pkts = -(-payload_bytes // per_pkt_payload)
    total_bytes = payload_bytes + pkts * overhead_bytes
    return total_bytes * 8 / rate_baud, pkts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--freq", type=float, default=868.5e6)
    ap.add_argument("--rate", type=float, default=100e3)
    ap.add_argument("--dev", type=float, default=50e3)
    ap.add_argument("--bw", type=float, default=325e3)
    ap.add_argument("--sync", action="store_true", help="Sync-Wort-Kandidaten listen")
    a = ap.parse_args()

    word, f2, f1, f0, f_act = freq_regs(a.freq)
    de, dm, r_act = drate_regs(a.rate)
    ve, vm, deviatn, d_act = deviatn_regs(a.dev)
    be, bm, bw_act = chanbw_regs(a.bw)
    mdmcfg4 = (be << 6) | (bm << 4) | de

    print(f"XOSC {XOSC/1e6:.0f} MHz")
    print(f"FREQ    = 0x{word:06X} -> FREQ2=0x{f2:02X} FREQ1=0x{f1:02X} FREQ0=0x{f0:02X}"
          f"   ist {f_act/1e6:.6f} MHz  (Soll {a.freq/1e6:.6f}, "
          f"Fehler {f_act-a.freq:+.0f} Hz)")
    print(f"DRATE   = E{de} M{dm}  -> MDMCFG3=0x{dm:02X}"
          f"   ist {r_act/1e3:.3f} kBaud (Soll {a.rate/1e3:.1f}, "
          f"Fehler {(r_act-a.rate)/a.rate*100:+.2f} %)")
    print(f"DEVIATN = E{ve} M{vm}  -> 0x{deviatn:02X}"
          f"   ist {d_act/1e3:.2f} kHz   (Soll {a.dev/1e3:.1f})")
    print(f"CHANBW  = E{be} M{bm}  -> MDMCFG4=0x{mdmcfg4:02X}"
          f"   ist {bw_act/1e3:.2f} kHz  (Soll {a.bw/1e3:.1f})")
    print()
    mod_index = 2 * d_act / r_act
    carson = 2 * d_act + r_act
    print(f"Modulationsindex 2*dev/rate = {mod_index:.2f}")
    print(f"Belegte Bandbreite (Carson, 2*dev+rate) = {carson/1e3:.0f} kHz")
    print(f"Offset-Reserve im RX-Filter = (BW - Carson)/2 = "
          f"{(bw_act-carson)/2/1e3:.0f} kHz je Seite")
    print()
    for name, size in (("RWE PSS (328P)", 28 * 1024), ("SCC (1284P)", 124 * 1024)):
        t, n = airtime(size, r_act, 55)
        print(f"{name:16s} {size:6d} B -> {n:4d} Pakete, "
              f"{t:5.2f} s reine Sendezeit ({t/36*100:.0f} % vom 1-%-Stundenbudget)")

    if a.sync:
        print("\nSync-Kandidaten (Nebenmaximum der 30/32-Folge, kleiner ist besser):")
        belegt = {0xE9CA, 0xC626, 0xD391, 0x2904, 0x0503, 0x9148, 0x5A5A, 0xAAAA}
        for score, w in pick_sync(belegt):
            print(f"  0x{w:04X}  worst |R| = {score}")


if __name__ == "__main__":
    main()
