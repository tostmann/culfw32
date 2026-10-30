# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
"""Lab-spezifische Defaults aus Environment-Variablen.

Test-Scripts importieren diese Konstanten anstelle hartzucodieren. Damit kann
jeder mit `export CULFW32_DEV=/dev/ttyACM0` etc. seine lokale Hardware
adressieren, ohne Patches an den Skripten.

Variablen (alle optional, mit Defaults):

    CULFW32_DEV       USB-Seriell-Pfad zum CULFW32-Board
                      (Default: /dev/ttyACM0)

    CULFW32_HOST      IP/Hostname des CULFW32-Boards im LAN (TCP-Tests)
                      (Default: 192.168.1.42 — Platzhalter, anpassen)

    CUL868_DEV        USB-Seriell-Pfad zu einem busware-CUL868
                      (Default: /dev/serial/by-id/usb-busware.de_CUL868-if00)

    CUL868_REMOTE_HOST IP/Hostname eines weiteren CUL868 fuer Distanz-RX-Tests
                      (Default: 192.0.2.150 — RFC-5737 Doku-Range, anpassen)

    CUL868_REMOTE_PORT TCP-Port des Remote-CUL868 (Default: 2323)

    LAB_HOST          IPv4 des Hosts, der die Test-Skripte laufen laesst
                      (relevant z.B. fuer Allowlist-Tests die "diese Maschine"
                      whitelisten muessen). Default: 192.0.2.10

    LAB_SUBNET        IPv4/CIDR des Lab-Subnets (Allowlist-Subnet-Tests)
                      (Default: 192.0.2.0/24)

Beispiel:

    export CULFW32_DEV=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_..-if00
    export CULFW32_HOST=192.168.1.42
    python3 firmware/tools/release_smoke_test.py
"""

from __future__ import annotations

import os


CULFW32_DEV         = os.environ.get("CULFW32_DEV",         "/dev/ttyACM0")
CULFW32_HOST        = os.environ.get("CULFW32_HOST",        "192.168.1.42")
CULFW32_PORT        = int(os.environ.get("CULFW32_PORT",    "2323"))

CUL868_DEV          = os.environ.get("CUL868_DEV",          "/dev/serial/by-id/usb-busware.de_CUL868-if00")
CUL868_REMOTE_HOST  = os.environ.get("CUL868_REMOTE_HOST",  "192.0.2.150")
CUL868_REMOTE_PORT  = int(os.environ.get("CUL868_REMOTE_PORT", "2323"))

LAB_HOST            = os.environ.get("LAB_HOST",            "192.0.2.10")
LAB_SUBNET          = os.environ.get("LAB_SUBNET",          "192.0.2.0/24")
