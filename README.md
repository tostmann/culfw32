# CULFW32

**ESP32-C6 + CC1101 — der Produktnachfolger der `busware`-CUL-Familie.**

Volles `culfw`-Funktionsspektrum auf modernem Frontend: WiFi, Matter/Thread-fähiger SoC, USB-C, Web-UI, OTA, Multi-Frontend (FHEM/Homegear/debmatic/SIGNALduino-kompatibel).

> Status: läuft auf Hardware (ESP32-C6 Devboard + CC1101 868 MHz). v1-Funktionsumfang DONE — siehe Feature-Matrix unten. Erste Hardware-Spin (`CULFW32-PCB`) parallel in Vorbereitung.

---

## Was kann es?

Sub-GHz-Funk-Gateway für Smart-Home-Stacks wie FHEM, Homegear, debmatic, RaspberryMatic, Home-Assistant. CULFW32 spricht die Funk-Protokolle der `busware`-CUL-Familie und exponiert dieselben Daten gleichzeitig auf mehreren Wegen: USB-CDC, TCP-ASCII (FHEM/`socat`), TCP-JSON (Web-UI/HASS), HMUARTLGW (FHEM-`uart://` + debmatic per `hb-rf-eth`), Web-UI im Browser.

### Funk-Protokolle (alle 868 MHz / CC1101)

| Cmd | Protokoll | Status |
|---|---|---|
| `A` | AskSin (HomeMatic / BidCos) — RX/TX, Crypt, CCA, BURST | DONE |
| `Z` | Moritz (eq-3 MAX!) — RX/TX, AutoAck | DONE |
| `F K M T` | SlowRF (FS20 / KS300 / HMS / FHT / EM, OOK) | DONE — FHT-Sub-Features partiell |
| `N` | rf_native (IT+ / PCA301) | DONE |
| `f` | fastrf (CC1101-P2P 250 kbps GFSK) | DONE |
| `b` | wMBus T1 / S1 (Wireless M-Bus) | DONE — Real-RF Cross-Check pre-release |
| signalduino | SIGNALduino-kompatibles Frontend (TCP 2326) | DONE |

> 433-MHz-OOK-Familie (`i k Y L h G`) ist auf v1 ausgeschlossen — die aktuelle Antennenanpassung ist 868 MHz; eine 433-MHz-SKU ist Follow-on.

### Frontends

| Pfad | Port / Transport | Use-Case |
|---|---|---|
| ASCII (`culfw`) | USB-CDC, UART, TCP `2323` | FHEM `CUL`-Modul, `socat`, manuelle Nutzung |
| JSON | TCP `2324`, WebSocket `/ws/json` | Web-UI, Home-Assistant, eigene Tools |
| Web-UI | HTTP `80` | Konfiguration & Live-Terminal im Browser |
| HMUARTLGW (FHEM) | TCP `2325` | FHEM `HMUARTLGW`-Modul mit `uart://`-Prefix |
| HMUARTLGW (DualCoPro) | TCP `2327` + UDP `3008` | debmatic / RaspberryMatic via `hb-rf-eth` |
| SIGNALduino | TCP `2326` | FHEM `SIGNALduino`-Modul |
| Improv-Serial | USB-CDC | erstmaliges WiFi-Provisioning via Browser (install.busware.de) |

Mehrere Frontends laufen **gleichzeitig** auf demselben Stick — z.B. FHEM auf 2323, debmatic auf 2327, Browser auf 80.

### Hardware-Features

- **ESP32-C6**: WiFi-6 (2.4 GHz), Bluetooth-LE, Matter / Thread / Zigbee-fähig (Funk-Stacks für späteren Ausbau)
- **CC1101**: 868 MHz Sub-GHz-Transceiver, FSK + OOK
- **USB-C**: Power, Konsole (USB-Serial-JTAG), parallel UART0 auf SBU-Pads
- **Flash 4 MB**: dual OTA-Slots (1.75 MB ota_0 + 1.75 MB ota_1) + 384 KB SPIFFS
- **OTA-Update**: drahtlos via Web-UI oder JSON-Frontend

---

## Build

Gebaut und geprüft mit **ESP-IDF 5.5.3**.

```bash
. $IDF_PATH/export.sh            # ESP-IDF 5.5.3
cd firmware/
idf.py build                     # Ziel und Konfiguration aus sdkconfig.defaults
idf.py -p <PORT> flash monitor
```

`firmware/sdkconfig.defaults` enthält die vollständige Konfiguration der ausgelieferten Firmware. Ein Build daraus ergibt dasselbe Image wie die veröffentlichte Version, bis auf das Kompilierdatum in der `V`-Zeile. Beim ersten Build lädt der Espressif-Component-Manager `improv-wifi-busware` (auf einen Commit gepinnt, `firmware/main/idf_component.yml`) und `espressif/mdns` (Stand in `firmware/dependencies.lock`). Wie man gegen eine lokale Kopie von `improv-wifi-busware` baut, steht als Kommentar in `firmware/main/idf_component.yml`.

### Factory-Image

So entsteht das Image, das der Web-Flasher auf install.busware.de ab Adresse 0 schreibt (an 0.1.912 byte-genau nachgeprüft):

```bash
cd firmware/build
python -m esptool --chip esp32c6 merge_bin -o culfw32.factory.bin \
  --flash_mode dio --flash_freq 80m --flash_size 4MB \
  0x0 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin \
  0x10000 ota_data_initial.bin 0x20000 culfw32.bin
```

Das Online-Update lädt nur das App-Image `culfw32.bin`.

### Build-Toggles

In `idf.py menuconfig` → "CULFW32 Configuration" lassen sich einzelne Funk-Klassen (AskSin / Moritz / SlowRF / Native / FastRF / wMBus) und Frontends (HMUARTLGW / SIGNALduino / JSON / Web-UI) einzeln abschalten. Linker-GC entfernt nicht-instantiierte Klassen aus dem Image. Zur Laufzeit listet `?build` die aktiven Module.

### Versionierung

`firmware/version.txt` und `firmware/build_number.txt` ergeben die Version in der `V`-Zeile und im App-Header (`esp_app_desc_t::version`), den das Online-Update auswertet. Veröffentlicht werden Release-Stände mit dem Tag `v<Version>`, zum Beispiel `v0.2.0`; der Link „Source“ in der Web-UI zeigt auf genau diesen Tag.

---

## Erstinbetriebnahme

1. **Flash & Boot.** USB-C anschließen, `idf.py flash monitor`.
2. **WiFi-Provisioning** — drei Wege:
   - **Improv-Serial**: Browser auf `install.busware.de` öffnen (Web-Serial-API), CULFW32 wählen, SSID/PSK eingeben.
   - **AP-Fallback**: `?wifi.mode=2` setzt CULFW32 in AP-Mode (`CULFW32-XXXX` SSID), dann via Web-UI Credentials setzen.
   - **ASCII**: über USB-CDC `EWs<SSID>` und `EWp<password>`.
3. **Web-UI** öffnen: `http://<culfw32-ip>` zeigt Config-Tab + Live-Terminal.
4. Optional: FHEM/debmatic anbinden — siehe `docs/HmuartlgwHowTo.md` und `docs/DebmaticHowTo.md`.

---

## Dokumentation

| Datei | Inhalt |
|---|---|
| [docs/NetworkHowTo.md](docs/NetworkHowTo.md) | WiFi-Setup, TCP-Server, Allowlist, EW-Befehle |
| [docs/HmuartlgwHowTo.md](docs/HmuartlgwHowTo.md) | FHEM `HMUARTLGW`-Anbindung, HM-Pair-Workflow |
| [docs/DebmaticHowTo.md](docs/DebmaticHowTo.md) | debmatic / RaspberryMatic über `hb-rf-eth`-Tunnel |

Test-Skripte unter `firmware/tools/` decken die Protokoll-Klassen byte-genau gegen ein Lokales/Remote `busware`-CUL868 ab. Konfiguration über Environment-Variablen — siehe `firmware/tools/_lab_env.py`.

---

## Architektur (Kurz-Überblick)

5-Schichten, alle in `firmware/components/`:

```
Transport        usbcdc / uart / tcpserver / websocket / hbrfeth_udp
   ↓
Frontend         culfw (ASCII) / json / web_server / hmuartlgw / signalduino
   ↓
Renderer         protokoll-spezifische ASCII/JSON-Formatter
   ↓
RF-Klasse        rf_proto_asksin / moritz / slowrf / native / fastrf / wmbus / signalduino
   ↓
CC1101-Driver    cc1101_driver + rf_manager (Slot-Routing)
```

Multi-Frontend-Coexistenz: mehrere Frontends abonnieren dieselbe RF-Klasse über den `rf_manager`. Senden serialisiert über `cc1101_slot`; alle Frontends sehen RX-Frames simultan.

---

## Verwandte Projekte

CULFW32 ist Teil einer Familie:

- **[`a-culfw`](https://github.com/tostmann/a-culfw)** — die original AVR/STM32-`culfw`-Codebase, deren Protokoll-Kerne wir vendoren und auf ESP-IDF/FreeRTOS portieren.
- **[`improv-wifi-busware`](https://github.com/tostmann/improv-wifi-busware)** — Improv-Serial-Implementierung für `install.busware.de`-Pfad.

---

## Lizenz

Copyright © 2026 Dirk Tostmann <tostmann@gmail.com>

CULFW32 steht unter der **GNU General Public License, Version 3 oder (nach Wahl) jede spätere Version** (`GPL-3.0-or-later`). Volltext: [COPYING](COPYING).

Einige Dateien enthalten Code aus anderen GPL-Projekten (a-culfw, SIGNALduino). Ihre Herkunfts- und Lizenzvermerke stehen im jeweiligen Dateikopf und in [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md). Wegen des SIGNALduino-Anteils und der Apache-2.0-Komponenten von ESP-IDF steht das Gesamtwerk unter Version 3 (oder später), nicht unter Version 2.

Die Frontends für HMUARTLGW und HB-RF-ETH sind eigenständige Implementierungen der jeweiligen Wire-Protokolle, erstellt nach FHEMs `00_HMUARTLGW.pm`, den piVCCU-Quellen und eigenen Mitschnitten. Übernommen sind Protokollfakten (Opcodes, Rahmenformat, CRC-Parameter).

Beiträge: siehe [CONTRIBUTING.md](CONTRIBUTING.md).

### Credits & Drittlizenzen

| Komponente | Lizenz | Verwendung |
|---|---|---|
| [`a-culfw`](https://github.com/tostmann/a-culfw) (R. Koenig, B. Hempel, D. Tostmann u. a.) | GPL-2.0-or-later | SlowRF-Pulsdecoder, CC1101-Registertabellen, Protokollabläufe und Befehlsformate (Details in den Dateiköpfen) |
| [SIGNALduino](https://github.com/RFD-FHEM/SIGNALDuino) (N. Butzek, S. Butzek, HomeAutoUser u. a.) | GPL-3.0-or-later | SIGNALduino-Decoder und Sendepfad |
| [FHEM `00_HMUARTLGW.pm`](https://svn.fhem.de/trac/browser/trunk/fhem/FHEM/00_HMUARTLGW.pm), piVCCU (A. Reinert) | GPL-2.0 / GPL-2.0-or-later | Protokollreferenz für HMUARTLGW und HB-RF-ETH |
| [Espressif ESP-IDF](https://github.com/espressif/esp-idf) mit FreeRTOS, lwIP, Mbed TLS, cJSON, SPIFFS und den Espressif-Binärbibliotheken | Apache-2.0; Bestandteile MIT, BSD-3-Clause, Apache-2.0 OR GPL-2.0-or-later | Framework, Treiber, Netz- und Funk-Stacks |
| [`improv-wifi-busware`](https://github.com/tostmann/improv-wifi-busware) (urspr. jnthas) | MIT | Improv-Serial-Provisioning |
| Abstimmwerte für den CC1101 im wM-Bus-Betrieb (über a-culfw `clib/mbus`, Herkunft Texas Instruments) | Registerwerte | AGC-, Filter-, Kalibrier- und Test-Register in `wmbus_cfg.cpp`; Tabelle neu geschrieben |

### Marken

`HomeMatic`, `MAX!`, `BidCoS`, `RaspberryMatic`, `debmatic`, `FHEM`, `Homegear`, `Home Assistant` sind Marken ihrer jeweiligen Eigentümer. CULFW32 ist eine **kompatible** Implementierung — keine Affiliation.
