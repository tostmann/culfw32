# CULFW32

**ESP32-C6 + CC1101 — the product successor of the `busware` CUL family.**

The full `culfw` feature set on a modern frontend: WiFi, a Matter/Thread-capable SoC, USB-C, web UI, OTA, multiple frontends (FHEM/Homegear/debmatic/SIGNALduino-compatible).

> Status: runs on hardware (ESP32-C6 devboard + CC1101 868 MHz). v1 feature set DONE — see the feature matrix below. The first hardware spin (`CULFW32-PCB`) is being prepared in parallel.

---

## What does it do?

Sub-GHz radio gateway for smart-home stacks such as FHEM, Homegear, debmatic, RaspberryMatic, Home Assistant. CULFW32 speaks the radio protocols of the `busware` CUL family and exposes the same data on several paths at the same time: USB-CDC, TCP ASCII (FHEM/`socat`), TCP JSON (web UI/HASS), HMUARTLGW (FHEM `uart://` + debmatic via `hb-rf-eth`), web UI in the browser.

### Radio protocols (all 868 MHz / CC1101)

| Cmd | Protocol | Status |
|---|---|---|
| `A` | AskSin (HomeMatic / BidCos) — RX/TX, Crypt, CCA, BURST | DONE |
| `Z` | Moritz (eq-3 MAX!) — RX/TX, AutoAck | DONE |
| `F K M T` | SlowRF (FS20 / KS300 / HMS / FHT / EM, OOK) | DONE — FHT sub-features partial |
| `N` | rf_native (IT+ / PCA301) | DONE |
| `f` | fastrf (CC1101 P2P 250 kbps GFSK) | DONE |
| `b` | wMBus T1 / S1 (Wireless M-Bus) | DONE — transmit verified on air against a-culfw |
| signalduino | SIGNALduino-compatible frontend (TCP 2326) | DONE |

> The 433 MHz OOK family (`i k Y L h G`) is excluded from v1 — the current antenna matching is for 868 MHz; a 433 MHz SKU is a follow-on.

**Send credit (duty cycle).** As in a-culfw, `X` shows the send credit in 10 ms units (maximum 900, +1 per second). SlowRF and MAX! transmissions are refused with `LOVF` when it runs out. Unlike a-culfw, AskSin transmissions are debited too (a burst frame costs about 38 units) but never refused, so heavy HomeMatic traffic can make FS20 or MAX! answer `LOVF` until the credit has recovered.

**CC1101 settings from the EEPROM.** As in a-culfw, SlowRF loads registers 0x00..0x28 from the EEPROM copy written with `W` (FHEM: frequency, `bWidth`, `rAmpl`, `sens`) whenever the mode starts and on every `X` with a non-zero mask. FSCTRL0 is the exception: the board's frequency calibration always replaces it. The SIGNALduino frontend uses its own fixed register set.

### Frontends

| Path | Port / transport | Use case |
|---|---|---|
| ASCII (`culfw`) | USB-CDC, UART, TCP `2323` | FHEM `CUL` module, `socat`, manual use |
| JSON | TCP `2324`, WebSocket `/ws/json` | web UI, Home Assistant, your own tools |
| Web UI | HTTP `80` | configuration & live terminal in the browser |
| HMUARTLGW (FHEM) | TCP `2325` | FHEM `HMUARTLGW` module with `uart://` prefix |
| HMUARTLGW (DualCoPro) | TCP `2327` + UDP `3008` | debmatic / RaspberryMatic via `hb-rf-eth` |
| SIGNALduino | TCP `2326` | FHEM `SIGNALduino` module |
| Improv-Serial | USB-CDC | first-time WiFi provisioning via the browser (install.busware.de) |

Several frontends run **at the same time** on the same stick — e.g. FHEM on 2323, debmatic on 2327, a browser on 80.

### Hardware features

- **ESP32-C6**: WiFi 6 (2.4 GHz), Bluetooth LE, Matter / Thread / Zigbee-capable (radio stacks for later extension)
- **CC1101**: 868 MHz sub-GHz transceiver, FSK + OOK
- **USB-C**: power, console (USB-Serial-JTAG), UART0 in parallel on the SBU pads
- **Flash 4 MB**: dual OTA slots (1.75 MB ota_0 + 1.75 MB ota_1) + 384 KB SPIFFS
- **OTA update**: wireless via the web UI or the JSON frontend

---

## Build

Built and tested with **ESP-IDF 5.5.3**.

```bash
. $IDF_PATH/export.sh            # ESP-IDF 5.5.3
cd firmware/
idf.py build                     # target and configuration from sdkconfig.defaults
idf.py -p <PORT> flash monitor
```

`firmware/sdkconfig.defaults` contains the complete configuration of the shipped firmware. A build from it produces the same image as the published version, except for the compile date in the `V` line. On the first build, the Espressif Component Manager downloads `improv-wifi-busware` (pinned to a commit, `firmware/main/idf_component.yml`) and `espressif/mdns` (version recorded in `firmware/dependencies.lock`). How to build against a local copy of `improv-wifi-busware` is described in a comment in `firmware/main/idf_component.yml`.

### Factory image

This is how the image is made that the web flasher on install.busware.de writes from address 0 (verified byte-exact against 0.1.912):

```bash
cd firmware/build
python -m esptool --chip esp32c6 merge_bin -o culfw32.factory.bin \
  --flash_mode dio --flash_freq 80m --flash_size 4MB \
  0x0 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin \
  0x10000 ota_data_initial.bin 0x20000 culfw32.bin
```

The online update downloads only the app image `culfw32.bin`.

### Build toggles

In `idf.py menuconfig` → "CULFW32 Configuration", individual radio classes (AskSin / Moritz / SlowRF / Native / FastRF / wMBus) and frontends (HMUARTLGW / SIGNALduino / JSON / Web-UI) can be switched off one by one. Linker GC removes classes that are not instantiated from the image. At runtime, `?build` lists the active modules.

### Versioning

`firmware/version.txt` and `firmware/build_number.txt` make up the version in the `V` line and in the app header (`esp_app_desc_t::version`), which the online update evaluates. Releases are published with the tag `v<Version>`, for example `v0.2.0`; the "Source" link in the web UI points to exactly this tag.

---

## First setup

1. **Flash & boot.** Connect USB-C, `idf.py flash monitor`.
2. **WiFi provisioning** — three ways:
   - **Improv-Serial**: open `install.busware.de` in the browser (Web Serial API), select CULFW32, enter SSID/PSK.
   - **AP fallback**: `?wifi.mode=2` puts CULFW32 into AP mode (SSID = host name, `culfw32-xxxxxx`), then set the credentials via the web UI.
   - **ASCII**: via USB-CDC `EWs<SSID>` and `EWp<password>`.
3. Open the **web UI**: `http://<culfw32-ip>` shows the config tab + live terminal.
4. Optional: connect FHEM/debmatic — see `docs/HmuartlgwHowTo.md` and `docs/DebmaticHowTo.md`.

---

## Documentation

| File | Content |
|---|---|
| [docs/NetworkHowTo.md](docs/NetworkHowTo.md) | WiFi setup, TCP server, allowlist, EW commands |
| [docs/HmuartlgwHowTo.md](docs/HmuartlgwHowTo.md) | FHEM `HMUARTLGW` connection, HM pairing workflow |
| [docs/DebmaticHowTo.md](docs/DebmaticHowTo.md) | debmatic / RaspberryMatic via the `hb-rf-eth` tunnel |

Test scripts in `firmware/tools/` cover the protocol classes byte-exact against a local/remote `busware` CUL868. Configuration via environment variables — see `firmware/tools/_lab_env.py`.

---

## Architecture (short overview)

Five layers, all in `firmware/components/`:

```
Transport        usbcdc / uart / tcpserver / websocket / hbrfeth_udp
   ↓
Frontend         culfw (ASCII) / json / web_server / hmuartlgw / signalduino
   ↓
Renderer         protocol-specific ASCII/JSON formatters
   ↓
RF class         rf_proto_asksin / moritz / slowrf / native / fastrf / wmbus / signalduino
   ↓
CC1101 driver    cc1101_driver + rf_manager (slot routing)
```

Multi-frontend coexistence: several frontends subscribe to the same RF class through the `rf_manager`. Transmission is serialized through `cc1101_slot`; all frontends see RX frames simultaneously.

---

## Related projects

CULFW32 is part of a family:

- **[`a-culfw`](https://github.com/tostmann/a-culfw)** — the original AVR/STM32 `culfw` code base whose protocol cores we vendor and port to ESP-IDF/FreeRTOS.
- **[`improv-wifi-busware`](https://github.com/tostmann/improv-wifi-busware)** — Improv-Serial implementation for the `install.busware.de` path.

---

## License

Copyright © 2026 Dirk Tostmann <tostmann@gmail.com>

CULFW32 is licensed under the **GNU General Public License, version 3 or (at your option) any later version** (`GPL-3.0-or-later`). Full text: [COPYING](COPYING).

Some files contain code from other GPL projects (a-culfw, SIGNALduino). Their origin and license notices are in the header of each file and in [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md). Because of the SIGNALduino part and the Apache-2.0 components of ESP-IDF, the work as a whole is licensed under version 3 (or later), not under version 2.

The frontends for HMUARTLGW and HB-RF-ETH are independent implementations of the respective wire protocols, written from FHEM's `00_HMUARTLGW.pm`, the piVCCU sources and our own captures. What was taken over are protocol facts (opcodes, frame format, CRC parameters).

Contributions: see [CONTRIBUTING.md](CONTRIBUTING.md).

### Credits & third-party licenses

| Component | License | Use |
|---|---|---|
| [`a-culfw`](https://github.com/tostmann/a-culfw) (R. Koenig, B. Hempel, D. Tostmann et al.) | GPL-2.0-or-later | SlowRF pulse decoder, CC1101 register tables, protocol sequences and command formats (details in the file headers) |
| [SIGNALduino](https://github.com/RFD-FHEM/SIGNALDuino) (N. Butzek, S. Butzek, HomeAutoUser et al.) | GPL-3.0-or-later | SIGNALduino decoder and transmit path |
| [FHEM `00_HMUARTLGW.pm`](https://svn.fhem.de/trac/browser/trunk/fhem/FHEM/00_HMUARTLGW.pm), piVCCU (A. Reinert) | GPL-2.0 / GPL-2.0-or-later | protocol reference for HMUARTLGW and HB-RF-ETH |
| [Espressif ESP-IDF](https://github.com/espressif/esp-idf) with FreeRTOS, lwIP, Mbed TLS, cJSON, SPIFFS and the Espressif binary libraries | Apache-2.0; parts MIT, BSD-3-Clause, Apache-2.0 OR GPL-2.0-or-later | framework, drivers, network and radio stacks |
| [`improv-wifi-busware`](https://github.com/tostmann/improv-wifi-busware) (originally jnthas) | MIT | Improv-Serial provisioning |
| Tuning values for the CC1101 in wM-Bus operation (via a-culfw `clib/mbus`, Texas Instruments origin) | register values | AGC, filter, calibration and test registers in `wmbus_cfg.cpp`; table rewritten |

### Trademarks

`HomeMatic`, `MAX!`, `BidCoS`, `RaspberryMatic`, `debmatic`, `FHEM`, `Homegear`, `Home Assistant` are trademarks of their respective owners. CULFW32 is a **compatible** implementation — no affiliation.
