# CULFW32 — HMUARTLGW frontend (FHEM/Homegear/RaspberryMatic connection)

CULFW32 emulates the **HM-MOD-RPI-PCB** — eQ-3's radio module for Raspberry Pi CCU clones — over TCP. FHEM, Homegear, debmatic, piVCCU or RaspberryMatic see CULFW32 as an ordinary HomeMatic radio stick and talk to it using the 0xfd binary HMUARTLGW protocol.

Wire protocol: **0xfd frame wrapper, CRC-16 (poly 0x8005, init 0xd77f), escape (0xfc + b^0x80), single-port TCP**. Identical to an HM-MOD-RPI-PCB connected directly via GPIO; **not** the LAN-gateway protocol of the HM-LGW-O-TW-W-EU (not implemented by CULFW32 and not planned). Port 2325 is an emulation inside CULFW32, not a pass-through to a module on the serial port.

> **Status 2026-04-28:** phases 0..3 DONE, byte-exact against FHEM in Docker + real eQ-3 HomeMatic hardware:
> - **HM-LC-Sw1-Pl-2** (plug-in switch, HMID `1A2B3C`, FW 0xA1 = v1.6): full pairing + `set on/off` verified physically (LED on/off)
> - **HM-PB-2-WM** (wall push button, HMID `4D5E6F`): DEVICE_INFO received + autoCreate; pairing completion marginal at 2 m because of the test board's RX sensitivity asymmetry (the hardware spin solves this)
> - **AES** (phase 4) remains open — see the AES section below

---

## Architecture

```
┌─────────────────────┐                  ┌──────────────────────────────┐
│ FHEM / Homegear /   │                  │ CULFW32 (ESP32-C6 + CC1101)  │
│ RaspberryMatic /    │  TCP :2325       │                              │
│ debmatic            │ ◄──── 0xfd ────► │  HmuartlgwFrontend           │
│                     │                  │       │                      │
│  CUL_HM/HM-Mod-     │                  │  RfManager                   │
│  Rpi-Pcb-Driver     │                  │       │                      │
│                     │                  │  AskSinRF  (slot 0)          │
└─────────────────────┘                  │       │                      │
                                         │  CC1101 868.299 MHz GFSK     │
                                         │       │                      │
                                         │       └─ Antenna ── radio    │
                                         └──────────────────────────────┘
                                                                │
                                                        ┌───────┴───────┐
                                                        │               │
                                                  HM-LC-Sw1-Pl-2   HM-PB-2-WM
                                                  HMID 1A2B3C      HMID 4D5E6F
```

On the radio side, CULFW32 is byte-exact to a-culfw `rf_asksin.c` (verified in phase 2 against a local and a remote CUL868). The wrapper layer above the AskSin stack is a **clean-room reimplementation against FHEM `00_HMUARTLGW.pm`** — `a-culfw` itself does not speak the HMUARTLGW protocol (it is specific to the eQ-3 firmware, a closed-source binary).

---

## Prerequisites

- CULFW32 image with `CONFIG_CULFW32_BUILD_FRONTEND_HMUARTLGW=y` and `CONFIG_CULFW32_BUILD_RF_ASKSIN=y` (both default in the standard builds)
- WiFi connected (see [`NetworkHowTo.md`](NetworkHowTo.md))
- TCP port (default 2325) reachable from the host that handles the connection
- FHEM with `00_HMUARTLGW.pm` **or** Homegear with `Homegear-HomeMaticBidCoS` **or** debmatic / piVCCU / RaspberryMatic with `rfd`

---

## FHEM — end-to-end setup walkthrough

The tested path. Result: the HomeMatic device pairs, control commands go on air byte-exact.

### 1. Find the CULFW32 IP

On the CULFW32 ASCII console (USB-CDC or via `nc <ip> 2323`):
```
?wifi.ip
? wifi.ip = 192.168.1.42  (str, ro)
```

### 2. FHEM define with the `uart://` prefix

In the FHEM web frontend, on the command line:

```fhem
define myhm HMUARTLGW uart://192.168.1.42:2325
attr   myhm hmId F11234
attr   myhm verbose 5
```

> **The `uart://` prefix is mandatory** — without it, `00_HMUARTLGW.pm` treats the address as an HM-LGW-O-TW-W-EU LAN gateway (`DevType` LGW, with a second keep-alive connection). That is a different protocol, not implemented by CULFW32; the interface never initializes. FHEM also opens its keep-alive connection on port + 1, which is 2326, the SIGNALduino port of CULFW32.

`hmId` is the 3-byte CCU address with which CULFW32 presents itself as a CCU to HomeMatic devices. Free to choose — preferably a hex number that does not collide with existing device HMIDs (e.g. `F11234`). After it is set for the first time, FHEM writes this ID into the CULFW32 EE via `APP_SET_HMID`; it persists across reboots.

### 3. Watch the init sequence

Within ~5 s, FHEM should run through the init sequence (see "Init sequence" below). `list myhm` shows:

```
Internals:
   model       HM-MOD-UART        ← FHEM recognizes CULFW32 as an HM-MOD-UART clone
   STATE       opened
   Initialized 1
Readings:
   cond        ok
   loadLvl     low
   D-firmware  1.4.1
   D-serialNr  CULAABBCC          ← MAC-derived (from the CULFW32 WiFi MAC)
   D-HMIdAssigned   34            ← (= 0xF11234, FHEM strips leading zeros)
   D-HMIdOriginal   34
   D-type      HM-MOD-UART
```

### 4. Open the pairing window + teach in the device

```fhem
set myhm hmPairForSec 300
```

Then **press the pair button on the HomeMatic device for >4 seconds** (factory reset → the device forgets the old CCU master), wait a moment, then **press the pair button briefly** (sends the DEVICE_INFO broadcast).

FHEM autocreate creates the device:
```
2026.04.28 23:19:29 CUL_HM pair: HM_1A2B3C switch, model HM-LC-SW1-PL2 serialNr <serial>
```

### 5. Switching

```fhem
set HM_1A2B3C on
set HM_1A2B3C off
```

The plug-in switch switches; FHEM reading `level=100/0`, `state=on/off`.

### 6. Optional: cross-sniff with a second CUL stick

With a second USB CUL (CUL868) running in parallel in `Ar` mode, you can see the radio frames byte-exact:
```
A 10 33 80 02 1A2B3C F11234 01 01 C8 00 ::-42:myhm
              ↑   ↑     ↑                ↑   ↑   ↑
            msg_nr  ACK-type            ch  state level
```

---

## Homegear / debmatic / piVCCU / RaspberryMatic — `socat` bridge

The eQ-3 OCCU tools (`rfd`, `HmIPServer.jar` — the latter only for the HmIP line, not relevant for HomeMatic classic on CULFW32) and Homegear expect a local character device such as `/dev/raw-uart` or `/dev/ttyAMA0`. CULFW32 speaks TCP. Bridge:

### On the CCU host

```bash
sudo socat pty,link=/dev/cul32-uart,raw,echo=0,mode=666 \
           tcp:192.168.1.42:2325,reuseaddr,keepalive
```

Effect: all bytes on TCP↔CULFW32 appear 1:1 on `/dev/cul32-uart`. Latency +1-3 ms — irrelevant for HMUARTLGW.

### Homegear config (`/etc/homegear/families/homematicbidcos.conf`)

```ini
[CULFW32-via-TCP]
id = CULFW32
deviceType = hm-mod-rpi-pcb
device = /dev/cul32-uart
responseDelay = 95
gpio1 = 0    # no RST_N GPIO because of the TCP bridge — Homegear then ignores it
```

Then `homegear restart`, and in the Homegear CLI:
```
hg> families list
hg> peers list
```

### RaspberryMatic / debmatic / piVCCU

Set up `/etc/config/cul32-bridge.service` (systemd unit) so that the `socat` command starts at boot, and configure `rfd` so that it uses `/dev/cul32-uart` instead of the built-in `/dev/ttyS<n>`. The `rfd.conf` entry is analogous to Homegear above.

> **Status**: `rfd` (debmatic, RaspberryMatic) runs against the Dual port 2327, see [DebmaticHowTo](DebmaticHowTo.md). Homegear has not been tested against CULFW32. Source inspection of Homegear `Hm-Mod-Rpi-Pcb.cpp`:
>
> - On connect, Homegear expects an **unsolicited push** of the `Co_CPU_App` frame (passive listen + 5 s timeout). FHEM, in contrast, actively sends `OS_GET_APP` and reacts to the answer.
> - The **first implementation iteration with a hello push** (write the frame to the fresh TCP socket as soon as accept() returns) drove FHEM into `cond=disconnected` — presumably the bytes arrive before FHEM has completed its own init setup. The push is therefore **disabled**; the OS_GET_APP active probe pattern (FHEM) works byte-exact.
> - Homegear bring-up will probably need a **delayed hello push** (e.g. 200 ms after attach, so that FHEM has finished its setup) — or a client-side heuristic. This still has to be verified against a real Homegear installation.
>
> Byte format differences between `00_HMUARTLGW.pm` and `Hm-Mod-Rpi-Pcb.cpp` are small (same frame structure, same CRC, both accept the `[0x00] + "Co_CPU_App"` answer).

---

## ConfigRegistry keys

On the CULFW32 ASCII console or via the web UI / JSON frontend:

| Key | Type | Description |
|---|---|---|
| `hmu.port` | U16, persist | Listen port. Default 2325. Persisted in EE 0x123 (LE). A change **restarts** the TCP listener live. |
| `hmu.hmid` | Str (read-only) | Current 3-byte HMID as 6 hex digits (e.g. `F11234`). FHEM sets it via `APP_SET_HMID`. |
| `?build` | Str (read-only) | contains `fe:hmuartlgw` if the frontend is compiled in |
| `?stats` | Tabular | `asksin rx N/M (B)` — RX/TX counters per protocol, helpful for debugging |

```
?hmu.port           # query the current port
?hmu.port=2400      # set the port, the server rebinds immediately
?hmu.hmid           # currently active BidCos address
```

The HMID **cannot** be set **directly** on CULFW32 — FHEM/Homegear is the authoritative source and writes it via `APP_SET_HMID`. The last HMID that was set is kept across reboots.

---

## AES (phase 4, v1.x+1)

HomeMatic classic has two security classes — and CULFW32 v1 covers only the unsigned one. In practice this does not block the vast majority of home automation setups, but it is critical for security equipment.

### AES mechanism in HomeMatic

The "secure" variant of the AskSin protocol is a challenge-response on the frame layer:

1. The CCU sends a control command to the peer
2. The peer answers with an AES challenge (6 random bytes)
3. The CCU computes AES-128(key, challenge ⊕ original-msg) and sends the response
4. The peer checks the AES response against its local key; on a match, the command is executed

For each peer relationship, a bit in master list 0 says whether AES is mandatory. The CCU manages the keys, and FHEM passes them to the module via `APP_SET_CURRENT_KEY` / `APP_SET_PREVIOUS_KEY` / `APP_SET_TEMP_KEY` (17 bytes each: 1-byte index + 16-byte AES key).

CULFW32 v1 ACKs these commands but **does not store the keys** — phase 4 (mbedtls/aes on the C6 hardware AES accelerator) is the next iteration.

### Which devices need AES?

| Device class | AES by default | Meaning without AES |
|---|---|---|
| **HM-Sec-** (security line) — `HM-Sec-SC-2`, `HM-Sec-MD`, `HM-Sec-Key`, `HM-Sec-Win`, `HM-Sec-SD`, `HM-Sec-Sir-WM`, `HM-Sec-RHS` | **Mandatory** | the device does not ACK control commands; FHEM reading `state=AES_KO` |
| **HM-LC-** (light/switch) — `HM-LC-Sw*-Pl*`, `HM-LC-Dim*`, `HM-LC-Bl*` | **Optional** (off) | fully functional |
| **HM-PB-** / **HM-RC-** (wall/remote push buttons) | **Optional** (off) | fully functional |
| **HM-CC-** (heating thermostats) — `HM-CC-RT-DN`, `HM-CC-TC` | **Optional** (off) | fully functional |
| **HM-WDS-** (weather sensors) | **Optional** (off) | fully functional |
| **HM-Dis-** / **HM-LGW-** (display/gateway) | mixed | mostly optional |
| **HmIP** (HomeMatic IP — new protocol stack) | **Mandatory** on L1 (CCM) | completely different stack — no classic HMUARTLGW AES, a separate component is required |

CULFW32 v1 without AES therefore serves the **vast majority** of HomeMatic classic home automation devices. HM-Sec devices and HmIP-only setups need phase 4+.

### What FHEM does without IODev AES

FHEM `00_HMUARTLGW.pm` tolerates AES-KO **per peer**: with a device that requires AES, `cond=ok` is still shown, but that single device gets `state=AES_KO`. The rest of the setup keeps running normally — no crash, no global lockout.

---

## Troubleshooting

### `cond=disconnected` remains after `define`

- Check reachability: `ping <culfw32-ip>` and `nc -zv <culfw32-ip> 2325` from the FHEM host
- Firewall? CULFW32 listens on `0.0.0.0:2325`; the FHEM host must be allowed to connect out
- `attr myhm verbose 5` and `tail -f fhem.log` — the first line should be `myhm device opened`
- Check CULFW32 itself: `?hmu.port` (should show `2325`), `?wifi.state` (should be `connected`)

### `cond=ERROR-Overload`

CULFW32 answers `OS_GET_CREDITS` with a load > 100% (= byte > 0x64). This should not happen — bug report:
- Get `?stats` on the ASCII console
- FHEM verbose log with `read raw` lines
- Bug report to `tostmann@gmail.com` with both

### `cond=ok`, but `Initialized` stays 0

Init hangs halfway. Look through the FHEM verbose log for `state ` — the highest state reached shows where it is stuck:
- `state 1` (QUERY_APP) hangs → the unsolicited push is not accepted; re-probe via `set myhm reopen`
- `state 7-9` (SET_TIME, GET_FIRMWARE, GET_SERIAL) hangs → bug in the CULFW32 answer format
- `state 14-16` (SET_*_KEY) → AES keys are being loaded; plain-ACKed by CULFW32 (not a bug)

### Device pairs, but `STATE=MISSING ACK` / `RESPONSE TIMEOUT`

The CULFW32 test board has **~14 dB weaker RX sensitivity** than a well-matched CUL868 (stub without BALUN on the C6 mini board). With battery-powered HM-PB-2-WM/HM-RC-* that have a short listening window, the pairing answer does not arrive in time.

Workarounds:
- Move the device **physically closer** to CULFW32 (10-30 cm)
- Retrigger pairing several times (`set myhm hmPairForSec 300`, then press the pair button)
- With mains-powered HM-LC-* it works reliably even at 2 m
- A hardware spin (CULFW32-PCB with a proper BALUN + antenna matching) solves it fundamentally

### Frames do not appear in FHEM, but the CUL868 sniffer sees them

Same diagnosis: RX sensitivity. The CUL868 hears them, CULFW32 is below its sensitivity floor at that location. Move the device closer or wait for the hardware spin.

---

## Verification tools

### Test tools in the repo

`test/hmuartlgw/`:
- `frame_parser.py` — frame wrapper as a Python reference (CRC, escape, encode/decode). Round-trip test passed.
- `trace_listener.py --mode silent|stub-ack` — TCP stub server that answers the FHEM init sequence byte-exact **without** real CULFW32 hardware. Good for testing the FHEM setup.
- FHEM reference `00_HMUARTLGW.pm`: https://svn.fhem.de/trac/browser/trunk/fhem/FHEM/00_HMUARTLGW.pm

Recommendation: before connecting the CULFW32 hardware for the first time, test whether the setup path works at all:
```bash
# on the FHEM host:
cd test/hmuartlgw
python3 trace_listener.py --mode stub-ack --port 2325 &
# in FHEM:
define teststub HMUARTLGW uart://localhost:2325
# → should show cond=ok / Initialized=1 immediately
```

### Layer verification status

| Layer | Byte-exact against what? | Status |
|---|---|---|
| ASCII frontend `A`/`Z`/`F`/`T`/etc. | `a-culfw clib` (original) | ✅ verified (phase 2 bring-up) |
| AskSin radio side (CC1101 reg + encrypt/decrypt) | `a-culfw rf_asksin.c` | ✅ verified (3-way against local + remote CUL868) |
| HMUARTLGW 0xfd wrapper | FHEM `00_HMUARTLGW.pm` expectation | ✅ verified (init sequence byte-exact, real pairing of the plug-in switch) |
| HMUARTLGW against a real eQ-3 module (rfd) | direct comparison against the HM-MOD-RPI-PCB firmware | ⚠ **not yet** — follow-up test once HW + a RaspberryMatic setup are in the lab |
| HMUARTLGW against Homegear-HomeMaticBidCoS | `Hm-Mod-Rpi-Pcb.cpp` (open source) | ⚠ **not yet** — follow-up test, plausibly compatible |

---

## Known limitations (v1)

- **AES**: storage-only, no crypto processing (see above — phase 4, v1.x+1).
- **Wall push button pairing** at >1 m with the test board is marginal — the hardware spin (BALUN + proper antenna matching) solves this. Mains-powered devices are reliable even at 2 m.
- **Peer table** is not persistent — empty after each FHEM reconnect. It works without it, because AskSinRF runs promiscuously and FHEM CUL_HM filters on the dst side. Persistence would be a nice-to-have for faster reconnects.
- **HMUARTLGW LAN-gateway mode** (HM-LGW-O-TW-W-EU protocol, what FHEM uses when the define has no `uart://` prefix) is not implemented and not planned.
- **HmIP line** (HomeMatic IP) is entirely outside this frontend. HmIP uses a different wire stack with AES-CCM and its own pairing daemon (`HmIPServer.jar`, eq-3 closed source). No plan for phase 5+.

---

## Wire protocol quick reference

This is not the complete specification; only the core facts.

### Frame layout
```
0xfd <len_hi> <len_lo> <dst> <cnt> <payload...> <crc_hi> <crc_lo>
                        │      │
                        │      └─ Counter byte. Host sends increment it;
                        │         the module mirrors it in the ACK. Module sends
                        │         (unsolicited APP_RECV) use their own counter.
                        └─ 0=OS, 1=APP, (254=DUAL, 255=DUAL_ERR — DualCoPro/HmIP)
length = dst+cnt+payload bytes (without CRC).
escape = 0xfc and 0xfd in the body are replaced by 0xfc + (b ^ 0x80).
crc16  = poly 0x8005, init 0xd77f. Computed over [0xfd, len_h, len_l, dst, cnt, payload].
```

### OS layer commands (dst=0)
| Code | Name | Answer |
|---|---|---|
| 0x00 | OS_GET_APP | `[0x00] + "Co_CPU_App"` |
| 0x02 | OS_GET_FIRMWARE | `[0x04, 0x02] + 3-pad + M.m.p` (3-byte version) |
| 0x03 | OS_CHANGE_APP | `[0x04, 0x01]` (plain ACK) |
| 0x06 | OS_NORMAL_MODE | `[0x04, 0x01]` |
| 0x08 | OS_GET_CREDITS | `[0x04, 0x02, load%]` (0=idle, >100→ERROR-Overload) |
| 0x09 | OS_ENABLE_CREDITS | `[0x04, 0x01]` |
| 0x0a | OS_ENABLE_CSMACA | `[0x04, 0x01]` |
| 0x0b | OS_GET_SERIAL | `[0x04, 0x02] + 10 ASCII bytes` |
| 0x0e | OS_SET_TIME | `[0x04, 0x01]` |

### APP layer commands (dst=1)
| Code | Name | Meaning |
|---|---|---|
| 0x00 | APP_SET_HMID | host sets the 3-byte HMID |
| 0x01 | APP_GET_HMID | answer `[0x04, 0x07] + 3-byte HMID` (ACK_WITH_MULTIPART_DATA) |
| 0x02 | APP_SEND | TX frame: `[02 00 00 mode] + msg_nr + flags + mtype + src(3) + dst(3) + payload` |
| 0x03 | APP_SET_CURRENT_KEY | 17 bytes (1-byte index + 16-byte AES key) |
| 0x05 | APP_RECV | RX frame (unsolicited from the module): `[0x05, status, info, RSSI_raw] + msg_nr + flags + mtype + src + dst + payload` |
| 0x10 | APP_DEFAULT_HMID | answer `[0x04, 0x07] + 3-byte HMID` |

ACK codes: 0x01=OK, 0x02=ACK_INFO, 0x05=ENO_CREDITS (LOVF), 0x06=ECSMACA, 0x07=ACK_WITH_MULTIPART_DATA, 0x0c=AES_OK, 0x0d=AES_KO.

### Init sequence (triggered by FHEM after the TCP connect)

1. `OS_GET_APP` → we answer `[0x00] + "Co_CPU_App"` (= app is active)
2. `APP_GET_HMID` → we answer with the HMID from EE
3. `APP_DEFAULT_HMID` → we answer with the original HMID (same as SET-HMID)
4. `OS_SET_TIME` → ACK
5. `OS_GET_FIRMWARE` → we answer 1.4.1 (non-outdated floor)
6. `OS_NORMAL_MODE` → ACK
7. `OS_GET_SERIAL` → we answer "CUL<MAC>"
8. `OS_ENABLE_CSMACA` → ACK
9. `OS_ENABLE_CREDITS` → ACK
10. `OS_GET_CREDITS` → we answer 0% (idle)
11. `APP_SET_CURRENT_KEY` → ACK (key currently not processed)
12. `APP_SET_PREVIOUS_KEY` → ACK
13. `APP_SET_TEMP_KEY` → ACK

After that, `cond=ok`, `Initialized=1`. The module is ready for operation.

---

## Sources

- FHEM `00_HMUARTLGW.pm`: https://github.com/mhop/fhem-mirror/blob/master/fhem/FHEM/00_HMUARTLGW.pm
- Homegear-HomeMaticBidCoS Hm-Mod-Rpi-Pcb.cpp: https://github.com/Homegear/Homegear-HomeMaticBidCoS/blob/master/src/PhysicalInterfaces/Hm-Mod-Rpi-Pcb.cpp
- eQ-3 OCCU: https://github.com/eq-3/occu
- alexreinert HB-RF-ETH (TCP bridge to the HM-MOD): https://github.com/alexreinert/HB-RF-ETH
- a-culfw rf_asksin.c (radio layer): https://github.com/tostmann/a-culfw/blob/master/culfw/clib/rf_asksin.c
