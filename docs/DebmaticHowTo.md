# CULFW32 — debmatic / RaspberryMatic / piVCCU connection

CULFW32 replaces the eQ-3 **HM-MOD-RPI-PCB with DualCoPro 2.x firmware** (RPI-RF-MOD clone) over the **HB-RF-ETH** UDP tunnel protocol. debmatic, RaspberryMatic and piVCCU attach to it the same way as to a physically wired Pi HAT.

> **Status 2026-05-01** — build 0.1.153, phase 4 DualCoPro DONE. A real HM-LC-Sw1-Pl-2 was paired, switched and tested byte-exact against `multimacd↔CULFW32`. Wire protocol verified byte-exact against a real HM-MOD-RPI-PCB with DualCoPro 2.8.6 as the gold standard.
>
> For the FHEM connection see `HmuartlgwHowTo.md` — different port, different module identity, can run in parallel in CULFW32.

---

## Architecture

CULFW32 opens **two independent frontends** on the same stick — both usable at the same time, no mode switching:

```
                ┌──────────────────────────────────────────┐
                │ CULFW32 (ESP32-C6 + CC1101)              │
                │                                          │
   FHEM        ─┤ TCP :2325  Legacy   (HM-MOD-RPI-PCB)     │
   Homegear     │            Hello: "Co_CPU_Bl"            │
                │            APP_RECV (dst=01) + status+RSSI│
                │                                          │
   debmatic    ─┤ TCP :2327  Dual     (RPI-RF-MOD/DualCoPro)│
   RaspberryMatic            Hello: "HMIP_TRX_Bl"           │
   piVCCU       │            LLMAC_RECV (dst=03) + ms+RSSI │
                │            COMMON layer (dst=FE) +       │
                │            COMMON_IDENTIFY → "DualCoPro_App"│
                │                                          │
                │ UDP :3008  hb-rf-eth tunnel (always Dual)│
                │            ALTERNATIVE to TCP :2327       │
                │                                          │
                │       ▼                                  │
                │ AskSinRF (slot 0) → CC1101 868.299 MHz   │
                └──────────────────────┬───────────────────┘
                                       ▼
                                   Antenna
                                       │
                       ┌───────────────┴────────────────┐
                       │                                │
                  HM-LC-Sw1-Pl-2               HM-PB-2-WM, HM-CC-RT-DN, …
                                               all HomeMatic classic
```

debmatic talks primarily over the **HB-RF-ETH UDP tunnel on port 3008** (this is the protocol of Alex Reinert's `hb-rf-eth` kernel module, which emulates `/dev/raw-uart` as a character device). The TCP path on 2327 works as an alternative, but UDP is the path debmatic expects out of the box.

---

## Prerequisites

- CULFW32 image with `CONFIG_CULFW32_BUILD_FRONTEND_HMUARTLGW=y` (default) and `CONFIG_CULFW32_BUILD_RF_ASKSIN=y` (default)
- Build **>= 0.1.149** (phase 4 DualCoPro), ideally **>= 0.1.153** (two ports + UAF fix)
- WiFi connected — see [`NetworkHowTo.md`](NetworkHowTo.md)
- Pi with debmatic, RaspberryMatic or piVCCU
- Kernel module `hb-rf-eth` loaded (included out of the box in the RaspberryMatic image / debmatic stack)
- Radio distance below 10 m for reliable pairing (the test board has ~14 dB weaker RX sensitivity than a well-matched CUL868 — the final hardware spin solves this)

---

## Setup walkthrough — debmatic on Raspberry Pi

The **bench-verified** path. Reproducible against a Pi5 with the debmatic stack.

### 1. Find the CULFW32 IP

USB-CDC console or `nc <ip> 2323`:
```
?wifi.ip
? wifi.ip = 192.168.1.42  (str, ro)
```

### 2. Point the hb-rf-eth tunnel at the CULFW32 UDP endpoint

```bash
echo 192.168.1.42 | sudo tee /sys/class/hb-rf-eth/hb-rf-eth/connect
sleep 2
ls -l /dev/raw-uart1
# crw------- 1 root root 248, 1 May  1 00:25 /dev/raw-uart1
```

`/dev/raw-uart1` is now the debmatic character device endpoint for CULFW32. The module wraps UART bytes in UDP frames and sends them to `192.168.1.42:3008`. If the module is not loaded:

```bash
sudo modprobe hb-rf-eth
```

> **Pi reboot trap:** on a Pi with an HM-MOD-RPI-PCB plugged in, `/dev/raw-uart` (without `1`) is the built-in pl011 UART. CULFW32 is `/dev/raw-uart1` — **do not mix them up**. After a reboot, `/dev/raw-uart` ends up on pl011 again.

### 3. debmatic configuration

debmatic reads `/var/hm_mode` at start-up; it contains `HM_HMRF_DEVNODE=/dev/raw-uart1` once the parent `debmatic` service has been started. Without a Pi reboot:

```bash
sudo systemctl start debmatic               # parent -> writes /var/hm_mode
sudo systemctl start debmatic-multimacd     # radio layer daemon
sudo systemctl start debmatic-rfd           # XML-RPC logic daemon (port 32001)
```

The baked-in config (`/etc/config/rfd.conf`) needs no change — the debmatic multimacd detection recognizes CULFW32 as an RPI-RF-MOD by its hello push (`HMIP_TRX_Bl`) and init probe (`COMMON_IDENTIFY` → `DualCoPro_App`).

### 4. Check the status

```bash
journalctl -u debmatic-multimacd -n 30 -f
# expected: "device detection ok", "RPI-RF-MOD found", "firmware 2.8.6"
```

XML-RPC on rfd:
```bash
python3 -c "
import xmlrpc.client
p = xmlrpc.client.ServerProxy('http://127.0.0.1:32001/')
print('count:', len(p.listDevices()))
print('first :', p.listDevices()[0] if p.listDevices() else 'empty')
"
```

Initially `count` should be 1 or 2 (debmatic's own entry + a pseudo entry). Once a device is paired, count goes up.

### 5. Open the pairing window + teach in a device

Via the debmatic WebUI (`http://<pi-ip>/`):
- "Geraete" (devices) → "Geraet anlernen" (teach in device)
- Activate pairing mode for 60 s

On the HomeMatic device, **press the pair button for >4 s** (factory reset → the device forgets its old CCU master), wait a moment, then **press the pair button briefly** (DEVICE_INFO broadcast).

The device should appear in the debmatic WebUI within 5-30 s. Accept it in the inbox → the device is paired.

### 6. Switching

```bash
python3 -c "
import xmlrpc.client
p = xmlrpc.client.ServerProxy('http://127.0.0.1:32001/')
p.setValue('<serial>:1', 'STATE', True)   # ON
"
```

The actuator clicks physically. STATE is reflected in rfd; all XML-RPC clients (FHEM bridge, ioBroker, your own tooling) see the state immediately.

---

## RaspberryMatic / piVCCU

Both are identical to debmatic in wire format (same rfd, same multimacd, same `/dev/raw-uart` concept). Workflow:

1. Make sure `hb-rf-eth` is in the kernel (included out of the box in official RaspberryMatic images; on piVCCU install it manually with `apt install hb-rf-eth`)
2. Point the `connect` sysfs entry at the CULFW32 IP as above — on RaspberryMatic via the WebUI: System → Funkmodul (radio module) → "HB-RF-ETH IP-Adresse setzen" (set HB-RF-ETH IP address)
3. The CCU2 WebUI is reachable after 30 s → pair as usual

---

## ioBroker

ioBroker does **not** talk to CULFW32 **directly**. Instead: the ioBroker adapter `homematic-rpc` connects to rfd on port 32001 (same Pi as debmatic). The chain is therefore `ioBroker → rfd → multimacd → CULFW32`.

```yaml
# in iobroker, adapter homematic-rpc:
Host:   <pi-ip-of-debmatic>
Port:   32001
Protocol:  XML-RPC (BIN-RPC = faster variant; both work)
```

Since rfd has already been validated (see step 6 above), the ioBroker connection is mechanical — no CULFW32-specific configuration is needed.

---

## ConfigRegistry keys

On the CULFW32 ASCII console:

| Key | Type | Default | Description |
|---|---|---|---|
| `hmu.port`     | U16, persist | 2325 | Legacy port (HM-MOD-RPI-PCB / FHEM). The Dual port is always `hmu.port + 2`. |
| `hmu.module`   | U8, persist  | 0    | **Default mode for statically bound transports** (USB-CDC, UART). 0=Legacy, 1=Dual. Over TCP/UDP the mode is determined per slot stamp — so **do not change** this key for a TCP/UDP connection. |
| `hmu.hmid`     | Str, ro      | -    | active 3-byte HMID (set by rfd via APP_SET_HMID) |
| `?build`       | Str, ro      | -    | contains `fe:hmuartlgw` if the frontend is compiled in |
| `?stats`       | Tabular      | -    | RX/TX counters per protocol, heap, in future also uptime + reset counters |

Example:
```
?hmu.port           # 2325 (Legacy); Dual is 2327
?hmu.hmid           # currently active BidCos address
```

---

## Troubleshooting

### `device detection failed` in the multimacd log

CULFW32 is not reachable or does not answer in the expected wire format.

```bash
# reachability:
ping 192.168.1.42
nc -uvz 192.168.1.42 3008    # UDP 3008 should be open

# check the CULFW32 frontend:
nc 192.168.1.42 2323
?build              # expected: contains 'fe:hmuartlgw'
?wifi.state         # expected: 'connected'
```

CULFW32 can also be reached directly over TCP:2327 — smoke test independent of hb-rf-eth:
```bash
python3 -c "
import socket
s = socket.create_connection(('192.168.1.42', 2327))
hello = s.recv(64)
print(repr(hello))   # expected: b'\\xfd\\x00\\x10\\xfe\\x01\\xff\\xff\\x05HMIP_TRX_Bl…'
"
```

### Pairing display stalls at `RECEIVED PAIR_ANNOUNCE`

multimacd has received the DEVICE_INFO broadcast, but rfd did not accept the follow-up frames (`CONFIG_REGISTER_PARAM` sequence). Up to build 0.1.148 this was caused by a mismatch between the expected on-air AskSin CNT and the CULFW32-internal counter — **fixed in 0.1.149**: CULFW32 now uses `opts[3]` of LLMAC_SEND as the on-air CNT, identical to the DualCoPro 2.x firmware.

If build < 0.1.149: upgrade.

### Double hello push / board crash under rapid reconnect

Up to build 0.1.152 there was a UAF/ABA bug in the hello timer: with very fast connect/disconnect (FHEM reconnect loop or stale hb-rf-eth UDP) the same slot could fire twice, or the board could crash with a store/load access fault. **Fixed in 0.1.153** by per-slot timer tracking and cleanup on detach.

If build < 0.1.153: upgrade.

### `bad ep-id N expected M` in the UDP listener log

hb-rf-eth reconnect with an **old endpoint ID** from the Pi tunnel. CULFW32 assigns an ep-id on the first connect (e.g. 1); if the Pi later comes back with ep-id 3 after a disconnect, CULFW32 rejects the packets. This is expected behavior — the Pi has to reconnect:

```bash
echo "" | sudo tee /sys/class/hb-rf-eth/hb-rf-eth/connect    # disconnect
sleep 1
echo 192.168.1.42 | sudo tee /sys/class/hb-rf-eth/hb-rf-eth/connect
```

Known edge case: with very stale UDP packets from an old Pi reboot, CULFW32 can panic in rare cases — out-of-scope follow-up item, to be addressed in v1.x.

### Device transmits, but the `multimacd` log shows nothing

CULFW32 RX sensitivity on the test board is ~14 dB worse than a well-matched CUL868. With battery-powered HM-PB-/HM-RC-* devices that have a short listening window, the pairing answer does not arrive in time.

Workarounds:
- Move the device **physically closer** to CULFW32 (10-30 cm)
- Retrigger pairing several times (press the pair button on the device again)
- With mains-powered HM-LC-* it works reliably even at 2 m
- A hardware spin (CULFW32-PCB with a proper BALUN + antenna matching) solves it fundamentally

### `/dev/raw-uart1` does not appear

```bash
lsmod | grep hb_rf_eth      # should show 'hb_rf_eth'
sudo modprobe hb-rf-eth     # if not
ls /sys/class/hb-rf-eth/    # should show a 'hb-rf-eth' entry
```

If the module is not in the kernel (e.g. your own custom Linux): build it from https://github.com/alexreinert/hb-rf-eth.

---

## Wire protocol quick reference

This is not the complete byte-exact specification; it covers only the parts relevant to debmatic.

### Frame wrapper (shared with the FHEM path)

```
0xfd <len_hi> <len_lo> <dst> <cnt> <payload...> <crc_hi> <crc_lo>

length = dst + cnt + payload (without CRC)
escape = 0xfc and 0xfd in the body replaced by 0xfc + (b ^ 0x80)
crc16  = poly 0x8005, init 0xd77f, over [0xfd, len_h, len_l, dst, cnt, payload]
```

### Routing DSTs (Dual mode = debmatic)

| DST  | Layer  | Meaning |
|------|--------|-----------|
| 0x00 | HMSYSTEM | OS level (shared with Legacy) |
| 0x01 | APP    | not active in Dual (Legacy path) |
| 0x02 | HMIP   | HmIP stack (HmIPServer.jar — out of scope for CULFW32) |
| 0x03 | LLMAC  | **radio layer**: AskSin frames TX/RX |
| 0xFE | COMMON | Identify / Tick / Capabilities |

### Hello push on connect (Dual)

```
Direction:  CULFW32 → debmatic
Frame:     fd 00 10 fe 01 ff ff 05 'H' 'M' 'I' 'P' '_' 'T' 'R' 'X' '_' 'B' 'l' <crc>
                          ^                                                    ^
                          unsolicited push, boot loader mode identifier        13-byte hello
```

debmatic expects this push on the hb-rf-eth connect; without it, multimacd ends the detection with "no module".

### Init sequence (triggered by debmatic)

1. **CULFW32 → debmatic**: hello push `HMIP_TRX_Bl` (see above)
2. **debmatic → CULFW32**: `COMMON_IDENTIFY` (dst=FE, cmd=01) → CULFW32 answers `[01 01]`+ASCII `DualCoPro_App` + 3-byte version `02 08 06`
3. **debmatic → CULFW32**: `OS_GET_FIRMWARE` (dst=00, cmd=02) → `[04 02] + [00] + [02 08 06]` (FW 2.8.6)
4. **debmatic → CULFW32**: `OS_GET_SERIAL` (dst=00, cmd=0b) → `[04 02] + 10-byte ASCII serial` (e.g. `CULxxxxxxxx`)
5. **debmatic → CULFW32**: `LLMAC_INIT_RADIO` (dst=03) → ACK
6. **debmatic → CULFW32**: `LLMAC_SET_HMID` (dst=03) → ACK; CULFW32 persists the HMID in NVS
7. **debmatic → CULFW32**: `LLMAC_GET_HMID` (dst=03) → ACK + 3-byte HMID
8. (ongoing) **OS_TICK** heartbeats from both sides

After that, CULFW32 is in `OperationalMode::Dual::Active`, and all radio frames are exchanged via LLMAC_RECV (from the module) and LLMAC_SEND (to the module).

### LLMAC_SEND (debmatic → CULFW32, for transmission on air)

```
Frame:     fd <len> 03 <hostcnt> 02 00 00 <opts:4> <askSin: msg-len + msg-bytes> <crc>
                       ^         ^
                       cnt mirror        cmd=02 (SEND), reserved 00 00
                                                ^
                                                opts[0..3]: flags for csma/burst/aes/...
                                                opts[3] = on-air AskSin CNT (dictated by multimacd)
```

**Important:** the on-air AskSin CNT (the CNT byte in the radio frame) is NOT the host cnt from the 0xfd wrapper. multimacd encodes the expected on-air CNT in `opts[3]`. CULFW32 has to use that one — actuator ACKs echo the request CNT back (AskSinPP `Device::sendAck` reuses the msg buffer). This was the decisive fix in build 0.1.149 for phase 4.

### LLMAC_RECV (CULFW32 → debmatic, received from the air)

```
Frame:    fd <len> 03 <modcnt> 05 <ts_hi> <ts_lo> <info> <rssi_raw> <askSin: msg-len + msg-bytes> <crc>
                                  ^            ^             ^         ^
                                  16-bit ms timestamp          info flags     CC1101 RSSI raw
```

In Legacy mode (FHEM/Homegear) the prefix is `[05 status info rssi]` (3 bytes, without the ms timestamp). This is the essential wire difference between Legacy and Dual.

---

## Verification tools

### Frontend smoke test

```bash
cd firmware
python3 tools/release_smoke_test.py frontends
```

Expects 7/7 PASS — including:
- `HMUARTLGW :2325 Hello Co_CPU_BL` (Legacy/FHEM)
- `HMUARTLGW :2325 OS_GET_APP -> Co_CPU_App` (Legacy init probe)
- `HMUARTLGW :2327 Hello HMIP_TRX_Bl` (Dual/debmatic)
- `HMUARTLGW :2327 COMMON_IDENTIFY -> DualCoPro_App` (Dual init probe)

### Live trace when something looks wrong

debmatic has `dump_traffic` as a diagnostic tool — it writes the UART bytes byte-exact to a file:
```bash
sudo dump_traffic /dev/raw-uart1 /tmp/cul32-trace.bin &
# ... perform the action (pair, switch, etc.)
sudo killall dump_traffic
xxd /tmp/cul32-trace.bin | head -50
```

This shows the `fd ...` wrappers byte-exact, and you can compare them with the wire protocol spec.

### Layer verification status

| Layer | Verified against | Status |
|---|---|---|
| AskSin radio side (CC1101 reg + encrypt/decrypt) | a-culfw `rf_asksin.c` | ✅ phase 2 (3-way against local + remote CUL868) |
| HMUARTLGW 0xfd wrapper Dual | real HM-MOD-RPI-PCB with DualCoPro 2.8.6 | ✅ byte-exact via `dump_traffic` reference |
| HM-LC-Sw1-Pl-2 pairing via debmatic | physical switching via XML-RPC `setValue STATE` | ✅ count=55, ON/OFF repeatable |
| HM-PB-2-WM pairing via debmatic | DEVICE_INFO + autoCreate | ✅ phase 3 (test board RX asymmetry marginal at 2 m) |
| HmIP line | HmIPServer.jar | ⛔ out of scope, separate stack with AES-CCM |

---

## Known limitations (v1)

- **AES** (phase 4 AES) is not implemented — `APP_SET_CURRENT_KEY/PREVIOUS_KEY/TEMP_KEY` frames are ACKed plainly, keys are neither persisted nor used for crypto. In concrete terms: HM-Sec-* devices (alarm- and security-relevant) do not ACK control commands (state=AES_KO in rfd). HM-LC/HM-PB/HM-CC/HM-WDS work fully. Phase 4 AES comes in v1.x+1.
- **Peer table** is not persistent across reboots of the stick. After a CULFW32 reboot, devices have to be detected again (rfd does this automatically within 60-120 s).
- **HmIP line** is entirely outside this frontend. HmIP uses a different wire stack with AES-CCM and its own pairing daemon (`HmIPServer.jar`, eq-3 closed source). No plan for phase 5+.
- **LAN-gateway mode** (HM-LGW-O-TW-W-EU protocol) is not implemented and not planned. CULFW32 implements only the UART/UDP path.

---

## Sources

- Alex Reinert hb-rf-eth (kernel module + sysfs control): https://github.com/alexreinert/hb-rf-eth
- debmatic sources: https://github.com/alexreinert/debmatic
- eQ-3 OCCU (rfd, multimacd, BidCoS stack): https://github.com/eq-3/occu
- a-culfw rf_asksin.c (radio layer): https://github.com/tostmann/a-culfw/blob/master/culfw/clib/rf_asksin.c
