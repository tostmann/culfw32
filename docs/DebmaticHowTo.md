# CULFW32 — debmatic / RaspberryMatic / piVCCU-Anbindung

CULFW32 ersetzt das eQ-3 **HM-MOD-RPI-PCB mit DualCoPro-2.x-Firmware** (RPI-RF-MOD-Klon) ueber das **HB-RF-ETH**-UDP-Tunnelprotokoll. debmatic, RaspberryMatic und piVCCU andocken sich genauso wie an einen physisch verkabelten Pi-HAT.

> **Status 2026-05-01** — Build 0.1.153, Phase 4-DualCoPro DONE. Echte HM-LC-Sw1-Pl-2 (KEQ0958686) byte-genau gegen `multimacd↔CULFW32` gepairt, geschaltet, getestet. Wire-Protokoll byte-genau verifiziert gegen reale HM-MOD-RPI-PCB mit DualCoPro 2.8.6 als Goldstandard (`hmuartlgw_dualcopro_real_reference_2026-04-30.md`).
>
> Fuer FHEM-Anbindung siehe `HmuartlgwHowTo.md` — anderer Port, andere Modul-Identitaet, in CULFW32 parallel betreibbar.

---

## Architektur

CULFW32 oeffnet **zwei voneinander unabhaengige Frontends** auf demselben Stick — beide gleichzeitig nutzbar, kein Mode-Umschalten:

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
                │            COMMON-Layer (dst=FE) +       │
                │            COMMON_IDENTIFY → "DualCoPro_App"│
                │                                          │
                │ UDP :3008  hb-rf-eth Tunnel (immer Dual) │
                │            ALTERNATIVE zu TCP :2327       │
                │                                          │
                │       ▼                                  │
                │ AskSinRF (Slot 0) → CC1101 868.299 MHz   │
                └──────────────────────┬───────────────────┘
                                       ▼
                                   Antenne
                                       │
                       ┌───────────────┴────────────────┐
                       │                                │
                  HM-LC-Sw1-Pl-2               HM-PB-2-WM, HM-CC-RT-DN, …
                  (KEQ0958686)                 alle HomeMatic-classic
```

debmatic spricht primaer ueber den **HB-RF-ETH-UDP-Tunnel auf Port 3008** (das ist das Protokoll von Alex Reinerts `hb-rf-eth`-Kernel-Modul, das `/dev/raw-uart` als Charakter-Device emuliert). Alternativ funktioniert auch der TCP-Pfad auf 2327, aber UDP ist der von debmatic out-of-the-box erwartete Pfad.

---

## Voraussetzungen

- CULFW32-Image mit `CONFIG_CULFW32_BUILD_FRONTEND_HMUARTLGW=y` (Default) und `CONFIG_CULFW32_BUILD_RF_ASKSIN=y` (Default)
- Build **>= 0.1.149** (Phase-4-DualCoPro), idealerweise **>= 0.1.153** (Two-Port + UAF-Fix)
- WiFi verbunden — siehe [`NetworkHowTo.md`](NetworkHowTo.md)
- Pi mit debmatic, RaspberryMatic oder piVCCU
- Kernel-Modul `hb-rf-eth` geladen (im RaspberryMatic-Image / debmatic-Stack ab Werk dabei)
- Funkdistanz unter 10 m fuer zuverlaessigen Pair (Test-Board hat ~14 dB schwaechere RX-Sensitivitaet als ein gut-angepasster CUL868 — finaler Hardware-Spin loest's)

---

## Setup-Walkthrough — debmatic auf Raspberry Pi

Der **bench-verifizierte** Pfad. Reproduzierbar gegen einen Pi5 mit debmatic-Stack.

### 1. CULFW32-IP herausfinden

USB-CDC-Konsole oder `nc <ip> 2323`:
```
?wifi.ip
? wifi.ip = 192.168.1.42  (str, ro)
```

### 2. hb-rf-eth-Tunnel auf den CULFW32-UDP-Endpunkt zeigen

```bash
echo 192.168.1.42 | sudo tee /sys/class/hb-rf-eth/hb-rf-eth/connect
sleep 2
ls -l /dev/raw-uart1
# crw------- 1 root root 248, 1 May  1 00:25 /dev/raw-uart1
```

`/dev/raw-uart1` ist jetzt der debmatic-Charakter-Device-Endpunkt fuer CULFW32. Das Modul kapselt UART-Bytes in UDP-Frames und schickt sie an `192.168.1.42:3008`. Wenn das Modul nicht geladen ist:

```bash
sudo modprobe hb-rf-eth
```

> **Pi-Reboot-Falle:** auf einem Pi mit aufgesteckter HM-MOD-RPI-PCB ist `/dev/raw-uart` (ohne `1`) der eingebaute pl011-UART. CULFW32 ist `/dev/raw-uart1` — **nicht verwechseln**. Nach Reboot landet `/dev/raw-uart` wieder auf pl011.

### 3. debmatic-Konfiguration

debmatic liest `/var/hm_mode` beim Start, dort steht `HM_HMRF_DEVNODE=/dev/raw-uart1` wenn der parent-`debmatic`-Service hochgefahren wurde. Ohne Pi-Reboot:

```bash
sudo systemctl start debmatic               # parent -> schreibt /var/hm_mode
sudo systemctl start debmatic-multimacd     # Funklayer-Daemon
sudo systemctl start debmatic-rfd           # XML-RPC-Logikdaemon (Port 32001)
```

Eingebrannte Konfig (`/etc/config/rfd.conf`) braucht keine Aenderung — debmatic-multimacd-Detect erkennt CULFW32 als RPI-RF-MOD durch dessen Hello-Push (`HMIP_TRX_Bl`) und Init-Probe (`COMMON_IDENTIFY` → `DualCoPro_App`).

### 4. Status pruefen

```bash
journalctl -u debmatic-multimacd -n 30 -f
# erwartet: "device detection ok", "RPI-RF-MOD found", "firmware 2.8.6"
```

XML-RPC am rfd:
```bash
python3 -c "
import xmlrpc.client
p = xmlrpc.client.ServerProxy('http://127.0.0.1:32001/')
print('count:', len(p.listDevices()))
print('first :', p.listDevices()[0] if p.listDevices() else 'empty')
"
```

Initial sollte `count` 1 oder 2 sein (debmatic-Selbst-Eintrag + Pseudoeintrag). Sobald ein Geraet gepairt ist, steigt count.

### 5. Pair-Window oeffnen + Geraet anlernen

ueber das debmatic-WebUI (`http://<pi-ip>/`):
- "Geraete" → "Geraet anlernen"
- Pair-Modus 60 s aktivieren

Am HomeMatic-Geraet **Pair-Knopf >4 s druecken** (Factory-Reset → Geraet vergisst alten CCU-Master), kurz warten, **Pair-Knopf kurz druecken** (DEVICE_INFO-Broadcast).

In der debmatic-WebUI sollte das Geraet binnen 5-30 s erscheinen. Akzeptiere Posteingang → Geraet ist gepairt.

### 6. Schalten

```bash
python3 -c "
import xmlrpc.client
p = xmlrpc.client.ServerProxy('http://127.0.0.1:32001/')
p.setValue('KEQ0958686:1', 'STATE', True)   # ON
"
```

Aktor klickt physisch. STATE wird im rfd reflektiert; alle XML-RPC-Clients (FHEM-Bridge, ioBroker, eigenes Tooling) sehen den State sofort.

---

## RaspberryMatic / piVCCU

Beide sind im Wire-Format mit debmatic identisch (gleicher rfd, gleicher multimacd, gleiches `/dev/raw-uart` Konzept). Workflow:

1. Stelle sicher dass `hb-rf-eth` im Kernel ist (in offiziellen RaspberryMatic-Images ab Werk dabei; in piVCCU manuell `apt install hb-rf-eth`)
2. `connect`-Sysfs-Eintrag wie oben auf CULFW32-IP zeigen lassen — bei RaspberryMatic ueber das WebUI: System → Funkmodul → "HB-RF-ETH IP-Adresse setzen"
3. CCU2-WebUI ist nach 30 s erreichbar → Pair wie ueblich

---

## ioBroker

ioBroker spricht **nicht direkt** mit CULFW32. Stattdessen: ioBroker-Adapter `homematic-rpc` zu rfd auf Port 32001 (gleicher Pi wie debmatic). Damit ist die Kette `ioBroker → rfd → multimacd → CULFW32`.

```yaml
# in iobroker, Adapter homematic-rpc:
Host:   <pi-ip-von-debmatic>
Port:   32001
Protokoll:  XML-RPC (BIN-RPC = beschleunigte Variante; beides geht)
```

Da rfd schon validiert wurde (siehe Schritt 6 oben), ist die ioBroker-Anbindung mechanisch — keine CULFW32-spezifische Konfiguration noetig.

---

## ConfigRegistry-Schluessel

Auf der CULFW32-ASCII-Konsole:

| Key | Type | Default | Beschreibung |
|---|---|---|---|
| `hmu.port`     | U16, persist | 2325 | Legacy-Port (HM-MOD-RPI-PCB / FHEM). Dual-Port ist immer `hmu.port + 2`. |
| `hmu.module`   | U8, persist  | 0    | **Default-Mode fuer statisch gebundene Transports** (USB-CDC, UART). 0=Legacy, 1=Dual. Ueber TCP/UDP wird der Mode pro Slot-Stempel ermittelt — diesen Schluessel daher bei TCP/UDP-Anbindung **nicht aendern**. |
| `hmu.hmid`     | Str, ro      | -    | aktive 3-Byte HMID (von rfd via APP_SET_HMID gesetzt) |
| `?build`       | Str, ro      | -    | enthaelt `fe:hmuartlgw` wenn das Frontend mitkompiliert ist |
| `?stats`       | Tabular      | -    | RX/TX-Counter pro Protokoll, Heap, kuenftig Uptime + Reset-Counters |

Beispiel:
```
?hmu.port           # 2325 (Legacy); Dual ist 2327
?hmu.hmid           # gerade aktive BidCos-Adresse
```

---

## Troubleshooting

### `device detection failed` in multimacd-Log

CULFW32 ist nicht erreichbar oder antwortet nicht im erwarteten Wire-Format.

```bash
# Erreichbarkeit:
ping 192.168.1.42
nc -uvz 192.168.1.42 3008    # UDP-3008 sollte open sein

# CULFW32-Frontend pruefen:
nc 192.168.1.42 2323
?build              # erwartet: enthaelt 'fe:hmuartlgw'
?wifi.state         # erwartet: 'connected'
```

CULFW32 ist auch direkt ueber TCP:2327 erreichbar — Smoke-Test unabhaengig von hb-rf-eth:
```bash
python3 -c "
import socket
s = socket.create_connection(('192.168.1.42', 2327))
hello = s.recv(64)
print(repr(hello))   # erwartet: b'\\xfd\\x00\\x10\\xfe\\x01\\xff\\xff\\x05HMIP_TRX_Bl…'
"
```

### Pair-Anzeige steht still bei `RECEIVED PAIR_ANNOUNCE`

multimacd hat den DEVICE_INFO-Broadcast empfangen, aber rfd hat die Folge-Frames (`CONFIG_REGISTER_PARAM`-Sequenz) nicht akzeptiert. Bis Build 0.1.148 lag das an einer Mismatch zwischen erwartetem on-air-AskSin-CNT und CULFW32-internem Counter — **gefixt in 0.1.149**: CULFW32 nutzt jetzt `opts[3]` der LLMAC_SEND als on-air-CNT, identisch zur DualCoPro-2.x-Firmware (siehe `hmuartlgw_phase4_done_2026-04-30.md`).

Wenn Build < 0.1.149: upgraden.

### Doppel-Hello-Push / Board-Crash unter rapid reconnect

Bis Build 0.1.152 gab es einen UAF/ABA-Bug im Hello-Timer: bei sehr schnellem connect/disconnect (FHEM-Reconnect-Schleife oder hb-rf-eth-stale-UDP) konnte derselbe Slot doppelt feuern oder das Board mit Store/Load-Access-Fault crashen. **Gefixt in 0.1.153** durch Pro-Slot-Timer-Tracking und Cleanup beim Detach.

Wenn Build < 0.1.153: upgraden.

### `bad ep-id N expected M` im UDP-Listener-Log

hb-rf-eth-Reconnect mit **alter Endpoint-ID** vom Pi-Tunnel. CULFW32 vergibt beim ersten Connect eine ep-id (z.B. 1); wenn der Pi nach Disconnect spaeter mit ep-id 3 kommt, lehnt CULFW32 die Pakete ab. Das ist erwartetes Verhalten — der Pi muss neu connecten:

```bash
echo "" | sudo tee /sys/class/hb-rf-eth/hb-rf-eth/connect    # disconnect
sleep 1
echo 192.168.1.42 | sudo tee /sys/class/hb-rf-eth/hb-rf-eth/connect
```

Bekannter Edge-Case: bei sehr stale-UDP-Packets aus altem Pi-Reboot kann CULFW32 selten panicken — Out-of-scope-Folge-Item, wird in v1.x adressiert.

### Geraet sendet, aber `multimacd`-Log zeigt nichts

CULFW32-RX-Sensitivitaet auf dem Test-Board ist ~14 dB schlechter als gut-angepasstes CUL868. Bei battery-powered HM-PB-/HM-RC-* mit kurzem Listening-Window kommt die Pair-Antwort nicht rechtzeitig.

Workarounds:
- Geraet **physisch naeher** an CULFW32 (10-30 cm)
- Pair mehrmals retriggern (Pair-Knopf am Geraet erneut druecken)
- Bei mains-powered HM-LC-* funktioniert es zuverlaessig auch auf 2 m
- Hardware-Spin (CULFW32-PCB mit ordentlichem BALUN + Antennenanpassung) loest's grundsaetzlich

### `/dev/raw-uart1` taucht nicht auf

```bash
lsmod | grep hb_rf_eth      # sollte 'hb_rf_eth' zeigen
sudo modprobe hb-rf-eth     # falls nicht
ls /sys/class/hb-rf-eth/    # sollte 'hb-rf-eth' Eintrag zeigen
```

Wenn das Modul nicht im Kernel ist (z.B. eigenes Custom-Linux): kompilieren aus https://github.com/alexreinert/hb-rf-eth.

---

## Wire-Protokoll Quick-Reference

Vollstaendige byte-genaue Spec siehe `hmuartlgw_wire_protocol_2026-05-01.md` im Memory-Layer. Hier nur die debmatic-relevanten Anteile.

### Frame-Wrapper (gemeinsam mit FHEM-Pfad)

```
0xfd <len_hi> <len_lo> <dst> <cnt> <payload...> <crc_hi> <crc_lo>

length = dst + cnt + payload (ohne CRC)
escape = 0xfc und 0xfd im Body durch 0xfc + (b ^ 0x80) ersetzt
crc16  = poly 0x8005, init 0xd77f, ueber [0xfd, len_h, len_l, dst, cnt, payload]
```

### Routing-DSTs (Dual-Mode = debmatic)

| DST  | Layer  | Bedeutung |
|------|--------|-----------|
| 0x00 | HMSYSTEM | OS-Level (gemeinsam mit Legacy) |
| 0x01 | APP    | nicht in Dual aktiv (Legacy-Pfad) |
| 0x02 | HMIP   | HmIP-Stack (HmIPServer.jar — out-of-scope fuer CULFW32) |
| 0x03 | LLMAC  | **Funklayer**: AskSin-Frames TX/RX |
| 0xFE | COMMON | Identify / Tick / Capabilities |

### Hello-Push beim Connect (Dual)

```
Direction:  CULFW32 → debmatic
Frame:     fd 00 10 fe 01 ff ff 05 'H' 'M' 'I' 'P' '_' 'T' 'R' 'X' '_' 'B' 'l' <crc>
                          ^                                                    ^
                          unsolicited push, Boot-Loader-Mode-Identifier        13-Byte Hello
```

debmatic erwartet diesen Push beim hb-rf-eth-Connect; ohne ihn beendet multimacd den Detect mit "no module".

### Init-Sequenz (debmatic-getriggert)

1. **CULFW32 → debmatic**: Hello-Push `HMIP_TRX_Bl` (s.o.)
2. **debmatic → CULFW32**: `COMMON_IDENTIFY` (dst=FE, cmd=01) → CULFW32 antwortet `[01 01]`+ASCII `DualCoPro_App` + 3-byte Version `02 08 06`
3. **debmatic → CULFW32**: `OS_GET_FIRMWARE` (dst=00, cmd=02) → `[04 02] + [00] + [02 08 06]` (FW 2.8.6)
4. **debmatic → CULFW32**: `OS_GET_SERIAL` (dst=00, cmd=0b) → `[04 02] + 10-Byte ASCII Serial` (z.B. `CUL168E9E5C`)
5. **debmatic → CULFW32**: `LLMAC_INIT_RADIO` (dst=03) → ACK
6. **debmatic → CULFW32**: `LLMAC_SET_HMID` (dst=03) → ACK; CULFW32 persistiert HMID in NVS
7. **debmatic → CULFW32**: `LLMAC_GET_HMID` (dst=03) → ACK + 3-byte HMID
8. (laufend) **OS_TICK**-Heartbeats von beiden Seiten

Danach ist CULFW32 in `OperationalMode::Dual::Active` und alle Funk-Frames werden via LLMAC_RECV (von Modul) bzw. LLMAC_SEND (zum Modul) ausgetauscht.

### LLMAC_SEND (debmatic → CULFW32, zum Senden auf Funk)

```
Frame:     fd <len> 03 <hostcnt> 02 00 00 <opts:4> <askSin: msg-len + msg-bytes> <crc>
                       ^         ^
                       cnt-Spiegel       cmd=02 (SEND), reserved 00 00
                                                ^
                                                opts[0..3]: flags fuer csma/burst/aes/...
                                                opts[3] = on-air AskSin-CNT (von multimacd diktiert!)
```

**Wichtig:** der on-air-AskSin-CNT (der CNT-Byte im Funk-Frame) ist NICHT der host-cnt aus dem 0xfd-Wrapper. multimacd encodiert den erwarteten on-air-CNT in `opts[3]`. CULFW32 muss den nehmen — Aktor-ACKs echoen Request-CNT zurueck (AskSinPP `Device::sendAck` wiederverwendet den msg-Buffer). Das war der entscheidende Fix in Build 0.1.149 fuer Phase-4.

### LLMAC_RECV (CULFW32 → debmatic, von Funk empfangen)

```
Frame:    fd <len> 03 <modcnt> 05 <ts_hi> <ts_lo> <info> <rssi_raw> <askSin: msg-len + msg-bytes> <crc>
                                  ^            ^             ^         ^
                                  16-bit ms-Timestamp          info-Flags     CC1101 RSSI raw
```

In Legacy-Mode (FHEM/Homegear) ist der Prefix `[05 status info rssi]` (3-byte ohne ms-Timestamp). Das ist der wesentliche Wire-Unterschied zwischen Legacy und Dual.

---

## Verifikations-Werkzeuge

### Frontend-Smoke-Test

```bash
cd firmware
python3 tools/release_smoke_test.py frontends
```

Erwartet 7/7 PASS — inklusive:
- `HMUARTLGW :2325 Hello Co_CPU_BL` (Legacy/FHEM)
- `HMUARTLGW :2325 OS_GET_APP -> Co_CPU_App` (Legacy Init-Probe)
- `HMUARTLGW :2327 Hello HMIP_TRX_Bl` (Dual/debmatic)
- `HMUARTLGW :2327 COMMON_IDENTIFY -> DualCoPro_App` (Dual Init-Probe)

### Live-Trace bei Verdacht

debmatic hat `dump_traffic` als Diagnose-Tool — schreibt UART-Bytes byte-genau in eine Datei:
```bash
sudo dump_traffic /dev/raw-uart1 /tmp/cul32-trace.bin &
# ... Action durchfuehren (Pair, schalten, etc.)
sudo killall dump_traffic
xxd /tmp/cul32-trace.bin | head -50
```

Damit sieht man `fd ...`-Wrapper byte-genau und kann gegen die Wire-Protokoll-Spec abgleichen.

### Schichten-Verifikations-Status

| Layer | Verifiziert gegen | Status |
|---|---|---|
| AskSin-Funkseite (CC1101 Reg + encrypt/decrypt) | a-culfw `rf_asksin.c` | ✅ Phase 2 (3-Wege gegen Lokales+Remote CUL868) |
| HMUARTLGW-0xfd-Wrapper Dual | echte HM-MOD-RPI-PCB mit DualCoPro 2.8.6 | ✅ byte-genau via `dump_traffic`-Reference |
| HM-LC-Sw1-Pl-2 Pair via debmatic | physikalisches Schalten via XML-RPC `setValue STATE` | ✅ count=55, ON/OFF wiederholbar |
| HM-PB-2-WM Pair via debmatic | DEVICE_INFO + autoCreate | ✅ Phase 3 (Test-Board-RX-Asymmetrie auf 2 m marginal) |
| HmIP-Linie | HmIPServer.jar | ⛔ out-of-scope, eigener Stack mit AES-CCM |

---

## Bekannte Einschraenkungen (v1)

- **AES** (Phase 4-AES) ist storage-only — `APP_SET_CURRENT_KEY/PREVIOUS_KEY/TEMP_KEY`-Frames werden plain ge-ACKed, Schluessel werden nicht persistiert oder zur Crypto verwendet. Bedeutet konkret: HM-Sec-* Geraete (alarm- und sicherheits-relevant) ACKen Steuerbefehle nicht (state=AES_KO im rfd). HM-LC/HM-PB/HM-CC/HM-WDS funktionieren voll. Phase 4-AES kommt in v1.x+1.
- **Peer-Tabelle** ist nicht persistent zwischen Reboots des Sticks. Beim CULFW32-Reboot muessen Geraete neu detected werden (rfd macht das automatisch in 60-120 s).
- **HmIP-Linie** komplett ausserhalb dieses Frontends. HmIP nutzt anderen Wire-Stack mit AES-CCM, eigenen Pair-Daemon (`HmIPServer.jar`, eq-3 closed-source). Kein Plan fuer Phase 5+.
- **LAN-Modus (HM-CFG-LAN)** mit ASCII-Wrapper + AES-Challenge auf Wire-Layer ist nicht implementiert und nicht geplant. CULFW32 implementiert nur den UART/UDP-Pfad.

---

## Quellen

- Alex Reinert hb-rf-eth (Kernel-Modul + sysfs-Steuerung): https://github.com/alexreinert/hb-rf-eth
- debmatic-Sources: https://github.com/alexreinert/debmatic
- eQ-3 OCCU (rfd, multimacd, BidCoS-Stack): https://github.com/eq-3/occu
- a-culfw rf_asksin.c (Funk-Layer): https://github.com/tostmann/a-culfw/blob/master/culfw/clib/rf_asksin.c
- DualCoPro 2.x-Firmware Reference (echte HM-MOD-RPI-PCB nach Flash): siehe Memory `hmuartlgw_dualcopro_real_reference_2026-04-30.md`
