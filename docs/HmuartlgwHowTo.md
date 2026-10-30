# CULFW32 — HMUARTLGW-Frontend (FHEM/Homegear/RaspberryMatic-Anbindung)

CULFW32 emuliert das **HM-MOD-RPI-PCB** — eQ-3's Funkmodul fuer Raspberry-Pi-CCU-Klone — ueber TCP. FHEM, Homegear, debmatic, piVCCU oder RaspberryMatic sehen CULFW32 als ganz normalen HomeMatic-Funkstick und reden auf dem 0xfd-binary HMUARTLGW-Protokoll mit ihm.

Wire-Protokoll: **0xfd-Frame-Wrapper, CRC-16 (poly 0x8005, init 0xd77f), Escape (0xfc + b^0x80), single-port TCP**. Identisch zum GPIO-direkt angeschlossenen HM-MOD-RPI-PCB; **nicht** der HM-CFG-LAN-Modus (anderes Protokoll mit ASCII-Wrapper + AES-Challenge — wird von CULFW32 nicht implementiert und ist nicht geplant).

> **Status 2026-04-28:** Phase 0..3 DONE byte-genau gegen FHEM-Docker + reale eQ-3 HomeMatic-Hardware:
> - **HM-LC-Sw1-Pl-2** (Steckdose, HMID `2580F4`, FW 0xA1 = v1.6, Serial `KEQ0958686`) voll-pairing + `set on/off` physisch verifiziert (LED leuchtet/aus)
> - **HM-PB-2-WM** (Wandtaster, HMID `19B5DF`, Serial `JEQ0064205`) DEVICE_INFO empfangen + autoCreate; Pair-Completion bei 2 m marginal wegen Test-Board-RX-Sensitivitaets-Asymmetrie (Hardware-Spin loest's)
> - **AES** (Phase 4) bleibt offen — siehe AES-Sektion unten

---

## Architektur

```
┌─────────────────────┐                  ┌──────────────────────────────┐
│ FHEM / Homegear /   │                  │ CULFW32 (ESP32-C6 + CC1101)  │
│ RaspberryMatic /    │  TCP :2325       │                              │
│ debmatic            │ ◄──── 0xfd ────► │  HmuartlgwFrontend           │
│                     │                  │       │                      │
│  CUL_HM/HM-Mod-     │                  │  RfManager                   │
│  Rpi-Pcb-Driver     │                  │       │                      │
│                     │                  │  AskSinRF  (Slot 0)          │
└─────────────────────┘                  │       │                      │
                                         │  CC1101 868.299 MHz GFSK     │
                                         │       │                      │
                                         │       └─ Antenne ── Funk     │
                                         └──────────────────────────────┘
                                                                │
                                                        ┌───────┴───────┐
                                                        │               │
                                                  HM-LC-Sw1-Pl-2   HM-PB-2-WM
                                                  HMID 2580F4      HMID 19B5DF
```

Auf der Funkseite ist CULFW32 byte-genau a-culfw `rf_asksin.c` (verifiziert in Phase 2 gegen Lokales- und Remote-CUL868). Die Wrapper-Schicht ueber dem AskSin-Stack ist **clean-room gegen FHEM `00_HMUARTLGW.pm` reimplementiert** — `a-culfw` selbst spricht das HMUARTLGW-Protokoll nicht (das ist eQ-3-firmware-spezifisch, closed-source binary).

---

## Voraussetzungen

- CULFW32-Image mit `CONFIG_CULFW32_BUILD_FRONTEND_HMUARTLGW=y` und `CONFIG_CULFW32_BUILD_RF_ASKSIN=y` (beides Default in den Standard-Builds)
- WiFi verbunden (siehe [`NetworkHowTo.md`](NetworkHowTo.md))
- TCP-Port (Default 2325) erreichbar vom Host der die Anbindung uebernimmt
- FHEM mit `00_HMUARTLGW.pm` **oder** Homegear mit `Homegear-HomeMaticBidCoS` **oder** debmatic / piVCCU / RaspberryMatic mit `rfd`

---

## FHEM — End-to-End Setup-Walkthrough

Der getestete Pfad. Ergebnis: HomeMatic-Geraet pairt sich, Steuerbefehle gehen byte-genau on-air.

### 1. CULFW32 IP herausfinden

Auf der CULFW32-ASCII-Konsole (USB-CDC oder per `nc <ip> 2323`):
```
?wifi.ip
? wifi.ip = 192.168.1.42  (str, ro)
```

### 2. FHEM-Define mit `uart://`-Prefix

Im FHEM-Webfrontend in der Kommandozeile:

```fhem
define myhm HMUARTLGW uart://192.168.1.42:2325
attr   myhm hmId F11234
attr   myhm verbose 5
```

> **Der `uart://`-Prefix ist Pflicht** — ohne ihn waehlt FHEM den HM-CFG-LAN-Modus (ASCII-Wrapper + AES-Challenge auf Port +1). Das ist ein anderes Protokoll, von CULFW32 nicht implementiert.

`hmId` ist die 3-Byte-CCU-Adresse, mit der CULFW32 sich gegenueber HomeMatic-Geraeten als CCU meldet. Frei waehlbar — am besten eine Hex-Zahl, die nicht mit existierenden Geraete-HMIDs kollidiert (z.B. `F11234`). Nach erstmaligem Setzen schreibt FHEM diese ID via `APP_SET_HMID` ins CULFW32-EE; persistiert ueber Reboot.

### 3. Init-Sequenz beobachten

Innerhalb von ~5 s sollte FHEM die 14-Schritt Init-Sequenz durchlaufen. `list myhm` zeigt:

```
Internals:
   model       HM-MOD-UART        ← FHEM erkennt CULFW32 als HM-MOD-UART-Klon
   STATE       opened
   Initialized 1
Readings:
   cond        ok
   loadLvl     low
   D-firmware  1.4.1
   D-serialNr  CULAABBCC          ← MAC-derived (von der WiFi-MAC der CULFW32)
   D-HMIdAssigned   34            ← (= 0xF11234, FHEM strippt fuehrende Nullen)
   D-HMIdOriginal   34
   D-type      HM-MOD-UART
```

### 4. Pair-Window oeffnen + Geraet anlernen

```fhem
set myhm hmPairForSec 300
```

Dann am HomeMatic-Geraet **Pair-Knopf >4 Sekunden druecken** (Factory-Reset → Geraet vergisst den alten CCU-Master), kurz warten, dann **Pair-Knopf kurz druecken** (sendet DEVICE_INFO-Broadcast).

FHEM autocreate erstellt das Geraet:
```
2026.04.28 23:19:29 CUL_HM pair: HM_2580F4 switch, model HM-LC-SW1-PL2 serialNr KEQ0958686
```

### 5. Schalten

```fhem
set HM_2580F4 on
set HM_2580F4 off
```

Die Steckdose schaltet, FHEM-Reading `level=100/0`, `state=on/off`.

### 6. Optional: Cross-Sniff via zweiten CUL-Stick

Mit einem zweiten USB-CUL (CUL868) parallel im `Ar`-Mode kann man die Funk-Frames byte-genau sehen:
```
A 10 33 80 02 2580F4 F11234 01 01 C8 00 ::-42:myhm
              ↑   ↑     ↑                ↑   ↑   ↑
            msg_nr  ACK-type            ch  state level
```

---

## Homegear / debmatic / piVCCU / RaspberryMatic — `socat`-Bridge

Die eQ-3-OCCU-Tools (`rfd`, `HmIPServer.jar` — letzteres nur fuer HmIP-Linie, nicht relevant fuer HomeMatic-classic-CULFW32) und Homegear erwarten ein lokales Charakter-Device wie `/dev/raw-uart` oder `/dev/ttyAMA0`. CULFW32 spricht TCP. Bridge:

### Auf dem CCU-Host

```bash
sudo socat pty,link=/dev/cul32-uart,raw,echo=0,mode=666 \
           tcp:192.168.1.42:2325,reuseaddr,keepalive
```

Hat den Effekt: alle Bytes auf TCP↔CULFW32 erscheinen 1:1 auf `/dev/cul32-uart`. Latenz +1-3 ms — fuer HMUARTLGW unerheblich.

### Homegear-Konfig (`/etc/homegear/families/homematicbidcos.conf`)

```ini
[CULFW32-via-TCP]
id = CULFW32
deviceType = hm-mod-rpi-pcb
device = /dev/cul32-uart
responseDelay = 95
gpio1 = 0    # kein RST_N-GPIO weil TCP-Bridge — Homegear ignoriert dann
```

Dann `homegear restart` und in der Homegear-CLI:
```
hg> families list
hg> peers list
```

### RaspberryMatic / debmatic / piVCCU

Setze `/etc/config/cul32-bridge.service` (systemd-Unit) auf, dass der `socat`-Befehl beim Boot startet, und konfiguriere `rfd` so dass es `/dev/cul32-uart` statt der eingebauten `/dev/ttyS<n>` nutzt. `rfd.conf` Eintrag analog zu Homegear oben.

> **Status 2026-04-28/29**: Homegear- und rfd-Tests sind **noch nicht durchgefuehrt** (Lab-Folge-Item — siehe `hmuartlgw_status.md`). Source-Inspection von Homegear `Hm-Mod-Rpi-Pcb.cpp`:
>
> - Homegear erwartet beim Connect einen **unsolicited Push** des `Co_CPU_App`-Frames (passive listen + 5 s Timeout). FHEM dagegen sendet aktiv `OS_GET_APP` und reagiert auf Antwort.
> - **Erste Implementierungs-Iteration mit Hello-Push** (Frame ans frische TCP-Socket schreiben sobald accept() durchgeht) hat FHEM in `cond=disconnected` getrieben — vermutlich kommen die Bytes vor FHEMs eigener Init-Setup-Komplettierung an. Push ist deshalb **deaktiviert**; OS_GET_APP-Active-Probe-Pattern (FHEM) funktioniert byte-genau.
> - Fuer Homegear-Bringup wird voraussichtlich ein **delayed Hello-Push** noetig (z.B. 200 ms nach attach, damit FHEM seinen Setup beendet hat) — oder eine clientseitige Heuristik. Wird zur Bringup-Zeit gegen echtes Homegear/rfd verifiziert.
>
> Byte-Format-Unterschiede zwischen `00_HMUARTLGW.pm` und `Hm-Mod-Rpi-Pcb.cpp` sind klein (gleiche Frame-Struktur, gleiche CRC, beide akzeptieren `[0x00] + "Co_CPU_App"`-Antwort).

---

## ConfigRegistry-Schluessel

Auf der CULFW32-ASCII-Konsole oder via Web-UI / JSON-Frontend:

| Key | Type | Beschreibung |
|---|---|---|
| `hmu.port` | U16, persist | Listen-Port. Default 2325. Persistiert in EE 0x123 (LE). Aenderung **restartet** den TCP-Listener live. |
| `hmu.hmid` | Str (read-only) | Aktuelle 3-Byte HMID als 6-Hex (z.B. `F11234`). FHEM setzt sie via `APP_SET_HMID`. |
| `?build` | Str (read-only) | enthaelt `fe:hmuartlgw` wenn das Frontend mit-kompiliert ist |
| `?stats` | Tabular | `asksin rx N/M (B)` — RX/TX-Counter pro Protokoll, hilfreich fuer Debug |

```
?hmu.port           # aktuellen Port abfragen
?hmu.port=2400      # Port setzen, Server bindet sofort um
?hmu.hmid           # gerade aktive BidCos-Adresse
```

Die HMID kann **nicht direkt** auf CULFW32 gesetzt werden — FHEM/Homegear ist die autorisierte Quelle und schreibt sie via `APP_SET_HMID`. Nach Reboot bleibt die letzte gesetzte HMID erhalten.

---

## AES (Phase 4, v1.x+1)

HomeMatic-classic kennt zwei Sicherheitsklassen — und CULFW32 v1 deckt nur die unsignierte ab. Pragmatisch ist das in den allermeisten Hausautomatisierungs-Setups zwar nicht-blockend, aber bei Sicherheitstechnik kritisch.

### AES-Mechanik bei HomeMatic

Die "secure"-Variante des AskSin-Protokolls ist Challenge-Response auf Frame-Layer:

1. CCU sendet ein Steuerkommando an Peer
2. Peer antwortet mit AES-Challenge (6 Random-Byte)
3. CCU berechnet AES-128(key, challenge ⊕ original-msg) und sendet Response
4. Peer prueft AES-Response gegen seinen lokalen Schluessel; bei Match wird das Kommando ausgefuehrt

Pro Peer-Beziehung wird in der Master-List-0 ein Bit gesetzt, ob AES zwingend erforderlich ist. CCU verwaltet die Schluessel und FHEM uebermittelt sie ans Modul via `APP_SET_CURRENT_KEY` / `APP_SET_PREVIOUS_KEY` / `APP_SET_TEMP_KEY` (17 Byte je: 1 Byte Index + 16 Byte AES-Key).

CULFW32 v1 ACKt diese Befehle aber **speichert die Schluessel nicht** — Phase 4 (mbedtls/aes auf der C6-HW-AES-Beschleunigung) ist die Folge-Iteration.

### Welche Geraete brauchen AES?

| Geraete-Klasse | AES per Default | Bedeutung ohne AES |
|---|---|---|
| **HM-Sec-** (Sicherheits-Linie) — `HM-Sec-SC-2`, `HM-Sec-MD`, `HM-Sec-Key`, `HM-Sec-Win`, `HM-Sec-SD`, `HM-Sec-Sir-WM`, `HM-Sec-RHS` | **Mandatory** | Geraet ACKt Steuerbefehle nicht; FHEM-Reading `state=AES_KO` |
| **HM-LC-** (Licht/Switch) — `HM-LC-Sw*-Pl*`, `HM-LC-Dim*`, `HM-LC-Bl*` | **Optional** (off) | Voll funktional |
| **HM-PB-** / **HM-RC-** (Wand-/Fern-Taster) | **Optional** (off) | Voll funktional |
| **HM-CC-** (Heizungs-Thermostate) — `HM-CC-RT-DN`, `HM-CC-TC` | **Optional** (off) | Voll funktional |
| **HM-WDS-** (Wetter-Sensoren) | **Optional** (off) | Voll funktional |
| **HM-Dis-** / **HM-LGW-** (Display/Gateway) | gemischt | meist optional |
| **HmIP** (HomeMatic IP — neuer Protokoll-Stack) | **Mandatory** auf L1 (CCM) | komplett anderer Stack — kein klassisches HMUARTLGW-AES, eigene Komponente erforderlich |

CULFW32 v1 ohne AES bedient damit die **ueberwiegende Mehrheit** der HomeMatic-classic-Heimautomations-Geraete. HM-Sec-Geraete und HmIP-only-Setups brauchen Phase 4+.

### Was FHEM macht ohne IODev-AES

FHEM `00_HMUARTLGW.pm` toleriert AES-KO **per-Peer**: bei einem AES-required Geraet zeigt `cond=ok` weiter, das einzelne Geraet bekommt aber `state=AES_KO`. Der Rest des Setups laeuft normal weiter — kein crash, kein global lockout.

---

## Troubleshooting

### `cond=disconnected` bleibt nach `define`

- Erreichbarkeit pruefen: `ping <culfw32-ip>` und `nc -zv <culfw32-ip> 2325` vom FHEM-Host aus
- Firewall? CULFW32 hoert auf `0.0.0.0:2325`, FHEM-Host muss raus duerfen
- `attr myhm verbose 5` und `tail -f fhem.log` — erste Zeile sollte `myhm device opened` sein
- CULFW32 selbst pruefen: `?hmu.port` (sollte `2325` zeigen), `?wifi.state` (sollte `connected`)

### `cond=ERROR-Overload`

CULFW32 antwortet auf `OS_GET_CREDITS` mit Load > 100% (= byte > 0x64). Sollte nicht auftreten — Bug-Report:
- `?stats` auf der ASCII-Konsole holen
- FHEM-Verbose-Log mit `read raw`-Zeilen
- Bug-Report an `tostmann@gmail.com` mit beidem

### `cond=ok`, aber `Initialized` bleibt 0

Init haengt mitten drin. FHEM-Verbose-Log mit `state ` durchsehen — der hoechste erreichte State zeigt wo's klemmt:
- `state 1` (QUERY_APP) haengt → unsolicited Push wird nicht akzeptiert; Re-Probe via `set myhm reopen`
- `state 7-9` (SET_TIME, GET_FIRMWARE, GET_SERIAL) haengt → Bug in CULFW32-Antwortformat
- `state 14-16` (SET_*_KEY) → AES-Keys werden geladen; bei CULFW32 plain-ACKed (kein Bug)

### Geraet pairt aber `STATE=MISSING ACK` / `RESPONSE TIMEOUT`

Das CULFW32-Test-Board hat **~14 dB schwaechere RX-Sensitivitaet** als ein gut-angepasster CUL868 (BALUN-loser Stub am C6-Mini-Board). Bei battery-powered HM-PB-2-WM/HM-RC-* mit kurzem Listening-Window: Pair-Antwort kommt nicht rechtzeitig.

Workarounds:
- Geraet **physisch naeher** an CULFW32 bringen (10-30 cm)
- Pair mehrmals retriggern (`set myhm hmPairForSec 300`, dann Pair-Knopf-Druck)
- Bei mains-powered HM-LC-* funktioniert es zuverlaessig auch auf 2 m
- Hardware-Spin (CULFW32-PCB mit ordentlichem BALUN + Antennenanpassung) loest's grundsaetzlich

### Frames erscheinen nicht in FHEM, aber CUL868-Sniffer sieht sie

Selbe Diagnose: RX-Sensitivitaet. CUL868 hoert mit, CULFW32 liegt unter dem Empfindlichkeits-Floor an dem Standort. Geraet naeher ranbringen oder Hardware-Spin abwarten.

---

## Verifikations-Werkzeuge

### Test-Tools im Repo

`firmware/test/hmuartlgw/`:
- `frame_parser.py` — Frame-Wrapper als Python-Reference (CRC, Escape, Encode/Decode). Roundtrip-test passed.
- `trace_listener.py --mode silent|stub-ack` — TCP-Stub-Server, der die FHEM-Init-Sequenz byte-genau beantwortet **ohne** echte CULFW32-Hardware. Gut fuer FHEM-Setup-Test.
- `traces/init_sequence_REFERENCE.log` — der byte-genaue 14-Schritt-Trace einer FHEM-init gegen den Stub.
- FHEM-Referenz `00_HMUARTLGW.pm`: https://svn.fhem.de/trac/browser/trunk/fhem/FHEM/00_HMUARTLGW.pm

Empfehlung: vor dem Erstanschluss CULFW32-Hardware testen ob der Setup-Pfad ueberhaupt klappt:
```bash
# auf dem FHEM-Host:
cd firmware/test/hmuartlgw
python3 trace_listener.py --mode stub-ack --port 2325 &
# in FHEM:
define teststub HMUARTLGW uart://localhost:2325
# → sollte sofort cond=ok / Initialized=1 zeigen
```

### Schichten-Verifikations-Status

| Layer | Gegen was byte-genau? | Status |
|---|---|---|
| ASCII-Frontend `A`/`Z`/`F`/`T`/etc. | `a-culfw clib` (Original) | ✅ verifiziert (Phase 2 Bringup) |
| AskSin-Funkseite (CC1101 Reg + encrypt/decrypt) | `a-culfw rf_asksin.c` | ✅ verifiziert (3-Wege gegen Lokales+Remote CUL868) |
| HMUARTLGW-0xfd-Wrapper | FHEM `00_HMUARTLGW.pm`-Erwartung | ✅ verifiziert (14 Init-Schritte byte-genau, real-Pair Steckdose) |
| HMUARTLGW gegen reales eQ-3-Modul (rfd) | direkter Vergleich gegen HM-MOD-RPI-PCB-Firmware | ⚠ **noch nicht** — Folge-Test wenn HW + RaspberryMatic-Setup im Lab |
| HMUARTLGW gegen Homegear-HomeMaticBidCoS | `Hm-Mod-Rpi-Pcb.cpp` (Open-Source) | ⚠ **noch nicht** — Folge-Test, plausibel kompatibel |

---

## Bekannte Einschraenkungen (v1)

- **AES**: storage-only, keine Crypto-Verarbeitung (siehe oben — Phase 4 v1.x+1).
- **Wandtaster-Pair** auf >1 m mit dem Test-Board marginal — Hardware-Spin (BALUN+ordentliche Antennenanpassung) loest's. Mains-powered Geraete zuverlaessig auch auf 2 m.
- **Peer-Tabelle** ist nicht persistent — pro FHEM-Reconnect leer. Funktioniert ohne, weil AskSinRF promiscuous laeuft und FHEM-CUL_HM dst-side filtert. Persistenz waere ein nice-to-have fuer schnelleren Reconnect.
- **HMUARTLGW-LAN-Modus** (HM-CFG-LAN, ASCII-Wrapper, AES-Challenge auf Wire-Layer) ist nicht implementiert und nicht geplant. Wer eine HM-CFG-LAN-kompatible Box braucht: bestehender CUL868 + eigener `socat`-Bridge ist der Pfad.
- **HmIP-Linie** (HomeMatic-IP) komplett ausserhalb dieses Frontends. HmIP nutzt anderen Wire-Stack mit AES-CCM, eigenen Pair-Daemon (`HmIPServer.jar`, eq-3 closed-source). Kein Plan fuer Phase 5+.

---

## Wire-Protokoll-Quick-Reference

Vollstaendige Spec siehe Memory `hmuartlgw_protocol_spec.md`. Hier nur die Kernfakten.

### Frame-Layout
```
0xfd <len_hi> <len_lo> <dst> <cnt> <payload...> <crc_hi> <crc_lo>
                        │      │
                        │      └─ Counter-Byte. Host-Sends inkrementieren;
                        │         Modul spiegelt im ACK. Module-Sends
                        │         (unsolicited APP_RECV) eigener counter.
                        └─ 0=OS, 1=APP, (254=DUAL, 255=DUAL_ERR — DualCoPro/HmIP)
length = dst+cnt+payload bytes (ohne CRC).
escape = 0xfc und 0xfd im Body werden durch 0xfc + (b ^ 0x80) ersetzt.
crc16  = poly 0x8005, init 0xd77f. Berechnet ueber [0xfd, len_h, len_l, dst, cnt, payload].
```

### OS-Layer-Befehle (dst=0)
| Code | Name | Antwort |
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

### APP-Layer-Befehle (dst=1)
| Code | Name | Bedeutung |
|---|---|---|
| 0x00 | APP_SET_HMID | Host setzt 3-byte HMID |
| 0x01 | APP_GET_HMID | Antwort `[0x04, 0x07] + 3-byte HMID` (ACK_WITH_MULTIPART_DATA) |
| 0x02 | APP_SEND | TX-Frame: `[02 00 00 mode] + msg_nr + flags + mtype + src(3) + dst(3) + payload` |
| 0x03 | APP_SET_CURRENT_KEY | 17 byte (1-byte index + 16-byte AES-key) |
| 0x05 | APP_RECV | RX-Frame (vom Modul ungefragt): `[0x05, status, info, RSSI_raw] + msg_nr + flags + mtype + src + dst + payload` |
| 0x10 | APP_DEFAULT_HMID | Antwort `[0x04, 0x07] + 3-byte HMID` |

ACK-Codes: 0x01=OK, 0x02=ACK_INFO, 0x05=ENO_CREDITS (LOVF), 0x06=ECSMACA, 0x07=ACK_WITH_MULTIPART_DATA, 0x0c=AES_OK, 0x0d=AES_KO.

### Init-Sequenz (FHEM-getriggert nach TCP-Connect)

1. `OS_GET_APP` → wir antworten `[0x00] + "Co_CPU_App"` (= App ist aktiv)
2. `APP_GET_HMID` → wir antworten mit HMID aus EE
3. `APP_DEFAULT_HMID` → wir antworten mit Original-HMID (selbe wie SET-HMID)
4. `OS_SET_TIME` → ACK
5. `OS_GET_FIRMWARE` → wir antworten 1.4.1 (nicht-outdated Floor)
6. `OS_NORMAL_MODE` → ACK
7. `OS_GET_SERIAL` → wir antworten "CUL<MAC>"
8. `OS_ENABLE_CSMACA` → ACK
9. `OS_ENABLE_CREDITS` → ACK
10. `OS_GET_CREDITS` → wir antworten 0% (idle)
11. `APP_SET_CURRENT_KEY` → ACK (Schluessel aktuell unverarbeitet)
12. `APP_SET_PREVIOUS_KEY` → ACK
13. `APP_SET_TEMP_KEY` → ACK

Danach `cond=ok`, `Initialized=1`. Modul ist betriebsbereit.

---

## Quellen

- FHEM `00_HMUARTLGW.pm`: https://github.com/mhop/fhem-mirror/blob/master/fhem/FHEM/00_HMUARTLGW.pm
- Homegear-HomeMaticBidCoS Hm-Mod-Rpi-Pcb.cpp: https://github.com/Homegear/Homegear-HomeMaticBidCoS/blob/master/src/PhysicalInterfaces/Hm-Mod-Rpi-Pcb.cpp
- eQ-3 OCCU: https://github.com/eq-3/occu
- alexreinert HB-RF-ETH (TCP-Bridge zum HM-MOD): https://github.com/alexreinert/HB-RF-ETH
- a-culfw rf_asksin.c (Funk-Layer): https://github.com/tostmann/a-culfw/blob/master/culfw/clib/rf_asksin.c
