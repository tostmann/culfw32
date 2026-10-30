# CULFW32 — Netzwerk-Handbuch

CULFW32 spricht das culfw-Protokoll **gleichzeitig** über mehrere Transports:

- **USB-CDC** (über den ESP32-C6 USB-C-Anschluss)
- **UART0** (über die SBU1/SBU2-Pins der USB-C-Buchse, 38400 8N1) — für einen benachbarten Mikrocontroller
- **TCP** (Port `2323` über WiFi, optional über W5500-LAN bei künftiger Hardware)

Jede Senke bekommt einen eigenen Eingabepuffer; Kommandos auf USB und TCP kollidieren nicht. Antworten auf eingegangene Kommandos gehen genau zu der Senke zurück, die das Kommando geschickt hat. Spontane Empfangs-Frames (`F…`/`A…`/`Z…`/…) gehen an alle gerade verbundenen Transports.

Diese Datei dokumentiert WiFi-Provisioning und TCP-Zugriff.

---

## Web-UI

Sobald WiFi `connected` ist, läuft eine Browser-UI auf Port 80:

```
http://<ip>/                    # z.B. http://192.168.1.42/
http://<hostname>.local/        # z.B. http://culfw32-keller.local/
```

**Tab "Config":** HF-Frontend-Karte oben (Mode/Freq/Mod/Datarate/BW/Dev/PA-Idx + RSSI-Heat-Bar mit Position-Marker), darunter Status (WiFi/IP/RSSI/SSID), Form-generierte Settings (alle ConfigRegistry-Einträge automatisch nach Type → Input).

**Tab "Terminal":** native culfw-Console im Browser, `↑/↓` History (persistent in localStorage), `Strg+L` clear. Alle culfw-Befehle laufen wie auf USB.

WebSocket-Endpoints:
- `ws://<ip>/ws/json` — typisierte JSON-API (für eigene Tools/HASS-Bridges)
- `ws://<ip>/ws/tty` — ASCII-Console-Stream

## Schnelleinstieg (TL;DR)

```
EWs MeinWLAN              # SSID setzen
EWp MeinPasswort          # Passwort setzen
EWm 1                     # Modus 1 (STA, normaler Client)
E                         # Status prüfen — wartet auf "state=connected ip=…"
```

Sobald der Status `connected` zeigt, ist der TCP-Server auf Port `2323` erreichbar:

```sh
nc 192.168.1.42 2323     # IP aus dem E-Status nehmen
```

Im TCP-Stream funktionieren alle culfw-Kommandos genauso wie auf USB.

---

## WiFi-Provisioning

Alle WiFi-Kommandos beginnen mit `E` (Ethernet/WiFi-Familie aus a-culfw, in CULFW32 für WiFi erweitert via `EW…`-Subkommandos). Die Konfiguration wird in einem eigenen NVS-Namespace `culfw32_wifi` gespeichert (getrennt vom culfw-EE-Blob), übersteht also Reboots und `e`-Factory-Resets.

### `E` — Kompakt-Status

```
> E
E connected mode=1 ssid=MeinWLAN ip=192.168.1.42 rssi=-58
```

Felder:
- `state`: `off` | `disconnected` | `connecting` | `connected`
- `mode`: `0`=aus · `1`=STA (Client) · `2`=AP-Fallback · `3`=STA+AP gleichzeitig
- `ssid`: aktuell konfigurierte SSID
- `ip`: aktuelle IPv4-Adresse (nur im `connected`-Zustand)
- `rssi`: Signalstärke in dBm (nur im `connected`-Zustand)

### `EW` — Vollständiger Status (5 Zeilen)

```
> EW
EW ssid=MeinWLAN
EW psk=***
EW mode=1
EW host=culfw32-prod
EW state=connected ip=192.168.1.42 rssi=-58
```

Das Passwort wird **nie** im Klartext angezeigt — auch nicht über `R`-Reads, weil WiFi-Credentials in einem eigenen NVS-Namespace liegen, nicht im culfw-EEPROM-Blob.

### `EWs<SSID>` — SSID setzen

```
> EWs MyHomeWifi
```

Maximal 32 Bytes (esp_wifi-Limit). Persistent. Triggert sofortigen Reconnect-Versuch wenn `mode != 0`.

### `EWp<Passwort>` — Passwort setzen

```
> EWp <your-wifi-password>
```

Maximal 63 Bytes. Persistent. Leeres Passwort = OPEN-Netz. Output zeigt das Passwort niemals zurück (nur die `EW`-Quittung mit `***`).

### `EWm<N>` — Modus

```
> EWm 0       # WiFi komplett aus
> EWm 1       # STA / Client-Modus (Standard)
> EWm 2       # AP-Fallback: zuerst STA versuchen; nach 30 s ohne Verbindung AP aufmachen
> EWm 3       # STA+AP gleichzeitig (sofort beide aktiv)
```

Mode 2/3 ist nützlich, wenn das Heim-WLAN ausfällt: das Gerät bleibt über sein eigenes AP erreichbar (SSID = Hostname, OPEN-Netz, Channel 6).

### `EWh<Hostname>` — mDNS-Hostname

```
> EWh culfw32-keller
```

Default ist `culfw32-XXXXXX` (die letzten 3 Bytes der MAC-Adresse). Sofort im Netz wirksam — von einem Linux-Host aus erreichbar als `culfw32-keller.local`:

```sh
ping culfw32-keller.local
nc culfw32-keller.local 2323
```

mDNS announciert zusätzlich den Service-Typ `_culfw._tcp` auf Port 2323. Mit `avahi-browse` findet man alle CULFW32-Geräte im Netz:

```sh
$ avahi-browse -r _culfw._tcp -t
+   eth0 IPv4 CULFW32                                       _culfw._tcp          local
=   eth0 IPv4 CULFW32                                       _culfw._tcp          local
   hostname = [culfw32-keller.local]
   address = [192.168.1.42]
   port = [2323]
```

### `EWS` — Active Scan

Blockt ~4–6 Sekunden, listet alle in der Umgebung sichtbaren Access Points:

```
> EWS
EWS aa:bb:cc:dd:ee:ff -65 ch=6 auth=3 MeinWLAN
EWS 11:22:33:44:55:66 -78 ch=11 auth=3 NachbarWLAN
EWS done 2
```

Felder pro Zeile:
- BSSID (MAC der Basisstation)
- RSSI in dBm
- Kanal
- Auth-Modus (`0`=OPEN, `3`=WPA2-PSK, …)
- SSID

### `EWP<port>` — TCP-Server-Port ändern

```
> EWP 8080
EW port=8080
```

Der TCP-Server bindet sich live auf den neuen Port (alter Listener wird geschlossen, bestehende Client-Verbindungen bleiben offen). Der Wert wird **persistent** in EE_IP4_TCPLINK_PORT gespeichert (a-culfw-Adresse 121, Little-Endian) und überlebt Reboots.

```
> EW
…
EW port=8080 (ee=8080)
```

Die Anzeige unterscheidet zwischen `port=` (aktuell vom TcpServer gebundener Port) und `(ee=…)` (im EE persistierter Port). Im Normalfall sind beide gleich; sie können sich kurz unterscheiden, falls ein `EWP`-Restart fehlschlägt (z. B. Port belegt).

Default-Port nach `e` (factory-reset) ist `2323` — gleicher Wert wie a-culfw.

```sh
$ nc 192.168.1.42 8080         # nach EWP 8080
V
V 0.1.0 CULFW32 Build: 1 …
```

Sinnvoll, wenn Port 2323 in der Netz-Infrastruktur blockiert oder belegt ist (z.B. wenn ein anderer CUL bereits 2323 nutzt und beide gleichzeitig erreichbar sein sollen).

### `EWA <ip>/<bits>` — IP-Allowlist (Zugriffsschutz)

```
> EWA 192.168.1.10/32        # nur dieser Host darf connecten
EW allow=192.168.1.10/32

> EWA 192.168.1.0/24         # nur dieses Subnetz
EW allow=192.168.1.0/24

> EWA any                    # Allowlist aus, jeder darf connecten (Default)
EW allow=any
```

Beim `accept()` prüft der TCP-Server die Client-IP gegen die konfigurierte Allow-Subnet. Bei mismatch wird die Verbindung **sofort beim accept** geschlossen (Client sieht `ECONNRESET`). Persistent in EE.

**Mehrwert:**
- **Schutz im LAN ohne FHEM-seitige Konfiguration**: Wenn FHEM auf einem festen Server läuft, ist `EWA <fhem-ip>/32` der schnellste Zugriffsschutz — Gäste-WLAN, IoT-VLAN und beliebige Port-Scanner werden komplett blockiert, ohne dass FHEM angepasst werden muss.
- **Standardfall**: Allowlist ist per Default aus (`any`). User-aktive Entscheidung — wer's nicht braucht, merkt nichts.

**Limitationen:**
- IP-Spoofing innerhalb des LANs ist möglich (ARP-Spoofing). Im Heim-LAN typisch ausreichend; übers Internet niemals als alleinigen Schutz nutzen.
- Allowlist gilt **nur** für TCP. USB und UART bleiben uneingeschränkt — wer physischen Zugang hat, kann immer steuern.

```
> EW
…
EW allow=192.168.1.10/32
```

Die `EW`-Statusabfrage zeigt die aktive Allowlist. Bei abgelehnten Verbindungen kommt im ESP-Log eine `denied <client-ip> (not in allow …)`-Zeile — sichtbar über `idf.py monitor` während der Bringup-Phase.

### `EWx` — Reset

```
> EWx
```

Löscht **alle** WiFi-Credentials aus dem NVS-Namespace `culfw32_wifi` und beendet die aktive Verbindung. Nach `EWx` ist der Zustand wie nach erstem Boot: `mode=0`, leere SSID/PSK.

`EWx` betrifft **nicht** die culfw-EEPROM-Werte (FHT-ID, PA-Tabelle, LED-Mode etc.). Diese bleiben erhalten.

---

## TCP-Server

Der Server lauscht auf `0.0.0.0:2323` (a-culfw-konform, `EE_IP4_TCPLINK_PORT`). Sobald WiFi `connected` ist (oder AP-Fallback aktiv), nimmt der Server Verbindungen an. Der Listener wartet auch im `disconnected`-Zustand und ist sofort verfügbar, wenn die Verbindung steht.

Pro Verbindung wird ein eigener LineBuf angelegt (max 16 gleichzeitige Transports inkl. USB+UART). TCP_NODELAY ist gesetzt — kleine culfw-Antworten gehen ohne Nagle-Verzögerung raus.

### Verbindung über `nc` (netcat)

```sh
$ nc 192.168.1.42 2323
V
V 0.1.0 CULFW32 Build: 1 (Apr 28 2026) CUL868 (F-Band: 868MHz)
t
00012345
^D
```

`Ctrl-D` schließt die TCP-Verbindung lokal. Der Server räumt sofort auf (Disconnect-Cleanup ist in CULFW32 als Race-frei verifiziert).

### Verbindung aus Python

```python
import socket
s = socket.create_connection(("192.168.1.42", 2323))
s.sendall(b"V\r\n")
print(s.recv(256).decode())
s.close()
```

### Verbindung aus FHEM

In `fhem.cfg`:

```
define myCUL CUL 192.168.1.42:2323 1234
```

(Der Port 2323 ist der CULFW32-Default; das `1234` ist der FHEM-üblicher Pseudo-FHEM-ID-Wert für culfw-Geräte.)

Genauso erreichbar über mDNS:

```
define myCUL CUL culfw32-keller.local:2323 1234
```

### `q` — Verbindung serverseitig schließen

```
> q
```

Wird vom CULFW32 nur akzeptiert, wenn das Kommando über eine TCP-Verbindung kommt (wird via Transport-Name geprüft, nicht via dynamic_cast). Server schließt die TCP-Verbindung sauber (recv = 0 bytes auf Client-Seite). Auf USB/UART ist `q` ein No-Op.

Sinnvoll als programmierter Shutdown vom Host aus, ohne TCP_RST und ohne TIME_WAIT-Akkumulation.

---

## Modi im Detail

### Mode 1 — STA (Client)

Standardfall: CULFW32 verbindet sich mit deinem Heim-WLAN, holt eine IP per DHCP, ist über die IP oder den mDNS-Hostnamen erreichbar.

Boot-Reihenfolge: `init()` lädt Config → bei `mode==1 && ssid != ""` → `start()` initialisiert lwIP/esp_netif/esp_wifi → triggert STA-Connect → bei `WIFI_EVENT_STA_DISCONNECTED` automatischer Reconnect.

### Mode 2 — AP-Fallback

Erst wird `mode 1` versucht. Wenn nach **30 Sekunden** keine Verbindung steht, schaltet CULFW32 zusätzlich seinen eigenen AP an:

- SSID: aktueller Hostname (z.B. `culfw32-keller`)
- Auth: OPEN (kein Passwort)
- Kanal: 6
- IP: 192.168.4.1 (ESP-IDF Default)

Sobald die STA-Verbindung später doch noch zustande kommt (etwa weil das Heim-Router gerade neu gestartet ist), wird der AP automatisch wieder ausgeschaltet.

Praktisch: Das Gerät ist nie „verloren". Wenn du dich an den AP `culfw32-keller` (offen) anmeldest, kannst du via `nc 192.168.4.1 2323` neu provisionieren und dann auf Mode 1 zurückschalten.

### Mode 3 — STA+AP gleichzeitig

Beide Interfaces aktiv. Sinnvoll für Lab/Bringup, weniger für Produktion (offener AP läuft permanent).

### Mode 0 — Aus

Kein Funkverkehr. CULFW32 bleibt auf USB/UART erreichbar. Stromsparend.

---

## Architektur-Hinweise

- **WiFi-Credentials separater NVS-Namespace** (`culfw32_wifi`): SSID/PSK haben variable Länge und sollen nicht über das culfw-`R`-Kommando aus dem EE-Blob lesbar sein. Daher eigener Namespace mit eigenen Keys.
- **Race-freier Reconfigure**: SSID/PSK/Mode-Wechsel laufen über `esp_wifi_stop() + esp_wifi_start()` (statt `disconnect+connect`). Beim Re-Start fired `STA_START` und apply-d die frische Config — keine Race mit dem Auto-Reconnect-Handler.
- **mDNS idempotent**: `mdns_init` + Service-Add laufen einmalig im `start()`-Pfad. Hostname-Updates gehen über `mdns_hostname_set` ohne Service-Re-Registrierung.
- **Multi-Transport-Frontend**: Renderer iterieren über einen Mutex-geschützten Snapshot der Transport-Liste — keine Race während Disconnect/Attach.

---

## Troubleshooting

| Symptom | Ursache | Fix |
|---|---|---|
| `E` zeigt `state=connecting` permanent | Falsches Passwort, oder AP nicht erreichbar | `EWS` zum Scan, RSSI prüfen; `EWp <psk>` neu setzen |
| `nc <ip> 2323` connected nicht | WiFi `disconnected`, oder Firewall, oder **Allowlist sperrt deine IP** | `E` prüfen; bei `EW allow=<x>` prüfen ob deine IP passt; ggf. `EWA any` |
| `Connection reset by peer` sofort beim Connect | Allowlist trifft nicht zu | `EW` zeigt aktuellen Filter; `EWA <deine-ip>/32` oder `EWA any` |
| `avahi-browse _culfw._tcp` zeigt nichts | mDNS-Multicast wird vom Router/Switch gefiltert | IP direkt verwenden statt mDNS-Hostname; Router-Setting "IGMP Snooping" prüfen |
| Nach `EWs <neueSSID>` kein Reconnect | mode war 0 | `EWm 1` setzen |
| Verbindung bricht nach Reboot ab | LwIP TIME_WAIT-Pool voll bei Rapid-Reconnect-Tests | unter realer Last (eine permanente Verbindung) irrelevant; bei Bedarf `CONFIG_LWIP_MAX_ACTIVE_TCP` hochziehen |

### Bekannte Limits

- **TLS**: keiner — Port 2323 ist Klartext, a-culfw-konform. Im LAN üblicherweise OK; übers Internet besser nicht direkt exposen.
- **Gleichzeitige TCP-Clients**: max 16 Transports gesamt (USB + UART + n × TCP). Typisches Setup hat 1–2 TCP-Clients (FHEM + Lab-Console), liegt komfortabel unter dem Limit.
- **Authentifizierung am TCP-Port**: keine — aber **IP-Allowlist via `EWA`** beschränkt mindestens *welche* IPs überhaupt connecten dürfen. Im LAN reicht das praktisch für die meisten Heimumgebungen.

---

## Beispiel-Sequenz: Erst-Inbetriebnahme über USB

```
> V
V 0.1.0 CULFW32 Build: 1 (Apr 28 2026) CUL868 (F-Band: 868MHz)

> EWS
EWS aa:bb:cc:dd:ee:ff -58 ch=6 auth=3 MeinWLAN
EWS 11:22:33:44:55:66 -82 ch=11 auth=3 Nachbar
EWS done 2

> EWs MeinWLAN
> EWp meingeheimespasswort
> EWm 1

> E
E connecting mode=1 ssid=MeinWLAN ip=0.0.0.0 rssi=0

> E
E connected mode=1 ssid=MeinWLAN ip=192.168.1.42 rssi=-58

> EWh culfw32-keller
> E
E connected mode=1 ssid=MeinWLAN ip=192.168.1.42 rssi=-58
```

Ab diesem Punkt ist das Gerät unter:

- `192.168.1.42:2323` (statisch, sofern DHCP-Lease lange genug)
- `culfw32-keller.local:2323` (mDNS, robust gegen IP-Wechsel)

erreichbar. USB kann abgesteckt werden — TCP+UART laufen unabhängig weiter.
