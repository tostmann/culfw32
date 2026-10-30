# CULFW32 — Network guide

CULFW32 speaks the culfw protocol over several transports **at the same time**:

- **USB-CDC** (via the ESP32-C6 USB-C connector)
- **UART0** (via the SBU1/SBU2 pins of the USB-C socket, 38400 8N1) — for a neighboring microcontroller
- **TCP** (port `2323` over WiFi, optionally over W5500 LAN on future hardware)

Each sink gets its own input buffer; commands on USB and TCP do not collide. Answers to received commands go back exactly to the sink that sent the command. Spontaneous receive frames (`F…`/`A…`/`Z…`/…) go to all currently connected transports.

This file documents WiFi provisioning and TCP access.

---

## Web UI

As soon as WiFi is `connected`, a browser UI runs on port 80:

```
http://<ip>/                    # e.g. http://192.168.1.42/
http://<hostname>.local/        # e.g. http://culfw32-basement.local/
```

**"Config" tab:** RF frontend card at the top (Mode/Freq/Mod/Datarate/BW/Dev/PA-Idx + RSSI heat bar with position marker), below it the status (WiFi/IP/RSSI/SSID), form-generated settings (all ConfigRegistry entries, automatically mapped by type → input).

**"Terminal" tab:** native culfw console in the browser, `↑/↓` history (persistent in localStorage), `Ctrl+L` clears. All culfw commands work as on USB.

WebSocket endpoints:
- `ws://<ip>/ws/json` — typed JSON API (for your own tools/HASS bridges)
- `ws://<ip>/ws/tty` — ASCII console stream

## Quick start (TL;DR)

```
EWs MyWiFi                # set the SSID
EWp MyPassword            # set the password
EWm 1                     # mode 1 (STA, normal client)
E                         # check the status — waiting for "state=connected ip=…"
```

As soon as the status shows `connected`, the TCP server is reachable on port `2323`:

```sh
nc 192.168.1.42 2323     # take the IP from the E status
```

In the TCP stream, all culfw commands work exactly as on USB.

---

## WiFi provisioning

All WiFi commands start with `E` (the Ethernet/WiFi family from a-culfw, extended for WiFi in CULFW32 with `EW…` subcommands). The configuration is stored in a separate NVS namespace `culfw32_wifi` (separate from the culfw EE blob), so it survives reboots and `e` factory resets.

### `E` — compact status

```
> E
E connected mode=1 ssid=MyWiFi ip=192.168.1.42 rssi=-58
```

Fields:
- `state`: `off` | `disconnected` | `connecting` | `connected`
- `mode`: `0`=off · `1`=STA (client) · `2`=AP fallback · `3`=STA+AP at the same time
- `ssid`: currently configured SSID
- `ip`: current IPv4 address (only in the `connected` state)
- `rssi`: signal strength in dBm (only in the `connected` state)

### `EW` — full status (5 lines)

```
> EW
EW ssid=MyWiFi
EW psk=***
EW mode=1
EW host=culfw32-prod
EW state=connected ip=192.168.1.42 rssi=-58
```

The password is **never** shown in plain text — not even via `R` reads, because WiFi credentials live in a separate NVS namespace, not in the culfw EEPROM blob.

### `EWs<SSID>` — set the SSID

```
> EWs MyHomeWifi
```

At most 32 bytes (esp_wifi limit). Persistent. Triggers an immediate reconnect attempt if `mode != 0`.

### `EWp<password>` — set the password

```
> EWp <your-wifi-password>
```

At most 63 bytes. Persistent. Empty password = OPEN network. The output never echoes the password (only the `EW` acknowledgement with `***`).

### `EWm<N>` — mode

```
> EWm 0       # WiFi completely off
> EWm 1       # STA / client mode (default)
> EWm 2       # AP fallback: try STA first; open the AP after 30 s without a connection
> EWm 3       # STA+AP at the same time (both active immediately)
```

Mode 2/3 is useful when the home WiFi fails: the device stays reachable through its own AP (SSID = host name, OPEN network, channel 6).

### `EWh<hostname>` — mDNS host name

```
> EWh culfw32-basement
```

The default is `culfw32-XXXXXX` (the last 3 bytes of the MAC address). Takes effect on the network immediately — reachable from a Linux host as `culfw32-basement.local`:

```sh
ping culfw32-basement.local
nc culfw32-basement.local 2323
```

mDNS also announces the service type `_culfw._tcp` on port 2323. `avahi-browse` finds all CULFW32 devices on the network:

```sh
$ avahi-browse -r _culfw._tcp -t
+   eth0 IPv4 CULFW32                                       _culfw._tcp          local
=   eth0 IPv4 CULFW32                                       _culfw._tcp          local
   hostname = [culfw32-basement.local]
   address = [192.168.1.42]
   port = [2323]
```

### `EWS` — active scan

Blocks for ~4–6 seconds and lists all access points visible nearby:

```
> EWS
EWS aa:bb:cc:dd:ee:ff -65 ch=6 auth=3 MyWiFi
EWS 11:22:33:44:55:66 -78 ch=11 auth=3 NeighborWiFi
EWS done 2
```

Fields per line:
- BSSID (MAC of the base station)
- RSSI in dBm
- channel
- auth mode (`0`=OPEN, `3`=WPA2-PSK, …)
- SSID

### `EWP<port>` — change the TCP server port

```
> EWP 8080
EW port=8080
```

The TCP server rebinds to the new port live (the old listener is closed, existing client connections stay open). The value is stored **persistently** in EE_IP4_TCPLINK_PORT (a-culfw address 121, little-endian) and survives reboots.

```
> EW
…
EW port=8080 (ee=8080)
```

The display distinguishes between `port=` (the port currently bound by the TcpServer) and `(ee=…)` (the port persisted in EE). Normally both are the same; they can differ briefly if an `EWP` restart fails (e.g. port in use).

The default port after `e` (factory reset) is `2323` — the same value as a-culfw.

```sh
$ nc 192.168.1.42 8080         # after EWP 8080
V
V 0.1.0 CULFW32 Build: 1 …
```

Useful if port 2323 is blocked or in use in the network infrastructure (e.g. if another CUL already uses 2323 and both should be reachable at the same time).

### `EWA <ip>/<bits>` — IP allowlist (access protection)

```
> EWA 192.168.1.10/32        # only this host may connect
EW allow=192.168.1.10/32

> EWA 192.168.1.0/24         # only this subnet
EW allow=192.168.1.0/24

> EWA any                    # allowlist off, anyone may connect (default)
EW allow=any
```

On `accept()`, the TCP server checks the client IP against the configured allow subnet. On a mismatch, the connection is closed **immediately at accept** (the client sees `ECONNRESET`). Persistent in EE.

**Benefit:**
- **Protection in the LAN without configuration on the FHEM side**: if FHEM runs on a fixed server, `EWA <fhem-ip>/32` is the quickest access protection — guest WiFi, IoT VLAN and any port scanner are blocked completely, without changing FHEM.
- **Default case**: the allowlist is off by default (`any`). It is an active user decision — if you do not need it, you will not notice it.

**Limitations:**
- IP spoofing inside the LAN is possible (ARP spoofing). Typically sufficient in a home LAN; never use it as the only protection over the Internet.
- The allowlist applies **only** to TCP. USB and UART remain unrestricted — anyone with physical access can always control the device.

```
> EW
…
EW allow=192.168.1.10/32
```

The `EW` status query shows the active allowlist. For rejected connections, the ESP log shows a `denied <client-ip> (not in allow …)` line — visible via `idf.py monitor` during the bring-up phase.

### `EWx` — reset

```
> EWx
```

Deletes **all** WiFi credentials from the NVS namespace `culfw32_wifi` and ends the active connection. After `EWx` the state is as after the first boot: `mode=0`, empty SSID/PSK.

`EWx` does **not** affect the culfw EEPROM values (FHT ID, PA table, LED mode etc.). These are kept.

---

## TCP server

The server listens on `0.0.0.0:2323` (a-culfw-compliant, `EE_IP4_TCPLINK_PORT`). As soon as WiFi is `connected` (or the AP fallback is active), the server accepts connections. The listener also waits in the `disconnected` state and is available immediately once the connection is up.

Each connection gets its own LineBuf (max. 16 concurrent transports incl. USB+UART). TCP_NODELAY is set — small culfw answers go out without Nagle delay.

### Connecting with `nc` (netcat)

```sh
$ nc 192.168.1.42 2323
V
V 0.1.0 CULFW32 Build: 1 (Apr 28 2026) CUL868 (F-Band: 868MHz)
t
00012345
^D
```

`Ctrl-D` closes the TCP connection locally. The server cleans up immediately (the disconnect cleanup in CULFW32 has been verified to be race-free).

### Connecting from Python

```python
import socket
s = socket.create_connection(("192.168.1.42", 2323))
s.sendall(b"V\r\n")
print(s.recv(256).decode())
s.close()
```

### Connecting from FHEM

In `fhem.cfg`:

```
define myCUL CUL 192.168.1.42:2323 1234
```

(Port 2323 is the CULFW32 default; the `1234` is the usual FHEM pseudo FHEM ID value for culfw devices.)

Equally reachable via mDNS:

```
define myCUL CUL culfw32-basement.local:2323 1234
```

### `q` — close the connection from the server side

```
> q
```

CULFW32 accepts it only if the command arrives over a TCP connection (checked via the transport name, not via dynamic_cast). The server closes the TCP connection cleanly (recv = 0 bytes on the client side). On USB/UART, `q` is a no-op.

Useful as a programmed shutdown from the host, without TCP_RST and without TIME_WAIT accumulation.

---

## Modes in detail

### Mode 1 — STA (client)

Default case: CULFW32 connects to your home WiFi, gets an IP via DHCP, and is reachable via the IP or the mDNS host name.

Boot order: `init()` loads the config → if `mode==1 && ssid != ""` → `start()` initializes lwIP/esp_netif/esp_wifi → triggers the STA connect → on `WIFI_EVENT_STA_DISCONNECTED`, automatic reconnect.

### Mode 2 — AP fallback

First, `mode 1` is tried. If there is no connection after **30 seconds**, CULFW32 additionally switches on its own AP:

- SSID: current host name (e.g. `culfw32-basement`)
- Auth: OPEN (no password)
- Channel: 6
- IP: 192.168.4.1 (ESP-IDF default)

If the STA connection does come up later (for example because the home router was just restarting), the AP is switched off again automatically.

In practice, the device is never "lost". If you connect to the AP `culfw32-basement` (open), you can provision it again via `nc 192.168.4.1 2323` and then switch back to mode 1.

### Mode 3 — STA+AP at the same time

Both interfaces active. Useful for lab/bring-up, less so for production (an open AP runs permanently).

### Mode 0 — off

No radio traffic. CULFW32 remains reachable via USB/UART. Saves power.

---

## Architecture notes

- **WiFi credentials in a separate NVS namespace** (`culfw32_wifi`): SSID/PSK have variable length and are not meant to be readable from the EE blob via the culfw `R` command. Hence a separate namespace with its own keys.
- **Race-free reconfiguration**: SSID/PSK/mode changes go through `esp_wifi_stop() + esp_wifi_start()` (instead of `disconnect+connect`). On restart, `STA_START` fires and applies the fresh config — no race with the auto-reconnect handler.
- **mDNS idempotent**: `mdns_init` + service add run once in the `start()` path. Host name updates go through `mdns_hostname_set` without re-registering the service.
- **Multi-transport frontend**: renderers iterate over a mutex-protected snapshot of the transport list — no race during disconnect/attach.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `E` shows `state=connecting` permanently | Wrong password, or AP not reachable | `EWS` to scan, check RSSI; set `EWp <psk>` again |
| `nc <ip> 2323` does not connect | WiFi `disconnected`, or firewall, or **the allowlist blocks your IP** | check `E`; with `EW allow=<x>`, check whether your IP matches; if necessary `EWA any` |
| `Connection reset by peer` immediately on connect | Allowlist does not match | `EW` shows the current filter; `EWA <your-ip>/32` or `EWA any` |
| `avahi-browse _culfw._tcp` shows nothing | mDNS multicast is filtered by the router/switch | use the IP directly instead of the mDNS host name; check the router setting "IGMP Snooping" |
| No reconnect after `EWs <newSSID>` | mode was 0 | set `EWm 1` |
| Connection drops after reboot | LwIP TIME_WAIT pool full during rapid reconnect tests | irrelevant under real load (one permanent connection); if needed, raise `CONFIG_LWIP_MAX_ACTIVE_TCP` |

### Known limits

- **TLS**: none — port 2323 is plain text, a-culfw-compliant. Usually OK in the LAN; better not exposed directly to the Internet.
- **Concurrent TCP clients**: max. 16 transports in total (USB + UART + n × TCP). A typical setup has 1–2 TCP clients (FHEM + lab console) and is comfortably below the limit.
- **Authentication on the TCP port**: none — but the **IP allowlist via `EWA`** at least restricts *which* IPs may connect at all. In the LAN, this is in practice sufficient for most home environments.

---

## Example sequence: first setup over USB

```
> V
V 0.1.0 CULFW32 Build: 1 (Apr 28 2026) CUL868 (F-Band: 868MHz)

> EWS
EWS aa:bb:cc:dd:ee:ff -58 ch=6 auth=3 MyWiFi
EWS 11:22:33:44:55:66 -82 ch=11 auth=3 Neighbor
EWS done 2

> EWs MyWiFi
> EWp mysecretpassword
> EWm 1

> E
E connecting mode=1 ssid=MyWiFi ip=0.0.0.0 rssi=0

> E
E connected mode=1 ssid=MyWiFi ip=192.168.1.42 rssi=-58

> EWh culfw32-basement
> E
E connected mode=1 ssid=MyWiFi ip=192.168.1.42 rssi=-58
```

From this point on, the device is reachable at:

- `192.168.1.42:2323` (static, provided the DHCP lease is long enough)
- `culfw32-basement.local:2323` (mDNS, robust against IP changes)

USB can be unplugged — TCP+UART keep running independently.
