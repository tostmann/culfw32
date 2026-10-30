// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "esp_err.h"
#include "esp_event_base.h"
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace culfw32 {

// Persisted WiFi-Credentials (NVS-Namespace `culfw32_wifi`).
// Bewusst NICHT im culfw-EE-Blob (variable-laenge SSID/PSK, PSK-Leak via R-Befehl).
struct WifiConfig {
    std::string ssid;          // up to 32 bytes (esp_wifi limit)
    std::string psk;            // up to 63 bytes; 0 length = open network
    std::string hostname;       // mDNS-Name; default culfw32-XXXXXX (MAC-suffix)
    // 0=off, 1=STA, 2=STA mit AP-Fallback nach 30 s ohne Connect, 3=STA+AP.
    // Der Provisioning-AP startet auf Kanal 6 und folgt dem Kanal der Station,
    // sobald diese verbunden ist (IDF: Station hat Vorrang). In Modus 3 geht er
    // vor dem Connect hoch und wird nach dem Connect (bzw. nach 30 s ohne)
    // erneut sichergestellt, weil die Improv-Lib ihn beim Boot abschaltet.
    uint8_t     mode = 0;
};

struct WifiScanResult {
    std::string ssid;
    uint8_t     bssid[6];
    int8_t      rssi;
    uint8_t     channel;
    uint8_t     auth_mode;       // wifi_auth_mode_t cast
};

class WifiManager {
public:
    // Singleton-Lebensdauer = Prozess-Lebensdauer; der Dtor schliesst nur den
    // einmalig in init() geoeffneten NVS-Handle sauber (kein Driver-Teardown).
    ~WifiManager();

    // NVS oeffnen + Config laden. Idempotent.
    esp_err_t init();

    // STA starten und connect anstoßen falls mode==1 und SSID gesetzt.
    // Blockiert nicht — Verbindung kommt asynchron via Event.
    esp_err_t start();

    // Setter (jeder persistent — speichert sofort in NVS).
    esp_err_t set_ssid(std::string_view s);
    esp_err_t set_psk(std::string_view s);
    esp_err_t set_mode(uint8_t mode);
    esp_err_t set_hostname(std::string_view s);

    // Atomarer Setter fuer SSID+PSK+Mode OHNE den Driver anzufassen. Pfad
    // fuer externe Provisioner (Improv-Lib's WiFiBackend), die den Driver
    // bereits in den Soll-Zustand gebracht haben — wir lassen die laufende
    // Connection in Ruhe und persistieren nur NVS+cfg_, damit nach Reboot
    // oder spaeterem STA_DISCONNECTED unsere Pfade die neuen Credentials
    // nutzen. Wenn WifiManager noch nicht gestartet war (mode war 0), wird
    // start() angestossen, damit Event-Handler/mDNS/AP-Fallback live sind.
    // Rueckgabe: Ergebnis von NVS-Schreiben + Commit (nicht von start()) --
    // der Improv-Glue startet nach der Ersteinrichtung nur damit neu.
    esp_err_t set_persisted_creds(std::string_view ssid,
                                   std::string_view psk,
                                   uint8_t mode);

    // Auto-Reconnect-Guard. Solange `on=true`, ueberspringt unser
    // STA_DISCONNECTED-Handler den `esp_wifi_connect()`-Aufruf — sonst
    // wuerde er mit Improv-Lib's `tryConnect()` um `esp_wifi_set_config()`
    // konkurrieren und der Driver returnt `sta is connecting`, was die Lib
    // als sofortiges UNABLE_TO_CONNECT meldet. Aufrufer (Improv-Task) setzt
    // das mit dem ersten Improv-Rahmen auf true und beim Window-Close auf
    // false; ohne Improv-Verkehr bleibt der Auto-Retry frei. Beim Setzen
    // bricht es einen laufenden, noch nicht assoziierten Connect-Versuch ab
    // (esp_wifi_disconnect), damit Suche und tryConnect der Lib den Treiber
    // frei vorfinden; das Disconnect-Ereignis dazu kommt (gemessen 19.10.2026:
    // nach einer reinen Suche am Fensterende Reconnect und Auto-Retry).
    // external_control_ wird vom Event-Task gelesen (STA_DISCONNECTED) und vom
    // Improv-Task geschrieben → atomic, relaxed reicht fuer ein reines Flag.
    // Beim Freigeben (on=false) schiebt es einen Connect nach, wenn die Station
    // getrennt ist: ein Disconnect waehrend des Fensters fiel in die Sperre, ein
    // weiteres Event kommt danach nicht -- der Stick blieb sonst bis zum
    // Neustart offline (gemessen 11.10.2026: erster Connect in Modus 3 scheiterte
    // mit NO_AP_FOUND, danach kein Versuch mehr).
    void set_external_control(bool on);

    // Wipe: NVS-Namespace leeren; laeuft WiFi, die Station trennen und AP samt
    // Fallback-Timer abschalten (Zustand wie nach einem Boot in Modus 0, der
    // Treiber bleibt an).
    esp_err_t wipe();

    const WifiConfig& config() const { return cfg_; }

    // Status: "connected"/"connecting"/"disconnected"/"off".
    enum class State : uint8_t { Off, Disconnected, Connecting, Connected };
    State state() const { return state_; }

    // Provisioning-AP (offen, SSID = Hostname) laeuft: Mode 3 immer, Mode 2 nach
    // dem Connect-Timeout bis zum naechsten Connect. state() beschreibt nur die
    // Station; ueber den AP ist das Geraet trotzdem erreichbar. Gelesen wird der
    // Modus des Treibers -- es gibt keinen eigenen Merker: andere Nutzer des
    // Treibers (die Improv-Lib setzt beim Anlegen ihres Backends WIFI_MODE_STA)
    // koennen den AP abschalten, ohne dass wir davon erfahren.
    bool        ap_active() const;

    // Connected-Info.
    int8_t      rssi() const          { return rssi_; }
    // Mit einem AP assoziiert (unabhaengig von einer IP). `state()` taugt dafuer
    // nicht: Connecting steht schon waehrend des Verbindungsversuchs.
    bool        associated() const;
    uint32_t    ip4() const           { return ip4_; }     // 0 wenn nicht verbunden
    const char* default_hostname() const { return default_hostname_.c_str(); }

    // mDNS (Hostname + _culfw/_hmuartlgw-Services) einmalig hochziehen. Laeuft
    // sonst nur aus start(); main ruft es zusaetzlich, wenn allein Ethernet
    // (W5500) das Netz stellt — die mdns-Komponente bedient das ETH-netif mit
    // (CONFIG_MDNS_PREDEF_NETIF_ETH). Idempotent.
    void ensure_mdns() { ensure_mdns_(); }

    // Blocking active-scan. Returns alle gefundenen APs nach Abschluss.
    // Vor `start()` aufrufbar (Treiber wird intern temporaer initialisiert
    // wenn `start()` noch nicht lief).
    esp_err_t scan(std::vector<WifiScanResult>& out);

private:
    static void event_handler(void* arg, esp_event_base_t base, int32_t id, void* data);
    void        on_event(esp_event_base_t base, int32_t id, void* data);
    esp_err_t   write_sta_config_();      // cfg_ -> esp_wifi_set_config(STA)
    esp_err_t   apply_connect_();         // STA -> connect mit cfg_
    void        on_got_ip_(uint32_t ip);  // Zustand, AP je Modus, mDNS (GOT_IP)
    void        adopt_connection_();      // start(): schon verbundene Station uebernehmen
    void        compute_default_hostname_(); // aus MAC

    void        start_ap_();              // AP-Mode hochfahren (Mode 2/3)
    void        stop_ap_();               // AP-Mode runter (Mode 2 bei Connect-OK)
    static void ap_fallback_timer_cb_(void* arg);
    esp_err_t   reconfigure_sta_();       // stop+start zyklus fuer config-Wechsel

    void        ensure_mdns_();           // einmalige mdns_init+service_add
    void        update_mdns_hostname_();  // bei hostname-change

    bool        nvs_inited_ = false;
    bool        sta_started_ = false;
    bool        mdns_inited_ = false;
    std::atomic<bool> external_control_{false};  // Event- vs Improv-Task
    // Waehrend reconfigure_sta_() laeuft: kein Auto-Retry im
    // STA_DISCONNECTED-Handler. esp_wifi_stop() erzeugt selbst ein
    // DISCONNECTED-Event; ein esp_wifi_connect() daraus laesst den Treiber
    // schon connecten, bevor apply_connect_() im STA_START die neue Config
    // schreiben kann — das quittiert er mit "sta is connecting, cannot set
    // config" (ESP_ERR_WIFI_STATE) und die neuen Credentials verpuffen.
    std::atomic<bool> reconfiguring_{false};      // Event- vs Aufrufer-Task
    void*       nvs_handle_  = nullptr;    // nvs_handle_t (opaque)
    void*       sta_netif_   = nullptr;    // esp_netif_t* (opaque)
    void*       ap_netif_    = nullptr;    // esp_netif_t* (opaque)
    void*       ap_timer_    = nullptr;    // esp_timer_handle_t (opaque)

    // Schuetzt cfg_ (std::string ssid/psk/hostname + mode): Setter laufen auf
    // Frontend-/Improv-Tasks, Leser (on_event/apply_connect_/start_ap_/mDNS)
    // auf dem WiFi-Event- bzw. Timer-Task. Ein assign() das den String-Buffer
    // reallokiert waehrend der Event-Task c_str() dereferenziert ist sonst
    // use-after-free. Tight-scoped locken, nie ueber Driver-Calls halten.
    mutable std::mutex cfg_mu_;
    WifiConfig  cfg_{};
    std::string default_hostname_;
    State       state_ = State::Off;
    int8_t      rssi_  = 0;
    uint32_t    ip4_   = 0;
};

} // namespace culfw32
