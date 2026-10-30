// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "wifi_manager.hpp"
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "lwip/inet.h"
#include "mdns.h"
#include "esp_timer.h"
#include <cstdio>
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG = "wifi";
constexpr const char* NS  = "culfw32_wifi";

constexpr const char* KEY_SSID  = "ssid";
constexpr const char* KEY_PSK   = "psk";
constexpr const char* KEY_MODE  = "mode";
constexpr const char* KEY_HOST  = "hostname";

esp_err_t nvs_get_str_safe(nvs_handle_t h, const char* key, std::string& out) {
    size_t sz = 0;
    esp_err_t e = nvs_get_str(h, key, nullptr, &sz);
    if (e == ESP_ERR_NVS_NOT_FOUND) { out.clear(); return ESP_OK; }
    if (e != ESP_OK) return e;
    out.resize(sz);
    if (sz == 0) return ESP_OK;
    e = nvs_get_str(h, key, out.data(), &sz);
    if (e == ESP_OK && !out.empty() && out.back() == '\0') out.pop_back();
    return e;
}

// Fuehrt der Treiber einen AP (Modus AP oder APSTA)? Einzige Grundlage fuer die
// AP-Entscheidungen hier -- ein eigener Merker veraltet: das Improv-Backend setzt
// beim Anlegen WIFI_MODE_STA, ohne dass wir davon erfahren, und
// esp_wifi_set_config(WIFI_IF_AP) startet den AP neu (AP_STOP, obwohl er bleibt).
// Vor esp_wifi_init() scheitert esp_wifi_get_mode() -> false.
bool driver_ap_mode() {
    wifi_mode_t m = WIFI_MODE_NULL;
    return esp_wifi_get_mode(&m) == ESP_OK && (m == WIFI_MODE_AP || m == WIFI_MODE_APSTA);
}
}

WifiManager::~WifiManager() {
    // Nur den einmalig in init() geoeffneten NVS-Handle freigeben. Kein
    // esp_wifi_*-Teardown: der WiFi-Stack lebt bis zum Reboot weiter.
    if (nvs_handle_) {
        nvs_close(static_cast<nvs_handle_t>(reinterpret_cast<uintptr_t>(nvs_handle_)));
        nvs_handle_ = nullptr;
    }
}

esp_err_t WifiManager::init() {
    if (nvs_inited_) return ESP_OK;

    nvs_handle_t h;
    esp_err_t e = nvs_open(NS, NVS_READWRITE, &h);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(%s): %s", NS, esp_err_to_name(e));
        return e;
    }
    nvs_handle_ = reinterpret_cast<void*>(static_cast<uintptr_t>(h));

    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        nvs_get_str_safe(h, KEY_SSID, cfg_.ssid);
        nvs_get_str_safe(h, KEY_PSK,  cfg_.psk);
        nvs_get_str_safe(h, KEY_HOST, cfg_.hostname);
        uint8_t mode_u8 = 0;
        if (nvs_get_u8(h, KEY_MODE, &mode_u8) == ESP_OK) cfg_.mode = mode_u8;
    }

    compute_default_hostname_();
    nvs_inited_ = true;
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        ESP_LOGI(TAG, "loaded config: mode=%u ssid='%.32s' hostname='%s'",
                 cfg_.mode, cfg_.ssid.c_str(),
                 cfg_.hostname.empty() ? default_hostname_.c_str() : cfg_.hostname.c_str());
    }
    return ESP_OK;
}

bool WifiManager::associated() const {
    if (!sta_started_) return false;
    wifi_ap_record_t ap{};
    return esp_wifi_sta_get_ap_info(&ap) == ESP_OK;   // ESP_ERR_WIFI_NOT_CONNECT sonst
}

void WifiManager::compute_default_hostname_() {
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char buf[24];
    std::snprintf(buf, sizeof(buf), "culfw32-%02x%02x%02x", mac[3], mac[4], mac[5]);
    default_hostname_ = buf;
}

void WifiManager::event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    static_cast<WifiManager*>(arg)->on_event(base, id, data);
}

void WifiManager::on_event(esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT) {
        switch (id) {
            case WIFI_EVENT_STA_START: {
                ESP_LOGI(TAG, "STA started");
                uint8_t mode; bool ssid_empty;
                {
                    std::lock_guard<std::mutex> lk(cfg_mu_);
                    mode = cfg_.mode;
                    ssid_empty = cfg_.ssid.empty();
                }
                // Mode 3: AP VOR dem Connect hochfahren (STA+AP gleichzeitig).
                // start_ap_() schaltet auf APSTA um; nach esp_wifi_connect()
                // gerufen, brach das den Suchlauf des gerade gestarteten
                // Connects ab (Disconnect mit NO_AP_FOUND, gemessen 11.10.2026),
                // und dieses Disconnect fiel in die Improv-/Reconfigure-Sperre
                // ohne Auto-Retry -- die Station verband sich nie.
                if (mode == 3) start_ap_();
                if ((mode == 1 || mode == 2 || mode == 3) && !ssid_empty) {
                    apply_connect_();
                    // Mode 2: bei Connect-Timeout AP-Fallback ausloesen.
                    // Mode 3: nach 30 s ohne Connect den AP sicherstellen (s. GOT_IP).
                    // Stop davor: ein AP_STOP kurz vorher (reconfigure_sta_) kann
                    // den Timer schon aufgezogen haben, start_once lehnt dann ab.
                    if ((mode == 2 || mode == 3) && ap_timer_) {
                        esp_timer_stop(static_cast<esp_timer_handle_t>(ap_timer_));
                        esp_timer_start_once(static_cast<esp_timer_handle_t>(ap_timer_),
                                             30ull * 1000ull * 1000ull); // 30 s
                    }
                }
                break;
            }
            case WIFI_EVENT_STA_DISCONNECTED: {
                state_ = State::Disconnected;
                ip4_   = 0;
                rssi_  = 0;
                if (reconfiguring_.load(std::memory_order_relaxed)) {
                    // Dieses Event stammt aus dem esp_wifi_stop() in
                    // reconfigure_sta_(). Ein Auto-Retry hier startet einen
                    // Connect mit der ALTEN Config und blockiert damit das
                    // esp_wifi_set_config() im folgenden STA_START
                    // ("sta is connecting, cannot set config"). Der Connect
                    // kommt gleich aus apply_connect_() — mit den neuen Daten.
                    ESP_LOGI(TAG, "disconnected — reconfigure laeuft, kein auto-retry");
                    break;
                }
                if (external_control_.load(std::memory_order_relaxed)) {
                    // Improv-Lib (oder anderer externer Provisioner) steuert
                    // den Driver gerade. Auto-retry konkurriert mit ihrem
                    // tryConnect()-Pfad und triggert `sta is connecting` →
                    // sofortiges UNABLE_TO_CONNECT auf der Browser-Seite.
                    ESP_LOGI(TAG, "disconnected — external control active, no auto-retry");
                    break;
                }
                ESP_LOGW(TAG, "disconnected — auto-retry");
                uint8_t mode; bool ssid_empty;
                {
                    std::lock_guard<std::mutex> lk(cfg_mu_);
                    mode = cfg_.mode;
                    ssid_empty = cfg_.ssid.empty();
                }
                if ((mode == 1 || mode == 2 || mode == 3) && !ssid_empty) {
                    state_ = State::Connecting;
                    esp_wifi_connect();
                }
                break;
            }
            case WIFI_EVENT_STA_CONNECTED:
                state_ = State::Connecting;
                ESP_LOGI(TAG, "associated");
                break;
            case WIFI_EVENT_AP_STACONNECTED:
                ESP_LOGI(TAG, "AP: client joined");
                break;
            case WIFI_EVENT_AP_STADISCONNECTED:
                ESP_LOGI(TAG, "AP: client left");
                break;
            case WIFI_EVENT_AP_STOP: {
                // AP_STOP heisst nicht immer "AP weg": esp_wifi_set_config() in
                // start_ap_() startet ihn neu (gemessen 11.10.2026: AP_START,
                // AP_STOP, AP_START). Weg ist er erst, wenn der Treiber keinen
                // AP-Modus mehr fuehrt -- nach stop_ap_() oder weil ein anderer
                // Nutzer des Treibers umgestellt hat (das Improv-Backend setzt beim
                // Anlegen WIFI_MODE_STA, gemessen auch erst nach dem GOT_IP).
                // In Modus 3 nicht sofort zurueckschalten: ein Moduswechsel
                // waehrend eines laufenden Connects bricht dessen Suchlauf ab
                // (NO_AP_FOUND, gemessen 11.10.2026). Stattdessen den 30-s-Timer
                // neu aufziehen; sein Callback fragt den Treiber erneut.
                // Waehrend reconfigure_sta_() bringt STA_START den AP selbst hoch.
                if (driver_ap_mode()) break;
                if (reconfiguring_.load(std::memory_order_relaxed)) break;
                uint8_t mode;
                {
                    std::lock_guard<std::mutex> lk(cfg_mu_);
                    mode = cfg_.mode;
                }
                if (mode == 3 && sta_started_ && ap_timer_) {
                    esp_timer_stop(static_cast<esp_timer_handle_t>(ap_timer_));
                    esp_timer_start_once(static_cast<esp_timer_handle_t>(ap_timer_),
                                         30ull * 1000ull * 1000ull); // 30 s
                }
                break;
            }
        }
    } else if (base == IP_EVENT) {
        if (id == IP_EVENT_STA_GOT_IP) {
            on_got_ip_(static_cast<ip_event_got_ip_t*>(data)->ip_info.ip.addr);
        }
    }
}

void WifiManager::on_got_ip_(uint32_t ip) {
    ip4_ = ip;
    wifi_ap_record_t ap{};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi_ = ap.rssi;
    state_ = State::Connected;
    esp_ip4_addr_t a{ip};
    ESP_LOGI(TAG, "got IP " IPSTR " rssi=%d", IP2STR(&a), rssi_);

    // Mode 2: STA hat sich verbunden -> AP-Fallback abbrechen + ggf. wieder runter.
    uint8_t mode;
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        mode = cfg_.mode;
    }
    if (mode == 2) {
        if (ap_timer_) esp_timer_stop(static_cast<esp_timer_handle_t>(ap_timer_));
        stop_ap_();
    }
    // Mode 3: der AP gehoert immer dazu. Beim Boot nimmt ihn die
    // Improv-Lib wieder weg (sie setzt beim Anlegen ihres Backends
    // WIFI_MODE_STA, nach unserem STA_START; gemessen 11.10.2026: nach
    // dem Boot meldete der Treiber STA statt APSTA). Mit verbundener
    // Station laeuft kein Connect-Suchlauf, den der Moduswechsel
    // abbrechen koennte. start_ap_() tut nichts, wenn der AP schon
    // laeuft. Legt die Lib ihr Backend erst nach dem GOT_IP an, holt
    // der Timer den AP zurueck (s. WIFI_EVENT_AP_STOP).
    if (mode == 3) {
        if (ap_timer_) esp_timer_stop(static_cast<esp_timer_handle_t>(ap_timer_));
        start_ap_();
    }

    // mDNS hat schon der `start()`-Pfad initialisiert; hier nur den
    // Hostname auf den evtl. neu negotiate-fertigen netif aktualisieren.
    update_mdns_hostname_();
}

void WifiManager::adopt_connection_() {
    // Ersteinrichtung per Improv aus Modus 0: die Lib startet den Treiber,
    // verbindet die Station und wartet ihr GOT_IP ab, BEVOR on_connected ->
    // set_persisted_creds() uns startet. Unsere Handler sind erst ab hier
    // registriert, ein weiteres GOT_IP kommt nicht (gemessen 11.10.2026:
    // wifi.state "disconnected", IP 0.0.0.0 bis zum Neustart, obwohl die
    // Station verbunden war). Den Zustand deshalb beim Treiber abholen.
    // Zuerst die IP: ohne Verbindung warnt esp_wifi_sta_get_ap_info() auf der
    // Konsole ("Haven't to connect to a suitable AP now!") -- bei jedem
    // normalen Boot. Assoziiert ohne IP: das GOT_IP kommt noch an unsere
    // Handler.
    esp_netif_ip_info_t ipi{};
    if (!sta_netif_ ||
        esp_netif_get_ip_info(static_cast<esp_netif_t*>(sta_netif_), &ipi) != ESP_OK ||
        ipi.ip.addr == 0) return;
    if (!associated()) return;
    on_got_ip_(ipi.ip.addr);
}

void WifiManager::set_external_control(bool on) {
    const bool was = external_control_.exchange(on, std::memory_order_relaxed);
    if (on) {
        // Die Improv-Lib uebernimmt (erster Improv-Rahmen). Ein laufender
        // Auto-Retry-Versuch -- mit gespeicherten, aber unerreichbaren Daten
        // ist die Station fast durchgehend "connecting" -- belegt den Treiber:
        // die Netzsuche der Lib scheitert dann mit ESP_ERR_WIFI_STATE, ihr
        // set_config ebenso (Review 0909 F1). Den Versuch abbrechen; das
        // folgende STA_DISCONNECTED faellt schon in die Sperre.
        if (!was && sta_started_ && !reconfiguring_.load(std::memory_order_relaxed) &&
            state_ == State::Connecting && !associated()) {
            ESP_LOGW(TAG, "external control: aborting the running connect attempt");
            esp_wifi_disconnect();
        }
        return;
    }
    if (!was || !sta_started_ || reconfiguring_.load(std::memory_order_relaxed)) return;
    if (state_ != State::Disconnected) return;
    // Der Treiber zaehlt, nicht unser Zustand: ein Connect auf eine stehende
    // Verbindung quittiert er nur mit "sta is connected, disconnect before
    // connecting to new ap", und state_ blieb danach auf Connecting haengen
    // (gemessen 11.10.2026 nach der Ersteinrichtung, s. adopt_connection_()).
    if (associated()) return;
    uint8_t mode; bool ssid_empty;
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        mode = cfg_.mode;
        ssid_empty = cfg_.ssid.empty();
    }
    if (mode == 0 || ssid_empty) return;
    ESP_LOGW(TAG, "external control released while disconnected - reconnecting");
    // Unsere Daten in den Treiber schreiben statt blind esp_wifi_connect():
    // nach einem gescheiterten Improv-tryConnect() haelt der Treiber dessen
    // Daten (on_connected kam nicht, cfg_ ist unveraendert) -- ein blinder
    // Connect liefe bis zum Neustart gegen die falschen Daten. Laeuft wider
    // Erwarten schon ein Connect, lehnt set_config mit ESP_ERR_WIFI_STATE ab,
    // und es gibt keinen zweiten Connect.
    if (auto e = apply_connect_(); e != ESP_OK) {
        ESP_LOGW(TAG, "reconnect after external control: %s", esp_err_to_name(e));
        // Auf einen fehlgeschlagenen Aufruf folgt kein Ereignis; der Zustand
        // bliebe sonst auf Connecting stehen.
        state_ = State::Disconnected;
    }
}

void WifiManager::ensure_mdns_() {
    if (mdns_inited_) return;
    if (mdns_init() != ESP_OK) {
        ESP_LOGW(TAG, "mdns_init failed");
        return;
    }
    std::string hn_s;
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        hn_s = cfg_.hostname.empty() ? default_hostname_ : cfg_.hostname;
    }
    const char* hn = hn_s.c_str();
    mdns_hostname_set(hn);
    mdns_instance_name_set("CULFW32");
    if (auto e = mdns_service_add(nullptr, "_culfw", "_tcp", 2323, nullptr, 0);
        e != ESP_OK && e != ESP_ERR_NO_MEM) {
        ESP_LOGW(TAG, "mdns_service_add(culfw): %s", esp_err_to_name(e));
    }
    // HMUARTLGW-Discovery fuer bmcd / FHEM / debmatic. Zwei Service-Instances
    // mit unterschiedlichen Instance-Namen — Standard-Trick um beide Modi am
    // gleichen Service-Type sichtbar zu machen ohne TXT-Inspect noetig zu
    // machen. Ports identisch zur HmuartlgwFrontend Two-Port-Architektur:
    //   :2325 = Legacy (HM-MOD-RPI-PCB) → FHEM, Homegear Hm-Mod-Rpi-Pcb
    //   :2327 = Dual   (RPI-RF-MOD)     → debmatic, RaspberryMatic
    // Capability-Bits: erlauben Discovery-Clients (bmcd) automatisch zu
    // erkennen welche Modi tx-faehig sind. CULFW32 kann kein HmIP — weder RX
    // noch TX; dafuer ist q-culfw zustaendig. Das Feld wird trotzdem gesendet
    // (`hmip=no`), damit ein Client "Stick kann kein HmIP" von "Stick ist zu
    // alt und meldet gar nichts" unterscheiden kann.
    mdns_txt_item_t txt_legacy[] = {
        {"mode",   "legacy"},
        {"model",  "HM-MOD-RPI-PCB"},
        {"fw",     "1.4.1"},
        {"bidcos", "tx,rx"},
    };
    mdns_txt_item_t txt_dual[] = {
        {"mode",   "dual"},
        {"model",  "RPI-RF-MOD"},
        {"fw",     "4.4.18"},
        {"bidcos", "tx,rx"},
        {"hmip",   "no"},
    };
    if (auto e = mdns_service_add("CULFW32-Legacy", "_hmuartlgw", "_tcp",
                                  2325, txt_legacy,
                                  sizeof(txt_legacy)/sizeof(txt_legacy[0]));
        e != ESP_OK && e != ESP_ERR_NO_MEM) {
        ESP_LOGW(TAG, "mdns_service_add(hmuartlgw legacy): %s", esp_err_to_name(e));
    }
    if (auto e = mdns_service_add("CULFW32-Dual", "_hmuartlgw", "_tcp",
                                  2327, txt_dual,
                                  sizeof(txt_dual)/sizeof(txt_dual[0]));
        e != ESP_OK && e != ESP_ERR_NO_MEM) {
        ESP_LOGW(TAG, "mdns_service_add(hmuartlgw dual): %s", esp_err_to_name(e));
    }
    mdns_inited_ = true;
    ESP_LOGI(TAG, "mDNS up: %s.local  _culfw._tcp:2323  _hmuartlgw._tcp:2325(L)+2327(D)", hn);
}

void WifiManager::update_mdns_hostname_() {
    if (!mdns_inited_) return;
    std::string hn;
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        hn = cfg_.hostname.empty() ? default_hostname_ : cfg_.hostname;
    }
    mdns_hostname_set(hn.c_str());
}

bool WifiManager::ap_active() const {
    // Nach esp_wifi_stop() bleibt der Modus des Treibers stehen; ob ueberhaupt
    // gefunkt wird, sagt sta_started_.
    return sta_started_ && driver_ap_mode();
}

void WifiManager::start_ap_() {
    // Den Treiber fragen, nicht ap_active(): im ersten STA_START nach dem
    // Boot ist sta_started_ noch nicht gesetzt.
    if (driver_ap_mode()) return;
    if (!ap_netif_) ap_netif_ = esp_netif_create_default_wifi_ap();

    // AP-Mode aktivieren — esp_wifi_set_mode setzt STA+AP wenn beide gewuenscht.
    wifi_mode_t cur = WIFI_MODE_STA;
    esp_wifi_get_mode(&cur);
    esp_wifi_set_mode(WIFI_MODE_APSTA);

    std::string hn;
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        hn = cfg_.hostname.empty() ? default_hostname_ : cfg_.hostname;
    }
    wifi_config_t apc = {};
    std::strncpy(reinterpret_cast<char*>(apc.ap.ssid), hn.c_str(),
                 sizeof(apc.ap.ssid) - 1);
    apc.ap.ssid_len     = std::strlen(reinterpret_cast<char*>(apc.ap.ssid));
    apc.ap.channel      = 6;
    apc.ap.max_connection = 4;
    apc.ap.authmode     = WIFI_AUTH_OPEN;     // Provisioning-AP, kein PSK
    apc.ap.pmf_cfg.required = false;
    esp_wifi_set_config(WIFI_IF_AP, &apc);

    // Kanal 6 gilt nur ohne verbundene Station; mit Station laeuft der AP auf
    // deren Kanal (IDF: Station hat Vorrang) -- den tatsaechlichen melden.
    uint8_t ch = 0;
    wifi_second_chan_t ch2 = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&ch, &ch2);
    ESP_LOGW(TAG, "AP up: SSID '%s' (open), channel %u", hn.c_str(), static_cast<unsigned>(ch));
}

void WifiManager::stop_ap_() {
    if (!driver_ap_mode()) return;
    esp_wifi_set_mode(WIFI_MODE_STA);
    ESP_LOGI(TAG, "AP-Fallback down (STA back to solo)");
}

void WifiManager::ap_fallback_timer_cb_(void* arg) {
    auto* self = static_cast<WifiManager*>(arg);
    uint8_t mode;
    {
        std::lock_guard<std::mutex> lk(self->cfg_mu_);
        mode = self->cfg_.mode;
    }
    if (mode == 2) {
        if (self->state_ == State::Connected) return; // schon connected, kein Fallback
        ESP_LOGW(TAG, "STA-Connect-Timeout (30s) -> AP-Fallback aktivieren");
        self->start_ap_();
        return;
    }
    if (mode == 3) {
        // Modus 3: der AP gehoert immer dazu, auch bei verbundener Station
        // (AP_STOP nach dem GOT_IP, s. on_event). Kein Connected-Kurzschluss.
        if (!self->sta_started_ || driver_ap_mode()) return;   // nichts zu tun
        ESP_LOGW(TAG, "mode 3: access point down - restarting it");
        self->start_ap_();
    }
}

esp_err_t WifiManager::write_sta_config_() {
    std::string ssid, psk;
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        ssid = cfg_.ssid;
        psk  = cfg_.psk;
    }
    wifi_config_t wc = {};
    std::strncpy(reinterpret_cast<char*>(wc.sta.ssid), ssid.c_str(),
                 sizeof(wc.sta.ssid) - 1);
    std::strncpy(reinterpret_cast<char*>(wc.sta.password), psk.c_str(),
                 sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = psk.empty() ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    wc.sta.pmf_cfg.capable    = true;
    wc.sta.pmf_cfg.required   = false;
    return esp_wifi_set_config(WIFI_IF_STA, &wc);
}

esp_err_t WifiManager::apply_connect_() {
    if (auto e = write_sta_config_(); e != ESP_OK) {
        ESP_LOGE(TAG, "set_config: %s", esp_err_to_name(e));
        return e;
    }
    state_ = State::Connecting;
    return esp_wifi_connect();
}

esp_err_t WifiManager::start() {
    if (sta_started_) return ESP_OK;
    if (!nvs_inited_) {
        if (auto e = init(); e != ESP_OK) return e;
    }

    // esp_netif_init / event_loop_create_default sind idempotent? Letzteres
    // returned ESP_ERR_INVALID_STATE wenn schon gemacht — ignorieren.
    esp_netif_init();
    esp_err_t e = esp_event_loop_create_default();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event_loop_create_default: %s", esp_err_to_name(e));
        return e;
    }

    if (!sta_netif_) {
        // Ein Fruehboot-Pfad (Improv-Lib WiFi-Init, AP-Fallback) kann das
        // default WIFI_STA_DEF-netif bereits angelegt haben, ohne unser
        // Member zu setzen. esp_netif_create_default_wifi_sta() assertet dann
        // hart auf „duplicate key" (wifi_default.c) -> PANIC. Vorhandenes
        // netif wiederverwenden statt blind neu zu erstellen.
        sta_netif_ = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (!sta_netif_) sta_netif_ = esp_netif_create_default_wifi_sta();
    }
    if (!sta_netif_) {
        ESP_LOGE(TAG, "create_default_wifi_sta failed");
        return ESP_FAIL;
    }

    // AP-Fallback-Timer (Mode 2: 30 s nach STA_START → AP an wenn nicht connected).
    if (!ap_timer_) {
        esp_timer_create_args_t targs = {
            .callback        = &WifiManager::ap_fallback_timer_cb_,
            .arg             = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name            = "wifi_ap_fb",
            .skip_unhandled_events = false,
        };
        esp_timer_handle_t h = nullptr;
        if (esp_timer_create(&targs, &h) == ESP_OK) ap_timer_ = h;
    }

    // Hostname setzen (vor wifi_init wirkt nicht; nach wifi_start ja, wir
    // setzen es spaeter im Event-Pfad. Erstmal setzen wir's am netif.)
    std::string hn;
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        hn = cfg_.hostname.empty() ? default_hostname_ : cfg_.hostname;
    }
    esp_netif_set_hostname(static_cast<esp_netif_t*>(sta_netif_), hn.c_str());

    wifi_init_config_t wic = WIFI_INIT_CONFIG_DEFAULT();
    if (auto err = esp_wifi_init(&wic); err != ESP_OK) {
        ESP_LOGE(TAG, "wifi_init: %s", esp_err_to_name(err));
        return err;
    }

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
        &WifiManager::event_handler, this, nullptr);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
        &WifiManager::event_handler, this, nullptr);

    esp_wifi_set_storage(WIFI_STORAGE_RAM);  // wir managen NVS selber
    esp_wifi_set_mode(WIFI_MODE_STA);

    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        if (cfg_.mode == 0) {
            ESP_LOGI(TAG, "WiFi configured but mode=0 (off) — driver started for scan only");
        }
    }

    if (auto err = esp_wifi_start(); err != ESP_OK) {
        ESP_LOGE(TAG, "wifi_start: %s", esp_err_to_name(err));
        return err;
    }
    // WiFi Power-Save aus. Default `WIFI_PS_MIN_MODEM` macht DTIM-Wakeup-Cycles
    // mit ~100 ms-Latenz-Spitzen — fuer HB-RF-ETH-Kernel-Modul (~135 ms
    // Connect-Timeout) und HMUARTLGW-Echtzeit-Pfad zu langsam. NONE laesst
    // den Modem dauerhaft an, leichter hoeherer Stromverbrauch
    // (~80 mA → ~120 mA), aber Round-Trips landen unter 20 ms statt 50-200 ms.
    esp_wifi_set_ps(WIFI_PS_NONE);
    sta_started_ = true;
    state_       = State::Disconnected;
    ensure_mdns_();
    adopt_connection_();
    return ESP_OK;
}

esp_err_t WifiManager::reconfigure_sta_() {
    // Aufruf-Pfad: SSID/PSK/Mode hat sich geaendert. Sauberster Weg ohne
    // disconnect-vs-connect-Race ist esp_wifi_stop() + esp_wifi_start():
    // dann triggert der STA_START-event mit der frischen cfg_ den connect.
    if (!sta_started_) return ESP_OK;
    if (ap_timer_) esp_timer_stop(static_cast<esp_timer_handle_t>(ap_timer_));
    stop_ap_();

    // Ab hier bis zum Ende: der STA_DISCONNECTED-Handler darf nicht dazwischen
    // connecten (siehe Kommentar dort). Guard statt Flag-Zuweisungen an jedem
    // der drei Rueckgabepunkte.
    struct ReconfGuard {
        std::atomic<bool>& f;
        explicit ReconfGuard(std::atomic<bool>& x) : f(x) { f.store(true, std::memory_order_relaxed); }
        ~ReconfGuard() { f.store(false, std::memory_order_relaxed); }
    } guard(reconfiguring_);

    state_ = State::Off;
    ip4_   = 0;
    rssi_  = 0;
    if (auto e = esp_wifi_stop(); e != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_stop: %s", esp_err_to_name(e));
    }

    uint8_t mode;
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        mode = cfg_.mode;
    }
    if (mode == 0) {
        // Stayed stopped — esp_wifi_start nicht nochmal aufrufen.
        sta_started_ = false;
        return ESP_OK;
    }

    // Neue Credentials JETZT schreiben, im gestoppten Zustand: hier nimmt der
    // Treiber sie immer an. Damit ist es egal, wer nach dem Start zuerst
    // connectet (apply_connect_ aus STA_START oder ein Auto-Retry) — beide
    // verwenden ab sofort die neuen Daten. Frueher wurde die Config erst im
    // STA_START-Handler gesetzt und dort abgelehnt, sobald schon ein Connect
    // lief; die Aenderung verpuffte dann still (gemessen 01.09.2026:
    // "sta is connecting, cannot set config" / ESP_ERR_WIFI_STATE, und der
    // Stick blieb auf den alten Credentials).
    if (auto e = write_sta_config_(); e != ESP_OK) {
        ESP_LOGW(TAG, "set_config im Stop-Zustand: %s", esp_err_to_name(e));
    }

    if (auto e = esp_wifi_start(); e != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start: %s", esp_err_to_name(e));
        sta_started_ = false;
        return e;
    }
    state_ = State::Disconnected;
    return ESP_OK;
}

esp_err_t WifiManager::set_ssid(std::string_view s) {
    // esp_wifi: ssid[32], password[64]. Lieber sauber ablehnen als stillschweigend
    // truncaten (frueher strncpy in apply_connect_) — Truncation produziert
    // unverbindbare Credentials ohne Fehlermeldung.
    if (s.size() > 32) return ESP_ERR_INVALID_ARG;
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        cfg_.ssid.assign(s);
        auto h = static_cast<nvs_handle_t>(reinterpret_cast<uintptr_t>(nvs_handle_));
        nvs_set_str(h, KEY_SSID, cfg_.ssid.c_str());
        nvs_commit(h);
    }
    return reconfigure_sta_();
}

esp_err_t WifiManager::set_psk(std::string_view s) {
    // WPA2-PSK: max 63 Zeichen (64-Byte-Buffer inkl. NUL). 0 = offenes Netz.
    if (s.size() > 63) return ESP_ERR_INVALID_ARG;
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        cfg_.psk.assign(s);
        auto h = static_cast<nvs_handle_t>(reinterpret_cast<uintptr_t>(nvs_handle_));
        nvs_set_str(h, KEY_PSK, cfg_.psk.c_str());
        nvs_commit(h);
    }
    return reconfigure_sta_();
}

esp_err_t WifiManager::set_mode(uint8_t mode) {
    bool was_started = sta_started_;
    bool ssid_empty;
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        cfg_.mode = mode;
        ssid_empty = cfg_.ssid.empty();
        auto h = static_cast<nvs_handle_t>(reinterpret_cast<uintptr_t>(nvs_handle_));
        nvs_set_u8(h, KEY_MODE, mode);
        nvs_commit(h);
    }
    if (was_started) {
        return reconfigure_sta_();
    }
    // Wenn STA-Stack noch nicht initialisiert war, jetzt anstossen.
    if (mode != 0 && !ssid_empty) {
        return start();
    }
    return ESP_OK;
}

esp_err_t WifiManager::set_persisted_creds(std::string_view ssid,
                                            std::string_view psk,
                                            uint8_t mode) {
    bool ssid_empty;
    esp_err_t stored;
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        cfg_.ssid.assign(ssid);
        cfg_.psk.assign(psk);
        cfg_.mode = mode;
        ssid_empty = cfg_.ssid.empty();
        auto h = static_cast<nvs_handle_t>(reinterpret_cast<uintptr_t>(nvs_handle_));
        stored = nvs_set_str(h, KEY_SSID, cfg_.ssid.c_str());
        if (stored == ESP_OK) stored = nvs_set_str(h, KEY_PSK, cfg_.psk.c_str());
        if (stored == ESP_OK) stored = nvs_set_u8(h, KEY_MODE, cfg_.mode);
        if (stored == ESP_OK) stored = nvs_commit(h);
        if (stored != ESP_OK) {
            ESP_LOGW(TAG, "persisting creds failed: %s", esp_err_to_name(stored));
        } else {
            ESP_LOGI(TAG, "persisted creds: mode=%u ssid='%.32s' (driver untouched)",
                     cfg_.mode, cfg_.ssid.c_str());
        }
    }
    // Wenn unser Manager noch nicht lief (Boot-Pfad mit mode=0 in NVS, Improv
    // kommt frisch dran): jetzt anstossen, damit Event-Handler/mDNS/AP-Fallback
    // fuer kuenftige Disconnects in dieser Session da sind. Best-effort —
    // wenn Lib's Backend den Driver schon initialisiert hat, kann esp_wifi_start
    // mit ESP_ERR_*INVALID_STATE returnen; Boot nach Reboot startet dann
    // sauber mit den persistierten Credentials.
    if (!sta_started_ && mode != 0 && !ssid_empty) {
        if (auto e = start(); e != ESP_OK) {
            ESP_LOGW(TAG, "post-improv start() returned %s — relying on next boot",
                     esp_err_to_name(e));
        }
    }
    return stored;
}

esp_err_t WifiManager::set_hostname(std::string_view s) {
    std::string hn;   // effektiver Hostname-Snapshot fuer Driver/mDNS-Calls
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        cfg_.hostname.assign(s);
        auto h = static_cast<nvs_handle_t>(reinterpret_cast<uintptr_t>(nvs_handle_));
        nvs_set_str(h, KEY_HOST, cfg_.hostname.c_str());
        nvs_commit(h);
        hn = cfg_.hostname.empty() ? default_hostname_ : cfg_.hostname;
    }
    if (sta_netif_) {
        esp_netif_set_hostname(static_cast<esp_netif_t*>(sta_netif_), hn.c_str());
    }
    if (mdns_inited_) mdns_hostname_set(hn.c_str());
    return ESP_OK;
}

esp_err_t WifiManager::wipe() {
    auto h = static_cast<nvs_handle_t>(reinterpret_cast<uintptr_t>(nvs_handle_));
    nvs_erase_all(h);
    nvs_commit(h);
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        cfg_ = WifiConfig{};
    }
    if (sta_started_) {
        // Zustand wie nach einem Boot in Modus 0: kein AP, kein Fallback-
        // Timer, Station getrennt. Bis 0.1.891 lief ein AP (Modus 3 oder der
        // Modus-2-Fallback) bis zum Neustart weiter (gemessen 12.10.2026).
        // Den Treiber nicht stoppen: das Improv-Backend braucht ihn im
        // laufenden Fenster, und nach einem Boot in Modus 0 laeuft er ebenso
        // (die Lib startet ihn).
        if (ap_timer_) esp_timer_stop(static_cast<esp_timer_handle_t>(ap_timer_));
        stop_ap_();
        esp_wifi_disconnect();
        state_ = State::Off;
    }
    return ESP_OK;
}

esp_err_t WifiManager::scan(std::vector<WifiScanResult>& out) {
    out.clear();
    if (auto e = start(); e != ESP_OK) return e;

    wifi_scan_config_t scfg = {};
    scfg.show_hidden = true;
    scfg.scan_type   = WIFI_SCAN_TYPE_ACTIVE;
    scfg.scan_time.active.min = 100;
    scfg.scan_time.active.max = 300;

    if (auto e = esp_wifi_scan_start(&scfg, /*block=*/true); e != ESP_OK) {
        ESP_LOGE(TAG, "scan_start: %s", esp_err_to_name(e));
        return e;
    }

    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    if (n == 0) return ESP_OK;
    std::vector<wifi_ap_record_t> aps(n);
    if (auto e = esp_wifi_scan_get_ap_records(&n, aps.data()); e != ESP_OK) {
        return e;
    }
    out.reserve(n);
    for (uint16_t i = 0; i < n; ++i) {
        WifiScanResult r{};
        r.ssid.assign(reinterpret_cast<const char*>(aps[i].ssid));
        std::memcpy(r.bssid, aps[i].bssid, 6);
        r.rssi      = aps[i].rssi;
        r.channel   = aps[i].primary;
        r.auth_mode = static_cast<uint8_t>(aps[i].authmode);
        out.push_back(std::move(r));
    }
    return ESP_OK;
}

} // namespace culfw32
