// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "sdkconfig.h"
#include "rf_manager.hpp"
#include "frontend_culfw.hpp"
#include "culfw_version.hpp"
#include "transport_usbcdc.hpp"
#include "led_drv.hpp"
#include "culfw_ee_addr.hpp"
#include "culfw_ee_defaults.hpp"
#include "nvs_eeprom.hpp"
#include "config_registry.hpp"
#include "rf_stats.hpp"
#include "system_monitor.hpp"
#ifdef CONFIG_CULFW32_BUILD_RF_MORITZ
#include "moritz_rf.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_ASKSIN
#include "asksin_rf.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_SLOWRF
#include "slowrf_rf.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_NATIVE
#include "native_rf.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_FASTRF
#include "fastrf_rf.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_DUDETTE
#include "dudette_rf.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_WMBUS
#include "wmbus_rf.hpp"
#include "hmip_rf.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_UART
#include "transport_uart.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
#include "wifi_manager.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_NET_W5500
#include "eth_manager.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_TCP
#include "ota_update.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_TCP
#include "esp_netif.h"
#include "tcp_server.hpp"
#include "tcp_rx_pool.hpp"
#include "tcp_client_transport.hpp"
#include "tx_queue.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_JSON
#include "frontend_json.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_SIGNALDUINO
#include "frontend_signalduino.hpp"
#include "signalduino_rf.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_HMUARTLGW
#include "hmuartlgw_frontend.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_HBRFETH_UDP
#include "hbrfeth_listener.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_WEBUI
#include "web_server.hpp"
#include "transport_websocket.hpp"
#endif
#ifdef CONFIG_CULFW32_BUILD_IMPROV_SERIAL
#include "improv_wifi/improv_wifi.h"
#include "improv_wifi/idf_backend.h"
#include "improv_wifi/serial_filter.h"
#include "freertos/stream_buffer.h"
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include "esp_system.h"
#endif
#include "freq_cal.hpp"
#include "esp_log.h"
#include <cstring>
#include <memory>

using namespace culfw32;

namespace {
constexpr const char* TAG = "culfw32";

constexpr CC1101SlotConfig kSlot0 = {
    .spi_host      = CONFIG_CULFW32_CC1101_SPI_HOST,
    .pin_cs        = CONFIG_CULFW32_CC1101_PIN_CS,
    .pin_gdo0      = CONFIG_CULFW32_CC1101_PIN_GDO0,
    .pin_gdo2      = CONFIG_CULFW32_CC1101_PIN_GDO2,
    .pin_mosi      = CONFIG_CULFW32_CC1101_PIN_MOSI,
    .pin_miso      = CONFIG_CULFW32_CC1101_PIN_MISO,
    .pin_sck       = CONFIG_CULFW32_CC1101_PIN_SCK,
    .spi_clock_hz  = CONFIG_CULFW32_CC1101_SPI_CLOCK_HZ,
};

std::unique_ptr<RfManager>      g_mgr;
std::unique_ptr<NvsEeprom>      g_eeprom;
std::unique_ptr<LedDriver>      g_led;
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
std::unique_ptr<WifiManager>    g_wifi;
#endif
#ifdef CONFIG_CULFW32_BUILD_NET_W5500
std::unique_ptr<EthManager>     g_eth;
#endif
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_TCP
std::unique_ptr<TcpServer>      g_tcp;
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_HMUARTLGW
std::unique_ptr<TcpServer>          g_tcp_hmu;
std::unique_ptr<HmuartlgwFrontend>  g_fe_hmu;
#endif
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_HBRFETH_UDP
std::unique_ptr<HbrfethListener>    g_hbrfeth;
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_JSON
std::unique_ptr<TcpServer>      g_tcp_json;
std::unique_ptr<JsonFrontend>   g_fe_json;
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_SIGNALDUINO
std::unique_ptr<TcpServer>           g_tcp_sd;
std::unique_ptr<SignalduinoFrontend> g_fe_sd;
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_WEBUI
std::unique_ptr<WebServer>      g_web;
#endif
std::unique_ptr<ConfigRegistry> g_cfg;
std::unique_ptr<CulfwFrontend>  g_fe_culfw;
std::shared_ptr<CC1101Slot>     g_slot0;
std::shared_ptr<UsbCdcTransport> g_usbcdc;

// Hilfsformatter: uint16 → dezimal string ohne weiteres Heap-Drama.
inline std::string u_to_str(uint32_t v) {
    char b[16];
    int n = snprintf(b, sizeof(b), "%u", static_cast<unsigned>(v));
    return std::string(b, n > 0 ? n : 0);
}

// CC1101-Status-Helpers — mirror der VR-Befehl-Decoder, aber als Standalone-
// Funktionen damit ConfigRegistry-Getter sie aufrufen koennen.
namespace rfstat {
constexpr uint8_t SLOT = 0;     // Single-CC v1
constexpr uint64_t F_XOSC = 26'000'000ULL;

inline uint8_t cfg(uint8_t a) {
    uint8_t v = 0xFF;
    if (g_mgr) g_mgr->cc_read_reg(SLOT, a, v);
    return v;
}
inline uint8_t st(uint8_t a) {
    uint8_t v = 0xFF;
    if (g_mgr) g_mgr->cc_read_status(SLOT, a, v);
    return v;
}
inline bool chip_alive() {
    uint8_t pn = st(0x30), ver = st(0x31);
    return !(pn == 0xFF && ver == 0xFF);
}
inline std::string chip_name() {
    if (!chip_alive()) return "no-chip";
    uint8_t v = st(0x31);
    char b[16];
    const char* n = "CC11xx";
    uint8_t pn = st(0x30);
    if (pn == 0x00) {
        switch (v) {
            case 0x04: n = "CC1100";    break;
            case 0x14: n = "CC1101";    break;
            case 0x17: n = "CC1101-Q1"; break;
        }
    }
    snprintf(b, sizeof(b), "%s(V=%02X)", n, v);
    return b;
}
inline std::string active_proto() {
    if (!g_mgr) return "none";
    auto p = g_mgr->active_protocol_on(SLOT);
    if (!p) return "none";
    switch (p->id()) {
        case ProtocolId::AskSin:       return "AskSin";
        case ProtocolId::Moritz:       return "Moritz";
        case ProtocolId::SlowRf:       return "SlowRf";
        case ProtocolId::PulseCapture: return "PulseCap";
        case ProtocolId::FastRf:       return "FastRf";
        case ProtocolId::Native:       return "Native";
        case ProtocolId::Mbus:         return "wMBus";
        case ProtocolId::Hmip:         return "HmIP";
        case ProtocolId::Dudette:      return "Dudette";
    }
    return "?";
}
inline std::string freq_mhz() {
    if (!chip_alive()) return "-";
    uint32_t fw = (uint32_t(cfg(0x0D)) << 16) | (uint32_t(cfg(0x0E)) << 8) | cfg(0x0F);
    uint64_t hz = (uint64_t(fw) * F_XOSC) >> 16;
    char b[24];
    snprintf(b, sizeof(b), "%lu.%03lu",
             static_cast<unsigned long>(hz / 1'000'000),
             static_cast<unsigned long>((hz / 1000) % 1000));
    return b;
}
inline std::string modulation() {
    if (!chip_alive()) return "-";
    uint8_t mod = (cfg(0x12) >> 4) & 0x07;
    switch (mod) {
        case 0: return "2-FSK";
        case 1: return "GFSK";
        case 3: return "OOK";
        case 4: return "4-FSK";
        case 7: return "MSK";
    }
    return "?";
}
inline std::string datarate_kbps() {
    if (!chip_alive()) return "-";
    uint8_t mdm4 = cfg(0x10), mdm3 = cfg(0x11);
    uint8_t e = mdm4 & 0x0F;
    uint64_t bps = ((uint64_t(256u + mdm3) << e) * F_XOSC) >> 28;
    char b[24];
    snprintf(b, sizeof(b), "%lu.%02lu",
             static_cast<unsigned long>(bps / 1000),
             static_cast<unsigned long>((bps / 10) % 100));
    return b;
}
inline std::string bw_khz() {
    if (!chip_alive()) return "-";
    uint8_t mdm4 = cfg(0x10);
    uint8_t e = (mdm4 >> 6) & 0x03;
    uint8_t m = (mdm4 >> 4) & 0x03;
    uint32_t hz = static_cast<uint32_t>(F_XOSC) / (8u * (4u + m) * (1u << e));
    char b[24];
    snprintf(b, sizeof(b), "%lu.%lu",
             static_cast<unsigned long>(hz / 1000),
             static_cast<unsigned long>((hz / 100) % 10));
    return b;
}
inline std::string deviation_khz() {
    if (!chip_alive()) return "-";
    uint8_t devr = cfg(0x15);
    uint8_t e = (devr >> 4) & 0x07;
    uint8_t m = devr & 0x07;
    uint32_t hz = static_cast<uint32_t>(((F_XOSC >> 17) * (8u + m)) << e);
    char b[24];
    snprintf(b, sizeof(b), "%lu.%01lu",
             static_cast<unsigned long>(hz / 1000),
             static_cast<unsigned long>((hz / 100) % 10));
    return b;
}
inline std::string marc_state() {
    if (!chip_alive()) return "-";
    uint8_t s = st(0x35) & 0x1F;
    switch (s) {
        case 0x01: return "IDLE";
        case 0x0D: return "RX";
        case 0x13: return "TX";
        case 0x11: return "RXOVF";
        case 0x16: return "TXUDF";
    }
    char b[12];
    snprintf(b, sizeof(b), "0x%02X", s);
    return b;
}
// CC1101 RSSI-Decode (SWRS061I §17.3): raw < 128 → raw/2 - 74; raw >= 128 →
// (raw-256)/2 - 74. RSSI ist primaer nur in RX (0x0D) valid; in den unmittel-
// baren Wrapping-States (0x0E RX_END, 0x0F RX_RST, 0x10 TXRX_SWITCH) ist der
// letzte RSSI noch sinnvoll. In IDLE/TX/SLEEP/Calibration gibt der Chip einen
// Sentinel (typisch 0x80 → -138 dBm) — wir blenden das aus.
inline bool rssi_valid_state(uint8_t marc) {
    return marc >= 0x0D && marc <= 0x10;
}
inline std::string rssi_dbm() {
    if (!chip_alive()) return "-";
    uint8_t marc = st(0x35) & 0x1F;
    if (!rssi_valid_state(marc)) return "n/a";
    uint8_t raw = st(0x34);
    int rssi;
    if (raw < 128) rssi = (raw / 2) - 74;
    else           rssi = ((static_cast<int>(raw) - 256) / 2) - 74;
    char b[12];
    snprintf(b, sizeof(b), "%d", rssi);
    return b;
}
} // namespace rfstat

inline bool parse_uint(std::string_view in, uint32_t& out, uint32_t max) {
    uint64_t v = 0; bool any = false;
    for (char c : in) {
        if (c == ' ' || c == '\t') { if (any) break; continue; }
        if (c < '0' || c > '9') return false;
        v = v * 10 + static_cast<uint32_t>(c - '0');
        if (v > max) return false;
        any = true;
    }
    if (!any) return false;
    out = static_cast<uint32_t>(v);
    return true;
}

#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_TCP
// Netz-Zustand fuer Online-Update und Rollback-Gate.
bool net_configured() {
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
    if (g_wifi && g_wifi->config().mode != 0 && !g_wifi->config().ssid.empty()) return true;
#endif
#ifdef CONFIG_CULFW32_BUILD_NET_W5500
    if (g_eth && g_eth->present()) return true;
#endif
    return false;
}

bool net_has_ip() {
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
    if (g_wifi && g_wifi->ip4() != 0) return true;
#endif
#ifdef CONFIG_CULFW32_BUILD_NET_W5500
    if (g_eth && g_eth->ip4() != 0) return true;
#endif
    return false;
}

// Verbindungsschicht: Ethernet mit physischem Link oder WLAN assoziiert.
bool net_link_up() {
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
    if (g_wifi && g_wifi->associated()) return true;
#endif
#ifdef CONFIG_CULFW32_BUILD_NET_W5500
    if (g_eth) {
        const auto s = g_eth->state();
        if (s == EthManager::State::LinkUp || s == EthManager::State::GotIp) return true;
    }
#endif
    return false;
}

// Rollback-Gate: siehe OtaUpdate::NetHealth.
OtaUpdate::NetHealth ota_net_health() {
    if (!net_configured() || net_has_ip()) return OtaUpdate::NetHealth::Ok;
    return net_link_up() ? OtaUpdate::NetHealth::LinkNoIp : OtaUpdate::NetHealth::NoLink;
}

bool ota_auto_enabled() { return g_eeprom && g_eeprom->read_byte(ee::OTA_AUTO) == 1; }
#endif

void setup_config_registry(ConfigRegistry& cfg) {
    using E = ConfigEntry;
    using T = ConfigEntry::Type;

    // -- WiFi (nur wenn WIFI eingebaut) -------------------------------------
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
    cfg.register_entry({
        .key  = "wifi.ssid", .type = T::Str, .flags = E::FLAG_PERSIST,
        .getter = [] { return g_wifi ? g_wifi->config().ssid : std::string{}; },
        .setter = [](std::string_view v) {
            return g_wifi && g_wifi->set_ssid(v) == ESP_OK;
        },
        .help = "WiFi SSID (max 32 chars)",
    });
    cfg.register_entry({
        .key  = "wifi.psk", .type = T::Secret,
        .flags = static_cast<uint8_t>(E::FLAG_PERSIST | E::FLAG_SENSITIVE),
        .getter = [] { return g_wifi ? g_wifi->config().psk : std::string{}; },
        .setter = [](std::string_view v) {
            return g_wifi && g_wifi->set_psk(v) == ESP_OK;
        },
        .help = "WiFi pre-shared key (max 63 chars)",
    });
    cfg.register_entry({
        .key  = "wifi.mode", .type = T::IntEnum, .flags = E::FLAG_PERSIST,
        .min_val = 0, .max_val = 3,
        .enum_labels = {"off", "sta", "ap-fallback", "sta+ap"},
        .getter = [] {
            return g_wifi ? u_to_str(g_wifi->config().mode) : std::string{"0"};
        },
        .setter = [](std::string_view v) {
            uint32_t m = 0;
            if (!parse_uint(v, m, 3) || !g_wifi) return false;
            return g_wifi->set_mode(static_cast<uint8_t>(m)) == ESP_OK;
        },
        .help = "WiFi mode: 0=off 1=STA 2=AP-fallback 3=STA+AP",
    });
    cfg.register_entry({
        .key  = "wifi.hostname", .type = T::Str, .flags = E::FLAG_PERSIST,
        .getter = [] {
            if (!g_wifi) return std::string{};
            auto& c = g_wifi->config();
            return c.hostname.empty() ? std::string{g_wifi->default_hostname()} : c.hostname;
        },
        .setter = [](std::string_view v) {
            return g_wifi && g_wifi->set_hostname(v) == ESP_OK;
        },
        .help = "mDNS hostname (default culfw32-XXXXXX)",
    });
    cfg.register_entry({
        .key  = "wifi.state", .type = T::Str, .flags = E::FLAG_READONLY,
        .hass_device_class = "connectivity",
        .getter = [] {
            if (!g_wifi) return std::string{"off"};
            switch (g_wifi->state()) {
                case WifiManager::State::Off:          return std::string{"off"};
                case WifiManager::State::Disconnected: return std::string{"disconnected"};
                case WifiManager::State::Connecting:   return std::string{"connecting"};
                case WifiManager::State::Connected:    return std::string{"connected"};
            }
            return std::string{"?"};
        },
        .help = "Current WiFi association state",
    });
    cfg.register_entry({
        .key  = "wifi.ap", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return std::string{g_wifi && g_wifi->ap_active() ? "on" : "off"}; },
        .help = "Provisioning access point (open, SSID = hostname): on/off",
    });
    cfg.register_entry({
        .key  = "wifi.ip", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] {
            if (!g_wifi) return std::string{"0.0.0.0"};
            uint32_t ip = g_wifi->ip4();
            char b[24];
            snprintf(b, sizeof(b), "%u.%u.%u.%u",
                     (unsigned)(ip & 0xFF), (unsigned)((ip >> 8) & 0xFF),
                     (unsigned)((ip >> 16) & 0xFF), (unsigned)((ip >> 24) & 0xFF));
            return std::string{b};
        },
        .help = "Current IPv4 (0.0.0.0 if not connected)",
    });
    cfg.register_entry({
        .key  = "wifi.rssi", .type = T::Str, .flags = E::FLAG_READONLY,
        .hass_device_class = "signal_strength",
        .getter = [] {
            if (!g_wifi) return std::string{"0"};
            char b[8];
            snprintf(b, sizeof(b), "%d", g_wifi->rssi());
            return std::string{b};
        },
        .help = "Signal strength to current AP",
        .unit = "dBm",
    });
#endif // CONFIG_CULFW32_BUILD_NET_WIFI

    // -- culfw-EEPROM: Medium (I2C-EEPROM U1 oder nur NVS) ---------------------
    cfg.register_entry({
        .key  = "sys.eeprom", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] {
            if (!g_eeprom) return std::string{"none"};
            std::string s = g_eeprom->backend() == NvsEeprom::Backend::I2c
                                ? "i2c M24C32 + nvs mirror" : "nvs only";
            if (g_eeprom->i2c_errors()) s += " (i2c errors " + u_to_str(g_eeprom->i2c_errors()) + ")";
            return s;
        },
        .help = "Storage of the culfw EEPROM image (U1 survives reflashing, NVS does not)",
    });

    // -- Ethernet (W5500 an J1, nur wenn eingebaut) ---------------------------
#ifdef CONFIG_CULFW32_BUILD_NET_W5500
    cfg.register_entry({
        .key  = "eth.state", .type = T::Str, .flags = E::FLAG_READONLY,
        .hass_device_class = "connectivity",
        .getter = [] {
            return std::string{g_eth ? EthManager::state_name(g_eth->state()) : "absent"};
        },
        .help = "Ethernet (W5500 board at J1): absent/stopped/nolink/link/connected",
    });
    cfg.register_entry({
        .key  = "eth.ip", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] {
            uint32_t ip = g_eth ? g_eth->ip4() : 0;
            char b[24];
            snprintf(b, sizeof(b), "%u.%u.%u.%u",
                     (unsigned)(ip & 0xFF), (unsigned)((ip >> 8) & 0xFF),
                     (unsigned)((ip >> 16) & 0xFF), (unsigned)((ip >> 24) & 0xFF));
            return std::string{b};
        },
        .help = "Current Ethernet IPv4 (0.0.0.0 if none)",
    });
    cfg.register_entry({
        .key  = "eth.mac", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] {
            if (!g_eth || !g_eth->present()) return std::string{};
            uint8_t m[6];
            g_eth->mac(m);
            char b[40];
            snprintf(b, sizeof(b), "%02X:%02X:%02X:%02X:%02X:%02X (%s)",
                     m[0], m[1], m[2], m[3], m[4], m[5],
                     EthManager::mac_source_name(g_eth->mac_source()));
            return std::string{b};
        },
        .help = "Ethernet MAC and its source (eeprom/efuse/chip)",
    });
#endif // CONFIG_CULFW32_BUILD_NET_W5500

    // -- TCP-Server ----------------------------------------------------------
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_TCP
    cfg.register_entry({
        .key  = "tcp.port", .type = T::U16, .flags = E::FLAG_PERSIST,
        .min_val = 1, .max_val = 65535,
        .getter = [] {
            if (g_tcp && g_tcp->port()) return u_to_str(g_tcp->port());
            if (!g_eeprom) return u_to_str(ee::IP4_TCPLINK_PORT_DEFAULT);
            uint8_t lo = g_eeprom->read_byte(ee::IP4_TCPLINK_PORT);
            uint8_t hi = g_eeprom->read_byte(ee::IP4_TCPLINK_PORT + 1);
            uint16_t p = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
            if (p == 0 || p == 0xFFFF) p = ee::IP4_TCPLINK_PORT_DEFAULT;
            return u_to_str(p);
        },
        .setter = [](std::string_view v) {
            uint32_t p = 0;
            if (!parse_uint(v, p, 65535) || p == 0) return false;
            if (!g_eeprom) return false;
            g_eeprom->write_byte(ee::IP4_TCPLINK_PORT,     static_cast<uint8_t>(p & 0xFF));
            g_eeprom->write_byte(ee::IP4_TCPLINK_PORT + 1, static_cast<uint8_t>((p >> 8) & 0xFF));
            if (g_tcp) g_tcp->restart(static_cast<uint16_t>(p));
            return true;
        },
        .help = "TCP server listen port (a-culfw EE_IP4_TCPLINK_PORT)",
    });
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_JSON
    cfg.register_entry({
        .key  = "json.port", .type = T::U16, .flags = E::FLAG_PERSIST,
        .min_val = 1, .max_val = 65535,
        .getter = [] {
            if (!g_eeprom) return u_to_str(ee::TCP_JSON_PORT_DEFAULT);
            uint8_t lo = g_eeprom->read_byte(ee::TCP_JSON_PORT);
            uint8_t hi = g_eeprom->read_byte(ee::TCP_JSON_PORT + 1);
            uint16_t p = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
            if (p == 0 || p == 0xFFFF) p = ee::TCP_JSON_PORT_DEFAULT;
            return u_to_str(p);
        },
        .setter = [](std::string_view v) {
            uint32_t p = 0;
            if (!parse_uint(v, p, 65535) || p == 0 || !g_eeprom) return false;
            g_eeprom->write_byte(ee::TCP_JSON_PORT,     static_cast<uint8_t>(p & 0xFF));
            g_eeprom->write_byte(ee::TCP_JSON_PORT + 1, static_cast<uint8_t>((p >> 8) & 0xFF));
            if (g_tcp_json) g_tcp_json->restart(static_cast<uint16_t>(p));
            return true;
        },
        .help = "JSON-Frontend TCP listen port",
    });
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_SIGNALDUINO
    cfg.register_entry({
        .key  = "sd.port", .type = T::U16, .flags = E::FLAG_PERSIST,
        .min_val = 1, .max_val = 65535,
        .getter = [] {
            if (!g_eeprom) return u_to_str(ee::TCP_SD_PORT_DEFAULT);
            uint8_t lo = g_eeprom->read_byte(ee::TCP_SD_PORT);
            uint8_t hi = g_eeprom->read_byte(ee::TCP_SD_PORT + 1);
            uint16_t p = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
            if (p == 0 || p == 0xFFFF) p = ee::TCP_SD_PORT_DEFAULT;
            return u_to_str(p);
        },
        .setter = [](std::string_view v) {
            uint32_t p = 0;
            if (!parse_uint(v, p, 65535) || p == 0 || !g_eeprom) return false;
            g_eeprom->write_byte(ee::TCP_SD_PORT,     static_cast<uint8_t>(p & 0xFF));
            g_eeprom->write_byte(ee::TCP_SD_PORT + 1, static_cast<uint8_t>((p >> 8) & 0xFF));
            if (g_tcp_sd) g_tcp_sd->restart(static_cast<uint16_t>(p));
            return true;
        },
        .help = "SIGNALduino-Frontend TCP listen port",
    });
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_HMUARTLGW
    cfg.register_entry({
        .key  = "hmu.port", .type = T::U16, .flags = E::FLAG_PERSIST,
        .min_val = 1, .max_val = 65535,
        .getter = [] {
            if (!g_eeprom) return u_to_str(ee::HMU_PORT_DEFAULT);
            uint8_t lo = g_eeprom->read_byte(ee::HMU_PORT);
            uint8_t hi = g_eeprom->read_byte(ee::HMU_PORT + 1);
            uint16_t p = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
            if (p == 0 || p == 0xFFFF) p = ee::HMU_PORT_DEFAULT;
            return u_to_str(p);
        },
        .setter = [](std::string_view v) {
            uint32_t p = 0;
            if (!parse_uint(v, p, 65535) || p == 0 || !g_eeprom) return false;
            g_eeprom->write_byte(ee::HMU_PORT,     static_cast<uint8_t>(p & 0xFF));
            g_eeprom->write_byte(ee::HMU_PORT + 1, static_cast<uint8_t>((p >> 8) & 0xFF));
            if (g_tcp_hmu) g_tcp_hmu->restart(static_cast<uint16_t>(p));
            return true;
        },
        .help = "HMUARTLGW TCP listen port (FHEM define: HMUARTLGW uart://<ip>:<port>)",
    });
    cfg.register_entry({
        .key  = "hmu.hmid", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] {
            if (!g_eeprom) return std::string{"FFFFFF"};
            uint8_t b[3] = {};
            g_eeprom->read_block(ee::HMU_HMID, std::span<uint8_t>(b, 3));
            char s[8]; snprintf(s, sizeof(s), "%02X%02X%02X", b[0], b[1], b[2]);
            return std::string{s};
        },
        .help = "Current 3-byte HMID (set by FHEM via APP_SET_HMID)",
    });
    cfg.register_entry({
        .key  = "hmu.module", .type = T::IntEnum, .flags = E::FLAG_PERSIST,
        .min_val = 0, .max_val = 1,
        .enum_labels = {"hm-mod-rpi-pcb", "rpi-rf-mod"},
        .getter = [] {
            if (!g_eeprom) return std::string{"0"};
            uint8_t m = g_eeprom->read_byte(ee::HMU_MODULE_MODE);
            return std::string{m == ee::HMU_MODULE_DUAL ? "1" : "0"};
        },
        .setter = [](std::string_view v) {
            uint32_t n = 0;
            if (!parse_uint(v, n, 1)) return false;
            uint8_t m = static_cast<uint8_t>(n);
            if (g_eeprom) g_eeprom->write_byte(ee::HMU_MODULE_MODE, m);
            if (g_fe_hmu) g_fe_hmu->set_module_mode(
                m == ee::HMU_MODULE_DUAL
                    ? HmuartlgwFrontend::ModuleMode::Dual
                    : HmuartlgwFrontend::ModuleMode::Legacy);
            if (g_hbrfeth) g_hbrfeth->set_module_identity(m == ee::HMU_MODULE_DUAL);
            return true;
        },
        .help = "HMUARTLGW default mode for statically bound transports "
                "(USB-CDC, UART): 0=hm-mod-rpi-pcb (FHEM/Homegear, default), "
                "1=rpi-rf-mod (DualCoPro). The TCP listeners on hmu.port (legacy) "
                "and hmu.port+2 (dual) keep their own fixed identity, so both "
                "modes are available at the same time without switching.",
    });
#endif

    cfg.register_entry({
        .key  = "tcp.allow", .type = T::Ipv4Cidr, .flags = E::FLAG_PERSIST,
        .getter = [] {
            if (!g_tcp || g_tcp->allow_bits() == 0) return std::string{"any"};
            uint32_t ip = g_tcp->allow_ip();
            char b[24];
            snprintf(b, sizeof(b), "%u.%u.%u.%u/%u",
                     (unsigned)(ip & 0xFF), (unsigned)((ip >> 8) & 0xFF),
                     (unsigned)((ip >> 16) & 0xFF), (unsigned)((ip >> 24) & 0xFF),
                     (unsigned)g_tcp->allow_bits());
            return std::string{b};
        },
        // Setter: noch nicht implementiert (komplexes parsing) — aktuell via
        // ASCII-Frontend EWA. Folge-Patch wenn JSON-Frontend kommt.
        .help = "TCP allow-subnet (\"any\" or <ipv4>/<bits>)",
    });
#endif // CONFIG_CULFW32_BUILD_TRANSPORT_TCP

    // -- RF / CC1101 ---------------------------------------------------------
    // Frequenzoffset (CC1101 FSCTRL0). Board-Eigenschaft, kein Protokoll-
    // Parameter — siehe components/freq_cal. Quelle: EE-Override > eFuse > 0.
    cfg.register_entry({
        .key  = "rf.freqoff", .type = T::Str, .flags = E::FLAG_PERSIST,
        .getter = [] {
            if (!g_slot0) return std::string{"0"};
            int want = g_slot0->driver().freq_offset();
            // Was im CHIP steht, kann abweichen: `Cw0C..` schreibt roh, und
            // `Ar`/`Zr` sind idempotent — steht der Mode schon, wird die CFG
            // nicht neu geladen und der Handwert bleibt stehen. Genau darauf
            // bin ich am Bench hereingefallen (Getter sagte -29, im Chip stand
            // +32, kein Empfang). Deshalb beide zeigen, wenn sie auseinander
            // laufen; `?rf.freqoff=<n>` macht den kalibrierten Wert wieder scharf.
            uint8_t chip = 0;
            if (g_slot0->driver().read_reg(CC1101Driver::REG_FSCTRL0, chip) == ESP_OK) {
                int have = static_cast<int8_t>(chip);
                if (have != want) {
                    char b[40];
                    std::snprintf(b, sizeof(b), "%d (chip %d)", want, have);
                    return std::string{b};
                }
            }
            return std::to_string(want);
        },
        .setter = [](std::string_view v) {
            if (!g_slot0 || !g_eeprom) return false;
            std::string t(v);
            // "auto"/"" loescht den EE-Override; danach gilt wieder eFuse bzw. 0.
            if (t.empty() || t == "auto" || t == "-") {
                freqcal::nvs_clear(*g_eeprom);
                freqcal::Source src = freqcal::Source::None;
                int8_t off = freqcal::effective(g_eeprom.get(), src);
                g_slot0->driver().set_freq_offset(off);
                g_slot0->driver().apply_freq_offset();
                return true;
            }
            char* end = nullptr;
            long n = std::strtol(t.c_str(), &end, 10);
            if (end == t.c_str() || *end != '\0' || n < -128 || n > 127) return false;
            freqcal::nvs_write(*g_eeprom, static_cast<int8_t>(n));
            g_slot0->driver().set_freq_offset(static_cast<int8_t>(n));
            g_slot0->driver().apply_freq_offset();
            return true;
        },
        .help = "CC1101 FSCTRL0 offset in LSB (1 LSB = 1.587 kHz), \"auto\" = clear override",
        .unit = "LSB",
    });
    cfg.register_entry({
        .key  = "rf.freqoff.khz", .type = T::Str,
        .flags = static_cast<uint8_t>(E::FLAG_READONLY),
        .getter = [] {
            if (!g_slot0) return std::string{"0.0"};
            char b[16];
            std::snprintf(b, sizeof(b), "%.1f",
                          g_slot0->driver().freq_offset() * freqcal::kHzPerLsb);
            return std::string{b};
        },
        .help = "Applied frequency offset",
        .unit = "kHz",
    });
    cfg.register_entry({
        .key  = "rf.freqoff.src", .type = T::Str,
        .flags = static_cast<uint8_t>(E::FLAG_READONLY),
        .getter = [] {
            freqcal::Source src = freqcal::Source::None;
            (void)freqcal::effective(g_eeprom.get(), src);
            return std::string{freqcal::source_name(src)};
        },
        .help = "Where the offset came from: nvs (field) | efuse (factory) | none",
    });
    // Messwert des laufenden Sammlers. Die FSK-RX-Pfade legen nach jedem
    // gueltigen Frame ein FREQEST-Sample ab (SWRS061I §14.1 — zwischen Frames
    // traeckt der Kompensator Rauschen, deshalb nur frame-synchron).
    cfg.register_entry({
        .key  = "rf.freqest", .type = T::Str,
        .flags = static_cast<uint8_t>(E::FLAG_READONLY),
        .getter = [] {
            if (!g_slot0) return std::string{"n=0"};
            int8_t med = 0; size_t n = 0;
            if (!g_slot0->driver().freqest_median(med, n)) return std::string{"n=0"};
            char b[48];
            std::snprintf(b, sizeof(b), "%d n=%u (%.1f kHz)", static_cast<int>(med),
                          static_cast<unsigned>(n), med * freqcal::kHzPerLsb);
            return std::string{b};
        },
        .help = "Median of FREQEST samples taken on received FSK frames",
    });
    // Bedienung der Auto-Kalibrierung. `apply` uebernimmt den gemessenen Median
    // als neuen Offset: FSCTRL0_neu = FSCTRL0_alt + FREQEST (DN015 —
    // "By writing the value from FREQEST into FSCTRL0.FREQOFF"; der Schaetzwert
    // ist relativ zur bereits eingestellten Frequenz, also aufaddieren).
    cfg.register_entry({
        .key  = "rf.freqcal", .type = T::Str, .flags = 0,
        .getter = [] {
            auto st = freqcal::scan_state();
            char b[96];
            if (st.running) {
                std::snprintf(b, sizeof(b), "scanning...");
            } else if (st.finished) {
                std::snprintf(b, sizeof(b),
                              "%s step=%d median=%d frames=%u peak=%ddBm -> %d (%.1f kHz)",
                              st.ok ? "ok" : "no-signal", static_cast<int>(st.best_step),
                              static_cast<int>(st.median), static_cast<unsigned>(st.samples),
                              static_cast<int>(st.rssi_dbm), static_cast<int>(st.result),
                              st.result * freqcal::kHzPerLsb);
            } else {
                std::snprintf(b, sizeof(b), "idle (scan|adopt|apply|reset|clear)");
            }
            return std::string{b};
        },
        .setter = [](std::string_view v) {
            if (!g_slot0 || !g_eeprom) return false;
            std::string t(v);
            if (t == "reset") { g_slot0->driver().freqest_reset(); return true; }
            if (t == "scan" || t.rfind("scan:", 0) == 0) {
                uint32_t dwell = 1200;
                if (t.size() > 5) {
                    char* e = nullptr;
                    long d = std::strtol(t.c_str() + 5, &e, 10);
                    if (e && *e == '\0' && d >= 100 && d <= 10000) dwell = static_cast<uint32_t>(d);
                }
                return freqcal::scan_start(g_slot0->driver(), dwell) == ESP_OK;
            }
            if (t == "adopt") {
                // Ergebnis des Suchlaufs uebernehmen und persistieren.
                auto st = freqcal::scan_state();
                if (!st.finished || !st.ok) return false;
                freqcal::nvs_write(*g_eeprom, st.result);
                g_slot0->driver().set_freq_offset(st.result);
                g_slot0->driver().apply_freq_offset();
                g_slot0->driver().freqest_reset();
                return true;
            }
            if (t == "clear") {
                freqcal::nvs_clear(*g_eeprom);
                freqcal::Source src = freqcal::Source::None;
                int8_t off = freqcal::effective(g_eeprom.get(), src);
                g_slot0->driver().set_freq_offset(off);
                g_slot0->driver().apply_freq_offset();
                return true;
            }
            if (t != "apply") return false;
            int8_t med = 0; size_t n = 0;
            if (!g_slot0->driver().freqest_median(med, n) || n < 4) return false;
            int sum = g_slot0->driver().freq_offset() + med;
            if (sum < -128 || sum > 127) return false;
            freqcal::nvs_write(*g_eeprom, static_cast<int8_t>(sum));
            g_slot0->driver().set_freq_offset(static_cast<int8_t>(sum));
            g_slot0->driver().apply_freq_offset();
            g_slot0->driver().freqest_reset();
            return true;
        },
        .help = "Frequency auto-calibration: scan[:dwell_ms] (carrier search) | adopt (take scan result) | apply (take FREQEST median of received frames) | reset | clear",
    });

    cfg.register_entry({
        .key  = "rf.pa.idx", .type = T::IntEnum, .flags = E::FLAG_PERSIST,
        .min_val = 0, .max_val = 9,
        .enum_labels = {
            "-10dBm/ramp", "-5dBm/ramp", "0dBm/ramp", "+5dBm/ramp", "+10dBm/ramp",
            "-10dBm",      "-5dBm",      "0dBm",      "+5dBm",      "+10dBm",
        },
        .getter = [] {
            // Index aus EE-PATABLE-Profil rueckwaerts ableiten — vergleicht
            // PATABLE[1] gegen die Default-Tabellen-Spalte. Bei nicht-Match:
            // "?" zurueck (User hat manuell W gesetzt).
            if (!g_eeprom) return std::string{"?"};
            uint8_t pa[ee::CC1100_PA_SIZE];
            g_eeprom->read_block(ee::CC1100_PA, std::span<uint8_t>(pa, sizeof(pa)));
            uint8_t frend0 = g_eeprom->read_byte(ee::CC1100_CFG + 0x22);
            for (int idx = 0; idx <= 9; ++idx) {
                bool match = true;
                for (size_t i = 0; i < ee::CC1100_PA_SIZE; ++i) {
                    if (pa[i] != ee::CC1100_PA_DEFAULTS[idx * ee::CC1100_PA_SIZE + i]) {
                        match = false; break;
                    }
                }
                uint8_t want_frend = (idx > 4 && idx < 10) ? 0x11 : 0x17;
                if (match && frend0 == want_frend) return u_to_str(idx);
            }
            return std::string{"?"};
        },
        .setter = [](std::string_view v) {
            uint32_t idx = 0;
            if (!parse_uint(v, idx, 99) || !g_eeprom) return false;
            cc_set_pa(*g_eeprom, static_cast<uint8_t>(idx));
            // Live-Apply auf alle CC1101-Slots — analog handle_x.
            if (g_mgr) {
                uint8_t pa[ee::CC1100_PA_SIZE];
                g_eeprom->read_block(ee::CC1100_PA, std::span<uint8_t>(pa, sizeof(pa)));
                uint8_t frend0 = g_eeprom->read_byte(ee::CC1100_CFG + 0x22);
                for (uint8_t s = 0; s < g_mgr->slot_count(); ++s) {
                    if (auto e = g_mgr->cc_write_burst(s, 0x3E,
                            std::span<const uint8_t>(pa, sizeof(pa)));
                        e != ESP_OK) {
                        ESP_LOGW(TAG, "rf.pa.idx: slot %u PATABLE burst failed: %s",
                                 s, esp_err_to_name(e));
                    }
                    if (auto e = g_mgr->cc_write_reg(s, 0x22, frend0); e != ESP_OK) {
                        ESP_LOGW(TAG, "rf.pa.idx: slot %u FREND0 write failed: %s",
                                 s, esp_err_to_name(e));
                    }
                }
            }
            return true;
        },
        .help = "PA table (a-culfw FULL_CC1100_PA): 5..9 = -10/-5/0/+5/+10 dBm without ramping "
                "(8 = factory default). 0..4 = the same levels WITH PA ramping - for a-culfw "
                "compatibility only, not recommended for OOK/FS20: the ramp runs at every bit "
                "edge (measured ~28 us instead of ~4 us), original FS20 devices do not ramp.",
    });

    // -- LED -----------------------------------------------------------------
    cfg.register_entry({
        .key  = "led.mode", .type = T::IntEnum, .flags = E::FLAG_PERSIST,
        .min_val = 0, .max_val = 4,
        .enum_labels = {"off", "on", "blink-slow", "blink-fast", "pulse"},
        .getter = [] {
            return g_led ? u_to_str(g_led->mode()) : std::string{"0"};
        },
        .setter = [](std::string_view v) {
            uint32_t m = 0;
            if (!parse_uint(v, m, 4) || !g_led) return false;
            g_led->set_mode(static_cast<uint8_t>(m));
            if (g_eeprom) g_eeprom->write_byte(ee::LED, static_cast<uint8_t>(m));
            return true;
        },
        .help = "LED mode (0=off 1=on 2..4=patterns), persists in EE_LED",
    });

    // -- Boot/Diagnose -------------------------------------------------------
    cfg.register_entry({
        .key  = "boot.reqbl", .type = T::U8, .flags = E::FLAG_PERSIST,
        .min_val = 0, .max_val = 1,
        .getter = [] {
            if (!g_eeprom) return std::string{"0"};
            return u_to_str(g_eeprom->read_byte(ee::REQBL));
        },
        .setter = [](std::string_view v) {
            uint32_t r = 0;
            if (!parse_uint(v, r, 1) || !g_eeprom) return false;
            g_eeprom->write_byte(ee::REQBL, static_cast<uint8_t>(r));
            return true;
        },
        .help = "Bootloader request flag (a-culfw EE_REQBL)",
    });
    // -- Build-Info: aktive Module zur Laufzeit sichtbar -------------------
    cfg.register_entry({
        .key  = "build", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] {
            std::string out;
            auto add = [&](const char* tag) {
                if (!out.empty()) out += ',';
                out += tag;
            };
#ifdef CONFIG_CULFW32_BUILD_RF_MORITZ
            add("rf:moritz");
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_ASKSIN
            add("rf:asksin");
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_SLOWRF
            add("rf:slowrf");
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_NATIVE
            add("rf:native");
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_FASTRF
            add("rf:fastrf");
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_DUDETTE
            add("rf:dudette");
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_WMBUS
            add("rf:wmbus");
            add("rf:hmip");
#endif
            add("tx:usbcdc");
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_UART
            add("tx:uart");
#endif
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_TCP
            add("tx:tcp");
#endif
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_HBRFETH_UDP
            add("tx:hbrfeth");
#endif
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
            add("net:wifi");
#endif
#ifdef CONFIG_CULFW32_BUILD_NET_W5500
            add("net:w5500");
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_JSON
            add("fe:json");
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_SIGNALDUINO
            add("fe:signalduino");
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_HMUARTLGW
            add("fe:hmuartlgw");
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_WEBUI
            add("fe:webui");
#endif
            return out;
        },
        .help = "Comma-separated list of the modules compiled into this image",
    });
    // -- Firmware-Version (Schema MAJOR.MINOR.<n>, n autom. pro Build) ------
    cfg.register_entry({
        .key  = "fw.version", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return std::string{CULFW_VERSION}; },
        .help = "Firmware version (MAJOR.MINOR.<n>, n = build number within the series)",
    });
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_TCP
    // -- Online-Update (install.busware.de/CUL/) ------------------------------
    cfg.register_entry({
        .key  = "ota", .type = T::Str, .flags = 0,
        .getter = [] {
            char b[128];
            OtaUpdate::instance().status_line(b, sizeof(b));
            return std::string{b};
        },
        .setter = [](std::string_view v) {
            if (v == "check")   return OtaUpdate::instance().request_check() == ESP_OK;
            if (v == "install") return OtaUpdate::instance().request_install() == ESP_OK;
            return false;
        },
        .help = "Online update from install.busware.de: check | install",
    });
    // Maschinenlesbar fuer die Web-UI (Firmware-Karte); readonly = Live-Push.
    cfg.register_entry({
        .key  = "ota.state", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return std::string{OtaUpdate::instance().state_word()}; },
        .help = "idle | checking | uptodate | available | installing | error | validating",
    });
    cfg.register_entry({
        .key  = "ota.latest", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] {
            char b[24];
            OtaUpdate::instance().latest_version(b, sizeof(b));
            return std::string{b};
        },
        .help = "Version in the update manifest at the last check (empty = not checked yet)",
    });
    cfg.register_entry({
        .key  = "ota.progress", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return std::to_string(OtaUpdate::instance().progress()); },
        .help = "Download progress in percent while installing, else -1",
    });
    cfg.register_entry({
        .key  = "ota.auto", .type = T::IntEnum, .flags = E::FLAG_PERSIST,
        .min_val = 0, .max_val = 1,
        .enum_labels = {"notify", "install"},
        .getter = [] { return std::string{ota_auto_enabled() ? "1" : "0"}; },
        .setter = [](std::string_view v) {
            uint32_t m = 0;
            if (!parse_uint(v, m, 1) || !g_eeprom) return false;
            return g_eeprom->write_byte(ee::OTA_AUTO, static_cast<uint8_t>(m)) == ESP_OK;
        },
        .help = "Daily update check: 0 = report only, 1 = install automatically",
    });
    cfg.register_entry({
        .key  = "ota.url", .type = T::Str, .flags = 0,
        .getter = [] {
            char b[160];
            OtaUpdate::instance().manifest_url(b, sizeof(b));
            return std::string{b};
        },
        .setter = [](std::string_view v) {
            std::string u(v);
            OtaUpdate::instance().set_manifest_url(u == "default" ? "" : u.c_str());
            return true;
        },
        .help = "Update manifest URL (RAM only, 'default' restores the built-in one)",
    });
#endif

    cfg.register_entry({
        .key  = "fw.build", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return std::string{CULFW_BUILD_NUM}; },
        .help = "Build counter (also in version.txt and esp_app_desc_t::version)",
    });
    cfg.register_entry({
        .key  = "fw.build_date", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return std::string{CULFW_BUILD_DATE}; },
        .help = "Build date (__DATE__ at compile time)",
    });
    // -- RF-Status (Live-Push pickt diese 9 Felder alle 5 s auf) -----------
    cfg.register_entry({
        .key  = "rf.chip", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return rfstat::chip_name(); },
        .help = "CC1101 chip ID (detected via PARTNUM/VERSION)",
    });
    cfg.register_entry({
        .key  = "rf.proto", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return rfstat::active_proto(); },
        .help = "Active RF protocol on slot 0",
    });
    cfg.register_entry({
        .key  = "rf.freq", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return rfstat::freq_mhz(); },
        .help = "Current carrier frequency from FREQ2:1:0",
        .unit = "MHz",
    });
    cfg.register_entry({
        .key  = "rf.mod", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return rfstat::modulation(); },
        .help = "Modulation from MDMCFG2.MOD_FORMAT",
    });
    cfg.register_entry({
        .key  = "rf.datarate", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return rfstat::datarate_kbps(); },
        .help = "Data rate from MDMCFG3:4 (DRATE_M, DRATE_E)",
        .unit = "kbps",
    });
    cfg.register_entry({
        .key  = "rf.bw", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return rfstat::bw_khz(); },
        .help = "RX bandwidth from MDMCFG4 (CHANBW_E, CHANBW_M)",
        .unit = "kHz",
    });
    cfg.register_entry({
        .key  = "rf.deviation", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return rfstat::deviation_khz(); },
        .help = "FSK deviation from DEVIATN (FSK modes only)",
        .unit = "kHz",
    });
    cfg.register_entry({
        .key  = "rf.state", .type = T::Str, .flags = E::FLAG_READONLY,
        .hass_device_class = "connectivity",
        .getter = [] { return rfstat::marc_state(); },
        .help = "Decoded MARCSTATE (IDLE/RX/TX/RXOVF/TXUDF)",
    });
    cfg.register_entry({
        .key  = "rf.rssi", .type = T::Str, .flags = E::FLAG_READONLY,
        .hass_device_class = "signal_strength",
        .getter = [] { return rfstat::rssi_dbm(); },
        .help = "Current RSSI from the CC1101 - in RX without a frame this is the noise floor",
        .unit = "dBm",
    });

    cfg.register_entry({
        .key  = "fht.id", .type = T::Str, .flags = E::FLAG_PERSIST,
        .getter = [] {
            if (!g_eeprom) return std::string{"FFFF"};
            uint8_t hi = g_eeprom->read_byte(ee::FHTID);
            uint8_t lo = g_eeprom->read_byte(ee::FHTID + 1);
            char b[8];
            snprintf(b, sizeof(b), "%02X%02X", hi, lo);
            return std::string{b};
        },
        // Setter: ueber ASCII-Frontend `T01<hh><cc>` — Setter hier optional
        // weil FHT-Init-Code im SlowRf-Frontend liegt. Bei JSON-Frontend
        // koennen wir den Setter-Pfad explizit nachziehen.
        .help = "FHT housecode (a-culfw EE_FHTID, hex 4 chars)",
    });

    // -- Stats: RF/System (alle FLAG_READONLY → automatischer Live-Push 5s) ----
    // Hilfs-Helper damit der Block kompakt bleibt.
    auto reg_stats_u32 = [&](const char* key, std::function<uint32_t()> fn,
                             const char* help, const char* unit = "",
                             const char* hass = "") {
        cfg.register_entry({
            .key = key, .type = T::U32, .flags = E::FLAG_READONLY,
            .hass_device_class = hass,
            .getter = [fn = std::move(fn)] { return u_to_str(fn()); },
            .help = help, .unit = unit,
        });
    };

    reg_stats_u32("stats.rf.credit",
                  [] { return (uint32_t)RfStats::instance().credit(); },
                  "Current TX credit (10 ms units, a-culfw credit_10ms; +1/s)");
    reg_stats_u32("stats.rf.credit_max",
                  [] { return (uint32_t)RfStats::instance().credit_max(); },
                  "Maximum TX credit (a-culfw MAX_CREDIT = 900 = 9 s burst)");
    reg_stats_u32("stats.rf.credit_pct",
                  [] { return (uint32_t)RfStats::instance().credit_pct(); },
                  "TX credit in percent of the maximum", "%");
    reg_stats_u32("stats.rf.lovf",
                  [] { return RfStats::instance().lovf_count(); },
                  "TX attempts rejected because the TX credit was used up");
    reg_stats_u32("stats.rf.tx_total",
                  [] { return RfStats::instance().tx_total(); },
                  "TX frames, sum over all protocols");
    reg_stats_u32("stats.rf.rx_total",
                  [] { return RfStats::instance().rx_total(); },
                  "RX frames, sum over all protocols");
    reg_stats_u32("stats.rf.tx_airtime",
                  [] { return RfStats::instance().tx_airtime_10ms_total(); },
                  "Total TX airtime in 10 ms units (1 % duty cycle = 3600 per hour)");

    // Pro-Protokoll-Counters. Lambda-Loop waere knackiger, geht aber nicht durch
    // ConfigEntry-getter (Lambda kann nicht parametrisiert werden ohne capture).
    static const std::pair<const char*, ProtocolId> kProtos[] = {
        {"moritz", ProtocolId::Moritz},
        {"asksin", ProtocolId::AskSin},
        {"slowrf", ProtocolId::SlowRf},
        {"native", ProtocolId::Native},
        {"fastrf", ProtocolId::FastRf},
        {"dudette", ProtocolId::Dudette},
        {"pulse",  ProtocolId::PulseCapture},
        {"hmip",   ProtocolId::Hmip},
    };
    for (const auto& kv : kProtos) {
        const char* name = kv.first;
        ProtocolId  pid  = kv.second;
        char keybuf[40];
        char helpbuf[80];

        snprintf(keybuf,  sizeof(keybuf),  "stats.%s.rx_msgs", name);
        snprintf(helpbuf, sizeof(helpbuf), "%s RX-Frames", name);
        cfg.register_entry({
            .key = std::string(keybuf), .type = T::U32, .flags = E::FLAG_READONLY,
            .getter = [pid] { return u_to_str(RfStats::instance().snapshot(pid).rx_msgs); },
            .help = std::string(helpbuf),
        });
        snprintf(keybuf,  sizeof(keybuf),  "stats.%s.tx_msgs", name);
        snprintf(helpbuf, sizeof(helpbuf), "%s TX-Frames", name);
        cfg.register_entry({
            .key = std::string(keybuf), .type = T::U32, .flags = E::FLAG_READONLY,
            .getter = [pid] { return u_to_str(RfStats::instance().snapshot(pid).tx_msgs); },
            .help = std::string(helpbuf),
        });
        snprintf(keybuf,  sizeof(keybuf),  "stats.%s.rx_bytes", name);
        snprintf(helpbuf, sizeof(helpbuf), "%s RX-Bytes", name);
        cfg.register_entry({
            .key = std::string(keybuf), .type = T::U32, .flags = E::FLAG_READONLY,
            .getter = [pid] { return u_to_str(RfStats::instance().snapshot(pid).rx_bytes); },
            .help = std::string(helpbuf), .unit = "B",
        });
        snprintf(keybuf,  sizeof(keybuf),  "stats.%s.tx_bytes", name);
        snprintf(helpbuf, sizeof(helpbuf), "%s TX-Bytes", name);
        cfg.register_entry({
            .key = std::string(keybuf), .type = T::U32, .flags = E::FLAG_READONLY,
            .getter = [pid] { return u_to_str(RfStats::instance().snapshot(pid).tx_bytes); },
            .help = std::string(helpbuf), .unit = "B",
        });
        snprintf(keybuf,  sizeof(keybuf),  "stats.%s.tx_airtime", name);
        snprintf(helpbuf, sizeof(helpbuf), "%s TX-Airtime in 10ms-Units", name);
        cfg.register_entry({
            .key = std::string(keybuf), .type = T::U32, .flags = E::FLAG_READONLY,
            .getter = [pid] { return u_to_str(RfStats::instance().snapshot(pid).tx_airtime_10ms); },
            .help = std::string(helpbuf),
        });
        snprintf(keybuf,  sizeof(keybuf),  "stats.%s.rx_err", name);
        snprintf(helpbuf, sizeof(helpbuf), "%s RX-Errors (CRC, FIFO, malformed)", name);
        cfg.register_entry({
            .key = std::string(keybuf), .type = T::U32, .flags = E::FLAG_READONLY,
            .getter = [pid] { return u_to_str(RfStats::instance().snapshot(pid).rx_err); },
            .help = std::string(helpbuf),
        });
        snprintf(keybuf,  sizeof(keybuf),  "stats.%s.tx_err", name);
        snprintf(helpbuf, sizeof(helpbuf), "%s TX-Errors (timeout, CCA, LOVF, MARC)", name);
        cfg.register_entry({
            .key = std::string(keybuf), .type = T::U32, .flags = E::FLAG_READONLY,
            .getter = [pid] { return u_to_str(RfStats::instance().snapshot(pid).tx_err); },
            .help = std::string(helpbuf),
        });
        snprintf(keybuf,  sizeof(keybuf),  "stats.%s.crc_err", name);
        snprintf(helpbuf, sizeof(helpbuf), "%s CRC-Errors (subset of rx_err)", name);
        cfg.register_entry({
            .key = std::string(keybuf), .type = T::U32, .flags = E::FLAG_READONLY,
            .getter = [pid] { return u_to_str(RfStats::instance().snapshot(pid).crc_err); },
            .help = std::string(helpbuf),
        });
        snprintf(keybuf,  sizeof(keybuf),  "stats.%s.last_rssi", name);
        snprintf(helpbuf, sizeof(helpbuf), "%s last RX-RSSI", name);
        cfg.register_entry({
            .key = std::string(keybuf), .type = T::Str, .flags = E::FLAG_READONLY,
            .hass_device_class = "signal_strength",
            .getter = [pid] {
                auto s = RfStats::instance().snapshot(pid);
                if (s.rx_msgs == 0) return std::string{"n/a"};
                char b[8]; snprintf(b, sizeof(b), "%ld", (long)s.last_rssi_dbm);
                return std::string{b};
            },
            .help = std::string(helpbuf), .unit = "dBm",
        });
        snprintf(keybuf,  sizeof(keybuf),  "stats.%s.last_rx_ago", name);
        snprintf(helpbuf, sizeof(helpbuf), "%s seconds since last RX", name);
        cfg.register_entry({
            .key = std::string(keybuf), .type = T::Str, .flags = E::FLAG_READONLY,
            .getter = [pid] {
                uint32_t ago = RfStats::instance().last_rx_ago_s(pid);
                if (ago == UINT32_MAX) return std::string{"n/a"};
                return u_to_str(ago);
            },
            .help = std::string(helpbuf), .unit = "s",
        });
    }

    // AskSin-spezifische Diagnostik: letzter on-air TX-Header. Beantwortet
    // "ist APP_SET_HMID wirklich aktiv?" ohne RF-Sniffer.
    cfg.register_entry({
        .key = "stats.bidcos.last_tx_src_hmid", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] {
            auto l = AskSinRF::last_tx();
            if (!l.valid) return std::string{"n/a"};
            char b[8]; snprintf(b, sizeof(b), "%06lX", (unsigned long)(l.src_hmid & 0xFFFFFFu));
            return std::string{b};
        },
        .help = "AskSin: sender HMID of the last frame sent on air (bytes 4..6, hex)",
    });
    cfg.register_entry({
        .key = "stats.bidcos.last_tx_dst_hmid", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] {
            auto l = AskSinRF::last_tx();
            if (!l.valid) return std::string{"n/a"};
            char b[8]; snprintf(b, sizeof(b), "%06lX", (unsigned long)(l.dst_hmid & 0xFFFFFFu));
            return std::string{b};
        },
        .help = "AskSin: destination HMID of the last frame sent on air (bytes 7..9, hex)",
    });
    cfg.register_entry({
        .key = "stats.bidcos.last_tx_cnt", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] {
            auto l = AskSinRF::last_tx();
            if (!l.valid) return std::string{"n/a"};
            char b[6]; snprintf(b, sizeof(b), "%u", (unsigned)l.cnt);
            return std::string{b};
        },
        .help = "AskSin: message counter of the last frame sent on air (byte 1)",
    });

    // System.
    reg_stats_u32("stats.sys.heap_free",
                  [] { return RfStats::instance().heap_free(); },
                  "Free heap (default caps)", "B");
    reg_stats_u32("stats.sys.heap_min",
                  [] { return RfStats::instance().heap_min(); },
                  "Lowest free heap since boot (watermark)", "B");

    // System Monitor: Uptime, Reset-Reason, Boot-/Crash-Counter.
    SystemMonitor::instance().register_with_registry(cfg);
}

void smoke_test_cc1101(CC1101Slot& slot) {
    auto& drv = slot.driver();
    uint8_t partnum = 0xAA, version = 0xAA;
    auto e1 = drv.read_status(CC1101Driver::REG_PARTNUM, partnum);
    auto e2 = drv.read_status(CC1101Driver::REG_VERSION, version);
    if (e1 == ESP_OK && e2 == ESP_OK) {
        ESP_LOGI(TAG, "CC1101 slot %u: PARTNUM=0x%02x VERSION=0x%02x", slot.id(), partnum, version);
    } else {
        ESP_LOGE(TAG, "CC1101 slot %u: read failed", slot.id());
    }
}
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "CULFW32 booting (skeleton)");

    // 0) RfStats — refill-Timer 1Hz starten BEVOR irgendein RF-Code laeuft.
    // Singleton, init+state lebt in der RfStats-Klasse selbst.
    if (auto e = RfStats::instance().start(); e != ESP_OK) {
        ESP_LOGW(TAG, "rf_stats start: %s", esp_err_to_name(e));
    }

    // 1) NVS-EEPROM (vor allen anderen Komponenten — RF-Klassen brauchen
    //    spaeter EE-Lesen waehrend ihres start(); ruft auch nvs_flash_init).
    g_eeprom = std::make_unique<NvsEeprom>();
    if (auto e = g_eeprom->init(); e != ESP_OK) {
        ESP_LOGE(TAG, "eeprom init failed: %s", esp_err_to_name(e));
    }

    // 1a) SystemMonitor — Reset-Reason persistieren, Boot/Crash-Counter
    //     hochzaehlen, periodischer Heap-Lowwater-Alarm. Eigener NVS-Namespace
    //     (cul32_sys), unabhaengig vom a-culfw EE-Blob. Muss NACH NvsEeprom
    //     kommen weil nvs_flash_init dort sitzt.
    if (auto e = SystemMonitor::instance().init(); e != ESP_OK) {
        ESP_LOGW(TAG, "system_monitor init: %s", esp_err_to_name(e));
    }

    // Magic-Cookie pruefen wie a-culfw `eeprom_init()` (fncollection.c:235).
    // Bei Mismatch (Erst-Boot, NVS gewipt, Format-Bruch) silent factory_reset.
    // Reihenfolge: Magic-Check VOR EE_LED-Read, damit der LED-Mode nach Reset
    // direkt aus den frischen Defaults geladen wird (a-culfw EE_LED=2 = Blink).
    if (g_eeprom->read_byte(ee::MAGIC_OFFSET)     != ee::MAGIC_V1 ||
        g_eeprom->read_byte(ee::MAGIC_OFFSET + 1) != ee::MAGIC_V2) {
        ESP_LOGW(TAG, "EE magic mismatch -> factory_reset (silent)");
        factory_reset(*g_eeprom);
    }
    // Netzblock 98..123 (a-culfw CUNO) bei Bestandsgeraeten nachtragen — mit
    // oder ohne W5500-Platine, damit `Ri*` ueberall dasselbe zeigt.
    ethernet_reset_if_blank(*g_eeprom);

    // 1b) LED — fruh init, damit Pattern-Engine waehrend Boot bereits aktiv sein kann.
    g_led = std::make_unique<LedDriver>();
    if (auto e = g_led->init(CONFIG_CULFW32_LED_PIN, CONFIG_CULFW32_LED_ACTIVE_LOW);
        e != ESP_OK) {
        ESP_LOGE(TAG, "led init failed: %s", esp_err_to_name(e));
    } else {
        // Persistierten LED-Mode aus EE_LED restaurieren (a-culfw-Verhalten).
        // Uninitialisierte EE-Bytes liefern 0xFF — dann Mode 0 (off) annehmen.
        uint8_t persisted = g_eeprom->read_byte(ee::LED);
        if (persisted == ee::DEFAULT) persisted = 0;
        g_led->set_mode(persisted);
    }

    // 2) Manager + Slot(s).
    g_mgr   = std::make_unique<RfManager>();
    g_slot0 = std::make_shared<CC1101Slot>(0, kSlot0);
    bool slot0_ok = false;
    if (auto e = g_slot0->init(); e != ESP_OK) {
        // Slot-Init fehlgeschlagen → Slot bleibt unbenutzbar. NICHT dem Manager
        // hinzufuegen, sonst laufen downstream cc_write_* (PA-Persist, Mode-
        // Switch) auf einem halb-toten Slot mit ungelatchtem Fehlerzustand.
        ESP_LOGE(TAG, "CC1101 slot 0 init failed: %s -> slot disabled",
                 esp_err_to_name(e));
        g_slot0.reset();
    } else {
        slot0_ok = true;
        smoke_test_cc1101(*g_slot0);
    }
    if (slot0_ok) {
        g_mgr->add_slot(g_slot0);

        // Board-Frequenzoffset (FSCTRL0) anlegen, BEVOR eine RF-Klasse ihre
        // Registertabelle laedt: der Driver legt den Wert ueber das FSCTRL0-Byte
        // jeder CFG. Quelle: EEPROM-Override > eFuse-Werkskalibrierung > 0.
        // Ohne den Wert bleibt FSK auf einem Board mit Quarz-Ziehfehler still
        // (CUL32-Serie: ~44 kHz, gemessen 2026-08-31) — OOK merkt davon nichts.
        freqcal::Source fsrc = freqcal::Source::None;
        int8_t foff = freqcal::effective(g_eeprom.get(), fsrc);
        g_slot0->driver().set_freq_offset(foff);
        g_slot0->driver().apply_freq_offset();
        ESP_LOGI(TAG, "cc0 freq offset %d LSB (%.1f kHz), source=%s",
                 static_cast<int>(foff), foff * freqcal::kHzPerLsb,
                 freqcal::source_name(fsrc));
    }

#ifdef CONFIG_CULFW32_BUILD_NET_W5500
    // 2a) W5500-Erweiterung an J1 suchen. Den SPI-Bus hat eben der CC1101-
    //     Treiber initialisiert; der W5500 haengt sich als zweites Device an.
    //     Muss VOR dem UART0-Transport laufen: GPIO17 ist entweder U0RXD oder
    //     W5500-SCSn — die Hardware laesst nur eins von beiden zu.
    g_eth = std::make_unique<EthManager>();
    g_eth->probe();
#endif

    // 2b) RF-Klassen registrieren — alle per Kconfig-Toggle einzeln aktivierbar.
#ifdef CONFIG_CULFW32_BUILD_RF_MORITZ
    auto moritz = std::make_shared<MoritzRF>();
    g_mgr->register_protocol(moritz);

    // CULFW32-Extension: persistierte Moritz-AA/WT-Adressen (Za/Zw) restaurieren.
    // EE-uninitialisiert (3x 0xFF) → als 0/0/0 behandeln (= Feature aus).
    {
        uint8_t aa[3] = {0, 0, 0};
        uint8_t wt[3] = {0, 0, 0};
        g_eeprom->read_block(ee::MORITZ_AA_ADDR, {aa, 3});
        g_eeprom->read_block(ee::MORITZ_WT_ADDR, {wt, 3});
        if (aa[0] == 0xFF && aa[1] == 0xFF && aa[2] == 0xFF) std::memset(aa, 0, 3);
        if (wt[0] == 0xFF && wt[1] == 0xFF && wt[2] == 0xFF) std::memset(wt, 0, 3);
        moritz->set_auto_ack_addr({aa, 3});
        moritz->set_wall_thermostat({wt, 3});
    }
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_ASKSIN
    g_mgr->register_protocol(std::make_shared<AskSinRF>());
#endif
    // PA-Tabelle + FREND0 aus dem EEPROM fuer die OOK-Klassen — a-culfw laedt
    // EE_CC1100_PA bei jedem ccInitChip(). Ohne das galt `x<idx>` nur bis zum
    // naechsten Mode-Start, danach sendete SlowRF fest mit +10 dBm.
    auto pa_from_eeprom = [](std::span<uint8_t, 8> pa, uint8_t& frend0) -> bool {
        if (!g_eeprom) return false;
        g_eeprom->read_block(ee::CC1100_PA, pa);
        bool valid = false;
        for (auto b : pa) if (b != 0xFF) { valid = true; break; }
        if (!valid) return false;
        const uint8_t f = g_eeprom->read_byte(ee::CC1100_CFG + 0x22);
        if (f == 0xFF) return false;
        frend0 = f;
        return true;
    };
#ifdef CONFIG_CULFW32_BUILD_RF_SLOWRF
    {
        auto slowrf = std::make_shared<SlowRfRF>();
        slowrf->set_pa_provider(pa_from_eeprom);
        g_mgr->register_protocol(slowrf);
    }
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_NATIVE
    g_mgr->register_protocol(std::make_shared<NativeRF>());
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_FASTRF
    g_mgr->register_protocol(std::make_shared<FastRfRF>());
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_DUDETTE
    g_mgr->register_protocol(std::make_shared<DudetteRF>());
#endif
#ifdef CONFIG_CULFW32_BUILD_RF_WMBUS
    g_mgr->register_protocol(std::make_shared<WmbusRF>());
    // rf_hmip — HmIP-PHY, reiner Sniffer: Empfang und roher Hex-Dump.
    // Kein TX, kein MAC-Layer, keine Krypto — dafuer ist q-culfw zustaendig.
    g_mgr->register_protocol(std::make_shared<HmipRF>());
#endif
#ifdef CONFIG_CULFW32_BUILD_FRONTEND_SIGNALDUINO
    // PulseCapture-Klasse fuer das SIGNALduino-Frontend. Eigene RfProtocol —
    // exklusiv mit SlowRfRF auf demselben Slot (gleiche OOK-Konfig, gleicher
    // RMT-Pin), Mode-Switch ueber den RfManager.
    {
        auto sd = std::make_shared<SignalduinoRF>();
        sd->set_pa_provider(pa_from_eeprom);
        g_mgr->register_protocol(sd);
    }
#endif

    // Persistierte PA-Tabelle (EE_CC1100_PA, gesetzt von factory_reset oder
    // letztem `x<idx>`-Aufruf) ins CC1101 schreiben — sonst hat die Hardware
    // nach dem Reset PATABLE = SRES-Defaults und der User-Wunsch ginge bei
    // jedem Reboot verloren. FREND0 sitzt schon in den RF-Klassen-Mode-Tabellen,
    // aber bei `x`-Aufrufen mit Ramping-Profilen muss er auch persistieren.
    {
        uint8_t pa[ee::CC1100_PA_SIZE];
        g_eeprom->read_block(ee::CC1100_PA, {pa, sizeof(pa)});
        const uint8_t frend0 = g_eeprom->read_byte(ee::CC1100_CFG + 0x22);
        // Nur schreiben wenn EE-Inhalt sinnvoll aussieht (factory_reset hat
        // immer eine valide Tabelle hinterlegt; uninit. EE = alles 0xFF).
        bool valid = false;
        for (auto b : pa) if (b != 0xFF) { valid = true; break; }
        if (valid) {
            constexpr uint8_t CC_PATABLE = 0x3E;
            for (uint8_t s = 0; s < g_mgr->slot_count(); ++s) {
                if (auto e = g_mgr->cc_write_burst(s, CC_PATABLE, {pa, sizeof(pa)});
                    e != ESP_OK) {
                    ESP_LOGW(TAG, "PA persist: slot %u PATABLE burst failed: %s",
                             s, esp_err_to_name(e));
                }
                if (frend0 != 0xFF) {
                    if (auto e = g_mgr->cc_write_reg(s, 0x22, frend0); e != ESP_OK) {
                        ESP_LOGW(TAG, "PA persist: slot %u FREND0 write failed: %s",
                                 s, esp_err_to_name(e));
                    }
                }
            }
        }
    }

    // 2c) WiFi-Manager. Liest Config aus NVS-Namespace `culfw32_wifi`. Treiber
    //     wird erst beim ersten Bedarf (start() oder scan()) initialisiert —
    //     spart Strom und Boot-Zeit bei mode=0.
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
    g_wifi = std::make_unique<WifiManager>();
    g_wifi->init();
    if (g_wifi->config().mode != 0 && !g_wifi->config().ssid.empty()) {
        if (auto e = g_wifi->start(); e != ESP_OK) {
            ESP_LOGW(TAG, "wifi start failed: %s", esp_err_to_name(e));
        }
    }
#endif

    // 3) Frontend + Transport.
    // TcpServer-Instanz schon hier anlegen (noch nicht gestartet), damit das
    // Frontend ueber FrontendServices darauf zeigen kann (EWP-Port-Restart).
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_TCP
    // lwIP muss stehen, BEVOR der erste Socket entsteht. esp_netif_init() ruft
    // tcpip_init(); fehlt das, trifft socket()/bind() eine ungueltige Mailbox
    // und lwIP assertet hart (tcpip.c:454 "Invalid mbox") -> Panic -> Bootloop,
    // aus dem nur `EWm0`-Spam ins Boot-Fenster oder ein NVS-Erase herausfuehrt.
    // Bisher kam der Aufruf ausschliesslich aus WifiManager::start(), und das
    // wird oben bei mode != 0 UND leerer SSID uebersprungen — eine Lage, die
    // jeder Anwender herstellt, der die SSID leert und den Mode anlaesst.
    // Gemessen 01.09.2026 auf CUL32-Serienhardware, Build 0.1.814.
    // esp_netif_init() ist idempotent; der WifiManager ruft es weiterhin selbst.
    if (auto e = esp_netif_init(); e != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s — TCP-Frontends bleiben aus",
                 esp_err_to_name(e));
    }
    g_tcp = std::make_unique<TcpServer>();
    // TcpRxPool: 8 langlebige Worker statt pro-Connect-xTaskCreate. Verhindert
    // tcp-rx-Zombie-Akkumulation unter Stress (alle 5 Frontends teilen sich
    // den Pool, da rx-Pfad identisch ist). 8 x 6 KB = 48 KB fester Footprint
    // vs ~66 KB Peak und unbegrenztes Wachstum vorher.
    if (auto e = TcpRxPool::instance().init(8, 6144); e != ESP_OK) {
        ESP_LOGE(TAG, "tcp rx pool init failed: %s", esp_err_to_name(e));
    }
#endif

#ifdef CONFIG_CULFW32_BUILD_NET_W5500
    // Ethernet starten (DHCP bzw. statisch aus dem a-culfw-Netzblock 98..123).
    // Hostname = derselbe Geraetename wie im WiFi (DHCP-Option 12, mDNS).
    if (g_eth && g_eth->present()) {
        std::string hn = "culfw32";
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
        if (g_wifi) {
            const auto& wc = g_wifi->config();
            hn = wc.hostname.empty() ? std::string{g_wifi->default_hostname()} : wc.hostname;
        }
#endif
        if (auto e = g_eth->start(g_eeprom.get(), hn); e != ESP_OK) {
            ESP_LOGW(TAG, "ethernet start failed: %s", esp_err_to_name(e));
        }
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
        // mDNS kommt sonst nur mit WiFi::start(); bei reinem Ethernet hier.
        if (g_wifi) g_wifi->ensure_mdns();
#endif
    }
#endif

    // Netz vorhanden = WiFi eingeschaltet ODER W5500 gesteckt. Die TCP-Dienste
    // binden auf 0.0.0.0 und bedienen damit beide Interfaces.
    [[maybe_unused]] bool net_up = false;
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
    net_up = g_wifi && g_wifi->config().mode != 0;
#endif
#ifdef CONFIG_CULFW32_BUILD_NET_W5500
    net_up = net_up || (g_eth && g_eth->present());
#endif

#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_TCP
    // Online-Update: der Task laeuft immer (auch wenn das Netz erst spaeter per
    // EW* konfiguriert wird) und prueft vor jeder planmaessigen Pruefung selbst.
    if (auto e = OtaUpdate::instance().start(&net_has_ip, &ota_auto_enabled); e != ESP_OK) {
        ESP_LOGW(TAG, "ota start failed: %s", esp_err_to_name(e));
    }
    // Waehrend das Online-Update prueft oder laedt, vergeben die Sendepuffer
    // der Netz-Clients nichts; wer puffern muesste, wird getrennt. Mit vier
    // gestauten Clients scheiterte der TLS-Download sonst am Heap (Review
    // 0903 N4, gemessen 14.10.2026). Nach einer Installation startet das
    // Geraet neu; nach einer Pruefung (300 s nach dem Start, dann taeglich)
    // oder einem Fehlschlag nicht — betroffene Clients verbinden sich neu.
    // Vor den Listenern setzen (TxQueue::set_shed_fn).
    TxQueue::set_shed_fn([] { return OtaUpdate::instance().busy(); });
#endif

    // ConfigRegistry — typed key/value introspection fuer ASCII-/JSON-/MQTT-
    // Frontends + kuenftige Web-UI. Eintraege koennen jederzeit registriert
    // werden; aktive Setter-Funktoren rufen die Subsystem-Setter (Wifi/Tcp/Pa).
    g_cfg = std::make_unique<ConfigRegistry>();
    setup_config_registry(*g_cfg);

    g_fe_culfw = std::make_unique<CulfwFrontend>();
    g_fe_culfw->attach(FrontendServices{
        .rf_manager = g_mgr.get(),
        .eeprom     = g_eeprom.get(),
        .led        = g_led.get(),
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
        .wifi       = g_wifi.get(),
#else
        .wifi       = nullptr,
#endif
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_TCP
        .tcp_server = g_tcp.get(),
#else
        .tcp_server = nullptr,
#endif
        .config     = g_cfg.get(),
#ifdef CONFIG_CULFW32_BUILD_NET_W5500
        .eth        = g_eth.get(),
#endif
    });
    g_usbcdc = std::make_shared<UsbCdcTransport>();
    g_fe_culfw->bind_transport(g_usbcdc);

#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_UART
    // Zweiter Transport: UART0 ueber USB-C SBU1/SBU2 nach aussen — damit ein
    // benachbarter Mikrocontroller die culfw-Daten direkt seriell mitlesen kann.
    // Mit gesteckter W5500-Platine ist GPIO17 deren SCSn, nicht U0RXD.
    bool uart_ok = true;
#ifdef CONFIG_CULFW32_BUILD_NET_W5500
    if (g_eth && g_eth->present() &&
        (CONFIG_CULFW32_UART_PIN_RX == CONFIG_CULFW32_W5500_PIN_CS ||
         CONFIG_CULFW32_UART_PIN_TX == CONFIG_CULFW32_W5500_PIN_CS)) {
        uart_ok = false;
        ESP_LOGW(TAG, "UART0 transport off: GPIO%d is W5500 SCSn (board at J1)",
                 CONFIG_CULFW32_W5500_PIN_CS);
    }
#endif
    if (uart_ok) {
        UartTransportConfig uart_cfg = {
            .port      = static_cast<uart_port_t>(CONFIG_CULFW32_UART_PORT),
            .baud_rate = CONFIG_CULFW32_UART_BAUD,
            .pin_tx    = CONFIG_CULFW32_UART_PIN_TX,
            .pin_rx    = CONFIG_CULFW32_UART_PIN_RX,
            .rx_buf_sz = 1024,
            .tx_buf_sz = 1024,
        };
        g_fe_culfw->bind_transport(std::make_shared<UartTransport>(uart_cfg));
    }
#endif

    g_fe_culfw->start();

    // TCP-Server: bindet auf 0.0.0.0:<port>. Port aus EE_IP4_TCPLINK_PORT
    // (Adresse 121, byte-genau a-culfw, Little-Endian). Default 2323 wird im
    // factory_reset gesetzt; uninitialisierte EE-Bytes (0xFFFF) → fallback auf
    // 2323. Listet auch wenn WiFi gerade nicht assoziiert ist — sobald die
    // Verbindung steht, ist der Server erreichbar.
#if defined(CONFIG_CULFW32_BUILD_NET_WIFI) && defined(CONFIG_CULFW32_BUILD_TRANSPORT_TCP)
    if (net_up) {
        uint8_t plo = g_eeprom->read_byte(ee::IP4_TCPLINK_PORT);
        uint8_t phi = g_eeprom->read_byte(ee::IP4_TCPLINK_PORT + 1);
        uint16_t port = static_cast<uint16_t>(plo) | (static_cast<uint16_t>(phi) << 8);
        if (port == 0xFFFF || port == 0) port = ee::IP4_TCPLINK_PORT_DEFAULT;

        // Allowlist aus EE laden (4 bytes IP NBO + 1 byte CIDR-Bits). Uninit
        // (0xFF) → bits=0 = alle Clients erlaubt (default Open-LAN-Verhalten).
        uint8_t a0 = g_eeprom->read_byte(ee::TCP_ALLOW_IP);
        uint8_t a1 = g_eeprom->read_byte(ee::TCP_ALLOW_IP + 1);
        uint8_t a2 = g_eeprom->read_byte(ee::TCP_ALLOW_IP + 2);
        uint8_t a3 = g_eeprom->read_byte(ee::TCP_ALLOW_IP + 3);
        uint8_t bits = g_eeprom->read_byte(ee::TCP_ALLOW_BITS);
        if (bits == 0xFF) bits = 0;
        uint32_t allow_ip = static_cast<uint32_t>(a0) |
                            (static_cast<uint32_t>(a1) << 8) |
                            (static_cast<uint32_t>(a2) << 16) |
                            (static_cast<uint32_t>(a3) << 24);
        g_tcp->set_allow(allow_ip, bits);

        auto e = g_tcp->start(port, [](std::shared_ptr<TcpClientTransport> t) {
            // on_closed bei recv-EOF/error: detach aus Frontend → shared_ptr-RC
            // -> 0 → dtor (idempotenter stop()).
            t->set_on_closed([](TcpClientTransport* self) {
                if (g_fe_culfw) g_fe_culfw->detach_dynamic_transport(self);
            });
            if (g_fe_culfw) g_fe_culfw->attach_dynamic_transport(std::move(t));
        });
        if (e != ESP_OK) ESP_LOGW(TAG, "tcp server start failed: %s", esp_err_to_name(e));
    }
#endif // WIFI && TCP

#if defined(CONFIG_CULFW32_BUILD_FRONTEND_JSON) && defined(CONFIG_CULFW32_BUILD_NET_WIFI)
    // JSON-Frontend auf separatem TCP-Port (Default 2324). Eigener TcpServer-
    // Listener — bestehender ASCII-TCP-Pfad bleibt unberuehrt.
    if (net_up) {
        // Port aus EE_TCP_JSON_PORT (LE wie IP4_TCPLINK_PORT). Uninit → Default.
        uint8_t plo = g_eeprom->read_byte(ee::TCP_JSON_PORT);
        uint8_t phi = g_eeprom->read_byte(ee::TCP_JSON_PORT + 1);
        uint16_t json_port = static_cast<uint16_t>(plo) | (static_cast<uint16_t>(phi) << 8);
        if (json_port == 0xFFFF || json_port == 0) {
            json_port = CONFIG_CULFW32_TCP_JSON_PORT_DEFAULT;
        }

        g_fe_json = std::make_unique<JsonFrontend>();
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_TCP
        // Waehrend das Online-Update prueft oder laedt, pusht das JSON-
        // Frontend nur ota.* (Heap fuer TLS; andere Tabs sehen den Fortschritt).
        g_fe_json->set_push_pause([] { return OtaUpdate::instance().busy(); }, "ota.");
#endif
        g_fe_json->attach(FrontendServices{
            .rf_manager = g_mgr.get(),
            .eeprom     = g_eeprom.get(),
            .led        = g_led.get(),
            .wifi       = g_wifi.get(),
            .tcp_server = g_tcp.get(),
            .config     = g_cfg.get(),
        });
        g_fe_json->start();

        g_tcp_json = std::make_unique<TcpServer>();
        auto je = g_tcp_json->start(json_port,
            [](std::shared_ptr<TcpClientTransport> t) {
                t->set_on_closed([](TcpClientTransport* self) {
                    if (g_fe_json) g_fe_json->detach_dynamic_transport(self);
                });
                if (g_fe_json) g_fe_json->attach_dynamic_transport(std::move(t));
            });
        if (je != ESP_OK) ESP_LOGW(TAG, "json tcp start failed: %s", esp_err_to_name(je));
    }
#endif // FRONTEND_JSON && NET_WIFI

#if defined(CONFIG_CULFW32_BUILD_FRONTEND_SIGNALDUINO) && defined(CONFIG_CULFW32_BUILD_NET_WIFI)
    // SIGNALduino-Frontend auf separatem TCP-Port (Default 2326). Aktiviert die
    // PulseCapture-RF-Klasse auf Slot 0 — exklusiv mit SlowRfRF (gleiche
    // Hardware-Resource). FHEM-Define: `SIGNALduino <ip>:2326`.
    if (net_up) {
        uint8_t plo = g_eeprom->read_byte(ee::TCP_SD_PORT);
        uint8_t phi = g_eeprom->read_byte(ee::TCP_SD_PORT + 1);
        uint16_t sd_port = static_cast<uint16_t>(plo) | (static_cast<uint16_t>(phi) << 8);
        if (sd_port == 0xFFFF || sd_port == 0) {
            sd_port = CONFIG_CULFW32_TCP_SD_PORT_DEFAULT;
        }

        g_fe_sd = std::make_unique<SignalduinoFrontend>();
        g_fe_sd->attach(FrontendServices{
            .rf_manager = g_mgr.get(),
            .eeprom     = g_eeprom.get(),
            .led        = g_led.get(),
            .wifi       = g_wifi.get(),
            .tcp_server = g_tcp.get(),
            .config     = g_cfg.get(),
        });
        g_fe_sd->start();

        g_tcp_sd = std::make_unique<TcpServer>();
        auto se = g_tcp_sd->start(sd_port,
            [](std::shared_ptr<TcpClientTransport> t) {
                t->set_on_closed([](TcpClientTransport* self) {
                    if (g_fe_sd) g_fe_sd->detach_dynamic_transport(self);
                });
                if (g_fe_sd) g_fe_sd->attach_dynamic_transport(std::move(t));
            });
        if (se != ESP_OK) ESP_LOGW(TAG, "sd tcp start failed: %s", esp_err_to_name(se));
        else ESP_LOGI(TAG, "signalduino listening on :%u", sd_port);
    }
#endif // FRONTEND_SIGNALDUINO && NET_WIFI

#if defined(CONFIG_CULFW32_BUILD_FRONTEND_HMUARTLGW) && defined(CONFIG_CULFW32_BUILD_NET_WIFI)
    // HMUARTLGW-Frontend auf separatem TCP-Port (Default 2325). Sprich
    // 0xfd-binary mit FHEM-Define `HMUARTLGW uart://<ip>:2325`.
    if (net_up) {
        uint8_t plo = g_eeprom->read_byte(ee::HMU_PORT);
        uint8_t phi = g_eeprom->read_byte(ee::HMU_PORT + 1);
        uint16_t hmu_port = static_cast<uint16_t>(plo) | (static_cast<uint16_t>(phi) << 8);
        if (hmu_port == 0xFFFF || hmu_port == 0) {
            hmu_port = CONFIG_CULFW32_HMU_PORT_DEFAULT;
        }

        g_fe_hmu = std::make_unique<HmuartlgwFrontend>();
        g_fe_hmu->attach(FrontendServices{
            .rf_manager = g_mgr.get(),
            .eeprom     = g_eeprom.get(),
            .led        = g_led.get(),
            .wifi       = g_wifi.get(),
            .tcp_server = g_tcp.get(),
            .config     = g_cfg.get(),
        });
        g_fe_hmu->start();

        // Zwei TCP-Listener parallel — kein Mode-Umschalten mehr noetig:
        //   :hmu_port      → Legacy (HM-MOD-RPI-PCB) fuer FHEM/Homegear
        //   :hmu_port + 2  → Dual   (RPI-RF-MOD)     fuer debmatic/RaspberryMatic
        // Default-Ports: 2325 (Legacy) / 2327 (Dual). Beide Listener routen in
        // denselben HmuartlgwFrontend, jeder Slot wird beim attach mit seinem
        // Mode gestempelt → dispatch_/Hello-Push/RX-Format wird per-Slot
        // entschieden.
        using ModuleMode = HmuartlgwFrontend::ModuleMode;
        g_tcp_hmu = std::make_unique<TcpServer>();
        auto he_legacy = g_tcp_hmu->start(hmu_port,
            [](std::shared_ptr<TcpClientTransport> t) {
                t->set_on_closed([](TcpClientTransport* self) {
                    if (g_fe_hmu) g_fe_hmu->detach_dynamic_transport(self);
                });
                if (g_fe_hmu) g_fe_hmu->attach_dynamic_transport(
                    std::move(t), ModuleMode::Legacy);
            });
        if (he_legacy != ESP_OK) ESP_LOGW(TAG, "hmu legacy tcp start failed: %s",
                                          esp_err_to_name(he_legacy));
        else ESP_LOGI(TAG, "hmuartlgw legacy (HM-MOD-RPI-PCB / FHEM) :%u", hmu_port);

        const uint16_t hmu_port_dual = static_cast<uint16_t>(hmu_port + 2);
        static std::unique_ptr<TcpServer> g_tcp_hmu_dual;
        g_tcp_hmu_dual = std::make_unique<TcpServer>();
        auto he_dual = g_tcp_hmu_dual->start(hmu_port_dual,
            [](std::shared_ptr<TcpClientTransport> t) {
                t->set_on_closed([](TcpClientTransport* self) {
                    if (g_fe_hmu) g_fe_hmu->detach_dynamic_transport(self);
                });
                if (g_fe_hmu) g_fe_hmu->attach_dynamic_transport(
                    std::move(t), ModuleMode::Dual);
            });
        if (he_dual != ESP_OK) ESP_LOGW(TAG, "hmu dual tcp start failed: %s",
                                        esp_err_to_name(he_dual));
        else ESP_LOGI(TAG, "hmuartlgw dual (RPI-RF-MOD / debmatic) :%u", hmu_port_dual);

#if defined(CONFIG_CULFW32_BUILD_TRANSPORT_HBRFETH_UDP)
        // HB-RF-ETH-UDP-Tunnel ist debmatic-spezifisch (multimacd verbindet sich
        // ueber den Tunnel). Slots werden mit Dual gestempelt sobald der erste
        // Frame ankommt — dafuer setzt set_frontend einen Default-Mode-Hint.
        g_hbrfeth = std::make_unique<HbrfethListener>();
        g_hbrfeth->set_frontend(g_fe_hmu.get());
        // mDNS-TXT tag/emulates spiegeln den aktuellen hmu.module-Modus
        {
            const uint8_t m = g_eeprom ? g_eeprom->read_byte(ee::HMU_MODULE_MODE) : 0;
            g_hbrfeth->set_module_identity(m == ee::HMU_MODULE_DUAL);
        }
        auto rfe = g_hbrfeth->start(3008);
        if (rfe != ESP_OK) ESP_LOGW(TAG, "hbrfeth udp start failed: %s", esp_err_to_name(rfe));
        else ESP_LOGI(TAG, "HB-RF-ETH UDP :3008 (debmatic OCCU-Tunnel, Dual-Mode)");
#endif
    }
#endif // FRONTEND_HMUARTLGW && NET_WIFI

#if defined(CONFIG_CULFW32_BUILD_FRONTEND_WEBUI) && defined(CONFIG_CULFW32_BUILD_NET_WIFI)
    // Web-UI: HTTP-Server auf Port 80 mit eingebetteten HTML/CSS/JS-Assets
    // und zwei WebSocket-Endpoints. Browser holt /, /app.css, /app.js und
    // upgraded auf /ws/json (Config-Tab) bzw. /ws/tty (Terminal-Tab gegen
    // ASCII-Frontend).
    if (net_up) {
        g_web = std::make_unique<WebServer>();
        g_web->set_json_hook([](std::shared_ptr<WebSocketTransport> t) {
            t->set_on_closed([](WebSocketTransport* self) {
                if (g_fe_json) g_fe_json->detach_dynamic_transport(self);
            });
            if (g_fe_json) g_fe_json->attach_dynamic_transport(std::move(t));
        });
        g_web->set_tty_hook([](std::shared_ptr<WebSocketTransport> t) {
            t->set_on_closed([](WebSocketTransport* self) {
                if (g_fe_culfw) g_fe_culfw->detach_dynamic_transport(self);
            });
            if (g_fe_culfw) g_fe_culfw->attach_dynamic_transport(std::move(t));
        });
        if (auto e = g_web->start(CONFIG_CULFW32_HTTP_PORT_DEFAULT); e != ESP_OK) {
            ESP_LOGW(TAG, "web server start failed: %s", esp_err_to_name(e));
        }
    }
#endif // FRONTEND_WEBUI && NET_WIFI

#ifdef CONFIG_CULFW32_BUILD_IMPROV_SERIAL
    // Improv-Serial-WiFi-Provisioning. Lauscht nach dem Boot passiv auf
    // USB-Serial-JTAG mit (kein Verschlucken — ASCII-Frontend bleibt aktiv).
    // Fenster nach der busware-weiten Vorgabe: 120 s ohne, 30 s mit
    // gespeicherter SSID, jeweils ab dem letzten Improv-Byte vom Host; die
    // Lib-Frist (windowMs) ist nur noch die harte Grenze von 1 h ab Boot.
    // Konsolenbytes zaehlen nicht: ein FHEM am USB hielte das Fenster sonst
    // bis zur Grenze offen. Die Sperre fuer den WLAN-Auto-Retry
    // (set_external_control) beginnt erst mit dem ersten Improv-Rahmen.
    // Erkennt das `IMPROV`-Magic im Stream, antwortet mit Device-Info /
    // Scan-Result / WIFI_SETTINGS-State; bei Erfolg schreibt onConnected die
    // Credentials in den `culfw32_wifi`-NVS-Namespace, sodass `wifi_drv`
    // beim naechsten Boot Single-Source-of-Truth ist. Liefen die Netzdienste
    // beim Boot nicht (net_up false: Modus 0, kein W5500), startet das Geraet
    // danach einmal neu, damit sie starten.
    {
        namespace ipw = improv_wifi_busware;

        // Globaler State (function-local statics gehen nicht — lokale
        // Klassen duerfen keine non-constexpr static members haben, und
        // Lambda-/SinkFn-Adressen brauchen file-scope-Sichtbarkeit).
        struct ImprovGlobals {
            StreamBufferHandle_t      rx_buf;
            ipw::EspIdfWiFiBackend    backend;
            ipw::ImprovWiFi*          inst;
            ipw::SerialFilter         filter;
            UsbCdcTransport::RxEmit   cur_emit;
            void*                     cur_emit_user;
            uint32_t                  idle_ms;      // Fenster ab dem letzten Improv-Byte
            uint32_t                  last_rx_ms;   // nur der Improv-Task schreibt (nach dem Start)
            bool                      session;      // erster Improv-Rahmen gesehen (nur der Improv-Task)
            bool                      restart_on_setup; // Netzdienste beim Boot nicht gestartet
        };
        static ImprovGlobals s_g{
            .rx_buf        = xStreamBufferCreate(/*size*/ 256, /*trigger*/ 1),
            .backend       = {},
            .inst          = nullptr,
            .filter        = {},
            .cur_emit      = nullptr,
            .cur_emit_user = nullptr,
            .idle_ms       = CONFIG_CULFW32_IMPROV_WINDOW_MS,
            .last_rx_ms    = 0,
            .session       = false,
            .restart_on_setup = false,
        };
        // Backward-compat-Aliase fuer die bestehende ImprovTask-Klasse:
        auto& s_improv_rx_buf = s_g.rx_buf;
        auto& s_improv_backend = s_g.backend;
        auto& s_improv = s_g.inst;

        struct ImprovTask {
            static void write_fn(const uint8_t* d, size_t n, void* /*user*/) {
                size_t sent = 0;
                while (sent < n) {
                    int rc = usb_serial_jtag_write_bytes(d + sent, n - sent, pdMS_TO_TICKS(100));
                    if (rc <= 0) break;
                    sent += static_cast<size_t>(rc);
                }
            }
            // Adapter: Transport's RxFilter-API hat die generische
            // (data,len,user,emit,emit_u)-Signatur (passt fuer jedes
            // Magic-Pattern), die Lib's SerialFilter ist enger. Shim
            // setzt die per-Aufruf emit-Pointer in s_g.cur_emit*,
            // damit der Lib's SinkFn sie sieht.
            static void to_console_sink(const uint8_t* d, size_t n, void* g_v) {
                auto* g = static_cast<ImprovGlobals*>(g_v);
                if (g->cur_emit) g->cur_emit(d, n, g->cur_emit_user);
            }
            static void to_improv_sink(const uint8_t* d, size_t n, void* g_v) {
                auto* g = static_cast<ImprovGlobals*>(g_v);
                if (g->rx_buf) xStreamBufferSend(g->rx_buf, d, n, 0);
            }

            static void rx_filter(const uint8_t* d, size_t n, void* g_v,
                                  UsbCdcTransport::RxEmit emit, void* emit_u) {
                auto* g = static_cast<ImprovGlobals*>(g_v);
                g->cur_emit      = emit;
                g->cur_emit_user = emit_u;
                g->filter.feed(d, n);
                g->cur_emit      = nullptr;
                g->cur_emit_user = nullptr;
            }
            static void on_error(ipw::Error e, void* /*user*/) {
                ESP_LOGW("improv", "error 0x%02x", static_cast<unsigned>(e));
            }
            static void on_connected(const char* ssid, const char* psk, void* /*user*/) {
                ESP_LOGW("improv", "provisioned to '%s' -> persisting in culfw32_wifi NVS",
                         ssid ? ssid : "?");
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
                if (g_wifi) {
                    // Atomar persistieren ohne den von der Lib just-connecteten
                    // Driver durch reconfigure_sta_() (esp_wifi_stop+start) zu
                    // zerlegen. Sonst geht die DHCP-Lease verloren, die die
                    // Lib der Browser-Seite gerade als URL gemeldet hat.
                    // Den Modus behalten, wenn einer gesetzt ist: mit fest 1
                    // verlor ein Stick in Modus 3 still seinen AP (spaetestens
                    // nach dem naechsten Neustart), in Modus 2 den AP-Fallback.
                    // Nur "aus" (0) wird zu 1 (Station).
                    const uint8_t mode = g_wifi->config().mode;
                    const esp_err_t e = g_wifi->set_persisted_creds(ssid ? ssid : "",
                                                                    psk  ? psk  : "",
                                                                    mode != 0 ? mode : 1);
                    // Ersteinrichtung ohne Netzdienste (s_g.restart_on_setup): Web 80
                    // und 2323..2327 starten nur beim Boot und nur mit Netz; die URL,
                    // die der Browser gerade bekommen hat, fuehrte sonst bis zum
                    // naechsten Neustart ins Leere (gemessen 19.10.2026). Also einmal
                    // neu starten -- nur mit gespeicherten Daten, sonst kaeme der
                    // Stick in Modus 0 ohne WLAN zurueck. Status und URL hat die Lib
                    // vor diesem Aufruf geschrieben, die Pause laesst sie ueber USB
                    // hinaus. ESP Web Tools sendet nach dem Erfolg nichts mehr, und
                    // esp_restart() meldet den USB-Port nicht neu an: der Browser
                    // bleibt verbunden.
                    if (s_g.restart_on_setup) {
                        if (e != ESP_OK) {
                            ESP_LOGW("improv", "settings not stored (%s), no restart",
                                     esp_err_to_name(e));
                        } else {
                            ESP_LOGW("improv", "first setup: restarting to start the network services");
                            vTaskDelay(pdMS_TO_TICKS(500));
                            esp_restart();
                        }
                    }
                }
#endif
            }
            static void task(void* /*arg*/) {
                while (true) {
                    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000);
                    s_improv->tick(now);
                    const bool idle = now - s_g.last_rx_ms >= s_g.idle_ms;
                    if (!s_improv->isArmed() || idle) {
                        if (g_usbcdc) g_usbcdc->set_rx_filter(nullptr, nullptr);
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
                        // Auto-retry-Guard wieder loesen: nach Window-Close
                        // soll der WifiManager bei Disconnects ganz normal
                        // mit den (neu) persistierten Credentials reconnecten.
                        if (g_wifi) g_wifi->set_external_control(false);
#endif
                        ESP_LOGW("improv", "window closed (%s), going silent",
                                 idle ? "idle" : "1 h limit");
                        break;
                    }
                    uint8_t buf[64];
                    size_t got = xStreamBufferReceive(s_improv_rx_buf, buf, sizeof(buf),
                                                      pdMS_TO_TICKS(100));
                    if (got > 0) {
                        // Nur Improv-Rahmen landen hier (SerialFilter), also
                        // zaehlt nur Improv-Verkehr als Aktivitaet.
                        s_g.last_rx_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
                        // Auto-Retry-Sperre ab dem ersten Improv-Rahmen, VOR
                        // feedBytes(): erst ab dann kann die Lib den Treiber
                        // anfassen (Suche, tryConnect).
                        if (!s_g.session && g_wifi) g_wifi->set_external_control(true);
#endif
                        s_g.session = true;
                        s_improv->feedBytes(buf, got);
                    }
                }
                vTaskDelete(nullptr);
            }
        };

        ipw::Config cfg;
        cfg.backend                 = &s_improv_backend;
        cfg.write                   = &ImprovTask::write_fn;
        cfg.userCtx                 = nullptr;
        cfg.windowMs                = 60u * 60u * 1000u;   // harte Grenze, s. o.
        cfg.device.chipFamily       = ipw::ChipFamily::Esp32C6;
        cfg.device.firmwareName     = "CULFW32";
        cfg.device.firmwareVersion  = CULFW_VERSION;
        cfg.device.deviceName       = "CULFW32";
        cfg.device.deviceUrl        = nullptr; // Lib fuellt http://<ip>/ auf
        cfg.onError                 = &ImprovTask::on_error;
        cfg.onConnected             = &ImprovTask::on_connected;

        static ipw::ImprovWiFi s_improv_inst{cfg};
        s_improv = &s_improv_inst;

        // Sinks einmal an die Lib-Filter-Instanz binden, dann Filter im Transport scharfstellen.
        s_g.filter.setSinks(&ImprovTask::to_console_sink, &s_g,
                            &ImprovTask::to_improv_sink,  &s_g);
        if (g_usbcdc) g_usbcdc->set_rx_filter(&ImprovTask::rx_filter, &s_g);
        // Die Auto-Retry-Sperre (set_external_control) setzt der Improv-Task
        // erst mit dem ersten Improv-Rahmen, nicht hier: bis 0.1.908 galt sie
        // ab dem Scharfschalten, und ein gescheiterter erster Connect nach
        // dem Start (gemessen in 4 von 825 USB-Resets) blieb bis zum
        // Fensterende ohne neuen Versuch -- WLAN erst nach ~30 s.
        bool has_creds = false;
#ifdef CONFIG_CULFW32_BUILD_NET_WIFI
        has_creds = g_wifi && !g_wifi->config().ssid.empty();
#endif
        s_g.idle_ms    = has_creds ? CONFIG_CULFW32_IMPROV_WINDOW_CREDS_MS
                                   : CONFIG_CULFW32_IMPROV_WINDOW_MS;
        s_g.last_rx_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_TCP
        // Jeder Dienst unter net_up haengt an TRANSPORT_TCP (Kconfig: JSON,
        // SIGNALduino, HMUARTLGW per depends on, die Web-UI ueber JSON).
        s_g.restart_on_setup = !net_up;
#endif
        xTaskCreate(&ImprovTask::task, "improv", 4096, nullptr, 4, nullptr);
        ESP_LOGW(TAG, "improv-serial armed: %u s after the last Improv byte (%s), at most 1 h",
                 static_cast<unsigned>(s_g.idle_ms / 1000),
                 has_creds ? "credentials stored" : "no credentials");
    }
#endif // IMPROV_SERIAL

    // Logging-Trennung: ab hier teilen sich Frontend und ESP_LOG denselben USB-CDC-Stream.
    // Boot-Logs sind oben gut sichtbar (idf.py monitor / fast nach Reset). Im Live-Betrieb
    // wuerden weitere INFO-Logs die culfw-Befehl-Antworten verschmutzen, also Console-Stille
    // ausser bei Warnings/Errors. Logs lassen sich spaeter ueber einen eigenen TCP-Log-Sink
    // (z.B. Port 4444) wieder einschalten.
    esp_log_level_set("*", ESP_LOG_WARN);

#ifdef CONFIG_CULFW32_BUILD_TRANSPORT_TCP
    // Rollback-Gate: ein per OTA gekommenes Image erst nach stabilem Lauf
    // freigeben (Bootloader: CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE).
    OtaUpdate::instance().start_validation(&ota_net_health);
#endif
}
