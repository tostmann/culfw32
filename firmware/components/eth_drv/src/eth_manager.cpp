// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "eth_manager.hpp"
#include "nvs_eeprom.hpp"
#include "culfw_ee_addr.hpp"
#include "sdkconfig.h"
#include "esp_log.h"
#include <cstring>

#ifdef CONFIG_CULFW32_BUILD_NET_W5500
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_efuse.h"
#include "esp_eth.h"
#include "esp_eth_mac_spi.h"
#include "esp_eth_phy.h"
#include "esp_eth_netif_glue.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/etharp.h"
#include "lwip/netif.h"
#include <algorithm>
#endif

namespace culfw32 {

namespace {
constexpr const char* TAG = "eth";

#ifdef CONFIG_CULFW32_BUILD_NET_W5500

// Unicast und nicht 00:00:00:00:00:00. FF:..:FF (unbeschriebenes EE) faellt
// schon am Multicast-Bit heraus.
bool mac_usable(const uint8_t* m) {
    bool all_00 = true;
    for (int i = 0; i < 6; ++i) all_00 = all_00 && m[i] == 0x00;
    return !all_00 && (m[0] & 0x01) == 0;
}

uint32_t ee_ip4(NvsEeprom* ee, uint16_t addr) {
    // Byte-Reihenfolge im EE wie a-culfw fromip(): a.b.c.d ab `addr`. Im
    // Speicher in dieser Reihenfolge abgelegt ist das lwIPs Netz-Byte-Order.
    uint8_t b[4];
    for (int i = 0; i < 4; ++i) b[i] = ee->read_byte(static_cast<uint16_t>(addr + i));
    uint32_t v;
    std::memcpy(&v, b, sizeof(v));
    return v;
}

// W5500 DS v1.1.0, Common Register Block. SPI-Frame: Offset-Adresse (16 bit),
// Control-Byte BSB[7:3] | RWB[2] | OM[1:0], dann die Daten.
constexpr uint16_t kRegRtr      = 0x0019;   // RTR, 2 Byte, Reset 0x07D0 (S. 38)
constexpr uint16_t kRegVersionr = 0x0039;   // VERSIONR, immer 0x04   (S. 43)
constexpr uint8_t  kVersion     = 0x04;
constexpr uint8_t  kCtlRead     = 0x00;     // BSB=0, RWB=0 (read),  OM=00 (VDM)
constexpr uint8_t  kCtlWrite    = 0x04;     // BSB=0, RWB=1 (write), OM=00 (VDM)

constexpr uint32_t    kLockTimeoutMs = 100;
// Unter den RF-RX-Tasks (Prioritaet 8): ein Schwall LAN-Frames darf den
// Funkempfang nicht verdraengen. Die SPI-Transfers laufen im Polling, belegen
// also die CPU fuer die Dauer des Frames (~0,7 ms fuer 1514 B bei 20 MHz).
constexpr UBaseType_t kRxTaskPrio    = 7;

struct SpiCtx {
    spi_device_handle_t dev   = nullptr;
    gpio_num_t          cs    = GPIO_NUM_NC;
    SemaphoreHandle_t   mtx   = nullptr;   // RX-Task, tcpip-Task und Ei teilen sich das Device
    size_t              chunk = 64;        // max. Einzeltransfer des Busses (ohne DMA 64 B)
};

// Eine W5500-Transaktion. cmd = Offset-Adresse, addr = Control-Byte — so
// verpackt sie der IDF-Treiber (esp_eth_mac_w5500.c, w5500_read/w5500_write).
// Mutex und Bus werden ueber die gesamte SCSn-low-Phase gehalten; laengere
// Frames gehen in Stuecken zu `chunk` Byte, ohne SCSn dazwischen anzuheben.
esp_err_t spi_xfer(SpiCtx* c, uint32_t cmd, uint32_t addr,
                   const uint8_t* tx, uint8_t* rx, uint32_t len) {
    if (!c || !c->dev || len == 0) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(c->mtx, pdMS_TO_TICKS(kLockTimeoutMs)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t e = spi_device_acquire_bus(c->dev, portMAX_DELAY);
    if (e != ESP_OK) {
        xSemaphoreGive(c->mtx);
        return e;
    }
    gpio_set_level(c->cs, 0);
    uint32_t off = 0;
    do {
        const uint32_t n = std::min<uint32_t>(len - off, static_cast<uint32_t>(c->chunk));
        spi_transaction_ext_t t = {};
        t.base.length    = n * 8;
        t.base.tx_buffer = tx ? tx + off : nullptr;
        t.base.rx_buffer = rx ? rx + off : nullptr;
        if (off == 0) {
            t.base.cmd  = static_cast<uint16_t>(cmd);
            t.base.addr = addr;
        } else {
            // Fortsetzung desselben Frames: ohne Adress- und Control-Phase.
            t.base.flags   = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
            t.command_bits = 0;
            t.address_bits = 0;
        }
        e = spi_device_polling_transmit(c->dev, &t.base);
        off += n;
    } while (e == ESP_OK && off < len);
    gpio_set_level(c->cs, 1);
    spi_device_release_bus(c->dev);
    xSemaphoreGive(c->mtx);
    return e;
}

// Gratuitous ARP im tcpip-Kontext (esp_netif_tcpip_exec). lwIP sendet nach
// `Ei` (esp_eth_stop/start) keinen — gemessen 08.10. per Mitschnitt. Nach
// einem MAC-Wechsel (`Wim`, Geraetetausch mit gleicher statischer IP) halten
// die Nachbarn dann ihre alte ARP-Zuordnung, der W5500-MAC-Filter verwirft
// alles an die alte Adresse: ~28 s stumm bis zum Ablauf des Peer-Caches.
esp_err_t send_garp(void* ctx) {
    auto* n = static_cast<struct netif*>(esp_netif_get_netif_impl(static_cast<esp_netif_t*>(ctx)));
    if (n && netif_is_up(n) && netif_is_link_up(n) && !ip4_addr_isany_val(*netif_ip4_addr(n))) {
        etharp_gratuitous(n);
    }
    return ESP_OK;
}

// eth_spi_custom_driver_config_t: das Device legt probe() an, es gehoert dem
// EthManager. init/deinit reichen nur den Kontext durch.
void* cb_init(const void* cfg) { return const_cast<void*>(cfg); }
esp_err_t cb_deinit(void* /*ctx*/) { return ESP_OK; }
esp_err_t cb_read(void* ctx, uint32_t cmd, uint32_t addr, void* data, uint32_t len) {
    return spi_xfer(static_cast<SpiCtx*>(ctx), cmd, addr, nullptr,
                    static_cast<uint8_t*>(data), len);
}
esp_err_t cb_write(void* ctx, uint32_t cmd, uint32_t addr, const void* data, uint32_t len) {
    return spi_xfer(static_cast<SpiCtx*>(ctx), cmd, addr,
                    static_cast<const uint8_t*>(data), nullptr, len);
}

#endif // CONFIG_CULFW32_BUILD_NET_W5500
} // namespace

const char* EthManager::state_name(State s) {
    switch (s) {
        case State::Absent:  return "absent";
        case State::Stopped: return "stopped";
        case State::NoLink:  return "nolink";
        case State::LinkUp:  return "link";
        case State::GotIp:   return "connected";
    }
    return "?";
}

const char* EthManager::mac_source_name(MacSource s) {
    switch (s) {
        case MacSource::Eeprom: return "eeprom";
        case MacSource::Efuse:  return "efuse";
        case MacSource::Chip:   return "chip";
    }
    return "?";
}

EthManager::State EthManager::state() const {
    if (!present_)  return State::Absent;
    if (!started_)  return State::Stopped;
    if (!link_up_)  return State::NoLink;
    return ip4() != 0 ? State::GotIp : State::LinkUp;
}

void EthManager::mac(uint8_t out[6]) const {
    std::lock_guard<std::mutex> lk(mac_mu_);
    std::memcpy(out, mac_, sizeof(mac_));
}

uint8_t EthManager::bump_debug() {
    debug_ = static_cast<uint8_t>((debug_ + 1) & 0x3);
    const esp_log_level_t lvl = debug_ ? ESP_LOG_INFO : ESP_LOG_WARN;
    for (const char* tag : {"eth", "w5500.mac", "w5500.phy", "esp_eth", "esp_netif_handlers"}) {
        esp_log_level_set(tag, lvl);
    }
    return debug_;
}

#ifdef CONFIG_CULFW32_BUILD_NET_W5500

EthManager::MacSource EthManager::effective_mac(NvsEeprom* ee, uint8_t out[6]) {
    uint8_t m[6];
    if (ee) {
        for (int i = 0; i < 6; ++i) m[i] = ee->read_byte(static_cast<uint16_t>(ee::MAC_ADDR + i));
        if (mac_usable(m)) {
            std::memcpy(out, m, 6);
            return MacSource::Eeprom;
        }
    }
    // busware-MAC aus dem Produktionstest (Fertigung, eFuse BLK3
    // Byte 25..30, OUI A4:50:55). Direkt aus dem Block gelesen: auf dem C6
    // liefert esp_efuse_mac_get_custom() die 8-Byte-EUI-64-Form (FF:FE in der
    // Mitte) und schriebe in einen 6-Byte-Puffer hinein.
    uint8_t blk[32] = {};
    if (esp_efuse_read_block(EFUSE_BLK_USER_DATA, blk, 0, sizeof(blk) * 8) == ESP_OK &&
        blk[0] == 'B' && blk[1] == 'W' && mac_usable(&blk[25])) {
        std::memcpy(out, &blk[25], 6);
        return MacSource::Efuse;
    }
    esp_read_mac(out, ESP_MAC_ETH);
    return MacSource::Chip;
}

bool EthManager::probe() {
    if (present_) return true;
    const auto host = static_cast<spi_host_device_t>(CONFIG_CULFW32_CC1101_SPI_HOST);
    const auto cs   = static_cast<gpio_num_t>(CONFIG_CULFW32_W5500_PIN_CS);

    // SCSn zuerst auf high legen, dann den Ausgang einschalten — kein Low-
    // Glitch, den ein gesteckter W5500 als Frame-Beginn lesen koennte.
    gpio_set_level(cs, 1);
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << cs,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io) != ESP_OK) return false;
    gpio_set_level(cs, 1);

    auto* c = new SpiCtx{};
    c->cs  = cs;
    c->mtx = xSemaphoreCreateMutex();
    size_t max_len = 0;
    if (spi_bus_get_max_transaction_len(host, &max_len) == ESP_OK && max_len > 0) {
        c->chunk = max_len;
    }

    spi_device_interface_config_t dc = {};
    dc.command_bits   = 16;   // W5500-Offset-Adresse
    dc.address_bits   = 8;    // W5500-Control-Byte
    dc.mode           = 0;
    dc.clock_speed_hz = CONFIG_CULFW32_W5500_SPI_CLOCK_HZ;
    dc.spics_io_num   = -1;   // SCSn von Hand: der Frame ueberspannt mehrere Transfers
    dc.queue_size     = 1;

    uint8_t ver = 0;
    bool ok = c->mtx && spi_bus_add_device(host, &dc, &c->dev) == ESP_OK;
    // Einige W5500 lesen direkt nach einem Reset kurz 0x00 (Kommentar im IDF-
    // Treiber, w5500_verify_id). Der POR liegt hier lange zurueck; die paar
    // Wiederholungen kosten ohne Platine 40 ms Bootzeit.
    for (int i = 0; ok && i < 5; ++i) {
        if (spi_xfer(c, kRegVersionr, kCtlRead, nullptr, &ver, 1) == ESP_OK && ver == kVersion) break;
        if (i == 4) ok = false;
        else vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (ok) {
        // VERSIONR beweist nur SCLK und MISO; erst das Zurueckschreiben eines
        // geaenderten RTR prueft MOSI (Muster wie im Selbsttest der Fertigung).
        const uint8_t pat[2] = {0x0F, 0xA0};
        uint8_t orig[2] = {0x07, 0xD0}, back[2] = {};
        spi_xfer(c, kRegRtr, kCtlRead, nullptr, orig, 2);
        spi_xfer(c, kRegRtr, kCtlWrite, pat, nullptr, 2);
        spi_xfer(c, kRegRtr, kCtlRead, nullptr, back, 2);
        spi_xfer(c, kRegRtr, kCtlWrite, orig, nullptr, 2);
        ok = back[0] == pat[0] && back[1] == pat[1];
        if (!ok) {
            ESP_LOGW(TAG, "W5500 answers VERSIONR but RTR readback is %02x%02x "
                          "(wrote 0FA0) - MOSI/SCSn wiring?", back[0], back[1]);
        }
    }
    if (!ok) {
        if (c->dev) spi_bus_remove_device(c->dev);
        if (c->mtx) vSemaphoreDelete(c->mtx);
        delete c;
        gpio_reset_pin(cs);   // GPIO17 zurueck an den UART0-Transport (U0RXD)
        ESP_LOGI(TAG, "no W5500 at J1 (VERSIONR 0x%02x)", ver);
        return false;
    }
    spi_     = c;
    present_ = true;
    ESP_LOGI(TAG, "W5500 at J1: VERSIONR 0x04, SPI rw ok, %d Hz, chunk %u B",
             CONFIG_CULFW32_W5500_SPI_CLOCK_HZ, static_cast<unsigned>(c->chunk));
    return true;
}

esp_err_t EthManager::start(NvsEeprom* ee, const std::string& hostname) {
    if (!present_) return ESP_ERR_NOT_FOUND;
    if (started_)  return ESP_OK;
    ee_ = ee;

    esp_err_t e = esp_netif_init();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
    e = esp_event_loop_create_default();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
    // INTn des W5500 haengt am GPIO-ISR-Service (prozessweit). Ist er schon
    // installiert (CC1101-GDO-IRQ), kommt INVALID_STATE plus ein E-Log aus
    // gpio.c — beides kein Fehler. Der CC1101-Treiber haelt es genauso.
    {
        const esp_log_level_t lvl = esp_log_level_get("gpio");
        esp_log_level_set("gpio", ESP_LOG_NONE);
        e = gpio_install_isr_service(0);
        esp_log_level_set("gpio", lvl);
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
    }

    const auto host = static_cast<spi_host_device_t>(CONFIG_CULFW32_CC1101_SPI_HOST);
    eth_w5500_config_t wcfg = ETH_W5500_DEFAULT_CONFIG(host, nullptr);
    wcfg.int_gpio_num              = CONFIG_CULFW32_W5500_PIN_INT;
    wcfg.custom_spi_driver.config  = spi_;
    wcfg.custom_spi_driver.init    = &cb_init;
    wcfg.custom_spi_driver.deinit  = &cb_deinit;
    wcfg.custom_spi_driver.read    = &cb_read;
    wcfg.custom_spi_driver.write   = &cb_write;

    eth_mac_config_t mcfg = ETH_MAC_DEFAULT_CONFIG();
    mcfg.rx_task_prio = kRxTaskPrio;
    esp_eth_mac_t* mac = esp_eth_mac_new_w5500(&wcfg, &mcfg);
    if (!mac) return ESP_FAIL;

    eth_phy_config_t pcfg = ETH_PHY_DEFAULT_CONFIG();
    pcfg.phy_addr       = 1;
    pcfg.reset_gpio_num = -1;   // Default waere GPIO5 = CC_RST-Reservenetz auf CUL60
    esp_eth_phy_t* phy = esp_eth_phy_new_w5500(&pcfg);
    if (!phy) {
        mac->del(mac);
        return ESP_FAIL;
    }

    esp_eth_config_t ecfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t h = nullptr;
    if (e = esp_eth_driver_install(&ecfg, &h); e != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_driver_install: %s", esp_err_to_name(e));
        phy->del(phy);
        mac->del(mac);
        return e;
    }
    eth_ = h;
    if (e = apply_mac_(); e != ESP_OK) {
        ESP_LOGE(TAG, "set MAC: %s", esp_err_to_name(e));
        return e;
    }

    esp_netif_config_t ncfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t* nif = esp_netif_new(&ncfg);
    if (!nif) return ESP_FAIL;
    netif_ = nif;
    // DHCP-Option 12 traegt den Namen ins Netz (FritzBox & Co. zeigen ihn an).
    esp_netif_set_hostname(nif, hostname.c_str());
    auto glue = esp_eth_new_netif_glue(h);
    if (!glue) return ESP_FAIL;
    glue_ = glue;
    if (e = esp_netif_attach(nif, glue); e != ESP_OK) return e;
    if (e = apply_ip_config_(); e != ESP_OK) {
        ESP_LOGW(TAG, "IP config: %s", esp_err_to_name(e));
    }

    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &EthManager::event_handler, this);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &EthManager::event_handler, this);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_LOST_IP, &EthManager::event_handler, this);

    if (e = esp_eth_start(h); e != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_start: %s", esp_err_to_name(e));
        return e;
    }
    started_ = true;
    ESP_LOGI(TAG, "started: host=%s dhcp=%u", hostname.c_str(),
             ee_ ? ee_->read_byte(ee::USE_DHCP) : 1u);
    return ESP_OK;
}

esp_err_t EthManager::reinit() {
    if (!started_) return ESP_ERR_INVALID_STATE;
    auto h = static_cast<esp_eth_handle_t>(eth_);
    esp_err_t e = esp_eth_stop(h);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
    link_up_ = false;
    if (e = apply_mac_(); e != ESP_OK) return e;
    // Der netif-Glue uebernimmt die Treiber-MAC nur beim Attach (IDF 5.5
    // esp_eth_netif_glue.c, esp_eth_post_attach), nicht bei ETHERNET_EVENT_
    // START. Ohne das hier antwortet lwIP nach `Wim`+`Ei` auf ARP weiter mit
    // der alten MAC, waehrend der W5500-MAC-Filter (SHAR) nur noch Frames an
    // die neue annimmt — das Geraet ist dann im LAN stumm (gemessen 08.10.).
    {
        uint8_t m[6];
        mac(m);
        if (e = esp_netif_set_mac(static_cast<esp_netif_t*>(netif_), m); e != ESP_OK) return e;
    }
    if (e = apply_ip_config_(); e != ESP_OK) {
        ESP_LOGW(TAG, "IP config: %s", esp_err_to_name(e));
    }
    return esp_eth_start(h);
}

uint32_t EthManager::ip4() const {
    if (!started_ || !link_up_ || !netif_) return 0;
    esp_netif_ip_info_t info = {};
    if (esp_netif_get_ip_info(static_cast<esp_netif_t*>(netif_), &info) != ESP_OK) return 0;
    return info.ip.addr;
}

esp_err_t EthManager::apply_mac_() {
    uint8_t m[6];
    const MacSource src = effective_mac(ee_, m);
    esp_err_t e = esp_eth_ioctl(static_cast<esp_eth_handle_t>(eth_), ETH_CMD_S_MAC_ADDR, m);
    if (e == ESP_OK) {
        std::lock_guard<std::mutex> lk(mac_mu_);
        std::memcpy(mac_, m, sizeof(mac_));
        mac_src_ = src;
    }
    return e;
}

esp_err_t EthManager::apply_ip_config_() {
    auto* nif = static_cast<esp_netif_t*>(netif_);
    if (!nif) return ESP_ERR_INVALID_STATE;

    // a-culfw ethernet_init(): `if(erb(EE_USE_DHCP))` — jeder Wert ausser 0
    // bedeutet DHCP, auch 0xFF (unbeschrieben).
    bool dhcp = !ee_ || ee_->read_byte(ee::USE_DHCP) != 0;
    esp_netif_ip_info_t ip = {};
    if (!dhcp) {
        ip.ip.addr      = ee_ip4(ee_, ee::IP4_ADDR);
        ip.netmask.addr = ee_ip4(ee_, ee::IP4_NETMASK);
        ip.gw.addr      = ee_ip4(ee_, ee::IP4_GATEWAY);
        // 0.0.0.0 bzw. 255.255.255.255 als eigene Adresse ist keine statische
        // Konfiguration, sondern ein leerer Block — dann lieber DHCP als stumm.
        if (ip.ip.addr == 0 || ip.ip.addr == 0xFFFFFFFFu) {
            ESP_LOGW(TAG, "static IP invalid (Wid0 without Wia?) - using DHCP");
            dhcp = true;
        }
    }

    esp_netif_dhcp_status_t st = ESP_NETIF_DHCP_INIT;
    esp_netif_dhcpc_get_status(nif, &st);
    if (dhcp) {
        if (st != ESP_NETIF_DHCP_STOPPED) return ESP_OK;   // laeuft bzw. startet mit dem Link
        esp_netif_ip_info_t zero = {};
        esp_netif_set_ip_info(nif, &zero);                  // statische Adresse eines Vorlaufs weg
        return esp_netif_dhcpc_start(nif);
    }
    if (esp_err_t e = esp_netif_dhcpc_stop(nif);
        e != ESP_OK && e != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        return e;
    }
    if (esp_err_t e = esp_netif_set_ip_info(nif, &ip); e != ESP_OK) return e;
    // a-culfw kennt keinen DNS-Eintrag (uIP brauchte keinen). Das Online-Update
    // loest install.busware.de auf — statisch konfiguriert gilt deshalb wie bei
    // den meisten Heimroutern: Gateway = DNS.
    esp_netif_dns_info_t dns = {};
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4.addr = ip.gw.addr;
    return esp_netif_set_dns_info(nif, ESP_NETIF_DNS_MAIN, &dns);
}

void EthManager::event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    static_cast<EthManager*>(arg)->on_event(base, id, data);
}

void EthManager::on_event(esp_event_base_t base, int32_t id, void* data) {
    if (base == ETH_EVENT) {
        if (!data || *static_cast<esp_eth_handle_t*>(data) != static_cast<esp_eth_handle_t>(eth_)) {
            return;
        }
        switch (id) {
            case ETHERNET_EVENT_CONNECTED:
                link_up_ = true;
                ESP_LOGI(TAG, "link up");
                // Kein GARP an dieser Stelle: esp_event ruft Base-Handler
                // (unser ESP_EVENT_ANY_ID) VOR den Id-Handlern des netif-Glue
                // auf (esp_event.c, esp_event_loop_run) — das netif ist hier
                // also noch down, send_garp() taete nichts. Die Ankuendigung
                // kommt statisch wie per DHCP ueber IP_EVENT_ETH_GOT_IP: mit
                // statischer Adresse postet esp_netif_action_connected()
                // dieses Event selbst (esp_netif_handlers.c).
                break;
            case ETHERNET_EVENT_DISCONNECTED:
                link_up_ = false;
                ESP_LOGI(TAG, "link down");
                break;
            case ETHERNET_EVENT_STOP:
                link_up_ = false;
                break;
            default:
                break;
        }
    } else if (base == IP_EVENT) {
        if (id == IP_EVENT_ETH_GOT_IP && data) {
            auto* ev = static_cast<ip_event_got_ip_t*>(data);
            if (ev->esp_netif != netif_) return;
            ESP_LOGI(TAG, "IP " IPSTR " mask " IPSTR " gw " IPSTR,
                     IP2STR(&ev->ip_info.ip), IP2STR(&ev->ip_info.netmask),
                     IP2STR(&ev->ip_info.gw));
            esp_netif_tcpip_exec(&send_garp, netif_);
        } else if (id == IP_EVENT_ETH_LOST_IP) {
            ESP_LOGI(TAG, "IP lost");
        }
    }
}

#else // !CONFIG_CULFW32_BUILD_NET_W5500

EthManager::MacSource EthManager::effective_mac(NvsEeprom*, uint8_t out[6]) {
    std::memset(out, 0, 6);
    return MacSource::Chip;
}
bool      EthManager::probe() { return false; }
esp_err_t EthManager::start(NvsEeprom*, const std::string&) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t EthManager::reinit() { return ESP_ERR_NOT_SUPPORTED; }
uint32_t  EthManager::ip4() const { return 0; }
esp_err_t EthManager::apply_mac_() { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t EthManager::apply_ip_config_() { return ESP_ERR_NOT_SUPPORTED; }
void      EthManager::event_handler(void*, esp_event_base_t, int32_t, void*) {}
void      EthManager::on_event(esp_event_base_t, int32_t, void*) {}

#endif // CONFIG_CULFW32_BUILD_NET_W5500

} // namespace culfw32
