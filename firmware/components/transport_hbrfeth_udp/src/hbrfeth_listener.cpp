// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "hbrfeth_listener.hpp"
#include "hbrfeth_virtual_transport.hpp"
#include "hmuartlgw_frontend.hpp"
#include "hmuartlgw_frame.hpp"   // fuer hmu_crc16
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "mdns.h"
#include <cstdio>
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG = "hbrfeth";

constexpr uint8_t T_CONNECT     = 0;
constexpr uint8_t T_DISCONNECT  = 1;
constexpr uint8_t T_KEEPALIVE   = 2;
constexpr uint8_t T_LED         = 3;
constexpr uint8_t T_RESET       = 4;
constexpr uint8_t T_STARTCONN   = 5;
constexpr uint8_t T_STOPCONN    = 6;
constexpr uint8_t T_FRAME       = 7;

constexpr int64_t kKeepAliveTimeoutUs = 5'000'000;  // 5 s laut HB-RF-ETH-source
constexpr int64_t kKeepAliveOutPeriodUs = 1'000'000;  // 1 s
}  // anon

esp_err_t HbrfethListener::start(uint16_t port) {
    if (started_) return ESP_OK;
    if (!mtx_) mtx_ = xSemaphoreCreateMutex();

    pcb_ = udp_new();
    if (!pcb_) {
        ESP_LOGE(TAG, "udp_new failed");
        return ESP_FAIL;
    }
    err_t err = udp_bind(pcb_, IP_ADDR_ANY, port);
    if (err != ERR_OK) {
        ESP_LOGE(TAG, "udp_bind to :%u failed (lwip err %d)", port, err);
        udp_remove(pcb_);
        pcb_ = nullptr;
        return ESP_FAIL;
    }
    udp_recv(pcb_, &HbrfethListener::udp_recv_trampoline_, this);

    // KeepAlive-Tick (1 Hz) startet auch fuer Timeout-check
    esp_timer_create_args_t a = {};
    a.callback = &HbrfethListener::keepalive_trampoline_;
    a.arg      = this;
    a.name     = "hbrfeth-ka";
    esp_timer_handle_t h = nullptr;
    if (esp_timer_create(&a, &h) == ESP_OK) {
        keepalive_timer_ = h;
        esp_timer_start_periodic(h, kKeepAliveOutPeriodUs);
    }

    // mDNS announce — hb_rf_eth-Kernel-Modul kann uns dadurch automatisch
    // entdecken. Service-Name muss exakt `_raw-uart._udp` sein.
    // TXT-Records nach Cross-Project-Schema (RFNetHM-kompatibel):
    //   wire=hb-rf-eth          Protokoll-Override (pflicht; bmcond api.c:1395+)
    //   caps=bidcos[,hmip]      explizites Capability-Bit, parse_caps_csv
    //                           consumes; haengt am Crypto-Build (ohne CCM kein
    //                           HmIP-TX moeglich → nur bidcos)
    //   model=CULFW32           ehrliche Box-Identitaet
    //   fw=v<x.y.z>             aus esp_app_desc_t (OTA-relevant)
    //   tag=<app-id>            emulierte Co-Prozessor-Persoenlichkeit, exakt
    //                           der String aus COMMON_IDENTIFY-Reply (siehe
    //                           HmuartlgwFrontend); RFNetHM-Symmetrie
    //   emulates=<hardware>     Hardware-Modell das der Co-Prozessor traegt
    // tag/emulates haengen an hmu.module → set_module_identity() patcht live.
    const esp_app_desc_t* desc = esp_app_get_description();
    // esp_app_desc_t::version ist 32 byte → mit "v"-Prefix + NUL bis 34 byte
    char fw_buf[40];
    snprintf(fw_buf, sizeof(fw_buf), "v%.32s", desc ? desc->version : "unknown");
    // CULFW32 spricht kein HmIP (nur Mithoeren per `Pr`); wer HmIP braucht,
    // nimmt q-culfw. Deshalb steht hier fest "bidcos".
    constexpr const char* kCaps = "bidcos";
    const char* tag_str      = module_dual_ ? "DualCoPro_App"  : "Co_CPU_App";
    const char* emulates_str = module_dual_ ? "RPI-RF-MOD"     : "HM-MOD-RPI-PCB";
    mdns_txt_item_t txts[] = {
        {"model",    "CULFW32"},
        {"fw",       fw_buf},
        {"tag",      tag_str},
        {"emulates", emulates_str},
        {"wire",     "hb-rf-eth"},
        {"caps",     kCaps},
    };
    if (mdns_service_add(NULL, "_raw-uart", "_udp", port,
                         txts, sizeof(txts)/sizeof(txts[0])) == ESP_OK) {
        mdns_added_ = true;
    } else {
        ESP_LOGW(TAG, "mDNS service add failed (mDNS not initialized?)");
    }

    started_ = true;
    ESP_LOGI(TAG, "HB-RF-ETH UDP-listener bound on :%u (mDNS _raw-uart._udp)", port);
    return ESP_OK;
}

void HbrfethListener::set_module_identity(bool dual) {
    if (module_dual_ == dual && started_) return; // no-op
    module_dual_ = dual;
    if (!mdns_added_) return; // initial-Set vor start() reicht
    const char* tag_str      = dual ? "DualCoPro_App"  : "Co_CPU_App";
    const char* emulates_str = dual ? "RPI-RF-MOD"     : "HM-MOD-RPI-PCB";
    mdns_service_txt_item_set("_raw-uart", "_udp", "tag", tag_str);
    mdns_service_txt_item_set("_raw-uart", "_udp", "emulates", emulates_str);
    ESP_LOGI(TAG, "mDNS TXT updated: tag=%s emulates=%s", tag_str, emulates_str);
}

esp_err_t HbrfethListener::stop() {
    if (!started_) return ESP_OK;
    started_ = false;
    if (keepalive_timer_) {
        esp_timer_stop(static_cast<esp_timer_handle_t>(keepalive_timer_));
        esp_timer_delete(static_cast<esp_timer_handle_t>(keepalive_timer_));
        keepalive_timer_ = nullptr;
    }
    if (pcb_) {
        udp_recv(pcb_, nullptr, nullptr);
        udp_remove(pcb_);
        pcb_ = nullptr;
    }
    if (mdns_added_) {
        mdns_service_remove("_raw-uart", "_udp");
        mdns_added_ = false;
    }
    return ESP_OK;
}

void HbrfethListener::udp_recv_trampoline_(void* arg, struct udp_pcb* /*pcb*/,
                                            struct pbuf* p, const ip_addr_t* addr, uint16_t port) {
    auto* self = static_cast<HbrfethListener*>(arg);
    if (!p) return;
    if (p->len < 4) { pbuf_free(p); return; }
    self->on_udp_packet_(static_cast<const uint8_t*>(p->payload), p->len, addr, port);
    pbuf_free(p);
}

void HbrfethListener::on_udp_packet_(const uint8_t* data, size_t len,
                                     const ip_addr_t* addr, uint16_t port) {
    // CRC16 ueber alle bytes ausser den letzten 2 (CRC selbst).
    if (len < 4) return;
    uint16_t crc_calc = hmu_crc16({data, len - 2});
    uint16_t crc_recv = (static_cast<uint16_t>(data[len - 2]) << 8) | data[len - 1];
    if (crc_calc != crc_recv) {
        ESP_LOGW(TAG, "bad CRC from %u.%u.%u.%u:%u (calc %04x recv %04x)",
                 ip4_addr1(&addr->u_addr.ip4), ip4_addr2(&addr->u_addr.ip4),
                 ip4_addr3(&addr->u_addr.ip4), ip4_addr4(&addr->u_addr.ip4),
                 port, crc_calc, crc_recv);
        return;
    }

    const uint8_t type = data[0];
    uint8_t cnt        = data[1];     // mutable damit wir &cnt als counter_override uebergeben koennen
    const uint8_t* payload = data + 2;
    const size_t   payload_len = len - 4;

    xSemaphoreTake(mtx_, portMAX_DELAY);
    ClientState* cs = (type == T_CONNECT) ? find_or_create_(addr->u_addr.ip4.addr, port)
                                          : find_(addr->u_addr.ip4.addr, port);
    if (!cs && type != T_CONNECT) {
        // Nicht-CONNECT von unbekanntem Client → ignorieren (HB-RF-ETH source
        // macht das auch).
        xSemaphoreGive(mtx_);
        return;
    }
    cs->last_rx_us = esp_timer_get_time();

    switch (type) {
    case T_CONNECT: {
        cs->connected = true;
        cs->started   = false;
        // Wire-Format vom Client: [type=0, cnt, ver, (ep_id), crc_hi, crc_lo]
        // V1: payload-len 1 (nur ver=1).  V2: payload-len 2 (ver=2, client-ep-id).
        // Antwort byte-genau Alex Reinerts HB-RF-ETH `RawUartUdpListener::on_packet
        //   case 0` (sendMessage haengt eigenen tx-counter im outer-header an;
        //   der mirror-counter `data[1]` lebt IM payload):
        //   V1-Reply: payload = [status=1, mirror_cnt]
        //   V2-Reply: payload = [status=2, mirror_cnt, server_ep_id]
        if (payload_len == 1 && payload[0] == 1) {
            cs->endpoint_id += 2;
            uint8_t resp[2] = { 1, cnt };
            send_typed_(addr, port, T_CONNECT, resp, 2, nullptr);
        } else if (payload_len == 2 && payload[0] == 2) {
            uint8_t client_ep = payload[1];
            if (client_ep == 0) {
                cs->endpoint_id += 2;
                cs->started = false;
            } else if (client_ep != cs->endpoint_id) {
                ESP_LOGW(TAG, "reconnect with bad ep-id %u (expected %u)", client_ep, cs->endpoint_id);
                xSemaphoreGive(mtx_);
                return;
            }
            uint8_t resp[3] = { 2, cnt, cs->endpoint_id };
            send_typed_(addr, port, T_CONNECT, resp, 3, nullptr);
        }

        // Neuen virtuellen Transport anlegen + an Frontend uebergeben.
        if (!cs->transport && frontend_) {
            cs->transport = std::make_shared<HbrfethVirtualTransport>(this,
                                                                       addr->u_addr.ip4.addr, port);
            // Frontend braucht den shared_ptr — release mutex bevor wir nach
            // aussen rufen damit kein Recursive-Lock entstehen kann.
            auto fe = frontend_;
            auto vt = cs->transport;
            xSemaphoreGive(mtx_);
            // hb-rf-eth ist debmatic/multimacd → immer Dual-Mode-Slot.
            fe->attach_dynamic_transport(vt, HmuartlgwFrontend::ModuleMode::Dual);
            return;
        }
        break;
    }
    case T_DISCONNECT:
        disconnect_(*cs);
        break;
    case T_KEEPALIVE:
        // last_rx_us schon gesetzt — nichts weiter zu tun.
        break;
    case T_LED:
        // noop — wir haben keine adressierbare LED fuer den Bridge-Mode
        break;
    case T_RESET:
        // noop — Reset koennten wir an HmuartlgwFrontend ableiten (z.B.
        // AskSinRF re-init), aber heute ignorieren.
        break;
    case T_STARTCONN:
        cs->started = true;
        break;
    case T_STOPCONN:
        cs->started = false;
        break;
    case T_FRAME: {
        // Type-7 Payload sind rohe 0xfd-HMUARTLGW-Bytes. WICHTIG: deliver_rx
        // ruft transitiv unsere send_frame auf (Antwort-Pfad), die wieder
        // mtx_ nehmen will. Ohne Release hier: Deadlock (mutex ist nicht
        // recursive). Wir kopieren den shared_ptr unter mtx, releasen, und
        // rufen dann ausserhalb.
        auto vt = cs->transport;
        xSemaphoreGive(mtx_);
        if (vt) vt->deliver_rx({payload, payload_len});
        return;  // mtx schon released
    }
    default:
        ESP_LOGW(TAG, "unknown type=%u from client", type);
        break;
    }
    xSemaphoreGive(mtx_);
}

void HbrfethListener::send_frame(uint32_t client_ip, uint16_t client_port,
                                  const uint8_t* data, size_t len) {
    xSemaphoreTake(mtx_, portMAX_DELAY);
    ClientState* cs = find_(client_ip, client_port);
    if (!cs || !cs->started || !cs->connected) {
        xSemaphoreGive(mtx_);
        return;  // kein StartConn empfangen → drop frame (HB-RF-ETH-Verhalten)
    }
    ip_addr_t addr;
    addr.type = IPADDR_TYPE_V4;
    addr.u_addr.ip4.addr = client_ip;
    send_typed_(&addr, client_port, T_FRAME, data, len, nullptr);
    xSemaphoreGive(mtx_);
}

void HbrfethListener::send_typed_(const ip_addr_t* addr, uint16_t port,
                                   uint8_t type, const uint8_t* payload, size_t payload_len,
                                   uint8_t* counter_override) {
    const size_t total = 4 + payload_len;
    pbuf* pb = pbuf_alloc(PBUF_TRANSPORT, total, PBUF_RAM);
    if (!pb) return;
    auto* buf = static_cast<uint8_t*>(pb->payload);

    buf[0] = type;
    if (counter_override) {
        buf[1] = *counter_override;
    } else {
        ClientState* cs = find_(addr->u_addr.ip4.addr, port);
        buf[1] = cs ? cs->tx_counter++ : 0;
    }
    if (payload_len) std::memcpy(buf + 2, payload, payload_len);
    uint16_t crc = hmu_crc16({buf, total - 2});
    buf[total - 2] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    buf[total - 1] = static_cast<uint8_t>(crc & 0xFF);

    udp_sendto(pcb_, pb, addr, port);
    pbuf_free(pb);
}

void HbrfethListener::keepalive_trampoline_(void* arg) {
    static_cast<HbrfethListener*>(arg)->keepalive_tick_();
}

void HbrfethListener::keepalive_tick_() {
    if (!started_) return;
    int64_t now = esp_timer_get_time();
    xSemaphoreTake(mtx_, portMAX_DELAY);
    for (auto& cs : clients_) {
        if (!cs.connected) continue;
        if (now - cs.last_rx_us > kKeepAliveTimeoutUs) {
            ESP_LOGW(TAG, "keepalive timeout for %u.%u.%u.%u:%u — disconnecting",
                     (cs.ip      ) & 0xFF, (cs.ip >>  8) & 0xFF,
                     (cs.ip >> 16) & 0xFF, (cs.ip >> 24) & 0xFF, cs.port);
            disconnect_(cs);
            continue;
        }
        // Out-KeepAlive senden.
        ip_addr_t addr;
        addr.type = IPADDR_TYPE_V4;
        addr.u_addr.ip4.addr = cs.ip;
        send_typed_(&addr, cs.port, T_KEEPALIVE, nullptr, 0, nullptr);
    }
    xSemaphoreGive(mtx_);
}

HbrfethListener::ClientState* HbrfethListener::find_(uint32_t ip, uint16_t port) {
    for (auto& cs : clients_) {
        if (cs.connected && cs.ip == ip && cs.port == port) return &cs;
    }
    return nullptr;
}

HbrfethListener::ClientState* HbrfethListener::find_or_create_(uint32_t ip, uint16_t port) {
    if (auto* cs = find_(ip, port)) return cs;
    for (auto& cs : clients_) {
        if (!cs.connected) {
            cs = ClientState{};
            cs.ip = ip;
            cs.port = port;
            cs.endpoint_id = 1;
            cs.tx_counter = 0;
            cs.last_rx_us = esp_timer_get_time();
            return &cs;
        }
    }
    return nullptr;  // alle Slots belegt
}

void HbrfethListener::disconnect_(ClientState& cs) {
    if (cs.transport && frontend_) {
        // Detach BEFORE clearing — frontend lookup uses transport ptr.
        Transport* raw = cs.transport.get();
        // Mutex ist gehalten — frontend->detach nimmt sein eigenes mtx_.
        // Cross-Mutex-Order: hbrfeth.mtx → frontend.mtx (selbe Reihenfolge
        // wie wir bei Connect haben → kein Deadlock-Pfad).
        frontend_->detach_dynamic_transport(raw);
    }
    cs = ClientState{};
}

} // namespace culfw32
