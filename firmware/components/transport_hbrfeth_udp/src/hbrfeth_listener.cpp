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

// lwIP haelt IPv4 in network byte order; auf dem C6 (little endian) ist das
// erste Oktett das niederwertigste Byte.
#define IP4_FMT "%u.%u.%u.%u"
#define IP4_ARGS(ip) static_cast<unsigned>((ip) & 0xFF), static_cast<unsigned>(((ip) >> 8) & 0xFF), \
                     static_cast<unsigned>(((ip) >> 16) & 0xFF), static_cast<unsigned>(((ip) >> 24) & 0xFF)

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

    const uint8_t  type        = data[0];
    const uint8_t  cnt         = data[1];
    const uint8_t* payload     = data + 2;
    const size_t   payload_len = len - 4;
    const uint32_t ip          = addr->u_addr.ip4.addr;

    if (type == T_CONNECT) {
        on_connect_(ip, port, cnt, payload, payload_len);
        return;
    }

    // Frontend-Aufrufe erst nach dem Freigeben von mtx_ (siehe Header).
    std::shared_ptr<HbrfethVirtualTransport> rx_to;     // T_FRAME
    std::shared_ptr<HbrfethVirtualTransport> forget;    // T_DISCONNECT

    xSemaphoreTake(mtx_, portMAX_DELAY);
    ClientState* cs = find_connected_(ip, port);
    if (!cs) {
        // Nicht-CONNECT von unbekannter Adresse (oder vom alten Port nach
        // einem Resume) → ignorieren, wie HB-RF-ETH.
        xSemaphoreGive(mtx_);
        return;
    }
    cs->last_rx_us = esp_timer_get_time();

    switch (type) {
    case T_DISCONNECT:
        // Erst DISCONNECT vergisst die Session samt Transport.
        ESP_LOGI(TAG, "DISCONNECT from " IP4_FMT ":%u", IP4_ARGS(ip), port);
        forget = std::move(cs->transport);
        *cs = ClientState{};
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
    case T_FRAME:
        // Type-7 Payload sind rohe 0xfd-HMUARTLGW-Bytes. deliver_rx ruft
        // transitiv send_frame auf (Antwort-Pfad), das wieder mtx_ nimmt —
        // deshalb ausserhalb zustellen.
        rx_to = cs->transport;
        break;
    default:
        ESP_LOGW(TAG, "unknown type=%u from client", type);
        break;
    }
    xSemaphoreGive(mtx_);

    if (forget && frontend_) frontend_->detach_dynamic_transport(forget.get());
    if (rx_to) rx_to->deliver_rx({payload, payload_len});
}

void HbrfethListener::on_connect_(uint32_t ip, uint16_t port, uint8_t cnt,
                                  const uint8_t* payload, size_t payload_len) {
    // Wire-Format vom Client: [type=0, cnt, ver, (ep_id), crc_hi, crc_lo]
    // V1: payload-len 1 (nur ver=1).  V2: payload-len 2 (ver=2, client-ep-id).
    const bool v1 = payload_len == 1 && payload[0] == 1;
    const bool v2 = payload_len == 2 && payload[0] == 2;
    if (!v1 && !v2) {
        ESP_LOGW(TAG, "invalid CONNECT from " IP4_FMT ":%u (payload %u bytes) ignored",
                 IP4_ARGS(ip), port, static_cast<unsigned>(payload_len));
        return;
    }
    if (ip == 0) return;  // 0 markiert einen freien Slot
    const uint8_t client_ep = v2 ? payload[1] : 0;

    // Frontend-Aufrufe erst nach dem Freigeben von mtx_ (siehe Header).
    std::shared_ptr<HbrfethVirtualTransport> evicted;   // Slot eines anderen Hosts uebernommen
    std::shared_ptr<HbrfethVirtualTransport> replaced;  // alte Session dieses Hosts
    std::shared_ptr<HbrfethVirtualTransport> created;   // neu beim Frontend anmelden

    xSemaphoreTake(mtx_, portMAX_DELAY);
    ClientState* cs = slot_for_connect_(ip, port, evicted);
    if (!cs) {
        xSemaphoreGive(mtx_);
        ESP_LOGW(TAG, "CONNECT from " IP4_FMT ":%u rejected: %u hosts connected",
                 IP4_ARGS(ip), port, static_cast<unsigned>(kMaxClients));
        return;
    }

    // Semantik (piVCCU kernel/hb_rf_eth.c, hb_rf_eth_try_connect und
    // hb_rf_eth_recv_threadproc):
    //   V1, V2 mit ep 0 → neue Session; StartConn folgt mit dem ersten
    //                     open() von /dev/raw-uart.
    //   V2 mit ep != 0  → Resume: der Kernel hat 5 s kein Paket gesehen und
    //                     baut mit frischem Socket neu auf. StartConn schickt
    //                     er danach nicht erneut, solange die CCU das Device
    //                     offen haelt.
    const char* how;
    if (client_ep == 0) {
        cs->endpoint_id = next_endpoint_id_(cs->endpoint_id);
        cs->started     = false;
        // Der Host faengt mit seinem Init neu an → frischer HMUARTLGW-Zustand.
        replaced = std::move(cs->transport);
        how = "new session";
    } else if (cs->has_session && client_ep == cs->endpoint_id) {
        // started und Transport (HMUARTLGW-Zustand) bleiben, wie in der
        // Referenz alexreinert/HB-RF-ETH.
        how = "resume";
    } else {
        // Unbekannte Session, meist nach unserem eigenen Neustart (OTA,
        // Stromausfall). Die Referenz lehnt das ab; hb_rf_eth.ko wiederholt
        // den Resume dann alle ~0,6 s mit derselben ID und kommt nicht mehr
        // herein. Wir uebernehmen seine ID und nehmen StartConn als gegeben
        // an, sonst bliebe die Richtung Modul → CCU stumm. Haelt niemand
        // /dev/raw-uart offen, puffert der Kernel bis 1 KB (generic_raw_uart.c,
        // CIRCBUF_SIZE), verwirft den Rest mit je einer Meldung "rx fifo full"
        // und leert den Puffer beim naechsten ersten open().
        cs->endpoint_id = client_ep;
        cs->started     = true;
        how = "resume of unknown session, StartConn assumed";
    }
    cs->has_session = true;
    cs->connected   = true;
    cs->last_rx_us  = esp_timer_get_time();

    // Antwort byte-genau Alex Reinerts HB-RF-ETH `RawUartUdpListener::handlePacket
    //   case 0` (sendMessage haengt eigenen tx-counter im outer-header an;
    //   der mirror-counter `data[1]` lebt IM payload):
    //   V1-Reply: payload = [status=1, mirror_cnt]
    //   V2-Reply: payload = [status=2, mirror_cnt, server_ep_id]
    if (v1) {
        const uint8_t resp[2] = { 1, cnt };
        send_typed_(*cs, T_CONNECT, resp, sizeof(resp));
    } else {
        const uint8_t resp[3] = { 2, cnt, cs->endpoint_id };
        send_typed_(*cs, T_CONNECT, resp, sizeof(resp));
    }

    if (!cs->transport && frontend_) {
        cs->transport = std::make_shared<HbrfethVirtualTransport>(this);
        created = cs->transport;
    }
    const unsigned ep      = cs->endpoint_id;
    const int      started = cs->started ? 1 : 0;
    xSemaphoreGive(mtx_);

    ESP_LOGI(TAG, "V%u CONNECT from " IP4_FMT ":%u (ep_id=%u, %s, started=%d)",
             v1 ? 1u : 2u, IP4_ARGS(ip), port, ep, how, started);

    if (frontend_) {
        if (evicted)  frontend_->detach_dynamic_transport(evicted.get());
        if (replaced) frontend_->detach_dynamic_transport(replaced.get());
        // hb-rf-eth ist debmatic/multimacd → immer Dual-Mode-Slot.
        if (created)  frontend_->attach_dynamic_transport(created, HmuartlgwFrontend::ModuleMode::Dual);
    }
}

void HbrfethListener::send_frame(const HbrfethVirtualTransport* from,
                                  const uint8_t* data, size_t len) {
    if (!from || !mtx_) return;
    xSemaphoreTake(mtx_, portMAX_DELAY);
    for (auto& cs : clients_) {
        if (cs.transport.get() != from) continue;
        // Ohne StartConn oder ohne Verbindung verwerfen (HB-RF-ETH-Verhalten).
        if (cs.connected && cs.started) send_typed_(cs, T_FRAME, data, len);
        break;
    }
    xSemaphoreGive(mtx_);
}

void HbrfethListener::send_typed_(ClientState& cs, uint8_t type,
                                   const uint8_t* payload, size_t payload_len) {
    if (!pcb_) return;
    const size_t total = 4 + payload_len;
    pbuf* pb = pbuf_alloc(PBUF_TRANSPORT, total, PBUF_RAM);
    if (!pb) return;
    auto* buf = static_cast<uint8_t*>(pb->payload);

    buf[0] = type;
    buf[1] = cs.tx_counter++;
    if (payload_len) std::memcpy(buf + 2, payload, payload_len);
    uint16_t crc = hmu_crc16({buf, total - 2});
    buf[total - 2] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    buf[total - 1] = static_cast<uint8_t>(crc & 0xFF);

    ip_addr_t addr;
    addr.type = IPADDR_TYPE_V4;
    addr.u_addr.ip4.addr = cs.ip;
    udp_sendto(pcb_, pb, &addr, cs.port);
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
            // Nur die Verbindung ist weg. Session (Endpoint-ID, StartConn)
            // und Transport bleiben fuer den Resume von hb_rf_eth.ko stehen;
            // die Referenz loescht hier ebenfalls nur Adresse und Port.
            cs.connected = false;
            ESP_LOGW(TAG, "keepalive timeout for " IP4_FMT ":%u (ep_id=%u started=%d kept for resume)",
                     IP4_ARGS(cs.ip), cs.port, static_cast<unsigned>(cs.endpoint_id), cs.started ? 1 : 0);
            continue;
        }
        send_typed_(cs, T_KEEPALIVE, nullptr, 0);
    }
    xSemaphoreGive(mtx_);
}

HbrfethListener::ClientState* HbrfethListener::find_connected_(uint32_t ip, uint16_t port) {
    for (auto& cs : clients_) {
        if (cs.connected && cs.ip == ip && cs.port == port) return &cs;
    }
    return nullptr;
}

HbrfethListener::ClientState* HbrfethListener::slot_for_connect_(
        uint32_t ip, uint16_t port, std::shared_ptr<HbrfethVirtualTransport>& evicted) {
    // Gleicher Host, auch mit neuem Quellport oder nach Keepalive-Timeout:
    // sein Slot samt Session. hb_rf_eth.ko nimmt fuer jeden Verbindungsversuch
    // einen frischen Socket; ohne diese Zuordnung belegte jeder Versuch einen
    // eigenen Slot.
    for (auto& cs : clients_) {
        if (cs.ip == ip) {
            cs.port = port;
            return &cs;
        }
    }
    // Neuer Host: freier Slot, sonst der Host mit der aeltesten gemerkten,
    // nicht verbundenen Session.
    ClientState* slot = nullptr;
    for (auto& cs : clients_) {
        if (cs.connected) continue;
        if (cs.ip == 0) { slot = &cs; break; }
        if (!slot || cs.last_rx_us < slot->last_rx_us) slot = &cs;
    }
    if (!slot) return nullptr;  // alle Hosts verbunden
    if (slot->ip != 0) {
        ESP_LOGI(TAG, "session of " IP4_FMT " dropped for new host", IP4_ARGS(slot->ip));
    }
    evicted = std::move(slot->transport);
    *slot = ClientState{};
    slot->ip   = ip;
    slot->port = port;
    return slot;
}

uint8_t HbrfethListener::next_endpoint_id_(uint8_t ep) {
    // Nach einer uebernommenen geraden ID liefe += 2 sonst auf 0.
    ep = static_cast<uint8_t>(ep + 2);
    return ep ? ep : 1;
}

} // namespace culfw32
