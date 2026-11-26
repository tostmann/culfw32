// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "hbrfeth_listener.hpp"
#include "hbrfeth_virtual_transport.hpp"
#include "hmuartlgw_frontend.hpp"
#include "hmuartlgw_frame.hpp"   // fuer hmu_crc16
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "sdkconfig.h"
#include "lwip/sockets.h"
#include "mdns.h"
#include <cerrno>
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

// Stack und Prioritaet wie die TcpRxPool-Worker, in denen dasselbe Frontend
// fuer TCP 2325/2327 laeuft (main.cpp: TcpRxPool init(8, 6144), Prio 6).
constexpr uint32_t    kTaskStack = 6144;
constexpr UBaseType_t kTaskPrio  = 6;
// Vor der Timeout-Pruefung alles abholen, was schon wartet: hoechstens so
// viele Datagramme, wie die Empfangs-Mailbox des Sockets fasst
// (sdkconfig.defaults: 32; hb_rf_eth.ko stellt hoechstens 31 hintereinander).
constexpr int kDrainMax = CONFIG_LWIP_UDP_RECVMBOX_SIZE;
// Hoechstens so viele Bytes Nutzlast wartender Datagramme (SO_RCVBUF): jedes
// haelt bis zum Abholen seinen Empfangspuffer (WLAN bzw. Ethernet). Bursts
// kleiner HMU-Rahmen passen ganz hinein, eine Flut grosser Pakete auf UDP 3008
// wird bei 8 KB gekappt, statt bis 32 x 1,5 KB zu binden, solange der Task in
// einem langen Dispatch steht (Review 0227 M1). Nicht begrenzt ist die Zahl:
// 32 kleine Datagramme koennen weiter alle 32 dynamischen WLAN-RX-Puffer
// halten (CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM), bis der Task sie abholt
// (Review 0228 N1, offen; mit der Raw-API galt dasselbe fuer die
// tcpip-Mailbox). Verworfen wird still, ohne Zaehler.
constexpr int kRcvBufBytes = 8192;
}  // anon

// lwIP haelt IPv4 in network byte order; auf dem C6 (little endian) ist das
// erste Oktett das niederwertigste Byte.
#define IP4_FMT "%u.%u.%u.%u"
#define IP4_ARGS(ip) static_cast<unsigned>((ip) & 0xFF), static_cast<unsigned>(((ip) >> 8) & 0xFF), \
                     static_cast<unsigned>(((ip) >> 16) & 0xFF), static_cast<unsigned>(((ip) >> 24) & 0xFF)

esp_err_t HbrfethListener::start(uint16_t port) {
    if (started_) return ESP_OK;
    if (!mtx_) mtx_ = xSemaphoreCreateMutex();
    if (!exited_) exited_ = xSemaphoreCreateBinary();
    if (!mtx_ || !exited_) return ESP_ERR_NO_MEM;

    const int fd = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        ESP_LOGE(TAG, "socket failed (errno %d)", errno);
        return ESP_FAIL;
    }
    sockaddr_in local = {};
    local.sin_family      = AF_INET;
    local.sin_port        = htons(port);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
        ESP_LOGE(TAG, "bind to :%u failed (errno %d)", port, errno);
        ::close(fd);
        return ESP_FAIL;
    }
    int rcvbuf = kRcvBufBytes;
    if (::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) != 0) {
        ESP_LOGW(TAG, "SO_RCVBUF %d failed (errno %d)", rcvbuf, errno);
    }
    xSemaphoreTake(mtx_, portMAX_DELAY);
    sock_ = fd;
    xSemaphoreGive(mtx_);
    // Ein Signal eines frueheren Tasks, auf den stop() nicht mehr gewartet
    // hat, darf das naechste stop() nicht vorzeitig enden lassen.
    xSemaphoreTake(exited_, 0);
    run_ = true;
    if (xTaskCreate(&HbrfethListener::task_trampoline_, "hbrfeth", kTaskStack, this,
                    kTaskPrio, &task_) != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        run_ = false;
        task_ = nullptr;
        xSemaphoreTake(mtx_, portMAX_DELAY);
        sock_ = -1;
        xSemaphoreGive(mtx_);
        ::close(fd);
        return ESP_ERR_NO_MEM;
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
    // Der Task prueft run_ spaetestens nach einer Keepalive-Periode (oder nach
    // einem laufenden Dispatch) und endet dann; erst danach den Socket
    // schliessen, damit select/recvfrom nicht auf einem fremden Deskriptor
    // landen, falls die Nummer neu vergeben wird.
    run_ = false;
    if (task_ && xSemaphoreTake(exited_, pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGW(TAG, "listener task did not stop within 10 s");
    }
    task_ = nullptr;
    xSemaphoreTake(mtx_, portMAX_DELAY);
    const int fd = sock_;
    sock_ = -1;
    xSemaphoreGive(mtx_);
    if (fd >= 0) ::close(fd);
    if (mdns_added_) {
        mdns_service_remove("_raw-uart", "_udp");
        mdns_added_ = false;
    }
    return ESP_OK;
}

void HbrfethListener::task_trampoline_(void* arg) {
    auto* self = static_cast<HbrfethListener*>(arg);
    xSemaphoreTake(self->mtx_, portMAX_DELAY);
    const int fd = self->sock_;
    xSemaphoreGive(self->mtx_);
    self->task_loop_(fd);
    xSemaphoreGive(self->exited_);
    vTaskDelete(nullptr);
}

void HbrfethListener::task_loop_(int fd) {
    int64_t next_tick = esp_timer_get_time() + kKeepAliveOutPeriodUs;
    while (run_.load()) {
        const int64_t now = esp_timer_get_time();
        if (now >= next_tick) {
            // Erst abholen, was schon im Socket liegt: nach einem langen
            // Dispatch (Funksendung, claim_radio_ bis 3 s) warten dort
            // Datagramme, deren Empfang noch nicht in last_rx_us steht.
            for (int i = 0; i < kDrainMax && receive_one_(fd, 0); ++i) {}
            keepalive_tick_();
            next_tick += kKeepAliveOutPeriodUs;
            if (next_tick <= now) next_tick = now + kKeepAliveOutPeriodUs;
            continue;
        }
        receive_one_(fd, static_cast<uint32_t>(next_tick - now));
    }
}

bool HbrfethListener::receive_one_(int fd, uint32_t wait_us) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    timeval tv = {};
    tv.tv_sec  = static_cast<decltype(tv.tv_sec)>(wait_us / 1000000);
    tv.tv_usec = static_cast<decltype(tv.tv_usec)>(wait_us % 1000000);
    const int r = ::select(fd + 1, &rfds, nullptr, nullptr, &tv);
    if (r == 0) return false;
    if (r < 0) {
        if (errno != EINTR) {
            ESP_LOGW(TAG, "select failed (errno %d)", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        return false;
    }
    sockaddr_in from = {};
    socklen_t from_len = sizeof(from);
    const ssize_t n = ::recvfrom(fd, rx_buf_, sizeof(rx_buf_), MSG_DONTWAIT,
                                 reinterpret_cast<sockaddr*>(&from), &from_len);
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            ESP_LOGW(TAG, "recvfrom failed (errno %d)", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        return false;
    }
    if (n >= 4 && from.sin_family == AF_INET) {
        on_udp_packet_(rx_buf_, static_cast<size_t>(n), from.sin_addr.s_addr,
                       ntohs(from.sin_port));
    }
    return true;
}

void HbrfethListener::on_udp_packet_(const uint8_t* data, size_t len,
                                     uint32_t ip, uint16_t port) {
    // CRC16 ueber alle bytes ausser den letzten 2 (CRC selbst).
    if (len < 4) return;
    uint16_t crc_calc = hmu_crc16({data, len - 2});
    uint16_t crc_recv = (static_cast<uint16_t>(data[len - 2]) << 8) | data[len - 1];
    if (crc_calc != crc_recv) {
        ESP_LOGW(TAG, "bad CRC from " IP4_FMT ":%u (calc %04x recv %04x)",
                 IP4_ARGS(ip), port, crc_calc, crc_recv);
        return;
    }

    const uint8_t  type        = data[0];
    const uint8_t  cnt         = data[1];
    const uint8_t* payload     = data + 2;
    const size_t   payload_len = len - 4;

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
    // mtx_ gehalten: serialisiert alle Schreiber auf sock_ und schuetzt tx_buf_.
    if (sock_ < 0) return;
    const size_t total = 4 + payload_len;
    if (total > sizeof(tx_buf_)) {
        ESP_LOGW(TAG, "frame too long for one datagram (%u bytes) dropped",
                 static_cast<unsigned>(total));
        return;
    }
    uint8_t* buf = tx_buf_;
    buf[0] = type;
    buf[1] = cs.tx_counter++;
    if (payload_len) std::memcpy(buf + 2, payload, payload_len);
    uint16_t crc = hmu_crc16({buf, total - 2});
    buf[total - 2] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    buf[total - 1] = static_cast<uint8_t>(crc & 0xFF);

    sockaddr_in to = {};
    to.sin_family      = AF_INET;
    to.sin_port        = htons(cs.port);
    to.sin_addr.s_addr = cs.ip;
    // UDP wartet nicht auf die Gegenstelle, nur auf den Rundlauf durch den
    // tcpip-Thread; scheitert sendto, entfaellt das Datagramm (ohne Zaehler).
    // MSG_DONTWAIT wirkt dabei nicht: lwip_sendto reicht die
    // Flags nur bei TCP weiter (lwIP sockets.c).
    ::sendto(sock_, buf, total, MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&to), sizeof(to));
}

// Laeuft im Listener-Task (task_loop_), einmal je Keepalive-Periode.
void HbrfethListener::keepalive_tick_() {
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
