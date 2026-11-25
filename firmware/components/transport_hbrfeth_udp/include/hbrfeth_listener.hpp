// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

// HB-RF-ETH UDP-Tunnel-Emulation. Macht CULFW32 fuer eq-3-OCCU-Stacks
// (debmatic, piVCCU, RaspberryMatic) ueber das `hb_rf_eth`-Kernel-Modul
// erreichbar — der Linux-Host sieht uns als ueber Ethernet angeschlossenen
// HM-MOD-RPI-PCB.
//
// Wire-Protokoll (verifiziert gegen alexreinert/HB-RF-ETH `rawuartudplistener.cpp`):
//
//   UDP-Port 3008
//   Frame: [Type:1][Counter:1][Payload:n-4][CRC16:2]
//   CRC16: poly 0x8005, init 0xd77f (identisch zu HMUARTLGW-Wrapper)
//
//   Type-Codes:
//     0  Connect    (V1: 1 byte version=1; V2: 2 byte version=2 + endpoint-id)
//     1  Disconnect
//     2  KeepAlive  (empty payload; 1 Hz host→modul, 5 s timeout)
//     3  LED        (1 byte rgb-bits — wir ignorieren / noop)
//     4  Reset      (modul-reset — wir ignorieren / noop)
//     5  StartConn  (Type-7-Frames werden erst nach diesem Type weitergeleitet;
//                   gilt bis StopConn/Disconnect, auch ueber Keepalive-Timeout
//                   und Resume hinweg)
//     6  StopConn   (gegenteil)
//     7  Frame      (raw HMUARTLGW 0xfd-binary frame durchgereicht)
//
// Ein Slot je Host-IP (max 4). hb_rf_eth.ko (piVCCU kernel/hb_rf_eth.c) baut
// nach 5 s ohne Paket mit frischem Socket, also neuem Quellport, neu auf und
// meldet sich per V2-CONNECT mit der zuletzt vergebenen Endpoint-ID. StartConn
// schickt nur generic_raw_uart_open() beim ersten open() von /dev/raw-uart,
// nach einem Resume also nicht erneut. Deshalb ueberlebt die Session eines
// Hosts (Endpoint-ID, StartConn-Zustand und der virtuelle Transport samt
// HMUARTLGW-Zustand im Frontend) Keepalive-Timeout und Resume von einem neuen
// Port; erst DISCONNECT vergisst sie, eine neue Session (V1, V2 mit ep 0)
// ersetzt sie. So haelt es auch die Referenz alexreinert/HB-RF-ETH
// (rawuartudplistener.cpp: _connectionStarted ueberlebt Timeout und Resume).
//
// Sperrreihenfolge: das Frontend schreibt unter seinem eigenen Mutex ueber
// send_frame() (frontend.mtx -> hbrfeth.mtx_). Deshalb ruft der Listener das
// Frontend (attach/detach, deliver_rx) nie unter mtx_ auf.

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include <cstdint>
#include <atomic>
#include <array>
#include <functional>
#include <memory>
#include <vector>

namespace culfw32 {

class HmuartlgwFrontend;
class HbrfethVirtualTransport;

class HbrfethListener {
public:
    // Frontend, dem neue Connections (= virtuelle Transports) zugewiesen
    // werden. Setze BEVOR start().
    void set_frontend(HmuartlgwFrontend* fe) { frontend_ = fe; }

    esp_err_t start(uint16_t port = 3008);
    esp_err_t stop();

    // mDNS-TXT Identitaet (tag + emulates) gemaess Cross-Project-Schema:
    //   dual=false (Default) → tag=Co_CPU_App,    emulates=HM-MOD-RPI-PCB
    //   dual=true            → tag=DualCoPro_App, emulates=RPI-RF-MOD
    // Vor start() gerufen: setzt nur internen Zustand. Nach start(): patcht
    // die Service-TXT-Records live via mdns_service_txt_item_set().
    void set_module_identity(bool dual);

    // Ausgehender Type-7-Frame (vom HmuartlgwFrontend ueber
    // HbrfethVirtualTransport::write()). Geht an die aktuelle Adresse der
    // Session, die diesen Transport haelt — nach einem Resume ist das der neue
    // Quellport. Ein Transport einer ersetzten Session findet keinen Slot mehr.
    void send_frame(const HbrfethVirtualTransport* from, const uint8_t* data, size_t len);

private:
    static void udp_recv_trampoline_(void* arg, struct udp_pcb* pcb,
                                     struct pbuf* p, const ip_addr_t* addr, uint16_t port);
    void on_udp_packet_(const uint8_t* data, size_t len, const ip_addr_t* addr, uint16_t port);
    void on_connect_(uint32_t ip, uint16_t port, uint8_t cnt,
                     const uint8_t* payload, size_t payload_len);

    static void keepalive_trampoline_(void* arg);
    void keepalive_tick_();

    struct ClientState {
        uint32_t ip = 0;                // network byte order; 0 = Slot frei
        uint16_t port = 0;              // letzter Quellport des Hosts
        uint8_t  endpoint_id = 1;       // zuletzt vergebene/uebernommene Endpoint-ID
        uint8_t  tx_counter = 0;        // unsere counter fuer outgoing
        bool     connected   = false;   // Verbindung lebt (letztes Paket < 5 s)
        bool     has_session = false;   // endpoint_id gehoert zu einer Session dieses Hosts
        bool     started     = false;   // Type-5 StartConn gilt
        int64_t  last_rx_us = 0;
        std::shared_ptr<HbrfethVirtualTransport> transport;
    };

    // mtx_ MUSS vom Caller gehalten sein.
    void send_typed_(ClientState& cs, uint8_t type, const uint8_t* payload, size_t payload_len);

    static constexpr size_t kMaxClients = 4;
    // Slot fuer einen CONNECT von ip:port (mtx_ gehalten). Ein abgeloester
    // Transport eines verdraengten Hosts landet in `evicted` und muss nach dem
    // Freigeben von mtx_ beim Frontend abgemeldet werden.
    ClientState* slot_for_connect_(uint32_t ip, uint16_t port,
                                   std::shared_ptr<HbrfethVirtualTransport>& evicted);
    ClientState* find_connected_(uint32_t ip, uint16_t port);
    // 0 heisst im V2-CONNECT "neue Session" und wird nie vergeben.
    static uint8_t next_endpoint_id_(uint8_t ep);

    HmuartlgwFrontend* frontend_ = nullptr;
    struct udp_pcb*    pcb_ = nullptr;
    void*              keepalive_timer_ = nullptr;
    SemaphoreHandle_t  mtx_ = nullptr;
    std::array<ClientState, kMaxClients> clients_{};
    bool started_ = false;
    bool mdns_added_ = false;
    bool module_dual_ = false;   // mDNS-TXT Identitaet, Default Legacy/Co_CPU_App
};

} // namespace culfw32
