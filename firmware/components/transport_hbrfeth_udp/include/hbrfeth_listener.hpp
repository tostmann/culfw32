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
//     5  StartConn  (Type-7-Frames werden erst nach diesem Type weitergeleitet)
//     6  StopConn   (gegenteil)
//     7  Frame      (raw HMUARTLGW 0xfd-binary frame durchgereicht)
//
// Per-Client-State: ein Eintrag pro (IP, Port). Multi-Client wird unterstuetzt
// (z.B. 2 rfd-Instanzen parallel — selten aber technisch ok).

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

    // Ausgehende Type-7-Frame an einen Client (vom HmuartlgwFrontend ueber
    // den HbrfethVirtualTransport.write() Pfad).
    void send_frame(uint32_t client_ip, uint16_t client_port, const uint8_t* data, size_t len);

private:
    static void udp_recv_trampoline_(void* arg, struct udp_pcb* pcb,
                                     struct pbuf* p, const ip_addr_t* addr, uint16_t port);
    void on_udp_packet_(const uint8_t* data, size_t len, const ip_addr_t* addr, uint16_t port);

    static void keepalive_trampoline_(void* arg);
    void keepalive_tick_();

    void send_typed_(const ip_addr_t* addr, uint16_t port,
                     uint8_t type, const uint8_t* payload, size_t payload_len, uint8_t* counter);

    struct ClientState {
        uint32_t ip = 0;
        uint16_t port = 0;
        uint8_t  endpoint_id = 1;       // V2-Reconnect-Tracking
        uint8_t  tx_counter = 0;        // unsere counter fuer outgoing
        bool     connected = false;
        bool     started   = false;     // Type-5 StartConn empfangen
        int64_t  last_rx_us = 0;
        std::shared_ptr<HbrfethVirtualTransport> transport;
    };

    static constexpr size_t kMaxClients = 4;
    ClientState* find_or_create_(uint32_t ip, uint16_t port);
    ClientState* find_(uint32_t ip, uint16_t port);
    void disconnect_(ClientState& cs);

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
