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
//
// Task und Socket: Ein eigener Task ("hbrfeth") liest einen BSD-UDP-Socket,
// haelt die Keepalives samt Timeout und stellt Type-7-Frames dem Frontend zu;
// der HMUARTLGW-Dispatch (Funksendung, claim_radio_ bis 3 s) laeuft damit in
// diesem Task. Gesendet wird aus diesem Task und aus Frontend-Tasks
// (send_frame) mit sendto(), immer unter mtx_: ESP-IDF erlaubt Lesen, Schreiben
// und Schliessen aus verschiedenen Threads auf einem Socket, mehrere
// gleichzeitige Schreiber aber nur mit eigener Sperre (ESP-IDF API Guide
// "lwIP"). Die Empfangs-Mailbox des Sockets fasst 32 Datagramme
// (CONFIG_LWIP_UDP_RECVMBOX_SIZE, Vorgabe 6); hb_rf_eth.ko stellt hoechstens
// 31 hintereinander in seine Sendewarteschlange (QUEUE_LENGTH 32). SO_RCVBUF
// kappt die Bytes wartender Datagramme bei 8 KB (CONFIG_LWIP_SO_RCVBUF).
// Bis 0.2.23 lief der Listener ueber die lwIP-Raw-API: Empfang und
// Dispatch im tcpip-Thread (3 KB Stack, der IP-Stack stand solange still),
// udp_sendto() aus esp_timer- und Frontend-Tasks — laut derselben Quelle ist
// die Raw-API in ESP-IDF nicht unterstuetzt.

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <cstddef>
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
    static void task_trampoline_(void* arg);
    void task_loop_(int fd);
    // Ein Datagramm vom Socket lesen und verarbeiten; false, wenn keins da war.
    bool receive_one_(int fd, uint32_t wait_us);
    // ip in network byte order, port in host byte order.
    void on_udp_packet_(const uint8_t* data, size_t len, uint32_t ip, uint16_t port);
    void on_connect_(uint32_t ip, uint16_t port, uint8_t cnt,
                     const uint8_t* payload, size_t payload_len);
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

    // hb_rf_eth.ko sendet Datagramme bis BUFFER_SIZE = 1500 Byte.
    static constexpr size_t kMaxDatagram = 1500;

    HmuartlgwFrontend* frontend_ = nullptr;
    int                sock_ = -1;          // unter mtx_ lesen/schreiben
    TaskHandle_t       task_ = nullptr;
    std::atomic<bool>  run_{false};
    SemaphoreHandle_t  exited_ = nullptr;   // gibt der Task beim Ende
    SemaphoreHandle_t  mtx_ = nullptr;
    std::array<ClientState, kMaxClients> clients_{};
    // Puffer als Member, nicht auf dem Stack: send_frame() laeuft auch im
    // RX-Task der Funkklasse. rx_buf_ gehoert dem Task, tx_buf_ steht unter mtx_.
    uint8_t rx_buf_[kMaxDatagram];
    uint8_t tx_buf_[kMaxDatagram];
    bool started_ = false;
    bool mdns_added_ = false;
    bool module_dual_ = false;   // mDNS-TXT Identitaet, Default Legacy/Co_CPU_App
};

} // namespace culfw32
