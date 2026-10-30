// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "transport.hpp"
#include "tcp_client_transport.hpp"
#include "esp_err.h"
#include <cstdint>
#include <functional>
#include <memory>

namespace culfw32 {

// TcpServer — bindet auf 0.0.0.0:<port> und ruft `on_accept` fuer jeden neuen
// Client mit einem fertigen TcpClientTransport-Pointer. Der Caller (Frontend-
// Komposition) haengt den Transport ans Frontend dynamisch an und kann auch
// noch on_closed verdrahten bevor er ihn der Frontend-Liste zureicht.
class TcpServer {
public:
    using OnAccept = std::function<void(std::shared_ptr<TcpClientTransport>)>;

    esp_err_t start(uint16_t port, OnAccept on_accept);
    esp_err_t stop();
    // Re-bind auf neuen Port unter Beibehaltung des on_accept-Callbacks.
    // Schliesst aktive listen-socket, startet listen-task neu — bestehende
    // Client-Sockets bleiben offen (lwip-Socket-State liegt nicht am Listener).
    esp_err_t restart(uint16_t new_port);
    uint16_t  port() const { return port_; }

    // IP-Allowlist: nur Clients aus <ip>/<bits>-Subnet duerfen connecten.
    // Default 0.0.0.0/0 = alles. `bits=32` ist single-Host-Allowlist.
    // `bits>32` wird auf 32 geclamped, `bits=0` = alles erlaubt (default).
    void      set_allow(uint32_t ipv4_be, uint8_t cidr_bits);
    uint32_t  allow_ip()   const { return allow_ip_; }
    uint8_t   allow_bits() const { return allow_bits_; }

private:
    static void listen_task_trampoline(void* arg);
    void        listen_task_loop();

    int       listen_sock_ = -1;
    uint16_t  port_        = 0;
    void*     task_handle_ = nullptr;
    OnAccept  on_accept_;
    bool      running_     = false;
    // Allow-Subnet (network byte order). Default 0/0 = alle Clients erlaubt.
    uint32_t  allow_ip_    = 0;
    uint8_t   allow_bits_  = 0;
};

} // namespace culfw32
