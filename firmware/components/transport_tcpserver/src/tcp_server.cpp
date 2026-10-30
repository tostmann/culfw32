// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "tcp_server.hpp"
#include "tcp_client_transport.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <cstring>
#include <cerrno>

namespace culfw32 {

namespace {
constexpr const char* TAG = "tcp-srv";
}

esp_err_t TcpServer::start(uint16_t port, OnAccept on_accept) {
    if (running_) return ESP_OK;
    on_accept_ = std::move(on_accept);
    port_      = port;

    listen_sock_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock_ < 0) {
        ESP_LOGE(TAG, "socket: %s", strerror(errno));
        return ESP_FAIL;
    }
    int opt = 1;
    ::setsockopt(listen_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr = {};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(port);
    if (::bind(listen_sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind(%u): %s", port, strerror(errno));
        ::close(listen_sock_);
        listen_sock_ = -1;
        return ESP_FAIL;
    }
    if (::listen(listen_sock_, 4) < 0) {
        ESP_LOGE(TAG, "listen: %s", strerror(errno));
        ::close(listen_sock_);
        listen_sock_ = -1;
        return ESP_FAIL;
    }

    running_ = true;
    TaskHandle_t h = nullptr;
    // 4 KB Stack. Vom 11.05. bis 08.10.2026 waren es 8 KB als Vorsicht — ein
    // Stack-Problem war fuer den :2325-Heisenbug nur VERDAECHTIGT, gemessen
    // waren 564 B Verbrauch. Am 08.10. auf allen fuenf Listenern nach
    // angenommenen Verbindungen ~600 B (?stats stack-min ~7,6 KB von 8 KB).
    // accept() wartet in lwIP auf eine Mailbox; die Callbacks laufen im
    // tcpip-Task (tiT), nicht auf diesem Stack. Die 5 x 4 KB, die hier frei
    // werden, braucht das Online-Update: der TLS-Empfangspuffer will 16 KB am
    // Stueck, und mit allen Netzdiensten blieb der groesste freie Block bei
    // 14,6 KB (mbedtls_ssl_setup -0x7F00, gemessen 08.10. auf Board 1).
    if (xTaskCreate(&TcpServer::listen_task_trampoline, "tcp-srv", 4096, this, 5, &h)
        != pdPASS) {
        running_ = false;
        ::close(listen_sock_);
        listen_sock_ = -1;
        return ESP_ERR_NO_MEM;
    }
    task_handle_ = h;
    ESP_LOGI(TAG, "listening on 0.0.0.0:%u", port);
    return ESP_OK;
}

esp_err_t TcpServer::stop() {
    if (!running_) return ESP_OK;
    running_ = false;
    if (listen_sock_ >= 0) {
        ::shutdown(listen_sock_, SHUT_RDWR);
        ::close(listen_sock_);
        listen_sock_ = -1;
    }
    // listen_task self-deletes nach accept-Fehler. Kurz warten, damit der task
    // und sein TaskHandle wirklich weg sind, bevor ein restart() neu start()-et.
    vTaskDelay(pdMS_TO_TICKS(50));
    task_handle_ = nullptr;
    return ESP_OK;
}

esp_err_t TcpServer::restart(uint16_t new_port) {
    auto cb = on_accept_;     // Callback erhalten
    stop();
    return start(new_port, std::move(cb));
}

void TcpServer::set_allow(uint32_t ipv4_be, uint8_t cidr_bits) {
    if (cidr_bits > 32) cidr_bits = 32;
    allow_ip_   = ipv4_be;
    allow_bits_ = cidr_bits;
}

void TcpServer::listen_task_trampoline(void* arg) {
    static_cast<TcpServer*>(arg)->listen_task_loop();
    vTaskDelete(nullptr);
}

void TcpServer::listen_task_loop() {
    while (running_) {
        sockaddr_in client = {};
        socklen_t   slen   = sizeof(client);
        int cs = ::accept(listen_sock_,
                          reinterpret_cast<sockaddr*>(&client), &slen);
        if (cs < 0) {
            if (!running_) break;
            ESP_LOGW(TAG, "accept: %s", strerror(errno));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        char ipbuf[16];
        inet_ntoa_r(client.sin_addr, ipbuf, sizeof(ipbuf));

        // Allowlist-Check: bits=0 → jeder erlaubt; sonst client-IP & mask
        // gegen erlaubte IP & mask vergleichen. IP-Werte sind beide in
        // network-byte-order, mask wird aus CIDR generiert.
        if (allow_bits_ > 0) {
            uint32_t mask = (allow_bits_ >= 32)
                ? 0xFFFFFFFFu
                : htonl(~((1u << (32 - allow_bits_)) - 1u));
            uint32_t client_ip = client.sin_addr.s_addr;
            if ((client_ip & mask) != (allow_ip_ & mask)) {
                ESP_LOGW(TAG, "denied %s:%u (not in allow %u.%u.%u.%u/%u)",
                         ipbuf, ntohs(client.sin_port),
                         (unsigned)(allow_ip_ & 0xFF),
                         (unsigned)((allow_ip_ >> 8) & 0xFF),
                         (unsigned)((allow_ip_ >> 16) & 0xFF),
                         (unsigned)((allow_ip_ >> 24) & 0xFF),
                         allow_bits_);
                ::close(cs);
                continue;
            }
        }

        // TCP_NODELAY damit kleine culfw-Antwortframes nicht von Nagle gepuffert werden.
        int opt = 1;
        ::setsockopt(cs, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

        ESP_LOGI(TAG, "accepted fd=%d from %s:%u",
                 cs, ipbuf, ntohs(client.sin_port));

        // Caller besitzt den Transport (shared_ptr) und verdrahtet on_closed
        // selbst (typisch: Frontend::detach_dynamic_transport).
        auto t = std::make_shared<TcpClientTransport>(cs);
        if (on_accept_) on_accept_(t);
    }
}

} // namespace culfw32
