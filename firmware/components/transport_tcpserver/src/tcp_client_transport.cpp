// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "tcp_client_transport.hpp"
#include "tcp_rx_pool.hpp"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include <cstring>
#include <cerrno>

namespace culfw32 {

namespace {
constexpr const char* TAG = "tcp-cli";
}

TcpClientTransport::TcpClientTransport(int sock_fd)
    : sock_(sock_fd), txq_(TxQueue::create(sock_fd, TxQueue::Kind::Tcp)) {}

TcpClientTransport::~TcpClientTransport() {
    stop();
}

esp_err_t TcpClientTransport::start() {
    if (running_.load()) return ESP_OK;
    running_ = true;
    // Pool-Pfad statt xTaskCreate: rx-Loop laeuft in einem TcpRxPool-Worker.
    // shared_from_this() erfordert dass bereits ein shared_ptr auf diesen
    // Transport existiert (typisch: caller hat ihn aus TcpServer::on_accept
    // bekommen und haelt eine Referenz).
    auto self = shared_from_this();
    auto e = TcpRxPool::instance().submit(std::move(self));
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "pool submit failed: %s — closing fd=%d",
                 esp_err_to_name(e), sock_);
        running_ = false;
        if (txq_) txq_->close();
        if (sock_ >= 0) {
            ::shutdown(sock_, SHUT_RDWR);
            ::close(sock_);
            sock_ = -1;
        }
        return e;
    }
    ESP_LOGI(TAG, "client started fd=%d", sock_);
    return ESP_OK;
}

esp_err_t TcpClientTransport::stop() {
    // Idempotent — auch wenn running_ schon false ist (rx_task hat sich nach
    // recv()=0 selbst beendet), muessen wir den Socket trotzdem schliessen.
    // Der frueher-Bail-out hat einen Socket-FD-Leak verursacht: nach ~9
    // Disconnects waren alle 16 LWIP_MAX_SOCKETS weg, neue accepts feuerten
    // dann FIN sofort raus → Hello-Push und alle Frontends still.
    running_ = false;
    // Erst die TxQueue schliessen: danach sendet kein Task mehr auf sock_, und
    // die Nummer darf an den naechsten Client gehen.
    if (txq_) txq_->close();
    if (sock_ >= 0) {
        ::shutdown(sock_, SHUT_RDWR);
        ::close(sock_);
        sock_ = -1;
    }
    return ESP_OK;
}

// write() aus fremden Tasks wartet nie am Socket. Der Renderer ruft es aus dem
// RX-Task der Funkklasse auf, das JSON-Frontend aus dem esp_timer-Task; bis
// 0.1.893 hing dort ein blockierendes ::send, und ein Client, der nichts mehr
// abnahm, hielt den Empfang fuer ALLE Transports an. 0.1.894..0.1.901
// begrenzten das Warten (500 ms, dann 3 s ohne Fortschritt) und trennten dabei
// unter Last auch lesende Clients bzw. hielten den esp_timer-Task bis ~6 s an.
// Seit 0.1.902 puffert die TxQueue, und ihr Sende-Task trennt erst nach 3 s
// ohne Abfluss.
esp_err_t TcpClientTransport::write(std::span<const uint8_t> data) {
    if (!running_.load() || sock_ < 0 || !txq_) return ESP_ERR_INVALID_STATE;
    // Der eigene Empfangs-Task antwortet auf Anfragen dieses Clients: er darf
    // warten und bremst damit nur den Client selbst — sofern das Frontend
    // dabei nichts sperrt, was der Funk-RX-Task braucht. HMUARTLGW und
    // SIGNALduino antworten unter ihrem Frontend-Mutex und lassen es deshalb
    // aus (Review 0903 H1).
    if (reply_wait_.load() && xTaskGetCurrentTaskHandle() == rx_task_.load()) {
        return txq_->write_blocking(data);
    }
    return txq_->write(data);
}

void TcpClientTransport::run_rx_loop_public() {
    rx_task_ = xTaskGetCurrentTaskHandle();
    uint8_t buf[256];
    while (running_.load() && sock_ >= 0) {
        ssize_t n = ::recv(sock_, buf, sizeof(buf), 0);
        if (n > 0) {
            if (rx_cb_) rx_cb_({buf, static_cast<size_t>(n)});
        } else if (n == 0) {
            ESP_LOGI(TAG, "client EOF fd=%d", sock_);
            break;
        } else {
            if (errno == EINTR) continue;
            ESP_LOGW(TAG, "client recv err fd=%d: %s", sock_, strerror(errno));
            break;
        }
    }
    running_ = false;
    rx_task_ = nullptr;
}

void TcpClientTransport::invoke_on_closed_public() {
    if (on_closed_) on_closed_(this);
}

} // namespace culfw32
