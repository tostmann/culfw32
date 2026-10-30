// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "transport_websocket.hpp"
#include "esp_log.h"
#include <cstring>
#include <cstdio>

namespace culfw32 {

namespace {
constexpr const char* TAG = "ws-tx";

// Kopf eines unmaskierten Server-Frames mit FIN (RFC 6455 §5.2). h braucht
// 10 Byte; zurueck kommt die Kopflaenge.
size_t ws_header(uint8_t* h, httpd_ws_type_t type, size_t len) {
    h[0] = static_cast<uint8_t>(0x80 | (static_cast<uint8_t>(type) & 0x0F));
    if (len < 126) {
        h[1] = static_cast<uint8_t>(len);
        return 2;
    }
    if (len <= 0xFFFF) {
        h[1] = 126;
        h[2] = static_cast<uint8_t>(len >> 8);
        h[3] = static_cast<uint8_t>(len);
        return 4;
    }
    h[1] = 127;
    const uint64_t l = len;
    for (int i = 0; i < 8; ++i) h[2 + i] = static_cast<uint8_t>(l >> (56 - 8 * i));
    return 10;
}
}  // namespace

WebSocketTransport::WebSocketTransport(httpd_handle_t server, int sock_fd, const char* tag)
    : server_(server), sock_fd_(sock_fd), txq_(TxQueue::create(sock_fd, TxQueue::Kind::Ws)) {
    std::snprintf(tag_, sizeof(tag_), "%s", tag ? tag : "ws");
}

WebSocketTransport::~WebSocketTransport() {
    // Kann vor close_fn sterben (mark_closed, Sendefehler); dann findet
    // close_fn keinen Transport mehr und schliesst den fd direkt. Ab hier
    // sendet niemand mehr darauf (Review 0903 N2).
    if (txq_) txq_->close();
    if (open_.load() && server_) {
        // Best-effort close
        httpd_sess_trigger_close(server_, sock_fd_);
    }
}

esp_err_t WebSocketTransport::start() {
    return ESP_OK;        // nothing to do — Verbindung wird vom httpd verwaltet
}

esp_err_t WebSocketTransport::stop() {
    if (!open_.exchange(false)) return ESP_OK;
    if (server_) httpd_sess_trigger_close(server_, sock_fd_);
    return ESP_OK;
}

esp_err_t WebSocketTransport::write(std::span<const uint8_t> data) {
    if (!open_.load() || !server_) return ESP_ERR_INVALID_STATE;
    if (data.empty()) return ESP_OK;

    // Frontend liefert die culfw-Antwort als Byte-Stream (typisch eine Zeile
    // mit \r\n). Wir packen das als TEXT-Frame (=opcode 0x1) — Browser-JS
    // sieht einen String. Bei Json-Frontend ist das ohnehin JSON-Text. Bei
    // ASCII-culfw auch Text. Wenn jemand Binary brauchen sollte: separates
    // write_binary().
    const bool wait = reply_wait_.load() && xTaskGetCurrentTaskHandle() == rx_task_.load();
    return send_frame_(HTTPD_WS_TYPE_TEXT, data, false, wait);
}

esp_err_t WebSocketTransport::write_control(httpd_ws_type_t type, std::span<const uint8_t> data) {
    if (!open_.load() || !server_) return ESP_ERR_INVALID_STATE;
    // Leerer Payload ist erlaubt (PONG auf einen PING ohne Daten): dann geht
    // nur der Kopf raus.
    return send_frame_(type, data, false, false);
}

esp_err_t WebSocketTransport::send_frame_(httpd_ws_type_t type, std::span<const uint8_t> data,
                                          bool last, bool wait) {
    if (!txq_) return ESP_ERR_INVALID_STATE;
    uint8_t hdr[10];
    const size_t hl = ws_header(hdr, type, data.size());
    const esp_err_t r = wait ? txq_->write_blocking({hdr, hl}, data, &reply_budget_us_)
                             : txq_->write({hdr, hl}, data, last);
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        // Verbindung weg oder getrennt (Puffer reichte nicht): Session als
        // geschlossen markieren, damit das Frontend sie abhaengt.
        ESP_LOGW(TAG, "frame fd=%d: %s", sock_fd_, esp_err_to_name(r));
        if (!last && open_.exchange(false)) {
            auto cb = on_closed_;
            if (cb) cb(this);
        }
    }
    return r;
}

esp_err_t WebSocketTransport::close_with_frame(std::span<const uint8_t> status) {
    const bool was_open = open_.exchange(false);
    if (!was_open) return ESP_ERR_INVALID_STATE;
    // last: die TxQueue nimmt danach nichts mehr an — auch keinen Frame aus
    // einem Task, der open_ eben noch offen gesehen hat.
    const esp_err_t r = server_ ? send_frame_(HTTPD_WS_TYPE_CLOSE, status, true, false)
                                : ESP_ERR_INVALID_STATE;
    if (r != ESP_OK) ESP_LOGW(TAG, "close frame fd=%d: %s", sock_fd_, esp_err_to_name(r));
    auto cb = on_closed_;
    if (cb) cb(this);
    return r;
}

void WebSocketTransport::feed_rx_frame(std::span<const uint8_t> bytes) {
    if (!open_.load() || !rx_cb_) return;
    // Was dieser Task (httpd) waehrenddessen an diese Session schreibt, ist
    // die Antwort auf ihren eigenen Frame (set_reply_wait). Darauf warten darf
    // er insgesamt kReplyBudgetUs: er bedient alle Sockets des Web-Servers.
    // Ein Deckel je write() reichte nicht, der JSON-dump schreibt ~149-mal
    // (Review 0905 F1). Befehle, die selbst dauern, zaehlen nicht mit.
    reply_budget_us_ = kReplyBudgetUs;
    rx_task_ = xTaskGetCurrentTaskHandle();
    rx_cb_(bytes);
    rx_task_ = nullptr;
}

void WebSocketTransport::mark_closed() {
    if (!open_.exchange(false)) return;
    auto cb = on_closed_;
    if (cb) cb(this);
}

// close_fn schliesst den Socket gleich danach; vorher muss feststehen, dass
// kein Task mehr darauf sendet (sonst koennte die fd-Nummer schon einem neuen
// Client gehoeren, Review 0897 M1). TxQueue::close() nimmt dafuer deren Mutex;
// gesendet wird darunter nur nicht blockierend, die Wartezeit ist also kurz.
void WebSocketTransport::close_for_fd_release() {
    if (txq_) txq_->close();
    if (open_.exchange(false)) {
        auto cb = on_closed_;
        if (cb) cb(this);
    }
}

} // namespace culfw32
