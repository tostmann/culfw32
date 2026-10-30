// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "transport_websocket.hpp"
#include <functional>
#include <memory>

namespace culfw32 {

// WebServer — ESP-IDF httpd mit URI-Handlers fuer Static-Files (Web-UI)
// und WebSocket-Endpoints (/ws/json, /ws/tty). Pro accept'ten WS-Client
// erzeugt der Server eine WebSocketTransport-Shared-Pointer und reicht
// sie an den jeweils passenden Frontend-Hook (on_open).
class WebServer {
public:
    using OnWsOpen = std::function<void(std::shared_ptr<WebSocketTransport>)>;

    esp_err_t start(uint16_t port);
    esp_err_t stop();

    // Vor start() aufrufen: jeweils ein Hook fuer JSON- und TTY-WS.
    void set_json_hook(OnWsOpen cb) { on_json_open_ = std::move(cb); }
    void set_tty_hook (OnWsOpen cb) { on_tty_open_  = std::move(cb); }

    httpd_handle_t handle() const { return server_; }

private:
    httpd_handle_t server_ = nullptr;
    OnWsOpen       on_json_open_;
    OnWsOpen       on_tty_open_;
};

} // namespace culfw32
