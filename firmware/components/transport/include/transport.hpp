// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include <cstdint>
#include <cstddef>
#include <span>
#include <functional>
#include "esp_err.h"

namespace culfw32 {

// Schicht 1: Byte-Pipe. Kennt keine Protokoll-Semantik.
// Implementierungen: UsbCdcTransport, TcpServerTransport (auf WiFi- oder W5500-netif), BleNusTransport, ...
// Frontends sind Transport-agnostisch — eine Frontend-Instanz kann an mehreren Transports gleichzeitig hängen.
class Transport {
public:
    using RxCallback = std::function<void(std::span<const uint8_t>)>;

    virtual ~Transport() = default;

    virtual esp_err_t start() = 0;
    virtual esp_err_t stop()  = 0;

    // Synchron senden. Blockiert bis Bytes im Treiber-Buffer sind oder Timeout.
    virtual esp_err_t write(std::span<const uint8_t> data) = 0;

    // Frontend abonniert RX. Callback läuft im Transport-Task-Kontext —
    // Frontend muss selbst entkoppeln wenn es eine eigene Task-Domäne hat.
    virtual void on_rx(RxCallback cb) = 0;

    // true: jeder RX-Callback traegt genau einen WebSocket-Frame, der als ganze
    // Nachricht gilt. Fragmentierte Nachrichten (RFC 6455 §5.4) reicht
    // web_server.cpp heute Frame fuer Frame durch, nicht zusammengesetzt.
    // false (Default): Bytestrom — wo ein Callback endet, ist Zufall
    // (TCP-Segmente, Groesse des recv()-Puffers).
    virtual bool message_framed() const { return false; }

    // Bytes, die der Transport angenommen, aber noch nicht abgegeben hat
    // (TxQueue der Netz-Clients). Der JSON-Live-Push laesst Clients mit
    // Rueckstand aus, statt ihren Puffer weiter zu fuellen.
    virtual size_t tx_backlog() const { return 0; }

    // true: Antworten auf Anfragen dieses Clients, die der Task schreibt, der
    // sie gerade empfangen hat, warten auf Abfluss statt im Puffer zu landen
    // (TcpClientTransport: sein Pool-Worker, WebSocketTransport: der httpd-Task
    // fuer diese Session). Ein langsamer Client bremst dann nur diesen Task.
    // Einschalten nur in Frontends, die beim Antworten keinen Mutex halten,
    // den auch der Funk-RX-Task braucht (Review 0903 H1), und vor start().
    // Default: aus; Transports ohne TxQueue ignorieren es.
    virtual void set_reply_wait(bool on) { (void)on; }

    virtual const char* name() const = 0;
};

} // namespace culfw32
