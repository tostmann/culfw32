// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "transport.hpp"
#include "rf_protocol.hpp"
#include <span>
#include <memory>

namespace culfw32 {

class RfManager;      // forward
class NvsEeprom;      // forward
class LedDriver;      // forward
class WifiManager;    // forward
class EthManager;     // forward
class TcpServer;      // forward
class ConfigRegistry; // forward

// System-Services die einem Frontend von der Komposition (main) gereicht werden.
// Wachsen mit jedem neuen Querschnitts-Modul (WiFi-Status, NTP, MQTT-Bus etc.) —
// das Frontend-Interface bleibt dabei stabil.
struct FrontendServices {
    RfManager*       rf_manager = nullptr;
    NvsEeprom*       eeprom     = nullptr;
    LedDriver*       led        = nullptr;
    WifiManager*     wifi       = nullptr;
    TcpServer*       tcp_server = nullptr;     // optional, fuer EWP-Port-Restart
    ConfigRegistry*  config     = nullptr;     // typed key/value introspection
    EthManager*      eth        = nullptr;     // W5500 an J1 (optional, Ei/Ec/Ed)
};

// Schicht 2: Frontend-Logik. Kommando-Sprache parsen, an Manager dispatchen,
// RF-Events rendern und auf Transport(s) schreiben.
class Frontend {
public:
    virtual ~Frontend() = default;

    virtual const char* name() const = 0;
    virtual std::span<const ProtocolId> compatible_protocols() const = 0;

    virtual void attach(const FrontendServices& services) = 0;
    virtual void bind_transport(std::shared_ptr<Transport> t) = 0;

    virtual esp_err_t start() = 0;
    virtual esp_err_t stop()  = 0;
};

} // namespace culfw32
