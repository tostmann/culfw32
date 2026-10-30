// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
// Portions derived from a-culfw (https://github.com/tostmann/a-culfw),
// GNU GPL version 2 or (at your option) any later version.

#pragma once

#include "frontend.hpp"
#include "renderer.hpp"
#include "parser.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <vector>
#include <memory>

namespace culfw32 {

// v1-Pflicht-Frontend: culfw-ASCII-Emulation (USB-CDC-tty / TCP-Server).
// Versteht das fntab-Kommando-Set aus a-culfw (A/Z/F/K/M/T/N/f/u/X/B/C/x/...).
//
// Kompatible RF-Klassen: AskSin, Moritz, SlowRf, FastRf, Native, ... (alle auf
// Hex-Frame-Decode-Basis). NICHT kompatibel: PulseCapture (=> Signalduino-Frontend).
class CulfwFrontend final : public Frontend {
public:
    CulfwFrontend();

    const char* name() const override { return "culfw-emu"; }
    std::span<const ProtocolId> compatible_protocols() const override;

    void attach(const FrontendServices& services) override;
    void bind_transport(std::shared_ptr<Transport> t) override;
    esp_err_t start() override;
    esp_err_t stop()  override;

    // Dynamische Anbindung waehrend der Laufzeit (TcpServer-Accept-Pfad).
    // Thread-safe — interne Mutex schuetzt die Transport-Liste gegen Renderer.
    // Startet den Transport sofort und verdrahtet on_rx mit dem Frontend.
    esp_err_t attach_dynamic_transport(std::shared_ptr<Transport> t);

    // Loescht einen dynamic Transport aus der Liste (z.B. bei TCP-EOF). Der
    // shared_ptr-Refcount geht auf 0 sobald keine in-flight Renderer-Snapshots
    // ihn mehr halten — dann ruft der dtor stop() (idempotent). Returns ok
    // auch wenn Transport nicht in Liste war.
    esp_err_t detach_dynamic_transport(Transport* t);

private:
    // Pro RF-Klasse genau ein Renderer + ein Parser im culfw-ASCII-Format.
    void register_codecs();

    // Transport-RX -> Kommando-Dispatch (entsprechend a-culfw fntab).
    void handle_rx(std::span<const uint8_t> data, Transport* src);

    // Snapshot der Transport-Liste fuer Renderer (lock-Free Iteration).
    std::vector<std::shared_ptr<Transport>> transports_snapshot();

    FrontendServices                                services_{};
    std::vector<std::shared_ptr<Transport>>         transports_;
    SemaphoreHandle_t                               transports_mtx_ = nullptr;
    std::vector<std::unique_ptr<Renderer>>          renderers_;
    std::vector<std::unique_ptr<Parser>>            parsers_;
};

} // namespace culfw32
