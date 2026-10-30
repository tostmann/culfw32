// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
//
// Contains code ported from SIGNALduino (https://github.com/RFD-FHEM/SIGNALDuino):
//   send.h  no individual notice; the SIGNALDuino repository is licensed under
//           the GNU GPL version 3.
// Modified for CULFW32 (ESP-IDF port), Dirk Tostmann, 2026.

#pragma once

#include "frontend.hpp"
#include "transport.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <array>
#include <memory>
#include <string_view>
#include <vector>

namespace culfw32 {

// SIGNALduino-Emulationsfrontend (CULFW32-Extension, parallel zum
// ASCII-CulfwFrontend und JsonFrontend).
//
// Protokoll: zeilenbasiert, eigenes TCP-Port (Default 2326). FHEM-Define:
//   define <name> SIGNALduino <ip>:2326
// FHEM-Init-Sequenz: V → erwartet "V 3.x.x SIGNALduino[-...] HH:MM:SS"
//                    → XE (Receiver enable, silent ack)
//                    → optional CG/C0DnF (config/CC1101-readback)
//                    → laufend RX-Frames (MS/MU/MC) als unsolicited Output.
//
// Output-Format: ein Frame pro Zeile, terminiert mit \r\n.
//   MU;P0=384;P1=-396;P2=684;P3=-7800;D=232323020303;CP=0;R=87;
class SignalduinoFrontend final : public Frontend {
public:
    const char* name() const override { return "signalduino-emu"; }
    std::span<const ProtocolId> compatible_protocols() const override;
    void   attach(const FrontendServices& services) override;
    void   bind_transport(std::shared_ptr<Transport> t) override;
    esp_err_t start() override;
    esp_err_t stop()  override;

    // Dynamic-Attach fuer TCP-Clients, gleiche Semantik wie JsonFrontend.
    void attach_dynamic_transport(std::shared_ptr<Transport> t);
    void detach_dynamic_transport(Transport* t);

private:
    static constexpr size_t kMaxTransports = 4;
    static constexpr size_t kLineBufSize   = 1024;     // SR-Frames bis ~600 chars (P0..P7 + D=<512>)

    struct LineBuf {
        std::array<char, kLineBufSize> data{};
        size_t len = 0;
    };

    void on_rx_data_(std::span<const uint8_t> bytes, Transport* src);
    void process_line_(std::span<const char> line, Transport* src);
    void emit_to_(Transport* t, std::string_view text);
    void broadcast_(std::string_view text);

    void on_rx_event_(const struct RxEvent& ev);

    FrontendServices services_{};
    bool             started_ = false;

    std::vector<std::shared_ptr<Transport>> transports_;
    LineBuf  line_buf_per_transport_[kMaxTransports]{};
    Transport* lb_owner_[kMaxTransports]{};
    SemaphoreHandle_t mtx_ = nullptr;
};

} // namespace culfw32
