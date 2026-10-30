// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "rf_protocol.hpp"
#include <span>
#include <cstdint>

namespace culfw32 {

// Schicht 3a (Output): Format-Konverter RF-Event -> wire bytes.
// Mehrere Renderer pro RF-Klasse möglich, austauschbar.
// v1: CulfwHexRenderer (byte-genau a-culfw), SignalduinoMsMuRenderer.
// Spaeter: MatterClusterRenderer, MqttJsonRenderer.
//
// Sink ist eine Funktion die fertige Bytes ans Frontend liefert
// (das sie auf seinen Transport schreibt).
class Renderer {
public:
    using Sink = std::function<void(std::span<const uint8_t>)>;

    virtual ~Renderer() = default;

    // Welches Protokoll versteht dieser Renderer?
    virtual ProtocolId proto() const = 0;

    // Bezeichner fuer Frontend-Konfig ("culfw_hex", "signalduino_msmu", "matter_cluster", ...).
    virtual const char* format_name() const = 0;

    virtual void render(const RxEvent& ev, Sink sink) = 0;
};

} // namespace culfw32
