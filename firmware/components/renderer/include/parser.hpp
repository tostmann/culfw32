// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "rf_protocol.hpp"
#include <span>
#include <cstdint>

namespace culfw32 {

// Schicht 3b (Input): Format-Konverter wire bytes -> typisierter TX-Payload.
// Symmetric zum Renderer. Gehoert zur RF-Klasse / Protokoll, ist aber
// Frontend-getrieben (welches Frontend versteht welche Eingabe-Sprache?).
//
// Beispiel culfw-Emu: ParseResult{proto=AskSin, payload=AskSinTxFrame{...}}
// kommt aus dem Parsen von "As123ABC...\n".
struct ParseResult {
    ProtocolId proto;
    const void* typed_payload;   // Lebensdauer = Aufruf, Frontend kopiert
    size_t      payload_size;
};

class Parser {
public:
    virtual ~Parser() = default;

    virtual ProtocolId  proto() const = 0;
    virtual const char* format_name() const = 0;

    // Returns true wenn die Eingabe vollstaendig konsumiert/erkannt wurde
    // und in `out` ein TX-Payload bereitsteht.
    virtual bool parse(std::span<const uint8_t> input, ParseResult& out) = 0;
};

} // namespace culfw32
