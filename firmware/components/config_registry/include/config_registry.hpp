// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace culfw32 {

// ConfigRegistry — zentraler typed-key-value-Layer fuer alle persistierten und
// sich aendernden Geraete-Konfigurations-Werte. Single Source of Truth.
//
// Zweck: ASCII-Frontend, kuenftiges JSON/WS-Frontend, MQTT-Bridge mit HASS-
// Auto-Discovery und eine Web-UI iterieren oder mutieren denselben Satz von
// Eintraegen — ohne Doppel-Pflege je Frontend.
//
// Konvention fuer Keys: dotted, lowercase, gruppiert nach Subsystem.
// Beispiele: "wifi.ssid", "wifi.mode", "tcp.port", "tcp.allow", "rf.pa.idx".
//
// Jeder Eintrag transitiert zwischen Subsystem-internem Zustand und dem
// Frontend ueber Strings — der Setter parst, der Getter formatiert. Damit ist
// JSON-/HTTP-Serialisierung trivial und der Setter-Pfad einheitlich.

struct ConfigEntry {
    enum class Type : uint8_t {
        Bool,        // "true"/"false" / "1"/"0"
        U8, U16, U32,// dezimal
        Str,         // beliebige UTF-8-Strings
        Secret,      // wie Str, aber Getter liefert "***" (PSK, Tokens)
        Ipv4Cidr,    // "192.168.1.0/24" oder "any"
        IntEnum,     // dezimal-int mit enum_labels[idx] als Klartext
    };

    static constexpr uint8_t FLAG_PERSIST   = 1 << 0;  // → EE
    static constexpr uint8_t FLAG_READONLY  = 1 << 1;  // nur Getter
    static constexpr uint8_t FLAG_SENSITIVE = 1 << 2;  // niemals Klartext echo

    // Reihenfolge ist absichtlich so gewaehlt, dass C++20-designated-initializer
    // mit `.key/.type/.flags/.min/.max/.enum_labels/.hass_device_class/.getter/.setter/.help/.unit`
    // funktioniert (declaration-order = aggregate-order).
    std::string key;
    Type        type      = Type::Str;
    uint8_t     flags     = 0;
    int32_t     min_val   = 0;
    int32_t     max_val   = 0;
    std::vector<std::string> enum_labels;                // fuer Type::IntEnum
    std::string hass_device_class;                        // fuer kuenftige HASS-Discovery
    std::function<std::string()>           getter;
    std::function<bool(std::string_view)>  setter;       // returns success
    std::string help;
    std::string unit;

    bool persist()   const { return flags & FLAG_PERSIST; }
    bool readonly()  const { return flags & FLAG_READONLY; }
    bool sensitive() const { return flags & FLAG_SENSITIVE; }
};

class ConfigRegistry {
public:
    // Eintrag registrieren. Doppel-Keys werden ueberschrieben (letzter Win).
    void                  register_entry(ConfigEntry e);

    // Lookup. Nullptr wenn key unbekannt.
    const ConfigEntry*    find(std::string_view key) const;

    // Set/Get ueber den Setter/Getter-Funktor. Bei Sensitive-Flag liefert
    // get() bei vorhandenem Wert die Maske "***", bei leerem Wert "(unset)".
    bool                  set(std::string_view key, std::string_view value);
    std::string           get(std::string_view key) const;

    // Iteration in Insertion-Reihenfolge (stable fuer Frontend-Output).
    void                  iter(std::function<void(const ConfigEntry&)> cb) const;
    size_t                size() const { return entries_.size(); }

private:
    std::vector<ConfigEntry> entries_;
};

} // namespace culfw32
