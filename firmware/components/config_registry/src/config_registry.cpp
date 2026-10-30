// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "config_registry.hpp"

namespace culfw32 {

void ConfigRegistry::register_entry(ConfigEntry e) {
    // Wenn key existiert: in-place ueberschreiben (Insertion-Order bleibt).
    for (auto& existing : entries_) {
        if (existing.key == e.key) { existing = std::move(e); return; }
    }
    entries_.emplace_back(std::move(e));
}

const ConfigEntry* ConfigRegistry::find(std::string_view key) const {
    for (auto& e : entries_) {
        if (e.key == key) return &e;
    }
    return nullptr;
}

bool ConfigRegistry::set(std::string_view key, std::string_view value) {
    auto* e = find(key);
    if (!e || e->readonly() || !e->setter) return false;
    return e->setter(value);
}

std::string ConfigRegistry::get(std::string_view key) const {
    auto* e = find(key);
    if (!e || !e->getter) return {};
    auto v = e->getter();
    if (e->sensitive()) {
        return v.empty() ? std::string{"(unset)"} : std::string{"***"};
    }
    return v;
}

void ConfigRegistry::iter(std::function<void(const ConfigEntry&)> cb) const {
    for (auto& e : entries_) cb(e);
}

} // namespace culfw32
