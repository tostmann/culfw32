// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "cc1101_slot.hpp"
#include "rf_protocol.hpp"
#include <vector>
#include <memory>
#include <optional>

namespace culfw32 {

// Schicht 5: Orchestrator.
// - Besitzt 1..n CC1101Slots.
// - Registriert RF-Klassen.
// - Vermittelt Frontend-Anfragen ("aktiviere AskSin auf Slot 0").
// - Garantiert culfw-Invariante: ein RF-Mode aktiv pro Slot zur Zeit.
class RfManager {
public:
    void add_slot(std::shared_ptr<CC1101Slot> slot);
    void register_protocol(std::shared_ptr<RfProtocol> proto);

    size_t slot_count() const { return slots_.size(); }
    bool   has_slot(uint8_t slot_id) const { return slot_id < slots_.size(); }

    // Frontend ruft dies, um eine RF-Klasse auf einem Slot zu aktivieren.
    // Wenn slot_id ungesetzt: erster Slot, der diese RF-Klasse vertraegt
    // (Default-Verhalten fuer Single-CC-Hardware).
    esp_err_t activate(ProtocolId proto, std::optional<uint8_t> slot_id = std::nullopt);

    // Slot leerraeumen: die laufende RF-Klasse stoppen und den Slot als LEER
    // vermerken.  Fuer die `*x`-Befehle ("Empfang aus") gedacht.
    //
    // WARUM DAS NOETIG IST: die Frontends riefen `proto->stop()` direkt.  Der
    // Slot hielt die Klasse danach weiter fuer aktiv, obwohl der Chip auf
    // SIDLE stand und `drv_` genullt war — `active_protocol_on()` log also.
    // Jeder, der sie als Wahrheit nahm, lag daneben: die `Ar`/`Pr`-Abkuerzung
    // ("schon aktiv, nichts zu tun") liess den Empfaenger tot zurueck, und
    // HMUARTLGW/JSON schalten auf derselben Grundlage nicht mehr zurueck.
    // Gemessen 07.09.2026: MARCSTATE blieb 0x01 (IDLE) nach `Ax`+`Ar`.
    esp_err_t deactivate(std::optional<uint8_t> slot_id = std::nullopt);

    // RX-Subscription: Frontend abonniert Events einer ProtocolId.
    // Manager fanout zu allen Subscribern (z.B. culfw-Emu UND Matter zugleich).
    using RxCallback = std::function<void(const RxEvent&)>;
    void subscribe(ProtocolId proto, RxCallback cb);

    // TX-Routing. lock_timeout_ms begrenzt das Warten auf die Slot-Sperre;
    // eine Quittung, die erst nach Hunderten ms raus koennte, ist wertlos.
    esp_err_t tx(ProtocolId proto, const void* typed_payload, size_t size,
                 uint32_t lock_timeout_ms = CC1101Slot::kOpTimeoutMs);

    // Lookup-Helfer fuer Frontends.
    std::shared_ptr<RfProtocol> find_protocol(ProtocolId proto) const;

    // Aktuell aktives Protokoll auf einem Slot. Nullptr wenn Slot nicht
    // existiert oder leer (Idle).
    std::shared_ptr<RfProtocol> active_protocol_on(uint8_t slot_id) const;

    // CC1101-Diagnose (fuer culfw-`C` und `Vr`-Befehl).
    // Frontend kennt `CC1101Driver` nicht direkt — Manager als Fassade.
    esp_err_t cc_read_reg   (uint8_t slot_id, uint8_t addr, uint8_t& value);
    esp_err_t cc_read_status(uint8_t slot_id, uint8_t addr, uint8_t& value);
    esp_err_t cc_read_burst (uint8_t slot_id, uint8_t addr, std::span<uint8_t> data);
    esp_err_t cc_write_reg  (uint8_t slot_id, uint8_t addr, uint8_t value);
    esp_err_t cc_write_burst(uint8_t slot_id, uint8_t addr, std::span<const uint8_t> data);

    // PLL-Telemetrie eines Slots (fuer `?stats`). lost = Heartbeat sah FSCAL1==0x3F
    // in RX (Lock-Verlust erkannt); relocks = Kalibrierung brauchte einen Retry;
    // failures = aufgegeben. lost != 0 heisst: der Chip war ohne Lock in RX — ohne
    // diese Zaehler bliebe die Rekalibrierung unsichtbar und verdeckte ihre Ursache.
    esp_err_t cc_pll_counters(uint8_t slot_id, uint32_t& lost, uint32_t& relocks, uint32_t& failures);
    // RX-FIFO-Zaehler des Slots (CC1101Driver::rx_extra/rx_stale/rx_resync).
    esp_err_t cc_rx_counters(uint8_t slot_id, uint32_t& extra, uint32_t& stale, uint32_t& resync);

private:
    void on_proto_rx_(const RxEvent& ev);    // interner Dispatcher

    std::vector<std::shared_ptr<CC1101Slot>>      slots_;
    std::vector<std::shared_ptr<RfProtocol>>      protocols_;
    // Subscriber pro ProtocolId. Subscribe passiert beim Boot, gelesen wird im
    // RX-Fanout des Protokoll-Tasks — deshalb hier keine Sperre.
    // ⚠️ Die frueher hier stehende Begruendung "Single-Task-Domaene" war FALSCH:
    // `dispatch_line()` laeuft im RX-Task jedes Transports (TCP-Pool mit acht
    // Workern, USB-CDC, UART), also rufen mehrere Tasks activate/deactivate/tx.
    // Dagegen schuetzt seit 0.1.854 die Slot-Sperre (`CC1101Slot::Lock`), nicht
    // die Annahme. Fuer die Subscriber-Liste bleibt sie richtig, weil sie nach
    // dem Boot nur noch gelesen wird.
    std::vector<std::pair<ProtocolId, RxCallback>> subscribers_;
};

} // namespace culfw32
