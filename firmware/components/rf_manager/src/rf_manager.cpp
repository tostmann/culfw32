// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "rf_manager.hpp"
#include "esp_log.h"
#include "rom/ets_sys.h"   // ets_delay_us
#include <unordered_map>

namespace culfw32 {

namespace { constexpr const char* TAG = "rf-mgr"; }

// CC1101Slot --------------------------------------------------------------

CC1101Slot::CC1101Slot(uint8_t slot_id, const CC1101SlotConfig& cfg)
    : slot_id_(slot_id), cfg_(cfg) {
    op_mtx_ = xSemaphoreCreateRecursiveMutex();
    // Schlaegt die Allokation fehl, laeuft der Slot wie vor 0.1.854 — ungesperrt.
    // Das ist schlechter, aber besser als ein Slot, der gar nicht anlaeuft; die
    // Zeile sagt laut, dass die Sperre fehlt.
    if (!op_mtx_) ESP_LOGE(TAG, "slot %u: kein Op-Mutex — Zugriffe ungesperrt", slot_id);
}

CC1101Slot::~CC1101Slot() {
    if (op_mtx_) vSemaphoreDelete(op_mtx_);
}

esp_err_t CC1101Slot::init() {
    return drv_.init(cfg_);
}

esp_err_t CC1101Slot::activate(std::shared_ptr<RfProtocol> proto) {
    // Umkonfiguration ist eine FOLGE von Zugriffen (stop, IRQ-Detach, SRES,
    // start). Der Bus-Mutex des Treibers schuetzt nur die einzelne Transaktion
    // und traegt hier nichts.
    Lock lk(op_mtx_, kOpTimeoutMs);
    if (!lk.held()) {
        ESP_LOGE(TAG, "slot %u: Umkonfiguration abgebrochen — Slot seit %lu ms belegt",
                 slot_id_, static_cast<unsigned long>(kOpTimeoutMs));
        return ESP_ERR_TIMEOUT;
    }
    state_ = SlotState::Reconfiguring;
    // Laeuft gerade ein Frequenz-Suchlauf, ist seine Voraussetzung mit diesem
    // Wechsel weg (er zaehlt dekodierte Rahmen der AKTIVEN Klasse) — und sein
    // gesweepter Offset wuerde die neue CFG verstimmen.  Abbrechen.
    drv_.request_scan_abort();
    if (active_) {
        active_->stop();
        active_.reset();
    }
    // GDO0/GDO2 IRQ-Handler der vorigen RF-Klasse entfernen — bevor wir SRES
    // strobben oder die neue Klasse anlaufen lassen. Ohne diesen Detach
    // koennten Sync-Edges auf GDO2/GDO0 (waehrend SRES oder waehrend die
    // neue Klasse den Pin als reines GPIO nutzt, z.B. wMBus polling) ISRs
    // der vorigen Klasse aufrufen, die ihr `this` durch stop() bereits
    // ueber drv_=nullptr fragmentiert hat → vTaskNotify auf stale state
    // → race + crash.  Verifiziert 2026-05-12 Build 0.1.759: ohne diesen
    // Detach crashte HmDual→wMBus deterministisch beim `brt`/`brs`-Switch.
    drv_.detach_gdo_irqs();
    if (proto) {
        // Software-Reset (SRES) bevor die naechste RF-Klasse ihre CFG-Tabelle
        // schreibt. Notwendig weil RF-Klassen typischerweise nur ihre eigenen
        // ASKSIN_CFG/MORITZ_CFG/...-Eintraege schreiben, aber Reg-State der
        // vorigen Klasse stehen lassen — z.B. SignalDuino setzt PKTCTRL0=0x32
        // (OOK Async Direct-Mode), AGCCTRL1/0 auf OOK-Werte; AskSin's CFG
        // ueberschreibt diese nicht, Frame-TX bleibt dann in MARC_TX hangen
        // (verifiziert 2026-04-30 Build 0.1.94: AERR313 + ?stats `tx 0/3`).
        // a-culfw rf_*_init() macht denselben SRES vor jeder Cfg-Load.
        constexpr uint8_t SRES = 0x30;
        drv_.strobe(SRES);
        ets_delay_us(100);

        if (auto err = proto->start(slot_id_, drv_); err != ESP_OK) {
            state_ = SlotState::Error;
            return err;
        }
        active_ = std::move(proto);
    }
    state_ = active_ ? SlotState::Active : SlotState::Idle;
    ESP_LOGI(TAG, "slot %u: %s", slot_id_, active_ ? "active" : "idle");
    return ESP_OK;
}

// RfManager ---------------------------------------------------------------

void RfManager::add_slot(std::shared_ptr<CC1101Slot> slot) {
    slots_.push_back(std::move(slot));
}

void RfManager::register_protocol(std::shared_ptr<RfProtocol> proto) {
    protocols_.push_back(std::move(proto));
}

std::shared_ptr<RfProtocol> RfManager::find_protocol(ProtocolId proto) const {
    for (const auto& p : protocols_) {
        if (p->id() == proto) return p;
    }
    return nullptr;
}

std::shared_ptr<RfProtocol> RfManager::active_protocol_on(uint8_t slot_id) const {
    if (slot_id >= slots_.size()) return nullptr;
    return slots_[slot_id]->active();
}

esp_err_t RfManager::activate(ProtocolId proto, std::optional<uint8_t> slot_id) {
    auto p = find_protocol(proto);
    if (!p) return ESP_ERR_NOT_FOUND;

    // RX-Pfad ans interne Dispatch haengen — egal ob das Frontend schon subscribed
    // hat oder erst spaeter. Idempotent (on_rx ueberschreibt vorherigen Callback).
    p->on_rx([this](const RxEvent& ev) { on_proto_rx_(ev); });

    auto slot_idx = slot_id.value_or(0);
    if (slot_idx >= slots_.size()) return ESP_ERR_INVALID_ARG;
    return slots_[slot_idx]->activate(std::move(p));
}

esp_err_t RfManager::deactivate(std::optional<uint8_t> slot_id) {
    auto slot_idx = slot_id.value_or(0);
    if (slot_idx >= slots_.size()) return ESP_ERR_INVALID_ARG;
    // activate(nullptr) IST der Teardown — samt des IRQ-Detach, den es seit
    // 0.1.759 gibt.  Einen zweiten Abrisspfad danebenzustellen hiesse, diese
    // Reihenfolge ein zweites Mal richtig zu treffen.
    return slots_[slot_idx]->activate(nullptr);
}

void RfManager::subscribe(ProtocolId proto, RxCallback cb) {
    subscribers_.emplace_back(proto, std::move(cb));
}

void RfManager::on_proto_rx_(const RxEvent& ev) {
    for (const auto& [pid, cb] : subscribers_) {
        if (pid == ev.proto) cb(ev);
    }
}

esp_err_t RfManager::tx(ProtocolId proto, const void* typed_payload, size_t size,
                        uint32_t lock_timeout_ms) {
    // Aktive Slot-Klasse pruefen — wenn die Multi-Proto-TX kann (HmDualRF etc.),
    // delegiert sie selbst. Sonst Standard-Pfad: find_protocol -> tx.
    //
    // Die Sperre des BESITZENDEN Slots wird ueber den ganzen Sendevorgang
    // gehalten: er dauert bis zu 1,5 s CCA plus Burst, und die Sendefunktionen
    // pruefen ihr Flag nur am Eintritt. Ohne die Sperre koennte ein `Zr` aus
    // einem zweiten Transport den Chip mittendrin umschreiben, und der Sender
    // fuehre mit fremdem Modem weiter.
    for (auto& slot : slots_) {
        auto active = slot->active();
        if (active && active->supports_tx(proto)) {
            auto lk = slot->lock(lock_timeout_ms);
            if (!lk.held()) {
                // Mit verkuerzter Frist ist Ablauf ein erwarteter Ausgang, den
                // der Aufrufer selbst meldet (HMU-Selbstquittung).
                if (lock_timeout_ms >= CC1101Slot::kOpTimeoutMs) {
                    ESP_LOGE(TAG, "tx: Slot %u belegt, Sendung verworfen", slot->id());
                }
                return ESP_ERR_TIMEOUT;
            }
            // Nach dem Warten neu lesen: der Vorbesitzer der Sperre kann den
            // Slot umkonfiguriert haben.
            auto now = slot->active();
            if (!now || !now->supports_tx(proto)) return ESP_ERR_INVALID_STATE;
            return now->tx_for(proto, typed_payload, size);
        }
    }
    auto p = find_protocol(proto);
    if (!p) return ESP_ERR_NOT_FOUND;
    // Kein Slot traegt dieses Protokoll aktiv — die Sendefunktion weist es ueber
    // ihr eigenes Flag ab. Nichts zu sperren.
    return p->tx(typed_payload, size);
}

// CC1101-Diagnose-Fassade ----------------------------------------------------

esp_err_t RfManager::cc_read_reg(uint8_t slot_id, uint8_t addr, uint8_t& value) {
    if (slot_id >= slots_.size()) return ESP_ERR_NOT_FOUND;
    return slots_[slot_id]->driver().read_reg(addr, value);
}

esp_err_t RfManager::cc_pll_counters(uint8_t slot_id, uint32_t& lost, uint32_t& relocks, uint32_t& failures) {
    if (slot_id >= slots_.size()) return ESP_ERR_NOT_FOUND;
    auto& d  = slots_[slot_id]->driver();
    lost     = d.pll_lost();
    relocks  = d.pll_relocks();
    failures = d.pll_failures();
    return ESP_OK;
}

esp_err_t RfManager::cc_rx_counters(uint8_t slot_id, uint32_t& extra, uint32_t& stale, uint32_t& resync) {
    if (slot_id >= slots_.size()) return ESP_ERR_NOT_FOUND;
    auto& d = slots_[slot_id]->driver();
    extra   = d.rx_extra();
    stale   = d.rx_stale();
    resync  = d.rx_resync();
    return ESP_OK;
}

esp_err_t RfManager::cc_read_status(uint8_t slot_id, uint8_t addr, uint8_t& value) {
    if (slot_id >= slots_.size()) return ESP_ERR_NOT_FOUND;
    return slots_[slot_id]->driver().read_status(addr, value);
}

esp_err_t RfManager::cc_read_burst(uint8_t slot_id, uint8_t addr,
                                   std::span<uint8_t> data) {
    if (slot_id >= slots_.size()) return ESP_ERR_NOT_FOUND;
    return slots_[slot_id]->driver().read_burst(addr, data);
}

esp_err_t RfManager::cc_write_reg(uint8_t slot_id, uint8_t addr, uint8_t value) {
    if (slot_id >= slots_.size()) return ESP_ERR_NOT_FOUND;
    // Bewusst raw: `Cw` ist der Handpfad fuer Messreihen — auch auf FSCTRL0,
    // das der Driver sonst mit dem Board-Kalibrierwert ueberschreibt. Der
    // naechste Mode-Wechsel laedt die CFG-Tabelle neu und stellt den
    // kalibrierten Wert wieder her.
    return slots_[slot_id]->driver().write_reg_raw(addr, value);
}

esp_err_t RfManager::cc_write_burst(uint8_t slot_id, uint8_t addr,
                                    std::span<const uint8_t> data) {
    if (slot_id >= slots_.size()) return ESP_ERR_NOT_FOUND;
    return slots_[slot_id]->driver().write_burst(addr, data);
}

} // namespace culfw32
