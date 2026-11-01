// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "rf_protocol.hpp"
#include "cc1101_driver.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <memory>

namespace culfw32 {

// Slot-Konfig = Driver-Konfig (Pinning + SPI-Host + Clock).
// Multi-CC v1.x reicht das hier durch.
using CC1101SlotConfig = CC1101Driver::Config;

enum class SlotState : uint8_t {
    Idle,
    Reconfiguring,
    Active,
    Error,
};

class CC1101Slot {
public:
    // Sperre fuer FOLGEN von Chip-Zugriffen — eine Ebene ueber dem Bus-Mutex
    // des Treibers, der nur die einzelne SPI-Transaktion serialisiert.
    //
    // WARUM: `dispatch_line()` laeuft im RX-Task JEDES Transports (TCP-Pool mit
    // acht Workern, USB-CDC, UART, alle Prioritaet 5). Zwei Klienten koennen
    // also gleichzeitig Kommandos schicken. Ohne diese Sperre reicht ein `Zr`
    // waehrend eines laufenden `As`: die Sendefunktion prueft ihr Flag nur am
    // Eintritt und strobt danach bis zu 1,5 s CCA plus Burst weiter — auf einem
    // Chip, den `activate()` inzwischen auf eine andere Klasse umgeschrieben
    // hat. Zwei gleichzeitige `activate()` verschraenken sogar stop()/SRES/
    // start() und hinterlassen zwei Klassen mit gesetztem Flag auf einem Chip.
    // Der Header behauptete bis 0.1.853 eine "Single-Task-Domaene"; das war
    // schon mit einem zweiten Klienten falsch.
    //
    // NICHT genommen wird sie in den RX-Tasks: deren Rueckruf kann ueber TCP
    // mit portMAX_DELAY blockieren, und eine dort gehaltene Sperre wuerde
    // `activate()` unbegrenzt aufhalten. Die RX-Seite bleibt beim Flag-Gating.
    // Einzige Ausnahme seit 0.2.3: die Selbstquittung des HMUARTLGW-Frontends
    // sendet aus dem AskSin-RX-Task ueber RfManager::tx, mit kurzer Frist und
    // nur fuer die Dauer der Sendung, nie ueber einen Transport-Aufruf hinweg.
    //
    // Rekursiv, damit ein spaeterer Pfad, der unter der Sperre erneut eine
    // Slot-Operation ruft, nicht gegen sich selbst laeuft.
    class Lock {
    public:
        Lock(SemaphoreHandle_t h, uint32_t timeout_ms)
            : h_(h),
              held_(h == nullptr ? true
                                 : xSemaphoreTakeRecursive(h, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {}
        ~Lock() { if (h_ && held_) xSemaphoreGiveRecursive(h_); }
        Lock(const Lock&)            = delete;
        Lock& operator=(const Lock&) = delete;
        // false = Zeit abgelaufen. Der Aufrufer bricht dann ab, statt den Chip
        // unter fremder Hand anzufassen.
        bool held() const { return held_; }
    private:
        SemaphoreHandle_t h_;
        bool              held_;
    };

    // Laenger als der laengste Sendevorgang (CCA bis 1,5 s + Burst 360 ms +
    // Poll 250 ms), damit ein normales `As` waehrend `Zr` nicht in den
    // Zeitablauf laeuft, aber kurz genug, dass ein haengender Sender das
    // Frontend nicht dauerhaft blockiert.
    static constexpr uint32_t kOpTimeoutMs = 3000;

    Lock lock(uint32_t timeout_ms = kOpTimeoutMs) { return Lock(op_mtx_, timeout_ms); }

    explicit CC1101Slot(uint8_t slot_id, const CC1101SlotConfig& cfg);
    ~CC1101Slot();

    uint8_t   id() const { return slot_id_; }
    SlotState state() const { return state_; }

    // Initialisiert den CC1101-Driver (SPI + Manual Reset). Einmalig vom Manager
    // beim Boot gerufen, BEVOR irgendeine RF-Klasse aktiviert wird.
    esp_err_t init();

    CC1101Driver& driver() { return drv_; }

    esp_err_t activate(std::shared_ptr<RfProtocol> proto);

    std::shared_ptr<RfProtocol> active() const { return active_; }

private:
    uint8_t                     slot_id_;
    CC1101SlotConfig            cfg_;
    CC1101Driver                drv_;
    SlotState                   state_ = SlotState::Idle;
    std::shared_ptr<RfProtocol> active_;
    SemaphoreHandle_t           op_mtx_ = nullptr;
};

} // namespace culfw32
