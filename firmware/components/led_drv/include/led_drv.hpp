// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "esp_err.h"
#include "esp_timer.h"
#include <cstdint>

namespace culfw32 {

// Einfache LED-Treiber-Komponente. Patterns analog a-culfw `ledfunc()`/XLED:
//   0 = OFF
//   1 = ON
//   2 = slow-blink (~1 Hz, 50 % duty)
//   3 = sparse blink (10 % duty, 1 Hz)
//   4 = fast double-blink (heartbeat)
//
// Pattern-Engine via esp_timer (1-shot, periodisch reschedule). Keine FreeRTOS-
// Task — esp_timer-Callbacks laufen im Timer-Service-Task, das ist fuer reines
// gpio_set_level reichlich.
class LedDriver {
public:
    esp_err_t init(int gpio, bool active_low);
    void      set_mode(uint8_t mode);
    uint8_t   mode() const { return mode_; }

private:
    static void timer_cb(void* arg);
    void        apply_step_();
    void        physical_set_(bool on);

    int           gpio_       = -1;
    bool          active_low_ = false;
    uint8_t       mode_       = 0;
    uint8_t       step_       = 0;     // Schritt im aktuellen Pattern
    esp_timer_handle_t timer_ = nullptr;
};

} // namespace culfw32
