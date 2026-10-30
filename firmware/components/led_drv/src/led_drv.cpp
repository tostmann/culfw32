// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "led_drv.hpp"
#include "driver/gpio.h"
#include "esp_log.h"

namespace culfw32 {

namespace {
constexpr const char* TAG = "led";

// Pattern-Definitionen: Liste von (on, period_us)-Schritten. Timer feuert,
// schaltet auf Schritt-Wert, advance ringfoermig.
struct PatternStep { bool on; uint32_t us; };

constexpr PatternStep kPattern2[] = {{true, 500'000}, {false, 500'000}};
constexpr PatternStep kPattern3[] = {{true, 100'000}, {false, 900'000}};
constexpr PatternStep kPattern4[] = {{true, 80'000}, {false, 120'000},
                                     {true, 80'000}, {false, 720'000}};

const PatternStep* pattern_for(uint8_t mode, size_t& n) {
    switch (mode) {
        case 2: n = sizeof(kPattern2)/sizeof(kPattern2[0]); return kPattern2;
        case 3: n = sizeof(kPattern3)/sizeof(kPattern3[0]); return kPattern3;
        case 4: n = sizeof(kPattern4)/sizeof(kPattern4[0]); return kPattern4;
    }
    n = 0;
    return nullptr;
}
} // namespace

esp_err_t LedDriver::init(int gpio, bool active_low) {
    gpio_       = gpio;
    active_low_ = active_low;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << gpio_,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (auto e = gpio_config(&io); e != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(%d) failed: %d", gpio_, e);
        return e;
    }
    physical_set_(false);   // start OFF

    esp_timer_create_args_t targs = {
        .callback        = &LedDriver::timer_cb,
        .arg             = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name            = "led-pat",
        .skip_unhandled_events = true,
    };
    if (auto e = esp_timer_create(&targs, &timer_); e != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create failed: %d", e);
        return e;
    }
    return ESP_OK;
}

void LedDriver::set_mode(uint8_t mode) {
    mode_ = mode;
    step_ = 0;
    if (timer_) esp_timer_stop(timer_);

    if (mode == 0)      { physical_set_(false); return; }
    if (mode == 1)      { physical_set_(true);  return; }
    apply_step_();
}

void LedDriver::physical_set_(bool on) {
    if (gpio_ < 0) return;
    int level = active_low_ ? (on ? 0 : 1) : (on ? 1 : 0);
    gpio_set_level(static_cast<gpio_num_t>(gpio_), level);
}

void LedDriver::apply_step_() {
    size_t n = 0;
    const PatternStep* pat = pattern_for(mode_, n);
    if (!pat || n == 0) return;
    if (step_ >= n) step_ = 0;
    physical_set_(pat[step_].on);
    if (timer_) esp_timer_start_once(timer_, pat[step_].us);
    step_ = static_cast<uint8_t>((step_ + 1) % n);
}

void LedDriver::timer_cb(void* arg) {
    auto* self = static_cast<LedDriver*>(arg);
    self->apply_step_();
}

} // namespace culfw32
