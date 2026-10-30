// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "system_monitor.hpp"
#include "config_registry.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/temperature_sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG       = "sysmon";
constexpr const char* NS        = "cul32_sys";
constexpr uint32_t    kFlushPeriodS = 5 * 60;     // alle 5 min
constexpr uint64_t    kPeriodicUs   = 60ULL * 1000ULL * 1000ULL;  // 60 s
constexpr uint64_t    kTempPeriodUs = 5ULL * 1000ULL * 1000ULL;   // 5 s
}

SystemMonitor& SystemMonitor::instance() {
    static SystemMonitor s;
    return s;
}

const char* SystemMonitor::reset_reason_short(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "POR";
        case ESP_RST_EXT:       return "EXT";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEP";
        case ESP_RST_BROWNOUT:  return "BROWN";
        case ESP_RST_SDIO:      return "SDIO";
        case ESP_RST_USB:       return "USB";
        case ESP_RST_JTAG:      return "JTAG";
        case ESP_RST_EFUSE:     return "EFUSE";
        case ESP_RST_PWR_GLITCH:return "GLITCH";
        case ESP_RST_CPU_LOCKUP:return "LOCK";
        case ESP_RST_UNKNOWN:
        default:                return "UNK";
    }
}

const char* SystemMonitor::reset_reason_long(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "Power-On Reset";
        case ESP_RST_EXT:       return "External Pin Reset";
        case ESP_RST_SW:        return "Software Reset (geplant)";
        case ESP_RST_PANIC:     return "Panic / Exception";
        case ESP_RST_INT_WDT:   return "Interrupt Watchdog";
        case ESP_RST_TASK_WDT:  return "Task Watchdog";
        case ESP_RST_WDT:       return "Other Watchdog";
        case ESP_RST_DEEPSLEEP: return "Deep-Sleep Wakeup";
        case ESP_RST_BROWNOUT:  return "Brownout (Spannungsabfall)";
        case ESP_RST_SDIO:      return "SDIO Reset";
        case ESP_RST_USB:       return "USB Peripheral Reset";
        case ESP_RST_JTAG:      return "JTAG Reset";
        case ESP_RST_EFUSE:     return "eFuse Error";
        case ESP_RST_PWR_GLITCH:return "Power Glitch";
        case ESP_RST_CPU_LOCKUP:return "CPU Lockup";
        case ESP_RST_UNKNOWN:
        default:                return "Unknown";
    }
}

bool SystemMonitor::reset_reason_is_crash(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_PANIC:
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:
        case ESP_RST_BROWNOUT:
        case ESP_RST_PWR_GLITCH:
        case ESP_RST_CPU_LOCKUP:
            return true;
        default:
            return false;
    }
}

std::string SystemMonitor::format_uptime(uint32_t seconds) {
    char buf[48];
    uint32_t d = seconds / 86400;
    uint32_t h = (seconds % 86400) / 3600;
    uint32_t m = (seconds % 3600)  / 60;
    uint32_t s = seconds % 60;
    if (d > 0)        std::snprintf(buf, sizeof(buf), "%ud %uh %um",
                                    static_cast<unsigned>(d),
                                    static_cast<unsigned>(h),
                                    static_cast<unsigned>(m));
    else if (h > 0)   std::snprintf(buf, sizeof(buf), "%uh %um",
                                    static_cast<unsigned>(h),
                                    static_cast<unsigned>(m));
    else if (m > 0)   std::snprintf(buf, sizeof(buf), "%um %us",
                                    static_cast<unsigned>(m),
                                    static_cast<unsigned>(s));
    else              std::snprintf(buf, sizeof(buf), "%us",
                                    static_cast<unsigned>(s));
    return std::string(buf);
}

esp_err_t SystemMonitor::init() {
    if (initialized_) return ESP_OK;

    // Temperatur unabhaengig von NVS: auch ohne Namespace messbar.
    init_temp();

    nvs_handle_t h = 0;
    auto err = nvs_open(NS, NVS_READWRITE, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // Erstboot — Namespace noch leer. Wir setzen alle Werte unten neu.
        ESP_LOGI(TAG, "First boot — initializing %s namespace", NS);
        err = nvs_open(NS, NVS_READWRITE, &h);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(%s) failed: %s", NS, esp_err_to_name(err));
        return err;
    }
    handle_ = reinterpret_cast<void*>(static_cast<uintptr_t>(h));

    // 1) Werte aus NVS lesen — alle optional (Default 0).
    nvs_get_u32(h, "boot_count",   &boot_count_);
    nvs_get_u32(h, "crash_count",  &crash_count_);
    nvs_get_u32(h, "uptime_total", &uptime_total_base_s_);
    nvs_get_u32(h, "last_uptime",  &last_uptime_s_);

    uint8_t prev_reason_raw = 0;
    nvs_get_u8(h, "last_reset", &prev_reason_raw);
    // prev_reason_raw ist die Reason des LETZTEN Boots (vor diesem) — wir
    // ueberschreiben sie unten mit der aktuellen Reason. Fuer ConfigRegistry
    // exposen wir die Reason des AKTUELLEN Boots (last_reset_).

    // 2) Reason des aktuellen Boots ermitteln + Counter aktualisieren.
    last_reset_ = esp_reset_reason();
    boot_count_ += 1;
    if (reset_reason_is_crash(last_reset_)) {
        crash_count_ += 1;
        ESP_LOGW(TAG, "Crash-type reset detected: %s (count=%u)",
                 reset_reason_short(last_reset_),
                 static_cast<unsigned>(crash_count_));
    } else {
        ESP_LOGI(TAG, "Boot reason: %s", reset_reason_short(last_reset_));
    }

    // 3) Persistieren.
    nvs_set_u32(h, "boot_count",   boot_count_);
    nvs_set_u32(h, "crash_count",  crash_count_);
    nvs_set_u8 (h, "last_reset",   static_cast<uint8_t>(last_reset_));
    // uptime_total / last_uptime werden im periodischen Tick und beim
    // graceful-Shutdown geschrieben. Hier nur den initialen Stand sichern.
    nvs_commit(h);

    boot_us_ = esp_timer_get_time();
    initialized_ = true;

    // 4) Periodischen Tick alle 60 s starten.
    esp_timer_create_args_t cfg = {};
    cfg.callback = &SystemMonitor::on_periodic_tick;
    cfg.arg      = this;
    cfg.dispatch_method = ESP_TIMER_TASK;
    cfg.name     = "sysmon";
    esp_timer_handle_t t = nullptr;
    if (auto e = esp_timer_create(&cfg, &t); e == ESP_OK) {
        esp_timer_start_periodic(t, kPeriodicUs);
        periodic_timer_ = t;
    } else {
        ESP_LOGW(TAG, "esp_timer_create failed: %s", esp_err_to_name(e));
    }

    ESP_LOGI(TAG, "Init done: boot=%u crash=%u uptime_total=%us last=%s",
             static_cast<unsigned>(boot_count_),
             static_cast<unsigned>(crash_count_),
             static_cast<unsigned>(uptime_total_base_s_),
             reset_reason_short(last_reset_));
    return ESP_OK;
}

void SystemMonitor::init_temp() {
    if (tsens_) return;
    // -10..80 C ist laut IDF-Tabelle der genaueste Bereich (Fehler <= 1 C,
    // soc/esp32c6/temperature_sensor_periph.c). Verlaesst die Messung ihn,
    // schaltet der Treiber selbst um (temp_sensor_get_raw_value in
    // esp_hw_support/sar_periph_ctrl_common.c); die Genauigkeit sinkt dann
    // laut derselben Tabelle auf 2..3 C.
    temperature_sensor_config_t tcfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    temperature_sensor_handle_t h = nullptr;
    esp_err_t e = temperature_sensor_install(&tcfg, &h);
    if (e == ESP_OK) {
        e = temperature_sensor_enable(h);
        if (e != ESP_OK) temperature_sensor_uninstall(h);
    }
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "temperature sensor: %s", esp_err_to_name(e));
        return;
    }
    tsens_ = h;
    sample_temp();

    esp_timer_create_args_t a = {};
    a.callback        = &SystemMonitor::on_temp_tick;
    a.arg             = this;
    a.dispatch_method = ESP_TIMER_TASK;
    a.name            = "tsens";
    esp_timer_handle_t t = nullptr;
    if (esp_timer_create(&a, &t) == ESP_OK) {
        esp_timer_start_periodic(t, kTempPeriodUs);
        temp_timer_ = t;
    } else {
        // Ohne Timer bliebe der Bootwert als "aktuell" stehen -- lieber n/a.
        ESP_LOGW(TAG, "temperature timer not created, reporting n/a");
        portENTER_CRITICAL(&temp_mux_);
        temp_.valid = false;
        portEXIT_CRITICAL(&temp_mux_);
    }
}

void SystemMonitor::on_temp_tick(void* arg) {
    static_cast<SystemMonitor*>(arg)->sample_temp();
}

void SystemMonitor::sample_temp() {
    if (!tsens_) return;
    float c = 0.0f;
    if (temperature_sensor_get_celsius(static_cast<temperature_sensor_handle_t>(tsens_), &c)
            != ESP_OK) {
        return;
    }
    const auto dc = static_cast<int16_t>(std::lround(c * 10.0f));
    portENTER_CRITICAL(&temp_mux_);
    if (!temp_.valid) {
        temp_.min_dc = dc;
        temp_.max_dc = dc;
        temp_.valid  = true;
    }
    temp_.now_dc = dc;
    if (dc < temp_.min_dc) temp_.min_dc = dc;
    if (dc > temp_.max_dc) temp_.max_dc = dc;
    portEXIT_CRITICAL(&temp_mux_);
}

SystemMonitor::ChipTemp SystemMonitor::chip_temp() const {
    portENTER_CRITICAL(&temp_mux_);
    const ChipTemp t = temp_;
    portEXIT_CRITICAL(&temp_mux_);
    return t;
}

std::string SystemMonitor::format_dc(int16_t dc) {
    char b[12];
    const int a = std::abs(static_cast<int>(dc));
    std::snprintf(b, sizeof(b), "%s%d.%d", dc < 0 ? "-" : "", a / 10, a % 10);
    return std::string(b);
}

uint32_t SystemMonitor::session_uptime_s() const {
    if (!initialized_) return 0;
    int64_t us = esp_timer_get_time() - boot_us_;
    if (us < 0) us = 0;
    return static_cast<uint32_t>(us / 1000000);
}

uint32_t SystemMonitor::uptime_total_s() const {
    return uptime_total_base_s_ + session_uptime_s();
}

void SystemMonitor::on_periodic_tick(void* arg) {
    auto* self = static_cast<SystemMonitor*>(arg);

    // Heap-Lowwater-Alarm — feuert genau einmal pro Boot.
    uint32_t hmin = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    if (!self->heap_alarm_fired_ && hmin < self->heap_alarm_threshold_) {
        ESP_LOGW(TAG, "Heap-Lowwater-Alarm: minimum free heap dropped to %u B "
                       "(threshold %u B)",
                 static_cast<unsigned>(hmin),
                 static_cast<unsigned>(self->heap_alarm_threshold_));
        self->heap_alarm_fired_ = true;
    }

    // Alle kFlushPeriodS Sekunden die NVS-Persistenz refreshen — damit
    // bei einem ungraceful-Crash hoechstens diese Periode an
    // uptime_total_s/last_uptime_s verloren geht.
    static uint32_t s_tick_count = 0;
    s_tick_count += 1;
    if ((s_tick_count * 60u) % kFlushPeriodS == 0) {
        self->flush_to_nvs();
    }
}

void SystemMonitor::flush_to_nvs() {
    if (!handle_) return;
    auto h = static_cast<nvs_handle_t>(reinterpret_cast<uintptr_t>(handle_));
    uint32_t total = uptime_total_s();
    uint32_t sess  = session_uptime_s();
    nvs_set_u32(h, "uptime_total", total);
    nvs_set_u32(h, "last_uptime",  sess);
    nvs_commit(h);
    last_uptime_s_       = sess;
    uptime_total_base_s_ = total - sess;
}

size_t SystemMonitor::format_task_stats(char* out, size_t out_size) const {
    if (!out || out_size == 0) return 0;
    UBaseType_t n = uxTaskGetNumberOfTasks();
    if (n == 0) { out[0] = '\0'; return 0; }

    // FreeRTOS-Trace-Allocation: dynamisch da n variabel.
    auto* arr = reinterpret_cast<TaskStatus_t*>(
        heap_caps_calloc(n, sizeof(TaskStatus_t), MALLOC_CAP_DEFAULT));
    if (!arr) { out[0] = '\0'; return 0; }

    UBaseType_t got = uxTaskGetSystemState(arr, n, nullptr);
    size_t pos = 0;
    pos += std::snprintf(out + pos, (pos < out_size ? out_size - pos : 0),
                         "task                stack-min(B) prio state\r\n");
    for (UBaseType_t i = 0; i < got && pos < out_size; ++i) {
        const char* st = "?";
        switch (arr[i].eCurrentState) {
            case eRunning:   st = "RUN";  break;
            case eReady:     st = "RDY";  break;
            case eBlocked:   st = "BLK";  break;
            case eSuspended: st = "SUS";  break;
            case eDeleted:   st = "DEL";  break;
            case eInvalid:   st = "INV";  break;
        }
        // High-Watermark ist in StackType_t-Worten — fuer ESP32(C6) = 4 Byte.
        uint32_t stack_min_b =
            static_cast<uint32_t>(arr[i].usStackHighWaterMark) * sizeof(StackType_t);
        pos += std::snprintf(out + pos, (pos < out_size ? out_size - pos : 0),
                             "%-19s %12u %4u %s\r\n",
                             arr[i].pcTaskName,
                             static_cast<unsigned>(stack_min_b),
                             static_cast<unsigned>(arr[i].uxCurrentPriority), st);
    }
    heap_caps_free(arr);
    return pos;
}

void SystemMonitor::register_with_registry(ConfigRegistry& cfg) {
    using T = ConfigEntry::Type;
    using E = ConfigEntry;

    // ConfigEntry hat Default-Initializer fuer alle Member; designated-init
    // mit weggelassenen Feldern ist hier bewusst — kein echter Verlust.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

    cfg.register_entry({
        .key = "sys.uptime", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return SystemMonitor::format_uptime(
                              SystemMonitor::instance().session_uptime_s()); },
        .help = "Uptime since the last boot (formatted)",
    });
    cfg.register_entry({
        .key = "sys.uptime_s", .type = T::U32, .flags = E::FLAG_READONLY,
        .getter = [] {
            char b[16];
            std::snprintf(b, sizeof(b), "%u", static_cast<unsigned>(
                          SystemMonitor::instance().session_uptime_s()));
            return std::string(b);
        },
        .help = "Uptime since the last boot in seconds", .unit = "s",
    });
    cfg.register_entry({
        .key = "sys.uptime_total", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return SystemMonitor::format_uptime(
                              SystemMonitor::instance().uptime_total_s()); },
        .help = "Accumulated uptime over all boots (persistent)",
    });
    cfg.register_entry({
        .key = "sys.boot_count", .type = T::U32, .flags = E::FLAG_READONLY,
        .getter = [] {
            char b[16];
            std::snprintf(b, sizeof(b), "%u", static_cast<unsigned>(
                          SystemMonitor::instance().boot_count()));
            return std::string(b);
        },
        .help = "Number of boots since the first boot",
    });
    cfg.register_entry({
        .key = "sys.crash_count", .type = T::U32, .flags = E::FLAG_READONLY,
        .getter = [] {
            char b[16];
            std::snprintf(b, sizeof(b), "%u", static_cast<unsigned>(
                          SystemMonitor::instance().crash_count()));
            return std::string(b);
        },
        .help = "Number of boots caused by a crash (PANIC/WDT/BROWN/LOCK/GLITCH)",
    });
    cfg.register_entry({
        .key = "sys.reset_reason", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return std::string(
            SystemMonitor::reset_reason_short(
                SystemMonitor::instance().last_reset())); },
        .help = "Reason of the last reset (POR/SW/PANIC/WDT/BROWN/...)",
    });
    cfg.register_entry({
        .key = "sys.reset_reason_text", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] { return std::string(
            SystemMonitor::reset_reason_long(
                SystemMonitor::instance().last_reset())); },
        .help = "Reset reason as plain text",
    });
    cfg.register_entry({
        .key = "sys.temp", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] {
            const auto t = SystemMonitor::instance().chip_temp();
            return t.valid ? SystemMonitor::format_dc(t.now_dc) : std::string("n/a");
        },
        .help = "ESP32-C6 die temperature (on-chip sensor, not ambient), sampled every 5 s",
        .unit = "C",
    });
    cfg.register_entry({
        .key = "sys.temp_min", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] {
            const auto t = SystemMonitor::instance().chip_temp();
            return t.valid ? SystemMonitor::format_dc(t.min_dc) : std::string("n/a");
        },
        .help = "Lowest die temperature since boot",
        .unit = "C",
    });
    cfg.register_entry({
        .key = "sys.temp_max", .type = T::Str, .flags = E::FLAG_READONLY,
        .getter = [] {
            const auto t = SystemMonitor::instance().chip_temp();
            return t.valid ? SystemMonitor::format_dc(t.max_dc) : std::string("n/a");
        },
        .help = "Highest die temperature since boot",
        .unit = "C",
    });

#pragma GCC diagnostic pop
}

} // namespace culfw32
