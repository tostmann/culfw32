// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "esp_err.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include <atomic>
#include <cstdint>
#include <string>

namespace culfw32 {

class ConfigRegistry;

// SystemMonitor — Reset-Reason-Persistenz, Crash-Counter, Uptime-Total.
//
// Persistiert in NVS-Namespace `cul32_sys`:
//   boot_count        u32  — gesamte Boots seit Erst-Boot
//   crash_count       u32  — Boots, die durch PANIC/WDT/BROWNOUT/CPU_LOCKUP
//                            getriggert wurden
//   uptime_total_s    u32  — kumulierte Uptime ueber alle Boots
//   last_reset        u8   — esp_reset_reason_t des aktuellen Boots
//   last_uptime_s     u32  — Session-Uptime des letzten Boots vor Reset
//
// Schreibstrategie: einmalig beim Boot (nach init/Reason-Klassifizierung) +
// alle 5 min in einem Background-Timer (uptime_total_s + last_uptime_s
// werden inkrementell weitergeschrieben). Damit verlieren wir bei einem
// ungraceful-Crash hoechstens ~5 min an `uptime_total_s` — fuer den Zweck
// (Stabilitaets-Trend) reicht das.
//
// Heap-Lowwater-Alarm: warning-Log wenn esp_get_minimum_free_heap_size()
// unter Threshold (~8 KB) faellt. Singleton-Hook im Periodic-Tick.
class SystemMonitor {
public:
    static SystemMonitor& instance();

    esp_err_t init();
    void      register_with_registry(ConfigRegistry& cfg);

    // Aktuelle Session-Uptime in Sekunden.
    uint32_t  session_uptime_s() const;
    uint32_t  uptime_total_s() const;
    uint32_t  boot_count() const   { return boot_count_; }
    uint32_t  crash_count() const  { return crash_count_; }
    esp_reset_reason_t last_reset() const { return last_reset_; }

    // "POR" / "WDT" / "PANIC" / "SW" / "BROWNOUT" — kurz, fuer UI/Header.
    static const char* reset_reason_short(esp_reset_reason_t r);
    // Volle Beschreibung — fuer Tooltip/JSON-Frontend.
    static const char* reset_reason_long(esp_reset_reason_t r);
    // True wenn die Reason auf einen ungraceful-Reset schliessen laesst
    // (PANIC, WDT, BROWNOUT, CPU_LOCKUP, PWR_GLITCH).
    static bool        reset_reason_is_crash(esp_reset_reason_t r);

    // Format-Helper fuer Web-UI / JSON-Frontend.
    // 0..59 s            -> "Ns"
    // 1 m .. 1 h         -> "Mm Ss"
    // 1 h .. 24 h        -> "Hh Mm"
    // > 24 h             -> "Dd Hh Mm"
    static std::string  format_uptime(uint32_t seconds);

    // Stack-Highwater pro Task: liefert Anzahl Tasks + fuellt buffer mit
    // mehrzeiligem ASCII-Output (eine Zeile pro Task: "name | stack-min B").
    // Wird vom ASCII-`?stats`-Pfad aufgerufen.
    size_t format_task_stats(char* out, size_t out_size) const;

    // Die-Temperatur des ESP32-C6 (driver/temperature_sensor.h) in Zehntel
    // Grad Celsius. Gemessen wird der Chip, nicht die Umgebung: WLAN, Funk und
    // CPU-Last heizen mit. Abgetastet alle kTempPeriodUs (system_monitor.cpp),
    // Min/Max seit Boot. valid == false, solange keine Messung gelang.
    struct ChipTemp {
        bool    valid  = false;
        int16_t now_dc = 0;
        int16_t min_dc = 0;
        int16_t max_dc = 0;
    };
    ChipTemp chip_temp() const;
    // Zehntelgrad als Text: 413 -> "41.3", -35 -> "-3.5".
    static std::string format_dc(int16_t dc);

private:
    SystemMonitor() = default;
    SystemMonitor(const SystemMonitor&) = delete;
    SystemMonitor& operator=(const SystemMonitor&) = delete;

    static void on_periodic_tick(void* arg);
    static void on_temp_tick(void* arg);
    void        init_temp();
    void        sample_temp();
    void        flush_to_nvs();

    void*       handle_ = nullptr;          // nvs_handle_t opaque
    bool        initialized_ = false;

    int64_t     boot_us_ = 0;                // esp_timer_get_time() at init

    uint32_t    boot_count_      = 0;
    uint32_t    crash_count_     = 0;
    uint32_t    uptime_total_base_s_ = 0;    // Total bis Boot-Start
    uint32_t    last_uptime_s_   = 0;        // Last-session Uptime
    esp_reset_reason_t last_reset_ = ESP_RST_UNKNOWN;

    // Heap-Lowwater-Alarm-Tracking.
    uint32_t    heap_alarm_threshold_ = 8192;   // 8 KB
    bool        heap_alarm_fired_     = false;

    void*       periodic_timer_ = nullptr;   // esp_timer_handle_t opaque

    // Chip-Temperatur: ein Schreiber (esp_timer-Task), Leser aus beliebigen
    // Tasks -- der Spinlock haelt now/min/max zusammen.
    void*       tsens_      = nullptr;       // temperature_sensor_handle_t opaque
    void*       temp_timer_ = nullptr;       // esp_timer_handle_t opaque
    mutable portMUX_TYPE temp_mux_ = portMUX_INITIALIZER_UNLOCKED;
    ChipTemp    temp_{};
};

} // namespace culfw32
