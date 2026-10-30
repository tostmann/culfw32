// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "esp_err.h"
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace culfw32 {

// Online-Update aus install.busware.de (Verzeichnis /CUL/). Vorlage ist
// CDC2NET firmware/main/ota_check.c — dieselben Waechter, hier als Klasse:
//
//   * Manifest = das esp-web-tools-Manifest des Web-Flashers, erweitert um
//     `version` und `ota.<chipFamily>.path` (Schema wie cdc2net). Eine Quelle
//     fuer Erstinstallation und Update, keine zweite Liste, die abdriften kann.
//   * project_name des geladenen Images muss "culfw32" sein (IDF prueft nur
//     den Chip, nicht die Anwendung); die Chip-ID prueft IDF vor dem ersten
//     Schreiben.
//   * Das Image muss neuer sein als das laufende — schuetzt vor einer Schleife,
//     wenn Manifest und Binary beim Deploy kurz auseinanderlaufen.
//   * Download mit Wanduhr-Frist: perform() meldet IN_PROGRESS auch bei einem
//     Lese-Timeout, ein haengender Server liesse die Schleife sonst ewig laufen.
//
// Nach dem Neustart steht das neue Image auf PENDING_VERIFY (Rollback im
// Bootloader, CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE). start_validation() gibt
// es erst nach stabilem Betrieb frei und rollt sonst zurueck.
class OtaUpdate {
public:
    enum class State : uint8_t { Idle, Checking, UpToDate, Available, Installing, Error };

    // true = Netz nutzbar (eine Schnittstelle hat eine IPv4-Adresse).
    using NetFn = bool (*)();
    // true = Updates selbst installieren (ee::OTA_AUTO).
    using AutoFn = bool (*)();

    static OtaUpdate& instance();

    // Hintergrund-Task: erste Pruefung kurz nach dem Start, danach taeglich.
    esp_err_t start(NetFn net_up, AutoFn auto_install);

    // ?ota=check / ?ota=install — asynchron, Ergebnis ueber status_line().
    esp_err_t request_check();
    esp_err_t request_install();

    // Eine Zeile fuer ?ota, z.B. "available 0.1.870 (running 0.1.864)".
    void status_line(char* buf, size_t cap) const;

    // Dasselbe maschinenlesbar, fuer die Web-UI (?ota.state/.latest/.progress):
    // ein Wort aus idle|checking|uptodate|available|installing|error|validating,
    // die Version aus dem zuletzt gelesenen Manifest ("" = noch nie gelesen)
    // und der Fortschritt 0..100 waehrend der Installation, sonst -1.
    const char* state_word() const;
    void        latest_version(char* buf, size_t cap) const;
    int         progress() const;
    // true, solange ein Manifest-Abruf oder eine Installation laeuft (TLS).
    bool        busy() const;

    // ?ota.url — nur im RAM, fuer Staging-Tests; leer = Default aus Kconfig.
    void        set_manifest_url(const char* url);
    void        manifest_url(char* buf, size_t cap) const;

    // Rollback-Gate (Review M2, Dirk 08.10.): das Netz zaehlt nur, wenn es das
    // Image ueberhaupt beweisen KANN — die Verbindungsschicht steht (Ethernet
    // mit Link, WLAN assoziiert), aber keine Adresse kommt. Router aus, Kabel ab,
    // AP weg sind keine Image-Fehler.
    //   Ok        kein Netz konfiguriert oder eine Schnittstelle hat eine IP
    //   NoLink    Netz konfiguriert, Verbindungsschicht unten
    //   LinkNoIp  Verbindungsschicht oben, keine Adresse
    enum class NetHealth : uint8_t { Ok, NoLink, LinkNoIp };
    using HealthFn = NetHealth (*)();

    // Nach kMinRunS stabilen Sekunden: gueltig bei Ok oder wenn die
    // Verbindungsschicht seit dem Start nie oben war. War sie oben, wird bis
    // kMaxWaitS auf eine Adresse gewartet; steht sie dann (LinkNoIp) ohne IP ->
    // zurueck auf das vorige Image, ist sie wieder weg -> gueltig.
    void start_validation(HealthFn health);
    bool validating() const { return validating_; }

    static constexpr uint32_t kMinRunS  = 60;
    static constexpr uint32_t kMaxWaitS = 900;

private:
    OtaUpdate() = default;
    static void task_trampoline(void* arg);
    void        run_();
    esp_err_t   check_();
    esp_err_t   install_();
    void        set_error_(const char* fmt, ...) __attribute__((format(printf, 2, 3)));

    void*     task_ = nullptr;   // TaskHandle_t
    void*     mtx_  = nullptr;   // SemaphoreHandle_t
    NetFn     net_up_ = nullptr;
    AutoFn    auto_   = nullptr;

    // Alles hier ist char[]: geschrieben vom OTA-Task, gelesen von Frontend-
    // Tasks. Ein std::string waere unter gleichzeitigem Lesen ein
    // use-after-free (Regel 12 der WLAN/DNS-Checkliste).
    State     state_ = State::Idle;
    char      url_override_[160] = {};
    char      latest_[24]        = {};
    char      fw_url_[200]       = {};
    char      error_[96]         = {};
    int64_t   checked_us_        = 0;
    // 0..100 waehrend der Installation. Der OTA-Task schreibt es in der
    // perform()-Schleife ohne Mutex, Frontends lesen es -- daher atomar.
    std::atomic<int> progress_{-1};
    std::atomic<bool> validating_{false};   // Validierungs-Task schreibt, Frontends lesen
};

} // namespace culfw32
