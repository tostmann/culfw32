// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "frontend.hpp"
#include "transport.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <array>
#include <memory>
#include <string>
#include <string_view>

// cJSON liegt im globalen Namespace (C-Library) — Forward-Decl hier raus, damit
// die Method-Signatur `handle_tx_(::cJSON*, ...)` aufloesen kann ohne dass wir
// cJSON.h vollumfaenglich in den Header ziehen.
struct cJSON;

namespace culfw32 {

// JSON-Line-Frontend (CULFW32-Extension, parallel zum ASCII-CulfwFrontend).
//
// Protokoll: ein JSON-Object pro Zeile, separiert durch \r\n oder \n.
// TCP/USB/UART (Bytestrom): eine Zeile endet nur am \n, \r wird ignoriert.
// WebSocket: ein Frame ist eine Zeile, das \n ist optional.
// Hoechstens 1023 Zeichen je Zeile (kLineBufSize - 1). Beim 1024. kommt
// {"event":"err","msg":"line too long"} -- noch bevor der Client die Zeile zu
// Ende geschickt hat --, und der Rest bis zum Zeilenende (WebSocket: bis zum
// Frame-Ende) wird ohne weitere Meldung verworfen.
//
// Server -> Client (Events, asynchron):
//   {"event":"hello","fw":"CULFW32","registry_size":145}
//   {"event":"rx","proto":"<name>","slot":0,"rssi_raw":-77,"ts_us":12345}   // nur Kopf, keine Nutzdaten
//   {"event":"value","key":"wifi.state","value":"connected"}
//       Antwort auf get, und als Live-Push alle 5 s fuer jeden readonly-Eintrag.
//       Solange set_push_pause() true liefert (main.cpp: Online-Update prueft
//       oder laedt, ota.state checking/installing), gehen ohne Ankuendigung nur
//       noch die ota.*-Eintraege raus -- wer Lebendigkeit an anderen Werten
//       misst, sieht dann eine Luecke von Pruefung + Download.
//   {"event":"ack","cmd":"set","key":"wifi.ssid","ok":true}
//   {"event":"tx_ack","proto":"asksin","ok":false,"err":"LENERR"}
//   {"event":"err","msg":"unknown key","key":"foo.bar"}
//
// Client -> Server (Commands):
//   {"cmd":"get","key":"wifi.ssid"}                          // -> value
//   {"cmd":"set","key":"wifi.ssid","value":"MyHomeWifi"}     // -> ack
//   {"cmd":"dump"}     // -> dump_begin {count}, je Eintrag dump_item {key, type,
//                      //    persist, readonly, sensitive, value, help, unit,
//                      //    enum_labels, hass_device_class}, dump_end
//   {"cmd":"tx","proto":"asksin","hex":"..."}               // -> tx_ack; Felder je
//                      //    Protokoll an handle_tx_() (frontend_json.cpp)
//   {"cmd":"ping"}                                           // -> {"event":"pong"}
//
// Eigener TCP-Port (Default 2324) — kommt via main.cpp + zweiter TcpServer-Instanz.
// USB-CDC und UART sind moeglich, aber primaerer Use-Case ist TCP fuer Web-UI/HASS.
class JsonFrontend final : public Frontend {
public:
    const char* name() const override { return "json"; }
    std::span<const ProtocolId> compatible_protocols() const override;
    void   attach(const FrontendServices& services) override;
    void   bind_transport(std::shared_ptr<Transport> t) override;
    esp_err_t start() override;
    esp_err_t stop()  override;

    // Dynamic-Attach fuer TCP-Clients (gleicher Pattern wie CulfwFrontend).
    void attach_dynamic_transport(std::shared_ptr<Transport> t);
    void detach_dynamic_transport(Transport* t);

    // Live-Push ausduennen, solange fn() true liefert: dann gehen nur Eintraege
    // raus, deren Schluessel mit keep_prefix beginnt. main.cpp haengt hier das
    // Online-Update ein (keep_prefix "ota."): ~130 Werte je Client alle 5 s
    // fielen sonst in den TLS-Abruf (gemessen 11.10.2026 ueber WLAN: mit zwei
    // offenen Browsern Heap-Tiefstmarke 10 KB, mit einem 28-51 KB), die drei
    // ota.*-Werte halten andere Tabs ueber Fortschritt und Ende im Bild.
    // Vor start() setzen; keep_prefix muss so lange leben wie das Frontend.
    using PushPauseFn = bool (*)();
    void set_push_pause(PushPauseFn fn, const char* keep_prefix = nullptr) {
        push_pause_ = fn;
        push_keep_  = keep_prefix;
    }

private:
    static constexpr size_t kMaxTransports = 8;
    static constexpr size_t kLineBufSize   = 1024;     // JSON-Frame kann groesser sein als ASCII-line

    struct LineBuf {
        std::array<char, kLineBufSize> data{};
        size_t len  = 0;
        bool   drop = false;   // Zeile zu lang: Rest bis zum Zeilenende verwerfen
    };

    void on_rx_data_(std::span<const uint8_t> bytes, Transport* src);
    void process_line_(std::span<const char> line, Transport* src);
    void emit_to_(Transport* t, std::string_view json);
    void broadcast_(std::string_view json);

    void send_hello_(Transport* t);
    void send_value_(Transport* t, std::string_view key);
    void send_ack_(Transport* t, const char* cmd, std::string_view key, bool ok,
                   const char* err = nullptr);
    void send_err_(Transport* t, const char* msg, const char* key = nullptr);
    void send_dump_(Transport* t);
    void send_tx_ack_(Transport* t, const char* proto, bool ok, const char* err);
    void handle_tx_(::cJSON* root, Transport* src);

    void on_rx_event_(const struct RxEvent& ev);  // RfManager-Subscription

    std::vector<std::shared_ptr<Transport>> transports_;
    LineBuf  line_buf_per_transport_[kMaxTransports]{};
    Transport* lb_owner_[kMaxTransports]{};      // welcher Transport gehoert zu welchem LineBuf
    SemaphoreHandle_t mtx_ = nullptr;

    FrontendServices services_{};
    bool started_ = false;

    // Live-Push: ESP-Timer fired ~5 s, broadcastet alle FLAG_READONLY-Eintraege
    // als {"event":"value", ...}. Browser updated dadurch wifi.state/ip/rssi
    // und uptime ohne explizites get/dump.
    void* push_timer_ = nullptr;     // esp_timer_handle_t (opaque)
    PushPauseFn push_pause_ = nullptr;
    const char* push_keep_  = nullptr;
    static void push_tick_trampoline_(void* arg);
    void push_tick_();
};

} // namespace culfw32
