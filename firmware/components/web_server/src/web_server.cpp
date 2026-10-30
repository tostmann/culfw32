// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "web_server.hpp"
#include "esp_log.h"
#include "esp_http_server.h"
#include <cstring>
#include <map>
#include <mutex>
#include <unistd.h>     // ::close fuer custom close_fn

namespace culfw32 {

namespace {
constexpr const char* TAG = "web";

// Embedded gz'd Web-UI-Assets (COMPONENT_EMBED_FILES, Symbol-Pattern wie
// vom esp-idf bei "binary linker symbols" dokumentiert).
extern const uint8_t INDEX_HTML_GZ_START[]   asm("_binary_index_html_gz_start");
extern const uint8_t INDEX_HTML_GZ_END[]     asm("_binary_index_html_gz_end");
extern const uint8_t APP_CSS_GZ_START[]      asm("_binary_app_css_gz_start");
extern const uint8_t APP_CSS_GZ_END[]        asm("_binary_app_css_gz_end");
extern const uint8_t APP_JS_GZ_START[]       asm("_binary_app_js_gz_start");
extern const uint8_t APP_JS_GZ_END[]         asm("_binary_app_js_gz_end");
extern const uint8_t LOGO_PNG_START[]        asm("_binary_busware_logo_png_start");
extern const uint8_t LOGO_PNG_END[]          asm("_binary_busware_logo_png_end");

// Mapping fd → WebSocketTransport — Dispatch im RX-Pfad und Cleanup beim Close.
std::map<int, std::weak_ptr<WebSocketTransport>> g_ws_sessions;
std::mutex g_ws_mtx;

// RFC 6455 §7.4: Status-Codes, die ein Endpunkt in ein CLOSE setzen darf und
// die wir deshalb zurueckspiegeln. 1005/1006/1015 "MUST NOT be set as a status
// code in a Close control frame" (§7.4.1), 0..999 "not used", 1004 reserviert,
// 1012..2999 in RFC 6455 nicht definiert (§7.4.2), 3000..4999 fuer Anwendungen.
bool ws_close_code_echoable(uint16_t code) {
    return (code >= 1000 && code <= 1003) || (code >= 1007 && code <= 1011)
        || (code >= 3000 && code <= 4999);
}

// Ein CLOSE-Frame, dessen Payload wir nicht (vollstaendig) lesen koennen oder
// wollen. httpd hat sd->ws_close schon VOR dem Handler gesetzt (httpd_parse.c
// httpd_req_new) -- auch fuer das CLOSE, das es selbst synthetisiert, wenn der
// Client die TCP-Verbindung ohne CLOSE-Frame mit FIN beendet (httpd_ws.c
// httpd_ws_get_frame_type, recv() == 0; einen RST erkennt es dort nicht, weil
// es den int-Rueckgabewert gegen sizeof vergleicht) -- und reicht nach dem
// Handler in httpd_req_cleanup ein httpd_sess_close mit dem sock_db-ZEIGER in
// die Work-Queue. Gaeben wir hier ESP_FAIL zurueck, loeschte httpd die Session
// sofort (httpd_main.c httpd_process_session); der Work-Eintrag bliebe stehen
// und traefe in der naechsten Loop-Iteration einen freien Slot -- oder die
// Verbindung, die der Accept derselben Iteration dort schon angelegt hat
// (httpd_sess.c httpd_sess_close; der Guard auf lru_counter == 0 greift nur,
// wenn sonst keine Session aktiv ist). Bis 0.1.878 am Board nachgestellt: die
// neue Verbindung wurde jedes Mal geschlossen. Darum: Session selbst als
// geschlossen markieren und ESP_OK -- den Socket schliesst der EINE schon
// geplante Work-Eintrag, gelesen wird aus ihm nicht mehr (httpd_req_new bei
// ws_close). Ungelesene Bytes verwirft lwIP beim close() per RST
// (tcp_close_shutdown).
// Rest, den nur httpd selbst beheben kann: enden zwei Sessions im selben
// Durchlauf, fasst httpd die zweite im Folgedurchlauf noch einmal an (ihr fd
// ist wieder lesbar, httpd_req_new kehrt bei ws_close frueh mit ESP_OK zurueck)
// und httpd_req_cleanup reiht einen ZWEITEN Close mit demselben Zeiger ein --
// der kann dieselbe Klasse Wettlauf ausloesen. Unser Handler laeuft dabei nicht.
esp_err_t ws_abandon_close(httpd_req_t* req) {
    const int fd = httpd_req_to_sockfd(req);
    std::shared_ptr<WebSocketTransport> t;
    {
        std::lock_guard<std::mutex> g(g_ws_mtx);
        auto it = g_ws_sessions.find(fd);
        if (it != g_ws_sessions.end()) t = it->second.lock();
    }
    if (t) t->mark_closed();
    return ESP_OK;
}

// Helper: Static-File mit gz Content-Encoding senden.
esp_err_t send_static_gz(httpd_req_t* req, const uint8_t* start, const uint8_t* end,
                         const char* mime) {
    httpd_resp_set_type(req, mime);
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    // Waehrend der aktiven Entwicklung: Browser soll bei jedem Reload neu
    // holen, damit UI-Patches (HTML/JS) sofort wirken. Sobald die UI stabil
    // ist, kann das wieder auf "max-age=600" oder besser ein ETag-Mechanismus.
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, must-revalidate");
    return httpd_resp_send(req, reinterpret_cast<const char*>(start), end - start);
}

esp_err_t handler_index(httpd_req_t* req) {
    return send_static_gz(req, INDEX_HTML_GZ_START, INDEX_HTML_GZ_END, "text/html");
}
esp_err_t handler_css(httpd_req_t* req) {
    return send_static_gz(req, APP_CSS_GZ_START, APP_CSS_GZ_END, "text/css");
}
esp_err_t handler_js(httpd_req_t* req) {
    return send_static_gz(req, APP_JS_GZ_START, APP_JS_GZ_END, "application/javascript");
}
esp_err_t handler_logo(httpd_req_t* req) {
    httpd_resp_set_type(req, "image/png");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=86400");
    return httpd_resp_send(req, reinterpret_cast<const char*>(LOGO_PNG_START),
                           LOGO_PNG_END - LOGO_PNG_START);
}

// Generic WS-Handler. Bei first call (HTTP-Upgrade) wird die Session geoeffnet
// und die WebSocketTransport-Instanz erstellt + dem Frontend uebergeben.
// Bei nachfolgenden frames empfaengt er die Bytes und routet sie via
// gespeichertem WebSocketTransport an das Frontend (on_rx).
esp_err_t ws_handler_generic(httpd_req_t* req,
                              WebServer::OnWsOpen* hook,
                              const char* tag) {
    if (req->method == HTTP_GET) {
        // HTTP-Upgrade-Phase. esp_http_server hat schon den 101-Switch
        // versendet. Wir merken uns die Session.
        int fd = httpd_req_to_sockfd(req);
        auto t = std::make_shared<WebSocketTransport>(req->handle, fd, tag);
        {
            std::lock_guard<std::mutex> g(g_ws_mtx);
            g_ws_sessions[fd] = t;
        }
        // Den Eintrag in g_ws_sessions entfernt allein close_fn. on_closed
        // gehoert dem Frontend: die Hooks in main.cpp setzen dort
        // detach_dynamic_transport.
        if (hook && *hook) (*hook)(t);
        ESP_LOGI(TAG, "ws[%s] open fd=%d", tag, fd);
        return ESP_OK;
    }

    // Frame empfangen.
    httpd_ws_frame_t f = {};
    f.type = HTTPD_WS_TYPE_TEXT;
    auto r = httpd_ws_recv_frame(req, &f, 0);    // Laenge holen
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "ws_recv len: %s", esp_err_to_name(r));
        // f.type steht schon (httpd_ws_recv_frame uebernimmt ihn vor dem Lesen);
        // bei CLOSE kein ESP_FAIL, s. ws_abandon_close().
        if (f.type == HTTPD_WS_TYPE_CLOSE) return ws_abandon_close(req);
        return r;
    }
    if (f.len > 4096) {
        ESP_LOGW(TAG, "ws frame too big: %u", static_cast<unsigned>(f.len));
        if (f.type == HTTPD_WS_TYPE_CLOSE) return ws_abandon_close(req);
        return ESP_FAIL;
    }
    // RFC 6455 §5.2: unbekannter Opcode (0x3..0x7, 0xB..0xF) -> "Fail the
    // WebSocket Connection"; §5.5: Control-Frames (Opcode-Bit 3 gesetzt) sind
    // hoechstens 125 Byte lang und nie fragmentiert. httpd prueft beides nicht
    // (httpd_ws.c uebernimmt first_byte & 0x0F ungeprueft) -- ein PING > 125
    // Byte ginge sonst als PONG > 125 Byte zurueck, ein reservierter Opcode
    // als Daten ins Frontend. ESP_FAIL laesst httpd die Session loeschen
    // (close_fn -> mark_closed), wie beim 4096er-Limit; das CLOSE-Frame mit
    // 1002, das §7.1.7 dafuer empfiehlt (SHOULD), schicken wir dabei nicht.
    // Ausnahme CLOSE selbst: ein zu langes oder fragmentiertes CLOSE darf nicht
    // auf ESP_FAIL laufen (ws_close ist gesetzt, s. ws_abandon_close()); wir
    // lesen seinen Payload (<= 4096) und antworten mit CLOSE 1002.
    const bool known_opcode = f.type == HTTPD_WS_TYPE_CONTINUE || f.type == HTTPD_WS_TYPE_TEXT
                           || f.type == HTTPD_WS_TYPE_BINARY   || f.type == HTTPD_WS_TYPE_CLOSE
                           || f.type == HTTPD_WS_TYPE_PING     || f.type == HTTPD_WS_TYPE_PONG;
    const bool control = (static_cast<unsigned>(f.type) & 0x8u) != 0;
    const bool bad_control = control && (f.len > 125 || !f.final);
    const bool close_protocol_error = bad_control && f.type == HTTPD_WS_TYPE_CLOSE;
    if (!known_opcode || (bad_control && !close_protocol_error)) {
        ESP_LOGW(TAG, "ws opcode 0x%X len=%u fin=%d rejected", static_cast<unsigned>(f.type),
                 static_cast<unsigned>(f.len), f.final ? 1 : 0);
        return ESP_FAIL;
    }
    if (close_protocol_error) {
        ESP_LOGW(TAG, "ws CLOSE len=%u fin=%d -> 1002", static_cast<unsigned>(f.len),
                 f.final ? 1 : 0);
    }
    // Dynamic alloc — Stack ist im http-Server-Task knapp. Ein leerer Frame
    // (z. B. PING ohne Daten) hat nichts zu lesen, wird aber nicht mehr vor der
    // Typ-Auswertung verworfen: auch er braucht sein PONG.
    std::unique_ptr<uint8_t[]> buf;
    if (f.len > 0) {
        buf.reset(new uint8_t[f.len + 1]);
        f.payload = buf.get();
        r = httpd_ws_recv_frame(req, &f, f.len);
        if (r != ESP_OK) return f.type == HTTPD_WS_TYPE_CLOSE ? ws_abandon_close(req) : r;
        buf[f.len] = 0;
    }

    // Dispatch zur passenden Session.
    int fd = httpd_req_to_sockfd(req);
    std::shared_ptr<WebSocketTransport> t;
    {
        std::lock_guard<std::mutex> g(g_ws_mtx);
        auto it = g_ws_sessions.find(fd);
        if (it != g_ws_sessions.end()) t = it->second.lock();
    }
    if (!t) return ESP_OK;

    // Mit handle_ws_control_frames = true landen PING/PONG/CLOSE hier, und httpd
    // beantwortet einen PING nicht mehr selbst (httpd_ws.c, nur bei
    // !ws_control_frames). RFC 6455: auf PING MUSS ein PONG folgen (§5.5.2),
    // mit denselben Anwendungsdaten (§5.5.3). Bis 0.1.876 ging ein PING ohne Daten
    // unbeantwortet unter, einer mit Daten landete als Text im Frontend.
    const std::span<const uint8_t> payload{buf.get(), f.len};
    switch (f.type) {
        case HTTPD_WS_TYPE_CLOSE: {
            // RFC 6455 §5.5.1: auf ein CLOSE MUSS ein CLOSE zurueck, typisch mit
            // dem empfangenen Status-Code (die ersten 2 Byte). httpd antwortet
            // mit handle_ws_control_frames = true nicht mehr selbst (httpd_ws.c,
            // nur bei !ws_control_frames); den TCP-Socket schliesst es nach dem
            // Handler ueber ws_close (httpd_parse.c, httpd_req_cleanup).
            // close_with_frame() kippt open_ und schickt das Echo als letzten
            // Frame in die TxQueue (last): ein write() aus esp_timer-Push oder
            // RF-Broadcast, das open_ eben noch offen sah, weist die Queue
            // danach ab (§5.5.1: keine Datenframes nach dem CLOSE).
            // Echo nur fuer einen nach §7.4 zulaessigen Code. Ein Payload von
            // 1 Byte (§5.5.1: "MUST be a 2-byte unsigned integer") oder ein Code,
            // den kein Endpunkt setzen darf (1005/1006/1015, 0..999, 1004), ist
            // ein Protokollfehler -> 1002 (§7.1.7 SHOULD). Ein leeres CLOSE
            // bleibt leer (§7.1.5: die Gegenseite sieht dann 1005).
            static constexpr uint8_t kProtocolError[2] = {0x03, 0xEA};   // 1002
            std::span<const uint8_t> echo{};
            if (close_protocol_error) {
                echo = std::span<const uint8_t>{kProtocolError};
            } else if (f.len >= 2) {
                const uint16_t code = static_cast<uint16_t>((payload[0] << 8) | payload[1]);
                echo = ws_close_code_echoable(code) ? payload.first(2)
                                                    : std::span<const uint8_t>{kProtocolError};
            } else if (f.len == 1) {
                echo = std::span<const uint8_t>{kProtocolError};
            }
            t->close_with_frame(echo);
            break;
        }
        case HTTPD_WS_TYPE_PING:
            t->write_control(HTTPD_WS_TYPE_PONG, payload);
            break;
        case HTTPD_WS_TYPE_PONG:
            break;
        default:                  // TEXT, BINARY, CONTINUE -> Frontend
            if (f.len > 0) t->feed_rx_frame(payload);
            break;
    }
    return ESP_OK;
}

// Single-purpose handler dispatch — `req->user_ctx` zeigt direkt auf den
// jeweiligen OnWsOpen-Functor.
esp_err_t ws_handler_dispatch(httpd_req_t* req) {
    auto* hook = static_cast<WebServer::OnWsOpen*>(req->user_ctx);
    // req->uri ist ein Array-Member (nie NULL); strstr akzeptiert den Pointer.
    const char* tag = std::strstr(req->uri, "tty") ? "ws-tty" : "ws-json";
    return ws_handler_generic(req, hook, tag);
}

} // anon namespace

esp_err_t WebServer::start(uint16_t port) {
    if (server_) return ESP_OK;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port    = port;
    cfg.max_uri_handlers = 12;
    cfg.lru_purge_enable = true;
    cfg.stack_size       = 8192;       // braucht mehr fuer cJSON-Path
    cfg.uri_match_fn     = httpd_uri_match_wildcard;
    // WS-Reconnects (Browser-Tab-Switch / Auto-Reconnect alle 2 s) leakten
    // Sockets, weil Browser kein WS-CLOSE-Frame schickt — TCP einfach RST.
    // Default ist 7; mit 13 + LRU-Purge haben wir Headroom; LWIP_MAX_SOCKETS
    // steht in sdkconfig.defaults auf 24 (TCP-Listener und -Clients, httpd,
    // mDNS teilen sie sich).
    cfg.max_open_sockets = 13;
    // close_fn-Hook: Beim httpd-internen Socket-Close (TCP-RST,Timeout, LRU-
    // Evict) muessen wir die WebSocketTransport-Session aufraeumen — sonst
    // bleiben weak_ptr-Eintraege in g_ws_sessions stehen und JSON-Frontend
    // kuemmert sich nur beim naechsten write().
    cfg.close_fn = +[](httpd_handle_t hd, int sockfd) {
        std::shared_ptr<WebSocketTransport> t;
        {
            std::lock_guard<std::mutex> g(g_ws_mtx);
            auto it = g_ws_sessions.find(sockfd);
            if (it != g_ws_sessions.end()) {
                t = it->second.lock();
                g_ws_sessions.erase(it);
            }
        }
        // Erst auf eine laufende Sendung warten, dann schliessen -- sonst kann
        // die fd-Nummer neu vergeben sein, waehrend noch gesendet wird.
        if (t) t->close_for_fd_release();
        // a-culfw-style: httpd default close_fn closet das socket. Wir sind
        // hier nur fuer das Aufraeumen unserer Session-Refs zustaendig. Aber
        // weil wir close_fn ueberschreiben muessen wir close() selbst rufen.
        ::close(sockfd);
    };

    if (auto e = httpd_start(&server_, &cfg); e != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(e));
        return e;
    }

    ESP_LOGW(TAG, "web_server: kein Auth - nur in vertrauenswuerdigem Netz betreiben");

    // ========================================================================
    // SICHERHEIT: Dieser Web-Server ist UNAUTHENTISIERT (bewusst, v1).
    // ------------------------------------------------------------------------
    // Die unten registrierten Endpunkte sind ohne Login/Token/TLS fuer JEDEN
    // im selben LAN erreichbar:
    //   /ws/json -> Config-Setter, RF-Steuerung, WiFi-Credentials (FrontendJson)
    //   /ws/tty  -> komplettes ASCII-Frontend (RF-TX, EEPROM, WiFi-Setup)
    // Auth/TLS ist ein separates v1.x-Item. Bis dahin: Server NUR in einem
    // vertrauenswuerdigen Netz betreiben - nicht ans offene Internet oder
    // Gast-WLAN haengen.
    // ========================================================================

    // Static-Routes
    // URIs mit Wildcard-Suffix damit Query-String-Cache-Buster (?v=N) matched.
    // is_websocket/handle_ws_control_frames/supported_subprotocol absichtlich
    // weggelassen — Default-Init aus dem httpd-Header reicht.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    httpd_uri_t r_idx = { .uri="/",            .method=HTTP_GET, .handler=handler_index, .user_ctx=nullptr };
    httpd_uri_t r_idx2= { .uri="/index.html*", .method=HTTP_GET, .handler=handler_index, .user_ctx=nullptr };
    httpd_uri_t r_css = { .uri="/app.css*",    .method=HTTP_GET, .handler=handler_css,   .user_ctx=nullptr };
    httpd_uri_t r_js  = { .uri="/app.js*",     .method=HTTP_GET, .handler=handler_js,    .user_ctx=nullptr };
    httpd_uri_t r_logo= { .uri="/assets/busware_logo.png*", .method=HTTP_GET, .handler=handler_logo, .user_ctx=nullptr };
#pragma GCC diagnostic pop
    httpd_register_uri_handler(server_, &r_idx);
    httpd_register_uri_handler(server_, &r_idx2);
    httpd_register_uri_handler(server_, &r_css);
    httpd_register_uri_handler(server_, &r_js);
    httpd_register_uri_handler(server_, &r_logo);

    // WS-Endpoints. esp_http_server signalisiert WS via .is_websocket = true.
    httpd_uri_t r_ws_json = {
        .uri      = "/ws/json",
        .method   = HTTP_GET,
        .handler  = ws_handler_dispatch,
        .user_ctx = &on_json_open_,
        .is_websocket = true,
        .handle_ws_control_frames = true,
        .supported_subprotocol = nullptr,
    };
    httpd_uri_t r_ws_tty = {
        .uri      = "/ws/tty",
        .method   = HTTP_GET,
        .handler  = ws_handler_dispatch,
        .user_ctx = &on_tty_open_,
        .is_websocket = true,
        .handle_ws_control_frames = true,
        .supported_subprotocol = nullptr,
    };
    httpd_register_uri_handler(server_, &r_ws_json);
    httpd_register_uri_handler(server_, &r_ws_tty);

    ESP_LOGI(TAG, "ready on port %u", port);
    return ESP_OK;
}

esp_err_t WebServer::stop() {
    if (!server_) return ESP_OK;
    httpd_stop(server_);
    server_ = nullptr;
    return ESP_OK;
}

} // namespace culfw32
