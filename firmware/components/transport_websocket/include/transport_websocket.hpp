// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "transport.hpp"
#include "tx_queue.hpp"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>
#include <functional>
#include <memory>

namespace culfw32 {

// WebSocketTransport — wraps eine durch esp_http_server angenommene WS-Connection.
//
// Pattern wie TcpClientTransport: pro accepted Browser-Connection wird eine
// shared_ptr<WebSocketTransport> erzeugt. Frontend (Json oder Culfw) haengt
// sie via attach_dynamic_transport an. on_closed-Callback bei WS-Close raeumt
// das Frontend wieder ab.
//
// RX: Der esp_http_server-WS-Handler ruft `feed_rx_frame()` aus dem httpd-Task
// pro empfangenem Frame. Wir leiten das per RxCallback weiter (Frontend muss
// selbst entkoppeln wenn es viel Zeit braucht).
//
// TX: Die Frames (Server -> Browser, unmaskiert, RFC 6455 §5.2) bauen wir selbst
// und schreiben Kopf + Nutzlast zusammen in eine TxQueue (transport_txq). Fremde
// Tasks (Funk-RX, JSON-Push) warten nie am Socket; der Sende-Task der TxQueue
// schiebt nach und trennt nach 3 s ohne Abfluss. Antworten auf einen Frame
// dieser Session, die der httpd-Task waehrend feed_rx_frame() schreibt, warten
// auf Abfluss, wenn das Frontend es erlaubt (set_reply_wait): der JSON-dump
// der Web-UI (~28 KB) sprengte sonst den Puffer (0.1.903 gemessen: 2 von 3
// Laeufen getrennt). Der httpd-Task steht dann so lange, wie der Browser fuer
// seine eigene Antwort braucht, je Frame hoechstens kReplyBudgetUs; danach
// wird die Session getrennt (?stats: ws_stall_drop, Log "too slow").
// httpd_ws_send_frame_async wird nicht mehr benutzt: es sendet synchron im
// aufrufenden Task. Der httpd selbst schreibt nach dem Upgrade nicht mehr auf
// die Session (handle_ws_control_frames = true).
class WebSocketTransport : public Transport, public std::enable_shared_from_this<WebSocketTransport> {
public:
    // sock_fd identifiziert die WS-Session im httpd. Wird vom Server beim
    // Connection-Open gesetzt.
    WebSocketTransport(httpd_handle_t server, int sock_fd, const char* tag);
    ~WebSocketTransport() override;

    esp_err_t   start() override;
    esp_err_t   stop()  override;
    esp_err_t   write(std::span<const uint8_t> data) override;
    void        on_rx(RxCallback cb) override { rx_cb_ = std::move(cb); }
    bool        message_framed() const override { return true; }   // ein Frame je Callback
    const char* name() const override { return tag_; }
    size_t      tx_backlog() const override { return txq_ ? txq_->pending() : 0; }
    void        set_reply_wait(bool on) override { reply_wait_ = on; }

    // Vom WS-URI-Handler aufgerufen: Daten an Frontend weiterleiten.
    void feed_rx_frame(std::span<const uint8_t> bytes);

    // Control-Frame senden (PONG auf PING, RFC 6455 §5.5.2/§5.5.3). Geht durch
    // dieselbe TxQueue wie write(); Kopf und Nutzlast bleiben zusammen.
    esp_err_t write_control(httpd_ws_type_t type, std::span<const uint8_t> data);

    // CLOSE senden und die Session schliessen. Die TxQueue nimmt danach nichts
    // mehr an, auch nicht von einem Task, der open_ noch offen gesehen hat
    // (RFC 6455 §5.5.1: "MUST NOT send any more data frames after sending a
    // Close frame"). Ruft on_closed genau einmal; ein spaeteres mark_closed()
    // ist dann ein No-op.
    esp_err_t close_with_frame(std::span<const uint8_t> status);

    // Vom WS-Disconnect-Pfad aufgerufen: markiert geschlossen, ruft on_closed.
    void mark_closed();

    // Vom httpd-close_fn VOR ::close(fd) aufgerufen: schliesst die TxQueue.
    // Danach sendet kein Task mehr auf die fd-Nummer; der httpd darf sie
    // schliessen und neu vergeben.
    void close_for_fd_release();

    using OnClosed = std::function<void(WebSocketTransport*)>;
    void set_on_closed(OnClosed cb) { on_closed_ = std::move(cb); }

    int sock_fd() const { return sock_fd_; }
    bool is_open() const { return open_.load(); }

    // WS-Sessions, die getrennt wurden, weil sie 3 s lang nichts abnahmen
    // oder ihr Antwortbudget (kReplyBudgetUs) aufgebraucht war.
    static uint32_t stall_drops() { return TxQueue::stall_drops(TxQueue::Kind::Ws); }

private:
    // Wartezeit auf Abfluss, die alle Antworten auf einen empfangenen Frame
    // zusammen verbrauchen duerfen (s. feed_rx_frame). Ein Browser im LAN
    // nimmt den ganzen dump (~28 KB) in Millisekunden ab. Einen einzelnen
    // Aussetzer regelt die Stau-Uhr (TxQueue::kStallUs); einer, den sie
    // toleriert, kostet vom Budget gut 1 s. Das Budget trifft erst die Summe
    // der Wartezeiten aller Antworten auf einen Frame.
    static constexpr int64_t kReplyBudgetUs = 10 * 1000 * 1000;

    // wait: Antwort auf den Frame, den dieser Task gerade zustellt
    // (write_blocking); sonst puffern. last: s. TxQueue::write.
    esp_err_t send_frame_(httpd_ws_type_t type, std::span<const uint8_t> data, bool last,
                          bool wait);

    httpd_handle_t            server_;
    int                       sock_fd_;
    char                      tag_[16];
    RxCallback                rx_cb_;
    OnClosed                  on_closed_;
    std::atomic<bool>         open_{true};
    std::atomic<bool>         reply_wait_{false};   // Transport::set_reply_wait
    std::atomic<TaskHandle_t> rx_task_{nullptr};    // stellt gerade einen Frame zu
    int64_t                   reply_budget_us_ = 0; // nur der rx_task_ liest/schreibt
    std::shared_ptr<TxQueue>  txq_;
};

} // namespace culfw32
