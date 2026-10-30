// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "transport.hpp"
#include "tx_queue.hpp"
#include "freertos/task.h"
#include <atomic>
#include <memory>

namespace culfw32 {

// TcpClientTransport — wraps eine vom TcpServer angenommene Client-Socket.
// Lebensdauer: vom Server angelegt, lebt solange Verbindung steht. Bei
// Disconnect (recv -> 0/EOF/error) ruft on_closed-Callback (z.B. server
// entfernt aus Frontend-Liste).
//
// rx-Pfad laeuft NICHT mehr in einem pro-Connect-Task (Build < 0.1.789),
// sondern in einem Pool langlebiger Worker-Tasks (TcpRxPool). Siehe
// heap_leak_root_cause_confirmed_2026-05-12.md fuer die Begruendung.
//
// tx-Pfad ueber eine TxQueue (transport_txq): fremde Tasks (RX-Task,
// esp_timer-Push) warten nie am Socket, was er nicht sofort nimmt, schiebt der
// Sende-Task nach. Antworten auf eigene Anfragen schreibt der Empfangs-Task
// dieses Clients; hat das Frontend es erlaubt (set_reply_wait), wartet er auf
// Abfluss (write_blocking), sonst puffert er wie alle anderen. Wer 3 s lang
// nichts abnimmt oder den Puffer sprengt, wird getrennt.
class TcpClientTransport final
    : public Transport,
      public std::enable_shared_from_this<TcpClientTransport> {
public:
    using OnClosed = std::function<void(TcpClientTransport*)>;

    TcpClientTransport(int sock_fd);
    ~TcpClientTransport() override;

    const char* name() const override { return "tcp"; }
    esp_err_t   start() override;
    esp_err_t   stop()  override;
    esp_err_t   write(std::span<const uint8_t> data) override;
    void        on_rx(RxCallback cb) override { rx_cb_ = std::move(cb); }
    size_t      tx_backlog() const override { return txq_ ? txq_->pending() : 0; }
    void        set_reply_wait(bool on) override { reply_wait_ = on; }

    // Wird gerufen aus dem rx-task NACH EOF/error/stop, BEVOR der Task sich
    // selbst loescht. Caller (Frontend) kann hier den Transport aus seiner
    // Liste entfernen — der shared_ptr-Refcount geht dann irgendwann auf 0.
    // self ist NICHT als shared_ptr gegeben (Lebensdauer-Race-Vermeidung):
    // Caller benutzt den raw-Pointer nur fuer Identifikation (find-by-pointer).
    void set_on_closed(OnClosed cb) { on_closed_ = std::move(cb); }

    // Bezeichner fuer Logs.
    int sock() const { return sock_; }

    // TCP-Clients, die getrennt wurden, weil sie 3 s lang nichts abnahmen.
    static uint32_t stall_drops() { return TxQueue::stall_drops(TxQueue::Kind::Tcp); }

    // Hooks fuer TcpRxPool — Pool-Worker rufen diese statt eines eigenen
    // Task-Loops. Friend waere sauberer; public hier weil Header-Boilerplate
    // billiger als Friend-Forward-Declarations.
    void run_rx_loop_public();
    void invoke_on_closed_public();

private:
    int                       sock_    = -1;
    std::shared_ptr<TxQueue>  txq_;
    std::atomic<bool>         running_ = false;
    std::atomic<TaskHandle_t> rx_task_{nullptr};   // TcpRxPool-Worker dieses Clients
    std::atomic<bool>         reply_wait_{false};  // Transport::set_reply_wait
    RxCallback                rx_cb_;
    OnClosed                  on_closed_;
};

} // namespace culfw32
