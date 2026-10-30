// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "transport.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace culfw32 {

// USB-CDC-Transport auf ESP32-C6 (USB Serial/JTAG controller, kein TinyUSB noetig
// fuer einfaches CDC — der C6 hat USB-OTG nur eingeschraenkt; das eingebaute
// USB-Serial-JTAG ist fuer culfw-tty-Use-Case ausreichend).
//
// Verifizierbarkeit: ESP-IDF Komponente `usb_serial_jtag` ist die unterstuetzte API.
// Pruefen vor Implementation: docs/api-reference/peripherals/usb_serial_jtag_console.html
class UsbCdcTransport final : public Transport {
public:
    // Optionaler Filter fuer eingehende Bytes — bekommt jeden Byte-Block VOR
    // dem ASCII-Frontend und entscheidet welche Bytes weitergereicht werden.
    // Der Filter darf via emit_to_frontend() Bytes an die Frontend-Pipe
    // forwarden (z.B. Bytes die NICHT zu einem IMPROV-Frame gehoeren).
    // Verwendung: Improv-Serial-Provisioning verschluckt waehrend eines
    // erkannten IMPROV-Frames die Bytes vor dem ASCII-Frontend, sodass das
    // Frontend nicht mit `? (IMPROV is unknown)` antwortet (was den
    // Improv-Client-Parser verwirrt da die Antwort selbst das IMPROV-Magic
    // enthielte).
    using RxEmit   = void(*)(const uint8_t* data, size_t len, void* emit_user);
    using RxFilter = void(*)(const uint8_t* data, size_t len, void* user,
                              RxEmit emit_to_frontend, void* emit_user);

    esp_err_t start() override;
    esp_err_t stop()  override;
    esp_err_t write(std::span<const uint8_t> data) override;
    void      on_rx(RxCallback cb) override;
    const char* name() const override { return "usb-cdc"; }

    // Setze/loesche den Filter. Setzen mit nullptr (Default) deaktiviert ihn —
    // RX-Bytes gehen dann unveraendert direkt an rx_cb_.
    // Atomic-store, sicher vom Main-Task gegen den RX-Task aus aufrufbar.
    void set_rx_filter(RxFilter fn, void* user);

    // Backpressure-Telemetrie — fuer ?stats.tx.cdc.* und JSON-frontend dump.
    // Modell: write() enqueued in einen FreeRTOS-Message-Buffer, ein
    // dedizierter tx_task drained blocking aufs USB-CDC-Endpoint.  Bei voller
    // Queue evict-oldest (max kMaxEvictsPerSend Runden); wenn das neue Frame
    // immer noch nicht passt zaehlt's als dropped_full.
    //
    // Sichtbar-machen war der ganze Punkt — vorheriges Verhalten war
    // `usb_serial_jtag_write_bytes(... 0)` mit verworfener Return-Value =
    // silent loss (Symptom 2026-05-11 Phase-5: Sniff-Python silent dead
    // nach 4 min unter HmIP-Burst-Load).
    uint32_t tx_enqueued() const       { return tx_enqueued_.load(std::memory_order_relaxed); }
    uint32_t tx_sent() const           { return tx_sent_.load(std::memory_order_relaxed); }
    uint32_t tx_dropped_full() const   { return tx_dropped_full_.load(std::memory_order_relaxed); }
    uint32_t tx_evicted_oldest() const { return tx_evicted_oldest_.load(std::memory_order_relaxed); }
    uint32_t tx_dropped_host_gone() const { return tx_dropped_host_gone_.load(std::memory_order_relaxed); }

    // Single-Instance-Accessor: main.cpp baut genau einen UsbCdcTransport.
    // Wird in start() gesetzt, in stop() geloescht. Erlaubt `?stats` und
    // anderen Diagnose-Pfaden Zugriff auf die Counter ohne durch die
    // Transport-Liste der Frontends iterieren zu muessen.
    static UsbCdcTransport* instance() { return s_instance_.load(std::memory_order_acquire); }

private:
    static void rx_task(void* arg);
    static void tx_task(void* arg);
    static void emit_to_frontend_(const uint8_t* d, size_t n, void* self);

    RxCallback rx_cb_;
    void*      task_handle_    = nullptr;   // RX task
    void*      tx_task_handle_ = nullptr;   // TX-pump task
    void*      tx_buf_         = nullptr;   // MessageBufferHandle_t (void* to keep FreeRTOS include out of header)
    bool       running_ = false;

    std::atomic<RxFilter> rx_filter_{nullptr};
    std::atomic<void*>    rx_filter_user_{nullptr};

    std::atomic<uint32_t> tx_enqueued_         {0};
    std::atomic<uint32_t> tx_sent_             {0};
    std::atomic<uint32_t> tx_dropped_full_     {0};
    std::atomic<uint32_t> tx_evicted_oldest_   {0};
    std::atomic<uint32_t> tx_dropped_host_gone_{0};

    static std::atomic<UsbCdcTransport*> s_instance_;
};

} // namespace culfw32
