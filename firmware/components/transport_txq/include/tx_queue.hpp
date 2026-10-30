// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace culfw32 {

// TxQueue — Sendeweg je Netz-Client (TCP oder WebSocket).
//
// Geschrieben wird aus fremden Tasks: dem RX-Task der Funkklasse (Renderer),
// den Frontend-Tasks und dem esp_timer-Task (JSON-Live-Push). Keiner von ihnen
// darf am Socket warten: ein haengender Client hielt sonst den Funkempfang an
// (bis 0.1.893 unbegrenzt) oder den esp_timer-Task mit HB-RF-ETH-Keepalive,
// wMBus-Takt und Temperatur-Sampler (0.1.900/0.1.901 bis ~6 s, Review 0901 H1).
//
// write() schickt sofort, was der Socket ohne Warten nimmt, und legt den Rest
// in einen Puffer, den der Sende-Task nachschiebt. Ist der Puffer nicht leer,
// wird jeder weitere Aufruf angehaengt; die Reihenfolge bleibt erhalten, auch
// fuer WS-Frames (Kopf + Nutzlast gehen zusammen hinein). Getrennt wird ein
// Client erst, wenn
//   - kStallUs lang weniger als kMinProgressBytes abflossen (stall),
//   - sein Puffer (kBufBytes) nicht reicht (overflow), oder
//   - kein Puffer zu haben ist (nobuf): alle zusammen hoechstens kBudgetBytes,
//     also vier gleichzeitig gestaute Clients, oder malloc scheitert, oder
//     ein Online-Update laeuft (set_shed_fn).
// 3 s decken einen Netzaussetzer bis ~1 s samt TCP-Wiederholung (gemessen
// 13.10.2026: 0,5-1 s Sperre -> bis 1,4 s ohne Fortschritt, 1,5-2 s -> bis
// 4,4 s). Getrennt heisst shutdown(); den Deskriptor schliesst sein Besitzer
// (TcpRxPool-Worker bzw. httpd), der vorher close() aufruft.
//
// Der Puffer faengt Funkzeilen, Live-Push und kleine Antworten. Grosse
// Antworten (JSON-dump ~28 KB, ??-Tabelle, ?stats) passen nicht hinein; sie
// gehen ueber write_blocking() (Transport::set_reply_wait).
class TxQueue {
public:
    enum class Kind : uint8_t { Tcp = 0, Ws = 1 };

    static constexpr int64_t kStallUs          = 3000 * 1000;
    static constexpr size_t  kMinProgressBytes = 256;
    static constexpr size_t  kBufBytes         = 4096;
    static constexpr size_t  kBudgetBytes      = 16384;

    static std::shared_ptr<TxQueue> create(int fd, Kind kind);
    ~TxQueue();
    TxQueue(const TxQueue&) = delete;
    TxQueue& operator=(const TxQueue&) = delete;

    // a und b gehen in dieser Reihenfolge zusammen hinaus. last: danach nimmt
    // die Queue nichts mehr an (WS-CLOSE, RFC 6455 §5.5.1).
    // ESP_OK = gesendet oder gepuffert; ESP_ERR_INVALID_STATE = geschlossen;
    // ESP_ERR_NO_MEM = Puffer reicht nicht, Client getrennt; ESP_FAIL =
    // Verbindung weg.
    esp_err_t write(std::span<const uint8_t> a, std::span<const uint8_t> b = {},
                    bool last = false);

    // Fuer den Task, der gerade eine Anfrage dieses Clients bearbeitet (TCP:
    // sein Pool-Worker, WS: der httpd-Task): wartet auf Abfluss statt zu
    // puffern. Steht schon etwas im Puffer, wartet es, bis der Sende-Task ihn
    // geleert hat, und sendet dann direkt; a und b gehen zusammen hinaus. Der
    // Puffer bleibt so fuer die anderen Schreiber frei. Gewartet wird,
    // solange Daten abfliessen. Fliessen kStallUs lang weniger als
    // kMinProgressBytes ab (beide Schritte zusammen gerechnet), wird der
    // Client getrennt: ESP_ERR_TIMEOUT. Ein Client, der stetig, aber langsam
    // abnimmt, haelt den Aufrufer entsprechend lange auf — deshalb nur fuer
    // Aufrufer, die dabei nichts sperren, was andere Tasks brauchen (Review
    // 0903 H1). budget_us deckelt das: die Zeit in diesem Aufruf geht davon
    // ab; ist es aufgebraucht, wird der Client getrennt (ESP_ERR_TIMEOUT),
    // auch wenn er noch abnimmt. Der WS-Pfad gibt jedem empfangenen Frame ein
    // Budget fuer alle Antworten darauf (Review 0905 F1). Was andere Tasks
    // waehrenddessen schreiben, wird dahinter angehaengt.
    esp_err_t write_blocking(std::span<const uint8_t> a, std::span<const uint8_t> b = {},
                             int64_t* budget_us = nullptr);

    // Vor dem Schliessen des Deskriptors: ab hier sendet niemand mehr darauf.
    void close();

    // Noch nicht abgeflossene Bytes im Puffer.
    size_t pending() const;

    // Online-Update (Review 0903 N4, gemessen 14.10.2026: vier gestaute,
    // flutende Clients liessen den TLS-Download am Heap scheitern, Tiefstmarke
    // 924 B). Solange fn true liefert, vergibt die TxQueue keine Puffer: wer
    // puffern muesste, wird getrennt, wer schon einen hat ebenso (nobuf_drop,
    // Log "shed"). Das trifft auch einen gesunden Client, dessen Socket
    // gerade voll ist oder fuer den eine blockierende Antwort laeuft
    // (writer_active_) — die Toleranz fuer Netzaussetzer gilt in dem Fenster
    // nicht. Der Sende-Task fragt fn je Runde und sonst alle 250 ms (dauerhaft,
    // sobald fn gesetzt ist); fn darf blockieren, aber keinen TxQueue-Mutex
    // brauchen. Vor dem ersten Netz-Client setzen: ein schon im Leerlauf
    // wartender Sende-Task sieht ein spaeter gesetztes fn erst beim naechsten
    // Wecken.
    using ShedFn = bool (*)();
    static void set_shed_fn(ShedFn fn);

    // Zaehler fuer ?stats. stall_drops zaehlt auch Trennungen wegen
    // aufgebrauchten Budgets (write_blocking), nobuf_drops auch die waehrend
    // eines Online-Updates (set_shed_fn).
    static uint32_t stall_drops(Kind k);
    static uint32_t overflow_drops();
    static uint32_t nobuf_drops();
    static uint32_t error_drops();
    static uint32_t peak_bytes();

private:
    TxQueue(int fd, Kind kind);
    friend class TxPump;

    enum class Drop : uint8_t { Stall, Slow, Overflow, NoBuf, Shed, Error };
    enum class Append : uint8_t { Ok, Full, NoBuf, Shed };

    bool   service_(int64_t now, bool shed, int& fd_out);   // Sende-Task
    Append append_(const uint8_t* p, size_t n);
    void   drop_(Drop why, int err = 0);
    void   release_buf_();

    int                  fd_;
    Kind                 kind_;
    SemaphoreHandle_t    mtx_ = nullptr;
    uint8_t*             buf_ = nullptr;
    size_t               head_ = 0;
    size_t               len_ = 0;
    int64_t              stall_since_ = 0;
    size_t               progress_ = 0;
    std::atomic<bool>    closed_{false};       // TxPump liest es ohne mtx_
    bool                 dropped_ = false;
    bool                 last_ = false;
    bool                 writer_active_ = false;   // write_blocking sendet direkt
};

} // namespace culfw32
