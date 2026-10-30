// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "tx_queue.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include <array>
#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG = "txq";
// Mehr Clients als Sockets gibt es nicht (CONFIG_LWIP_MAX_SOCKETS = 24).
constexpr size_t   kMaxQueues  = 24;
// Wartet ein Client auf Platz, sieht der Sende-Task spaetestens so oft nach;
// neue Daten anderer Clients wecken ihn direkt.
constexpr int      kPollUs     = 20 * 1000;
constexpr uint32_t kPumpStack  = 3072;
constexpr UBaseType_t kPumpPrio = 6;      // wie die TcpRxPool-Worker, unter den Funk-Tasks
// Ohne wartende Clients sieht der Sende-Task so oft nach, ob ein Online-Update
// laeuft (set_shed_fn), damit s_shedding nicht veraltet.
constexpr TickType_t kShedPollTicks = pdMS_TO_TICKS(250);

std::atomic<TxQueue::ShedFn> s_shed_fn{nullptr};
std::atomic<bool>     s_shedding{false};   // nur der Sende-Task schreibt
std::atomic<uint32_t> s_stall_drops[2];
std::atomic<uint32_t> s_overflow_drops{0};
std::atomic<uint32_t> s_nobuf_drops{0};
std::atomic<uint32_t> s_error_drops{0};
std::atomic<uint32_t> s_peak{0};
std::atomic<size_t>   s_budget_used{0};
}  // namespace

// Sende-Task: schiebt die Puffer aller Clients nach und trennt, wer staut.
class TxPump {
public:
    static TxPump& instance() {
        static TxPump p;
        return p;
    }

    void add(const std::shared_ptr<TxQueue>& q) {
        xSemaphoreTake(mtx_, portMAX_DELAY);
        for (auto& s : slots_) {
            auto cur = s.lock();
            if (!cur || cur->closed_.load()) {
                s = q;
                xSemaphoreGive(mtx_);
                return;
            }
        }
        xSemaphoreGive(mtx_);
        // Nicht moeglich, solange jede Queue zu einem Socket gehoert.
        ESP_LOGE(TAG, "no free slot for fd=%d", q->fd_);
    }

    void wake() {
        if (task_) xTaskNotifyGive(task_);
    }

private:
    TxPump() {
        mtx_ = xSemaphoreCreateMutex();
        if (xTaskCreate(&TxPump::run_trampoline_, "tx-pump", kPumpStack, this,
                        kPumpPrio, &task_) != pdPASS) {
            ESP_LOGE(TAG, "tx-pump task not created");
            task_ = nullptr;
        }
    }

    static void run_trampoline_(void* arg) { static_cast<TxPump*>(arg)->run_(); }

    void run_() {
        std::array<std::shared_ptr<TxQueue>, kMaxQueues> snap;
        for (;;) {
            // Vor jedem Queue-Mutex fragen: fn darf selbst einen Mutex nehmen.
            const TxQueue::ShedFn fn = s_shed_fn.load();
            const bool shed = fn && fn();
            s_shedding.store(shed);
            size_t n = 0;
            xSemaphoreTake(mtx_, portMAX_DELAY);
            for (auto& s : slots_) {
                if (auto q = s.lock()) snap[n++] = std::move(q);
            }
            xSemaphoreGive(mtx_);
            fd_set wfds;
            FD_ZERO(&wfds);
            int maxfd = -1;
            const int64_t now = esp_timer_get_time();
            for (size_t i = 0; i < n; ++i) {
                int fd = -1;
                if (snap[i]->service_(now, shed, fd) && fd >= 0) {
                    FD_SET(fd, &wfds);
                    if (fd > maxfd) maxfd = fd;
                }
                snap[i].reset();
            }
            if (maxfd >= 0) {
                timeval tv = {};
                tv.tv_usec = kPollUs;
                if (::select(maxfd + 1, nullptr, &wfds, nullptr, &tv) < 0 && errno != EINTR) {
                    // Ein Deskriptor wurde zwischen Liste und select() geschlossen;
                    // nicht im Kreis drehen, die naechste Runde sieht die Queue zu.
                    vTaskDelay(1);
                }
                ulTaskNotifyTake(pdTRUE, 0);
            } else {
                ulTaskNotifyTake(pdTRUE, fn ? kShedPollTicks : portMAX_DELAY);
            }
        }
    }

    SemaphoreHandle_t mtx_ = nullptr;
    TaskHandle_t      task_ = nullptr;
    std::array<std::weak_ptr<TxQueue>, kMaxQueues> slots_;
};

std::shared_ptr<TxQueue> TxQueue::create(int fd, Kind kind) {
    std::shared_ptr<TxQueue> q(new TxQueue(fd, kind));
    TxPump::instance().add(q);
    return q;
}

TxQueue::TxQueue(int fd, Kind kind) : fd_(fd), kind_(kind) {
    mtx_ = xSemaphoreCreateMutex();
}

TxQueue::~TxQueue() {
    release_buf_();
    if (mtx_) vSemaphoreDelete(mtx_);
}

esp_err_t TxQueue::write(std::span<const uint8_t> a, std::span<const uint8_t> b, bool last) {
    xSemaphoreTake(mtx_, portMAX_DELAY);
    if (closed_ || dropped_ || last_) {
        xSemaphoreGive(mtx_);
        return ESP_ERR_INVALID_STATE;
    }
    if (last) last_ = true;
    const bool was_empty = (len_ == 0);
    std::span<const uint8_t> parts[2] = {a, b};
    esp_err_t res = ESP_OK;
    for (auto part : parts) {
        const uint8_t* p = part.data();
        size_t left = part.size();
        // Direkt senden nur, solange nichts im Puffer wartet und kein
        // blockierender Schreiber mitten in seiner Antwort steht (Reihenfolge).
        while (left > 0 && len_ == 0 && !writer_active_) {
            const ssize_t n = ::send(fd_, p, left, MSG_DONTWAIT);
            if (n > 0) {
                p += n;
                left -= static_cast<size_t>(n);
                continue;
            }
            if (n < 0 && errno == EINTR) continue;
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
            drop_(Drop::Error, n < 0 ? errno : EIO);
            res = ESP_FAIL;
            break;
        }
        if (res != ESP_OK) break;
        if (left > 0) {
            const Append r = append_(p, left);
            if (r != Append::Ok) {
                drop_(r == Append::Full   ? Drop::Overflow
                      : r == Append::Shed ? Drop::Shed : Drop::NoBuf);
                res = ESP_ERR_NO_MEM;
                break;
            }
        }
    }
    const bool wake = (res == ESP_OK && was_empty && len_ > 0);
    if (wake) {
        stall_since_ = esp_timer_get_time();
        progress_ = 0;
    }
    xSemaphoreGive(mtx_);
    if (wake) TxPump::instance().wake();
    return res;
}

TxQueue::Append TxQueue::append_(const uint8_t* p, size_t n) {
    if (s_shedding.load()) return Append::Shed;
    if (len_ + n > kBufBytes) return Append::Full;
    if (!buf_) {
        size_t used = s_budget_used.load();
        do {
            if (used + kBufBytes > kBudgetBytes) return Append::NoBuf;
        } while (!s_budget_used.compare_exchange_weak(used, used + kBufBytes));
        buf_ = static_cast<uint8_t*>(std::malloc(kBufBytes));
        if (!buf_) {
            s_budget_used.fetch_sub(kBufBytes);
            return Append::NoBuf;
        }
        head_ = 0;
        len_ = 0;
    }
    size_t tail = (head_ + len_) % kBufBytes;
    const size_t first = (n < kBufBytes - tail) ? n : kBufBytes - tail;
    std::memcpy(buf_ + tail, p, first);
    if (n > first) std::memcpy(buf_, p + first, n - first);
    len_ += n;
    uint32_t peak = s_peak.load();
    while (len_ > peak && !s_peak.compare_exchange_weak(peak, static_cast<uint32_t>(len_))) {}
    return Append::Ok;
}

void TxQueue::release_buf_() {
    if (buf_) {
        std::free(buf_);
        buf_ = nullptr;
        s_budget_used.fetch_sub(kBufBytes);
    }
    head_ = 0;
    len_ = 0;
}

// Unter mtx_. Nur shutdown(): den Deskriptor schliesst der Besitzer.
void TxQueue::drop_(Drop why, int err) {
    if (dropped_) return;
    dropped_ = true;
    const size_t backlog = len_;
    release_buf_();
    const char* kind = kind_ == Kind::Tcp ? "tcp" : "ws";
    switch (why) {
        case Drop::Stall:
        case Drop::Slow:     s_stall_drops[static_cast<int>(kind_)].fetch_add(1); break;
        case Drop::Overflow: s_overflow_drops.fetch_add(1); break;
        case Drop::NoBuf:
        case Drop::Shed:     s_nobuf_drops.fetch_add(1); break;
        case Drop::Error:    s_error_drops.fetch_add(1); break;
    }
    if (why == Drop::Error) {
        // Meist hat der Client die Verbindung schon beendet (ECONNRESET,
        // EPIPE) oder das Netif ist weg (EHOSTUNREACH beim WLAN-Reconnect);
        // nur zaehlen, die Konsole nicht bei jedem Verbindungsende vollschreiben.
        ESP_LOGI(TAG, "%s fd=%d send error %d, disconnecting", kind, fd_, err);
    } else {
        ESP_LOGW(TAG, "%s fd=%d %s (backlog %u B, buffers %u/%u B, heap %u B), disconnecting",
                 kind, fd_,
                 why == Drop::Stall    ? "stalled"
                 : why == Drop::Slow   ? "too slow"
                 : why == Drop::Overflow ? "overflow"
                 : why == Drop::Shed   ? "shed (online update)" : "no buffer",
                 static_cast<unsigned>(backlog),
                 static_cast<unsigned>(s_budget_used.load()),
                 static_cast<unsigned>(kBudgetBytes),
                 static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_DEFAULT)));
    }
    ::shutdown(fd_, SHUT_RDWR);
}

bool TxQueue::service_(int64_t now, bool shed, int& fd_out) {
    xSemaphoreTake(mtx_, portMAX_DELAY);
    bool waiting = false;
    if (shed && !closed_ && !dropped_ && buf_) {
        // Online-Update laeuft: den Puffer fuer den TLS-Download freigeben und
        // keine weiteren Antworten erzeugen. Die lwIP-Sendepuffer bleiben,
        // bis der Peer bestaetigt oder mit RST antwortet: shutdown() schliesst
        // geordnet, ein toter Peer haelt sie ueber die Wiederholungen (Review
        // 0907 F1).
        drop_(Drop::Shed);
    } else if (!closed_ && !dropped_ && !writer_active_ && len_ > 0) {
        size_t sent = 0;
        int err = 0;
        while (len_ > 0) {
            const size_t chunk = (len_ < kBufBytes - head_) ? len_ : kBufBytes - head_;
            const ssize_t n = ::send(fd_, buf_ + head_, chunk, MSG_DONTWAIT);
            if (n > 0) {
                head_ = (head_ + static_cast<size_t>(n)) % kBufBytes;
                len_ -= static_cast<size_t>(n);
                sent += static_cast<size_t>(n);
                continue;
            }
            if (n < 0 && errno == EINTR) continue;
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
            err = (n < 0 && errno != 0) ? errno : EIO;
            break;
        }
        // Auch der letzte Rest zaehlt als Fortschritt: write_blocking uebernimmt
        // die Stau-Uhr, wenn es auf das Leeren gewartet hat.
        progress_ += sent;
        if (progress_ >= kMinProgressBytes) {
            progress_ = 0;
            stall_since_ = now;
        }
        if (err) {
            drop_(Drop::Error, err);
        } else if (len_ == 0) {
            release_buf_();
        } else if (now - stall_since_ >= kStallUs) {
            drop_(Drop::Stall);
        } else {
            fd_out = fd_;
            waiting = true;
        }
    }
    xSemaphoreGive(mtx_);
    return waiting;
}

esp_err_t TxQueue::write_blocking(std::span<const uint8_t> a, std::span<const uint8_t> b,
                                  int64_t* budget_us) {
    if (a.empty() && b.empty()) return ESP_OK;
    const int64_t t_enter = esp_timer_get_time();
    int64_t stall_since = t_enter;
    size_t progress = 0;
    bool waited = false;
    // Die Zeit in diesem Aufruf geht vom Budget ab, auf jedem Rueckweg.
    struct Charge {
        int64_t* budget;
        int64_t  t0;
        ~Charge() {
            if (!budget) return;
            *budget -= esp_timer_get_time() - t0;
            if (*budget < 0) *budget = 0;
        }
    } charge{budget_us, t_enter};
    // Budget aufgebraucht: der Client nimmt zwar ab, aber so langsam, dass der
    // Aufrufer schon zu lange steht (WS: der httpd-Task, Review 0905 F1).
    const auto over_budget = [&](int64_t now) {
        return budget_us && now - t_enter >= *budget_us;
    };
    // 1) Steht etwas im Puffer, warten, bis der Sende-Task ihn geleert hat;
    //    dann selbst direkt senden (2). Die Antwort kommt nie in den Puffer:
    //    der bleibt fuer Funkzeilen und Push. Haengten sich Antworten an
    //    (0.1.903), fuellte ein dump im Aussetzer den Puffer bis zum Rand, und
    //    die naechste Funkzeile trennte den Client als overflow (0.1.904
    //    gemessen). Gewartet wird, solange der Puffer abfliesst; es gilt seine
    //    Stau-Uhr. Laeuft sie ab, wird getrennt, statt die Antwort still zu
    //    verwerfen (Review 0903 M2).
    for (;;) {
        xSemaphoreTake(mtx_, portMAX_DELAY);
        if (closed_ || dropped_ || last_) {
            xSemaphoreGive(mtx_);
            return ESP_ERR_INVALID_STATE;
        }
        const int64_t now = esp_timer_get_time();
        if (over_budget(now)) {
            drop_(Drop::Slow);
            xSemaphoreGive(mtx_);
            return ESP_ERR_TIMEOUT;
        }
        if (len_ == 0 && !writer_active_) {
            writer_active_ = true;
            if (waited) {
                // Der Puffer ist eben leer geworden: seine Stau-Uhr laeuft in
                // Schritt 2 weiter, statt neu anzufangen.
                stall_since = stall_since_;
                progress = progress_;
            } else {
                stall_since = now;
            }
            xSemaphoreGive(mtx_);
            break;
        }
        // Ohne Puffer (anderer blockierender Schreiber aktiv) zaehlt die
        // eigene Wartezeit.
        const bool ring = (len_ > 0);
        const int64_t since = ring ? stall_since_ : t_enter;
        if (now - since >= kStallUs) {
            drop_(Drop::Stall);
            xSemaphoreGive(mtx_);
            return ESP_ERR_TIMEOUT;
        }
        xSemaphoreGive(mtx_);
        if (ring) waited = true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // 2) Direkt senden; andere Schreiber haengen solange an den Puffer an, und
    //    der Sende-Task laesst ihn liegen (writer_active_).
    esp_err_t res = ESP_OK;
    const std::span<const uint8_t> parts[2] = {a, b};
    for (auto part : parts) {
        const uint8_t* p = part.data();
        size_t left = part.size();
        while (left > 0) {
            xSemaphoreTake(mtx_, portMAX_DELAY);
            if (closed_ || dropped_) {
                xSemaphoreGive(mtx_);
                res = ESP_ERR_INVALID_STATE;
                break;
            }
            const int64_t now = esp_timer_get_time();
            if (over_budget(now)) {
                drop_(Drop::Slow);
                xSemaphoreGive(mtx_);
                res = ESP_ERR_TIMEOUT;
                break;
            }
            const ssize_t n = ::send(fd_, p, left, MSG_DONTWAIT);
            const int e = errno;
            if (n > 0) {
                xSemaphoreGive(mtx_);
                p += n;
                left -= static_cast<size_t>(n);
                progress += static_cast<size_t>(n);
                if (progress >= kMinProgressBytes) {
                    progress = 0;
                    stall_since = now;
                }
                continue;
            }
            if (n < 0 && (e == EAGAIN || e == EWOULDBLOCK || e == EINTR)) {
                if (now - stall_since >= kStallUs) {
                    drop_(Drop::Stall);
                    xSemaphoreGive(mtx_);
                    res = ESP_ERR_TIMEOUT;
                    break;
                }
                const int fd = fd_;
                xSemaphoreGive(mtx_);
                fd_set wfds;
                FD_ZERO(&wfds);
                FD_SET(fd, &wfds);
                timeval tv = {};
                tv.tv_usec = kPollUs;
                ::select(fd + 1, nullptr, &wfds, nullptr, &tv);
                continue;
            }
            drop_(Drop::Error, n < 0 ? e : EIO);
            xSemaphoreGive(mtx_);
            res = ESP_FAIL;
            break;
        }
        if (res != ESP_OK) break;
    }
    // 3) Freigeben; was andere inzwischen angehaengt haben, schiebt der
    //    Sende-Task nach, mit frischer Stau-Uhr.
    xSemaphoreTake(mtx_, portMAX_DELAY);
    writer_active_ = false;
    const bool wake = (!closed_ && !dropped_ && len_ > 0);
    if (wake) {
        stall_since_ = esp_timer_get_time();
        progress_ = 0;
    }
    xSemaphoreGive(mtx_);
    if (wake) TxPump::instance().wake();
    return res;
}

void TxQueue::close() {
    xSemaphoreTake(mtx_, portMAX_DELAY);
    closed_ = true;
    release_buf_();
    xSemaphoreGive(mtx_);
}

size_t TxQueue::pending() const {
    xSemaphoreTake(mtx_, portMAX_DELAY);
    const size_t n = len_;
    xSemaphoreGive(mtx_);
    return n;
}

void TxQueue::set_shed_fn(ShedFn fn) { s_shed_fn.store(fn); }

uint32_t TxQueue::stall_drops(Kind k) { return s_stall_drops[static_cast<int>(k)].load(); }
uint32_t TxQueue::overflow_drops()     { return s_overflow_drops.load(); }
uint32_t TxQueue::nobuf_drops()        { return s_nobuf_drops.load(); }
uint32_t TxQueue::error_drops()        { return s_error_drops.load(); }
uint32_t TxQueue::peak_bytes()         { return s_peak.load(); }

} // namespace culfw32
