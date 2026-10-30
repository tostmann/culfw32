// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "ota_update.hpp"
#include "sdkconfig.h"
#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG = "ota";

constexpr uint32_t kBitCheck   = 1u << 0;
constexpr uint32_t kBitInstall = 1u << 1;

constexpr uint32_t kFirstCheckS   = 300;          // Netz und Dienste erst einschwingen lassen
constexpr uint32_t kRetryS        = 600;          // Netz noch nicht da
constexpr uint32_t kPeriodS       = 24 * 3600;    // danach taeglich, plus Zufall:
constexpr uint32_t kJitterS       = 3600;         // nicht alle Sticks zur selben Minute
constexpr size_t   kManifestMax   = 4096;         // das Manifest hat ~0,6 KB
constexpr int      kInstallTimeoutS = 300;
// Rollback-Gate: Probenabstand und wie lange "Link oben, keine Adresse" am
// Stueck anliegen muss, bevor zurueckgerollt wird. Eine einzelne Probe kann
// ein Link sein, der gerade erst kam und dessen DHCP noch laeuft.
constexpr uint32_t kSampleS       = 5;
constexpr uint32_t kNoIpConfirmS  = 60;

struct Accum {
    char   buf[kManifestMax + 1];
    size_t used;
    bool   overflow;
};

esp_err_t http_event(esp_http_client_event_t* evt) {
    if (evt->event_id != HTTP_EVENT_ON_DATA) return ESP_OK;
    auto* a = static_cast<Accum*>(evt->user_data);
    if (!a || a->overflow) return ESP_OK;
    if (a->used + evt->data_len > kManifestMax) { a->overflow = true; return ESP_OK; }
    std::memcpy(a->buf + a->used, evt->data, evt->data_len);
    a->used += evt->data_len;
    a->buf[a->used] = '\0';
    return ESP_OK;
}

bool parse_ver(const char* s, int v[3]) {
    return s && std::sscanf(s, "%d.%d.%d", &v[0], &v[1], &v[2]) == 3;
}

// >0 a neuer als b, 0 gleich, <0 aelter. Unlesbar zaehlt als "nicht neuer".
int cmp_ver(const char* a, const char* b) {
    int x[3], y[3];
    if (!parse_ver(a, x) || !parse_ver(b, y)) return 0;
    for (int i = 0; i < 3; ++i) {
        if (x[i] != y[i]) return x[i] > y[i] ? 1 : -1;
    }
    return 0;
}

// Pfad aus dem Manifest gegen dessen eigene URL aufloesen (absolut, "/..." oder
// relativ zum Verzeichnis des Manifests) — wie esp-web-tools es tut.
void resolve_url(const char* base, const char* path, char* out, size_t cap) {
    if (std::strncmp(path, "http://", 7) == 0 || std::strncmp(path, "https://", 8) == 0) {
        std::snprintf(out, cap, "%s", path);
        return;
    }
    const char* scheme_end = std::strstr(base, "://");
    const char* host_end = scheme_end ? std::strchr(scheme_end + 3, '/') : nullptr;
    if (path[0] == '/') {
        const int n = host_end ? static_cast<int>(host_end - base) : static_cast<int>(std::strlen(base));
        std::snprintf(out, cap, "%.*s%s", n, base, path);
        return;
    }
    const char* dir_end = std::strrchr(base, '/');
    const int n = dir_end ? static_cast<int>(dir_end - base + 1) : 0;
    std::snprintf(out, cap, "%.*s%s", n, base, path);
}

const char* running_version() { return esp_app_get_description()->version; }
}

OtaUpdate& OtaUpdate::instance() {
    static OtaUpdate inst;
    return inst;
}

void OtaUpdate::set_error_(const char* fmt, ...) {
    char msg[sizeof(error_)];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    ESP_LOGW(TAG, "%s", msg);
    xSemaphoreTake(static_cast<SemaphoreHandle_t>(mtx_), portMAX_DELAY);
    std::snprintf(error_, sizeof(error_), "%s", msg);
    state_      = State::Error;
    progress_   = -1;
    checked_us_ = esp_timer_get_time();
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mtx_));
}

void OtaUpdate::set_manifest_url(const char* url) {
    if (!mtx_) return;
    xSemaphoreTake(static_cast<SemaphoreHandle_t>(mtx_), portMAX_DELAY);
    std::snprintf(url_override_, sizeof(url_override_), "%s", url ? url : "");
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mtx_));
}

void OtaUpdate::manifest_url(char* buf, size_t cap) const {
    if (mtx_) xSemaphoreTake(static_cast<SemaphoreHandle_t>(mtx_), portMAX_DELAY);
    std::snprintf(buf, cap, "%s", url_override_[0] ? url_override_ : CONFIG_CULFW32_OTA_MANIFEST_URL);
    if (mtx_) xSemaphoreGive(static_cast<SemaphoreHandle_t>(mtx_));
}

esp_err_t OtaUpdate::start(NetFn net_up, AutoFn auto_install) {
    if (task_) return ESP_OK;
    if (!mtx_) mtx_ = xSemaphoreCreateMutex();
    if (!mtx_) return ESP_ERR_NO_MEM;
    net_up_ = net_up;
    auto_   = auto_install;
    TaskHandle_t h = nullptr;
    // 8 KB wie bei cdc2net: TLS-Handshake plus esp_https_ota. Prioritaet unter
    // den Transporten (5) und den RF-Tasks (8).
    if (xTaskCreate(&OtaUpdate::task_trampoline, "ota", 8192, this, 3, &h) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    task_ = h;
    return ESP_OK;
}

esp_err_t OtaUpdate::request_check() {
    if (!task_) return ESP_ERR_INVALID_STATE;
    xTaskNotify(static_cast<TaskHandle_t>(task_), kBitCheck, eSetBits);
    return ESP_OK;
}

esp_err_t OtaUpdate::request_install() {
    if (!task_) return ESP_ERR_INVALID_STATE;
    if (validating_) return ESP_ERR_INVALID_STATE;   // erst das laufende Image freigeben
    xTaskNotify(static_cast<TaskHandle_t>(task_), kBitInstall, eSetBits);
    return ESP_OK;
}

void OtaUpdate::task_trampoline(void* arg) {
    static_cast<OtaUpdate*>(arg)->run_();
}

void OtaUpdate::run_() {
    uint32_t wait_s = kFirstCheckS;
    for (;;) {
        uint32_t bits = 0;
        const bool woke = xTaskNotifyWait(0, UINT32_MAX, &bits, pdMS_TO_TICKS(wait_s * 1000ull)) == pdTRUE;
        if (!woke) {
            // Planmaessige Pruefung.
            if (net_up_ && !net_up_()) { wait_s = kRetryS; continue; }
            bits = kBitCheck;
            // Solange das laufende Image PENDING_VERIFY ist, nur pruefen:
            // esp_ota_begin() lehnt eine Installation dann ohnehin ab
            // (ESP_ERR_OTA_ROLLBACK_INVALID_STATE, esp_ota_ops.c) — das gaebe
            // nur eine irrefuehrende Fehlerzeile in ?ota. request_install()
            // prueft dasselbe fuer den manuellen Weg.
            if (auto_ && auto_() && !validating_) bits |= kBitInstall;
            wait_s = kPeriodS + esp_random() % kJitterS;
        }
        if (bits & kBitInstall) {
            // Install prueft selbst vorher, ob etwas Neueres da ist.
            install_();
        } else if (bits & kBitCheck) {
            check_();
        }
    }
}

esp_err_t OtaUpdate::check_() {
    char url[sizeof(url_override_)];
    manifest_url(url, sizeof(url));
    xSemaphoreTake(static_cast<SemaphoreHandle_t>(mtx_), portMAX_DELAY);
    state_ = State::Checking;
    error_[0] = '\0';
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mtx_));

    auto* acc = static_cast<Accum*>(std::calloc(1, sizeof(Accum)));
    if (!acc) { set_error_("no memory"); return ESP_ERR_NO_MEM; }
    esp_http_client_config_t cfg = {};
    cfg.url               = url;
    cfg.timeout_ms        = 8000;
    cfg.event_handler     = &http_event;
    cfg.user_data         = acc;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.keep_alive_enable = false;
    esp_http_client_handle_t cli = esp_http_client_init(&cfg);
    if (!cli) { std::free(acc); set_error_("http client init failed"); return ESP_FAIL; }
    esp_err_t e = esp_http_client_perform(cli);
    const int code = esp_http_client_get_status_code(cli);
    esp_http_client_cleanup(cli);
    if (e != ESP_OK)     { std::free(acc); set_error_("fetch: %s", esp_err_to_name(e)); return e; }
    if (code != 200)     { std::free(acc); set_error_("manifest HTTP %d", code); return ESP_FAIL; }
    if (acc->overflow || acc->used == 0) {
        const bool overflow = acc->overflow;
        std::free(acc);
        set_error_("%s", overflow ? "manifest too large" : "empty manifest");
        return ESP_FAIL;
    }

    cJSON* root = cJSON_Parse(acc->buf);
    std::free(acc);
    if (!root) { set_error_("manifest: JSON parse failed"); return ESP_FAIL; }
    const cJSON* ver  = cJSON_GetObjectItemCaseSensitive(root, "version");
    const cJSON* ota  = cJSON_GetObjectItemCaseSensitive(root, "ota");
    const cJSON* chip = ota ? cJSON_GetObjectItemCaseSensitive(ota, CONFIG_CULFW32_OTA_CHIP_FAMILY) : nullptr;
    const cJSON* path = chip ? cJSON_GetObjectItemCaseSensitive(chip, "path") : nullptr;
    if (!cJSON_IsString(ver) || !cJSON_IsString(path)) {
        cJSON_Delete(root);
        set_error_("manifest: no version or ota.%s.path", CONFIG_CULFW32_OTA_CHIP_FAMILY);
        return ESP_FAIL;
    }
    char fw[sizeof(fw_url_)];
    resolve_url(url, path->valuestring, fw, sizeof(fw));
    const bool newer = cmp_ver(ver->valuestring, running_version()) > 0;

    xSemaphoreTake(static_cast<SemaphoreHandle_t>(mtx_), portMAX_DELAY);
    std::snprintf(latest_, sizeof(latest_), "%s", ver->valuestring);
    std::snprintf(fw_url_, sizeof(fw_url_), "%s", fw);
    state_      = newer ? State::Available : State::UpToDate;
    checked_us_ = esp_timer_get_time();
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mtx_));
    cJSON_Delete(root);
    ESP_LOGI(TAG, "manifest %s: running %s -> %s", latest_, running_version(),
             newer ? "update available" : "up to date");
    return ESP_OK;
}

esp_err_t OtaUpdate::install_() {
    if (esp_err_t e = check_(); e != ESP_OK) return e;
    char fw[sizeof(fw_url_)];
    xSemaphoreTake(static_cast<SemaphoreHandle_t>(mtx_), portMAX_DELAY);
    const bool avail = state_ == State::Available;
    std::snprintf(fw, sizeof(fw), "%s", fw_url_);
    if (avail) { state_ = State::Installing; progress_ = 0; }
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mtx_));
    if (!avail) return ESP_OK;   // nichts Neueres — check_() hat den Zustand gesetzt

    ESP_LOGW(TAG, "installing from %s", fw);
    esp_http_client_config_t http = {};
    http.url               = fw;
    http.timeout_ms        = 20000;
    http.crt_bundle_attach = esp_crt_bundle_attach;
    http.keep_alive_enable = true;
    esp_https_ota_config_t cfg = {};
    cfg.http_config = &http;

    esp_https_ota_handle_t h = nullptr;
    esp_err_t e = esp_https_ota_begin(&cfg, &h);
    if (e != ESP_OK || !h) { set_error_("ota begin: %s", esp_err_to_name(e)); return e; }

    // Waechter 1: es muss culfw32 sein und neuer als das laufende Image. Das
    // Lesen des Kopfes schreibt noch nichts — ein Abbruch hier kostet nichts.
    esp_app_desc_t img = {};
    if (e = esp_https_ota_get_img_desc(h, &img); e != ESP_OK) {
        esp_https_ota_abort(h);
        set_error_("image header unreadable");
        return e;
    }
    const esp_app_desc_t* self = esp_app_get_description();
    if (std::strncmp(img.project_name, self->project_name, sizeof(img.project_name)) != 0) {
        esp_https_ota_abort(h);
        set_error_("foreign image '%.*s'", static_cast<int>(sizeof(img.project_name)), img.project_name);
        return ESP_ERR_INVALID_ARG;
    }
    char img_ver[sizeof(img.version) + 1] = {};
    std::memcpy(img_ver, img.version, sizeof(img.version));
    if (cmp_ver(img_ver, self->version) <= 0) {
        esp_https_ota_abort(h);
        set_error_("image %s is not newer than %s", img_ver, self->version);
        return ESP_ERR_INVALID_VERSION;
    }
    // Waechter 1b: ein Image, das auf diesem Geraet schon einmal zurueckgerollt
    // wurde (otadata: INVALID/ABORTED), wird nicht erneut geholt. Sonst
    // installiert `ota.auto=1` dasselbe Image bei jeder planmaessigen Pruefung
    // wieder — jeder Versuch kostet einen Neustart plus kMaxWaitS Ausfall bis
    // zum naechsten Rollback. Muster: IDF examples/system/ota/native_ota_example
    // (esp_ota_get_last_invalid_partition + Versionsvergleich).
    if (const esp_partition_t* bad = esp_ota_get_last_invalid_partition(); bad) {
        esp_app_desc_t bad_desc = {};
        if (esp_ota_get_partition_description(bad, &bad_desc) == ESP_OK &&
            std::memcmp(bad_desc.version, img.version, sizeof(img.version)) == 0) {
            esp_https_ota_abort(h);
            set_error_("image %s was rolled back on this device before", img_ver);
            return ESP_ERR_INVALID_VERSION;
        }
    }

    // Waechter 2: die Chip-ID prueft IDF im ersten perform() VOR dem ersten
    // Schreiben (ESP_ERR_INVALID_VERSION) — ein fremdes Image kommt nie an.
    const int64_t deadline = esp_timer_get_time() + static_cast<int64_t>(kInstallTimeoutS) * 1000000;
    while ((e = esp_https_ota_perform(h)) == ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
        if (esp_timer_get_time() > deadline) { e = ESP_ERR_TIMEOUT; break; }
        const int size = esp_https_ota_get_image_size(h);
        const int got  = esp_https_ota_get_image_len_read(h);
        if (size > 0) progress_ = got * 100 / size;
    }
    if (e != ESP_OK) {
        esp_https_ota_abort(h);
        if (e == ESP_ERR_INVALID_VERSION)  set_error_("incompatible image (chip id/revision)");
        else if (e == ESP_ERR_TIMEOUT)     set_error_("download stalled");
        else                               set_error_("download: %s", esp_err_to_name(e));
        return e;
    }
    if (!esp_https_ota_is_complete_data_received(h)) {
        esp_https_ota_abort(h);
        set_error_("incomplete download");
        return ESP_FAIL;
    }
    // finish() prueft das ganze Image (SHA-256) und setzt die Boot-Partition;
    // es verbraucht den Handle — danach kein abort() mehr.
    if (e = esp_https_ota_finish(h); e != ESP_OK) {
        if (e == ESP_ERR_OTA_VALIDATE_FAILED) set_error_("image validation failed");
        else                                  set_error_("finish: %s", esp_err_to_name(e));
        return e;
    }
    progress_ = 100;
    ESP_LOGW(TAG, "update to %s installed - rebooting", img_ver);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

void OtaUpdate::status_line(char* buf, size_t cap) const {
    if (mtx_) xSemaphoreTake(static_cast<SemaphoreHandle_t>(mtx_), portMAX_DELAY);
    const State st = state_;
    char latest[sizeof(latest_)], err[sizeof(error_)];
    std::memcpy(latest, latest_, sizeof(latest));
    std::memcpy(err, error_, sizeof(err));
    const int prog = progress_.load();
    const int64_t age_s = checked_us_ ? (esp_timer_get_time() - checked_us_) / 1000000 : -1;
    if (mtx_) xSemaphoreGive(static_cast<SemaphoreHandle_t>(mtx_));

    const char* run = running_version();
    if (validating_) {
        std::snprintf(buf, cap, "validating %s (needs %u s stable run; rollback only if the "
                      "link is up but no address within %u s)", run,
                      static_cast<unsigned>(kMinRunS), static_cast<unsigned>(kMaxWaitS));
        return;
    }
    switch (st) {
        case State::Idle:       std::snprintf(buf, cap, "idle, running %s (check|install)", run); break;
        case State::Checking:   std::snprintf(buf, cap, "checking..."); break;
        case State::UpToDate:   std::snprintf(buf, cap, "up-to-date %s (checked %llds ago)", run,
                                              static_cast<long long>(age_s)); break;
        case State::Available:  std::snprintf(buf, cap, "available %s (running %s) - ?ota=install",
                                              latest, run); break;
        case State::Installing: std::snprintf(buf, cap, "installing %s %d%%", latest, prog < 0 ? 0 : prog); break;
        case State::Error:      std::snprintf(buf, cap, "error: %s (running %s)", err, run); break;
    }
}

const char* OtaUpdate::state_word() const {
    if (validating_) return "validating";
    if (mtx_) xSemaphoreTake(static_cast<SemaphoreHandle_t>(mtx_), portMAX_DELAY);
    const State st = state_;
    if (mtx_) xSemaphoreGive(static_cast<SemaphoreHandle_t>(mtx_));
    switch (st) {
        case State::Idle:       return "idle";
        case State::Checking:   return "checking";
        case State::UpToDate:   return "uptodate";
        case State::Available:  return "available";
        case State::Installing: return "installing";
        case State::Error:      return "error";
    }
    return "idle";
}

void OtaUpdate::latest_version(char* buf, size_t cap) const {
    if (mtx_) xSemaphoreTake(static_cast<SemaphoreHandle_t>(mtx_), portMAX_DELAY);
    std::snprintf(buf, cap, "%s", latest_);
    if (mtx_) xSemaphoreGive(static_cast<SemaphoreHandle_t>(mtx_));
}

int OtaUpdate::progress() const {
    if (mtx_) xSemaphoreTake(static_cast<SemaphoreHandle_t>(mtx_), portMAX_DELAY);
    const int p = state_ == State::Installing ? progress_.load() : -1;
    if (mtx_) xSemaphoreGive(static_cast<SemaphoreHandle_t>(mtx_));
    return p;
}

bool OtaUpdate::busy() const {
    if (mtx_) xSemaphoreTake(static_cast<SemaphoreHandle_t>(mtx_), portMAX_DELAY);
    const bool b = state_ == State::Checking || state_ == State::Installing;
    if (mtx_) xSemaphoreGive(static_cast<SemaphoreHandle_t>(mtx_));
    return b;
}

void OtaUpdate::start_validation(HealthFn health) {
    const esp_partition_t* run = esp_ota_get_running_partition();
    esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
    if (!run || esp_ota_get_state_partition(run, &st) != ESP_OK || st != ESP_OTA_IMG_PENDING_VERIFY) {
        return;   // per USB/Web-Flasher installiert oder schon freigegeben
    }
    validating_ = true;
#if CONFIG_CULFW32_OTA_TEST_CRASH_S > 0
    xTaskCreate([](void*) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_CULFW32_OTA_TEST_CRASH_S * 1000));
        ESP_LOGE(TAG, "TEST build: crashing on purpose while pending verify");
        abort();
    }, "ota-crash", 2048, nullptr, 2, nullptr);
#endif
    struct Ctx { OtaUpdate* self; HealthFn health; };
    static Ctx ctx;
    ctx = {this, health};
    xTaskCreate([](void* arg) {
        auto* c = static_cast<Ctx*>(arg);
        ESP_LOGW(TAG, "new image %s pending verify", running_version());
        bool     link_seen  = false;
        uint32_t noip_since = 0;   // Uptime [s] der ersten LinkNoIp-Probe der laufenden Serie, 0 = keine
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(kSampleS * 1000));
            const uint32_t up_s = static_cast<uint32_t>(esp_timer_get_time() / 1000000);
            const NetHealth h = c->health ? c->health() : NetHealth::Ok;
            if (h == NetHealth::LinkNoIp) {
                link_seen = true;
                if (!noip_since) noip_since = up_s;
            } else {
                noip_since = 0;
            }
            if (up_s < kMinRunS) continue;

            const char* why = nullptr;
            if (h == NetHealth::Ok)                                   why = "network ok";
            else if (!link_seen)                                      why = "no link since boot, cannot judge the image";
            else if (up_s >= kMaxWaitS && h == NetHealth::NoLink)     why = "link lost, cannot judge the image";
            if (why) {
                esp_ota_mark_app_valid_cancel_rollback();
                c->self->validating_ = false;
                ESP_LOGW(TAG, "image %s marked valid after %u s (%s)", running_version(),
                         static_cast<unsigned>(up_s), why);
                break;
            }
            // Ab hier: Verbindungsschicht steht, aber keine Adresse. Zurueck
            // auf das vorige Image erst, wenn das kMaxWaitS Uptime UND
            // kNoIpConfirmS am Stueck so war -- genau der Fall, in dem das
            // Image selbst das Netz verliert.
            if (up_s >= kMaxWaitS && noip_since && up_s - noip_since >= kNoIpConfirmS) {
                ESP_LOGE(TAG, "image %s: link up but no address for %u s (uptime %u s) - rolling back",
                         running_version(), static_cast<unsigned>(up_s - noip_since),
                         static_cast<unsigned>(up_s));
                const esp_err_t e = esp_ota_mark_app_invalid_rollback_and_reboot();
                // Kehrt nur zurueck, wenn es kein Ziel gibt (anderer Slot ohne
                // gueltiges Abbild, esp_ota_check_rollback_is_possible). Dann
                // das laufende Image behalten, statt alle kSampleS erneut zu
                // scheitern und ?ota=install dauerhaft zu sperren.
                ESP_LOGE(TAG, "rollback not possible (%s) - keeping %s",
                         esp_err_to_name(e), running_version());
                esp_ota_mark_app_valid_cancel_rollback();
                c->self->validating_ = false;
                break;
            }
        }
        vTaskDelete(nullptr);
    }, "ota-valid", 4096, &ctx, 2, nullptr);   // Review N11: Flash-Write + Log, 3 KB nie vermessen
}

} // namespace culfw32
