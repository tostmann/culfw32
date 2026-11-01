// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "hmuartlgw_frontend.hpp"

#include "rf_manager.hpp"
#include "rf_protocol.hpp"
#include "asksin_rf.hpp"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "nvs_eeprom.hpp"
#include "culfw_ee_addr.hpp"
#include <cstdio>
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG = "fe-hmu";

constexpr ProtocolId kCompatible[] = { ProtocolId::AskSin };

// COMMON-Layer (DualCoPro/RPI-RF-MOD) — pivccu hmframe.h:79-84.
constexpr uint8_t COMMON_IDENTIFY     = 0x01;
constexpr uint8_t COMMON_START_BL     = 0x02;
constexpr uint8_t COMMON_START_APP    = 0x03;
constexpr uint8_t COMMON_GET_SGTIN    = 0x04;
constexpr uint8_t COMMON_ACK          = 0x05;
// TRX-Layer (DualCoPro/RPI-RF-MOD).
constexpr uint8_t TRX_GET_VERSION         = 0x02;
constexpr uint8_t TRX_ACK                 = 0x04;
constexpr uint8_t TRX_GET_MCU_TYPE        = 0x09;
constexpr uint8_t TRX_GET_DEFAULT_RF_ADDR = 0x10;
// HMIP-Layer (DualCoPro). Werte wie in hmframe.h / radiomoduledetector.cpp der
// HB-RF-ETH-Firmware (A. Reinert, Apache-2.0); die Antwort 0x06 mit Status 01
// und den Empfangsrahmen 0x07 zeigt auch der Verkehr eines echten RPI-RF-MOD.
constexpr uint8_t HMIP_GET_DEFAULT_RF_ADDR = 0x01;
constexpr uint8_t HMIP_RECEIVED_EVENT      = 0x07;
constexpr uint8_t HMIP_ACK                 = 0x06;
// LLMAC-Layer (DualCoPro BidCoS-Address-Pfad).
constexpr uint8_t LLMAC_ACK                = 0x01;
constexpr uint8_t LLMAC_GET_SERIAL         = 0x07;
constexpr uint8_t LLMAC_GET_DEFAULT_RF_ADDR= 0x08;
// Module-Type-Enum byte-genau pivccu radiomoduledetector.h:24.
constexpr uint8_t MODULE_TYPE_HMIP_RFUSB     = 1;
constexpr uint8_t MODULE_TYPE_HM_MOD_RPI_PCB = 3;
constexpr uint8_t MODULE_TYPE_RPI_RF_MOD     = 4;

// HMUARTLGW Befehlscodes (siehe hmuartlgw_protocol_spec.md).
constexpr uint8_t OS_GET_APP          = 0x00;
constexpr uint8_t OS_GET_FIRMWARE     = 0x02;
constexpr uint8_t OS_CHANGE_APP       = 0x03;
constexpr uint8_t OS_UPDATE_FIRMWARE  = 0x05;
constexpr uint8_t OS_NORMAL_MODE      = 0x06;
constexpr uint8_t OS_UPDATE_MODE      = 0x07;
constexpr uint8_t OS_GET_CREDITS      = 0x08;
constexpr uint8_t OS_ENABLE_CREDITS   = 0x09;
constexpr uint8_t OS_ENABLE_CSMACA    = 0x0A;
constexpr uint8_t OS_GET_SERIAL       = 0x0B;
constexpr uint8_t OS_SET_TIME         = 0x0E;

constexpr uint8_t APP_SET_HMID            = 0x00;
constexpr uint8_t APP_GET_HMID            = 0x01;
constexpr uint8_t APP_SEND                = 0x02;
constexpr uint8_t APP_SET_CURRENT_KEY     = 0x03;
constexpr uint8_t APP_RECV                = 0x05;
constexpr uint8_t APP_ADD_PEER            = 0x06;
constexpr uint8_t APP_REMOVE_PEER         = 0x07;
constexpr uint8_t APP_SET_TEMP_KEY        = 0x0B;
constexpr uint8_t APP_SET_PREVIOUS_KEY    = 0x0F;
constexpr uint8_t APP_DEFAULT_HMID        = 0x10;
// HB-RF-ETH-Detection-Probes (alexreinert/HB-RF-ETH src/radiomoduledetector.cpp).
// Wir antworten silent-timeout bei dst=COMMON (0xfe) — HB-RF-ETH faellt dann
// auf den HMSYSTEM-Pfad zurueck und akzeptiert uns als Legacy CoPro.
// `TRX_GET_VERSION (0x02)` = unser APP_SEND und `TRX_GET_DEFAULT_RF_ADDR
// (0x10)` = APP_DEFAULT_HMID — schon richtig abgedeckt. Nur:
constexpr uint8_t APP_TRX_GET_MCU_TYPE    = 0x09;  // HB-RF-ETH-spezifisch — explizit NACK

constexpr uint8_t ACK             = 0x04;
constexpr uint8_t ACK_OK          = 0x01;
constexpr uint8_t ACK_INFO        = 0x02;
constexpr uint8_t ACK_WITH_RESPONSE = 0x03;   // gesendet, Antwort des Geraets folgt im Rahmen
constexpr uint8_t ACK_EUNKNOWN    = 0x04;
constexpr uint8_t ACK_ENOCREDITS  = 0x05;
constexpr uint8_t ACK_ECSMACA     = 0x06;
constexpr uint8_t ACK_MULTIPART   = 0x07;
constexpr uint8_t ACK_EINPROGRESS = 0x08;     // beschaeftigt, FHEM wiederholt nach 50 ms

// BidCoS-Rahmen ab dem Laengenbyte: [0]=LEN [1]=CNT [2]=FLAG [3]=TYP
// [4..6]=Absender [7..9]=Ziel.
constexpr uint8_t BIDCOS_FLAG_BIDI = 0x20;   // Absender erwartet Quittung
constexpr uint8_t BIDCOS_FLAG_WKUP = 0x01;   // in der Quittung: Empfaenger bleibt wach
constexpr uint8_t BIDCOS_TYPE_ACK  = 0x02;

// RSSI im Format des Moduls: Betrag in dBm, FHEM rechnet `0 - hex()`
// (00_HMUARTLGW.pm). Bis 0.2.2 ging das rohe CC1101-Byte hinaus: 0x41 wurde
// zu -65 statt -41,5 dBm, ab 0x80 zu Werten um -200.
uint8_t hmu_rssi(uint8_t raw) {
    const int dbm = cc1101_rssi_dbm(raw);   // -138..-11
    return static_cast<uint8_t>(-dbm);
}

// Forward — Definition unten direkt nach send_hello_.
HmuFrame make_co_cpu_app_push(uint8_t cnt);

} // anon

std::span<const ProtocolId> HmuartlgwFrontend::compatible_protocols() const {
    return std::span<const ProtocolId>(kCompatible, std::size(kCompatible));
}

void HmuartlgwFrontend::attach(const FrontendServices& services) {
    services_ = services;
    if (!mtx_) mtx_ = xSemaphoreCreateMutex();
    if (!peer_mtx_) peer_mtx_ = xSemaphoreCreateMutex();
    if (!job_mtx_) job_mtx_ = xSemaphoreCreateMutex();

    // HMID aus EE laden — falls 0xFFFFFF (noch nie provisioned), bei
    // 0xCAFE42 (Bringup-Default) bleiben damit FHEM einen lesbaren Wert sieht.
    if (services_.eeprom) {
        uint8_t buf[3] = {};
        services_.eeprom->read_block(ee::HMU_HMID, std::span<uint8_t>(buf, 3));
        if (!(buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF)) {
            std::memcpy(hmid_, buf, 3);
        }
    }
    // Serial aus MAC ableiten — letzte 4 MAC-Hex-Bytes als Suffix.
    uint8_t mac[6] = {};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        std::snprintf(serial_, sizeof(serial_), "CUL%02X%02X%02X%02X",
                      mac[2], mac[3], mac[4], mac[5]);
    }
    // Module-Mode aus EE laden. 0xFF (Erst-Boot) → Legacy (FHEM-Default).
    if (services_.eeprom) {
        uint8_t m = services_.eeprom->read_byte(ee::HMU_MODULE_MODE);
        module_mode_ = (m == ee::HMU_MODULE_DUAL) ? ModuleMode::Dual
                                                  : ModuleMode::Legacy;
    }
}

void HmuartlgwFrontend::set_module_mode(ModuleMode m) {
    module_mode_ = m;
    if (services_.eeprom) {
        services_.eeprom->write_byte(ee::HMU_MODULE_MODE, static_cast<uint8_t>(m));
    }
    ESP_LOGI(TAG, "module-mode → %s", m == ModuleMode::Dual ? "rpi-rf-mod (DualCoPro)"
                                                            : "hm-mod-rpi-pcb (Legacy)");
}

void HmuartlgwFrontend::bind_transport(std::shared_ptr<Transport> t) {
    if (!t || !mtx_) return;
    xSemaphoreTake(mtx_, portMAX_DELAY);
    transports_.push_back(t);
    xSemaphoreGive(mtx_);
}

void HmuartlgwFrontend::attach_dynamic_transport(std::shared_ptr<Transport> t) {
    attach_dynamic_transport(t, module_mode_);
}

void HmuartlgwFrontend::attach_dynamic_transport(std::shared_ptr<Transport> t, ModuleMode mode) {
    if (!t) return;
    bind_transport(t);
    Transport* raw = t.get();
    // Slot-Eintrag pre-allozieren mit Mode-Stempel — sonst hat slot_for_ den
    // Slot erst nach erstem on_rx_data_ (HostMode-Detection vergibt erst dann
    // einen Slot via slot_for_/insert). Wir wollen aber ZUERST die Mode-Info
    // setzen, damit der Hello-Push schon den richtigen "Co_CPU_BL"
    // (Legacy) bzw. den DualCoPro-Hello sendet.
    if (mtx_) {
        xSemaphoreTake(mtx_, portMAX_DELAY);
        for (auto& s : per_transport_) {
            if (s.owner == nullptr) {
                s.owner  = raw;
                s.mode   = HostMode::AwaitingProbe;
                s.module = mode;
                s.radio_claimed = false;
                s.decoder.reset();
                break;
            }
        }
        xSemaphoreGive(mtx_);
    }
    t->on_rx([this, raw](std::span<const uint8_t> bytes) {
        on_rx_data_(bytes, raw);
    });
    if (started_) {
        // Fehlerpfad wie im JSON-Frontend; detach gibt auch den oben vorab
        // belegten per_transport_-Slot wieder frei.
        if (t->start() != ESP_OK) {
            detach_dynamic_transport(raw);
            return;
        }
        // Frueher: 200 ms-delayed unsolicited "Co_CPU_BL"/"HMIP_TRX_Bl"-Push.
        // Bricht rfd::CCU2CommController::improvedInit() — sieht den Banner
        // als Identify-Response und crasht mit "Identify response string not
        // handled" (siehe notes/rfd_identify_unsolicited_push_2026-05-13.md).
        // Echte HM-MOD-RPI-PCB / RPI-RF-MOD senden den Banner NUR als Antwort
        // auf COMMON_IDENTIFY (dst=0xfe cmd=0x01). Der Push wird jetzt im
        // dispatch_-Pfad bei COMMON_IDENTIFY synchron emittiert.
    }
}

void HmuartlgwFrontend::detach_dynamic_transport(Transport* t) {
    if (!t || !mtx_) return;
    xSemaphoreTake(mtx_, portMAX_DELAY);
    for (size_t i = 0; i < transports_.size(); ++i) {
        if (transports_[i].get() == t) {
            transports_.erase(transports_.begin() + i);
            break;
        }
    }
    void* timer_to_kill = nullptr;
    bool legacy_left = false;
    for (auto& pt : per_transport_) {
        if (pt.owner == t) {
            pt.owner = nullptr;
            pt.radio_claimed = false;
            pt.decoder.reset();
            // Pending Hello-Timer fuer diesen Slot stoppen+loeschen — sonst
            // feuert das Trampoline asynchron gegen den schon zerstoerten
            // Transport (UAF) oder auf einen wieder benutzten Address-Slot
            // (ABA → doppelte Hello-Pushes).
            if (pt.hello_timer) {
                timer_to_kill = pt.hello_timer;
                pt.hello_timer = nullptr;
            }
        } else if (pt.owner && pt.module == ModuleMode::Legacy && pt.radio_claimed) {
            // Nur Verbindungen, die schon einen gueltigen Rahmen geschickt
            // haben: eine stumme auf 2325 haelt die Tabelle nicht fest.
            legacy_left = true;
        }
    }
    // Ein offener Sendeauftrag dieser Verbindung entfaellt (der Sende-Task
    // hoert dann auf zu wiederholen).
    if (job_mtx_) {
        xSemaphoreTake(job_mtx_, portMAX_DELAY);
        if (job_.active && job_.owner == t) job_.active = false;
        xSemaphoreGive(job_mtx_);
    }
    // Ohne Legacy-Verbindung quittiert niemand mehr selbst: ein spaeterer
    // Klient (etwa ein CUL-Device auf 2323) quittiert selbst, eine zweite
    // Quittung waere doppelt. FHEM traegt nach dem naechsten Init neu ein.
    if (!legacy_left && peer_mtx_) {
        xSemaphoreTake(peer_mtx_, portMAX_DELAY);
        if (peer_count_) ESP_LOGI(TAG, "peer table cleared (%u)", static_cast<unsigned>(peer_count_));
        peer_count_ = 0;
        xSemaphoreGive(peer_mtx_);
    }
    xSemaphoreGive(mtx_);
    if (timer_to_kill) {
        auto h = static_cast<esp_timer_handle_t>(timer_to_kill);
        esp_timer_stop(h);
        esp_timer_delete(h);
    }
}

esp_err_t HmuartlgwFrontend::start() {
    if (started_) return ESP_OK;
    started_ = true;
    for (auto& t : transports_) {
        Transport* raw = t.get();
        t->on_rx([this, raw](std::span<const uint8_t> bytes) {
            on_rx_data_(bytes, raw);
        });
        t->start();
        // Kein unsolicited Connect-Time-Push mehr (rfd-improvedInit-Kompat,
        // siehe attach_dynamic_transport-Kommentar). Banner kommt jetzt als
        // Reply auf COMMON_IDENTIFY im dispatch_-Pfad.
    }

    // Phase 2 wiring: AskSinRF auf Slot 0 aktivieren (HMUARTLGW pinnt sich
    // dort fest), RX-Events des Manager-Fanout abonnieren. Schlaegt fehl wenn
    // CC1101 nicht ansprechbar — das logged der Manager selbst, wir bleiben
    // dann ohne RF-Pfad aktiv (Init-Sequenz mit FHEM funktioniert weiter,
    // nur APP_SEND/APP_RECV liefern keine Funk-Frames).
    if (services_.rf_manager) {
        ProtocolId target = ProtocolId::AskSin;
        if (services_.rf_manager->activate(target, 0) != ESP_OK) {
            ESP_LOGW(TAG, "RF activation on slot 0 failed (target=%u)", static_cast<unsigned>(target));
        }
        services_.rf_manager->subscribe(ProtocolId::AskSin,
            [this](const RxEvent& ev) { on_asksin_rx_(ev); });
    }

    // Sende-Task fuer Auftraege mit Quittungswunsch (siehe SendJob). Prio 7:
    // unter dem AskSin-RX-Task (8), der die Antworten liefert, ueber den
    // Transport-Tasks. 5 KB wie der RX-Task, der auf demselben Weg sendet.
    if (!send_task_ && job_mtx_) {
        TaskHandle_t h = nullptr;
        if (xTaskCreate(&HmuartlgwFrontend::send_task_trampoline_, "hmu-send", 5120,
                        this, 7, &h) == pdPASS) {
            send_task_ = h;
        } else {
            ESP_LOGW(TAG, "send task not created, APP_SEND answers 04 01 at once");
        }
    }

    ESP_LOGI(TAG, "ready, %u transport(s)", static_cast<unsigned>(transports_.size()));
    return ESP_OK;
}

esp_err_t HmuartlgwFrontend::stop() {
    started_ = false;
    return ESP_OK;
}

void HmuartlgwFrontend::on_rx_data_(std::span<const uint8_t> bytes, Transport* src) {
    if (!mtx_) return;
    // F6 (Code-Review 0.1.802): mtx_ wird ueber die GESAMTE try_pop/dispatch_-
    // Schleife gehalten — inkl. dispatch_ -> APP_SEND -> handle_app_send_ ->
    // asksin_send(), also einem on-air-RF-TX (CCA + Sende-Airtime, ~zig ms).
    // Folge: ein zweiter Transport (Zwei-Port-Setup :2325/:2327) blockiert in
    // seinem on_rx_data_ fuer dieses Fenster. Das ist reine Prioritaets-
    // Inversion / Latenz — KEIN Deadlock. mtx_ ist non-recursive; ausser hier
    // nimmt ihn nur on_asksin_rx_ kurz fuer den Transport-Snapshot, und der
    // RX-Task haelt dabei keine RF-Sperre (AskSinRF::drain_fifo gibt die
    // Chipsperre vor dem Callback frei). Reihenfolge: mtx_ -> Slot -> Chip ->
    // Bus, und mtx_ -> peer_mtx_. Der RX-Task nimmt selbst nur Chip -> Bus und
    // betritt Frontend-Code nur ohne Chipsperre (Review 0202 F2). Im Callback
    // nimmt er nacheinander, nie verschachtelt: peer_mtx_ allein (auto_ack_
    // prueft), Slot -> Chip -> Bus (Selbstquittung, Frist bis 200 ms nach dem Rahmen), mtx_ allein
    // (Snapshot). claim_radio_() schaltet unter mtx_ um (mtx_ -> Slot, wie
    // APP_SEND).
    // Sauberer Fix (deferred bis Build-/Regressions-Env wieder steht): in
    // handle_app_send_ die per-Transport-State-Mutation (pt->llmac_tx_cnt_++) +
    // Frame-Bau unter dem Lock erledigen, Lock freigeben, DANN asksin_send()
    // ausserhalb. Achtung: byte-genaue ACK-Reihenfolge (LLMAC opts[3]->on-air-
    // CNT, hmuartlgw_phase4) muss erhalten bleiben — daher nicht blind umbauen.
    xSemaphoreTake(mtx_, portMAX_DELAY);

    PerTransport* pt = nullptr;
    for (auto& slot : per_transport_) {
        if (slot.owner == src) { pt = &slot; break; }
    }
    if (!pt) {
        for (auto& slot : per_transport_) {
            if (!slot.owner) {
                slot.owner = src;
                slot.radio_claimed = false;
                pt = &slot;
                break;
            }
        }
    }
    if (!pt) {
        xSemaphoreGive(mtx_);
        ESP_LOGW(TAG, "out of per-transport slots");
        return;
    }

    pt->decoder.feed(bytes);
    HmuFrame frame;
    while (pt->decoder.try_pop(frame)) {
        if (!pt->radio_claimed) {
            pt->radio_claimed = true;
            claim_radio_();
        }
        dispatch_(frame, src);
    }

    xSemaphoreGive(mtx_);
}

void HmuartlgwFrontend::dispatch_(const HmuFrame& req, Transport* src) {
    // Host-Mode-Detection: erste OS-Probe entscheidet. Wir sind hier unter
    // mtx_ (gehalten von on_rx_data_), Zugriff auf per_transport_ ist sicher.
    PerTransport* pt = slot_for_(src);
    // Per-Slot Module-Mode (Legacy / Dual). Beim attach_dynamic_transport(t,mode)
    // gestempelt; sonst module_mode_-Default. Damit funktioniert Zwei-Port-
    // Setup (Listener A=Legacy fuer FHEM, Listener B=Dual fuer debmatic) und
    // Single-Port-Backward-Compat parallel.
    const ModuleMode mod = pt ? pt->module : module_mode_;
    enum class PostPush { None, BL, App };
    PostPush post_push = PostPush::None;
    if (req.dst == kHmuDstOs && (req.cmd() == OS_GET_APP || req.cmd() == OS_CHANGE_APP) &&
        job_mtx_) {
        // Init des Klienten: ein offener Auftrag gehoert zur alten Sitzung,
        // ein spaetes `04` daraus schaltete FHEMs Init-Automaten weiter
        // (Review 0208 F3). mtx_ -> job_mtx_.
        xSemaphoreTake(job_mtx_, portMAX_DELAY);
        if (job_.active && job_.owner == src) job_.active = false;
        xSemaphoreGive(job_mtx_);
    }
    if (pt && req.dst == kHmuDstOs) {
        if (pt->mode == HostMode::AwaitingProbe) {
            if (req.cmd() == OS_GET_APP) {
                pt->mode = HostMode::Fhem;
                ESP_LOGI(TAG, "host detected: FHEM/rfd-detector (OS_GET_APP first)");
            } else if (req.cmd() == OS_CHANGE_APP) {
                // Direkt CHANGE_APP ohne vorherige OS_GET_APP-Probe — Homegear.
                pt->mode = HostMode::Homegear;
                post_push = PostPush::App;
                ESP_LOGI(TAG, "host detected: Homegear (OS_CHANGE_APP first)");
            }
        } else if (req.cmd() == OS_CHANGE_APP) {
            // OS_CHANGE_APP-Folgesequenz fuer rfd-Detector:
            //   STATE_START_BL: detector hat unsere Co_CPU_App-Reply gesehen
            //     → erkennt "Legacy CoPro in app", schickt CHANGE_APP, erwartet
            //       jetzt Co_CPU_BL-Push (= "Modul resettet zu BL").
            //   STATE_START_APP: detector hat Co_CPU_BL gesehen, schickt
            //       erneut CHANGE_APP, erwartet Co_CPU_App-Push (= "Modul aus
            //       BL in App gestartet"). Erst dann advance zu LEGACY_GET_*.
            //   Quelle: pivccu detect_radio_module radiomoduledetector.cpp:138-205.
            // FHEM sendet im Init niemals OS_CHANGE_APP, daher kein Konflikt.
            if (pt->mode == HostMode::Fhem) {
                pt->mode = HostMode::DetectorAwaitBL;
                post_push = PostPush::BL;
                ESP_LOGI(TAG, "rfd-detector: 1st CHANGE_APP → push Co_CPU_BL");
            } else if (pt->mode == HostMode::DetectorAwaitBL) {
                pt->mode = HostMode::Active;
                post_push = PostPush::App;
                ESP_LOGI(TAG, "rfd-detector: 2nd CHANGE_APP → push Co_CPU_App, init done");
            } else if (pt->mode == HostMode::Active) {
                // Re-detect cycle — push Co_CPU_App damit Detector durch geht.
                post_push = PostPush::App;
            }
        }
    }

    // Im Dual-Mode behandeln wir COMMON/HMIP/LLMAC eigenstaendig. COMMON
    // wird auch fuer Init-Probe-Sequence vom Detector benutzt — bei Empfang
    // von COMMON_START_BL/START_APP loesen wir den entsprechenden Push aus.
    if (mod == ModuleMode::Dual && req.dst == kHmuDstCommon) {
        if (req.cmd() == COMMON_IDENTIFY) {
            // Echte RPI-RF-MOD-Hardware antwortet auf COMMON_IDENTIFY direkt
            // mit dem aktuellen Modus-Banner-Frame (cmd=0x05 + "HMIP_TRX_Bl"
            // wenn im BL, "DualCoPro_App" wenn im App) — KEIN vorgeschalteter
            // ACK + DualCoPro_App. Reply-Form siehe build_bl_push_().
            // rfd::improvedInit liest die Reply per single read() und parsed
            // alle Frames; ein zusaetzliches ACK-Frame triggert "Identify
            // response string not handled". Early-return statt durch den
            // normalen build_common_response_-Pfad, damit's bei einem Frame
            // bleibt. Siehe notes/rfd_identify_unsolicited_push_2026-05-13.md
            HmuFrame push = build_bl_push_(req.cnt, mod);
            send_to_(src, push);
            return;
        }
        if (req.cmd() == COMMON_START_BL) {
            post_push = PostPush::BL;
        } else if (req.cmd() == COMMON_START_APP) {
            post_push = PostPush::App;
        }
    }

    HmuFrame resp;
    // DualCoPro: rfd schickt BidCoS-TX-Frames als APP_SEND (cmd=0x02) auf
    // dst=0x00 (HMSYSTEM). Numerisch kollidiert APP_SEND mit OS_GET_FIRMWARE
    // (auch 0x02). Disambiguation: OS_GET_FIRMWARE-Probe hat payload-len 1
    // (nur cmd-byte), APP_SEND-Frame hat min 13 bytes (cmd + meta + AskSin-frame).
    if (mod == ModuleMode::Dual && req.dst == kHmuDstOs &&
        req.cmd() == APP_SEND && req.payload.size() >= 13) {
        resp = handle_app_send_(req);
        send_to_(src, resp);
        return;
    }
    if (req.dst == kHmuDstOs) {
        resp = build_os_response_(req);
    } else if (req.dst == kHmuDstApp) {
        // App-Layer = TRX im Dual-Mode (TRX_GET_MCU_TYPE, TRX_GET_VERSION etc.)
        // ODER BidCoS-App im Legacy-Mode (APP_GET_HMID, APP_SEND etc.).
        // Wir routen erst zum BidCoS-Handler; falls req unbekannt UND Dual-Mode,
        // dann TRX-Handler — sind disjunkte command-IDs.
        if (mod == ModuleMode::Dual &&
            (req.cmd() == TRX_GET_MCU_TYPE || req.cmd() == TRX_GET_VERSION)) {
            resp = build_trx_response_(req);
        } else {
            resp = build_app_response_(req, mod, src);
        }
    } else if (mod == ModuleMode::Dual && req.dst == kHmuDstCommon) {
        resp = build_common_response_(req, mod);
    } else if (mod == ModuleMode::Dual && req.dst == kHmuDstHmip) {
        resp = build_hmip_response_(req);
    } else if (mod == ModuleMode::Dual && req.dst == kHmuDstLlmac) {
        // LLMAC = BidCoS-Frame-Pfad im DualCoPro-Mode.
        // - Detection-Probes (LLMAC_GET_DEFAULT_RF_ADDR=0x08, LLMAC_GET_SERIAL=0x07)
        //   → build_llmac_response_ mit konkreten Reply-Daten.
        // - APP_SEND=0x02 mit voller Frame-Payload (>= 13 bytes) → BidCoS-TX
        //   ueber AskSinRF (build_app_response_ delegiert an handle_app_send_).
        // - LLMAC_SEND=0x06 (multimacd CONFIG_PAIRING + Generic BidCoS-TX) →
        //   AskSin-Wrapper: 4-byte options + AskSin ab FLAG (kein LEN, kein CNT).
        //   Verifiziert 2026-04-30 byte-genau gegen echte DualCoPro 2.8.6
        //   (Memory hmuartlgw_dualcopro_real_reference_2026-04-30.md).
        // - Alles andere (z.B. multimacd-Init-Probes mit kurzen Payloads) →
        //   generic LLMAC_ACK damit kein 3 s-Timeout-Loop in multimacd entsteht.
        if (req.cmd() == LLMAC_GET_DEFAULT_RF_ADDR ||
            req.cmd() == LLMAC_GET_SERIAL) {
            resp = build_llmac_response_(req);
        } else if (req.cmd() == APP_SEND && req.payload.size() >= 13) {
            resp = build_app_response_(req, mod);
        } else if (req.cmd() == 0x06 && req.payload.size() >= 14) {
            // LLMAC_SEND: payload = [06][o1][o2][o3][o4][flags][mtype][src(3)][dst(3)][body...]
            // opts[3] (= req.payload[4]) ist die on-air AskSin-CNT die multimacd
            // diktiert. DualCoPro 2.8.6 Reference-Trace bestaetigt: Aktor-ACK
            // hat cnt == opts[3] (z.B. opts=80008001 → ACK cnt=01; opts=8000800a
            // → ACK cnt=0a). Multimacd erwartet diese Echo-Antwort und repeatet
            // sonst CONFIG_REGISTER_PARAM. Frueher (Build <=0.1.147) haben wir
            // einen eigenen llmac_tx_cnt_ Counter benutzt → Aktor-cnt ging
            // sequentiell 01,02,03... statt der von multimacd vorgegebenen
            // Sequenz 01,0a,13,1c → CONFIG_REGISTER_PARAM-Repeat-Loop.
            // Synthesize APP_SEND-Format = [02][00][00][00][CNT][flags][mtype][src(3)][dst(3)][body]
            // damit handle_app_send_ unveraendert bleibt.
            HmuFrame syn = req;
            syn.payload.assign({APP_SEND, 0x00, 0x00, 0x00, req.payload[4]});
            syn.payload.insert(syn.payload.end(),
                               req.payload.begin() + 5, req.payload.end());
            HmuFrame app_resp = handle_app_send_(syn);
            // Antwort an multimacd in LLMAC_ACK-Form (1 Byte status).
            resp.dst = req.dst;
            resp.cnt = req.cnt;
            uint8_t status = (app_resp.payload.size() >= 2 &&
                              app_resp.payload[1] == ACK_OK) ? 0x01 : 0x00;
            resp.payload.insert(resp.payload.end(), {LLMAC_ACK, status});
        } else {
            resp.dst = req.dst;
            resp.cnt = req.cnt;
            resp.payload.insert(resp.payload.end(), {LLMAC_ACK, 0x01});
        }
    } else {
        // unbekannter dst — silent timeout (HMSYSTEM-Fallback bei Legacy-Mode
        // fuer COMMON-Probes von HB-RF-ETH-Detection).
        ESP_LOGW(TAG, "unknown dst=0x%02x cnt=0x%02x cmd=0x%02x (silent timeout)", req.dst, req.cnt, req.cmd());
        return;
    }
    ESP_LOGD(TAG, "dst=%02x cnt=%02x cmd=%02x → resp.cmd=%02x len=%u",
             req.dst, req.cnt, req.cmd(),
             resp.payload.empty() ? 0 : resp.payload[0],
             static_cast<unsigned>(resp.payload.size()));
    // Ohne Nutzlast: Sendeauftrag, die Antwort schickt spaeter der Sende-Task
    // bzw. der RX-Pfad (`04 03`/`04 04`).
    if (!resp.payload.empty()) send_to_(src, resp);

    if (post_push == PostPush::App) {
        HmuFrame push = build_app_push_(0, mod);
        send_to_(src, push);
    } else if (post_push == PostPush::BL) {
        HmuFrame push = build_bl_push_(0, mod);
        send_to_(src, push);
    }
}

void HmuartlgwFrontend::send_to_(Transport* t, const HmuFrame& f) {
    if (!t) return;
    auto bytes = f.encode();
    t->write(std::span<const uint8_t>(bytes.data(), bytes.size()));
}

void HmuartlgwFrontend::broadcast_(const HmuFrame& f) {
    if (!mtx_) return;
    auto bytes = f.encode();
    std::span<const uint8_t> view(bytes.data(), bytes.size());
    xSemaphoreTake(mtx_, portMAX_DELAY);
    for (auto& t : transports_) {
        if (t) t->write(view);
    }
    xSemaphoreGive(mtx_);
}

HmuFrame HmuartlgwFrontend::build_bl_push_(uint8_t cnt, ModuleMode mode) const {
    // Legacy (HM-MOD-RPI-PCB): Co_CPU_BL im ACK_INFO-Wrapper auf HMSYSTEM.
    //   Format: dst=0x00, payload=[ACK=04, ACK_INFO=02, "Co_CPU_BL"]
    //   pivccu detect_radio_module STATE_START_BL erwartet data_len=10,
    //   data[0]=2, data[1..9]="Co_CPU_BL".
    // Dual (RPI-RF-MOD): HMIP_TRX_Bl im ACK-Wrapper auf COMMON-Layer.
    //   Format: dst=0xFE, payload=[ACK=COMMON_ACK=05, status=01, "HMIP_TRX_Bl"]
    //   pivccu detect erwartet data_len=12, data[0]=1, data[1..11]="HMIP_TRX_Bl".
    //   (Es gibt kein "DualCoPro_Bl" im Detector — der DualCoPro-BL identifiziert
    //   sich als HMIP_TRX_Bl, weil im BL HmIP/BidCoS noch nicht differenziert.)
    HmuFrame f;
    f.cnt = cnt;
    if (mode == ModuleMode::Dual) {
        f.dst = kHmuDstCommon;
        f.payload.insert(f.payload.end(), {COMMON_ACK, 0x01});
        const char s[] = "HMIP_TRX_Bl";
        for (size_t i = 0; i < sizeof(s) - 1; ++i)
            f.payload.push_back(static_cast<uint8_t>(s[i]));
    } else {
        f.dst = kHmuDstOs;
        f.payload.insert(f.payload.end(), {ACK, ACK_INFO});
        const char s[] = "Co_CPU_BL";
        for (size_t i = 0; i < sizeof(s) - 1; ++i)
            f.payload.push_back(static_cast<uint8_t>(s[i]));
    }
    return f;
}

HmuFrame HmuartlgwFrontend::build_app_push_(uint8_t cnt, ModuleMode mode) const {
    // Legacy: Co_CPU_App auf HMSYSTEM (= Homegear-Erwartung).
    // Dual:   DualCoPro_App auf COMMON.
    HmuFrame f;
    f.cnt = cnt;
    if (mode == ModuleMode::Dual) {
        f.dst = kHmuDstCommon;
        f.payload.insert(f.payload.end(), {COMMON_ACK, 0x01});
        const char s[] = "DualCoPro_App";
        for (size_t i = 0; i < sizeof(s) - 1; ++i)
            f.payload.push_back(static_cast<uint8_t>(s[i]));
    } else {
        f.dst = kHmuDstOs;
        f.payload.insert(f.payload.end(), {ACK, ACK_INFO});
        const char s[] = "Co_CPU_App";
        for (size_t i = 0; i < sizeof(s) - 1; ++i)
            f.payload.push_back(static_cast<uint8_t>(s[i]));
    }
    return f;
}

void HmuartlgwFrontend::send_hello_(Transport* t) {
    if (!t) return;
    // Hello-Push beim Connect: Bootloader-Indikator. Homegear-Hm-Mod-Rpi-Pcb
    // erwartet strikt "Co_CPU_BL"; FHEM ignoriert in STATE_NONE; rfd-detector
    // braucht's auch (STATE_START_BL). Mode aus Slot — fuer Dual-Mode-Listener
    // ist das HMIP_TRX_Bl, fuer Legacy Co_CPU_BL.
    ModuleMode mode = module_mode_;
    if (mtx_) {
        xSemaphoreTake(mtx_, portMAX_DELAY);
        for (auto& s : per_transport_) {
            if (s.owner == t) { mode = s.module; break; }
        }
        xSemaphoreGive(mtx_);
    }
    HmuFrame f = build_bl_push_(0, mode);
    send_to_(t, f);
}

namespace {
// Hilfsfunktion: Co_CPU_App-Push-Frame fuer Homegear (nach OS_CHANGE_APP).
// Nur Legacy-Mode-Helper — Dual-Mode nutzt build_app_push_().
HmuFrame make_co_cpu_app_push(uint8_t cnt) {
    HmuFrame f;
    f.dst = kHmuDstOs;
    f.cnt = cnt;
    f.payload.insert(f.payload.end(), {0x04, 0x02});  // ACK + ACK_INFO
    const char app[] = "Co_CPU_App";
    for (size_t i = 0; i < sizeof(app) - 1; ++i)
        f.payload.push_back(static_cast<uint8_t>(app[i]));
    return f;
}
}  // anon

HmuartlgwFrontend::PerTransport* HmuartlgwFrontend::slot_for_(Transport* t) {
    for (auto& slot : per_transport_) {
        if (slot.owner == t) return &slot;
    }
    return nullptr;
}

namespace {
struct DelayedHelloCtx {
    HmuartlgwFrontend*  fe;
    Transport*          transport;
    esp_timer_handle_t  timer;
};
} // anon

void HmuartlgwFrontend::schedule_hello_(Transport* t) {
    if (!t) return;
    auto* ctx = new DelayedHelloCtx{this, t, nullptr};
    esp_timer_create_args_t a = {};
    a.callback = &HmuartlgwFrontend::hello_timer_trampoline_;
    a.arg      = ctx;
    a.name     = "hmu-hello";
    if (esp_timer_create(&a, &ctx->timer) != ESP_OK) {
        delete ctx;
        return;
    }
    // Timer-Handle im Slot ablegen damit detach ihn stoppen kann (UAF/ABA-Schutz).
    if (mtx_) {
        xSemaphoreTake(mtx_, portMAX_DELAY);
        for (auto& s : per_transport_) {
            if (s.owner == t) { s.hello_timer = ctx->timer; break; }
        }
        xSemaphoreGive(mtx_);
    }
    // 200 ms — empirisch lange genug damit FHEM seinen Init-Setup
    // (Verbose-Logger, State-Init etc.) komplettieren kann, kurz genug
    // damit Homegear/rfd-5s-Passive-Listen-Window nicht abgelaufen ist.
    esp_timer_start_once(ctx->timer, 200'000);
}

void HmuartlgwFrontend::hello_timer_trampoline_(void* arg) {
    auto* ctx = static_cast<DelayedHelloCtx*>(arg);
    HmuartlgwFrontend* fe = ctx->fe;
    Transport*         t  = ctx->transport;
    esp_timer_handle_t h  = ctx->timer;

    // Race-Schutz: pruefen ob Transport noch in unseren Slots ist (nicht
    // schon ueber detach_dynamic_transport entfernt). Mutex-protected weil
    // detach asynchron passieren kann.
    bool still_alive = false;
    if (fe && fe->mtx_) {
        xSemaphoreTake(fe->mtx_, portMAX_DELAY);
        for (auto& slot : fe->per_transport_) {
            if (slot.owner == t) { still_alive = true; break; }
        }
        // Manche Transports werden ueber bind_transport gehalten ohne in
        // per_transport_ einzutragen (statisch gebunden, on_rx noch nicht
        // gefeuert). Auch die transports_-Vector pruefen.
        if (!still_alive) {
            for (auto& sp : fe->transports_) {
                if (sp.get() == t) { still_alive = true; break; }
            }
        }
        xSemaphoreGive(fe->mtx_);
    }

    if (still_alive && fe) {
        fe->send_hello_(t);
    }

    // One-shot — alles aufraeumen. Frueher gab es einen Legacy-only 3s-Re-Arm
    // (Homegear-Init-Hilfe), der aber gegen detach_dynamic_transport gerennt
    // ist: rearm konnte zwischen detach's mtx-Release und esp_timer_delete
    // feuern → double-delete des Timer-Handles + ctx-Leak → PANIC im
    // esp_timer-Pool. Verifiziert 2026-05-11 (:2325 Hello-Push 0/3 PASS,
    // crash_count saturiert auf 255). Legacy- und Dual-Pfad sind jetzt
    // symmetrisch one-shot. Slot-hello_timer-Pointer saeubern damit detach
    // kein doppeltes esp_timer_delete macht.
    if (fe && fe->mtx_) {
        xSemaphoreTake(fe->mtx_, portMAX_DELAY);
        for (auto& slot : fe->per_transport_) {
            if (slot.hello_timer == h) { slot.hello_timer = nullptr; break; }
        }
        xSemaphoreGive(fe->mtx_);
    }
    if (h) esp_timer_delete(h);
    delete ctx;
}

HmuFrame HmuartlgwFrontend::build_os_response_(const HmuFrame& req) {
    HmuFrame out;
    out.dst = kHmuDstOs;
    out.cnt = req.cnt;

    switch (req.cmd()) {
        case OS_GET_APP: {
            // payload = [0x00] + ASCII "Co_CPU_App". FHEM-Source-Line 1263
            // matcht das direkt und springt nach GETSET_PARAMETERS — der
            // ACK_INFO-Pfad (Line 1289) triggert eine zusaetzliche
            // OS_CHANGE_APP-Roundtrip-Schleife.
            //
            // Homegear-HomeMaticBidCoS Hm-Mod-Rpi-Pcb.cpp:
            // `request->response.begin() + 6 .. request->response.end() - 2`
            // == "Co_CPU_App" — das passt mit `[00] + "Co_CPU_App"` ueber
            // den 0xfd-haltigen Response auch (responsePacket inkl. 0xfd
            // ist 18 byte, byte 6..15 ist "Co_CPU_App").
            out.payload.push_back(0x00);
            const char app[] = "Co_CPU_App";
            for (size_t i = 0; i < sizeof(app) - 1; ++i)
                out.payload.push_back(static_cast<uint8_t>(app[i]));
            break;
        }
        case OS_GET_FIRMWARE: {
            // payload = [0x04, 0x02] + 3 BL-version + 3 FW-version
            // Erste Triple ist Bootloader-Version, zweite ist Application-Version.
            // Echte HM-MOD-RPI-PCB (TEQ2822427) sendet [01 00 03 + 01 02 01]
            // = BL 1.0.3 + FW 1.2.1 (verifiziert 2026-04-30 am Pi5-Header).
            // FHEM 00_HMUARTLGW.pm:1715 parsed nur die zweite Triple (`substr msg, 6, 6`),
            // pivccu/Homegear lesen evtl. die erste — daher byte-genau setzen.
            out.payload.insert(out.payload.end(),
                {ACK, ACK_INFO, 0x01, 0x00, 0x03, fw_major_, fw_minor_, fw_patch_});
            break;
        }
        case OS_GET_SERIAL: {
            out.payload.push_back(ACK);
            out.payload.push_back(ACK_INFO);
            for (size_t i = 0; i < 10 && serial_[i]; ++i)
                out.payload.push_back(static_cast<uint8_t>(serial_[i]));
            // pad bis 10 Bytes falls serial_ kuerzer
            while (out.payload.size() < 12) out.payload.push_back(0x20);
            break;
        }
        case OS_GET_CREDITS: {
            // FHEM fragt die Last alle 15 s und nach jeder zehnten Sendung ab
            // (00_HMUARTLGW.pm CheckCredits / Write). Ein echtes Modul steht
            // dabei immer im BidCoS-Empfang; hat inzwischen ein anderer Klient
            // umgeschaltet (`X21` auf dem culfw-Port nach dem Verbinden), holt
            // dieser Takt AskSin zurueck — bis 0.2.2 erst das naechste APP_SEND
            // (Review 0203 H2). Ohne Abweichung tut claim_radio_() nichts.
            claim_radio_();
            // 1-Byte Last in halben Prozent der Stundenquote (FHEM:
            // msgLoadCurrent = (load+1)/2, ab 199 sperrt es das Senden). Wir
            // melden 0: der Sendekredit (?stats) lehnt AskSin nie ab, und die
            // Abbildung des a-culfw-Kontos (Start bei der Haelfte) auf diese
            // Skala ist nicht entschieden.
            uint8_t load = 0;
            out.payload.insert(out.payload.end(), {ACK, ACK_INFO, load});
            break;
        }
        case OS_SET_TIME:
        case OS_NORMAL_MODE:
        case OS_CHANGE_APP:
        case OS_ENABLE_CSMACA:
        case OS_ENABLE_CREDITS:
            out.payload.insert(out.payload.end(), {ACK, ACK_OK});
            break;
        // APP_SEND (0x02) auf dst=0x00 wird nicht hier behandelt — kollidiert
        // numerisch mit OS_GET_FIRMWARE (0x02) im OS-switch. Disambiguation
        // ueber payload-len findet im dispatch_ statt (siehe dort).
        case OS_UPDATE_FIRMWARE:
        case OS_UPDATE_MODE:
            // Firmware-Update koennen wir nicht — explizit NACK
            // (ACK_EUNKNOWN) statt plain-ACK, sonst denkt rfd / Homegear wir
            // nehmen das Update an und beginnt FW-Bytes zu streamen.
            out.payload.insert(out.payload.end(), {ACK, ACK_EUNKNOWN});
            break;
        default:
            out.payload.insert(out.payload.end(), {ACK, ACK_OK});
            break;
    }
    return out;
}

HmuFrame HmuartlgwFrontend::build_app_response_(const HmuFrame& req, ModuleMode mod,
                                                Transport* src) {
    HmuFrame out;
    // Antwort auf der gleichen dst-Layer wie der Request, damit multimacd den
    // Reply auf das richtige Mmd-Slave-Device routet (im DualCoPro-Mode kommt
    // der Request auf dst=LLMAC=0x03 von rfd, im Legacy-Mode auf dst=APP=0x01
    // von FHEM/Homegear).
    out.dst = req.dst;
    out.cnt = req.cnt;

    switch (req.cmd()) {
        case APP_GET_HMID:
        case APP_DEFAULT_HMID: {
            // FHEM-Konstante (00_HMUARTLGW.pm:65): ACK_WITH_MULTIPART_DATA
            // Format = `04 07 XX YY <data>` mit XX=part_no, YY=total_parts.
            // pivccu detect_radio_module erwartet *exakt* `data_len == 6` und
            // liest die HMID aus `data[3..5]` (radiomoduledetector.cpp:308).
            // Ohne part_no/total_parts kommt unser Reply als data_len=4 an,
            // detector verwirft → debmatic generiert per shuf eine zufaellige
            // 0xXXXXXX-Adresse und benutzt die fortan als source-HMID; reale
            // HM-Geraete sehen Pair-Requests mit unbekannter source und
            // antworten nicht. Format jetzt byte-genau:
            //   payload[0] = 0x04 (ACK)
            //   payload[1] = 0x07 (ACK_WITH_MULTIPART_DATA)
            //   payload[2] = 0x01 (part_no)
            //   payload[3] = 0x01 (total_parts)
            //   payload[4..6] = HMID (3 bytes)
            // → frame.data = payload[1..6] = 6 bytes, data[3..5] = HMID. ✓
            out.payload.insert(out.payload.end(), {ACK, ACK_MULTIPART,
                0x01, 0x01,
                hmid_[0], hmid_[1], hmid_[2]});
            break;
        }
        case APP_SET_HMID: {
            // payload[1..3] = neue HMID
            if (req.payload.size() >= 4) {
                // Unter peer_mtx_: auto_ack_() liest hmid_ im RX-Task.
                xSemaphoreTake(peer_mtx_, portMAX_DELAY);
                hmid_[0] = req.payload[1];
                hmid_[1] = req.payload[2];
                hmid_[2] = req.payload[3];
                xSemaphoreGive(peer_mtx_);
                if (services_.eeprom) {
                    services_.eeprom->write_block(ee::HMU_HMID,
                        std::span<const uint8_t>(hmid_, 3));
                }
            }
            out.payload.insert(out.payload.end(), {ACK, ACK_OK});
            break;
        }
        case APP_SET_CURRENT_KEY:
        case APP_SET_PREVIOUS_KEY:
        case APP_SET_TEMP_KEY:
            // Phase 2: AES-Keys storage-only akzeptieren (echte Krypto
            // erst Phase 4). Wir bestaetigen mit ACK damit FHEM sich nicht
            // im Set-Key-Loop aufhaengt.
            // FIXME Phase 4: in EE_HMU_KEY_CURRENT/PREVIOUS/TEMP schreiben
            //                und mbedtls/aes encrypt/decrypt vor RF-RX/TX.
            out.payload.insert(out.payload.end(), {ACK, ACK_OK});
            break;
        case APP_SEND:
            return handle_app_send_(req, mod == ModuleMode::Legacy ? src : nullptr);
        case APP_ADD_PEER:
        case APP_REMOVE_PEER: {
            // FHEM: ADD_PEER = [06][id(3)][kNo][wakeup][00], REMOVE_PEER =
            // [07][id(3)]. Bis 0.2.2 bekamen beide nur `04 01`; FHEM meldete
            // "Adding peer ... failed! You have probably forced an unknown
            // aesKey" und verwarf seine Quittungen trotzdem (Nutzerbericht 31.10.).
            // Im Dual-Modus gibt es diese Befehle nicht — Antwort wie bisher.
            if (mod != ModuleMode::Legacy || req.payload.size() < 4) {
                out.payload.insert(out.payload.end(), {ACK, ACK_OK});
                break;
            }
            const uint8_t* id = req.payload.data() + 1;
            // Byte 5 = Wakeup (00_HMUARTLGW.pm UpdatePeerReq); FHEM schickt
            // ADD_PEER fuer einen schon eingetragenen Peer erneut, wenn sich
            // nur dieses Byte aendert (CUL_HM hmInitMsgUpdt: "init:" bei
            // geaenderter newChn) — der Eintrag wird dann aktualisiert.
            const bool wakeup = req.payload.size() >= 6 && req.payload[5] != 0;
            bool added = false;
            xSemaphoreTake(peer_mtx_, portMAX_DELAY);
            if (req.cmd() == APP_ADD_PEER) {
                added = peer_add_(id, wakeup);
            } else {
                peer_remove_(id);
            }
            HmuFrame r = build_peer_reply_(req, added);
            xSemaphoreGive(peer_mtx_);
            if (req.cmd() == APP_ADD_PEER && !added) {
                ESP_LOGW(TAG, "peer table full (%u), %02X%02X%02X not added",
                         static_cast<unsigned>(kMaxPeers), id[0], id[1], id[2]);
            }
            return r;
        }
        case APP_TRX_GET_MCU_TYPE:
            // HB-RF-ETH/rfd-Detection-Probe fuer TRX-Class-Module (RPI-RF-MOD,
            // HmIP-RFUSB). Wir sind Legacy CoPro, NICHT TRX — explizit NACK
            // statt 3 s Probe-Timeout. Bringt rfd schneller in den richtigen
            // Detection-Pfad.
            out.payload.insert(out.payload.end(), {ACK, ACK_EUNKNOWN});
            break;
        default:
            out.payload.insert(out.payload.end(), {ACK, ACK_OK});
            break;
    }
    return out;
}

HmuFrame HmuartlgwFrontend::handle_app_send_(const HmuFrame& req, Transport* legacy_src) {
    HmuFrame out;
    out.dst = req.dst;   // mirror — Legacy=APP, Dual=LLMAC.
    out.cnt = req.cnt;

    // Wire-Layout des APP_SEND payloads (FHEM 00_HMUARTLGW.pm:1712ff):
    //   [0]=0x02 (APP_SEND cmd byte)
    //   [1..2]=0x00 0x00 (constant)
    //   [3]=mode-byte (nur bei FW > 0x010006 — wir reporten 0x010401, also DA)
    //   [4..]=msg_nr, flags, mtype, src(3), dst(3), payload...
    //
    // Mindestlaenge: 4 prefix + 9 (msg_nr+flags+mtype+src+dst) = 13 bytes.
    if (req.payload.size() < 13) {
        // Ohne mode-byte (FW < 0x010006) waere min. 12 — aber wir reporten 1.4.1.
        ESP_LOGW(TAG, "APP_SEND too short: %u bytes",
                 static_cast<unsigned>(req.payload.size()));
        out.payload.insert(out.payload.end(), {ACK, ACK_EUNKNOWN});
        return out;
    }

    if (!services_.rf_manager) {
        ESP_LOGW(TAG, "APP_SEND without rf_manager");
        out.payload.insert(out.payload.end(), {ACK, ACK_EUNKNOWN});
        return out;
    }

    // Body = msg_nr + flags + mtype + src + dst + payload (alles nach prefix).
    const uint8_t* body  = req.payload.data() + 4;
    const size_t   blen  = req.payload.size() - 4;
    if (blen + 1 > AskSinFrame::kMaxPayload) {
        out.payload.insert(out.payload.end(), {ACK, ACK_EUNKNOWN});
        return out;
    }

    // AskSin-Wire-Frame bauen: [LEN][body...] mit LEN = body-Bytes (a-culfw
    // rf_asksin.c: msg[0]=len, msg[1..len]=body).
    AskSinTxFrame f{};
    f.len     = static_cast<uint8_t>(blen);
    f.data[0] = static_cast<uint8_t>(blen);
    // 4 Byte Praeambel wie die echten eQ-3-Sender (RFUSB, Geraete; HackRF
    // 01.11.2026), nicht die 10 ms von a-culfw. Burst-Rahmen behalten 360 ms.
    f.extra_preamble_ms = 0;
    std::memcpy(f.data + 1, body, blen);

    // Legacy-Klient, Quittungswunsch, Ziel ungleich 000000: Sendeauftrag.
    // Body: [0]=CNT [1]=FLAG [2]=TYP [3..5]=Absender [6..8]=Ziel.
    static constexpr uint8_t kNoAddr[3] = {0, 0, 0};
    if (legacy_src && send_task_ && job_mtx_ && blen >= 9 &&
        (body[1] & BIDCOS_FLAG_BIDI) && std::memcmp(body + 6, kNoAddr, 3) != 0) {
        xSemaphoreTake(job_mtx_, portMAX_DELAY);
        if (job_.active) {
            // FHEM schickt waehrend eines offenen Auftrags nichts (es wartet
            // auf `04`); ein zweiter Legacy-Klient bekommt "beschaeftigt"
            // und wiederholt nach 50 ms (00_HMUARTLGW.pm EINPROGRESS).
            xSemaphoreGive(job_mtx_);
            out.payload.insert(out.payload.end(), {ACK, ACK_EINPROGRESS});
            return out;
        }
        ++job_.gen;
        job_.active  = true;
        job_.owner   = legacy_src;
        job_.hmu_cnt = req.cnt;
        job_.cnt     = body[0];
        std::memcpy(job_.src, body + 3, 3);
        std::memcpy(job_.dst, body + 6, 3);
        std::memcpy(job_.msg, f.data, static_cast<size_t>(f.len) + 1u);
        job_.t_start = static_cast<uint32_t>(esp_timer_get_time());
        xSemaphoreGive(job_mtx_);
        xTaskNotifyGive(static_cast<TaskHandle_t>(send_task_));
        out.payload.clear();                 // Antwort kommt spaeter
        return out;
    }

    ensure_asksin_();
    esp_err_t e = services_.rf_manager->tx(ProtocolId::AskSin, &f, sizeof(f));
    if (e == ESP_OK) {
        out.payload.insert(out.payload.end(), {ACK, ACK_OK});
    } else if (e == ESP_ERR_INVALID_STATE) {
        // a-culfw `LOVF` aequivalent — Credit/CCA/MARC stuck.
        // Genauer differenzieren waere besser; ohne LOVF-Stats-Zugriff im
        // Manager-Pfad mappen wir uniform auf NoCredits.
        out.payload.insert(out.payload.end(), {ACK, ACK_ENOCREDITS});
    } else if (e == ESP_ERR_TIMEOUT) {
        out.payload.insert(out.payload.end(), {ACK, ACK_ECSMACA});
    } else {
        out.payload.insert(out.payload.end(), {ACK, ACK_EUNKNOWN});
    }
    return out;
}

void HmuartlgwFrontend::ensure_asksin_() {
    // Slot-0 ggf. neu auf AskSin schalten falls anderer Frontend (ASCII `Zr`/
    // `Fr` etc.) inzwischen umgeschaltet hat. Defensiv — in der HMUARTLGW-
    // Domaene gehoert Slot 0 dem AskSin.
    if (!services_.rf_manager) return;
    auto active = services_.rf_manager->active_protocol_on(0);
    if (!active || active->id() != ProtocolId::AskSin) {
        services_.rf_manager->activate(ProtocolId::AskSin, 0);
    }
}

void HmuartlgwFrontend::send_task_trampoline_(void* arg) {
    static_cast<HmuartlgwFrontend*>(arg)->send_task_loop_();
    vTaskDelete(nullptr);
}

void HmuartlgwFrontend::finish_job_(uint32_t gen, std::initializer_list<uint8_t> ack) {
    Transport* owner = nullptr;
    uint8_t    cnt   = 0;
    xSemaphoreTake(job_mtx_, portMAX_DELAY);
    if (job_.active && job_.gen == gen) {
        job_.active = false;
        owner = job_.owner;
        cnt   = job_.hmu_cnt;
    }
    xSemaphoreGive(job_mtx_);
    if (!owner) return;                      // schon beantwortet oder verworfen
    // Nur an einen noch verbundenen Transport schreiben (shared_ptr haelt ihn
    // fuer das write am Leben).
    std::shared_ptr<Transport> sp;
    xSemaphoreTake(mtx_, portMAX_DELAY);
    for (const auto& t : transports_) {
        if (t.get() == owner) { sp = t; break; }
    }
    xSemaphoreGive(mtx_);
    if (!sp) return;
    HmuFrame f;
    f.dst = kHmuDstApp;
    f.cnt = cnt;
    f.payload.assign(ack);
    auto bytes = f.encode();
    sp->write(std::span<const uint8_t>(bytes.data(), bytes.size()));
}

void HmuartlgwFrontend::send_task_loop_() {
    auto now_us = [] { return static_cast<uint32_t>(esp_timer_get_time()); };
    uint32_t done_gen = 0;
    for (;;) {
        // Naechsten offenen Auftrag holen; ohne Auftrag auf die Meldung aus
        // handle_app_send_ warten.
        uint32_t gen = 0;
        uint32_t deadline = 0;
        AskSinTxFrame f{};
        xSemaphoreTake(job_mtx_, portMAX_DELAY);
        const bool have = job_.active && job_.gen != done_gen;
        if (have) {
            gen      = job_.gen;
            deadline = job_.t_start + kJobDeadlineUs;
            f.len = job_.msg[0];
            std::memcpy(f.data, job_.msg, static_cast<size_t>(f.len) + 1u);
        }
        xSemaphoreGive(job_mtx_);
        if (!have) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        done_gen = gen;
        // 4 Byte Praeambel wie die eQ-3-Sender, Burst-Rahmen behalten 360 ms.
        f.extra_preamble_ms = 0;

        auto open = [&] {
            xSemaphoreTake(job_mtx_, portMAX_DELAY);
            const bool o = job_.active && job_.gen == gen;
            xSemaphoreGive(job_mtx_);
            return o;
        };
        // Bis t_us warten; true, sobald der Auftrag erledigt ist (der RX-Pfad
        // hat die Antwort zugestellt und weckt den Task).
        auto wait_done = [&](uint32_t t_us) {
            for (;;) {
                if (!open()) return true;
                const int32_t r = static_cast<int32_t>(t_us - now_us());
                if (r <= 0) return false;
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(r / 1000) + 1);
            }
        };

        uint32_t last_start = 0;
        bool     done       = false;
        int      busy       = 0;
        auto past = [&](uint32_t t) { return static_cast<int32_t>(t - now_us()) <= 0; };
        // Burst-Rahmen (Flag 0x10) nur einmal: jede Sendung traegt 360 ms
        // Praeambel, und CUL_HM wiederholt bei ausbleibender Antwort ohnehin
        // selbst (am 01.11. an einem stummen Aktor 4 Versuche); mit Wiederholung
        // im Stick kaeme ein Befehl an ein totes Burst-Geraet auf mehrere
        // Sekunden Sendezeit (Review 0208 F5, entschieden 01.11.2026).
        const int max_retries = (f.data[2] & 0x10) ? 0 : kMaxRetries;
        for (int attempt = 0; attempt <= max_retries && !done; ) {
            if (!open()) { done = true; break; }
            if (past(deadline)) {
                if (attempt > 0) break;                     // keine Wiederholung mehr
                // Erster Versuch, aber die Frist ist schon um: der Task hing
                // noch in der Sendung des Vorgaengers (Review 0208b B2). Nicht
                // mehr senden, FHEM wiederholt nach 200 ms (CSMA/CA).
                finish_job_(gen, {ACK, ACK_ECSMACA});
                done = true;
                break;
            }
            f.cca_once   = attempt > 0;
            f.sent_at_us = 0;
            // Ohne mtx_: im Wettlauf mit claim_radio_ schlimmstenfalls eine
            // doppelte Aktivierung derselben Klasse (harmlos).
            ensure_asksin_();
            // Erster Versuch: Slot-Sperre bis 3 s, aber nicht ueber die Frist
            // hinaus; Wiederholungen nur kurz. Ein Auftrag soll FHEMs 10-s-Frist
            // nie erreichen. Erst nach ensure_asksin_ rechnen (das kann selbst
            // bis 3 s an der Slot-Sperre warten) und mit Vorzeichen, sonst
            // liefe die Restzeit unter (Review 0209 C3).
            uint32_t lock_ms = kRetryLockMs;
            if (attempt == 0) {
                const int32_t left_us = static_cast<int32_t>(deadline - now_us());
                if (left_us <= 0) {
                    finish_job_(gen, {ACK, ACK_ECSMACA});
                    done = true;
                    break;
                }
                const uint32_t left_ms = static_cast<uint32_t>(left_us) / 1000u;
                lock_ms = left_ms < CC1101Slot::kOpTimeoutMs ? (left_ms ? left_ms : 1u)
                                                             : CC1101Slot::kOpTimeoutMs;
            }
            const esp_err_t e = services_.rf_manager
                ? services_.rf_manager->tx(ProtocolId::AskSin, &f, sizeof(f), lock_ms)
                : ESP_ERR_INVALID_STATE;
            if (e == ESP_ERR_NOT_FINISHED) {
                // Wiederholung, Kanal belegt: womoeglich laeuft gerade die
                // Antwort ein. In 20-ms-Schritten neu pruefen, hoechstens
                // 10 Mal, dann keine weitere Wiederholung.
                if (wait_done(now_us() + 20'000)) { done = true; break; }
                if (++busy > 10) break;
                continue;
            }
            if (e != ESP_OK) {
                if (attempt == 0) {
                    // Wie bisher: nichts gesendet, FHEM entscheidet (CSMA/CA
                    // wiederholt es nach 200 ms).
                    const uint8_t code = (e == ESP_ERR_TIMEOUT) ? ACK_ECSMACA : ACK_EUNKNOWN;
                    finish_job_(gen, {ACK, code});
                    done = true;
                }
                break;
            }
            last_start = f.sent_at_us ? f.sent_at_us : now_us();
            busy = 0;
            ++attempt;
            if (attempt > max_retries) break;
            if (wait_done(last_start + kRetryAfterUs)) done = true;
            else ESP_LOGD(TAG, "APP_SEND cnt %02X: no answer, repeat %d", f.data[1], attempt);
        }
        if (done) continue;
        // Nach der letzten Sendung immer das volle Antwortfenster, auch wenn
        // die Frist dabei verstreicht: sie begrenzt Wiederholungen, nicht das
        // Zuhoeren (Review 0208b B1). Im Regelfall dauert ein Auftrag unter
        // ~6 s, im Extremfall (Slot-Sperre, CCA, Burst) bis ~8 s, unter FHEMs
        // 10 s (Review 0209).
        if (last_start && wait_done(last_start + kLastWaitUs)) continue;
        finish_job_(gen, {ACK, ACK_EUNKNOWN});
    }
}

void HmuartlgwFrontend::claim_radio_() {
    if (!services_.rf_manager) return;
    auto active = services_.rf_manager->active_protocol_on(0);
    if (active && active->id() == ProtocolId::AskSin) return;
    // Bis 0.2.2 schaltete erst das erste APP_SEND um; bis dahin empfing der
    // Stick keine BidCoS-Rahmen (Nutzerbericht 31.10.: nach `X21` keine Thermostat-
    // Meldungen, bis FHEM das erste Mal sendete).
    if (services_.rf_manager->activate(ProtocolId::AskSin, 0) == ESP_OK) {
        ESP_LOGI(TAG, "slot 0 -> AskSin (HMU client)");
    } else {
        ESP_LOGW(TAG, "slot 0 -> AskSin failed");
    }
}

int HmuartlgwFrontend::peer_index_(const uint8_t* id) const {
    for (size_t i = 0; i < peer_count_; ++i) {
        if (std::memcmp(peers_[i].id, id, 3) == 0) return static_cast<int>(i);
    }
    return -1;
}

bool HmuartlgwFrontend::peer_add_(const uint8_t* id, bool wakeup) {
    const int i = peer_index_(id);
    if (i >= 0) {
        peers_[static_cast<size_t>(i)].wakeup = wakeup;
        return true;
    }
    if (peer_count_ >= kMaxPeers) return false;
    std::memcpy(peers_[peer_count_].id, id, 3);
    peers_[peer_count_].wakeup = wakeup;
    ++peer_count_;
    return true;
}

void HmuartlgwFrontend::peer_remove_(const uint8_t* id) {
    const int i = peer_index_(id);
    if (i < 0) return;
    peers_[static_cast<size_t>(i)] = peers_[peer_count_ - 1];
    --peer_count_;
}

HmuFrame HmuartlgwFrontend::build_peer_reply_(const HmuFrame& req, bool added) const {
    // `04 07 01 01 <Anzahl:2>` und beim Eintragen 8 Byte Kanalmaske dahinter.
    // FHEM (HMUARTLGW_ParsePeer) unterscheidet "eingetragen" von "ausgetragen"
    // allein an der Laenge; die Maske liest es negiert, jedes gesetzte Bit
    // heisst dort "AES fuer diesen Kanal abschalten". 0xFF = nichts
    // abzuschalten: dieses Modul kann kein AES. Beispiel aus dem FHEM-Quelltext:
    // `040701010002fffffffffffffff9`; dieselbe Antwort hat die mCCU-Bruecke am
    // 19.08.2026 gegen FHEM und einen echten HM-LC-Sw1-Pl-2 bestanden.
    HmuFrame out;
    out.dst = req.dst;
    out.cnt = req.cnt;
    out.payload.insert(out.payload.end(), {ACK, ACK_MULTIPART, 0x01, 0x01,
        static_cast<uint8_t>(peer_count_ >> 8), static_cast<uint8_t>(peer_count_)});
    if (added) out.payload.insert(out.payload.end(), 8, 0xFF);
    return out;
}

void HmuartlgwFrontend::auto_ack_(const AskSinFrame& af, uint32_t air_start_us) {
    // Regeln wie die mCCU-Bruecke (`_selbst_quittieren`, gegen FHEM und einen
    // echten Aktor getestet): nur mit Quittungswunsch, nur an die eigene HMID,
    // nur von eingetragenen Peers. Nicht quittiert werden 0x02 (Quittung),
    // 0x03 (AES-Antwort) und 0x3F (Zeitanfrage, will eine Antwort). 0x00
    // (Geraeteinfo) schon, anders als die Bruecke: FHEM unterdrueckt seine
    // reine Quittung an eingetragene Peers unabhaengig vom Typ (Review 0203 F2).
    // Status-Quittungen (`8002 … 0101<st>…`, z. B. fuer HM-SEC-SC) schickt FHEM
    // zusaetzlich selbst, sie haben nicht die Laenge 24 (Review 0203 F1).
    // Das tut es am echten Modul genauso; ob dieses vorher selbst quittiert,
    // ist nicht gemessen.
    if (af.len < 9 || !services_.rf_manager || !peer_mtx_) return;
    const uint8_t* m = af.data;
    if (!(m[2] & BIDCOS_FLAG_BIDI)) return;
    const uint8_t type = m[3];
    if (type == BIDCOS_TYPE_ACK || type == 0x03 || type == 0x3F) return;

    uint8_t own[3];
    bool    wakeup = false;
    xSemaphoreTake(peer_mtx_, portMAX_DELAY);
    std::memcpy(own, hmid_, 3);
    const int idx = (std::memcmp(m + 7, own, 3) == 0) ? peer_index_(m + 4) : -1;
    if (idx >= 0) wakeup = peers_[static_cast<size_t>(idx)].wakeup;
    xSemaphoreGive(peer_mtx_);
    if (idx < 0) return;

    // Quittung wie FHEM sie baut (CUL_HM: `<cnt> 80 02 <eigene> <Geraet> 00`).
    // Fuer einen Peer mit Wakeup-Byte im ADD_PEER traegt sie das Wach-Bit
    // (`81 02`): so quittiert CUL_HM ueber einen CUL, solange Befehle fuer ein
    // Weck- oder Lazy-Config-Geraet anstehen (10_CUL_HM.pm 3777–3783,
    // 3791–3797: `8102` nur fuer andere IO-Typen, "HMLAN and HMUARTLGW does it
    // automatically if configured to lazy config"; 3730–3735: Flag |= 0x01 fuer
    // jede Quittung, die reine 0x80 ist bei HMUARTLGW uebersprungen). Das Bit
    // haelt das Geraet wach (AskSinPP Message.h WKUP = 0x01 "keep the device
    // awake", Device.h: stayAwake 500 ms), bis 0.2.2 schlief es nach der
    // Quittung sofort wieder ein und FHEMs Befehle liefen ins Leere.
    // Zeitpunkt: Rahmenstart + 124 ms, Start zu Start, unabhaengig von der
    // Laenge (Rahmenstart = Sync minus 4 Byte Praeambel). Die eQ-3-Folien
    // nennen fuer BidCoS 125 ms Antwortzeit. Im tsculfw-Mitschnitt vom
    // 01.11.2026 (4-ms-Aufloesung) antworteten HM-MOD-RE-8 und HM-SEC-SC-2 mit
    // 124 ms, der SC-2 gleich bei Anfragen der Laenge 0x0B und 0x13; tsculfw
    // quittiert ebenso. HackRF 01.11.: HM-TC-IT-WM-W-EU antwortet nach
    // 129..131 ms, die OCCU ueber eine RFUSB quittiert nach 126,4 ms (Laenge
    // 14). Zu frueh wird ignoriert: ~12 ms und ~60 ms nach seinem Rahmen nahm
    // der HM-TC-IT-WM-W-EU nicht an (dreimal wiederholt, RESPONSE TIMEOUT);
    // mit 124 ms nahm er 8 von 8 Quittungen ohne Wiederholung an. 0.2.5 wartete
    // 100..120 ms ab der Lesezeit nach dem Rahmenende und sendete 0..10 ms
    // Praeambel davor: auf der Luft 129..175 ms Start zu Start, mit der Laenge
    // wachsend; ein HM-SEN-MDIR-SM wiederholte im tsculfw-Mitschnitt nach einer
    // Quittung bei 156 ms.
    // air_start_us kommt aus der GDO2-Flanke (Paketende) minus Luftzeit,
    // AskSinRF::tx wartet den Rest unter der Chipsperre per esp_timer ab.
    // Ein Rahmen, der waehrend dieser Wartezeit eintrifft, behaelt so seinen
    // echten Start (seine Flanke; ein weiterer Rahmen in derselben Leserunde
    // bekommt die Lesezeit, Review 0206 F1) und wird, wenn die Zeit schon um
    // ist, sofort quittiert.
    // Ist der Rahmen vor dem Warten an der Slot-Sperre schon kAutoAckLateMs
    // alt, entfaellt die Quittung: die Wiederholung des Geraets kommt dann
    // ohnehin und wird rechtzeitig quittiert. Das gilt auch fuer einen Rahmen,
    // der lange im FIFO lag (seine Flanke bleibt alt, Review 0206 F2).
    // Waehrenddessen steht der RX-Task samt Heartbeat (recover_rx_): ein
    // weiterer Rahmen bleibt im FIFO (der Chip geht danach in IDLE, MCSM1 =
    // 0x33) und wird danach gelesen; ein dritter vor dem STX der Quittung ginge
    // verloren, nach ihr steht der Chip per TXOFF_MODE = RX wieder auf Empfang.
    // Das Geraet, dem die Quittung gilt, sendet in dieser Zeit nicht.
    // RfManager::tx nimmt Slot- und Chipsperre; der RX-Task haelt hier keine
    // (AskSinRF::drain_fifo gibt die Chipsperre vor dem Callback frei). Die
    // Slot-Sperre wartet nur bis kAutoAckLateMs, nicht 3 s (Review 0203 F4).
    const uint8_t flags = wakeup ? (0x80 | BIDCOS_FLAG_WKUP) : 0x80;
    auto now_us = [] { return static_cast<uint32_t>(esp_timer_get_time()); };
    const uint32_t target = air_start_us + kAutoAckStartUs;
    // Grob vorwarten, bis hoechstens ~20 ms vor dem Ziel: vTaskDelay(n) dauert
    // bei 100 Hz 10*(n-1)..10*n ms. Den Rest wartet tx() unter der Chipsperre.
    if (const int32_t r = static_cast<int32_t>(target - now_us()); r > 30'000) {
        vTaskDelay(static_cast<TickType_t>((r / 1000 - 20) / 10));
    }
    const uint32_t age = (now_us() - air_start_us) / 1000u;
    if (age >= kAutoAckLateMs) {
        ESP_LOGW(TAG, "auto-ACK %02X%02X%02X cnt %02X skipped, %u ms late",
                 m[4], m[5], m[6], m[1], static_cast<unsigned>(age));
        return;
    }
    AskSinTxFrame f{};
    f.len = 0x0A;
    f.extra_preamble_ms = 0;            // 4 Byte wie die eQ-3-Sender
    f.tx_at_us          = target | 1u;  // 0 hiesse "sofort"
    // Spaeter als kAutoAckLateMs nach dem Rahmenstart nicht mehr senden, auch
    // wenn das CCA den Kanal erst dann freigibt: ein HM-TC-IT-WM-W-EU
    // wiederholte nach ~210 ms (andere Geraete spaeter), eine spaetere Quittung
    // kaeme seiner Wiederholung in die Quere.
    f.tx_until_us       = (air_start_us + kAutoAckLateMs * 1000u) | 1u;
    const uint8_t frame[] = {0x0A, m[1], flags, BIDCOS_TYPE_ACK,
                             own[0], own[1], own[2], m[4], m[5], m[6], 0x00};
    std::memcpy(f.data, frame, sizeof(frame));
    const esp_err_t e = services_.rf_manager->tx(ProtocolId::AskSin, &f, sizeof(f),
                                                 kAutoAckLateMs - age);
    if (e == ESP_ERR_TIMEOUT && !f.sent_at_us) {
        // Nicht gesendet, Frist abgelaufen (Slot-Sperre oder belegter Kanal).
        // Nicht als W auf die Konsole: bei Fremdverkehr kommt das regelmaessig
        // vor. Mit sent_at_us war die Quittung auf der Luft (z. B. NoReturnRx),
        // das meldet der Zweig darunter (Review 0209 C4).
        ESP_LOGI(TAG, "auto-ACK %02X%02X%02X cnt %02X dropped (deadline)",
                 m[4], m[5], m[6], m[1]);
    } else if (e != ESP_OK) {
        ESP_LOGW(TAG, "auto-ACK %02X%02X%02X cnt %02X: %s",
                 m[4], m[5], m[6], m[1], esp_err_to_name(e));
    }
}

void HmuartlgwFrontend::on_asksin_rx_(const RxEvent& ev) {
    auto* af = static_cast<const AskSinFrame*>(ev.payload);
    if (!af || af->len < 9) return;     // < flags+mtype+src+dst+...

    // RX-Frame-Format ist Mode-abhaengig (verifiziert 2026-04-30 byte-genau):
    //
    // Legacy (HM-MOD-RPI-PCB Co_CPU_App 1.2.1) per dump_traffic:
    //   dst=0x01 (App), cmd=0x05 (APP_RECV)
    //   body = [status=0x00] [info=0x00] [RSSI] [AskSin from CNT, no LEN]
    //
    // DualCoPro (HM-MOD-RPI-PCB-flashed 2.8.6 == RPI-RF-MOD-Familie):
    //   dst=0x03 (LLMAC), cmd=0x05 (LLMAC_RECV)
    //   body = [ms_h] [ms_l] [info=0x00] [RSSI] [AskSin from CNT, no LEN]
    //
    // Mit zwei TCP-Ports (Legacy fuer FHEM, Dual fuer debmatic) parallel-aktiv
    // muessen wir jeden Slot in seinem eigenen Mode-Format beliefern. Statt
    // broadcast_() iterieren wir pro-Slot und bauen den Frame pro-Mode.
    if (!mtx_) return;
    const uint16_t ms = static_cast<uint16_t>(esp_timer_get_time() / 1000ULL);
    auto build_frame = [&](ModuleMode mode, uint8_t cnt) {
        HmuFrame out;
        out.dst = (mode == ModuleMode::Dual) ? kHmuDstLlmac : kHmuDstApp;
        out.cnt = cnt;
        out.payload.reserve(6 + af->len);
        out.payload.push_back(APP_RECV);  // 0x05 RECV-cmd in both modes
        if (mode == ModuleMode::Dual) {
            out.payload.push_back(static_cast<uint8_t>(ms >> 8));
            out.payload.push_back(static_cast<uint8_t>(ms & 0xff));
        } else {
            out.payload.push_back(0x00);  // status=0x00 (kein AES)
        }
        out.payload.push_back(0x00);          // info = unsolicited
        out.payload.push_back(hmu_rssi(af->rssi_raw));
        out.payload.insert(out.payload.end(), af->data + 1, af->data + 1 + af->len);
        return out;
    };
    // Snapshot der live transports + ihrer Modes UNTER mutex. Dann release —
    // shared_ptr halten Transport am Leben fuer den write()-Loop. Verhindert
    // Use-After-Free wenn TCP-Close zwischen Iteration und write() haendelt.
    xSemaphoreTake(mtx_, portMAX_DELAY);
    const uint8_t cnt = rx_cnt_++;
    std::vector<std::pair<std::shared_ptr<Transport>, ModuleMode>> snapshot;
    snapshot.reserve(transports_.size());
    for (const auto& sp : transports_) {
        if (!sp) continue;
        ModuleMode mode = module_mode_;
        Transport* raw = sp.get();
        for (const auto& slot : per_transport_) {
            if (slot.owner == raw) { mode = slot.module; break; }
        }
        snapshot.emplace_back(sp, mode);
    }
    xSemaphoreGive(mtx_);

    HmuFrame legacy = build_frame(ModuleMode::Legacy, cnt);
    HmuFrame dual   = build_frame(ModuleMode::Dual,   cnt);
    auto bytes_legacy = legacy.encode();
    auto bytes_dual   = dual.encode();

    // Antwort auf den offenen Sendeauftrag (gleicher Zaehler, vom Ziel an den
    // Absender, beliebiger Typ: Quittung 02 oder inhaltliche Antwort wie 10)?
    // Der Auftraggeber bekommt sie genau einmal, als `04 03` unter dem Zaehler
    // seiner Anfrage -- als `05` dazu saehe FHEM sie doppelt. Die uebrigen
    // Verbindungen bekommen sie wie jeden Rahmen als `05`.
    Transport*         answer_to = nullptr;
    std::vector<uint8_t> bytes_answer;
    if (job_mtx_ && af->len >= 9) {
        const uint8_t* m = af->data;
        xSemaphoreTake(job_mtx_, portMAX_DELAY);
        if (job_.active && m[1] == job_.cnt &&
            std::memcmp(m + 4, job_.dst, 3) == 0 && std::memcmp(m + 7, job_.src, 3) == 0) {
            job_.active = false;
            answer_to   = job_.owner;
            HmuFrame a;
            a.dst = kHmuDstApp;
            a.cnt = job_.hmu_cnt;
            a.payload.reserve(4 + af->len);
            a.payload.insert(a.payload.end(), {ACK, ACK_WITH_RESPONSE, 0x00,
                                               hmu_rssi(af->rssi_raw)});
            a.payload.insert(a.payload.end(), af->data + 1, af->data + 1 + af->len);
            bytes_answer = a.encode();
        }
        xSemaphoreGive(job_mtx_);
        if (answer_to && send_task_) xTaskNotifyGive(static_cast<TaskHandle_t>(send_task_));
    }

    for (auto& [sp, mode] : snapshot) {
        if (answer_to && sp.get() == answer_to) {
            sp->write(std::span<const uint8_t>(bytes_answer.data(), bytes_answer.size()));
            continue;
        }
        const auto& bytes = (mode == ModuleMode::Dual) ? bytes_dual : bytes_legacy;
        sp->write(std::span<const uint8_t>(bytes.data(), bytes.size()));
    }

    // Erst danach quittieren: die Quittung darf ohnehin erst kAutoAckStartUs
    // nach dem Rahmenstart raus (siehe auto_ack_), die Hosts bekommen den Rahmen
    // so ohne diese Wartezeit. FHEM sendet nach einem Rahmen mit
    // Quittungswunsch fruehestens 300 ms spaeter an dasselbe Geraet
    // (00_HMUARTLGW.pm nextSend), die Quittung liegt davor.
    // Ohne Rahmenstart (sollte AskSin immer liefern) gilt die Lesezeit.
    auto_ack_(*af, ev.air_start_us ? ev.air_start_us : ev.timestamp_us);
}

// ============================================================================
// DualCoPro-Mode (RPI-RF-MOD) — Replier fuer COMMON/TRX/HMIP/LLMAC-Layer.
// Alle Antwort-Formate byte-genau pivccu detect_radio_module
// radiomoduledetector.cpp:138-326. Werden nur im Dual-Mode aufgerufen.
// ============================================================================

void HmuartlgwFrontend::fill_sgtin_(uint8_t* sgtin12) const {
    // 12-Byte SGTIN. Wir generieren stable aus MAC + HMID.
    // Pattern aus pivccu-Beispielen: `3014F711AAAABBCC...` mit eq-3-Prefix
    // 0x3014 (company-id) und letzten Bytes als geraete-spezifische ID.
    // Die letzten 5 Bytes (data[7..11]) sind die "Serial-Suffix" — werden vom
    // Detector als Serial-String per `sprintf("%02X%02X%02X%02X%02X", ...)`
    // formatiert; rfd nutzt das u.a. als BidCoS-MAC-Fallback wenn nichts
    // anderes verfuegbar ist.
    sgtin12[0] = 0x30; sgtin12[1] = 0x14;          // eq-3 GS1-Prefix
    sgtin12[2] = 0xF7; sgtin12[3] = 0x11;          // class-id (RPI-RF-MOD)
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sgtin12[4] = mac[0]; sgtin12[5] = mac[1]; sgtin12[6] = mac[2];
    sgtin12[7] = mac[3]; sgtin12[8] = mac[4]; sgtin12[9] = mac[5];
    sgtin12[10] = hmid_[1]; sgtin12[11] = hmid_[2];
}

HmuFrame HmuartlgwFrontend::build_common_response_(const HmuFrame& req, ModuleMode mode) {
    // COMMON-Layer (dst=0xFE). pivccu hmframe.h:79-84.
    HmuFrame out;
    out.dst = kHmuDstCommon;
    out.cnt = req.cnt;
    switch (req.cmd()) {
    case COMMON_IDENTIFY:
        // Reply = "DualCoPro_App"-ACK-frame. Wird vom Detector in
        // STATE_START_BL/APP als "Dual CoPro in app" erkannt → STATE_GET_MCU_TYPE.
        // Format: payload[0]=COMMON_ACK, payload[1]=0x01, payload[2..14]="DualCoPro_App"
        out.payload.insert(out.payload.end(), {COMMON_ACK, 0x01});
        {
            const char s[] = "DualCoPro_App";
            for (size_t i = 0; i < sizeof(s) - 1; ++i)
                out.payload.push_back(static_cast<uint8_t>(s[i]));
        }
        break;
    case COMMON_START_APP:
        // cmd=0x03 ohne body ist multimacd's OS_TICK Heartbeat (alle 10 s in
        // DualCoPro-Mode). Reference-Trace 2026-04-30 zeigt Reply als
        // dst=01 cmd=04 body=[01, 05] — nicht COMMON_ACK. cmd=0x03 *mit* body
        // ist Detector-START_APP (klassische ACK-Form).
        if (mode == ModuleMode::Dual && req.payload.size() == 1) {
            out.payload.assign({0x04, 0x01, 0x05});
            break;
        }
        [[fallthrough]];
    case COMMON_START_BL:
        // ACK only — der eigentliche BL/APP-Push kommt im dispatch_-Tail
        // (post_push) als unsolicited Frame.
        out.payload.insert(out.payload.end(), {COMMON_ACK, 0x01});
        break;
    case COMMON_GET_SGTIN: {
        // pivccu STATE_GET_SGTIN: data_len=13, data[0]=1, data[1..12]=SGTIN.
        // Total payload = 14 bytes (cmd + status + 12 sgtin).
        out.payload.insert(out.payload.end(), {COMMON_ACK, 0x01});
        uint8_t sgtin[12];
        fill_sgtin_(sgtin);
        out.payload.insert(out.payload.end(), sgtin, sgtin + 12);
        break;
    }
    default:
        // Unbekannte COMMON-cmds (z.B. FW-Update-Probes) — explizit NACK,
        // damit Detector/multimacd nicht in 3-s-Timeout-Schleifen laeuft.
        out.payload.insert(out.payload.end(), {COMMON_ACK, 0x00});
        break;
    }
    return out;
}

HmuFrame HmuartlgwFrontend::build_trx_response_(const HmuFrame& req) {
    // TRX-Layer (dst=0x01) — *nur* die Dual-Mode-spezifischen Cmds. Legacy-
    // BidCoS-App-Cmds (APP_GET_HMID, APP_SEND etc.) gehen weiterhin zu
    // build_app_response_.
    HmuFrame out;
    out.dst = kHmuDstApp;
    out.cnt = req.cnt;
    switch (req.cmd()) {
    case TRX_GET_MCU_TYPE:
        // pivccu STATE_GET_MCU_TYPE: data_len=2, data[0]=0x01, data[1]=type.
        // Total payload = 3 bytes (cmd + status + module-type).
        out.payload.insert(out.payload.end(),
            {TRX_ACK, 0x01, MODULE_TYPE_RPI_RF_MOD});
        break;
    case TRX_GET_VERSION:
        // pivccu STATE_GET_VERSION: data_len=10, data[0]=0x01, data[1..3]=fw.
        // Total payload = 11 bytes (cmd + status + 3-byte version + 6 bytes
        // unused-padding wie pivccu erwartet).
        out.payload.insert(out.payload.end(),
            {TRX_ACK, 0x01, fw_major_, fw_minor_, fw_patch_,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
        break;
    default:
        out.payload.insert(out.payload.end(), {TRX_ACK, 0x00});
        break;
    }
    return out;
}

HmuFrame HmuartlgwFrontend::build_hmip_response_(const HmuFrame& req) {
    // HMIP-Layer (dst=0x02) — wir haben keinen HmIP-Funk, antworten aber
    // strukturierte Stubs, damit der HmIP-Dienst der Zentrale nicht abbricht.
    HmuFrame out;
    out.dst = kHmuDstHmip;
    out.cnt = req.cnt;
    switch (req.cmd()) {
    case HMIP_GET_DEFAULT_RF_ADDR: {
        // pivccu STATE_GET_HMIP_RF_ADDRESS: data_len=4, data[0]=1, data[1..3]=MAC.
        // Total payload = 5 bytes. HmIP-MAC byte-genau aus MAC abgeleitet —
        // 3 unteren Bytes der WiFi-MAC. Der HmIP-Dienst der Zentrale wird damit nichts anlernen
        // (wir beantworten keine echten HmIP-Frames), aber der Detect-Pfad ist
        // happy.
        uint8_t mac[6] = {};
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        out.payload.insert(out.payload.end(),
            {HMIP_ACK, 0x01, mac[3], mac[4], mac[5]});
        break;
    }
    default:
        // Alle weiteren HmIP-Operationen (Pair, Send, Set-Key) — silent NACK.
        out.payload.insert(out.payload.end(), {HMIP_ACK, 0x00});
        break;
    }
    return out;
}

HmuFrame HmuartlgwFrontend::build_llmac_response_(const HmuFrame& req) {
    // LLMAC-Layer (dst=0x03) — DualCoPro-spezifischer BidCoS-MAC-Pfad.
    HmuFrame out;
    out.dst = kHmuDstLlmac;
    out.cnt = req.cnt;
    switch (req.cmd()) {
    case LLMAC_GET_DEFAULT_RF_ADDR:
        // pivccu STATE_GET_BIDCOS_RF_ADDRESS: data_len=4, data[0]=1, data[1..3]=MAC.
        // Wir liefern unsere HMID — gleicher Wert wie BidCoS-Source-Address.
        out.payload.insert(out.payload.end(),
            {LLMAC_ACK, 0x01, hmid_[0], hmid_[1], hmid_[2]});
        break;
    case LLMAC_GET_SERIAL: {
        // pivccu STATE_GET_SERIAL (auch im DualCoPro-Pfad bei HM_MOD_RPI_PCB-
        // Fallback): data_len=11, data[0]=1, data[1..10]=10 ASCII serial.
        out.payload.push_back(LLMAC_ACK);
        out.payload.push_back(0x01);
        for (size_t i = 0; i < 10; ++i)
            out.payload.push_back(serial_[i] ? static_cast<uint8_t>(serial_[i]) : 0x20);
        break;
    }
    default:
        out.payload.insert(out.payload.end(), {LLMAC_ACK, 0x00});
        break;
    }
    return out;
}

} // namespace culfw32
