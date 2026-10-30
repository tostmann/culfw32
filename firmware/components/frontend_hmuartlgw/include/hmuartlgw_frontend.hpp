// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "frontend.hpp"
#include "hmuartlgw_frame.hpp"
#include "transport.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <array>
#include <memory>
#include <vector>

namespace culfw32 {

// HMUARTLGW-Emulationsfrontend — spricht das HM-MOD-RPI-PCB UART-Protokoll
// (0xfd-binary) gegen FHEM, debmatic oder andere HMUARTLGW-Hosts. Eigener
// TCP-Port (Default 2325). FHEM-Define: `HMUARTLGW uart://<ip>:<port>`.
//
// Phase 0 + 1: Frame-Layer + OS-Layer Dispatcher byte-genau gegen
// FHEM-Trace 2026-04-28. APP-Layer (HMID, send/recv, peer-table) folgt
// in Phase 2.
class HmuartlgwFrontend final : public Frontend {
public:
    const char* name() const override { return "hmuartlgw"; }
    std::span<const ProtocolId> compatible_protocols() const override;
    void   attach(const FrontendServices& services) override;
    void   bind_transport(std::shared_ptr<Transport> t) override;
    esp_err_t start() override;
    esp_err_t stop()  override;

    void attach_dynamic_transport(std::shared_ptr<Transport> t);
    void detach_dynamic_transport(Transport* t);

    // Module-Identitaet — entscheidet welche Init-Sequenz wir simulieren:
    //  Legacy (0)  = HM-MOD-RPI-PCB: Co_CPU_App-Push, HM_DST_HMSYSTEM-Pfad
    //                — kompatibel mit FHEM, Homegear, alle klassischen
    //                HMUARTLGW-Hosts.
    //  Dual   (1)  = RPI-RF-MOD: DualCoPro_App-Push, HM_DST_COMMON+TRX+HMIP+
    //                LLMAC-Pfad — kompatibel mit debmatic-multimacd,
    //                RaspberryMatic-rfd. Alle HmIP-Aktionen sind Stubs (wir
    //                haben keinen HmIP-Funk), BidCoS funktioniert voll.
    enum class ModuleMode : uint8_t {
        Legacy = 0,
        Dual   = 1,
    };
    // Per-Transport-Mode-Variante: stempelt den Slot mit der angegebenen
    // ModuleMode beim attach. Damit kann CULFW32 zwei TCP-Listener parallel
    // betreiben (Port A = Legacy fuer FHEM, Port B = Dual fuer debmatic) ohne
    // Mode-Umschaltung. Default-Variante (ohne mode) nimmt module_mode_ als
    // Default fuer Backward-Compat (UDP-hbrfeth, USB/UART statisch gebunden).
    void attach_dynamic_transport(std::shared_ptr<Transport> t, ModuleMode mode);

private:
    // Slot-Pool fuer per-Transport-State. 4 war zu eng wenn Static (USB-CDC,
    // UART) + Two-Port-TCP (Legacy/Dual) + UDP (hb-rf-eth) parallel attached
    // sind und alte Slot-Eintraege nicht sofort freigegeben werden (rapid
    // reconnect / detach-Race). Bump auf 8 mit minimalem RAM-Overhead.
    static constexpr size_t kMaxTransports = 8;

    // Host-Mode-Detection per-Transport. Homegear/rfd erwartet andere
    // Init-Sequenz als FHEM (Co_CPU_BL-zuerst-dann-CHANGE_APP-dann-Co_CPU_App
    // vs FHEM-style Co_CPU_App-direkt). Wir starten im AwaitingProbe-Modus
    // und erkennen den Host am ersten Befehl.
    enum class HostMode : uint8_t {
        AwaitingProbe,    // Hello (Co_CPU_BL) gepushed, warten auf Host-Probe
        Fhem,             // OS_GET_APP zuerst empfangen → FHEM-Pfad
        Homegear,         // OS_CHANGE_APP zuerst empfangen → Homegear/rfd-Pfad
        DetectorAwaitBL,  // OS_GET_APP→Co_CPU_App, dann CHANGE_APP empfangen
                          // → wir haben Co_CPU_BL gepusht, warten auf 2.
                          //   CHANGE_APP (debmatic detect_radio_module-Pfad)
        Active,           // Init durch — normale Operation
    };

    struct PerTransport {
        Transport*       owner = nullptr;
        HmuFrameDecoder  decoder;
        HostMode         mode  = HostMode::AwaitingProbe;
        // Pro-Slot Module-Mode (Legacy = HM-MOD-RPI-PCB / Dual = RPI-RF-MOD).
        // Wird beim attach gesetzt — entweder explizit ueber attach_dynamic_transport
        // (Transport, ModuleMode) oder als Default aus module_mode_ kopiert.
        ModuleMode       module = ModuleMode::Legacy;
        // Pending Hello-Push-Timer Handle. Wird beim attach gesetzt (200ms one-shot
        // + Legacy 3s repeat). Beim detach gestoppt+geloescht — sonst feuert das
        // Trampoline gegen das schon freigegebene Transport-Objekt (UAF/ABA bei
        // Memory-Reuse → doppelte Hello-Pushes oder Crash).
        void*            hello_timer = nullptr;
    };

    void on_rx_data_(std::span<const uint8_t> bytes, Transport* src);
    void dispatch_(const HmuFrame& req, Transport* src);
    PerTransport* slot_for_(Transport* t);  // mtx_ MUSS vom Caller gehalten sein
    void send_to_(Transport* t, const HmuFrame& f);
    void broadcast_(const HmuFrame& f);
    // Unsolicited "Co_CPU_App"-Push beim Connect. Homegear-HomeMaticBidCoS
    // (`Hm-Mod-Rpi-Pcb.cpp`) wartet passiv 30 s darauf; rfd vermutlich
    // genauso. FHEM dagegen sendet aktiv `OS_GET_APP` und reagiert reaktiv.
    // **Direkter Push beim attach bricht FHEM** (TCP-disconnect-Loop, Bytes
    // treffen FHEM vor seinem Init-Setup) — erste Iteration vom 2026-04-29.
    // Loesung: 200 ms Delay via esp_timer_start_once. FHEM ist dann ready,
    // Homegear/rfd warten ohnehin passiv — beide kompatibel.
    void send_hello_(Transport* t);
    void schedule_hello_(Transport* t);
    static void hello_timer_trampoline_(void* arg);

    // OS-Layer (Phase 1) — alle Antwort-Formate byte-genau gegen FHEM-Trace
    // 2026-04-28 (siehe Memory hmuartlgw_protocol_spec.md).
    HmuFrame build_os_response_(const HmuFrame& req);
    // APP-Layer (Phase 2): APP_SEND → AskSinRF::tx, GET_HMID/SET_HMID, peer-table stub.
    HmuFrame build_app_response_(const HmuFrame& req);
    HmuFrame handle_app_send_(const HmuFrame& req);

    // RX-Subscription auf AskSinRF — wrappt RxEvent als APP_RECV Frame und
    // broadcastet an alle aktuellen HMUARTLGW-Transports.
    void on_asksin_rx_(const struct RxEvent& ev);

    std::vector<std::shared_ptr<Transport>> transports_;
    std::array<PerTransport, kMaxTransports> per_transport_{};
    SemaphoreHandle_t mtx_ = nullptr;

    FrontendServices services_{};
    bool started_ = false;

    // 3-Byte HMID, persistiert in EE_HM_ID. Default = leer/0xFFFFFF.
    uint8_t hmid_[3] = {0xCA, 0xFE, 0x42};
    // 10-Byte ASCII Serial. Bei Init via attach() aus MAC abgeleitet.
    // Buffer 12 = 10 Wire-Chars + NUL + 1 (snprintf in attach() schreibt
    // "CUL%02X%02X%02X%02X" = 11 chars + NUL; Wire-Loop liest nur die ersten
    // 10, byte-genau gegen echte HM-MOD-RPI-PCB verifiziert).
    char    serial_[12] = "CULFW32001";
    // Firmware-Version-Triplet. Wir reporten 1.4.1 damit FHEM kein
    // "(outdated)" zeigt (Floor laut Source: < 0x010401 → outdated).
    uint8_t fw_major_ = 0x01;
    uint8_t fw_minor_ = 0x04;
    uint8_t fw_patch_ = 0x01;

    // Counter fuer ungefragte APP_RECV-Frames. Steigt mit jeder rx-Broadcast,
    // wraparound bei 0xFF -> 0x00. Startet bei 0 (verifiziert 2026-04-30
    // gegen echte HM-MOD-RPI-PCB am Pi5-Header).
    uint8_t rx_cnt_ = 0;
    // AskSin-CNT-Counter fuer LLMAC SEND (Mode-1). Multimacd liefert keinen
    // CNT in dst=0x03 cmd=0x06 — die echte DualCoPro-Firmware inkrementiert
    // intern. Wir tun's auch.
    uint8_t llmac_tx_cnt_ = 0;

public:
    // Module-Identitaet — entscheidet welche Init-Sequenz wir simulieren:
    //  Legacy (0)  = HM-MOD-RPI-PCB: Co_CPU_App-Push, HM_DST_HMSYSTEM-Pfad
    //                — kompatibel mit FHEM, Homegear, alle klassischen
    //                HMUARTLGW-Hosts.
    //  Dual   (1)  = RPI-RF-MOD: DualCoPro_App-Push, HM_DST_COMMON+TRX+HMIP+
    //                LLMAC-Pfad — kompatibel mit debmatic-multimacd,
    //                RaspberryMatic-rfd. Alle HmIP-Aktionen sind Stubs (wir
    //                haben keinen HmIP-Funk), BidCoS funktioniert voll.
    void  set_module_mode(ModuleMode m);
    ModuleMode module_mode() const { return module_mode_; }

private:
    // Default-Mode fuer Transports die ohne explizite Mode-Angabe attached
    // werden (UDP hbrfeth, USB-CDC, UART). Mit dem 2-Port-Refactor wird das
    // im Default Dual gesetzt vom main.cpp (UDP hbrfeth ist immer debmatic),
    // alte EE-Default-Logik bleibt fuer UART/USB-CDC erhalten.
    ModuleMode module_mode_ = ModuleMode::Legacy;
    // Helper: bauen die format-Identifier-Pushs (Co_CPU_BL/App vs.
    // DualCoPro_App auf HM_DST_COMMON je nach mode).
    HmuFrame   build_app_push_(uint8_t cnt, ModuleMode mode) const;
    HmuFrame   build_bl_push_(uint8_t cnt, ModuleMode mode) const;
    // DualCoPro-Mode-Replies — alle Probes byte-genau pivccu detect_radio_module
    // radiomoduledetector.cpp:138-326. Werden nur in Dual-Mode aufgerufen;
    // Legacy-Mode ignoriert die HM_DST_COMMON/TRX/HMIP/LLMAC-Probes wie bisher.
    HmuFrame   build_common_response_(const HmuFrame& req, ModuleMode mode);
    HmuFrame   build_trx_response_(const HmuFrame& req);
    HmuFrame   build_hmip_response_(const HmuFrame& req);
    HmuFrame   build_llmac_response_(const HmuFrame& req);
    // SGTIN: 12 bytes. Wir generieren statisch aus MAC + HMID, damit es ueber
    // Reboots stabil bleibt. Pattern: `3014F711<MAC4..5><HMID0..2>` (3014=eq-3
    // company-prefix; F711=class HM-MOD; sieht legitim aus fuer Detector).
    void       fill_sgtin_(uint8_t* sgtin12) const;
};

} // namespace culfw32
