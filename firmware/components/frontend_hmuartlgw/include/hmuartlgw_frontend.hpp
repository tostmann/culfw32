// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "frontend.hpp"
#include "hmuartlgw_frame.hpp"
#include "transport.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <array>
#include <initializer_list>
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
        // true, sobald diese Verbindung einen gueltigen Rahmen geschickt hat und
        // AskSin auf Slot 0 eingefordert wurde (siehe claim_radio_()).
        bool             radio_claimed = false;
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
    // APP-Layer (Phase 2): APP_SEND → AskSinRF::tx, GET_HMID/SET_HMID; im
    // Legacy-Modus zusaetzlich ADD_PEER/REMOVE_PEER mit Peer-Tabelle.
    // src: der Transport, von dem die Anfrage kommt (fuer Sendeauftraege).
    HmuFrame build_app_response_(const HmuFrame& req, ModuleMode mod, Transport* src = nullptr);
    // legacy_src != nullptr: Legacy-Klient; ein Rahmen mit Quittungswunsch
    // wird dann zum Sendeauftrag (siehe SendJob) und die Antwort kommt spaeter
    // -- zurueck kommt ein Rahmen ohne Nutzlast ("noch keine Antwort").
    HmuFrame handle_app_send_(const HmuFrame& req, Transport* legacy_src = nullptr);
    // Slot 0 auf AskSin, falls ein anderer Frontend umgeschaltet hat.
    void     ensure_asksin_();

    // Sendeauftrag (Legacy, APP_SEND mit Quittungswunsch an eine Adresse
    // ungleich 000000). FHEM (00_HMUARTLGW.pm) kennt als Antwort darauf
    // `04 03 <info> <rssi> <Antwort>` (ACK_WITH_RESPONSE, derselbe Auswertepfad
    // wie `05`) und `04 04` (ACK_EUNKNOWN, geloggt als "no response?"); so
    // antwortet der Stick jetzt, wenn das Geraet geantwortet hat bzw. nicht.
    // Bis 0.2.7 kam sofort `04 01`, und blieb
    // die Antwort aus, wiederholte niemand: CUL_HM wartet bei Weck- und
    // Lazy-Config-Geraeten dann auf deren naechstes Aufwachen (Mitschnitt
    // 01.11.2026, HM-SEC-SC-2 und HM-SEN-MDIR-SM). Jetzt sendet der Sende-Task,
    // wartet und wiederholt hoechstens kMaxRetries Mal (Burst-Rahmen gar nicht),
    // jeweils kRetryAfterUs nach dem Rahmenstart der letzten Sendung (124 ms
    // Antwortzeit + 128 ms);
    // nach der letzten wartet er bis kLastWaitUs, langsame Geraete brauchten
    // fuer eine Liste bis ~470 ms. Jeder Auftrag endet spaetestens nach
    // kJobDeadlineUs: FHEM sendet nach 10 s (SEND_TIMEOUT) selbst neu und
    // verwirft ein spaeteres `04` als "delayed ACK" samt Antwort (Review 0208
    // F2). Ein neues OS_GET_APP/OS_CHANGE_APP des Auftraggebers (FHEM-Init)
    // verwirft den Auftrag. Die Antwort
    // erkennt der RX-Pfad (on_asksin_rx_) an Zaehler, Absender und Ziel und
    // schickt sie dem Auftraggeber als `04 03` statt als `05`. Grenze: Geraete
    // zaehlen bei mehrteiligen Antworten selbst hoch (HM-TC-IT-WM-W-EU: Teil 2
    // traegt Zaehler + 1). CUL_HM zaehlt je Nachricht +1, bei Listen-Anfragen
    // +16 (am 01.11. FA -> 0B). Traegt FHEMs naechste Anfrage den Zaehler von
    // Teil 2 und kommt vor ihm, gilt Teil 2 als Antwort und die echte Antwort
    // kommt als `05` -- jeder Rahmen erreicht FHEM trotzdem genau einmal. FHEM
    // sendet nach einem Rahmen mit Quittungswunsch erst 300 ms spaeter an
    // dasselbe Geraet (nextSend), Teil 2 lag am 01.11. davor.
    struct SendJob {
        bool       active = false;
        uint32_t   gen    = 0;          // zaehlt je Auftrag hoch
        Transport* owner  = nullptr;
        uint8_t    hmu_cnt = 0;         // Zaehler der HMU-Anfrage, FHEM ordnet `04` daran zu
        uint8_t    cnt     = 0;         // BidCoS-Zaehler
        uint8_t    src[3]{};
        uint8_t    dst[3]{};
        uint8_t    msg[64]{};           // [0] = Laenge, [1..] = Klartext ab Zaehler
        uint32_t   t_start = 0;         // Annahme (esp_timer, untere 32 Bit)
    };
    static constexpr uint32_t kJobDeadlineUs  = 5'000'000;
    static constexpr uint32_t kRetryLockMs    = 300;   // Slot-Sperre fuer Wiederholungen
    static constexpr uint32_t kRetryAfterUs = 252'000;
    static constexpr uint32_t kLastWaitUs   = 500'000;
    static constexpr int      kMaxRetries   = 2;
    SendJob           job_{};
    SemaphoreHandle_t job_mtx_   = nullptr;   // Reihenfolge: mtx_ -> job_mtx_
    void*             send_task_ = nullptr;
    static void send_task_trampoline_(void* arg);
    void        send_task_loop_();
    // Auftrag gen mit `04 <ack...>` abschliessen, falls er noch offen ist.
    void        finish_job_(uint32_t gen, std::initializer_list<uint8_t> ack);

    // Ein echtes Modul empfaengt immer BidCoS. Steht Slot 0 auf einer anderen
    // Klasse (z. B. `X21` ueber den culfw-Port), schaltet der erste gueltige
    // Rahmen einer HMU-Verbindung auf AskSin zurueck. mtx_ gehalten.
    void claim_radio_();

    // Peer-Tabelle des HM-MOD-RPI-PCB (Legacy). FHEM verwirft seine eigene
    // Quittung an jedes Geraet, das es dort eingetragen hat (00_HMUARTLGW.pm:
    // "Acks are generally send by HMUARTLGW autonomously") — das Modul muss
    // fuer diese Geraete selbst quittieren. Im Dual-Modus quittiert multimacd
    // selbst; dort wird nichts eingetragen. Leer, sobald keine Legacy-
    // Verbindung mehr besteht (FHEM traegt nach jedem Init alles neu ein).
    // 256 * 4 Byte. Ein volles Modul meldet "nicht eingetragen", und FHEM
    // loggt dann dieselbe "aesKey"-Meldung wie bei einer falschen Antwort.
    static constexpr size_t kMaxPeers = 256;
    struct Peer {
        uint8_t id[3];
        // ADD_PEER Byte 5 ("Wakeup?", 00_HMUARTLGW.pm UpdatePeerReq). FHEM
        // setzt es fuer Weck- und Lazy-Config-Geraete, solange Befehle fuer
        // sie anstehen (10_CUL_HM.pm hmInitMsgUpdt: flgs |= 0x02, dann
        // "init:" -> neues ADD_PEER), und nimmt es danach zurueck. Das Modul
        // quittiert solche Geraete mit dem Wach-Bit (Flag 0x81), damit sie
        // auf die Befehle warten — siehe auto_ack_().
        bool    wakeup;
    };
    int      peer_index_(const uint8_t* id) const;     // peer_mtx_ MUSS gehalten sein; -1 = unbekannt
    bool     peer_add_(const uint8_t* id, bool wakeup); // dito; aktualisiert wakeup bei bekanntem Peer
    void     peer_remove_(const uint8_t* id);           // dito
    HmuFrame build_peer_reply_(const HmuFrame& req, bool added) const;
    // Rahmen an die eigene HMID mit Quittungswunsch von einem eingetragenen
    // Peer quittieren: die Quittung beginnt (nominell) kAutoAckStartUs nach dem
    // Start des Rahmens (air_start_us). Ist er vor dem Warten an der Slot-Sperre
    // schon kAutoAckLateMs alt, entfaellt sie; das Warten selbst ist auf diese
    // Frist begrenzt, CCA danach nicht.
    // Laeuft im AskSin-RX-Task, ohne mtx_, nach der Weitergabe an die Hosts.
    static constexpr uint32_t kAutoAckStartUs = 124'000;
    static constexpr uint32_t kAutoAckLateMs  = 200;
    void     auto_ack_(const struct AskSinFrame& af, uint32_t air_start_us);

    // RX-Subscription auf AskSinRF — wrappt RxEvent als APP_RECV Frame und
    // broadcastet an alle aktuellen HMUARTLGW-Transports.
    void on_asksin_rx_(const struct RxEvent& ev);

    std::vector<std::shared_ptr<Transport>> transports_;
    std::array<PerTransport, kMaxTransports> per_transport_{};
    SemaphoreHandle_t mtx_ = nullptr;

    // Eigene Sperre fuer Peer-Tabelle und hmid_-Schreiben: der RX-Task fragt
    // sie je Rahmen ab und soll nicht an mtx_ warten, den on_rx_data_ ueber
    // eine ganze Sendung haelt. Reihenfolge: mtx_ -> peer_mtx_.
    SemaphoreHandle_t peer_mtx_ = nullptr;
    std::array<Peer, kMaxPeers> peers_{};
    size_t peer_count_ = 0;

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
