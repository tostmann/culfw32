// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "rf_protocol.hpp"
#include "cc1101_driver.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <atomic>
#include <span>

namespace culfw32 {

// Typed AskSin-RX-Frame. Lebensdauer = RxCallback-Aufruf. Wer ihn ueber den
// Aufruf hinaus braucht, kopiert. Inhalt ist bereits DECRYPTED (Crypt-Decoder
// im RX-Task angewandt, byte-genau a-culfw rf_asksin.c:175ff).
struct AskSinFrame {
    static constexpr size_t kMaxPayload = 64;  // a-culfw MAX_ASKSIN_MSG=30 (50 mit FUP), mit Reserve
    uint8_t  len;                              // payload length (= byte 0 des Frames, decrypted)
    uint8_t  data[kMaxPayload];                // [0]=len, [1..len]=decrypted payload
    uint8_t  rssi_raw;                         // CC1101 RSSI status byte
    uint8_t  lqi_raw;                          // CC1101 LQI status byte
};

// TX-Payload (Klartext). MoritzTxFrame-Aequivalent. AskSin-Crypt wird in tx()
// VOR dem FIFO-Write angewandt.
struct AskSinTxFrame {
    uint8_t len;
    uint8_t data[AskSinFrame::kMaxPayload];    // [0]=len, [1..len]=Klartext-Payload
    // Wartezeit zwischen STX und FIFO-Schreiben ohne Burst-Bit, in ms. Der Chip
    // sendet so lange Praeambel, mindestens aber die 4 Byte aus MDMCFG1.
    // a-culfw (rf_asksin.c:276) wartet 10 ms. Echte eQ-3-Sender (RFUSB,
    // HM-TC-IT, Wettersensor) senden nur die 4 Byte: 3,3 ms vom Traegerstart
    // bis zum Sync, mit HackRF gemessen am 01.11.2026. Der HMU-Pfad nimmt 0.
    uint8_t  extra_preamble_ms = 10;
    // Wenn ungleich 0: nomineller Rahmenstart (Sync minus 4 Byte Praeambel,
    // esp_timer, untere 32 Bit), zu dem die Sendung beginnen soll. tx() wartet
    // unter der Chipsperre mit µs-Aufloesung und strobt STX entsprechend
    // frueher. Liegt der Zeitpunkt schon zurueck, wird sofort gesendet. Nur
    // ohne Burst-Bit: die 360 ms folgen erst nach dem STX und sind im Vorlauf
    // nicht eingerechnet (kein Aufrufer braucht das).
    uint32_t tx_at_us = 0;
    // true: genau ein STX-Versuch. Weist das CCA ihn ab, kehrt tx() mit
    // ESP_ERR_NOT_FINISHED zurueck, ohne gesendet zu haben (Wiederholungen
    // der HMU-Sendeauftraege: ist der Kanal belegt, laeuft womoeglich gerade
    // die Antwort ein, und die Wiederholung waere ueberfluessig).
    bool     cca_once = false;
    // Wenn ungleich 0: spaetester Zeitpunkt fuer das STX (esp_timer, untere
    // 32 Bit). Ist er erreicht, bevor das CCA den Kanal freigibt, kehrt tx()
    // mit ESP_ERR_TIMEOUT zurueck, ohne gesendet zu haben (Selbstquittung:
    // eine Quittung nach der Wiederholung des Geraets nuetzt nichts mehr).
    // Zusammen mit cca_once gilt cca_once (genau ein Versuch).
    uint32_t tx_until_us = 0;
    // Ausgabe von tx(): nomineller Rahmenstart der Sendung (Sync minus 4 Byte,
    // esp_timer, untere 32 Bit), gesetzt sobald der Chip sendet.
    mutable uint32_t sent_at_us = 0;
};

class AskSinRF final : public RfProtocol {
public:
    ProtocolId           id() const override { return ProtocolId::AskSin; }
    const CC1101Profile& profile() const override;

    esp_err_t start(uint8_t slot_id, CC1101Driver& drv) override;
    esp_err_t stop()  override;
    esp_err_t tx(const void* typed_payload, size_t payload_size) override;
    void      on_rx(RxCallback cb) override { rx_cb_ = std::move(cb); }

    // Letzter TX-Error-Code fuer Frontend-Output. CCA-Timeout ist a-culfw-konform
    // `ERR:CCA\r\n`; die anderen Pfade lehnen sich an Moritz' ZERR1/2/3 an.
    enum class TxError : uint8_t { None, NotInRx, CcaTimeout, NoReturnRx, FifoNotEmpty };
    TxError last_tx_error()    const { return last_tx_err_; }
    uint8_t last_tx_marcstate() const { return last_tx_marc_; }

    // Letzter on-air TX-Header — Klartext-Bytes des Frames der zuletzt an den
    // CC1101 uebergeben wurde (vor encrypt). Antwortet "ist APP_SET_HMID
    // wirklich aktiv?" und aehnliche Fragen ohne RF-Sniffer.
    struct LastTx {
        uint32_t src_hmid;   // Bytes 4..6, 24-bit. 0 = noch keine TX.
        uint32_t dst_hmid;   // Bytes 7..9, 24-bit.
        uint8_t  cnt;        // Byte 1, on-air message counter.
        bool     valid;      // false bevor erstmals ge-tx't.
    };
    static LastTx last_tx() {
        LastTx l{};
        l.src_hmid = s_last_tx_src_hmid_.load(std::memory_order_relaxed);
        l.dst_hmid = s_last_tx_dst_hmid_.load(std::memory_order_relaxed);
        l.cnt      = s_last_tx_cnt_.load(std::memory_order_relaxed);
        l.valid    = s_last_tx_valid_.load(std::memory_order_relaxed);
        return l;
    }

    // Crypt/Decrypt-Helfer (oeffentlich fuer Tests). Modifizieren in-place.
    // Byte-genau a-culfw `asksin_send` (encrypt) und `rf_asksin_task` (decrypt).
    static void encrypt(uint8_t* msg);
    static void decrypt(uint8_t* msg);

    // Luftzeit eines Rahmens mit Laengenbyte `len` ohne Zusatzpraeambel:
    // 4 Byte Praeambel + 4 Byte Sync (SYNC_MODE 3) + Laenge + Nutzdaten +
    // 2 Byte CRC, 0,8006 ms je Byte bei 9993 Bd. Nur das Normalprofil, nicht
    // das FUP-Profil (AR, ~50 kbit/s).
    static constexpr uint32_t air_us(uint8_t len) { return (len + 11u) * 8006u / 10u; }

    // FUP-Mode (Firmware-Update via OTA fuer HomeMatic-Geraete) — `AR` statt `Ar`.
    // Bei aktivem Mode wird beim naechsten start() zusaetzlich ASKSIN_UPDATE_CFG
    // ueber das normale ASKSIN_CFG drueber geschrieben (hoehere Datarate ~50 kbps,
    // andere AGC-/IF-Werte). Byte-genau a-culfw rf_asksin.c:104-110.
    void set_update_mode(bool on) { update_mode_ = on; }
    bool update_mode() const      { return update_mode_; }

    // TX-only-Bind: setzt drv_/slot_id_ ohne Slot-Init und ohne RX-Task. Wird
    // von HmDualRF benutzt, das den CC1101 selbst initialisiert (HMIP_CFG) und
    // nur die AskSin-TX-Logik (encrypt + CCA + Burst-Preamble) leihen will.
    void bind_for_tx(uint8_t slot_id, CC1101Driver& drv) {
        slot_id_ = slot_id;
        drv_     = &drv;
        // `enabled_` MUSS hier mit, sonst weist tx() den geliehenen Sendepfad
        // ab: seit 0.1.842 gatet tx() ueber das Flag statt ueber den Zeiger.
        // Heute ruft das niemand (der Kommentar oben nennt HmDualRF, die Stelle
        // existiert nicht) — genau deshalb waere es beim ersten Aufrufer ein
        // stiller Fehlschlag geworden statt eines Compilerfehlers.
        enabled_ = true;
    }

private:
    static void rx_task_trampoline(void* arg);
    void        rx_task_loop();
    void        drain_fifo();        // alle fertigen Rahmen: read, SRX, decrypt + callback
    void        recover_rx_(bool heartbeat);   // Chip aus IDLE/Overflow zurueck nach RX
    void        rearm_rx_();         // SRX aus IDLE, aber nicht waehrend tx()
    static void IRAM_ATTR gdo2_isr(void* arg);

    // Bis t_us (esp_timer, untere 32 Bit) warten: grob im 10-ms-Tick, den Rest
    // ueber einen esp_timer-One-Shot, der wait_sem_ gibt (kein aktives Warten
    // ueber Millisekunden). Nur aus tx(); setzt eine Sendung zur Zeit je
    // Instanz voraus (Slot-Sperre in RfManager::tx, dazu in aller Regel die
    // Chipsperre), denn wait_sem_/wait_tmr_ gibt es einmal.
    void        sleep_until_(uint32_t t_us);
    static void wait_cb_(void* arg);
    SemaphoreHandle_t  wait_sem_ = nullptr;
    esp_timer_handle_t wait_tmr_ = nullptr;

    // Zeitpunkt der letzten GDO2-Flanke (CRC OK = Paketende), gesetzt in der
    // ISR, in drain_fifo() beim Lesen verbraucht (0 = keine frische Flanke).
    std::atomic<uint32_t> gdo2_edge_us_{0};

    // Haelt tx() fuer die ganze Sendung; der RX-Task liest, flusht (SIDLE/SFRX)
    // und strobt SRX nur, wenn er ihn ohne Warten bekommt, und gibt ihn vor dem
    // Callback frei. Ein SIDLE oder SRX nach dem STX beendet die Sendung
    // (SWRS061I 19.4, S. 52). Angelegt in start(); ohne (bind_for_tx) ungesperrt.
    SemaphoreHandle_t chip_mtx_ = nullptr;

    CC1101Driver* drv_         = nullptr;

    // true zwischen start() und stop().  Der Treiberzeiger bleibt dabei

    // immer gueltig (Wert-Member des Slots); gegated wird ueber dieses Flag.

    // volatile: gesetzt vom Frontend-Task, gelesen vom RX-Task (Prio 8).

    volatile bool enabled_ = false;
    uint8_t       slot_id_     = 0;
    RxCallback    rx_cb_;
    void*         task_handle_ = nullptr;
    volatile bool task_running_ = false;

    TxError last_tx_err_ = TxError::None;
    uint8_t last_tx_marc_ = 0;
    bool    update_mode_ = false;
    // Sendezeit unter 10 ms, die noch nicht in den Sendekredit gebucht ist
    // (0..9). Nur in tx() angefasst; serialisiert durch die Slot-Sperre, die
    // RfManager::tx ueber die ganze Sendung haelt (dazu chip_mtx_).
    uint8_t air_rest_ms_ = 0;

    // Diagnostik-State ist process-global: HmDualRF haelt eine zweite
    // AskSinRF-Instanz als TX-Helper, beide tx-Pfade landen am selben CC1101.
    // Statische Atomics → "letzte on-air TX" ist eindeutig, egal welche Instanz.
    static std::atomic<uint32_t> s_last_tx_src_hmid_;
    static std::atomic<uint32_t> s_last_tx_dst_hmid_;
    static std::atomic<uint8_t>  s_last_tx_cnt_;
    static std::atomic<bool>     s_last_tx_valid_;
};

} // namespace culfw32
