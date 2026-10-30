// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "esp_err.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic>
#include <cstdint>
#include <span>

namespace culfw32 {

// Bare CC1101 SPI-Driver. Kennt keine Protokolle.
// Wird vom CC1101Slot gehalten; RF-Klassen greifen via Slot zu.
//
// Verifizierbarkeit: TI-Datenblatt SWRS061I.
//   - SPI Mode 0 (CPOL=0, CPHA=0), Table 22 p.30
//   - Manual Reset: Section 19.1.2, p.51, Figure 27
//   - Header-Byte: bit7=R/Wbar, bit6=Burst, bits5..0=addr; Section 10.2 p.31
//   - Status-Register Read braucht Burst-Bit gesetzt (Header 0xC0|addr)
class CC1101Driver {
public:
    struct Config {
        int      spi_host;       // SPI2_HOST (1) oder SPI3_HOST (2)
        int      pin_cs;
        int      pin_gdo0;
        int      pin_gdo2;
        int      pin_mosi;
        int      pin_miso;
        int      pin_sck;
        int      spi_clock_hz;
    };

    // Standard CC1101 register addresses.
    static constexpr uint8_t REG_PARTNUM   = 0x30;  // status
    static constexpr uint8_t REG_VERSION   = 0x31;  // status
    static constexpr uint8_t REG_MARCSTATE = 0x35;  // status
    static constexpr uint8_t REG_FSCAL1    = 0x25;  // config, Kalibrier-Ergebnis
    static constexpr uint8_t REG_FSCTRL0   = 0x0C;  // config, Frequenz-Offset (signed)
    static constexpr uint8_t REG_FREQEST   = 0x32;  // status, geschaetzter RX-Offset
    static constexpr uint8_t REG_PKTSTATUS = 0x38;  // status, Bit 3 = SFD
    static constexpr uint8_t REG_RXBYTES   = 0x3B;  // status, Bit 7 = RXFIFO_OVERFLOW
    static constexpr uint8_t REG_FIFO      = 0x3F;  // RX-FIFO (lesend)
    // Strobe commands.
    static constexpr uint8_t SRES  = 0x30;
    static constexpr uint8_t SCAL  = 0x33;
    static constexpr uint8_t SRX   = 0x34;
    static constexpr uint8_t SIDLE = 0x36;
    static constexpr uint8_t SFRX  = 0x3A;
    static constexpr uint8_t SNOP  = 0x3D;
    // MARCSTATE-Werte (SWRS061I Fig. 25 p.50 / Register 0x35), Maske 0x1F.
    static constexpr uint8_t MARC_IDLE             = 0x01;
    static constexpr uint8_t MARC_RX               = 0x0D;
    static constexpr uint8_t MARC_RX_END           = 0x0E;
    static constexpr uint8_t MARC_RX_RST           = 0x0F;
    static constexpr uint8_t MARC_TXFIFO_UNDERFLOW = 0x16;
    static constexpr uint8_t MARC_RXFIFO_OVERFLOW  = 0x11;

    // Erzeugt den Bus-Mutex (siehe spi_mtx_). Der CC1101 wird von mehreren
    // FreeRTOS-Tasks geteilt (RX-Tasks, TX, ConfigRegistry-Stats-Getter ueber
    // RfManager::cc_read_*). Ohne Serialisierung kann ein zweiter Task waehrend
    // cs_low_wait_ready() eines anderen preempten und mit eigenem CS-Toggle
    // dessen Transfer mit deassertiertem CS laufen lassen -> 0xFF/Datenmuell
    // (plausibler Root vieler "RX intermittierend"-Faelle). Kein ISR-SPI-Zugriff
    // existiert (GDO-ISR macht nur vTaskNotifyGiveFromISR), daher ist ein
    // FreeRTOS-Mutex (kein Spinlock) zulaessig.
    CC1101Driver() { spi_mtx_ = xSemaphoreCreateMutex(); }

    esp_err_t init(const Config& cfg);

    // Liest ein Konfig-Register (0x00..0x2E). Header = 0x80 | addr.
    esp_err_t read_reg(uint8_t addr, uint8_t& value);
    // Schreibt ein Konfig-Register. FSCTRL0 (0x0C) wird dabei durch den
    // kalibrierten Board-Offset ersetzt (siehe set_freq_offset).
    esp_err_t write_reg(uint8_t addr, uint8_t value);
    // Wie write_reg, aber ohne FSCTRL0-Ueberschreibung (Cw-Pfad, Messungen).
    esp_err_t write_reg_raw(uint8_t addr, uint8_t value);

    // Liest ein Status-Register (0x30..0x3D). Header = 0xC0 | addr (Burst-Bit Pflicht!).
    esp_err_t read_status(uint8_t addr, uint8_t& value);

    esp_err_t write_burst(uint8_t addr, std::span<const uint8_t> data);
    esp_err_t read_burst(uint8_t addr, std::span<uint8_t> data);

    // Command-Strobe (SRES, SIDLE, SRX, STX, ...). Nur Header, keine Daten.
    esp_err_t strobe(uint8_t cmd);
    // Wie strobe(cmd), erfasst aber das Status-Byte UNTER DEMSELBEN Bus-Lock
    // wie der Strobe und schreibt es in status_out (= demselben Byte das auch
    // last_status_ erhaelt). Wer "strobe(SRX); ... last_status()" race-frei
    // braucht, nutzt diese Ueberladung — last_status() spiegelt nur die letzte
    // Bus-Transaktion und kann bei geteiltem Bus von einer anderen Task stammen.
    esp_err_t strobe(uint8_t cmd, uint8_t& status_out);

    // MARCSTATE (0x35) maskiert auf 0x1F. 0xFF wenn der SPI-Read scheitert.
    uint8_t marc_state();

    // RX-FIFO leeren — datenblattkonform. SWRS061I p.33: "A SFTX or SFRX command
    // strobe can only be issued in the IDLE, TXFIFO_UNDERFLOW, or RXFIFO_OVERFLOW
    // states." Ein SFRX aus RX heraus (culfw/a-culfw rf_asksin_reset_rx macht das)
    // verpufft: der Muell bleibt im FIFO stehen. Deshalb erst SIDLE, auf einen
    // zulaessigen Zustand warten, dann flushen. Laesst den Chip in IDLE zurueck —
    // der Aufrufer strobed anschliessend selbst SRX.
    esp_err_t flush_rx();

    // PLL-Lock-Pruefung. SWRS061I §22.1 p.58: "As an alternative the user can read
    // register FSCAL1. The PLL is in lock if the register content is different from
    // 0x3F." Ohne Lock empfaengt/sendet der Chip STILL nichts — kein Fehlerbit,
    // keine Meldung, nur ausbleibende Frames.
    // Was das genau ist: FSCAL1 ist das ERGEBNIS der letzten (Auto-)Kalibrierung,
    // kein Live-Lock-Detektor. 0x3F = die Kalibrierung hat keinen Lock gefunden —
    // on hardware bewiesen 2026-08-17 (FREQ2=0x00 -> jede SCAL endet auf 0x3F).
    // Ein Lock, der NACH gelungener Kalibrierung durch Drift verloren geht, aendert
    // FSCAL1 nicht; dafuer nennt das Datenblatt den GDO-Lock-Detektor (IOCFGx=0x0A)
    // oder eine erneute Kalibrierung. Beides deckt dieser Check NICHT ab.
    bool pll_locked();

    // SCAL + Lock-Pruefung. Muss aus IDLE gerufen werden (SCAL ist nur dort
    // zulaessig). Bei fehlendem Lock bis `retries` Mal nachkalibrieren — SWRS061I
    // §22.1 p.58: "The PLL must be recalibrated until PLL lock is achieved if the
    // PLL does not lock the first time." Die Kalibrierwerte gelten nur ueber einen
    // endlichen Temperaturbereich (typ. +-40 C, ebd.) — Neukalibrieren gehoert
    // deshalb zum Betrieb; wann, entscheidet der Aufrufer (heute: Mode-Start und
    // Heartbeat bei FSCAL1==0x3F). Hinweis: mit MCSM0.FS_AUTOCAL=01 (alle CFGs
    // ausser Moritz) kalibriert der Chip beim folgenden SRX ohnehin erneut; die
    // Pruefung HIER sichert die manuelle Kalibrierung, die Auto-Kalibrierung
    // sieht erst der Heartbeat (liest FSCAL1 in RX).
    // Ersetzt das fruehere `strobe(SCAL); ets_delay_us(4000);` der RF-Klassen:
    // wartet auf MARCSTATE==IDLE statt blind, und prueft das Ergebnis.
    // Gibt ESP_ERR_INVALID_STATE zurueck wenn am Ende kein Lock steht.
    esp_err_t calibrate(int retries = 2);

    // Zaehler fuer ?stats — ohne sie bleibt eine Rekalibrierung unsichtbar und
    // verdeckt genau die Ursache, die sie behebt. Drei getrennte Groessen:
    //   pll_lost     — Heartbeat hat FSCAL1==0x3F in RX gesehen (Lock-Verlust
    //                  erkannt), unabhaengig davon, wie die Recovery ausging.
    //                  Meldet die RF-Klasse per note_lock_lost() VOR calibrate().
    //   pll_relocks  — calibrate() brauchte einen RETRY, bis Lock stand.
    //   pll_failures — calibrate() hat aufgegeben, kein Lock.
    // Nur relock+fail (erste Fassung) machte eine gelungene Recovery im ersten
    // Anlauf unsichtbar — Bench 2026-08-17: FSCAL1:=0x3F -> Recal <1 s, und
    // beide Zaehler blieben 0.
    void     note_lock_lost() { pll_lost_.fetch_add(1, std::memory_order_relaxed); }
    uint32_t pll_lost()     const { return pll_lost_.load(std::memory_order_relaxed); }
    uint32_t pll_relocks()  const { return pll_relocks_.load(std::memory_order_relaxed); }
    uint32_t pll_failures() const { return pll_failures_.load(std::memory_order_relaxed); }

    // -- RX-FIFO der Paketklassen (variable Laenge, APPEND_STATUS, GDO2_CFG=7) --
    // GDO2_CFG=7 meldet einen Rahmen je Flanke: es steigt bei "packet received
    // with CRC OK" und faellt erst, wenn das erste Byte aus dem RX-FIFO gelesen
    // wird (SWRS061I Tab. 41, S. 62). Lagen zwei fertige Rahmen im FIFO, bevor
    // der erste gelesen war, bekam der zweite kein Signal und blieb liegen; jeder
    // neue Rahmen gab danach den vorigen frei (Verzug um n Rahmen bis zum
    // Moduswechsel, gemessen 13.10.2026 an CULFW32 und a-culfw 1.30.1, Klassen
    // mit RXOFF=RX). read_rx_packet() liest deshalb nach dem ersten Rahmen jeden
    // weiteren fertigen nach. Fertig heisst: gerade laeuft kein Paket ein
    // (PKTSTATUS.SFD = 0, S. 94) — dann gehoert jedes Byte im FIFO zu einem
    // beendeten Paket; fehlerhafte raeumt CRC_AUTOFLUSH samt FIFO weg (S. 73).
    enum class RxPacket : uint8_t {
        Frame,      // gelesen: buf[0] = Laenge, buf[1..Laenge] = Nutzlast
        Empty,      // nichts (mehr) im FIFO
        Busy,       // ein Paket laeuft ein; dessen CRC_OK-Flanke kommt noch
        Overflow,   // RXFIFO_OVERFLOW, der Aufrufer flusht
        Resync,     // FIFO-Inhalt unplausibel, bereits geflusht (Chip in IDLE)
        Error,      // SPI-Fehler
    };
    // Obergrenze je Durchlauf: 64-Byte-FIFO, BidCoS-/MAX!-Rahmen belegen >= 13
    // Byte — mehr als vier passen real nicht hinein; 8 begrenzt nur die Schleife.
    static constexpr int kRxFramesPerDrain = 8;
    // first = true fuer den ersten Rahmen nach einer GDO2-Meldung (der Kopf des
    // FIFO gehoert dann sicher zu einem beendeten Paket), false fuer jeden
    // weiteren. buf.size() begrenzt die Laenge (Laenge >= buf.size() = Resync).
    RxPacket read_rx_packet(std::span<uint8_t> buf, uint8_t& rssi, uint8_t& lqi, bool first);
    // Liegt ein fertiger Rahmen im FIFO, den keine Flanke gemeldet hat? Netz
    // unter der Flanke fuer den Heartbeat der RX-Tasks.
    bool rx_frame_waiting();
    // RXBYTES stabil gelesen (Rohwert, Bit 7 = Overflow). SWRZ020E S. 5/6:
    // waehrend des Empfangs kann ein einzelner Zugriff ein Mischmuster liefern;
    // zweimal hintereinander derselbe Wert ist gueltig.
    esp_err_t rx_fifo_bytes(uint8_t& rxbytes);
    // PKTSTATUS.SFD: Syncwort empfangen, Paketende noch nicht erreicht. Einzel-
    // bitfeld, vom SPI-Lesefehler laut SWRZ020E S. 5 nicht betroffen.
    bool rx_packet_in_progress();
    // Darf der RX-Task den Chip jetzt anfassen (FIFO lesen, flushen, SRX)? Nur in
    // IDLE, RX, RX_END, RX_RST oder RXFIFO_OVERFLOW (SWRS061I S. 93), zweimal
    // gleich gelesen (SWRZ020E S. 5). Alles andere gehoert einem Uebergang oder
    // einer Sendung: tx() der Klassen laeuft in einem anderen Task ohne Sperre
    // gegen den RX-Task, und SIDLE/SRX beenden eine laufende Sendung (SWRS061I
    // 19.4, S. 52) — bis 0.1.895 gemessen: ein Moritz-Zs mit langer Praeambel
    // ging verloren, wenn der RX-Task waehrenddessen aus einem blockierten
    // Callback zurueckkam (0 von 4 an der Gegenstelle, tx() meldete Erfolg).
    bool rx_state_safe();
    // SRX nur aus IDLE (zweimal gelesen). Gibt zurueck, ob gestrobt wurde.
    bool restart_rx_if_idle();
    // Zaehler fuer ?stats:
    //   rx_extra  — Folgerahmen: in derselben Runde hinter einem anderen
    //               gelesen (Bergungen durch den Heartbeat zaehlen nicht)
    //   rx_stale  — GDO2 stand, der FIFO war leer (CRC_OK ueberlebt Autoflush)
    //   rx_resync — Rahmenkopf unplausibel (Laenge passt nicht) oder CRC_OK
    //               fehlte, FIFO geflusht; Overflow-Flushes zaehlen nicht
    uint32_t rx_extra()  const { return rx_extra_.load(std::memory_order_relaxed); }
    uint32_t rx_stale()  const { return rx_stale_.load(std::memory_order_relaxed); }
    uint32_t rx_resync() const { return rx_resync_.load(std::memory_order_relaxed); }

    // -- Frequenz-Offset-Kalibrierung (FSCTRL0) ------------------------------
    // Jedes Board hat seinen eigenen Quarz-Ziehfehler; auf der CUL32-Serien-
    // hardware sind es ~44 kHz (~50 ppm) gegen ein CUL868/a-culfw, gemessen
    // 2026-08-31 — FSK (AskSin/Moritz) kommt damit in KEINE Richtung an, OOK
    // laeuft weiter (SWRS061I §14.1: keine Offset-Kompensation bei ASK/OOK).
    // SWRS061I §14.1 nennt genau diesen Weg: FREQEST (0x32) auslesen und in
    // FSCTRL0.FREQOFF schreiben ("permanent frequency offset compensation",
    // DN015). LSB = f_XOSC/2^14 = 1,587 kHz bei 26 MHz.
    //
    // Der Wert gehoert zum BOARD, nicht zum Protokoll: er wird hier zentral
    // gehalten und beim Laden JEDER Protokoll-Registertabelle ueber deren
    // FSCTRL0-Wert gelegt (write_reg-Hook). Sonst muesste jede der ~8 CFG-
    // Tabellen den Wert kennen. `write_reg_raw()` umgeht den Hook — das ist
    // der Pfad fuer `Cw0C..`, damit Messungen von Hand weiter moeglich sind.
    void   set_freq_offset(int8_t off) { freq_offset_.store(off, std::memory_order_relaxed); }
    int8_t freq_offset() const { return freq_offset_.load(std::memory_order_relaxed); }
    // Schreibt den aktuellen Offset sofort in FSCTRL0 (Live-Apply ohne Mode-Reset).
    esp_err_t apply_freq_offset();

    // Der Frequenz-Suchlauf (freq_cal) faehrt FSCTRL0 ueber ±64 LSB (±102 kHz,
    // rund eine ganze Kanalbandbreite) und braucht dafuer bis zu 21 s.  Wer in
    // diesem Fenster die RF-Klasse wechselt, bekommt einen Empfaenger, der bis
    // zum Ende des Laufs neben der Frequenz steht: die neue CFG uebernimmt den
    // gerade gesweepten Offset ueber den write_reg-Hook.  Der Slot meldet den
    // Wechsel deshalb hierher, der Suchlauf bricht ab und stellt den
    // Ausgangswert sofort wieder her.  Der Treiber ist die richtige Stelle
    // dafuer: ihn kennen beide Seiten, ohne dass rf_manager von freq_cal
    // abhaengen muss.
    void request_scan_abort()        { scan_abort_.store(true, std::memory_order_relaxed); }
    void clear_scan_abort()          { scan_abort_.store(false, std::memory_order_relaxed); }
    bool scan_abort_requested() const { return scan_abort_.load(std::memory_order_relaxed); }

    // FREQEST-Sampler: die FSK-RX-Pfade rufen das direkt nach einem gueltig
    // empfangenen Frame — nur dann ist der Schaetzwert belastbar (zwischen
    // Frames traeckt der Kompensator Rauschen, SWRS061I §14.1). Ringpuffer,
    // Auswertung per Median gegen Ausreisser.
    // Das gilt fuer Klassen, die nach dem Paket in IDLE gehen (MCSM1 RXOFF =
    // IDLE, AskSin). Bleibt der Chip in RX (MAX!), ist der Wert nach dem
    // FIFO-Lesen nicht mehr der des Pakets: dort liest die Klasse FREQEST
    // waehrend des Pakets (read_freqest) und legt ihn nach dem CRC ab.
    void sample_freqest();            // read_freqest + push_freqest + note_rx_frame
    bool read_freqest(int8_t& out);   // nur lesen (Bus); false, wenn kein Wert zweimal gleich kommt (SWRZ020E)
    void push_freqest(int8_t v);      // nur ablegen (RAM)
    // Dekodierte Frames seit freqest_reset(), auch ohne FREQEST-Wert dazu --
    // das Kriterium des Frequenz-Suchlaufs (freq_cal.cpp).
    void   note_rx_frame();
    size_t rx_frames() const;
    // Median der Samples + Anzahl. false wenn noch nichts gesammelt wurde.
    bool freqest_median(int8_t& out, size_t& count) const;
    void freqest_reset();

    // Status-Byte aus letztem CS-Toggle (Header-Byte aller SPI-Transaktionen).
    // atomic: lock-free vom unsynchronisierten Getter lesbar. Hinweis: der Wert
    // spiegelt die LETZTE Bus-Transaktion — bei geteiltem Bus kann das die einer
    // anderen Task sein. Wer das Status-Byte exakt zu einem Strobe braucht, muss
    // es innerhalb derselben gelockten Op erfassen (heute nur best-effort genutzt).
    uint8_t   last_status() const { return last_status_.load(std::memory_order_relaxed); }

    // GDO-Pin-Getter — RF-Klassen wie SlowRfRF binden RMT/GPIO direkt an die GDOs.
    int pin_gdo0() const { return cfg_.pin_gdo0; }
    int pin_gdo2() const { return cfg_.pin_gdo2; }

    // GDO-Pin-Level-Getter (synchron, kein SPI). Wichtig fuer Edge-IRQ-Recovery:
    // wenn GDOx vor dem Attach schon HIGH war, feuert kein rising-Edge-IRQ und
    // die rx_task verpasst alle nachfolgenden Frames. a-culfw rf_asksin.c:147
    // pollt deshalb GDO2-Level statt edge — wir nutzen das Level als Heartbeat-
    // Recovery (siehe AskSinRF::rx_task_loop und MoritzRF::rx_task_loop).
    int gdo0_level() const;
    int gdo2_level() const;

    using IrqHandler = void (*)(void* arg);
    esp_err_t attach_gdo0_irq(IrqHandler h, void* arg, bool rising);
    esp_err_t attach_gdo2_irq(IrqHandler h, void* arg, bool rising);
    // Beide GDO-IRQ-Handler entfernen.  Wird vom RfManager beim Slot-
    // Reconfigure aufgerufen, damit Handler der vorigen RF-Klasse nicht mehr
    // feuern koennen wenn die naechste Klasse den Pin als reines GPIO nutzt
    // (z.B. wMBus polled GDO2 anstelle IRQ → ohne Detach loest eine Sync-Edge
    // die HmDual/AskSin/Moritz-ISR aus mit `this`-Closure gegen schon
    // stop()ed-Objekt → Crash).
    void      detach_gdo_irqs();

    // IRQ-Gating waehrend TX. AskSinPP-Pattern: vor sndData() disable, nach
    // dem Send wieder enable — sonst feuert IOCFG=0x06 (sync RX/TX) auch beim
    // TX-end und der rx_task wird waehrend laufendem TX aufgeweckt.
    esp_err_t set_gdo0_irq_enabled(bool enable);
    esp_err_t set_gdo2_irq_enabled(bool enable);

private:
    esp_err_t manual_reset();
    // Selbsttest nach Reset: erzwingt CC1101 IOCFG0/IOCFG2 statisch HIGH/LOW
    // (4 Toggles je Pin) und vergleicht ESP-GPIO-Level. Erkennt Fehl-Wiring
    // ESP↔CC1101 sofort statt sich in "RX broken" zu verstecken.
    esp_err_t gdo_pin_check();
    esp_err_t cs_low_wait_ready();      // CSn low + warten bis MISO low (CHIP_RDYn).
    void      cs_high();
    esp_err_t spi_xfer(const uint8_t* tx, uint8_t* rx, size_t len);
    void      clear_stale_rx_signal();

    Config                cfg_{};
    spi_device_handle_t   spi_dev_ = nullptr;
    std::atomic<uint8_t>  last_status_{0};
    std::atomic<uint32_t> pll_lost_{0};       // FSCAL1==0x3F in RX gesehen
    std::atomic<bool>     scan_abort_{false};  // Modewechsel waehrend des Suchlaufs
    std::atomic<uint32_t> pll_relocks_{0};    // Rekalibrierung erst im Retry ok
    std::atomic<uint32_t> pll_failures_{0};   // aufgegeben, kein Lock erreicht
    std::atomic<uint32_t> rx_extra_{0};       // Folgerahmen derselben Runde
    std::atomic<uint32_t> rx_stale_{0};       // GDO2 hoch, FIFO leer
    std::atomic<uint32_t> rx_resync_{0};      // Kopf unplausibel/CRC_OK fehlt, geflusht

    // Board-Frequenzoffset (FSCTRL0-Rohwert, signed). 0 = keine Korrektur.
    std::atomic<int8_t>   freq_offset_{0};
    // FREQEST-Ringpuffer. Klein gehalten: der Wert ist eine Board-Konstante,
    // 32 Frames reichen fuer einen stabilen Median. Geschuetzt vom SPI-Mutex,
    // weil sample_freqest() ohnehin ueber den Bus liest.
    static constexpr size_t kFreqEstCap = 32;
    int8_t                freqest_buf_[kFreqEstCap] = {};
    size_t                freqest_n_   = 0;   // gueltige Eintraege (<= Cap)
    size_t                freqest_pos_ = 0;   // naechste Schreibposition
    size_t                rx_frames_   = 0;   // dekodierte Frames seit freqest_reset()

    // Bus-Mutex: serialisiert jede komplette SPI-Transaktion (cs_low..cs_high)
    // gegen Task-Interleave. Dynamisch im Ctor erzeugt (einmalig, early boot).
    SemaphoreHandle_t     spi_mtx_ = nullptr;
};

} // namespace culfw32
