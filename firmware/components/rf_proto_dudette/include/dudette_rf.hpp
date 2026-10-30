// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "rf_protocol.hpp"
#include "cc1101_driver.hpp"
#include <array>

namespace culfw32 {

// Dudette v2 — Funk-Bootloader fuer AVR-Ziele.
// Spezifikation: notes/dudette_v2_protocol_2026-09-04.md
//
// Diese Stufe (M2) hoert nur zu: sie sammelt die HELLO-Beacons startender
// Geraete zu einer Bestandsliste.  Der Sende- und Uebertragungspfad
// (SELECT/DATA/COMMIT) kommt in M3.

namespace dudette {

constexpr uint8_t kMagicVer   = 0xD1;   // 'D'-Magic + Protokollversion 1
constexpr uint8_t kTypeHello      = 0x01;
constexpr uint8_t kTypeSelect     = 0x02;
constexpr uint8_t kTypeErase      = 0x04;
constexpr uint8_t kTypeData       = 0x05;
constexpr uint8_t kTypeCommit     = 0x06;
constexpr uint8_t kTypeCommitAck  = 0x07;
constexpr uint8_t kTypeVerify     = 0x08;
constexpr uint8_t kTypeVerifyAck  = 0x09;
constexpr uint8_t kTypeGo         = 0x0A;
constexpr uint8_t kTypeStay       = 0x0B;
constexpr uint8_t kTypeStatus     = 0x0C;
// EEPROM pruefen (Bootloader v2.3+, DUDETTE_EEPROM): Rahmen wie VERIFY,
// Antwort VERIFY_ACK, Quelle ist das EEPROM des Geraets.  Kein Lesen —
// die CRC verraet den Inhalt nicht.  Stick-Verb `DV<von>,<len>`.
constexpr uint8_t kTypeEeVerify   = 0x11;
// Textausgabe einer Anwendung (include/dudette_debug.h im Dudette-Baum).
// Eigener Kanal: 869,525 MHz statt 868,4998 (10 % Sendezeit statt 1 %) und
// Syncwort 0xD1DB statt 0x0D3B — Debug-Rahmen koennen so nie in einer
// laufenden Sitzung landen und umgekehrt.
constexpr uint8_t kTypePrint      = 0x0D;
constexpr uint8_t kTypeCmd        = 0x0E;   // Rueckkanal an die Anwendung
constexpr uint8_t kDbgSync1       = 0xD1;
constexpr uint8_t kDbgSync0       = 0xDB;
constexpr uint8_t kDbgFreq2       = 0x21;
constexpr uint8_t kDbgFreq1       = 0x71;
constexpr uint8_t kDbgFreq0       = 0x7A;
// Stick-eigener Statuscode, KEIN Wert des Bootloaders: die Board-Erwartung
// passt nicht zu dem, was das Geraet im HELLO meldet.  Liegt bewusst ausserhalb
// des Bereichs, den dudette.c vergibt (0x00..0x0F), damit er nicht mit einer
// Geraeteantwort verwechselt werden kann.
constexpr uint8_t kStBoardMismatch = 0xB0;
// Ebenfalls stick-eigen: unter dieser Adresse melden sich MEHRERE Bausteine
// (upsert() zaehlt HELLOs mit abweichender Signaturzeile).  Die Adresse ist
// ein auf 24 bit gekuerzter Hash, eindeutig ist sie nur wahrscheinlich.
constexpr uint8_t kStAmbiguous     = 0xB1;

constexpr uint8_t kFragData   = 49;     // 58 PKTLEN - 6 Kopf - 3 (Seite+Fragment)
constexpr uint8_t kMaxFrags   = 8;      // Bitmap ist ein Byte
// Seitennummer mit Bit 15: die Seite geht ins EEPROM des Geraets statt in den
// Flash (Bootloader v2.3+, DUDETTE_EEPROM, Adresse = Seite * Seitengroesse).
// Der Stick behandelt sie wie jede andere Seite — nur der COMMIT wartet laenger.
constexpr uint16_t kPageEeprom = 0x8000;
constexpr uint8_t kHeaderLen  = 6;
constexpr uint8_t kHelloLen   = 31;   // Discovery-Datensatz
constexpr uint8_t kHelloDbgLen = 22;  // optionaler Diagnoseanhang dahinter
constexpr uint8_t kMaxFrame   = 58;     // = PKTLEN

// Mindestabstand, bevor ein unbeantwortetes SELECT auf ein neues HELLO hin
// wiederholt wird.  Der Beacon-Abstand des Bootloaders war 100 + (ADDR & 0x0F)
// ms, also hoechstens 115 — 150 lag sicher darueber, damit kein HELLO, das
// unser SELECT gekreuzt haben koennte, einen Fehlversuch ausloest.  Seit
// Bootloader v2.3 (12.09.2026) sind es 1000 + 4*(ADDR & 0x0F) ms; der
// Mindestabstand bleibt richtig, er ist nur noch weit unter dem Takt.
constexpr uint32_t kArmRetryMs = 150;

// Identitaetsquelle, HELLO-Offset 8
enum class IdentSource : uint8_t {
    SignatureRow = 0,
    EepromMac    = 1,
    None         = 0xFF,
};

} // namespace dudette

// Ein Eintrag der Bestandsliste — was ein Geraet ueber sich meldet, plus was
// der Empfang darueber sagt.
struct DudetteDevice {
    uint32_t addr        = 0;        // 24 bit Kurzadresse
    uint8_t  bl_version  = 0;
    uint8_t  caps        = 0;
    dudette::IdentSource ident = dudette::IdentSource::None;
    char     lot[7]      = {};       // Los-Code aus der Signaturzeile, 6 Zeichen, sonst '.'
    uint8_t  die[4]      = {};       // Wafer- und Die-Kennung
    uint8_t  sig[3]      = {};       // Signatur-Bytes des AVR
    uint16_t page_size   = 0;
    uint32_t app_size    = 0;
    uint16_t board_id    = 0;
    uint8_t  flags       = 0;
    int8_t   rssi_dbm    = 0;
    uint8_t  lqi         = 0;
    uint32_t last_seen_ms = 0;
    uint32_t hello_count  = 0;
    // Diagnoseanhang des Bootloaders (HELLO ab 39 Byte): Sync-Flanken, Rahmen
    // mit gueltiger CRC, davon DATA, zuletzt gesehene Laenge, Laengenfehler,
    // SPI-Timeouts, CHIP_RDYn-Timeouts, MCUSR des letzten Starts.
    bool     has_dbg      = false;
    uint8_t  dbg[22]      = {};
    // Zaehlt HELLOs, die unter dieser Adresse ankamen, aber eine ANDERE
    // Signaturzeile melden (Los oder Die).  Die Adresse ist ein auf 24 bit
    // gekuerzter FNV-1a ueber die Signaturzeile — eindeutig ist sie damit
    // nicht garantiert, nur wahrscheinlich (Geburtstagsproblem: unter 1000
    // Geraeten rund 3 %).  Ohne diesen Zaehler wuerde eine Kollision still
    // zu EINEM Listeneintrag verschmelzen, und ein Transfer ginge an das
    // falsche Geraet, ohne dass es jemand merkt.
    uint8_t  collisions   = 0;
};

// RX-Event-Payload fuer das Frontend.
struct DudetteRxEvent {
    const DudetteDevice* dev;
    bool                 is_new;     // erstmals gehoert
    // Ist `text` gesetzt, ist es KEIN HELLO, sondern eine Debug-Zeile einer
    // Anwendung (kTypePrint); `dev` ist dann nullptr und `addr` traegt den
    // Absender.  Ein Payload-Typ statt zweier, damit der Manager-Pfad und die
    // Frontend-Anmeldung unveraendert bleiben.
    const char*          text = nullptr;
    uint32_t             addr = 0;
};

class DudetteRF final : public RfProtocol {
public:
    static constexpr size_t kMaxDevices = 16;

    ProtocolId           id() const override { return ProtocolId::Dudette; }
    const CC1101Profile& profile() const override;

    esp_err_t start(uint8_t slot_id, CC1101Driver& drv) override;
    esp_err_t stop()  override;
    esp_err_t tx(const void*, size_t) override;
    void      on_rx(RxCallback cb) override { rx_cb_ = std::move(cb); }

    // Gesendet wird ueber die Transfer-API unten, nicht ueber den generischen
    // tx()-Pfad des Managers.
    bool supports_tx(ProtocolId) const override { return false; }

    bool active() const { return active_; }

    // --- Transfer (M3) ----------------------------------------------------
    // Eine Sitzung eroeffnen.  Danach beziehen sich alle Seitenoperationen
    // auf dieses Geraet.
    esp_err_t select(uint32_t addr, uint8_t& status);

    // Scharfstellen: sobald ein HELLO von `addr` eintrifft, schickt die
    // RX-Task das SELECT selbst — innerhalb der rund 90 ms, in denen das
    // Geraet zwischen zwei Beacons sicher lauscht.  Damit entfaellt das
    // blinde Wiederholen aus select(), und der Bediener kann das Geraet
    // resetten, WANN er will (Sicherung, Stecker), statt im Takt des Sticks.
    enum class ArmState : uint8_t { Idle, Armed, Sent, Done };
    void      arm(uint32_t addr);
    void      disarm();
    ArmState  arm_state()  const { return arm_state_; }
    uint32_t  arm_addr()   const { return arm_addr_; }
    uint8_t   arm_status() const { return arm_status_; }

    // App-Bereich ab Seite `from_page` loeschen.  Der Bootloader braucht
    // dafuer bis zu gut 2 s (1284P) und antwortet erst danach.
    esp_err_t erase(uint16_t from_page, uint8_t& status);

    // Seitenpuffer auf dem Stick.  Der Host fuellt ihn fragmentweise; erst
    // `send_page()` geht auf die Luft.
    //
    // Was die CRC dieses Puffers NICHT beweist — das zuerst, weil die alte
    // Fassung nur die Zusage machte und dadurch als groessere Zusage gelesen
    // wurde: sie sagt nichts darueber, ob das Geraet die Seite geschrieben
    // hat, und nichts darueber, ob der Puffer die gewollte Nutzlast trug.
    // Ist er ungefuellt (0xFF), empfaengt das Geraet 0xFF und rechnet
    // dieselbe CRC — ST_OK ist dann korrekt und wertlos.  Beleg fuer das
    // Schreiben ist allein verify() ueber das ganze Abbild.
    //
    // Was sie beweist: den Funkweg.
    void      page_begin(uint16_t page);
    esp_err_t page_fill(uint8_t frag, const uint8_t* data, uint8_t len);
    // `dbg` nimmt, wenn gesetzt, die vier Diagnosebytes auf, die der
    // Bootloader ab Bootloader 2.0 hinter das Ergebnis des COMMIT_ACK haengt:
    // Sync-Flanken, Rahmen mit gueltiger CRC, davon DATA, zuletzt gesehene
    // Laenge.  Damit laesst sich "der Rahmen war nie in der Luft" von "er kam
    // an und wurde verworfen" unterscheiden, ohne den ISP-Clip anzulegen.
    esp_err_t send_page(uint8_t& status, uint8_t& bitmap, uint8_t& rounds,
                        uint8_t* dbg = nullptr, uint16_t* want_crc = nullptr);

    // `ee`: CRC ueber das EEPROM statt ueber den Flash (T_EE_VERIFY).
    esp_err_t verify(uint32_t from, uint32_t len, uint16_t& crc, uint8_t& status,
                     bool ee = false);
    esp_err_t go(uint8_t& status);

    // --- Board-Erwartung --------------------------------------------------
    // Der Board-Typ steht in jedem HELLO (Bytes 28..29), ein Intel-HEX traegt
    // dagegen keine Kennung.  Wer weiss, fuer welche Platine sein Abbild
    // gedacht ist, sagt es hier — und der Stick verweigert dann die Sitzung,
    // wenn das Geraet etwas anderes meldet.  0 = keine Pruefung.
    //
    // Warum im Stick und nicht im Werkzeug: der Stick ist die Stelle, durch
    // die JEDER Klient muss (Python-Werkzeug, Web-UI, JSON-Frontend).  Eine
    // Pruefung im Werkzeug schuetzt nur dessen Benutzer.  Und im Bootloader
    // hat sie nichts zu suchen — dort sind beim 328P 112 B frei, und die
    // Zuordnung Board-ID -> Abbild aendert sich, sobald neue Images dazukommen.
    void      expect_board(uint16_t id) { expect_board_ = id; }
    uint16_t  expect_board() const { return expect_board_; }
    // Was beim letzten abgelehnten SELECT tatsaechlich gemeldet wurde.
    uint16_t  last_board()   const { return last_board_; }
    // Wie viele fremde Signaturzeilen unter der Zieladresse standen.
    uint8_t   last_collisions() const { return last_collisions_; }

    uint32_t  session_addr() const { return session_addr_; }
    uint16_t  page_number()  const { return page_no_; }
    uint16_t  page_size()    const { return page_size_; }

    // Bestandsliste.  `count` ist die Zahl belegter Eintraege.
    const std::array<DudetteDevice, kMaxDevices>& devices() const { return devs_; }
    size_t device_count() const { return dev_count_; }
    void   clear_devices();

    // Debug-Kanal: schaltet Frequenz und Syncwort um.  Der Stick kann dann
    // KEINE Dudette-Sitzung mehr fahren — ein Radio, ein Kanal.  `filter`
    // ungleich 0 zeigt nur dieses eine Geraet, 0 zeigt alle.  Die Trennung
    // laeuft ueber die Adresse im Rahmen, nicht ueber das Syncwort: ein
    // Syncwort je Geraet verhindert keine Kollisionen (gleiche Frequenz,
    // gleiche Zeit) und wuerde nur dazu fuehren, dass ein Stick nur noch ein
    // einziges Geraet sieht.
    void   debug_channel(bool on, uint32_t filter = 0);
    // Ein Kommando an eine Anwendung, die dudette_dbg_poll() aufruft.
    // Nur sinnvoll bei eingeschaltetem Debug-Kanal.
    esp_err_t debug_cmd(uint32_t addr, uint8_t cmd, uint8_t arg);
    bool   debug_on() const { return dbg_on_; }

private:
    static void rx_task_trampoline(void* arg);
    void        rx_task_loop();
    static void IRAM_ATTR gdo_isr(void* arg);

    // Traegt ein HELLO in die Liste ein; liefert den Eintrag und ob er neu ist.
    DudetteDevice* upsert(const uint8_t* frame, uint8_t flen, int8_t rssi_dbm, uint8_t lqi,
                          bool& is_new);

    CC1101Driver* drv_          = nullptr;
    uint8_t       slot_id_      = 0;
    RxCallback    rx_cb_;
    void*         task_handle_  = nullptr;
    volatile bool task_running_ = false;
    volatile bool          active_       = false;

    // Einen Rahmen senden; wartet nicht auf Antwort.
    esp_err_t send_frame(uint8_t type, const uint8_t* body, uint8_t blen);
    // Senden und auf die Antwort des Geraets warten.
    esp_err_t request(uint8_t type, const uint8_t* body, uint8_t blen,
                      uint8_t expect, uint8_t* reply, uint8_t& rlen,
                      uint32_t timeout_ms);

    std::array<DudetteDevice, kMaxDevices> devs_{};
    size_t        dev_count_    = 0;

    void*    lock_        = nullptr;     // SemaphoreHandle_t, serialisiert TX
    void*    reply_sem_   = nullptr;     // SemaphoreHandle_t, Antwort da
    uint8_t  reply_[dudette::kMaxFrame]{};
    volatile uint8_t reply_len_ = 0;
    volatile uint8_t await_type_ = 0;
    uint8_t  seq_         = 0;

    uint16_t          expect_board_    = 0;
    uint16_t          last_board_      = 0;
    uint8_t           last_collisions_ = 0;

    volatile ArmState arm_state_  = ArmState::Idle;
    uint32_t          arm_addr_   = 0;
    uint8_t           arm_seq_    = 0;
    volatile uint8_t  arm_status_ = 0xFF;
    uint32_t          arm_sent_ms_ = 0;  // wann das letzte SELECT rausging
    volatile bool     dbg_on_      = false;
    volatile uint32_t dbg_filter_  = 0;

    uint32_t session_addr_ = 0;
    uint16_t page_no_      = 0xFFFF;
    uint16_t page_size_    = 256;
    uint8_t  page_map_     = 0;
    uint8_t  page_[256]{};
};

} // namespace culfw32
