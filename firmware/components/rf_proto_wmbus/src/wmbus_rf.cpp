// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
//
// wMBus T1/S1 RF-Klasse — Vendor-Port a-culfw culfw/clib/rf_mbus.c (D.Tostmann).

#include "wmbus_rf.hpp"
#include "wmbus_cfg.hpp"
#include "rf_stats.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG = "wmbus";

// Strobes / Reg-Adressen (CC1101 Datenblatt SWRS061I §10.4 + §29).
constexpr uint8_t REG_FIFOTHR = 0x03;
constexpr uint8_t REG_PKTLEN  = 0x06;
constexpr uint8_t REG_PKTCTRL0= 0x08;
constexpr uint8_t REG_SYNC1   = 0x04;
constexpr uint8_t REG_SYNC0   = 0x05;
constexpr uint8_t REG_DEVIATN = 0x15;
constexpr uint8_t REG_MDMCFG4 = 0x10;
constexpr uint8_t REG_MDMCFG3 = 0x11;
constexpr uint8_t REG_MCSM1   = 0x17;
constexpr uint8_t REG_IOCFG2  = 0x00;
constexpr uint8_t REG_IOCFG0  = 0x02;
constexpr uint8_t REG_MARC    = 0x35;

constexpr uint8_t CC_RXFIFO   = 0x3F;
constexpr uint8_t CC_TXFIFO   = 0x3F;

constexpr uint8_t SRES  = 0x30;
constexpr uint8_t SCAL  = 0x33;
constexpr uint8_t SRX   = 0x34;
constexpr uint8_t STX   = 0x35;
constexpr uint8_t SIDLE = 0x36;
constexpr uint8_t SFRX  = 0x3A;
constexpr uint8_t SFTX  = 0x3B;
constexpr uint8_t SNOP  = 0x3D;

constexpr uint8_t MARCSTATE_IDLE = 0x01;
constexpr uint8_t MARCSTATE_RX   = 0x0D;
constexpr uint8_t CC_STATUS_TX_UNDERFLOW = 0x70;  // (status>>4)&0x07 == 0x07 = TX_UNDERFLOW

uint8_t marc_state(CC1101Driver& d) {
    uint8_t v = 0xFF;
    d.read_status(REG_MARC, v);
    return v & 0x1F;
}

inline uint16_t imin(uint16_t a, uint16_t b) { return (a < b) ? a : b; }

// Sperre zwischen Frontend (Moduswechsel, Senden, Stopp) und RX-Task.  Ohne
// sie durfte der Task nicht selbst wieder auf Empfang schalten (Wettlauf mit
// init_radio/enter_rx des Frontends) -- und ohne Re-Arm war der Empfaenger
// nach dem ersten Fehlsync taub.  Ohne Mutex (Anlage gescheitert) laeuft alles
// wie vorher ungesperrt.
class OpLock {
public:
    explicit OpLock(void* m, TickType_t wait = portMAX_DELAY)
        : m_(static_cast<SemaphoreHandle_t>(m)),
          held_(!m_ || xSemaphoreTake(m_, wait) == pdTRUE) {}
    ~OpLock() { if (m_ && held_) xSemaphoreGive(m_); }
    OpLock(const OpLock&) = delete;
    OpLock& operator=(const OpLock&) = delete;
    bool held() const { return held_; }
private:
    SemaphoreHandle_t m_;
    bool held_;
};

// RX-Takt wieder anwerfen, falls set_off() ihn angehalten hat.
void tick_timer_resume(void* t, uint32_t period_us) {
    if (!t) return;
    auto th = static_cast<esp_timer_handle_t>(t);
    if (!esp_timer_is_active(th)) esp_timer_start_periodic(th, period_us);
}

// Cfg-Tabelle (Sentinel-terminiert, addr > 0x40 => Ende) in Chip schreiben.
esp_err_t push_cfg(CC1101Driver& drv, const uint8_t* cfg) {
    for (size_t i = 0; i < 200; i += 2) {
        uint8_t addr = cfg[i];
        if (addr > 0x40) break;
        if (auto e = drv.write_reg(addr, cfg[i + 1]); e != ESP_OK) return e;
    }
    return ESP_OK;
}
} // namespace

const CC1101Profile& WmbusRF::profile() const {
    static const CC1101Profile p = {
        .freq_hz        = 868'949'700,    // T-Mode default
        .datarate_bps   = 100'000,
        .deviation_hz   = 50'000,
        .mod            = CC1101Profile::Modulation::FSK_2,
        .sync_word      = {0x54, 0x3D, 0, 0},
        .sync_len       = 16,
        .pa_table       = {0xC2, 0, 0, 0, 0, 0, 0, 0},
        .raw_regs       = nullptr,
    };
    return p;
}

void WmbusRF::rx_task_trampoline(void* arg) {
    static_cast<WmbusRF*>(arg)->rx_task_loop();
    vTaskDelete(nullptr);
}

esp_err_t WmbusRF::init_radio(uint8_t mbus_mode, RadioMode rmode) {
    if (!enabled_) return ESP_ERR_INVALID_STATE;
    // mbus_mode_=0 stoppt rx_step's guard sofort — neuer Mode wird erst am
    // Ende gesetzt; in der Zwischenzeit ignoriert rx_step alle Ticks.
    mbus_mode_  = 0;
    radio_mode_ = RadioMode::None;

    // Software-Reset bei jedem Mode-Switch (vgl. AskSinRF/MoritzRF).
    drv_->strobe(SRES);
    ets_delay_us(100);

    const uint8_t* cfg = nullptr;
    switch (mbus_mode) {
        case wmbus::MODE_S: cfg = wmbus::WMBUS_S_CFG; break;
        case wmbus::MODE_T: cfg = wmbus::WMBUS_T_CFG; break;
        default: return ESP_OK;  // OFF
    }
    if (auto e = push_cfg(*drv_, cfg); e != ESP_OK) {
        ESP_LOGE(TAG, "push_cfg failed: %s", esp_err_to_name(e));
        return e;
    }

    if (rmode == RadioMode::Tx) {
        // a-culfw rf_mbus.c:172ff: TX-spezifische Overrides.
        drv_->write_reg(REG_MCSM1,   0x00);
        drv_->write_reg(REG_FIFOTHR, wmbus::TX_FIFO_THRESHOLD);
        if (mbus_mode == wmbus::MODE_S) {
            // letztes Sync-Word-Byte muss im TX-FIFO mitgeliefert werden;
            // Sync-Suche bleibt deaktiviert (aber Wert in SYNC1/0 stimmen).
            drv_->write_reg(REG_SYNC1, 0x54);
            drv_->write_reg(REG_SYNC0, 0x76);
        } else {
            drv_->write_reg(REG_SYNC1, 0x54);
            drv_->write_reg(REG_SYNC0, 0x3D);
            drv_->write_reg(REG_DEVIATN, 0x50);
            drv_->write_reg(REG_MDMCFG4, 0x5B);
            drv_->write_reg(REG_MDMCFG3, 0xF8);
        }
        drv_->write_reg(REG_IOCFG0, 0x02);  // GDO0 = TX-FIFO threshold
        drv_->write_reg(REG_IOCFG2, 0x2E);  // GDO2 = high-Z
    }

    mbus_mode_  = mbus_mode;
    radio_mode_ = rmode;
    // SCAL mit Lock-Pruefung (SWRS061I §22.1 p.58: FSCAL1 != 0x3F == Lock);
    // wartet selbst auf MARCSTATE==IDLE, daher kein pauschales vTaskDelay mehr.
    if (drv_->calibrate() != ESP_OK) {
        ESP_LOGE(TAG, "PLL ohne Lock — RX/TX wird ausfallen");
    }
    rxi_ = {};
    return ESP_OK;
}

bool WmbusRF::enter_rx(bool force) {
    if (!enabled_) return false;
    // bereits in RX?
    if (!force && marc_state(*drv_) == MARCSTATE_RX) return false;

    rxi_.state = 0;
    drv_->strobe(SIDLE);
    // Auf IDLE warten — max ~50 ms. Calibrate-Pfad braucht typisch < 1 ms, und
    // die Pruefung steht vor dem Delay, der Normalfall kostet also nichts.
    // vTaskDelay(1) statt pdMS_TO_TICKS(1): bei CONFIG_FREERTOS_HZ=100 ist
    // letzteres ganzzahlig 0, blockiert nicht und liess die Schleife nach
    // ~2 ms statt 50 ms aufgeben (50 SPI-Reads am Stueck) — dazu ohne jede
    // Luft fuer den IDLE-Task. 5 x 1 Tick = 5 x 10 ms trifft das dokumentierte
    // Budget. ets_delay_us-Busy-Wait wuerde idle starven.
    for (uint16_t i = 0; i < 5; i++) {
        if (marc_state(*drv_) == MARCSTATE_IDLE) break;
        vTaskDelay(1);
    }
    drv_->strobe(SFTX);
    drv_->strobe(SFRX);

    rxi_.l_field    = 0;
    rxi_.length     = 0;
    rxi_.bytes_left = 0;
    rxi_.p_byte     = enc_buf_;
    rxi_.format     = 0;          // INFINITE
    rxi_.start      = true;
    rxi_.complete   = false;

    drv_->write_reg(REG_FIFOTHR, wmbus::RX_FIFO_START_THRESHOLD);
    drv_->write_reg(REG_PKTCTRL0, wmbus::PKTCTRL0_INFINITE);

    drv_->strobe(SRX);
    // IDLE -> RX dauert mit Kalibrierung 799 us (SWRS061I Tabelle 34; beide
    // CFGs haben MCSM0.FS_AUTOCAL = 01). Erst kurz pollen: vTaskDelay(1)
    // schlaeft bei 100 Hz bis zur naechsten Tick-Grenze, im Mittel 5 ms,
    // hoechstens 10 ms -- so lange blieb rxi_.state nach jedem Rahmen und
    // jedem Fehlsync auf 0, und ein direkt folgender Rahmen ging verloren
    // (a-culfw rf_mbus_on spinnt hier, Blindzeit ~1 ms). Danach wie bisher
    // 5 Ticks als Sicherheitsnetz, falls der Chip nicht nach RX kommt.
    bool in_rx = false;
    for (uint8_t i = 0; i < 15 && !in_rx; i++) {         // <= ~2 ms
        in_rx = (marc_state(*drv_) == MARCSTATE_RX);
        if (!in_rx) esp_rom_delay_us(100);
    }
    for (uint16_t i = 0; i < 5 && !in_rx; i++) {
        vTaskDelay(1);
        in_rx = (marc_state(*drv_) == MARCSTATE_RX);
    }

    rxi_.state = 1;
    return true;
}

void WmbusRF::rx_step() {
    // Guard: kein drv oder Mode-Switch im Gange.  Bei Mode-Switch setzt
    // init_radio mbus_mode_=0 als allererstes — also pollt rx_step waehrend
    // SRES/Reconfigure auf einem leeren Wert und ueberspringt den Tick.
    if (!enabled_ || radio_mode_ != RadioMode::Rx || mbus_mode_ == 0) return;

    uint8_t bytes_decoded[2] = {};

    switch (rxi_.state) {
        case 0:
            // a-culfw rf_mbus_task case 0: rf_mbus_on(TRUE) -- nach jedem
            // Rahmen und jedem Fehlsync sofort wieder auf Empfang.  Bis 0.1.868
            // fehlte das ("wartet auf das naechste brt"): Rauschen loest einen
            // Fehlsync aus, und der Empfaenger blieb danach taub, bis der CC1101
            // im RXFIFO_OVERFLOW stand (gemessen 08.10.2026, `Vr` MS=11).  Der
            // Wettlauf mit dem Frontend, der das Re-Arm hier verbot, ist durch
            // op_mtx_ ausgeschlossen: rx_step laeuft nur unter der Sperre.
            enter_rx(true);
            return;

        case 1:
            // RX-active, warte auf Sync (GDO2 = HIGH).
            if (!drv_->gdo2_level()) {
                if (++idle_check_ >= kIdleCheckTicks) {
                    // a-culfw ruft am Ende jedes Durchlaufs rf_mbus_on(FALSE):
                    // steht der Chip nicht mehr in RX (Ueberlauf nach einem
                    // zwischen zwei Takten verpassten Sync-Puls), neu aufsetzen.
                    // Hier nur alle kIdleCheckTicks Takte, um den mit dem W5500
                    // geteilten SPI-Bus nicht in jedem Takt mit MARCSTATE-
                    // Abfragen zu belegen.
                    idle_check_ = 0;
                    if (marc_state(*drv_) != MARCSTATE_RX) {
                        enter_rx(true);
                        return;
                    }
                }
                break;
            }
            // Sync gesehen: im SELBEN Takt weiter nach case 2.  Im T-Mode
            // (100 kchip/s) muss PKTLEN stehen, bevor der Rahmen zu Ende ist --
            // bei einem kurzen Rahmen sind das ~2 ms nach dem Sync.  Ein Takt
            // pro Zustandswechsel war dafuer zu langsam (0/20 gemessen).
            rxi_.state = 2;
            idle_check_ = 0;
            [[fallthrough]];

        case 2: {
            // Warte auf erste 3 Bytes im RX-FIFO (GDO0 = HIGH, Schwelle 4 Byte).
            // Im T-Mode kommen sie 240..320 us nach dem Sync; kurz im selben
            // Takt warten statt bis zum naechsten, sonst wird PKTLEN fuer kurze
            // Rahmen zu spaet geschrieben.  Im S-Mode (16 kbit/s netto) dauern
            // 4 Byte ~2 ms -- dort wuerde das Warten nur Rechenzeit kosten.
            if (mbus_mode_ == wmbus::MODE_T) {
                for (uint16_t us = 0; !drv_->gdo0_level() && us < kL3WaitUs; us += 20) {
                    esp_rom_delay_us(20);
                }
            }
            if (drv_->gdo0_level()) {
                if (drv_->read_burst(CC_RXFIFO, {rxi_.p_byte, 3}) != ESP_OK) {
                    rxi_.state = 0;
                    return;
                }
                if (rxi_.format == 0 && mbus_mode_ == wmbus::MODE_S) {
                    if (wmbus::manch_decode(rxi_.p_byte, bytes_decoded) != wmbus::MAN_DECODING_OK) {
                        rxi_.state = 0;
                        return;
                    }
                    rxi_.l_field = bytes_decoded[0];
                    rxi_.length  = wmbus::byte_size(1, 0, wmbus::packet_size(rxi_.l_field));
                } else {
                    if (wmbus::decode_3outof6(rxi_.p_byte, bytes_decoded, 0)
                            != wmbus::DECODING_3OUTOF6_OK) {
                        rxi_.state = 0;
                        return;
                    }
                    rxi_.l_field = bytes_decoded[0];
                    rxi_.length  = wmbus::byte_size(0, 0, wmbus::packet_size(rxi_.l_field));
                }

                if (rxi_.length > kEncBufSize || rxi_.length < 4) {
                    // Defense-in-Depth: length < 4 liesse bytes_left (length - 3)
                    // unterlaufen; kleinste Laenge ist 5 (Modus T, L = 0).
                    rxi_.state = 0;
                    return;
                }

                // Der Tick laeuft mit Prio 1 und wird von jedem Frontend-Task
                // (5/6) verdraengt.  Ohne diese Wiederholung des Gates schreibt
                // ein verdraengter rx_step nach einem Mode-Wechsel PKTLEN und
                // PKTCTRL0 in die frisch geladene Konfiguration der NACHFOLGE-
                // klasse -- die empfaengt dann nichts mehr, und niemand merkt
                // es.  Fruehes Return ist gefahrlos: stop() setzt mbus_mode_
                // und radio_mode_ selbst zurueck, halbe Zustaende ueberleben
                // also nicht.
                if (!enabled_) return;
                if (rxi_.length < wmbus::MAX_FIXED_LENGTH) {
                    drv_->write_reg(REG_PKTLEN, static_cast<uint8_t>(rxi_.length));
                    drv_->write_reg(REG_PKTCTRL0, wmbus::PKTCTRL0_FIXED);
                    rxi_.format = 1;  // FIXED
                } else {
                    uint8_t fixed_len = static_cast<uint8_t>(rxi_.length % wmbus::MAX_FIXED_LENGTH);
                    drv_->write_reg(REG_PKTLEN, fixed_len);
                }

                rxi_.p_byte    += 3;
                rxi_.bytes_left = static_cast<uint16_t>(rxi_.length - 3);
                rxi_.start      = false;
                rxi_.state      = 3;
                drv_->write_reg(REG_FIFOTHR, wmbus::RX_FIFO_THRESHOLD);
            }
            break;
        }

        case 3:
            // Mehr Daten lesen (GDO0 = HIGH ueber Threshold).
            if (drv_->gdo0_level()) {
                if (!enabled_) return;   // siehe oben: Gate nach Verdraengung
                if (rxi_.bytes_left < wmbus::MAX_FIXED_LENGTH && rxi_.format == 0) {
                    drv_->write_reg(REG_PKTCTRL0, wmbus::PKTCTRL0_FIXED);
                    rxi_.format = 1;
                }
                // Datenblatt-Errata: 1 Byte kleiner als FIFO frei lesen,
                // damit nie auf leeren FIFO gelesen wird.
                uint8_t chunk = wmbus::RX_AVAILABLE_FIFO - 1;
                // Am Paketende liegen die zwei Statusbytes (RSSI, LQI) mit im
                // FIFO und zaehlen fuer die Schwelle mit (GDO0 = 0x00, SWRS061I
                // Tabelle 41): bleiben genau 30 Datenbytes, steht der FIFO
                // mit 32 Bytes ueber der Schwelle, obwohl nur 30 zu lesen
                // sind. Dann nur den Rest lesen -- der Abbruch an dieser
                // Stelle verwarf bis 0.1.871 jeden Rahmen mit 31k+33 On-Air-
                // Bytes (T-Mode L-Feld 17, 54, 73, 108, ...; S-Mode 54, 81,
                // 108, ...). a-culfw liest hier blind 31 und laeuft unter.
                if (chunk > rxi_.bytes_left) chunk = static_cast<uint8_t>(rxi_.bytes_left);
                // Defense-in-Depth: bounds-check vor read_burst. Bei Race
                // koennten l_field / bytes_left korrupt sein; ohne diesen
                // Check wuerde p_byte ueber enc_buf_ hinaus zeigen und
                // read_burst Heap-Memory zerstoeren.
                if (rxi_.p_byte < enc_buf_ ||
                    rxi_.p_byte + chunk > enc_buf_ + kEncBufSize) {
                    rxi_.state = 0;
                    return;
                }
                // chunk == 0: alles gelesen, der Chip hat mehr als erwartet --
                // nichts holen, das Paketende (GDO2 low) raeumt unten auf.
                if (chunk > 0 && drv_->read_burst(CC_RXFIFO, {rxi_.p_byte, chunk}) != ESP_OK) {
                    rxi_.state = 0;
                    return;
                }
                rxi_.bytes_left = static_cast<uint16_t>(rxi_.bytes_left - chunk);
                rxi_.p_byte    += chunk;
            }
            break;
    }

    // End-of-Packet: GDO2 LOW + state>1 -> Rest-FIFO leeren + dekodieren.
    if (!drv_->gdo2_level() && rxi_.state > 1) {
        if (rxi_.bytes_left > 0 || rxi_.state == 3) {
            uint8_t rssi = 0, lqi = 0;
            uint8_t left = static_cast<uint8_t>(rxi_.bytes_left);
            if (left > 0) {
                drv_->read_burst(CC_RXFIFO, {rxi_.p_byte, left});
            }
            // Append-Status (PKTCTRL1.bit2=1) liefert RSSI + LQI nach den Daten.
            drv_->read_reg(CC_RXFIFO, rssi);
            drv_->read_reg(CC_RXFIFO, lqi);

            uint16_t status = wmbus::PACKET_CODING_ERROR;
            uint16_t pkt_size = wmbus::packet_size(rxi_.l_field);
            if (mbus_mode_ == wmbus::MODE_S) {
                status = wmbus::decode_rx_bytes_smode(enc_buf_, pkt_buf_, pkt_size);
            } else {
                status = wmbus::decode_rx_bytes_tmode(enc_buf_, pkt_buf_, pkt_size);
            }

            if (status == wmbus::PACKET_OK) {
                int8_t rssi_s = static_cast<int8_t>(rssi);
                int16_t rssi_dbm = (rssi_s < 0) ? (rssi_s / 2 - 74) : (rssi_s / 2 + 64 - 74);
                if (rssi_dbm < -128) rssi_dbm = -128;
                if (rssi_dbm > 127)  rssi_dbm =  127;
                RfStats::instance().on_rx(ProtocolId::Mbus,
                                          static_cast<uint16_t>(pkt_size),
                                          static_cast<int8_t>(rssi_dbm));
                // Ausgeben tut rx_task_loop -- ausserhalb der Sperre.
                out_frame_          = {};
                out_frame_.mode     = mbus_mode_;
                out_frame_.len      = pkt_size;
                out_frame_.rssi_raw = rssi;
                out_frame_.lqi_raw  = lqi;
                std::memcpy(out_frame_.data, pkt_buf_, pkt_size);
                out_rssi_dbm_ = static_cast<int8_t>(rssi_dbm);
                out_ts_us_    = static_cast<uint32_t>(esp_timer_get_time());
                frame_ready_  = true;
            } else {
                RfStats::instance().on_crc_error(ProtocolId::Mbus);
            }
        }
        // state 0: der naechste Takt setzt den Empfang wieder auf (s. case 0).
        rxi_.state = 0;
        return;
    }
}

void WmbusRF::tick_timer_cb(void* arg) {
    auto* self = static_cast<WmbusRF*>(arg);
    if (auto h = static_cast<TaskHandle_t>(self->task_handle_)) {
        xTaskNotifyGive(h);
    }
}

void WmbusRF::rx_task_loop() {
    // Eternal-Task — analog Moritz/AskSin. Takt kTickUs = 500 us (a-culfw
    // rf_mbus_task wird in der main-Loop ohne Delay gepollt).  Die 2 ms bis
    // 0.1.869 reichten fuer den FIFO (32-Byte-Schwelle bei 100 kchip/s), aber
    // nicht fuer PKTLEN: das muss im T-Mode stehen, bevor ein kurzer Rahmen
    // endet (~2 ms nach dem Sync) -- gemessen 0/20, mit 500 us siehe Commit.
    //
    // Die Kadenz kommt vom esp_timer per Task-Notify, NICHT von vTaskDelay:
    // bei CONFIG_FREERTOS_HZ=100 ist ein Tick 10 ms, und pdMS_TO_TICKS(2)
    // rechnet ganzzahlig zu **0**. vTaskDelay(0) blockiert nicht — es ist nur
    // ein Yield an Tasks gleicher oder hoeherer Prioritaet. Der IDLE-Task
    // (Prio 0) kam damit nie dran, und der Task-Watchdog schlug alle 5 s zu
    // (gemessen 01.09.2026; der Zustand ueberlebte auch das Verlassen des
    // wMBus-Modus, weil der `continue`-Pfad genauso spinnt).
    // `vTaskDelay(1)` waere die falsche Antwort: 10 ms Poll-Abstand laesst bei
    // T-Mode 100 kbaud die 64-Byte-FIFO ueberlaufen (Threshold 32 B = 320 µs).
    // Der Timeout unten ist nur ein Sicherheitsnetz fuer den Fall, dass der
    // Timer nicht laeuft (wMBus nicht aktiv) — dann blockiert der Task sauber
    // statt zu pollen.
    while (true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        if (!enabled_ || radio_mode_ != RadioMode::Rx || mbus_mode_ == 0) continue;
        {
            // Frontend (Moduswechsel, Senden, Stopp) geht vor: ist die Sperre
            // belegt, faellt dieser Takt aus, der naechste kommt kTickUs spaeter.
            OpLock lk(op_mtx_, 0);
            if (!lk.held()) continue;
            rx_step();
        }
        // Ausgabe ausserhalb der Sperre: rx_cb_ rendert bis in TCP-Transporte,
        // die blockieren koennen.  out_frame_ gehoert nur diesem Task.
        if (frame_ready_) {
            frame_ready_ = false;
            if (rx_cb_) {
                RxEvent ev{
                    .proto        = ProtocolId::Mbus,
                    .slot_id      = slot_id_,
                    .rssi_dbm     = out_rssi_dbm_,
                    .timestamp_us = out_ts_us_,
                    .payload      = &out_frame_,
                };
                rx_cb_(ev);
            }
        }
    }
}

esp_err_t WmbusRF::start(uint8_t slot_id, CC1101Driver& drv) {
    drv_     = &drv;
    slot_id_ = slot_id;
    if (!op_mtx_) op_mtx_ = xSemaphoreCreateMutex();

    if (!task_handle_) {
        TaskHandle_t h = nullptr;
        // Priority 8 wie die anderen RF-RX-Tasks (AskSin/Moritz/native).  Bis
        // 0.1.870 lief er auf 1, weil er frueher mit vTaskDelay(0) spann und den
        // IDLE-Task aushungerte; seit dem esp_timer-Notify blockiert er zwischen
        // den Takten.  Auf 1 verdraengten ihn Netz- und Frontend-Tasks so weit,
        // dass PKTLEN im T-Mode zu spaet kam.  Es ist ohnehin nur eine RF-Klasse
        // je Slot aktiv.  Stack 6144 (Reserve fuer Mutex-Code + ESP_LOGE-Pfade).
        if (xTaskCreate(&WmbusRF::rx_task_trampoline, "wmbus-rx", 6144, this, 8, &h) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
        task_handle_ = h;
    }

    // kTickUs-Takt fuer den rx-Task. Laeuft nur, solange wMBus die aktive Klasse
    // ist — sonst blockiert der Task im Notify-Timeout und kostet nichts.
    if (!tick_timer_) {
        esp_timer_create_args_t targs = {};
        targs.callback        = &WmbusRF::tick_timer_cb;
        targs.arg             = this;
        targs.dispatch_method = ESP_TIMER_TASK;
        targs.name            = "wmbus-tick";
        // Ist der esp_timer-Task einmal laenger als einen Takt weg (W5500-
        // Link-Poll laeuft in demselben Task), nicht alle versaeumten Takte
        // nachholen: ein Notify je Takt reicht, der Zustand ist im Chip.
        targs.skip_unhandled_events = true;
        esp_timer_handle_t th = nullptr;
        if (auto e = esp_timer_create(&targs, &th); e != ESP_OK) {
            ESP_LOGE(TAG, "tick timer create: %s", esp_err_to_name(e));
            return e;
        }
        tick_timer_ = th;
    }
    auto th = static_cast<esp_timer_handle_t>(tick_timer_);
    esp_timer_stop(th);   // idempotent machen: start_periodic auf laufendem Timer waere INVALID_STATE
    if (auto e = esp_timer_start_periodic(th, kTickUs); e != ESP_OK) {
        ESP_LOGE(TAG, "tick timer start: %s", esp_err_to_name(e));
        return e;
    }

    ESP_LOGI(TAG, "started on slot %u", slot_id);

    enabled_ = true;   // ab hier darf der RX-Task ans Radio
    return ESP_OK;
}

esp_err_t WmbusRF::stop() {
    OpLock lk(op_mtx_);   // ein laufender rx_step endet vorher
    // Takt zuerst anhalten: der rx-Task soll nicht noch auf einen Chip zugreifen,
    // dessen drv_ gleich nullptr wird. Der Task selbst bleibt (Eternal-Task) und
    // blockiert danach im Notify-Timeout.
    if (tick_timer_) {
        esp_timer_stop(static_cast<esp_timer_handle_t>(tick_timer_));
    }
    if (drv_) {
        drv_->strobe(SIDLE);
    }
    // Der Zeiger wird NICHT mehr genullt: der CC1101Driver ist ein WERT-Member
    // des Slots (cc1101_slot.hpp) und lebt, solange der Manager lebt -- er wird
    // also nie ungueltig.  Ihn zu loeschen brachte keine Sicherheit, sondern
    // verwandelte "Task fasst ein stillgelegtes Radio an" in einen Null-Zugriff:
    // der RX-Task laeuft hoeher priorisiert (8) als die Frontend-Tasks (5/6) und
    // kann in drain_one_frame() blockieren (SPI-Mutex; Renderer ->
    // TcpClientTransport::write, seit 0.1.902 ohne Warten am Socket; bis 0.1.893 unbegrenzt).
    // Kehrte er zurueck, waehrend das Frontend zwischen stop() und start() stand,
    // lief ein strobe() auf einen geloeschten Zeiger.  Gegated wird ueber das Flag.
    mbus_mode_  = 0;
    radio_mode_ = RadioMode::None;
    enabled_    = false;
    return ESP_OK;
}

void WmbusRF::set_mode_smode() {
    OpLock lk(op_mtx_);
    if (init_radio(wmbus::MODE_S, RadioMode::Rx) == ESP_OK) {
        enter_rx(true);
        tick_timer_resume(tick_timer_, kTickUs);
    }
}

void WmbusRF::set_mode_tmode() {
    OpLock lk(op_mtx_);
    if (init_radio(wmbus::MODE_T, RadioMode::Rx) == ESP_OK) {
        enter_rx(true);
        tick_timer_resume(tick_timer_, kTickUs);
    }
}

void WmbusRF::set_off() {
    OpLock lk(op_mtx_);
    if (drv_) drv_->strobe(SIDLE);
    mbus_mode_  = 0;
    radio_mode_ = RadioMode::None;
    // Ohne Empfang keinen Takt: sonst wecken sich esp_timer-Task und RX-Task
    // 2000-mal je Sekunde nur, um am Guard abzuprallen -- bis zur naechsten
    // Klasse, also unter Umstaenden fuer immer. brs/brt starten ihn wieder.
    if (tick_timer_) esp_timer_stop(static_cast<esp_timer_handle_t>(tick_timer_));
}

esp_err_t WmbusRF::tx_send_packet(const uint8_t* pkt, uint16_t pkt_size, uint8_t mbus_mode) {
    if (!enabled_) return ESP_ERR_INVALID_STATE;
    if (pkt_size == 0 || pkt_size > 290) return ESP_ERR_INVALID_ARG;

    uint8_t  prev_mode = (radio_mode_ == RadioMode::Rx) ? mbus_mode_ : 0;
    uint16_t bytes_left = 0;

    if (auto e = init_radio(mbus_mode, RadioMode::Tx); e != ESP_OK) return e;

    // On-air-Encode in enc_buf_.
    if (mbus_mode == wmbus::MODE_S) {
        wmbus::encode_tx_bytes_smode(enc_buf_, pkt, pkt_size);
        bytes_left = wmbus::byte_size(1, 1, pkt_size);
    } else {
        wmbus::encode_tx_bytes_tmode(enc_buf_, pkt, pkt_size);
        bytes_left = wmbus::byte_size(0, 1, pkt_size);
    }

    // TX-State-Check.  Status unter demselben Bus-Lock wie der Strobe holen:
    // last_status() spiegelt die LETZTE Transaktion, und die kann von einem
    // anderen Task stammen (C/Vr/?stats lesen ohne Slot-Sperre).
    uint8_t st = 0;
    drv_->strobe(SNOP, st);
    if (((st >> 4) & 0x07) != 0x00 /*IDLE*/) {
        drv_->strobe(SIDLE);
        if (prev_mode) init_radio(prev_mode, RadioMode::Rx);
        if (prev_mode) enter_rx(true);
        return ESP_ERR_INVALID_STATE;
    }
    drv_->strobe(SFTX);

    const uint8_t* p = enc_buf_;
    bool fixed_format = false;

    if (bytes_left < wmbus::MAX_FIXED_LENGTH) {
        drv_->write_reg(REG_PKTLEN, static_cast<uint8_t>(bytes_left));
        drv_->write_reg(REG_PKTCTRL0, wmbus::PKTCTRL0_FIXED);
        fixed_format = true;
    } else {
        drv_->write_reg(REG_PKTLEN, static_cast<uint8_t>(bytes_left % wmbus::MAX_FIXED_LENGTH));
        drv_->write_reg(REG_PKTCTRL0, wmbus::PKTCTRL0_INFINITE);
    }

    // Zeitdeckel fuer den ganzen Sendevorgang nach der On-Air-Laenge: T-Mode
    // 100 kchip/s = 80 us je Byte, S-Mode 32,768 kchip/s Manchester = 488 us
    // je Byte, je mit Reserve.  Die festen 100 ms bis 0.1.872 reichten fuer
    // lange S-Mode-Rahmen nicht (560 Byte = 273 ms).
    const int64_t tx_budget_us = 20000 + static_cast<int64_t>(bytes_left) *
                                 (mbus_mode == wmbus::MODE_S ? 600 : 120);

    // Initial-Fill TX-FIFO + STX.
    uint16_t to_write = imin(wmbus::TX_FIFO_SIZE, bytes_left);
    if (drv_->write_burst(CC_TXFIFO, {p, to_write}) != ESP_OK) {
        if (prev_mode) { init_radio(prev_mode, RadioMode::Rx); enter_rx(true); }
        return ESP_ERR_INVALID_STATE;
    }
    p          += to_write;
    bytes_left = static_cast<uint16_t>(bytes_left - to_write);

    drv_->strobe(STX);

    // Refill-Loop. GDO0 = IOCFG0 0x02: HIGH, solange der TX-FIFO bei oder ueber
    // der Schwelle steht (SWRS061I Tabelle 41, S. 62), mit FIFO_THR 7 also ab
    // 33 Byte (S. 72).  Nachgefuellt wird bei LOW: dann sind hoechstens 32 Byte
    // drin und 32 frei.  Bis 0.1.872 stand die Bedingung umgekehrt (aus a-culfw,
    // das auf dem CUL ohne wMBus-Senden gebaut wird): gleich nach dem ersten
    // Fuellen mit 64 Byte kamen 32 weitere hinterher, der FIFO lief ueber, und
    // jeder Rahmen mit mehr als 64 On-Air-Bytes ging kaputt raus (gemessen
    // 08.10.2026: L-Feld 36 und 48 0/10, bis L-Feld 17 alles da).
    auto t_start = esp_timer_get_time();
    while (bytes_left > 0) {
        if (!drv_->gdo0_level()) {
            // Erst pruefen, ob der FIFO schon leergelaufen ist -- dann ist der
            // Rahmen ohnehin kaputt, und Nachschieben aendert daran nichts.
            // Status atomar zum Strobe (s. TX-State-Check oben).
            uint8_t st2 = 0;
            drv_->strobe(SNOP, st2);
            if (((st2 >> 4) & 0x07) == 0x07 /*TX_UNDERFLOW*/) {
                drv_->strobe(SFTX);
                if (prev_mode) { init_radio(prev_mode, RadioMode::Rx); enter_rx(true); }
                return ESP_ERR_INVALID_STATE;
            }
            uint16_t chunk = imin(wmbus::TX_AVAILABLE_FIFO, bytes_left);
            if (drv_->write_burst(CC_TXFIFO, {p, chunk}) != ESP_OK) {
                if (prev_mode) { init_radio(prev_mode, RadioMode::Rx); enter_rx(true); }
                return ESP_ERR_INVALID_STATE;
            }
            p          += chunk;
            bytes_left = static_cast<uint16_t>(bytes_left - chunk);
            if (bytes_left < (wmbus::MAX_FIXED_LENGTH - wmbus::TX_FIFO_SIZE) && !fixed_format) {
                drv_->write_reg(REG_PKTCTRL0, wmbus::PKTCTRL0_FIXED);
                fixed_format = true;
            }
        } else {
            // FIFO noch ueber der Schwelle: der Chip sendet, warten.
            if (esp_timer_get_time() - t_start > tx_budget_us) {
                drv_->strobe(SIDLE);
                if (prev_mode) { init_radio(prev_mode, RadioMode::Rx); enter_rx(true); }
                return ESP_ERR_TIMEOUT;
            }
            ets_delay_us(50);
        }
    }
    // TX-Ende abwarten: der letzte FIFO-Inhalt (<= 64 Byte) braucht im S-Mode
    // bis ~31 ms, im T-Mode ~5 ms.  Max ~50 ms.
    for (uint16_t i = 0; i < 1000; i++) {
        if (marc_state(*drv_) == MARCSTATE_IDLE) break;
        ets_delay_us(50);
    }

    RfStats::instance().on_tx(ProtocolId::Mbus, pkt_size, /*credits*/0);

    if (prev_mode) {
        init_radio(prev_mode, RadioMode::Rx);
        enter_rx(true);
    }
    return ESP_OK;
}

esp_err_t WmbusRF::tx(const void* typed_payload, size_t payload_size) {
    if (!enabled_ || !typed_payload || payload_size < sizeof(WmbusTxFrame)) {
        return ESP_ERR_INVALID_ARG;
    }
    auto* f = static_cast<const WmbusTxFrame*>(typed_payload);
    if (f->len == 0 || f->len > WmbusFrame::kMaxPacket) return ESP_ERR_INVALID_ARG;
    if (f->mode != wmbus::MODE_S && f->mode != wmbus::MODE_T) return ESP_ERR_INVALID_ARG;
    OpLock lk(op_mtx_);   // TX samt Rueckkehr nach RX am Stueck
    return tx_send_packet(f->packet, f->len, f->mode);
}

} // namespace culfw32
