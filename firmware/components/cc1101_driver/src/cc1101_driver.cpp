// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "cc1101_driver.hpp"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include <algorithm>
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG = "cc1101";
// IRQ-Service ist Prozess-weit, nicht pro Driver — installiere einmal.
// File-scope (frueher unten) damit gdo_pin_check ihn auch nutzen kann.
bool g_isr_service_installed = false;

// RAII-Guard fuer den Bus-Mutex. Haelt den Lock ueber die GESAMTE
// cs_low_wait_ready()..spi_xfer()..cs_high()-Sequenz einer public-Methode,
// damit kein zweiter Task dazwischen CS togglen oder einen polling-Transfer
// starten kann. Release auf jedem Return-Pfad (RAII). Max Hold-Zeit ist
// gebounded (CHIP_RDYn-Wait <= 10 ms, polling-xfer <= einige us) -> kein
// Deadlock bei portMAX_DELAY. Null-Mutex (Ctor-Allokation fehlgeschlagen) ->
// no-op-Degradation statt Crash.
//
// Zusaetzlich wird der SPI-Bus selbst belegt (spi_device_acquire_bus): am
// selben SCLK/MOSI haengt ueber J1 der W5500 (eth_drv). Ohne die Belegung
// koennte dessen Transfer zwischen zwei CC1101-Transfers bei gesenktem CSn
// laufen — der CC1101 laese den W5500-Frame als eigenen Header. Der Mutex
// bleibt fuer die Tasks, die DIESES Device teilen; der Bus-Lock regelt das
// Gegenueber. Reihenfolge immer Mutex -> Bus, eth_drv haelt es genauso.
struct SpiLock {
    SemaphoreHandle_t   m_;
    spi_device_handle_t dev_;
    SpiLock(SemaphoreHandle_t m, spi_device_handle_t dev) : m_(m), dev_(dev) {
        if (m_) xSemaphoreTake(m_, portMAX_DELAY);
        if (dev_ && spi_device_acquire_bus(dev_, portMAX_DELAY) != ESP_OK) dev_ = nullptr;
    }
    ~SpiLock() {
        if (dev_) spi_device_release_bus(dev_);
        if (m_) xSemaphoreGive(m_);
    }
    SpiLock(const SpiLock&) = delete;
    SpiLock& operator=(const SpiLock&) = delete;
};
}

esp_err_t CC1101Driver::init(const Config& cfg) {
    cfg_ = cfg;

    // 1) GPIO-Setup: CS als Output (start high), GDOs als Input.
    gpio_config_t cs_io = {
        .pin_bit_mask = 1ULL << cfg.pin_cs,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (auto e = gpio_config(&cs_io); e != ESP_OK) return e;
    gpio_set_level(static_cast<gpio_num_t>(cfg.pin_cs), 1);

    gpio_config_t gdo_io = {
        .pin_bit_mask = (1ULL << cfg.pin_gdo0) | (1ULL << cfg.pin_gdo2),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (auto e = gpio_config(&gdo_io); e != ESP_OK) return e;

    // MISO als Input fuer den manuellen CHIP_RDYn-Wait waehrend Reset.
    // Spaeter uebernimmt der SPI-Treiber das Pin-Routing.
    gpio_config_t miso_io = {
        .pin_bit_mask = 1ULL << cfg.pin_miso,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&miso_io);

    // 2) SPI-Bus initialisieren.
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = cfg.pin_mosi;
    buscfg.miso_io_num = cfg.pin_miso;
    buscfg.sclk_io_num = cfg.pin_sck;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 64;

    // ESP32-C6 hat nur SPI2_HOST (=1). Direktes Cast haelt das Skelett portabel
    // fuer kuenftige Multi-SPI-Targets (S3 etc.) ohne enum-Konstanten-Liste.
    auto host = static_cast<spi_host_device_t>(cfg.spi_host);
    if (auto e = spi_bus_initialize(host, &buscfg, SPI_DMA_DISABLED); e != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(e));
        return e;
    }

    // 3) SPI-Device. CS manuell, weil CC1101 nach CSn-low ein MISO-Low (CHIP_RDYn)
    //    erwartet bevor das erste Byte gesendet wird. Das kann der ESP-IDF-Treiber
    //    mit Hardware-CS nicht.
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = cfg.spi_clock_hz;
    devcfg.mode           = 0;            // CPOL=0, CPHA=0
    devcfg.spics_io_num   = -1;           // CS manuell
    devcfg.queue_size     = 1;
    devcfg.flags          = 0;

    if (auto e = spi_bus_add_device(host, &devcfg, &spi_dev_); e != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(e));
        return e;
    }

    // 4) Manual Reset nach SWRS061I §19.1.2.
    if (auto e = manual_reset(); e != ESP_OK) return e;

    // 5) GDO-Pin-Verdrahtungs-Selbsttest (Build 0.1.117). Laeuft einmalig beim
    //    Boot VOR jedem RX-Task — eliminiert HW-Wiring-Hypothesen aus jeder
    //    "RX broken"-Bug-Suche (Boot-Log "GDOx wiring + IRQ OK"). Deadlock-frei:
    //    init() haelt keinen Bus-Lock, manual_reset()s SpiLock ist hier laengst
    //    freigegeben, gdo_pin_check komponiert nur die je selbst gelockten
    //    read_reg()/write_reg(). Fehler ist nicht-fatal (reine Diagnose).
    if (auto e = gdo_pin_check(); e != ESP_OK) {
        ESP_LOGW(TAG, "gdo_pin_check failed: %s", esp_err_to_name(e));
    }
    return ESP_OK;
}

// Counter fuer den IRQ-Selbsttest. Wird vom temporaeren ISR inkrementiert.
static volatile uint32_t s_gdo_test_isr_count = 0;
static void IRAM_ATTR gdo_test_isr(void*) { s_gdo_test_isr_count++; }

esp_err_t CC1101Driver::gdo_pin_check() {
    // Bewusst KEIN SpiLock hier: diese Funktion komponiert read_reg()/write_reg()
    // (die selbst je den Bus-Mutex nehmen) — ein eigener Lock waere ein
    // Self-Deadlock (nicht-rekursiver Mutex). Die IOCFG-save/restore-Sequenz ist
    // damit nicht als Ganzes atomar; das ist ok, weil dies ein Boot-Self-Test ist
    // (Aufruf einmalig aus init() nach manual_reset, vor jedem RX-Task).
    // CC1101 IOCFG-Werte (SWRS061I Table 41 "Output Pin Configuration"):
    //   0x2F = HW to 0       → GDOx-pin = LOW
    //   0x6F = INV | 0x2F    → GDOx-pin = HIGH (bit 6 = GDOx_INV)
    // Auf jedem GDOx-Pin testen wir 4 Toggles um Stuck-Pin und Kurzschluesse
    // zu erkennen. Originalwerte am Ende restored.
    constexpr uint8_t IOCFG_FORCE_LOW  = 0x2F;
    constexpr uint8_t IOCFG_FORCE_HIGH = 0x6F;

    struct Test {
        const char* name;
        int         esp_pin;
        uint8_t     iocfg_addr;
    };
    const Test tests[] = {
        {"GDO0", cfg_.pin_gdo0, 0x02},     // IOCFG0
        {"GDO2", cfg_.pin_gdo2, 0x00},     // IOCFG2
    };

    bool any_fail = false;
    for (const auto& t : tests) {
        if (t.esp_pin < 0) continue;            // pin disabled in cfg
        // Originalwert sichern fuer Restore.
        uint8_t orig = 0;
        read_reg(t.iocfg_addr, orig);

        bool pin_ok = true;
        // 4 Toggles: low, high, low, high. Dazwischen 50 us settling.
        const uint8_t pattern[] = {IOCFG_FORCE_LOW, IOCFG_FORCE_HIGH,
                                   IOCFG_FORCE_LOW, IOCFG_FORCE_HIGH};
        for (size_t i = 0; i < sizeof(pattern); ++i) {
            uint8_t want = pattern[i];
            int     want_level = (want == IOCFG_FORCE_HIGH) ? 1 : 0;
            if (auto e = write_reg(t.iocfg_addr, want); e != ESP_OK) {
                ESP_LOGE(TAG, "%s: IOCFG write failed: %s", t.name, esp_err_to_name(e));
                pin_ok = false; break;
            }
            ets_delay_us(50);
            int got = gpio_get_level(static_cast<gpio_num_t>(t.esp_pin));
            if (got != want_level) {
                ESP_LOGE(TAG, "%s wiring error: CC1101 IOCFG=0x%02x (expect %s) "
                              "but ESP-GPIO %d reads %s — check Kconfig + Loeterei",
                         t.name, want, want_level ? "HIGH" : "LOW",
                         t.esp_pin, got ? "HIGH" : "LOW");
                pin_ok = false; break;
            }
        }

        // ── Stage 2: Edge-IRQ-Verify (DEAKTIVIERT) ──────────────────────────
        // 2026-04-30 (Build 0.1.130 Diagnose mit DSLogic + Reg-Dump): Stage 2
        // hinterliess GPIO-Pad-State (gpio_reset_pin + repeated isr_handler_add/
        // remove) so dass die spaeteren AskSin/MoritzRF::attach_gdo2_irq IRQs
        // zwar attached wurden, aber auf reale CRCOK-Edges nicht mehr feuerten.
        // PKT_CRCOK assertete am Chip, RXBYTES wuchs, isr_count blieb 0.
        // Stage 1 reicht voellig aus (statische Pin-Wiring-Verifikation per
        // gpio_get_level). Edge-Test an ESP32-C6-GPIO-Pad braucht es nicht
        // — wenn Stage 1 OK + RF-Klasse nichts sieht, ist es Software-Bug,
        // nicht Pad-Problem.

        // Originalwert restoren.
        write_reg(t.iocfg_addr, orig);

        if (pin_ok) {
            ESP_LOGI(TAG, "%s wiring OK (CC1101 ↔ ESP-GPIO %d)",
                     t.name, t.esp_pin);
        } else {
            any_fail = true;
        }
    }

    if (any_fail) {
        ESP_LOGE(TAG, "GDO pin check FAILED — RX/IRQ-Pfade werden nicht "
                      "zuverlaessig funktionieren. Hardware pruefen.");
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

esp_err_t CC1101Driver::manual_reset() {
    SpiLock lk(spi_mtx_, spi_dev_);
    auto cs   = static_cast<gpio_num_t>(cfg_.pin_cs);
    auto miso = static_cast<gpio_num_t>(cfg_.pin_miso);

    // Sequenz aus Datasheet Figure 27:
    //   CSn high (idle), pulse low/high/low, CSn low, wait MISO low, SRES, wait MISO low.
    gpio_set_level(cs, 1);
    ets_delay_us(40);
    gpio_set_level(cs, 0);
    ets_delay_us(40);
    gpio_set_level(cs, 1);
    ets_delay_us(45);

    gpio_set_level(cs, 0);
    // CHIP_RDYn: MISO geht low wenn der Chip bereit ist. Timeout 10 ms.
    int wait_us = 0;
    while (gpio_get_level(miso) == 1) {
        ets_delay_us(10);
        if ((wait_us += 10) > 10000) {
            gpio_set_level(cs, 1);
            ESP_LOGE(TAG, "reset: MISO never went low (CHIP_RDYn timeout)");
            return ESP_ERR_TIMEOUT;
        }
    }

    // SRES strobe.
    uint8_t tx = SRES, rx = 0;
    if (auto e = spi_xfer(&tx, &rx, 1); e != ESP_OK) {
        gpio_set_level(cs, 1);
        return e;
    }
    last_status_ = rx;

    // Wait for CHIP_RDYn again.
    wait_us = 0;
    while (gpio_get_level(miso) == 1) {
        ets_delay_us(10);
        if ((wait_us += 10) > 10000) {
            gpio_set_level(cs, 1);
            ESP_LOGE(TAG, "reset: MISO never went low after SRES");
            return ESP_ERR_TIMEOUT;
        }
    }

    gpio_set_level(cs, 1);
    ESP_LOGI(TAG, "reset complete (status=0x%02x)",
             static_cast<unsigned>(last_status_.load(std::memory_order_relaxed)));
    return ESP_OK;
}

esp_err_t CC1101Driver::cs_low_wait_ready() {
    auto cs   = static_cast<gpio_num_t>(cfg_.pin_cs);
    auto miso = static_cast<gpio_num_t>(cfg_.pin_miso);
    gpio_set_level(cs, 0);
    int wait_us = 0;
    while (gpio_get_level(miso) == 1) {
        ets_delay_us(2);
        if ((wait_us += 2) > 1000) {
            gpio_set_level(cs, 1);
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

void CC1101Driver::cs_high() {
    gpio_set_level(static_cast<gpio_num_t>(cfg_.pin_cs), 1);
}

esp_err_t CC1101Driver::spi_xfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    // Ohne DMA nimmt der Host hoechstens SOC_SPI_MAXIMUM_BUFFER_SIZE = 64 Byte je
    // Transaktion (max_transfer_sz in init()).  CS ist eine eigene GPIO-Leitung
    // und bleibt ueber die Teilstuecke low -- fuer den CC1101 ein durchgehender
    // Burst.  Ein voller TX-FIFO (64 Datenbytes + Header = 65 Byte) wurde bis
    // 0.1.873 mit "txdata transfer > host maximum" abgewiesen: kein wMBus-Rahmen
    // mit 64 oder mehr On-Air-Bytes ging je raus (gemessen 08.10.2026).
    constexpr size_t kMaxXfer = 64;
    while (len > 0) {
        const size_t n = (len < kMaxXfer) ? len : kMaxXfer;
        spi_transaction_t t = {};
        t.length    = n * 8;
        t.tx_buffer = tx;
        t.rx_buffer = rx;
        if (auto e = spi_device_polling_transmit(spi_dev_, &t); e != ESP_OK) return e;
        len -= n;
        if (tx) tx += n;
        if (rx) rx += n;
    }
    return ESP_OK;
}

esp_err_t CC1101Driver::strobe(uint8_t cmd, uint8_t& status_out) {
    esp_err_t e;
    {
        SpiLock lk(spi_mtx_, spi_dev_);
        if (auto ee = cs_low_wait_ready(); ee != ESP_OK) return ee;
        uint8_t rx = 0;
        e = spi_xfer(&cmd, &rx, 1);
        last_status_ = rx;
        status_out   = rx;   // selbes Byte wie last_status_, unter demselben Lock
        cs_high();
    }
    // SRES setzt FSCTRL0 auf den Chip-Default 0x00 zurueck, und die CFG-Tabellen
    // von AskSin, Moritz, native und HmDual (das laedt ASKSIN_CFG) schreiben das
    // Register nicht — a-culfw kommt ohne aus, weil es dort keinen Boardoffset
    // gibt. Der write_reg()-Hook kann nur greifen, wo die Tabelle FSCTRL0
    // ueberhaupt anfasst; ohne dieses Nachziehen verpufft der kalibrierte
    // Ziehfehler in genau den FSK-Protokollen, fuer die er gebraucht wird.
    // Gemessen 01.09.2026 auf CUL32-Serienhardware: Kaltstart mit -40 LSB in
    // NVS -> FSCTRL0 blieb nach Ar/Zr/Nr1/Pr auf 0x00, nur X/b/f trugen den Wert.
    // SRES laeuft an neun Stellen (RfManager + je Protokollklasse), deshalb sitzt
    // das Nachziehen hier und nicht in reset(). Der Lock ist nicht rekursiv —
    // write_reg_raw() darf erst nach dem Scope oben laufen.
    if (cmd == SRES && e == ESP_OK) {
        int8_t off = freq_offset_.load(std::memory_order_relaxed);
        if (off != 0) write_reg_raw(REG_FSCTRL0, static_cast<uint8_t>(off));
    }
    return e;
}

esp_err_t CC1101Driver::strobe(uint8_t cmd) {
    uint8_t status = 0;
    return strobe(cmd, status);
}

esp_err_t CC1101Driver::read_reg(uint8_t addr, uint8_t& value) {
    SpiLock lk(spi_mtx_, spi_dev_);
    // Read+Burst-Header (0xC0) ist universell sicher:
    //   - Konfig-Register (0x00..0x2E): Auto-Increment, bei einem Datenbyte == Single-Read.
    //   - Status-Register (0x30..0x3D): Burst-Bit ist PFLICHT, sonst werden sie als Strobes
    //     interpretiert (SWRS061I §10.4). Beobachtet auf Hardware: ohne Burst-Bit liefert
    //     PARTNUM=0xFF statt 0x00. a-culfw cc1100.c:431 macht es genauso.
    uint8_t tx[2] = {static_cast<uint8_t>(0xC0 | (addr & 0x3F)), 0xFF};
    uint8_t rx[2] = {};
    if (auto e = cs_low_wait_ready(); e != ESP_OK) return e;
    auto e = spi_xfer(tx, rx, 2);
    cs_high();
    if (e != ESP_OK) return e;
    last_status_ = rx[0];
    value        = rx[1];
    return ESP_OK;
}

esp_err_t CC1101Driver::read_status(uint8_t addr, uint8_t& value) {
    SpiLock lk(spi_mtx_, spi_dev_);
    // Status-Register (0x30..0x3D) Read braucht das Burst-BIT im Header gesetzt,
    // sonst werden sie als Strobes interpretiert (SWRS061I §10.4). Das Bit
    // erlaubt aber KEIN Burst-Lesen: SWRS061I §10.2 (S.32) — "burst access is
    // not available for status registers and they must be accessed one at a
    // time". Deshalb hier bewusst genau EIN Datenbyte je Transaktion und CS
    // danach hoch; PARTNUM/VERSION nie am Stueck lesen.
    uint8_t tx[2] = {static_cast<uint8_t>(0xC0 | (addr & 0x3F)), 0xFF};
    uint8_t rx[2] = {};
    if (auto e = cs_low_wait_ready(); e != ESP_OK) return e;
    auto e = spi_xfer(tx, rx, 2);
    cs_high();
    if (e != ESP_OK) return e;
    last_status_ = rx[0];
    value        = rx[1];
    return ESP_OK;
}

uint8_t CC1101Driver::marc_state() {
    uint8_t v = 0;
    if (read_status(REG_MARCSTATE, v) != ESP_OK) return 0xFF;
    return static_cast<uint8_t>(v & 0x1F);
}

esp_err_t CC1101Driver::flush_rx() {
    // SWRS061I p.33: SFRX ist nur in IDLE / TXFIFO_UNDERFLOW / RXFIFO_OVERFLOW
    // zulaessig. Erst in einen solchen Zustand bringen, dann flushen.
    if (auto e = strobe(SIDLE); e != ESP_OK) return e;
    for (int i = 0; i < 100; ++i) {           // max ~10 ms
        uint8_t ms = marc_state();
        if (ms == MARC_IDLE || ms == MARC_TXFIFO_UNDERFLOW || ms == MARC_RXFIFO_OVERFLOW)
            break;
        ets_delay_us(100);
    }
    return strobe(SFRX);
}

esp_err_t CC1101Driver::rx_fifo_bytes(uint8_t& rxbytes) {
    // SWRZ020E S. 6: lesen, bis derselbe Wert zweimal hintereinander kommt.
    // Bei hinreichend schnellem Lesen (hier µs gegen 0,8 ms je Byte bei 10 kBd)
    // reichen laut Errata hoechstens vier Zugriffe.
    uint8_t prev = 0;
    if (auto e = read_status(REG_RXBYTES, prev); e != ESP_OK) return e;
    for (int i = 0; i < 4; ++i) {
        uint8_t v = 0;
        if (auto e = read_status(REG_RXBYTES, v); e != ESP_OK) return e;
        if (v == prev) break;
        prev = v;
    }
    rxbytes = prev;
    return ESP_OK;
}

bool CC1101Driver::rx_packet_in_progress() {
    uint8_t v = 0;
    if (read_status(REG_PKTSTATUS, v) != ESP_OK) return true;   // im Zweifel nicht lesen
    return (v & 0x08) != 0;                                      // SFD, SWRS061I S. 94
}

bool CC1101Driver::rx_state_safe() {
    // Bis zu fuenf Lesungen: ein Uebergang (etwa RX_END -> RX) ist nach wenigen
    // µs vorbei; erst zwei gleiche Werte zaehlen.
    uint8_t prev = marc_state();
    for (int i = 0; i < 4; ++i) {
        const uint8_t v = marc_state();
        if (v == prev) {
            return v == MARC_IDLE || v == MARC_RX || v == MARC_RX_END
                || v == MARC_RX_RST || v == MARC_RXFIFO_OVERFLOW;
        }
        prev = v;
    }
    return false;
}

bool CC1101Driver::restart_rx_if_idle() {
    // In IDLE ist MARCSTATE stabil lesbar; zweimal IDLE heisst: niemand sendet.
    if (marc_state() != MARC_IDLE || marc_state() != MARC_IDLE) return false;
    strobe(SRX);
    return true;
}

bool CC1101Driver::rx_frame_waiting() {
    if (rx_packet_in_progress()) return false;
    uint8_t rxb = 0;
    if (rx_fifo_bytes(rxb) != ESP_OK) return false;
    if ((rxb & 0x80) || (rxb & 0x7F) == 0) return false;
    return !rx_packet_in_progress();
}

CC1101Driver::RxPacket CC1101Driver::read_rx_packet(std::span<uint8_t> buf, uint8_t& rssi,
                                                    uint8_t& lqi, bool first) {
    // Folgerahmen nur lesen, solange kein Paket einlaeuft — geprueft vor UND
    // nach RXBYTES, damit ein eben begonnenes Paket nicht mitgezaehlt wird und
    // sein Laengenbyte als Kopf eines fertigen Rahmens durchgeht.
    if (!first && rx_packet_in_progress()) return RxPacket::Busy;
    uint8_t rxb = 0;
    if (rx_fifo_bytes(rxb) != ESP_OK) return RxPacket::Error;
    if (rxb & 0x80) return RxPacket::Overflow;
    const uint8_t avail = rxb & 0x7F;
    if (avail == 0) {
        if (first && gdo2_level()) {
            rx_stale_.fetch_add(1, std::memory_order_relaxed);
            clear_stale_rx_signal();
        }
        return RxPacket::Empty;
    }
    if (!first && rx_packet_in_progress()) return RxPacket::Busy;

    uint8_t len_raw = 0;
    if (read_reg(REG_FIFO, len_raw) != ESP_OK) return RxPacket::Error;
    const uint8_t len = len_raw & 0x7F;          // Maske wie a-culfw rf_asksin/rf_moritz
    // Ein Rahmen belegt Laenge + Nutzlast + 2 Statusbytes. Stehen weniger im
    // FIFO, ist der Kopf kein fertiger Rahmen — etwa ein neu einlaufendes Paket
    // hinter einem CRC_OK-Signal, das einen Autoflush ueberlebt hat. Neu
    // aufsetzen, statt einen Teilrahmen als Rahmen auszugeben.
    if (len == 0 || len >= buf.size() || len + 3u > avail) {
        rx_resync_.fetch_add(1, std::memory_order_relaxed);
        flush_rx();
        return RxPacket::Resync;
    }
    buf[0] = len;
    if (read_burst(REG_FIFO, buf.subspan(1, len)) != ESP_OK) return RxPacket::Error;
    if (read_reg(REG_FIFO, rssi) != ESP_OK) return RxPacket::Error;
    if (read_reg(REG_FIFO, lqi) != ESP_OK) return RxPacket::Error;
    // Mit CRC_AUTOFLUSH (SWRS061I S. 73; PKTCTRL1 = 0x0C in AskSin, Moritz
    // und HmIP) verwirft der Chip ein Paket mit falscher CRC selbst; ein
    // fertiger Rahmen traegt daher immer CRC_OK (Bit 7 im zweiten Statusbyte,
    // Tab. 28, S. 37). Fehlt es, wurde ein unfertiges Paket gelesen -- etwa
    // hinter einem CRC_OK-Signal, das einen Autoflush ueberlebt hat, mit
    // einem Laengenbyte, das zufaellig zur Fuellung passte (Review 0896 N2).
    // Erkannt wird ein solcher Lesevorgang nur, wenn das als Statusbyte
    // gelesene Byte Bit 7 = 0 hat; ausschliessen kann die Pruefung ihn nicht.
    if (!(lqi & 0x80)) {
        rx_resync_.fetch_add(1, std::memory_order_relaxed);
        flush_rx();
        return RxPacket::Resync;
    }
    if (!first) rx_extra_.fetch_add(1, std::memory_order_relaxed);
    return RxPacket::Frame;
}

void CC1101Driver::clear_stale_rx_signal() {
    // GDO2 steht, der FIFO ist leer: das CRC_OK-Signal eines Rahmens, den
    // CRC_AUTOFLUSH zusammen mit dem naechsten, fehlerhaften Paket weggeraeumt
    // hat. GDO2_CFG=7 faellt laut SWRS061I Tab. 41 (S. 62) nur, wenn das erste
    // Byte aus dem RX-FIFO gelesen wird; der Autoflush loescht es nicht
    // (gemessen 13.10.2026). Wer jetzt einen Rahmen liest, bekommt den alten
    // Inhalt aus dem FIFO-RAM noch einmal und laesst den Bytezaehler
    // unterlaufen (RXBYTES 0x72 = -14), danach zerfallen die Rahmen. Deshalb:
    // IDLE, FIFO leeren; steht GDO2 dann noch, genau ein Byte lesen (das
    // loescht das Signal) und den FIFO-Zeiger erneut zuruecksetzen. Der
    // Aufrufer startet RX anschliessend selbst.
    // Abweichung vom Datenblatt, bewusst: SWRS061I S. 56 verlangt, nicht ueber
    // den leeren RX-FIFO hinaus zu lesen, weil der Unterlauf die gelesenen
    // Daten verfaelscht. Das Byte hier wird verworfen, und SFRX setzt den FIFO
    // danach zurueck; gemessen 13.10.2026 (Signal weg, Folgerahmen sauber).
    // Eine Alternative ohne Leerlesen (GDO2 kurz umkonfigurieren) ist nicht
    // gemessen (Review 0896 N3).
    flush_rx();
    if (gdo2_level()) {
        uint8_t dummy = 0;
        read_reg(REG_FIFO, dummy);
        flush_rx();
    }
}

bool CC1101Driver::pll_locked() {
    uint8_t v = 0;
    // FSCAL1 ist ein Konfig-Register (0x25) — read_reg, nicht read_status.
    if (read_reg(REG_FSCAL1, v) != ESP_OK) return false;   // kein Lesen = kein Beleg
    return v != 0x3F;                                      // SWRS061I §22.1 p.58
}

esp_err_t CC1101Driver::calibrate(int retries) {
    for (int attempt = 0; attempt <= retries; ++attempt) {
        strobe(SCAL);
        // Manuelle Kalibrierung ~735 us bei 26 MHz, TEST0=0x0B (SWRS061I
        // Table 34 p.54). Auf IDLE warten statt blind 4 ms zu schlafen.
        bool idle = false;
        for (int i = 0; i < 100; ++i) {       // max ~10 ms
            if (marc_state() == MARC_IDLE) { idle = true; break; }
            ets_delay_us(100);
        }
        if (idle && pll_locked()) {
            if (attempt > 0) {
                pll_relocks_.fetch_add(1, std::memory_order_relaxed);
                ESP_LOGW(TAG, "PLL lock erst im %d. Anlauf", attempt + 1);
            }
            return ESP_OK;
        }
        // Vor dem naechsten SCAL sicher zurueck nach IDLE.
        strobe(SIDLE);
    }
    pll_failures_.fetch_add(1, std::memory_order_relaxed);
    ESP_LOGE(TAG, "PLL kein Lock nach %d Kalibrierungen (FSCAL1=0x3F)", retries + 1);
    return ESP_ERR_INVALID_STATE;
}

esp_err_t CC1101Driver::write_reg(uint8_t addr, uint8_t value) {
    // FSCTRL0 gehoert dem Board, nicht der Protokoll-Tabelle: traegt eine
    // CFG-Tabelle dort einen Wert, wird der kalibrierte Ziehfehler hier
    // darueber gelegt.  Tabellen OHNE FSCTRL0-Eintrag (ASKSIN, MORITZ, NATIVE)
    // erreicht dieser Hook nicht — fuer die zieht strobe(SRES) den Wert nach.
    // Wer den Rohwert braucht (Cw, Messreihen), nimmt write_reg_raw().
    if (addr == REG_FSCTRL0) {
        int8_t off = freq_offset_.load(std::memory_order_relaxed);
        if (off != 0) value = static_cast<uint8_t>(off);
    }
    return write_reg_raw(addr, value);
}

esp_err_t CC1101Driver::apply_freq_offset() {
    return write_reg_raw(REG_FSCTRL0,
                         static_cast<uint8_t>(freq_offset_.load(std::memory_order_relaxed)));
}

void CC1101Driver::sample_freqest() {
    int8_t v = 0;
    if (read_freqest(v)) push_freqest(v);
    note_rx_frame();
}

bool CC1101Driver::read_freqest(int8_t& out) {
    // Waehrend des Empfangs aendert sich FREQEST laufend: lesen, bis derselbe
    // Wert zweimal hintereinander kommt, wie rx_fifo_bytes() (SWRZ020E S. 6).
    // Pendelt er sich nicht ein, lieber keinen Wert als einen verfaelschten.
    uint8_t prev = 0;
    if (read_status(REG_FREQEST, prev) != ESP_OK) return false;
    for (int i = 0; i < 4; ++i) {
        uint8_t v = 0;
        if (read_status(REG_FREQEST, v) != ESP_OK) return false;
        if (v == prev) {
            out = static_cast<int8_t>(v);
            return true;
        }
        prev = v;
    }
    return false;
}

void CC1101Driver::push_freqest(int8_t v) {
    // Nur der Ringpuffer: kein Buszugriff, also auch kein spi_device_acquire_bus
    // — der haette hinter einem laufenden W5500-Frame (bis ~1 ms) gewartet und
    // umgekehrt den W5500 fuer eine reine RAM-Operation ausgesperrt.
    SpiLock lk(spi_mtx_, nullptr);
    freqest_buf_[freqest_pos_] = v;
    freqest_pos_ = (freqest_pos_ + 1) % kFreqEstCap;
    if (freqest_n_ < kFreqEstCap) ++freqest_n_;
}

void CC1101Driver::note_rx_frame() {
    SpiLock lk(spi_mtx_, nullptr);   // RAM-only, siehe push_freqest()
    ++rx_frames_;
}

size_t CC1101Driver::rx_frames() const {
    SpiLock lk(spi_mtx_, nullptr);   // RAM-only, siehe push_freqest()
    return rx_frames_;
}

bool CC1101Driver::freqest_median(int8_t& out, size_t& count) const {
    SpiLock lk(spi_mtx_, nullptr);   // RAM-only, siehe sample_freqest()
    count = freqest_n_;
    if (freqest_n_ == 0) return false;
    int8_t tmp[kFreqEstCap];
    std::copy(freqest_buf_, freqest_buf_ + freqest_n_, tmp);
    std::sort(tmp, tmp + freqest_n_);
    out = tmp[freqest_n_ / 2];
    return true;
}

void CC1101Driver::freqest_reset() {
    SpiLock lk(spi_mtx_, nullptr);   // RAM-only, siehe sample_freqest()
    freqest_n_ = 0;
    freqest_pos_ = 0;
    rx_frames_ = 0;
}

esp_err_t CC1101Driver::write_reg_raw(uint8_t addr, uint8_t value) {
    SpiLock lk(spi_mtx_, spi_dev_);
    uint8_t tx[2] = {static_cast<uint8_t>(addr & 0x3F), value};
    uint8_t rx[2] = {};
    if (auto e = cs_low_wait_ready(); e != ESP_OK) return e;
    auto e = spi_xfer(tx, rx, 2);
    cs_high();
    if (e != ESP_OK) return e;
    last_status_ = rx[0];
    return ESP_OK;
}

esp_err_t CC1101Driver::write_burst(uint8_t addr, std::span<const uint8_t> data) {
    if (data.empty() || data.size() > 64) return ESP_ERR_INVALID_ARG;
    SpiLock lk(spi_mtx_, spi_dev_);
    uint8_t buf[65];
    buf[0] = static_cast<uint8_t>(0x40 | (addr & 0x3F));    // WRITE + BURST
    std::memcpy(buf + 1, data.data(), data.size());
    if (auto e = cs_low_wait_ready(); e != ESP_OK) return e;
    auto e = spi_xfer(buf, nullptr, data.size() + 1);
    cs_high();
    return e;
}

esp_err_t CC1101Driver::read_burst(uint8_t addr, std::span<uint8_t> data) {
    if (data.empty() || data.size() > 64) return ESP_ERR_INVALID_ARG;
    SpiLock lk(spi_mtx_, spi_dev_);
    uint8_t tx[65] = {};
    uint8_t rx[65] = {};
    tx[0] = static_cast<uint8_t>(0xC0 | (addr & 0x3F));     // READ + BURST
    if (auto e = cs_low_wait_ready(); e != ESP_OK) return e;
    auto e = spi_xfer(tx, rx, data.size() + 1);
    cs_high();
    if (e != ESP_OK) return e;
    last_status_ = rx[0];
    std::memcpy(data.data(), rx + 1, data.size());
    return ESP_OK;
}

static esp_err_t attach_gdo_irq_impl(int pin, CC1101Driver::IrqHandler h, void* arg, bool rising) {
    if (pin < 0) return ESP_ERR_INVALID_ARG;
    auto p = static_cast<gpio_num_t>(pin);
    // gpio_reset_pin entfernt jegliches Peripheral-Matrix-Routing (z.B. RMT-RX
    // von SignalDuino-RF) das auf diesem Pin liegen koennte. Ohne reset bleibt
    // GDO2 nach dem PulseCapture→AskSin/Moritz Mode-Switch im RMT-input-Routing
    // stecken; gpio_intr_handler feuert nicht und gpio_get_level liest den
    // (jetzt toten) Matrix-Wert statt das Pin-Pad. Verifiziert 2026-04-30 Build
    // 0.1.96+ als Pflicht-Reset, beim Revert auf 0.1.94-baseline irrtuemlich
    // mitverloren — Build 0.1.136 stellt das wieder her.
    gpio_reset_pin(p);
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = rising ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE,
    };
    if (auto e = gpio_config(&io); e != ESP_OK) return e;

    if (!g_isr_service_installed) {
        // Der Dienst ist prozessweit. Hat ihn schon ein anderer installiert
        // (eth_drv fuer W5500-INTn), kommt ESP_ERR_INVALID_STATE plus ein
        // E-Logeintrag aus gpio.c — fuer uns kein Fehler, also ohne Log.
        const esp_log_level_t lvl = esp_log_level_get("gpio");
        esp_log_level_set("gpio", ESP_LOG_NONE);
        const esp_err_t e = gpio_install_isr_service(0);
        esp_log_level_set("gpio", lvl);
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
        g_isr_service_installed = true;
    }
    gpio_isr_handler_remove(p);  // idempotent — vorherigen Handler entfernen
    return gpio_isr_handler_add(p, h, arg);
}

esp_err_t CC1101Driver::attach_gdo0_irq(IrqHandler h, void* arg, bool rising) {
    return attach_gdo_irq_impl(cfg_.pin_gdo0, h, arg, rising);
}

esp_err_t CC1101Driver::attach_gdo2_irq(IrqHandler h, void* arg, bool rising) {
    return attach_gdo_irq_impl(cfg_.pin_gdo2, h, arg, rising);
}

int CC1101Driver::gdo0_level() const {
    return (cfg_.pin_gdo0 < 0) ? 0 : gpio_get_level(static_cast<gpio_num_t>(cfg_.pin_gdo0));
}

int CC1101Driver::gdo2_level() const {
    return (cfg_.pin_gdo2 < 0) ? 0 : gpio_get_level(static_cast<gpio_num_t>(cfg_.pin_gdo2));
}

esp_err_t CC1101Driver::set_gdo0_irq_enabled(bool enable) {
    if (cfg_.pin_gdo0 < 0) return ESP_ERR_INVALID_ARG;
    auto p = static_cast<gpio_num_t>(cfg_.pin_gdo0);
    return enable ? gpio_intr_enable(p) : gpio_intr_disable(p);
}

esp_err_t CC1101Driver::set_gdo2_irq_enabled(bool enable) {
    if (cfg_.pin_gdo2 < 0) return ESP_ERR_INVALID_ARG;
    auto p = static_cast<gpio_num_t>(cfg_.pin_gdo2);
    return enable ? gpio_intr_enable(p) : gpio_intr_disable(p);
}

void CC1101Driver::detach_gdo_irqs() {
    // remove() nur, wenn attach den ISR-Dienst je installiert hat: sonst gibt
    // es keinen Handler, und gpio.c quittiert den Aufruf mit einem E-Log — auf
    // Boards ohne W5500 bei jedem Boot mit Netz, weil die Frontends dann den
    // Funkmodus umschalten.
    if (cfg_.pin_gdo0 >= 0) {
        auto p = static_cast<gpio_num_t>(cfg_.pin_gdo0);
        gpio_intr_disable(p);
        if (g_isr_service_installed) gpio_isr_handler_remove(p);
    }
    if (cfg_.pin_gdo2 >= 0) {
        auto p = static_cast<gpio_num_t>(cfg_.pin_gdo2);
        gpio_intr_disable(p);
        if (g_isr_service_installed) gpio_isr_handler_remove(p);
    }
}

} // namespace culfw32
