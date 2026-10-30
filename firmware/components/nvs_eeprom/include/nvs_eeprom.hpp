// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "esp_err.h"
#include "sdkconfig.h"
#include <cstdint>
#include <mutex>
#include <span>
#include <vector>

namespace culfw32 {

// Persistenter flacher Adressraum (16-bit), das culfw-EEPROM. Ungeschriebene
// Bytes liefern 0xFF (analog uninitialisierter AVR-EEPROM).
//
// Medium: das I2C-EEPROM U1 (M24C32, CUL60.sch), sofern bestueckt — es
// ueberlebt wie das AVR-EEPROM jedes Flashen, auch das Factory-Image des
// Web-Flashers, das NVS loescht. Ohne U1 (Laborboards) traegt NVS allein.
// NVS laeuft in jedem Fall als Spiegel mit: aeltere Firmware ohne I2C-Treiber
// findet dort denselben Stand.
//
// Beim Boot gilt: EEPROM mit gueltigem Magic gewinnt (NVS wird nachgezogen);
// EEPROM leer, NVS gueltig -> einmalige Migration NVS -> EEPROM; beide leer
// -> der Werksreset in main schreibt in beide.
class NvsEeprom {
public:
    enum class Backend : uint8_t { Nvs, I2c };

    static constexpr size_t kSize = CONFIG_CULFW32_EEPROM_SIZE;

    esp_err_t init();

    // Lesen: out-of-range -> 0xFF (wie EEPROM nach Power-On).
    uint8_t   read_byte(uint16_t addr) const;

    // Schreiben: out-of-range -> ESP_ERR_INVALID_ARG, sonst sofort persistiert.
    esp_err_t write_byte(uint16_t addr, uint8_t value);

    // Bulk-Operationen fuer effizientes Pair-Table-Handling.
    void      read_block (uint16_t addr, std::span<uint8_t> out) const;
    esp_err_t write_block(uint16_t addr, std::span<const uint8_t> data);

    size_t    size() const { return kSize; }

    // Welches Medium das Abbild traegt, plus Fehlerzaehler der I2C-Schreibpfade
    // (ein Fehler dort laesst NVS trotzdem aktuell; Abbild bleibt konsistent
    // bis zum naechsten Boot, der dann das EEPROM liest).
    Backend     backend() const { return backend_; }
    const char* backend_name() const;
    uint32_t    i2c_errors() const { return i2c_errors_; }

private:
    friend void factory_reset(NvsEeprom& ee);
    esp_err_t flush_();
    esp_err_t commit_(uint16_t lo, uint16_t hi);   // geaenderter Bereich -> I2C + NVS
    void      attach_i2c_();

    std::vector<uint8_t> buf_;       // kSize Bytes, default 0xFF
    void*                handle_ = nullptr;  // nvs_handle_t (opaque to header)
    void*                i2c_    = nullptr;  // M24c32* (opaque to header)
    bool                 initialized_ = false;
    Backend              backend_ = Backend::Nvs;
    uint32_t             i2c_errors_ = 0;
    std::mutex           wr_mu_;              // Schreiber kommen aus mehreren Frontend-Tasks
};

// Byte-genau a-culfw `cc_factory_reset()` + `eeprom_factory_reset()`-Pfad
// (fncollection.c:268). Setzt alle EE-Bytes auf a-culfw-Defaults, **ohne**
// Reset auszuloesen — Caller entscheidet ob ein Restart folgt (`e\n` ja, `ex\n`
// nein, Boot-Pfad bei Magic-Mismatch nein).
//   * EE_MAGIC          = (1, 26)
//   * EE_CC1100_CFG     = CC1100_CFG_DEFAULTS (41 bytes)
//   * EE_CC1100_PA      = Profil 8 (5 dBm no PA ramping, 8 bytes) — via cc_set_pa()
//   * EE_CC1100_CFG[0x22] FREND0 = 0x11 (no PA ramping)
//   * EE_RF_ROUTER_ID   = 0
//   * EE_RF_ROUTER_ROUTER = 0
//   * EE_REQBL          = 0
//   * EE_LED            = 2
//   * Netzblock 98..123 = ethernet_reset()
void factory_reset(NvsEeprom& ee);

// Byte-genau a-culfw `ethernet.c::ethernet_reset()` (CUNO): DHCP=1,
// 192.168.0.244 / 255.255.255.0 / GW 192.168.0.1, NTP 0.0.0.0, Port 2323,
// NTP-Offset 0. Einzige Abweichung: EE_MAC_ADDR wird FF..FF (= automatisch,
// siehe eth_drv), nicht mit einer eingefrorenen MAC beschrieben.
void ethernet_reset(NvsEeprom& ee);

// ethernet_reset(), aber nur wenn der Netzblock nie beschrieben wurde
// (EE_USE_DHCP == 0xFF). Bestandsgeraete haben einen gueltigen Magic und
// laufen deshalb nie durch den Werksreset; ohne das zeigen `Ria`/`Rid` dort
// 255.255.255.255/FF. Gehoert in den Boot, unabhaengig von der W5500-Platine.
void ethernet_reset_if_blank(NvsEeprom& ee);

// Byte-genau a-culfw `cc1100.c::cc_set_pa()` mit FULL_CC1100_PA. Schreibt das
// gewuenschte 8-Byte-PA-Profil in EE_CC1100_PA und passt FREND0 (Reg 0x22) in
// EE_CC1100_CFG an: idx 5..9 → 0x11 (no ramp), sonst 0x17 (ramp).
//   idx > 9 → auf 8 geclamped (factory-default 5 dBm no ramp).
// Wendet das Profil NICHT direkt am Chip an — Caller (Frontend handle_x oder
// Boot-Pfad) entscheidet, ob ein Burst zum CC1101 gehoert.
void cc_set_pa(NvsEeprom& ee, uint8_t idx);

} // namespace culfw32
