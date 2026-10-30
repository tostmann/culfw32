// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "esp_err.h"
#include <cstddef>
#include <cstdint>

namespace culfw32 {

// ST M24C32-F (U1 auf CUL60.sch), 4 KB I2C-EEPROM. Werte aus dem Datenblatt
// DocID4578 Rev 30:
//   * Page Write bis 32 Byte, ueber das Seitenende hinaus rollt die Adresse
//     innerhalb der Seite um (S. 16) -> nie ueber eine Seitengrenze schreiben
//   * tW max 5 ms (S. 30/31), Ende per ACK-Polling erkennbar (S. 18)
//   * WC high sperrt das Schreiben, Datenbytes werden dann nicht quittiert;
//     WC low oder offen gibt frei (S. 8)
//   * mehr als 4 Mio. Schreibzyklen (S. 1)
// Nur fuer den Gebrauch durch NvsEeprom; die Aufteilung des Bausteins
// (culfw-Abbild 0x0000..0x03FF, q-culfw 0x0400..0x07ED, Werksdatensatz ab
// 0x0800) steht dort.
class M24c32 {
public:
    static constexpr uint16_t kSize = 4096;
    static constexpr uint16_t kPage = 32;

    // Bus + Device anlegen und den Baustein per Adress-ACK suchen. Ohne ACK
    // (Laborboards ohne U1): ESP_ERR_NOT_FOUND, alle Ressourcen wieder frei.
    esp_err_t init(int sda, int scl, int wc, uint8_t addr, uint32_t hz);

    // Gibt Device und Bus wieder frei (NvsEeprom::attach_i2c_ verwirft die
    // Instanz, wenn das Abbild nicht lesbar war). WC bleibt high.
    ~M24c32();

    esp_err_t read(uint16_t addr, uint8_t* out, size_t len);

    // Seitenweise schreiben (WC nur dafuer low), je Seite ACK-Polling, danach
    // den ganzen Bereich zuruecklesen und vergleichen.
    esp_err_t write(uint16_t addr, const uint8_t* data, size_t len);

private:
    esp_err_t wait_ready_();

    void*   bus_  = nullptr;   // i2c_master_bus_handle_t
    void*   dev_  = nullptr;   // i2c_master_dev_handle_t
    int     wc_   = -1;
    uint8_t addr_ = 0x50;
};

} // namespace culfw32
