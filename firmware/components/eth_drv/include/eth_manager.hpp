// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "esp_err.h"
#include "esp_event_base.h"
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

namespace culfw32 {

class NvsEeprom;

// W5500-Ethernet auf der Erweiterungsplatine an J1 (CUL60.sch, W5500EXT).
//
// Hardware-Fakten, die das Design bestimmen:
//   * Der W5500 teilt SCLK/MISO/MOSI (GPIO19/20/21) mit dem CC1101. Getrennt
//     sind die beiden nur ueber ihre Chip-Selects: CC1101 = GPIO18, W5500 =
//     GPIO17. Den Bus initialisiert der CC1101-Treiber (ohne DMA, 64-Byte-
//     Transfers); dieser Treiber haengt sich nur als zweites Device daran.
//   * GPIO17 ist zugleich U0RXD. Mit gesteckter Platine gibt es den UART0-
//     Transport auf USB-C SBU nicht — das ist eine Eigenschaft der Hardware.
//     Deshalb muss probe() VOR dem UART-Transport laufen.
//   * Kein Reset-Pin am Steckverbinder; der W5500 verlaesst sich auf seinen
//     Power-on-Reset (wie beim TUL32 mit derselben Platine).
//   * MISO teilen sich beide Chips: der W5500 schaltet seinen Ausgang bei
//     SCSn high hochohmig (W5500 DS v1.1.0 S. 61, T_CHZ max 2,16 ns), der
//     CC1101 nur solange IOCFG1 auf 0x2E (3-state) steht (SWRS061I §11.2).
//
// Die Busteilung: jede W5500-Transaktion haelt den SPI-Bus per
// spi_device_acquire_bus() ueber die ganze SCSn-low-Phase, genau wie der
// CC1101-Treiber ueber seine CSn-low-Phase. Dazwischen kann keiner der beiden
// den anderen anfahren. Ohne DMA sind Einzeltransfers auf 64 Byte begrenzt —
// laengere Frames gehen deshalb in Stuecken bei durchgehend gehaltenem SCSn
// (W5500 Variable Data Length Mode, der Frame endet erst mit SCSn high).
//
// Konfiguration byte-genau a-culfw (CUNO): EE_MAC_ADDR 98..103, EE_USE_DHCP
// 104, EE_IP4_ADDR 105..108, EE_IP4_NETMASK 109..112, EE_IP4_GATEWAY
// 113..116. Wirksam beim Start und nach `Ei`, wie in a-culfw.
class EthManager {
public:
    enum class State : uint8_t { Absent, Stopped, NoLink, LinkUp, GotIp };

    // Woher die Ethernet-MAC stammt (Ausgabe in `Ec`-Diagnose und Registry).
    enum class MacSource : uint8_t { Eeprom, Efuse, Chip };

    // Sucht den W5500 an J1: VERSIONR (0x0039) == 0x04 und ein Schreib-/Lese-
    // Test auf RTR (0x0019). Muss NACH der CC1101-Initialisierung laufen (der
    // SPI-Bus gehoert dem CC1101-Treiber) und VOR dem UART0-Transport.
    // Ohne Platine wird das SPI-Device wieder entfernt und GPIO17 freigegeben.
    bool probe();
    bool present() const { return present_; }

    // Treiber + netif starten. DHCP oder statische Adresse aus dem culfw-
    // EEPROM. Blockiert nicht — Link und Lease kommen asynchron per Event.
    esp_err_t start(NvsEeprom* ee, const std::string& hostname);

    // `Ei`: Einstellungen neu aus dem EEPROM lesen, Interface neu aufsetzen.
    esp_err_t reinit();

    State    state() const;
    uint32_t ip4() const;                  // NBO wie esp_ip4_addr_t; 0 = keine
    void     mac(uint8_t out[6]) const;    // wirksame MAC (nach start())
    MacSource mac_source() const { return mac_src_; }

    // `Ed`: Debug-Stufe 0..3 hochzaehlen wie a-culfw `eth_debug`. Wirkung hier:
    // Log-Level der Ethernet-Tags (0 = WARN, sonst INFO); keine Paket-Dumps.
    uint8_t bump_debug();

    // Wirksame MAC: EEPROM-Override (gueltige Unicast-MAC) -> busware-MAC aus
    // eFuse BLOCK3 (nur mit "BW"-Datensatz) -> ESP_MAC_ETH des Chips.
    static MacSource effective_mac(NvsEeprom* ee, uint8_t out[6]);

    static const char* state_name(State s);
    static const char* mac_source_name(MacSource s);

private:
    static void event_handler(void* arg, esp_event_base_t base, int32_t id, void* data);
    void        on_event(esp_event_base_t base, int32_t id, void* data);
    esp_err_t   apply_mac_();
    esp_err_t   apply_ip_config_();

    NvsEeprom*        ee_       = nullptr;
    void*             spi_      = nullptr;   // SpiCtx (eth_manager.cpp)
    void*             eth_      = nullptr;   // esp_eth_handle_t
    void*             netif_    = nullptr;   // esp_netif_t*
    void*             glue_     = nullptr;   // esp_eth_netif_glue_handle_t
    bool              present_  = false;
    std::atomic<bool> started_{false};
    std::atomic<bool> link_up_{false};
    uint8_t           debug_    = 0;

    mutable std::mutex mac_mu_;              // mac_ wird von Ei (Frontend) geschrieben
    uint8_t           mac_[6]   = {};
    MacSource         mac_src_  = MacSource::Chip;
};

} // namespace culfw32
