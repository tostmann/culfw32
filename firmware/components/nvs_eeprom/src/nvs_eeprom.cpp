// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann
// Portions derived from a-culfw (https://github.com/tostmann/a-culfw),
// GNU GPL version 2 or (at your option) any later version.

#include "nvs_eeprom.hpp"
#include "culfw_ee_addr.hpp"
#include "culfw_ee_defaults.hpp"
#include "m24c32.hpp"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG = "nvs-ee";
constexpr const char* NS  = "culfw_ee";
constexpr const char* KEY = "ee";

#ifdef CONFIG_CULFW32_EEPROM_I2C
// Aufteilung des M24C32: 0x0000..0x03FF culfw-Abbild (CULFW32), 0x0400..0x07ED
// q-culfw (Daten bis 0x07E9, Formatmarke "qcE1" ab 0x07EA, seit q-culfw
// 2.0.114), 0x07F0 Scratch des Selbsttests, 0x0800..0x0F7F Werksdatensatz
// "BWFD", ab 0x0F80 reserviert (Fertigungs-Selbsttest).
// Jede Firmware schreibt nur in ihren Bereich, damit ein Stick zwischen
// CULFW32 und q-culfw wechseln kann, ohne dass eine die Daten der anderen
// loescht. Bis 0.1.907 liess diese Pruefung das Abbild bis 0x07EF zu.
constexpr uint16_t kI2cBase = CONFIG_CULFW32_EEPROM_I2C_BASE;
constexpr uint16_t kI2cEnd  = 0x0400;
static_assert(kI2cBase + NvsEeprom::kSize <= kI2cEnd,
              "culfw-EEPROM passt nicht in den CULFW32-Bereich des M24C32 (0x0000..0x03FF)");
#endif

bool magic_ok(const std::vector<uint8_t>& b) {
    return b.size() > ee::MAGIC_OFFSET + 1 &&
           b[ee::MAGIC_OFFSET] == ee::MAGIC_V1 && b[ee::MAGIC_OFFSET + 1] == ee::MAGIC_V2;
}
}

esp_err_t NvsEeprom::init() {
    if (initialized_) return ESP_OK;

    // NVS-Flash initialisieren. Bei korruptem oder neu-formatiertem Flash:
    // einmalig erase + retry. Standard-Pattern aus ESP-IDF storage examples.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "nvs_flash_init: %s -> erase + retry", esp_err_to_name(err));
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return err;
    }

    nvs_handle_t h;
    if ((err = nvs_open(NS, NVS_READWRITE, &h)) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(%s) failed: %s", NS, esp_err_to_name(err));
        return err;
    }
    handle_ = reinterpret_cast<void*>(static_cast<uintptr_t>(h));

    buf_.assign(kSize, 0xFF);

    size_t expected = kSize;
    err = nvs_get_blob(h, KEY, buf_.data(), &expected);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "first boot: %u bytes default-0xFF", static_cast<unsigned>(kSize));
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_get_blob: %s", esp_err_to_name(err));
        std::memset(buf_.data(), 0xFF, kSize);
    } else if (expected != kSize) {
        // Groesse hat sich geaendert (Kconfig anders) — alten Inhalt vorne behalten,
        // Rest mit 0xFF. Pragmatisch, kein Format-Bruch da culfw-EEPROM
        // ohnehin "0xFF = ungeschrieben" semantik hat.
        ESP_LOGW(TAG, "blob size mismatch: stored=%u expected=%u — keep prefix",
                 static_cast<unsigned>(expected), static_cast<unsigned>(kSize));
        if (expected > kSize) buf_.resize(kSize);
        else buf_.resize(kSize, 0xFF);
    } else {
        ESP_LOGI(TAG, "loaded %u bytes from NVS", static_cast<unsigned>(kSize));
    }

    initialized_ = true;
    attach_i2c_();
    return ESP_OK;
}

void NvsEeprom::attach_i2c_() {
#ifdef CONFIG_CULFW32_EEPROM_I2C
    auto* m = new M24c32();
    esp_err_t e = m->init(CONFIG_CULFW32_EEPROM_I2C_SDA, CONFIG_CULFW32_EEPROM_I2C_SCL,
                          CONFIG_CULFW32_EEPROM_I2C_WC, CONFIG_CULFW32_EEPROM_I2C_ADDR,
                          CONFIG_CULFW32_EEPROM_I2C_HZ);
    if (e != ESP_OK) {
        delete m;
        ESP_LOGI(TAG, "no I2C EEPROM at 0x%02x (%s) -> NVS only",
                 CONFIG_CULFW32_EEPROM_I2C_ADDR, esp_err_to_name(e));
        return;
    }
    std::vector<uint8_t> img(kSize, 0xFF);
    if (e = m->read(kI2cBase, img.data(), kSize); e != ESP_OK) {
        // Baustein antwortet, liefert aber nicht: lieber beim Stand von NVS
        // bleiben, als ein halb gelesenes Abbild zum Original zu machen.
        delete m;
        ESP_LOGE(TAG, "I2C EEPROM read failed (%s) -> NVS only", esp_err_to_name(e));
        return;
    }
    i2c_     = m;
    backend_ = Backend::I2c;

    if (magic_ok(img)) {
        if (img != buf_) {
            buf_ = img;
            flush_();
            ESP_LOGI(TAG, "loaded %u bytes from I2C EEPROM, NVS mirror refreshed",
                     static_cast<unsigned>(kSize));
        } else {
            ESP_LOGI(TAG, "loaded %u bytes from I2C EEPROM (= NVS)", static_cast<unsigned>(kSize));
        }
    } else if (magic_ok(buf_)) {
        // Erster Start dieser Firmware auf einem Bestandsgeraet: das NVS-Abbild
        // wandert einmal komplett ins EEPROM. Magic ZULETZT: bricht die
        // Migration ab (Stromausfall nach der ersten Seite), traegt das EEPROM
        // sonst einen gueltigen Magic vor 0xFF — und gewinnt beim naechsten
        // Boot gegen den intakten NVS-Spiegel, der dann ueberschrieben wird.
        static_assert(ee::MAGIC_OFFSET == 0, "migration order assumes the magic at offset 0");
        constexpr size_t kMagicLen = 2;
        e = m->write(static_cast<uint16_t>(kI2cBase + kMagicLen), buf_.data() + kMagicLen,
                     kSize - kMagicLen);
        if (e == ESP_OK) e = m->write(kI2cBase, buf_.data(), kMagicLen);
        if (e != ESP_OK) {
            ++i2c_errors_;
            ESP_LOGE(TAG, "migration NVS -> I2C EEPROM failed: %s", esp_err_to_name(e));
        } else {
            ESP_LOGW(TAG, "migrated %u bytes NVS -> I2C EEPROM", static_cast<unsigned>(kSize));
        }
    } else {
        ESP_LOGI(TAG, "I2C EEPROM and NVS blank -> factory reset follows");
    }
#endif
}

const char* NvsEeprom::backend_name() const {
    return backend_ == Backend::I2c ? "i2c" : "nvs";
}

esp_err_t NvsEeprom::commit_(uint16_t lo, uint16_t hi) {
    esp_err_t e_i2c = ESP_OK;
#ifdef CONFIG_CULFW32_EEPROM_I2C
    if (i2c_) {
        e_i2c = static_cast<M24c32*>(i2c_)->write(static_cast<uint16_t>(kI2cBase + lo),
                                                  &buf_[lo], static_cast<size_t>(hi - lo + 1));
        if (e_i2c != ESP_OK) ++i2c_errors_;
    }
#else
    (void)lo; (void)hi;
#endif
    const esp_err_t e_nvs = flush_();
    return e_i2c != ESP_OK ? e_i2c : e_nvs;
}

uint8_t NvsEeprom::read_byte(uint16_t addr) const {
    if (!initialized_ || addr >= kSize) return 0xFF;
    return buf_[addr];
}

esp_err_t NvsEeprom::write_byte(uint16_t addr, uint8_t value) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (addr >= kSize)  return ESP_ERR_INVALID_ARG;
    std::lock_guard<std::mutex> lk(wr_mu_);
    if (buf_[addr] == value) return ESP_OK;     // nichts zu tun
    buf_[addr] = value;
    return commit_(addr, addr);
}

void NvsEeprom::read_block(uint16_t addr, std::span<uint8_t> out) const {
    for (size_t i = 0; i < out.size(); ++i) {
        out[i] = read_byte(static_cast<uint16_t>(addr + i));
    }
}

esp_err_t NvsEeprom::write_block(uint16_t addr, std::span<const uint8_t> data) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (addr + data.size() > kSize) return ESP_ERR_INVALID_ARG;
    std::lock_guard<std::mutex> lk(wr_mu_);
    // Nur den tatsaechlich geaenderten Bereich ins EEPROM: schont die
    // Schreibzyklen und spart je unberuehrter Seite ~5 ms tW.
    int lo = -1, hi = -1;
    for (size_t i = 0; i < data.size(); ++i) {
        if (buf_[addr + i] != data[i]) {
            buf_[addr + i] = data[i];
            if (lo < 0) lo = static_cast<int>(addr + i);
            hi = static_cast<int>(addr + i);
        }
    }
    return lo >= 0 ? commit_(static_cast<uint16_t>(lo), static_cast<uint16_t>(hi)) : ESP_OK;
}

esp_err_t NvsEeprom::flush_() {
    auto h = static_cast<nvs_handle_t>(reinterpret_cast<uintptr_t>(handle_));
    if (auto e = nvs_set_blob(h, KEY, buf_.data(), kSize); e != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_blob: %s", esp_err_to_name(e));
        return e;
    }
    return nvs_commit(h);
}

void cc_set_pa(NvsEeprom& ee, uint8_t idx) {
    // Byte-genau a-culfw cc1100.c:276 cc_set_pa() unter FULL_CC1100_PA.
    if (idx > 9) idx = 8;

    const uint8_t* profile = &ee::CC1100_PA_DEFAULTS[idx * ee::CC1100_PA_SIZE];
    ee.write_block(ee::CC1100_PA,
                   std::span<const uint8_t>(profile, ee::CC1100_PA_SIZE));

    // FREND0 (Reg 0x22) anpassen wie a-culfw cc1100.c:289:
    //   idx 5..9 (no PA ramping) → 0x11
    //   sonst                    → 0x17
    const uint8_t frend0 = (idx > 4 && idx < 10) ? 0x11 : 0x17;
    ee.write_byte(ee::CC1100_CFG + 0x22, frend0);
}

void factory_reset(NvsEeprom& ee) {
    // Magic-Cookie zuerst (wie a-culfw fncollection.c:271-272).
    ee.write_byte(ee::MAGIC_OFFSET,     ee::MAGIC_V1);
    ee.write_byte(ee::MAGIC_OFFSET + 1, ee::MAGIC_V2);

    // CC1100_CFG-Defaults ueber den Block-Pfad — ein Flush statt 41.
    ee.write_block(ee::CC1100_CFG, ee::CC1100_CFG_DEFAULTS);

    // FIXME 433-SKU: a-culfw cc1100.c:329 patcht FREQ2/1/0 (Reg 0x0D..0x0F) auf
    // 10:B0:71 (433.92 MHz) wenn MARK433_PIN low — das ist ein Hardware-
    // Strapping-Pin, der nur auf der 433er-Hardware verbaut ist. CULFW32 v1
    // hat nur 868 MHz BALUN; Pin existiert auf der jetzigen Hardware nicht.
    // Bei 433-SKU-Spin: Kconfig `CULFW32_FBAND_DETECT_PIN` einfuehren,
    // hier per gpio_get_level() lesen und FREQ-Bytes ueberschreiben:
    //   ee.write_byte(ee::CC1100_CFG + 0x0D, 0x10);
    //   ee.write_byte(ee::CC1100_CFG + 0x0E, 0xB0);
    //   ee.write_byte(ee::CC1100_CFG + 0x0F, 0x71);
    // Gleicher Patch dann fuer EE_FASTRF_CFG + 0x0D (a-culfw gleicher Block).

    // Skalare Defaults byte-genau a-culfw `eeprom_factory_reset()`.
    ee.write_byte(ee::RF_ROUTER_ID,     0);
    ee.write_byte(ee::RF_ROUTER_ROUTER, 0);
    ee.write_byte(ee::REQBL,            0);
    ee.write_byte(ee::LED,              2);

    // PA-Profil 8 (5 dBm no PA ramping) byte-genau a-culfw cc_factory_reset()
    // → cc_set_pa(8) — schreibt EE_CC1100_PA + ueberschreibt FREND0 mit 0x11.
    cc_set_pa(ee, 8);

    // a-culfw eeprom_factory_reset() ruft unter HAS_ETHERNET ethernet_reset().
    ethernet_reset(ee);
}

void ethernet_reset(NvsEeprom& ee) {
    // Byte-genau a-culfw ethernet.c ethernet_reset(): DHCP an, Fallback-
    // Adresse 192.168.0.244/24, Gateway 192.168.0.1, Port 2323, NTP-Server
    // 0.0.0.0 (= Gateway), NTP-Offset 0.
    static constexpr uint8_t net[] = {
        1,                      // 104 EE_USE_DHCP
        192, 168,   0, 244,     // 105 EE_IP4_ADDR
        255, 255, 255,   0,     // 109 EE_IP4_NETMASK
        192, 168,   0,   1,     // 113 EE_IP4_GATEWAY
          0,   0,   0,   0,     // 117 EE_IP4_NTPSERVER
        static_cast<uint8_t>(ee::IP4_TCPLINK_PORT_DEFAULT & 0xFF),         // 121 LE
        static_cast<uint8_t>((ee::IP4_TCPLINK_PORT_DEFAULT >> 8) & 0xFF),  // 122
        0,                      // 123 EE_IP4_NTPOFFSET
    };
    static_assert(sizeof(net) == ee::IP4_NTPOFFSET + 1 - ee::USE_DHCP);
    ee.write_block(ee::USE_DHCP, net);

    // Abweichung von a-culfw: dort schreibt ethernet_reset() die MAC (busware-
    // OUI aus EE_DUDETTE_MAC bzw. aus der Chip-Signatur) ins EEPROM. Hier steht
    // FF..FF = "automatisch": eth_drv nimmt dann die busware-MAC aus eFuse
    // BLOCK3 oder die Chip-MAC. Eine beim Werksreset eingefrorene MAC waere
    // falsch, sobald BLOCK3 erst nach dem ersten Boot gebrannt wird — und genau
    // so laeuft die Fertigung (Firmware bootet vor provision.py).
    static constexpr uint8_t mac_auto[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    ee.write_block(ee::MAC_ADDR, mac_auto);
}

void ethernet_reset_if_blank(NvsEeprom& ee) {
    if (ee.read_byte(ee::USE_DHCP) != 0xFF) return;
    // Bestandsgeraete haben an 121/122 schon einen TCP-Port: factory_reset()
    // schrieb dort vor diesem Block 2323, `EWP` bzw. `?tcp.port` den Wunsch
    // des Anwenders. EE_USE_DHCP (104) war dagegen nie beschrieben — der Port
    // bleibt deshalb erhalten, nur der Rest bekommt die a-culfw-Defaults.
    const uint8_t keep[2] = {ee.read_byte(ee::IP4_TCPLINK_PORT),
                             ee.read_byte(ee::IP4_TCPLINK_PORT + 1)};
    const unsigned port = static_cast<unsigned>(keep[0] | (keep[1] << 8));
    const bool port_valid = port != 0 && port != 0xFFFF;
    ethernet_reset(ee);
    if (port_valid) ee.write_block(ee::IP4_TCPLINK_PORT, keep);
    ESP_LOGI(TAG, "network block was blank -> a-culfw ethernet_reset() defaults (tcp port %u %s)",
             port_valid ? port : static_cast<unsigned>(ee::IP4_TCPLINK_PORT_DEFAULT),
             port_valid ? "kept" : "default");
}

} // namespace culfw32
