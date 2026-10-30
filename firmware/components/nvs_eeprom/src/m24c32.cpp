// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#include "m24c32.hpp"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include <algorithm>
#include <cstring>

namespace culfw32 {

namespace {
constexpr const char* TAG = "m24c32";
constexpr int kXferTimeoutMs = 100;
constexpr int64_t kReadyTimeoutUs = 20000;   // tW max 5 ms (S. 30), Reserve fuer Bus-Stoerungen
constexpr size_t kReadChunk = 128;
}

esp_err_t M24c32::init(int sda, int scl, int wc, uint8_t addr, uint32_t hz) {
    addr_ = addr;
    wc_   = wc;

    // WC zuerst auf high (gesperrt), dann den Ausgang einschalten.
    if (wc_ >= 0) {
        gpio_set_level(static_cast<gpio_num_t>(wc_), 1);
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << wc_,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        if (auto e = gpio_config(&io); e != ESP_OK) return e;
        gpio_set_level(static_cast<gpio_num_t>(wc_), 1);
    }

    i2c_master_bus_config_t bc = {};
    bc.i2c_port          = -1;
    bc.sda_io_num        = static_cast<gpio_num_t>(sda);
    bc.scl_io_num        = static_cast<gpio_num_t>(scl);
    bc.clk_source        = I2C_CLK_SRC_DEFAULT;
    bc.glitch_ignore_cnt = 7;
    bc.flags.enable_internal_pullup = 1;   // zusaetzlich zu den Pull-ups auf der Platine
    i2c_master_bus_handle_t bus = nullptr;
    if (auto e = i2c_new_master_bus(&bc, &bus); e != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus: %s", esp_err_to_name(e));
        return e;
    }
    if (auto e = i2c_master_probe(bus, addr_, kXferTimeoutMs); e != ESP_OK) {
        i2c_del_master_bus(bus);
        return ESP_ERR_NOT_FOUND;
    }
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address  = addr_;
    dc.scl_speed_hz    = hz;
    i2c_master_dev_handle_t dev = nullptr;
    if (auto e = i2c_master_bus_add_device(bus, &dc, &dev); e != ESP_OK) {
        i2c_del_master_bus(bus);
        return e;
    }
    bus_ = bus;
    dev_ = dev;
    return ESP_OK;
}

M24c32::~M24c32() {
    if (dev_) i2c_master_bus_rm_device(static_cast<i2c_master_dev_handle_t>(dev_));
    if (bus_) i2c_del_master_bus(static_cast<i2c_master_bus_handle_t>(bus_));
    dev_ = bus_ = nullptr;
}

esp_err_t M24c32::read(uint16_t addr, uint8_t* out, size_t len) {
    if (!dev_) return ESP_ERR_INVALID_STATE;
    if (addr + len > kSize) return ESP_ERR_INVALID_ARG;
    auto dev = static_cast<i2c_master_dev_handle_t>(dev_);
    for (size_t done = 0; done < len;) {
        const size_t   n = std::min(kReadChunk, len - done);
        const uint16_t a = static_cast<uint16_t>(addr + done);
        const uint8_t  ab[2] = {static_cast<uint8_t>(a >> 8), static_cast<uint8_t>(a & 0xFF)};
        if (auto e = i2c_master_transmit_receive(dev, ab, sizeof(ab), out + done, n, kXferTimeoutMs);
            e != ESP_OK) {
            return e;
        }
        done += n;
    }
    return ESP_OK;
}

esp_err_t M24c32::wait_ready_() {
    // ACK-Polling (S. 18): solange der interne Schreibzyklus laeuft, quittiert
    // der Baustein seine Adresse nicht. probe() meldet das still als NOT_FOUND.
    const int64_t t0 = esp_timer_get_time();
    auto bus = static_cast<i2c_master_bus_handle_t>(bus_);
    while (i2c_master_probe(bus, addr_, kXferTimeoutMs) != ESP_OK) {
        if (esp_timer_get_time() - t0 > kReadyTimeoutUs) return ESP_ERR_TIMEOUT;
        esp_rom_delay_us(200);
    }
    return ESP_OK;
}

esp_err_t M24c32::write(uint16_t addr, const uint8_t* data, size_t len) {
    if (!dev_) return ESP_ERR_INVALID_STATE;
    if (addr + len > kSize) return ESP_ERR_INVALID_ARG;
    auto dev = static_cast<i2c_master_dev_handle_t>(dev_);

    if (wc_ >= 0) gpio_set_level(static_cast<gpio_num_t>(wc_), 0);
    esp_err_t e = ESP_OK;
    for (size_t done = 0; done < len && e == ESP_OK;) {
        const uint16_t a = static_cast<uint16_t>(addr + done);
        const size_t   n = std::min<size_t>(kPage - (a % kPage), len - done);   // nie ueber die Seitengrenze
        uint8_t buf[2 + kPage];
        buf[0] = static_cast<uint8_t>(a >> 8);
        buf[1] = static_cast<uint8_t>(a & 0xFF);
        std::memcpy(buf + 2, data + done, n);
        e = i2c_master_transmit(dev, buf, 2 + n, kXferTimeoutMs);
        if (e == ESP_OK) e = wait_ready_();
        done += n;
    }
    if (wc_ >= 0) gpio_set_level(static_cast<gpio_num_t>(wc_), 1);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "write 0x%04x+%u: %s", addr, static_cast<unsigned>(len), esp_err_to_name(e));
        return e;
    }

    // Zuruecklesen: ein nicht quittiertes Datenbyte (WC haengt high, Stoerung)
    // faellt sonst erst beim naechsten Boot auf.
    uint8_t back[kReadChunk];
    for (size_t done = 0; done < len;) {
        const size_t n = std::min(kReadChunk, len - done);
        if (e = read(static_cast<uint16_t>(addr + done), back, n); e != ESP_OK) return e;
        if (std::memcmp(back, data + done, n) != 0) {
            ESP_LOGE(TAG, "verify 0x%04x+%u failed", static_cast<unsigned>(addr + done),
                     static_cast<unsigned>(n));
            return ESP_ERR_INVALID_CRC;
        }
        done += n;
    }
    return ESP_OK;
}

} // namespace culfw32
