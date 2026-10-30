idf_component_register(
    SRCS          "src/eth_manager.cpp"
    INCLUDE_DIRS  "include"
    REQUIRES      esp_event esp_netif nvs_eeprom
    PRIV_REQUIRES esp_eth esp_driver_spi esp_driver_gpio efuse lwip
)
