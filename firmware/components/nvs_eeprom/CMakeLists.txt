idf_component_register(
    SRCS          "src/nvs_eeprom.cpp" "src/m24c32.cpp"
    INCLUDE_DIRS  "include"
    REQUIRES      nvs_flash
    PRIV_REQUIRES esp_driver_i2c esp_driver_gpio esp_timer
)
