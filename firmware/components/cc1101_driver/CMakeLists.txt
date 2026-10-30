idf_component_register(
    SRCS         "src/cc1101_driver.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     driver esp_driver_spi esp_driver_gpio
)
