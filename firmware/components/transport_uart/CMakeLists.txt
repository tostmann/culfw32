idf_component_register(
    SRCS         "src/transport_uart.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     transport driver esp_driver_uart freertos
)
