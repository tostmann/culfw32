idf_component_register(
    SRCS         "src/transport_usbcdc.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     transport driver esp_driver_usb_serial_jtag freertos
)
