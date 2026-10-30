idf_component_register(
    SRCS     "main.cpp"
    REQUIRES rf_manager freq_cal frontend_culfw frontend_json frontend_signalduino frontend_hmuartlgw transport_usbcdc transport_uart transport_tcpserver transport_hbrfeth_udp transport_websocket web_server nvs_eeprom led_drv wifi_drv eth_drv ota_update config_registry rf_stats system_monitor rf_proto_moritz rf_proto_asksin rf_proto_slowrf rf_proto_signalduino rf_proto_native rf_proto_fastrf rf_proto_dudette rf_proto_wmbus rf_proto_hmip improv_wifi_busware esp_driver_usb_serial_jtag esp_timer
)

# C++20 fuer std::span. ESP-IDF 5.x default ist gnu++17 — explizit hochziehen.
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++20)
