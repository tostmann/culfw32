idf_component_register(
    SRCS         "src/hbrfeth_listener.cpp"
                 "src/hbrfeth_virtual_transport.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     transport frontend_hmuartlgw esp_timer lwip espressif__mdns esp_app_format
)
