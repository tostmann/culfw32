idf_component_register(
    SRCS         "src/hmuartlgw_frame.cpp"
                 "src/hmuartlgw_frontend.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     frontend rf_manager rf_protocol config_registry nvs_eeprom rf_stats esp_timer
    PRIV_REQUIRES rf_proto_asksin
)
