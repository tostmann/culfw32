idf_component_register(
    SRCS         "src/frontend_signalduino.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     frontend rf_manager rf_protocol config_registry esp_timer
    PRIV_REQUIRES rf_proto_signalduino
)
