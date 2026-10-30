idf_component_register(
    SRCS         "src/frontend_json.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     frontend rf_manager rf_protocol config_registry json esp_timer
    PRIV_REQUIRES rf_proto_moritz rf_proto_asksin rf_proto_slowrf rf_proto_fastrf
)
