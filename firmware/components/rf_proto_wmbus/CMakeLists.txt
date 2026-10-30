idf_component_register(
    SRCS         "src/wmbus_codec.cpp"
                 "src/wmbus_cfg.cpp"
                 "src/wmbus_rf.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     rf_protocol rf_stats cc1101_driver freertos esp_timer
)
