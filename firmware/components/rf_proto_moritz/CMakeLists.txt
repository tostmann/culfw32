idf_component_register(
    SRCS         "src/moritz_cfg.cpp"
                 "src/moritz_rf.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     rf_protocol rf_stats cc1101_driver freertos esp_timer
)
