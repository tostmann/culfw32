idf_component_register(
    SRCS         "src/slowrf_cfg.cpp"
                 "src/slowrf_rf.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     rf_protocol rf_stats cc1101_driver freertos esp_timer driver
)
