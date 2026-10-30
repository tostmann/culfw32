idf_component_register(
    SRCS         "src/rf_stats.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     rf_protocol esp_timer
)
