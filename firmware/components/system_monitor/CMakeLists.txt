idf_component_register(
    SRCS         "src/system_monitor.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     esp_timer nvs_flash esp_system esp_hw_support config_registry
    PRIV_REQUIRES esp_driver_tsens
)
