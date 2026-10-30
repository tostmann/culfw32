idf_component_register(
    SRCS         "src/led_drv.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     driver esp_timer
)
