idf_component_register(
    SRCS         "src/wifi_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     esp_wifi esp_netif esp_event nvs_flash esp_timer espressif__mdns
)
