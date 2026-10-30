idf_component_register(
    SRCS          "src/ota_update.cpp"
    INCLUDE_DIRS  "include"
    PRIV_REQUIRES app_update esp_http_client esp_https_ota esp_timer json mbedtls
)
