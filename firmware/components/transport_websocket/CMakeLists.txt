idf_component_register(
    SRCS         "src/transport_websocket.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     transport transport_txq esp_http_server freertos
)
