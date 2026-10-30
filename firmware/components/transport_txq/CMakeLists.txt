idf_component_register(
    SRCS         "src/tx_queue.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     freertos
    PRIV_REQUIRES lwip esp_timer
)
