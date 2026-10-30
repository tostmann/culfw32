idf_component_register(
    SRCS         "src/tcp_server.cpp"
                 "src/tcp_client_transport.cpp"
                 "src/tcp_rx_pool.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     transport transport_txq esp_netif lwip freertos
)
