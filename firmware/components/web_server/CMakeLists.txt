idf_component_register(
    SRCS         "src/web_server.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     esp_http_server transport_websocket
    EMBED_FILES
        "../../web/index.html.gz"
        "../../web/app.css.gz"
        "../../web/app.js.gz"
        "../../web/assets/busware_logo.png"
)
