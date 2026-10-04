idf_component_register(
    SRCS "http_server.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_http_server led_ctrl
    EMBED_FILES "index.html"
)