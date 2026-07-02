idf_component_register(
    SRCS "captive_portal.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_http_server esp_netif esp_event lwip nvs_creds json
)
