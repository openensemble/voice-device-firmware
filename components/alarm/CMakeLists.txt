idf_component_register(
    SRCS "alarm.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash esp_timer json oe_client audio_io mp3_decode esp_http_client spiffs
)
