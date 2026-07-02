idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        audio_io
        xvf3800_ctrl
        wakeword
        vad
        mp3_decode
        oe_client
        captive_portal
        nvs_creds
        leds_buttons
        alarm
        airplay
        mbedtls
    EMBED_FILES
        "xvf_firmware/xvf_ha_v1_0_7.bin"
)
