# AirPlay 1 receiver component for OE voice-device.
#
# Built from three pieces:
#   alac/   - Apple's ALAC reference decoder (Apache 2.0) + our C++→C
#             wrapper that exposes the small API rtp.c expects.
#   raop/   - philippe44's RAOP receiver vendored from squeezelite-esp32
#             (MIT). raop_sink.c rewritten for our event flow / IDF mdns.
#   .       - airplay.c is our driver glue: start/stop arbitration, route
#             PCM into audio_io_write_pcm, fire FreeRTOS events on
#             SETUP/PLAY/STOP so main.c can pause for wake-word.
#
# IMPORTANT: this component is OPT-IN. main.c calls airplay_init() only
# after WS auth succeeds, so a device that has never paired never
# advertises the _raop._tcp service.
#
# See ../../CHANGES_AIRPLAY.md in the firmware-project root for the
# full revert procedure if this experiment is rolled back.

set(SRCS
    airplay.c
    alac/alac_wrapper.cpp
    alac/ALACBitUtilities.c
    alac/ALACDecoder.cpp
    alac/EndianPortable.c
    alac/ag_dec.c
    alac/dp_dec.c
    alac/matrix_dec.c
    raop/raop.c
    raop/raop_sink.c
    raop/rtp.c
    raop/dmap_parser.c
    raop/util.c
)

set(INCLUDE_DIRS
    include
    alac
    raop
)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    PRIV_REQUIRES
        # IDF transports / RTOS
        lwip
        pthread
        freertos
        esp_timer
        esp_netif
        esp_wifi
        esp_system
        esp_event
        # util.c #includes nvs.h and esp_console.h unconditionally even
        # though it never calls anything from them — pull them in so
        # the compile doesn't error on a missing header.
        nvs_flash
        console
        # mbedTLS for the AES-CBC frame decrypt + the Apple RSA key wrap.
        # raop.c is hard-coded to mbedtls in the non-WIN32 path.
        mbedtls
        # mDNS announce of _raop._tcp + search for _dacp._tcp.
        espressif__mdns
        # Audio sink — raop's data callback resamples + pushes into here.
        audio_io
        # Amplifier control — airplay owns enable/disable for its own
        # sessions, since TTS / alarm / ambient leave the amp in their
        # own state and the first AirPlay session after any of those
        # would otherwise play into a muted speaker.
        xvf3800_ctrl
)

# raop.c trips -Wmisleading-indentation in places that are not actually
# bugs (the upstream pulls in mixed tab/space style). Tolerate it.
set_source_files_properties(raop/raop.c
    PROPERTIES COMPILE_FLAGS
    "-Wno-misleading-indentation -Wno-format -Wno-unused-variable -Wno-unused-but-set-variable"
)
# Apple ALAC sources also have legacy warning patterns. Silence to keep
# the build readable without modifying the upstream code.
set_source_files_properties(
    alac/ALACDecoder.cpp
    alac/ag_dec.c
    alac/dp_dec.c
    alac/matrix_dec.c
    alac/ALACBitUtilities.c
    PROPERTIES COMPILE_FLAGS
    "-Wno-unused-variable -Wno-unused-but-set-variable -Wno-implicit-fallthrough -Wno-sign-compare"
)
