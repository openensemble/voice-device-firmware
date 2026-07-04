cmake_minimum_required(VERSION 3.16)

# Bumping this is the canonical place to declare a new firmware version.
# IDF reads PROJECT_VER into the esp_app_desc baked into oe_voice_device.bin;
# the device reports it via esp_app_get_description()->version on WS auth so
# the server can decide whether an OTA push is needed. Keep in sync with
# public/firmware/voice-device/manifest.json "version" on the OE side.
set(PROJECT_VER "0.2.73-waithint")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(oe_voice_device VERSION 0.2.13)

# Generate the wakewords SPIFFS partition image from the ./wakewords/ dir.
# Contents (stock defaults — a fresh flash ships these; OE replaces them
# per-user over the WS ww_upload path once the device pairs):
#   slot0.tflite  - "Hey Ensemble"  (microWakeWord v2, ~61 KB)
#   slot1.tflite  - "Hey Computer"  (microWakeWord v2, ~61 KB)
#   slotN.json    - per-model manifest (parsed by wakeword.cpp for the
#                   probability_cutoff / sliding_window_size / tensor_arena
#                   values; if missing the okay_nabu defaults are used).
#
# This runs at build time; `idf.py flash` writes the resulting binary to
# the `wakewords` partition declared in partitions.csv.
spiffs_create_partition_image(wakewords ${CMAKE_SOURCE_DIR}/wakewords FLASH_IN_PROJECT)
