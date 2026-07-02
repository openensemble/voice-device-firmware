idf_component_register(
    SRCS
        "wakeword.cpp"
        "upstream/streaming_model.cpp"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "upstream"
    REQUIRES
        esp-tflite-micro
        esp-micro-speech-features
        spiffs
        esp_timer
        json
)

# C++17 required for std::make_unique with new<nothrow>-style construction
# in streaming_model.cpp + std::unique_ptr-with-deleter in wakeword.cpp.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-deprecated-declarations)
set_source_files_properties(
    "wakeword.cpp"
    "upstream/streaming_model.cpp"
    PROPERTIES COMPILE_FLAGS "-std=gnu++17"
)
