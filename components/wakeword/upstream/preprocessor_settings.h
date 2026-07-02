// Vendored from esphome/components/micro_wake_word/preprocessor_settings.h
// Source commit: esphome/esphome@dev (2026-05-11). Apache 2.0.
// Stripped: USE_ESP32 guard, esphome:: namespace.
//
// Constants for the spectrogram frontend. These must match how the wake-word
// model was trained — all microWakeWord v2 models use these parameters.

#pragma once

#include <cstdint>

// Number of features per slice produced by the audio frontend.
static const uint8_t PREPROCESSOR_FEATURE_SIZE = 40;
// Window duration (one slice = one window of audio).
static const uint8_t FEATURE_DURATION_MS = 30;

static const float FILTERBANK_LOWER_BAND_LIMIT = 125.0;
static const float FILTERBANK_UPPER_BAND_LIMIT = 7500.0;

static const uint8_t NOISE_REDUCTION_SMOOTHING_BITS = 10;
static const float NOISE_REDUCTION_EVEN_SMOOTHING = 0.025;
static const float NOISE_REDUCTION_ODD_SMOOTHING = 0.06;
static const float NOISE_REDUCTION_MIN_SIGNAL_REMAINING = 0.05;

static const bool PCAN_GAIN_CONTROL_ENABLE_PCAN = true;
static const float PCAN_GAIN_CONTROL_STRENGTH = 0.95;
static const float PCAN_GAIN_CONTROL_OFFSET = 80.0;
static const uint8_t PCAN_GAIN_CONTROL_GAIN_BITS = 21;

static const bool LOG_SCALE_ENABLE_LOG = true;
static const uint8_t LOG_SCALE_SCALE_SHIFT = 6;
