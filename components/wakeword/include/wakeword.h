#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WW_SAMPLE_RATE 16000
// Capture frame size used by main.c (80 ms at 16 kHz). The audio frontend
// inside wakeword.cpp is fed arbitrary chunks and emits 30 ms feature slices
// every `feature_step_ms` (10 ms by default, matching microWakeWord v2
// training). 1280 samples per call yields ~8 slices per call.
#define WW_FRAME_SAMPLES 1280

typedef struct wakeword_s wakeword_t;

typedef struct {
    uint8_t slot;
    float threshold;
    uint32_t cooldown_ms;
    uint32_t refractory_after_speak_ms;
} wakeword_config_t;

esp_err_t wakeword_mount_partition(const char *partition_label, const char *base_path);

wakeword_t *wakeword_create(const wakeword_config_t *cfg);
void wakeword_destroy(wakeword_t *ww);

esp_err_t wakeword_load_slot(wakeword_t *ww, uint8_t slot);
uint8_t   wakeword_active_slot(const wakeword_t *ww);

// Tear down the currently-loaded model and free its arena WITHOUT destroying
// the wakeword_t. After this the detector is inert (wakeword_feed returns
// false) until a later wakeword_load_slot repopulates it. Used when the
// server clears a slot whose user was removed from the voice config — the
// wakeword_t is kept so the slot can be reused without a reboot. Mutex-safe
// against a concurrent wakeword_feed, same as wakeword_load_slot.
void wakeword_unload_slot(wakeword_t *ww);

bool wakeword_feed(wakeword_t *ww, const int16_t *samples, size_t n_samples);

// Average probability (0..255) of the most recent detection that caused
// wakeword_feed to return true. Reset each detection; meaningless to read
// before any wake has fired. Used by main.c to resolve which slot wins when
// two overlapping wake-word phrases fire in the same or adjacent frames.
uint8_t wakeword_last_wake_prob(const wakeword_t *ww);

void wakeword_notify_speaking_began(wakeword_t *ww);
void wakeword_notify_speaking_ended(wakeword_t *ww);

// Runtime probability-cutoff tuning. When audio is playing through the I²S
// TX (TTS / AirPlay / ambient), per-frame inference probability builds
// slower because the active TX line degrades the RX signal on this board.
// Lowering the cutoff during playback restores first-try wake latency.
// Restore via the default (read once at slot init).
uint8_t wakeword_get_default_cutoff(const wakeword_t *ww);
void    wakeword_set_cutoff(wakeword_t *ww, uint8_t cutoff);

#ifdef __cplusplus
}
#endif
