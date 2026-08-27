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
// Canonical verifier slug derived from manifest.wake_word. OE and firmware use
// the same contract: lowercase ASCII alphanumeric tokens joined by underscores,
// 39 characters maximum plus NUL.
#define WW_WAKE_SLUG_MAX 40

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

// Copy the canonical manifest-derived slug belonging to the most recent
// detection. This is snapshotted while the model mutex is held, so a concurrent
// slot hot-swap cannot relabel an already-fired wake. Returns false if no valid
// detection identity is available or the destination is too small.
bool wakeword_last_wake_slug(const wakeword_t *ww, char *out, size_t out_len);

void wakeword_notify_speaking_began(wakeword_t *ww);
void wakeword_notify_speaking_ended(wakeword_t *ww);

// Detector telemetry for one observation window, used to localize a "no wake"
// failure along the capture path:
//   audio_lvl     — peak |int16 sample| fed to the detector. 0 means I²S
//                   delivered silence; a small non-zero value under normal
//                   speech means the mic path is alive but starved (the
//                   AGC-freeze failure mode: gain locked too low, so the
//                   frontend never sees enough signal to build a confident
//                   score).
//   feat_max      — peak INT8 feature after the frontend. -128 = nothing but
//                   silence is reaching the model.
//   peak_avg_prob — highest sliding-window probability observed. This is the
//                   near-miss metric and the reason this accessor exists:
//                   wake_avg_prob only reaches the server when a fire clears
//                   the cutoff, so every sub-cutoff attempt — exactly the
//                   population you need to see when tuning sensitivity — is
//                   otherwise unobservable off-device.
//   slices        — feature slices processed, to confirm the window had data.
//   scored/gated — how many probability windows the detector actually scored
//                  vs. skipped because it was disabled or inside its
//                  post-detection ignore window. determine_detected() reports
//                  average_probability = 0 in the skipped case, so without
//                  this split a peak of 0 cannot be read: "heard nothing" and
//                  "wasn't scoring yet" produce the identical number.
typedef struct {
    int16_t  audio_lvl;
    int8_t   feat_max;
    uint8_t  peak_avg_prob;
    uint32_t slices;
    uint32_t scored_windows;
    uint32_t gated_windows;
} wakeword_stats_t;

// Snapshot the telemetry window and reset it. The caller owns the window
// cadence — nothing else resets these counters, so two concurrent readers
// would each see only a partial window. Mutex-safe against wakeword_feed.
void wakeword_read_stats(wakeword_t *ww, wakeword_stats_t *out);

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
