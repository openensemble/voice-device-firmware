#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

typedef void (*mp3_pcm_callback_t)(const int16_t *pcm, size_t samples, uint32_t sample_rate, void *user);

typedef struct mp3_dec_s mp3_dec_t;

mp3_dec_t *mp3_dec_create(mp3_pcm_callback_t cb, void *user);
void       mp3_dec_destroy(mp3_dec_t *d);

esp_err_t mp3_dec_feed(mp3_dec_t *d, const uint8_t *bytes, size_t n);
void      mp3_dec_flush(mp3_dec_t *d);

// Module-wide decode-error counter (monotonic, all decoders contribute).
// Used by the heartbeat ambient-telemetry log to compute per-interval
// decode error rate without instrumenting every decode call site.
uint32_t mp3_dec_get_total_errors(void);
