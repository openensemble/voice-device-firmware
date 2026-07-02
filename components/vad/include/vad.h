#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    uint32_t energy_threshold;
    uint32_t silence_ms_to_end;
    uint32_t max_utterance_ms;
    uint32_t sample_rate;
} vad_config_t;

typedef struct vad_state_s vad_state_t;

vad_state_t *vad_create(const vad_config_t *cfg);
void vad_destroy(vad_state_t *vad);
void vad_reset(vad_state_t *vad);

bool vad_feed(vad_state_t *vad, const int16_t *samples, size_t n_samples, bool *utterance_ended);
uint32_t vad_elapsed_ms(const vad_state_t *vad);
