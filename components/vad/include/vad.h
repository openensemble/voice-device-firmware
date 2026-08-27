#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    uint32_t energy_threshold;
    uint32_t silence_ms_to_end;
    uint32_t no_speech_ms_to_end;
    uint32_t max_utterance_ms;
    uint32_t sample_rate;
} vad_config_t;

typedef enum {
    VAD_END_NONE = 0,
    VAD_END_SILENCE,
    VAD_END_NO_SPEECH,
    VAD_END_MAX_UTTERANCE,
} vad_end_reason_t;

typedef struct vad_state_s vad_state_t;

// Per-utterance accounting, valid from vad_reset until the next one. Exists to
// answer "why did the turn end there?" after the fact:
//   peak_energy — highest per-frame mean-square seen this utterance. Compare
//                 against `threshold`: a ratio near 1 means the speech barely
//                 registered as speech at all, so a normal mid-sentence pause
//                 trips silence_ms_to_end and the user gets cut off. Because
//                 the threshold is ABSOLUTE, a mic gain that drifts or freezes
//                 low walks the whole utterance toward that floor.
//   speech_ms   — milliseconds of frames above the threshold. Much smaller
//                 than total_ms means most of what the user said was scored as
//                 silence, which is the same failure wearing a different hat.
typedef struct {
    uint32_t peak_energy;
    uint32_t threshold;
    uint32_t total_ms;
    uint32_t speech_ms;
} vad_utterance_stats_t;

vad_state_t *vad_create(const vad_config_t *cfg);
void vad_destroy(vad_state_t *vad);
void vad_reset(vad_state_t *vad);

bool vad_feed(vad_state_t *vad, const int16_t *samples, size_t n_samples, vad_end_reason_t *end_reason);
uint32_t vad_elapsed_ms(const vad_state_t *vad);
void vad_get_utterance_stats(const vad_state_t *vad, vad_utterance_stats_t *out);
// Retune the speech floor between utterances. The threshold is what separates
// "still talking" from "done", so it must never move mid-utterance — call this
// alongside vad_reset, not during a capture.
void vad_set_energy_threshold(vad_state_t *vad, uint32_t threshold);
// Stable short names for logs: "silence" | "no_speech" | "max_utterance".
const char *vad_end_reason_name(vad_end_reason_t r);
