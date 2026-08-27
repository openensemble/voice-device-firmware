#include "vad.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
static const char *VAD_TAG = "vad";

struct vad_state_s {
    vad_config_t cfg;
    uint32_t silence_ms;
    uint32_t total_ms;
    bool any_speech_seen;
    // Debug stats: rolling per-second max RMS so we can see what the mic
    // looks like during "silence" and tune energy_threshold accordingly.
    uint32_t stats_window_ms;
    uint32_t stats_max_rms;
    // Per-utterance accounting, cleared by vad_reset. Reported at the end of
    // every utterance so a truncated turn carries the evidence for WHY it
    // ended rather than just the fact that it did.
    uint32_t utt_peak_energy;
    uint32_t utt_speech_ms;
};

vad_state_t *vad_create(const vad_config_t *cfg)
{
    vad_state_t *v = calloc(1, sizeof(*v));
    if (!v) return NULL;
    v->cfg = *cfg;
    return v;
}

void vad_destroy(vad_state_t *vad) { free(vad); }

void vad_reset(vad_state_t *vad)
{
    vad->silence_ms = 0;
    vad->total_ms = 0;
    vad->any_speech_seen = false;
    vad->utt_peak_energy = 0;
    vad->utt_speech_ms = 0;
}

void vad_set_energy_threshold(vad_state_t *vad, uint32_t threshold)
{
    if (!vad || threshold == 0) return;
    vad->cfg.energy_threshold = threshold;
}

void vad_get_utterance_stats(const vad_state_t *vad, vad_utterance_stats_t *out)
{
    if (!vad || !out) return;
    out->peak_energy = vad->utt_peak_energy;
    out->threshold   = vad->cfg.energy_threshold;
    out->total_ms    = vad->total_ms;
    out->speech_ms   = vad->utt_speech_ms;
}

const char *vad_end_reason_name(vad_end_reason_t r)
{
    switch (r) {
        case VAD_END_SILENCE:       return "silence";
        case VAD_END_NO_SPEECH:     return "no_speech";
        case VAD_END_MAX_UTTERANCE: return "max_utterance";
        default:                    return "none";
    }
}

bool vad_feed(vad_state_t *vad, const int16_t *samples, size_t n_samples, vad_end_reason_t *end_reason)
{
    *end_reason = VAD_END_NONE;
    if (n_samples == 0) return false;

    uint64_t sum_sq = 0;
    for (size_t i = 0; i < n_samples; ++i) {
        int32_t s = samples[i];
        sum_sq += (uint64_t)(s * s);
    }
    uint32_t rms = (uint32_t)(sum_sq / n_samples);
    uint32_t chunk_ms = (uint32_t)((n_samples * 1000ULL) / vad->cfg.sample_rate);
    vad->total_ms += chunk_ms;

    if (rms > vad->utt_peak_energy) vad->utt_peak_energy = rms;

    bool is_speech = rms > vad->cfg.energy_threshold;
    if (is_speech) {
        vad->any_speech_seen = true;
        vad->silence_ms = 0;
        vad->utt_speech_ms += chunk_ms;
    } else {
        vad->silence_ms += chunk_ms;
    }

    if (vad->any_speech_seen && vad->silence_ms >= vad->cfg.silence_ms_to_end) {
        *end_reason = VAD_END_SILENCE;
    } else if (!vad->any_speech_seen &&
               vad->cfg.no_speech_ms_to_end > 0 &&
               vad->total_ms >= vad->cfg.no_speech_ms_to_end) {
        *end_reason = VAD_END_NO_SPEECH;
    } else if (vad->total_ms >= vad->cfg.max_utterance_ms) {
        *end_reason = vad->any_speech_seen ? VAD_END_MAX_UTTERANCE : VAD_END_NO_SPEECH;
    }

    // Per-second rolling max RMS so the tuner can see what their room
    // actually produces. "rms=X thr=Y speech=Z" lines line up with the
    // is_speech path the VAD picked. Quiet rooms should print <50k;
    // any line consistently >threshold during the silent tail of an
    // utterance means the threshold needs to go up.
    if (rms > vad->stats_max_rms) vad->stats_max_rms = rms;
    vad->stats_window_ms += chunk_ms;
    if (vad->stats_window_ms >= 1000) {
        ESP_LOGI(VAD_TAG, "rms_max_1s=%u thr=%u silence=%ums any_speech=%d",
                 (unsigned) vad->stats_max_rms,
                 (unsigned) vad->cfg.energy_threshold,
                 (unsigned) vad->silence_ms,
                 (int) vad->any_speech_seen);
        vad->stats_max_rms = 0;
        vad->stats_window_ms = 0;
    }
    return is_speech;
}

uint32_t vad_elapsed_ms(const vad_state_t *vad) { return vad->total_ms; }
