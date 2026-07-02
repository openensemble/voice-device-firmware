// Wake-word detector glue for the OE voice-device firmware.
//
// Wires together:
//   - esphome/esp-micro-speech-features  (FrontendConfig/FrontendState +
//     FrontendProcessSamples → 40 mel features per 30 ms window)
//   - the vendored streaming_model.{h,cpp}  (WakeWordModel: streaming TFLM
//     interpreter with sliding-window probability averaging)
//
// Public API in wakeword.h is plain C; everything below is wrapped in a
// per-instance struct (`wakeword_t`) so multiple wake-word instances are
// possible in the future (e.g. parallel slots). v1 uses one.
//
// Per-slot model loading: tflite files live in /ww/slotN.tflite on a
// dedicated SPIFFS partition (mounted by wakeword_mount_partition).
// Per-model probability cutoff + sliding window size + tensor arena size
// are read from a sibling /ww/slotN.json manifest at load time. If the
// manifest is missing or malformed we fall back to okay_nabu defaults
// (0.97 cutoff, 5-window, 26080-byte arena). Manifest schema mirrors the
// esphome/micro-wake-word-models v2 format — see slot0.okay_nabu.bak.json
// for the canonical example.

#include "wakeword.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "upstream/preprocessor_settings.h"
#include "upstream/streaming_model.h"

#include "cJSON.h"

extern "C" {
#include <frontend.h>
#include <frontend_util.h>
}

static const char *const TAG = "ww";

// microWakeWord v2 models — see esphome/micro-wake-word-models/models/v2/*.json
// These match okay_nabu / hey_jarvis / hey_mycroft manifests; if a future slot
// uses different params, swap to a per-slot manifest read.
static const uint8_t  WW_DEFAULT_PROBABILITY_CUTOFF = 247;   // ≈ 0.97 * 255
static const size_t   WW_DEFAULT_SLIDING_WINDOW     = 5;
static const size_t   WW_DEFAULT_TENSOR_ARENA       = 26080;
static const uint8_t  WW_DEFAULT_FEATURE_STEP_MS    = 10;

struct wakeword_s {
    wakeword_config_t           cfg;
    uint8_t                     active_slot;

    // Streaming wake-word model + model bytes lifetime-tied to it.
    std::unique_ptr<WakeWordModel> model;
    uint8_t                     *model_buf;
    size_t                       model_len;

    // Audio frontend (windowing + mel filterbank + PCAN + log + INT8 quant).
    FrontendConfig               frontend_cfg;
    FrontendState                frontend_state;
    bool                         frontend_ready;

    // Speaking-state gate. main.c calls notify_speaking_began/ended around
    // TTS playback so we don't re-trigger on our own speaker output. A short
    // refractory after speak end avoids spurious wakes from speaker decay.
    bool                         speaking;
    int64_t                      speaking_ended_us;
    int64_t                      last_detection_us;
    uint8_t                      last_wake_avg_prob;

    // Bring-up diagnostics: periodically dump audio-path activity. Layered
    // so we can localize a "no wake" failure:
    //   raw_max_abs  — max |int16 sample| in input buffers (0 → I²S dead)
    //   feature_max  — max INT8 feature post-frontend (-128 → silence)
    //   last_avg_prob — last sliding-window probability
    // Initial extremes use INT8_MAX / INT8_MIN so the first reading shows
    // the real range; the 5 s dump resets back to those sentinels.
    int64_t                      stats_last_us;
    uint32_t                     stats_slices;
    uint32_t                     stats_calls;
    int16_t                      stats_raw_max_abs;
    int8_t                       stats_max_feature;
    int8_t                       stats_min_feature;
    uint8_t                      stats_last_avg_prob;

    // Guards the model pointer + arena lifetime across the hot-swap path.
    // wakeword_load_slot (called from the WS task on ww_upload) tears down
    // the old MicroInterpreter and allocates a new one; wakeword_feed
    // (called every 80 ms from the audio task) reads the same pointers.
    // Without this lock a swap can free the interpreter mid-Invoke and the
    // feed task crashes with LoadProhibited inside InvokeSubgraph.
    SemaphoreHandle_t            model_mutex;
};

esp_err_t wakeword_mount_partition(const char *partition_label, const char *base_path) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = base_path,
        .partition_label = partition_label,
        .max_files = 5,
        .format_if_mount_failed = false,
    };
    esp_err_t e = esp_vfs_spiffs_register(&conf);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "spiffs mount '%s' -> '%s' failed: %s",
                 partition_label, base_path, esp_err_to_name(e));
    } else {
        size_t total = 0, used = 0;
        esp_spiffs_info(partition_label, &total, &used);
        ESP_LOGI(TAG, "wakewords spiffs: total=%u used=%u",
                 (unsigned) total, (unsigned) used);
    }
    return e;
}

static void populate_frontend_config(FrontendConfig &cfg) {
    cfg.window.size_ms = FEATURE_DURATION_MS;
    cfg.window.step_size_ms = WW_DEFAULT_FEATURE_STEP_MS;
    cfg.filterbank.num_channels = PREPROCESSOR_FEATURE_SIZE;
    cfg.filterbank.lower_band_limit = FILTERBANK_LOWER_BAND_LIMIT;
    cfg.filterbank.upper_band_limit = FILTERBANK_UPPER_BAND_LIMIT;
    cfg.noise_reduction.smoothing_bits = NOISE_REDUCTION_SMOOTHING_BITS;
    cfg.noise_reduction.even_smoothing = NOISE_REDUCTION_EVEN_SMOOTHING;
    cfg.noise_reduction.odd_smoothing = NOISE_REDUCTION_ODD_SMOOTHING;
    cfg.noise_reduction.min_signal_remaining = NOISE_REDUCTION_MIN_SIGNAL_REMAINING;
    cfg.pcan_gain_control.enable_pcan = PCAN_GAIN_CONTROL_ENABLE_PCAN;
    cfg.pcan_gain_control.strength = PCAN_GAIN_CONTROL_STRENGTH;
    cfg.pcan_gain_control.offset = PCAN_GAIN_CONTROL_OFFSET;
    cfg.pcan_gain_control.gain_bits = PCAN_GAIN_CONTROL_GAIN_BITS;
    cfg.log_scale.enable_log = LOG_SCALE_ENABLE_LOG;
    cfg.log_scale.scale_shift = LOG_SCALE_SCALE_SHIFT;
}

wakeword_t *wakeword_create(const wakeword_config_t *cfg) {
    auto ww = new (std::nothrow) wakeword_s();
    if (!ww) return nullptr;
    ww->cfg = *cfg;
    ww->active_slot = 0xFF;

    populate_frontend_config(ww->frontend_cfg);
    if (!FrontendPopulateState(&ww->frontend_cfg, &ww->frontend_state, WW_SAMPLE_RATE)) {
        ESP_LOGE(TAG, "FrontendPopulateState failed");
        delete ww;
        return nullptr;
    }
    ww->frontend_ready = true;
    ww->stats_max_feature = INT8_MIN;
    ww->stats_min_feature = INT8_MAX;
    ww->model_mutex = xSemaphoreCreateMutex();
    if (!ww->model_mutex) {
        ESP_LOGE(TAG, "model_mutex alloc failed");
        delete ww;
        return nullptr;
    }
    return ww;
}

void wakeword_destroy(wakeword_t *ww) {
    if (!ww) return;
    if (ww->model) {
        ww->model->unload_model();
        ww->model.reset();
    }
    if (ww->model_buf) heap_caps_free(ww->model_buf);
    if (ww->frontend_ready) FrontendFreeStateContents(&ww->frontend_state);
    if (ww->model_mutex) vSemaphoreDelete(ww->model_mutex);
    delete ww;
}

// Per-slot manifest values, populated by read_slot_manifest. Mirrors the
// fields in slot<N>.json's "micro" object. All fields fall back to the
// hardcoded defaults if the manifest is missing or malformed.
struct slot_manifest_t {
    uint8_t probability_cutoff;   // uint8 in 0..255 (= float cutoff * 255 + 0.5)
    size_t  sliding_window_size;
    size_t  tensor_arena_size;
};

static void read_slot_manifest(uint8_t slot, slot_manifest_t *m) {
    // Defaults match okay_nabu (matches the comment block at file head).
    m->probability_cutoff = WW_DEFAULT_PROBABILITY_CUTOFF;
    m->sliding_window_size = WW_DEFAULT_SLIDING_WINDOW;
    m->tensor_arena_size  = WW_DEFAULT_TENSOR_ARENA;

    char path[64];
    snprintf(path, sizeof(path), "/ww/slot%u.json", (unsigned) slot);
    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGW(TAG, "no manifest at %s, using defaults", path);
        return;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len <= 0 || len > 4096) {
        fclose(f);
        ESP_LOGW(TAG, "manifest %s size %ld out of range, using defaults", path, len);
        return;
    }
    char *buf = (char *) malloc(len + 1);
    if (!buf) { fclose(f); return; }
    size_t r = fread(buf, 1, len, f);
    fclose(f);
    if (r != (size_t) len) { free(buf); return; }
    buf[len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        ESP_LOGW(TAG, "manifest %s parse failed, using defaults", path);
        return;
    }
    cJSON *micro = cJSON_GetObjectItem(root, "micro");
    if (cJSON_IsObject(micro)) {
        cJSON *pc = cJSON_GetObjectItem(micro, "probability_cutoff");
        if (cJSON_IsNumber(pc)) {
            double v = pc->valuedouble;
            if (v < 0.0) v = 0.0;
            if (v > 1.0) v = 1.0;
            m->probability_cutoff = (uint8_t)(v * 255.0 + 0.5);
        }
        cJSON *sw = cJSON_GetObjectItem(micro, "sliding_window_size");
        if (cJSON_IsNumber(sw) && sw->valueint > 0) {
            m->sliding_window_size = (size_t) sw->valueint;
        }
        cJSON *ta = cJSON_GetObjectItem(micro, "tensor_arena_size");
        if (cJSON_IsNumber(ta) && ta->valueint > 0) {
            m->tensor_arena_size = (size_t) ta->valueint;
        }
    }
    cJSON_Delete(root);
    ESP_LOGI(TAG, "slot %u manifest: cutoff=%u/255 window=%u arena=%u",
             slot,
             (unsigned) m->probability_cutoff,
             (unsigned) m->sliding_window_size,
             (unsigned) m->tensor_arena_size);
}

static esp_err_t load_model_file(uint8_t slot, uint8_t **out_buf, size_t *out_len) {
    char path[64];
    snprintf(path, sizeof(path), "/ww/slot%u.tflite", (unsigned) slot);
    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "wake-word slot %u not found at %s", slot, path);
        return ESP_ERR_NOT_FOUND;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len <= 0 || len > 256 * 1024) {
        fclose(f);
        return ESP_ERR_INVALID_SIZE;
    }
    uint8_t *buf = (uint8_t *) heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) buf = (uint8_t *) heap_caps_malloc(len, MALLOC_CAP_8BIT);
    if (!buf) { fclose(f); return ESP_ERR_NO_MEM; }
    size_t r = fread(buf, 1, len, f);
    fclose(f);
    if (r != (size_t) len) { heap_caps_free(buf); return ESP_FAIL; }
    *out_buf = buf;
    *out_len = (size_t) len;
    return ESP_OK;
}

esp_err_t wakeword_load_slot(wakeword_t *ww, uint8_t slot) {
    if (!ww) return ESP_ERR_INVALID_ARG;
    // Block the audio task from touching the interpreter while we tear it
    // down + rebuild it. Worst-case wait is one slice (~10 ms inference).
    xSemaphoreTake(ww->model_mutex, portMAX_DELAY);

    if (ww->model) {
        ww->model->unload_model();
        ww->model.reset();
    }
    if (ww->model_buf) {
        heap_caps_free(ww->model_buf);
        ww->model_buf = nullptr;
        ww->model_len = 0;
    }

    esp_err_t e = load_model_file(slot, &ww->model_buf, &ww->model_len);
    if (e != ESP_OK) {
        xSemaphoreGive(ww->model_mutex);
        return e;
    }

    // The friendly name + id stay generic at this layer — the manifest+name
    // mapping is a UI concern in OE (the Settings page already shows the
    // slot label).
    char id_buf[16];
    snprintf(id_buf, sizeof(id_buf), "slot%u", (unsigned) slot);

    // Pull probability_cutoff / sliding_window_size / tensor_arena_size from
    // the slot's JSON manifest so heterogeneous slots (e.g. okay_nabu @ 0.97
    // alongside a custom single-word model @ 0.78) load with the right thresholds.
    slot_manifest_t manifest;
    read_slot_manifest(slot, &manifest);

    // Exceptions are disabled in ESP-IDF, so plain `new` here. The vector
    // resize() inside the ctor cannot recover from OOM with -fno-exceptions
    // and will abort the system, but that is consistent with how every other
    // OOM in this firmware is handled (we don't try to limp on after a heap
    // exhaustion early in boot).
    ww->model.reset(new (std::nothrow) WakeWordModel(
        std::string(id_buf),
        ww->model_buf,
        manifest.probability_cutoff,
        manifest.sliding_window_size,
        std::string(id_buf),
        manifest.tensor_arena_size,
        /*default_enabled=*/true));
    if (!ww->model) {
        ESP_LOGE(TAG, "WakeWordModel alloc failed");
        heap_caps_free(ww->model_buf);
        ww->model_buf = nullptr;
        ww->model_len = 0;
        xSemaphoreGive(ww->model_mutex);
        return ESP_ERR_NO_MEM;
    }

    ww->model->log_model_config();
    ww->active_slot = slot;
    ESP_LOGI(TAG, "slot %u loaded (%u bytes)", slot, (unsigned) ww->model_len);
    xSemaphoreGive(ww->model_mutex);
    return ESP_OK;
}

void wakeword_unload_slot(wakeword_t *ww) {
    if (!ww) return;
    // Same mutex discipline as wakeword_load_slot: block the audio task from
    // touching the interpreter while we tear it down. Once model is null,
    // wakeword_feed early-returns false (line ~366) so the slot goes silent.
    xSemaphoreTake(ww->model_mutex, portMAX_DELAY);
    if (ww->model) {
        ww->model->unload_model();
        ww->model.reset();
    }
    if (ww->model_buf) {
        heap_caps_free(ww->model_buf);
        ww->model_buf = nullptr;
        ww->model_len = 0;
    }
    ww->active_slot = 0xFF;
    xSemaphoreGive(ww->model_mutex);
    ESP_LOGI(TAG, "slot unloaded");
}

uint8_t wakeword_active_slot(const wakeword_t *ww) {
    return ww ? ww->active_slot : 0xFF;
}

uint8_t wakeword_last_wake_prob(const wakeword_t *ww) {
    return ww ? ww->last_wake_avg_prob : 0;
}

// Scale FrontendOutput.values[] (uint16, ~0..670) to INT8 expected by the
// streaming model. Math copied from
// esphome/components/micro_wake_word/micro_wake_word.cpp ::generate_features_,
// which derives it from the TFLite audio frontend's training scale.
static inline int8_t scale_feature(uint16_t v) {
    constexpr int32_t value_scale = 256;
    constexpr int32_t value_div = 666;  // 25.6 * 26.0 ≈ 665.6
    int32_t value = (((int32_t) v) * value_scale + (value_div / 2)) / value_div;
    value += INT8_MIN;
    if (value < INT8_MIN) value = INT8_MIN;
    if (value > INT8_MAX) value = INT8_MAX;
    return (int8_t) value;
}

bool wakeword_feed(wakeword_t *ww, const int16_t *samples, size_t n_samples) {
    if (!ww || !ww->frontend_ready) return false;
    if (n_samples == 0) return false;
    // Hold the lock for the duration of the feed so a concurrent
    // wakeword_load_slot can't free the interpreter mid-Invoke. Inference
    // on this path runs ~10 ms; OTA swap waits at most one slice.
    if (xSemaphoreTake(ww->model_mutex, portMAX_DELAY) != pdTRUE) return false;
    if (!ww->model) { xSemaphoreGive(ww->model_mutex); return false; }

    // Track raw audio level over this call's input buffer. Cheap (~1280 abs
    // compares per 80 ms call). 0 means I²S delivered all-zero samples;
    // non-zero proves the audio path is alive even if the model never matches.
    ww->stats_calls++;
    for (size_t i = 0; i < n_samples; ++i) {
        int16_t s = samples[i];
        int16_t a = s < 0 ? (int16_t)(-s) : s;
        if (a > ww->stats_raw_max_abs) ww->stats_raw_max_abs = a;
    }

    // Walk the buffer in front-end-sized bites until it's empty. Each call
    // consumes up to `step_size_ms * 16` samples and emits exactly one slice.
    // We pass the full remaining buffer per call so the frontend can advance
    // by as many steps as fit; processed_samples tells us how much it ate.
    const int16_t *p = samples;
    size_t remaining = n_samples;

    bool detected = false;

    while (remaining > 0) {
        size_t processed = 0;
        FrontendOutput out = FrontendProcessSamples(&ww->frontend_state, (int16_t *) p, remaining, &processed);
        if (processed == 0) {
            // Not enough samples to advance by one full step.
            break;
        }
        p += processed;
        remaining -= processed;

        if (out.size == 0) continue;

        // Out should be PREPROCESSOR_FEATURE_SIZE values; assert at runtime
        // would be too aggressive, just clip.
        size_t take = out.size < PREPROCESSOR_FEATURE_SIZE ? out.size : PREPROCESSOR_FEATURE_SIZE;
        int8_t features[PREPROCESSOR_FEATURE_SIZE] = {0};
        for (size_t i = 0; i < take; ++i) features[i] = scale_feature(out.values[i]);

        // Bring-up stats — track feature range over recent slices to detect
        // a dead audio path (all -128) vs. live audio (varied values).
        for (size_t i = 0; i < take; ++i) {
            if (features[i] > ww->stats_max_feature) ww->stats_max_feature = features[i];
            if (features[i] < ww->stats_min_feature) ww->stats_min_feature = features[i];
        }
        ww->stats_slices++;

        // Track the speaking state for the refractory window, but DO NOT
        // skip inference — we want barge-in to work. Previously this block
        // `continue`d while speaking=true so the user couldn't interrupt
        // a long TTS reply by saying the wake word. With the noise-
        // augmented wake-word models (2026-05-15 retrain on MUSAN-mixed
        // positives), the model tolerates the small AEC-residual bleed
        // from our own playback well enough that running inference
        // through the speaking window is a net UX win. If false barge-
        // ins from AEC residual become a problem, restore the
        // `else { continue; }` arm to skip inference during speaking.
        // See memory project_xvf3800_bargein_during_speaking.
        if (ww->speaking) {
            int64_t now = esp_timer_get_time();
            int64_t since_end = now - ww->speaking_ended_us;
            if (ww->speaking_ended_us != 0 &&
                since_end > (int64_t) ww->cfg.refractory_after_speak_ms * 1000) {
                ww->speaking = false;
            }
        }

        if (!ww->model->perform_streaming_inference(features)) {
            ESP_LOGW(TAG, "streaming inference returned error — skipping slice");
            continue;
        }

        if (!ww->model->get_unprocessed_probability_status()) continue;

        DetectionEvent ev = ww->model->determine_detected();
        ww->stats_last_avg_prob = ev.average_probability;
        if (!ev.detected) continue;

        int64_t now = esp_timer_get_time();
        if (now - ww->last_detection_us < (int64_t) ww->cfg.cooldown_ms * 1000) continue;
        ww->last_detection_us = now;
        ww->last_wake_avg_prob = ev.average_probability;
        ww->model->reset_probabilities();

        const char *name = ev.wake_word ? ev.wake_word->c_str() : "?";
        ESP_LOGI(TAG, "wake! slot=%u name=%s avg=%u max=%u",
                 ww->active_slot, name, ev.average_probability, ev.max_probability);
        detected = true;
        // Don't break — finish consuming the buffer so the next call doesn't
        // skip features. The caller will react on the returned bool.
    }

    // Periodic stats — slim version (every ~10 s) so we can confirm audio is
    // alive and the model is processing slices. Demoted to LOGD: same data
    // streams server-side via the wake_avg_prob channel, and with multiple
    // wake-word slots configured the per-slot per-10s prints clutter the
    // monitor without adding signal. Re-enable via `idf.py menuconfig` →
    // "Log output" → set component "ww" to Debug, if you ever need it.
    {
        int64_t now = esp_timer_get_time();
        if (ww->stats_last_us == 0) ww->stats_last_us = now;
        if (now - ww->stats_last_us >= 10 * 1000 * 1000) {
            ESP_LOGD(TAG, "audio_lvl=%d feat_max=%d prob=%u/255",
                     (int) ww->stats_raw_max_abs,
                     (int) ww->stats_max_feature,
                     (unsigned) ww->stats_last_avg_prob);
            ww->stats_calls = 0;
            ww->stats_slices = 0;
            ww->stats_raw_max_abs = 0;
            ww->stats_max_feature = INT8_MIN;
            ww->stats_min_feature = INT8_MAX;
            ww->stats_last_us = now;
        }
    }

    xSemaphoreGive(ww->model_mutex);
    return detected;
}

void wakeword_notify_speaking_began(wakeword_t *ww) {
    if (!ww) return;
    ww->speaking = true;
    ww->speaking_ended_us = 0;
}

void wakeword_notify_speaking_ended(wakeword_t *ww) {
    if (!ww) return;
    ww->speaking_ended_us = esp_timer_get_time();
}

uint8_t wakeword_get_default_cutoff(const wakeword_t *ww) {
    if (!ww || !ww->model) return 0;
    return ww->model->get_default_probability_cutoff();
}

void wakeword_set_cutoff(wakeword_t *ww, uint8_t cutoff) {
    if (!ww) return;
    if (xSemaphoreTake(ww->model_mutex, portMAX_DELAY) != pdTRUE) return;
    if (ww->model) ww->model->set_probability_cutoff(cutoff);
    xSemaphoreGive(ww->model_mutex);
}
