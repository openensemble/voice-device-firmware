/**
 * Device-side alarm registry. See alarm.h for the API contract.
 *
 * Pass 1 scope:
 *   - In-memory + NVS-persisted registry of armed alarms
 *   - esp_timer scheduling against NTP-synced wall-clock
 *   - Boot-replay: persisted alarms with past trigger times fire immediately
 *   - Ring task that wakes on fire; stubs the audio playback (TODO pass 2)
 *   - Local dismiss + WS ack send-side
 *
 * Pass 2 will plug in the chime + per-alarm TTS audio playback and the
 * built-in/uploaded chime selection.
 */
#include "alarm.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "oe_client.h"
#include "audio_io.h"
#include "mp3_decode.h"
#include "esp_spiffs.h"
#include <sys/stat.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define STORAGE_PARTITION_LABEL "storage"
#define STORAGE_MOUNT_POINT     "/storage"
#define CUSTOM_CHIME_PATH       "/storage/chime.mp3"
// Reject any chime MP3 over 256 KB (the boot-time loader uses the same
// limit). At 64 kbps mono that's ~32 s — well past any reasonable chime,
// and capped well below what would fit in the SPIFFS storage partition.
#define MAX_CHIME_MP3_BYTES     (256 * 1024)

static const char *TAG = "alarm";

#define ALARM_MAX           8
#define NVS_NS              "alarms"
#define NVS_KEY             "all"
#define RING_CYCLE_MS       5500
#define HARD_STOP_MS        (10 * 60 * 1000)
// 2024-01-01 in epoch ms — below this, time() probably isn't NTP-synced.
#define MIN_VALID_EPOCH_MS  1704067200000LL

typedef enum {
    ALARM_STATE_ARMED  = 0,  // esp_timer pending, not yet fired
    ALARM_STATE_FIRING = 1,  // ring task is active for this alarm
} alarm_state_t;

typedef struct {
    bool used;
    char id[40];
    char label[64];
    char type[16];
    int64_t trigger_at_ms;
    uint8_t *audio_mp3;       // PSRAM, may be NULL post-reboot or wallclock-type
    size_t audio_mp3_len;
    esp_timer_handle_t timer;
    alarm_state_t state;
    int64_t fired_at_us;      // esp_timer_get_time when fire began
    uint8_t ring_refs;        // ring_task snapshots holding audio_mp3 alive
    bool delete_pending;      // remove once ring_refs drains to zero
} alarm_entry_t;

static alarm_entry_t s_alarms[ALARM_MAX];
static SemaphoreHandle_t s_mutex;
static SemaphoreHandle_t s_ring_signal;
static QueueHandle_t s_event_q;

static alarm_speaking_cb_t s_speaking_cb = NULL;
static alarm_amp_cb_t      s_amp_cb      = NULL;

typedef enum {
    ALARM_EVT_FIRED = 1,
} alarm_event_type_t;

typedef struct {
    alarm_event_type_t type;
    char id[40];
} alarm_event_t;

// Pre-built chime PCM. Either the procedural two-tone (B5→E6) built at
// init, or PCM decoded from a user-uploaded MP3 at /storage/chime.mp3.
// User uploads override the built-in. Both forms are stereo-interleaved
// to match audio_io_write_pcm's expected input shape.
static int16_t *s_chime_pcm = NULL;
static size_t   s_chime_pcm_total = 0;  // interleaved sample count (frames*2)
static uint32_t s_chime_rate = 16000;
static bool     s_chime_is_custom = false;
// The chime buffer the ring task is currently reading inside play_chime()
// (NULL when not playing), plus a swapped-out buffer whose free was deferred
// because it was in use. audio_io_write_pcm() reads s_chime_pcm for the whole
// (potentially blocking) push, so a concurrent chime swap must not free it out
// from under the ring task — free_chime_buf_locked() hands the old buffer here
// instead, and play_chime() frees it once the write returns. Guarded by s_mutex.
static int16_t *s_chime_playing = NULL;
static int16_t *s_chime_free_pending = NULL;

// Free a chime PCM buffer, or defer the free if the ring task is mid-play on
// it (play_chime marks it via s_chime_playing). Caller MUST hold s_mutex.
static void free_chime_buf_locked(int16_t *buf)
{
    if (!buf) return;
    if (buf == s_chime_playing) {
        if (s_chime_free_pending && s_chime_free_pending != buf) {
            heap_caps_free(s_chime_free_pending);
        }
        s_chime_free_pending = buf;
    } else {
        heap_caps_free(buf);
    }
}

void alarm_set_speaking_callback(alarm_speaking_cb_t cb) { s_speaking_cb = cb; }
void alarm_set_amp_callback(alarm_amp_cb_t cb)           { s_amp_cb = cb; }

static int64_t now_epoch_ms(void)
{
    time_t now;
    time(&now);
    return (int64_t)now * 1000;
}

static bool wallclock_ready(void)
{
    return now_epoch_ms() > MIN_VALID_EPOCH_MS;
}

static alarm_entry_t *find_by_id_locked(const char *id)
{
    for (int i = 0; i < ALARM_MAX; ++i) {
        if (s_alarms[i].used && !s_alarms[i].delete_pending &&
            strcmp(s_alarms[i].id, id) == 0) return &s_alarms[i];
    }
    return NULL;
}

static alarm_entry_t *alloc_slot_locked(void)
{
    for (int i = 0; i < ALARM_MAX; ++i) {
        if (!s_alarms[i].used) return &s_alarms[i];
    }
    return NULL;
}

static void free_entry_locked(alarm_entry_t *e)
{
    if (!e) return;
    if (e->timer) {
        esp_timer_stop(e->timer);
        esp_timer_delete(e->timer);
        e->timer = NULL;
    }
    if (e->ring_refs > 0) {
        e->delete_pending = true;
        e->state = ALARM_STATE_ARMED;
        return;
    }
    if (e->audio_mp3) {
        heap_caps_free(e->audio_mp3);
        e->audio_mp3 = NULL;
        e->audio_mp3_len = 0;
    }
    memset(e, 0, sizeof(*e));
}

// NVS layout — single blob `all`:
//   u8 count
//   for each: u8 id_len, id; u16 label_len(le), label; u8 type_len, type; i64 trigger_at_ms(le)
// Audio buffer is intentionally NOT persisted — RAM-only by design.
static esp_err_t nvs_save(void)
{
    typedef struct {
        char id[40];
        char label[64];
        char type[16];
        int64_t trigger_at_ms;
    } alarm_persist_t;

    alarm_persist_t snap[ALARM_MAX] = {0};
    uint8_t count = 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < ALARM_MAX; ++i) {
        if (!s_alarms[i].used || s_alarms[i].delete_pending) continue;
        strncpy(snap[count].id, s_alarms[i].id, sizeof(snap[count].id) - 1);
        strncpy(snap[count].label, s_alarms[i].label, sizeof(snap[count].label) - 1);
        strncpy(snap[count].type, s_alarms[i].type, sizeof(snap[count].type) - 1);
        snap[count].trigger_at_ms = s_alarms[i].trigger_at_ms;
        count++;
    }
    xSemaphoreGive(s_mutex);

    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;

    size_t cap = 1;
    for (int i = 0; i < count; ++i) {
        cap += 1 + strlen(snap[i].id);
        cap += 2 + strlen(snap[i].label);
        cap += 1 + strlen(snap[i].type);
        cap += 8;
    }
    if (count == 0) {
        // No alarms — remove the blob entirely
        nvs_erase_key(h, NVS_KEY);
        nvs_commit(h);
        nvs_close(h);
        return ESP_OK;
    }
    uint8_t *buf = malloc(cap);
    if (!buf) { nvs_close(h); return ESP_ERR_NO_MEM; }
    uint8_t *p = buf;
    *p++ = count;
    for (int i = 0; i < count; ++i) {
        size_t idl = strlen(snap[i].id);
        size_t lbl = strlen(snap[i].label);
        size_t tpl = strlen(snap[i].type);
        *p++ = (uint8_t)idl; memcpy(p, snap[i].id, idl); p += idl;
        *p++ = (uint8_t)(lbl & 0xff); *p++ = (uint8_t)(lbl >> 8); memcpy(p, snap[i].label, lbl); p += lbl;
        *p++ = (uint8_t)tpl; memcpy(p, snap[i].type, tpl); p += tpl;
        memcpy(p, &snap[i].trigger_at_ms, 8); p += 8;
    }
    e = nvs_set_blob(h, NVS_KEY, buf, p - buf);
    if (e == ESP_OK) e = nvs_commit(h);
    free(buf);
    nvs_close(h);
    return e;
}

static esp_err_t nvs_load(void)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (e != ESP_OK) return ESP_OK;  // namespace not created yet = no alarms

    size_t needed = 0;
    e = nvs_get_blob(h, NVS_KEY, NULL, &needed);
    if (e == ESP_ERR_NVS_NOT_FOUND) { nvs_close(h); return ESP_OK; }
    if (e != ESP_OK) { nvs_close(h); return e; }

    uint8_t *buf = malloc(needed);
    if (!buf) { nvs_close(h); return ESP_ERR_NO_MEM; }
    e = nvs_get_blob(h, NVS_KEY, buf, &needed);
    nvs_close(h);
    if (e != ESP_OK) { free(buf); return e; }

    const uint8_t *p = buf;
    const uint8_t *end = buf + needed;
    uint8_t count = (p < end) ? *p++ : 0;
    for (uint8_t i = 0; i < count && p < end; ++i) {
        if (p + 1 > end) break;
        uint8_t idl = *p++;
        if (p + idl > end) break;
        char id[40] = {0};   memcpy(id, p, idl < sizeof(id)-1 ? idl : sizeof(id)-1); p += idl;

        if (p + 2 > end) break;
        uint16_t lbl = (uint16_t)p[0] | ((uint16_t)p[1] << 8); p += 2;
        if (p + lbl > end) break;
        char label[64] = {0}; memcpy(label, p, lbl < sizeof(label)-1 ? lbl : sizeof(label)-1); p += lbl;

        if (p + 1 > end) break;
        uint8_t tpl = *p++;
        if (p + tpl > end) break;
        char type[16] = {0};  memcpy(type, p, tpl < sizeof(type)-1 ? tpl : sizeof(type)-1); p += tpl;

        if (p + 8 > end) break;
        int64_t ts; memcpy(&ts, p, 8); p += 8;

        alarm_entry_t *slot = alloc_slot_locked();
        if (!slot) break;
        slot->used = true;
        strncpy(slot->id, id, sizeof(slot->id) - 1);
        strncpy(slot->label, label, sizeof(slot->label) - 1);
        strncpy(slot->type, type, sizeof(slot->type) - 1);
        slot->trigger_at_ms = ts;
        slot->state = ALARM_STATE_ARMED;
        // audio_mp3 stays NULL — RAM cache lost on reboot, chime-only ring.
    }
    free(buf);
    return ESP_OK;
}

static void timer_cb(void *arg)
{
    alarm_entry_t *e = (alarm_entry_t *)arg;
    if (!e) return;
    bool should_ring = false;
    alarm_event_t ev = { .type = ALARM_EVT_FIRED };
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (e->used && !e->delete_pending) {
        e->state = ALARM_STATE_FIRING;
        e->fired_at_us = esp_timer_get_time();
        ESP_LOGI(TAG, "alarm fired: id=%s label=%s", e->id, e->label);
        strncpy(ev.id, e->id, sizeof(ev.id) - 1);
        should_ring = true;
    }
    xSemaphoreGive(s_mutex);
    if (should_ring) {
        if (s_event_q) xQueueSend(s_event_q, &ev, 0);
        xSemaphoreGive(s_ring_signal);
    }
}

static void alarm_event_task(void *arg)
{
    (void)arg;
    alarm_event_t ev;
    while (1) {
        if (xQueueReceive(s_event_q, &ev, portMAX_DELAY) != pdTRUE) continue;
        if (ev.type == ALARM_EVT_FIRED) oe_ws_send_alarm_fired(ev.id);
    }
}

// Caller holds s_mutex.
static esp_err_t schedule_locked(alarm_entry_t *e)
{
    if (!wallclock_ready()) {
        // Caller (deferred_schedule_task) retries once NTP acquires.
        return ESP_ERR_INVALID_STATE;
    }
    if (!e->timer) {
        esp_timer_create_args_t args = {
            .callback = timer_cb,
            .arg = e,
            .name = "alarm_t",
        };
        esp_err_t er = esp_timer_create(&args, &e->timer);
        if (er != ESP_OK) return er;
    } else {
        esp_timer_stop(e->timer);
    }
    int64_t delay_ms = e->trigger_at_ms - now_epoch_ms();
    uint64_t delay_us = (delay_ms <= 0) ? 0 : (uint64_t)delay_ms * 1000ULL;
    return esp_timer_start_once(e->timer, delay_us);
}

// ── Audio: procedural chime + MP3 playback ───────────────────────────────────

#define CHIME_RATE       16000
#define CHIME_TONE_MS    110
#define CHIME_GAP_MS     30
#define CHIME_AMP_PEAK   13000   // 40% of int16 max — matches the old server chime

// SPIFFS mount: shared `storage` partition (1 MB per partitions.csv) used
// for custom chime persistence today; can hold other generic device assets
// later. Mounted once at alarm_init; safe to call repeatedly.
static bool s_storage_mounted = false;

static void ensure_storage_mounted(void)
{
    if (s_storage_mounted) return;
    esp_vfs_spiffs_conf_t cfg = {
        .base_path = STORAGE_MOUNT_POINT,
        .partition_label = STORAGE_PARTITION_LABEL,
        .max_files = 4,
        .format_if_mount_failed = true,
    };
    esp_err_t e = esp_vfs_spiffs_register(&cfg);
    if (e == ESP_OK || e == ESP_ERR_INVALID_STATE) {
        s_storage_mounted = true;
    } else {
        ESP_LOGW(TAG, "storage SPIFFS mount failed: %s", esp_err_to_name(e));
    }
}

// PCM collector — feed every decoded chunk into a growing PSRAM buffer so
// the whole decoded chime can be cached in memory for playback.
typedef struct {
    int16_t *buf;
    size_t   cap;       // int16 count
    size_t   used;      // int16 count
    uint32_t rate;
} pcm_collect_t;

static void pcm_collect_cb(const int16_t *pcm, size_t samples, uint32_t rate, void *user)
{
    pcm_collect_t *c = (pcm_collect_t *)user;
    c->rate = rate;
    size_t need = c->used + samples;
    if (need > c->cap) {
        size_t new_cap = c->cap ? c->cap : 4096;
        while (new_cap < need) new_cap *= 2;
        int16_t *nb = heap_caps_realloc(c->buf, new_cap * sizeof(int16_t),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!nb) nb = realloc(c->buf, new_cap * sizeof(int16_t));
        if (!nb) return;
        c->buf = nb;
        c->cap = new_cap;
    }
    memcpy(c->buf + c->used, pcm, samples * sizeof(int16_t));
    c->used += samples;
}

// Decode an MP3 buffer to interleaved-stereo PCM. mp3_decode duplicates
// mono to stereo for us, so we can hand the output straight to
// audio_io_write_pcm. Returns ESP_OK on success with the PCM buffer
// allocated via PSRAM (caller owns).
static esp_err_t decode_mp3_to_pcm(const uint8_t *mp3, size_t mp3_len,
                                   int16_t **out_pcm, size_t *out_samples,
                                   uint32_t *out_rate)
{
    if (!mp3 || !mp3_len) return ESP_ERR_INVALID_ARG;
    pcm_collect_t c = {0};
    mp3_dec_t *d = mp3_dec_create(pcm_collect_cb, &c);
    if (!d) return ESP_ERR_NO_MEM;
    mp3_dec_feed(d, mp3, mp3_len);
    mp3_dec_flush(d);
    mp3_dec_destroy(d);
    if (!c.buf || !c.used) {
        if (c.buf) heap_caps_free(c.buf);
        return ESP_FAIL;
    }
    *out_pcm = c.buf;
    *out_samples = c.used;
    *out_rate = c.rate ? c.rate : 16000;
    return ESP_OK;
}

// Try to load /storage/chime.mp3 → decode → install as the chime. Returns
// ESP_OK if a custom chime was loaded; any error indicates fall back to
// the procedural chime.
static esp_err_t try_load_custom_chime(void)
{
    ensure_storage_mounted();
    if (!s_storage_mounted) return ESP_ERR_INVALID_STATE;

    struct stat st;
    if (stat(CUSTOM_CHIME_PATH, &st) != 0) return ESP_ERR_NOT_FOUND;
    if (st.st_size <= 0 || st.st_size > 256 * 1024) {
        ESP_LOGW(TAG, "custom chime size unreasonable: %ld bytes", (long)st.st_size);
        return ESP_ERR_INVALID_SIZE;
    }
    FILE *f = fopen(CUSTOM_CHIME_PATH, "rb");
    if (!f) return ESP_FAIL;
    uint8_t *mp3 = heap_caps_malloc(st.st_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!mp3) mp3 = malloc(st.st_size);
    if (!mp3) { fclose(f); return ESP_ERR_NO_MEM; }
    size_t got = fread(mp3, 1, st.st_size, f);
    fclose(f);
    if (got != (size_t)st.st_size) {
        heap_caps_free(mp3);
        return ESP_FAIL;
    }

    int16_t *pcm = NULL;
    size_t samples = 0;
    uint32_t rate = 16000;
    esp_err_t e = decode_mp3_to_pcm(mp3, got, &pcm, &samples, &rate);
    heap_caps_free(mp3);
    if (e != ESP_OK) return e;

    // Replace any existing chime (procedural or previous custom). Swap under
    // the mutex; free_chime_buf_locked defers the free if the ring task is
    // mid-play on the old buffer.
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    free_chime_buf_locked(s_chime_pcm);
    s_chime_pcm = pcm;
    s_chime_pcm_total = samples;
    s_chime_rate = rate;
    s_chime_is_custom = true;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "custom chime loaded: %u samples @ %u Hz from %s",
             (unsigned)samples, (unsigned)rate, CUSTOM_CHIME_PATH);
    return ESP_OK;
}

// Build the chime PCM once at init. Two sine tones at B5 (988 Hz) and E6
// (1318 Hz) separated by a short gap; envelope fades each tone in/out so
// onset/release is clean. Stereo-interleaved so audio_io_write_pcm doesn't
// have to duplicate per sample.
static void build_chime_pcm(void)
{
    const float freqs[2] = { 988.0f, 1318.0f };
    const int tone_samples = (CHIME_RATE * CHIME_TONE_MS) / 1000;
    const int gap_samples  = (CHIME_RATE * CHIME_GAP_MS) / 1000;
    const int frames = tone_samples + gap_samples + tone_samples;
    const int total = frames * 2;  // stereo
    int16_t *buf = heap_caps_malloc(total * sizeof(int16_t),
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) buf = malloc(total * sizeof(int16_t));
    if (!buf) {
        ESP_LOGE(TAG, "chime alloc failed (%d bytes)", (int)(total * sizeof(int16_t)));
        return;
    }
    int idx = 0;
    for (int tone = 0; tone < 2; ++tone) {
        for (int i = 0; i < tone_samples; ++i) {
            float t = (float)i / CHIME_RATE;
            // Fade-in 5 ms, fade-out last 20 ms.
            float env = 1.0f;
            if (t < 0.005f) env = t / 0.005f;
            else if (t > (CHIME_TONE_MS / 1000.0f - 0.020f))
                env = (CHIME_TONE_MS / 1000.0f - t) / 0.020f;
            if (env < 0) env = 0;
            int16_t v = (int16_t)(CHIME_AMP_PEAK * env * sinf(2.0f * (float)M_PI * freqs[tone] * t));
            buf[idx++] = v;
            buf[idx++] = v;
        }
        if (tone == 0) {
            // Silent gap between tones
            for (int i = 0; i < gap_samples; ++i) {
                buf[idx++] = 0;
                buf[idx++] = 0;
            }
        }
    }
    s_chime_pcm = buf;
    s_chime_pcm_total = total;
    s_chime_rate = CHIME_RATE;
    s_chime_is_custom = false;
    ESP_LOGI(TAG, "chime built (procedural): %d frames stereo @ %d Hz",
             frames, CHIME_RATE);
}

static void play_chime(void)
{
    // Snapshot + mark the buffer in-use under the lock so a concurrent chime
    // swap defers its free (free_chime_buf_locked) instead of pulling it out
    // from under the blocking write below. Release any deferred buffer once
    // the write returns and we're no longer reading it.
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int16_t *chime = s_chime_pcm;
    size_t   total = s_chime_pcm_total;
    uint32_t rate  = s_chime_rate;
    s_chime_playing = chime;
    xSemaphoreGive(s_mutex);

    if (chime) audio_io_write_pcm(chime, total, rate);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_chime_playing = NULL;
    if (s_chime_free_pending) {
        heap_caps_free(s_chime_free_pending);
        s_chime_free_pending = NULL;
    }
    xSemaphoreGive(s_mutex);
}

// Streaming MP3→PCM relay: each decoded chunk is fed directly to audio_io,
// matching the pattern oe_tts.c uses for chat replies. Mono MP3s are
// duplicated to stereo inside mp3_decode.
static void mp3_to_audio_cb(const int16_t *pcm, size_t samples, uint32_t rate, void *user)
{
    (void)user;
    audio_io_write_pcm(pcm, samples, rate);
}

static void play_mp3_buf(const uint8_t *mp3, size_t len)
{
    if (!mp3 || !len) return;
    mp3_dec_t *d = mp3_dec_create(mp3_to_audio_cb, NULL);
    if (!d) return;
    mp3_dec_feed(d, mp3, len);
    mp3_dec_flush(d);
    mp3_dec_destroy(d);
}

static void ring_session_begin(void)
{
    if (s_speaking_cb) s_speaking_cb(true);
    if (s_amp_cb)      s_amp_cb(true);
    audio_io_start_playback();
}

static void ring_session_end(void)
{
    audio_io_stop_playback();
    audio_io_flush_playback();
    if (s_amp_cb)      s_amp_cb(false);
    if (s_speaking_cb) s_speaking_cb(false);
}

static void ring_task(void *arg)
{
    (void)arg;
    while (1) {
        if (xSemaphoreTake(s_ring_signal, portMAX_DELAY) != pdTRUE) continue;

        ring_session_begin();
        bool session_active = true;

        while (session_active) {
            // Snapshot firing entries — separate copy of (audio_mp3, len) so
            // the actual playback happens outside the mutex (decode + I2S
            // writes take ~3-5 s per alarm and we don't want to block
            // alarm_arm / alarm_disarm / alarm_stop for that long).
            struct { alarm_entry_t *entry; uint8_t *mp3; size_t len; bool used; } snap[ALARM_MAX] = {0};
            int firing_count = 0;
            int64_t earliest_us = INT64_MAX;
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            for (int i = 0; i < ALARM_MAX; ++i) {
                if (!s_alarms[i].used || s_alarms[i].delete_pending ||
                    s_alarms[i].state != ALARM_STATE_FIRING) continue;
                s_alarms[i].ring_refs++;
                snap[i].entry = &s_alarms[i];
                snap[i].mp3  = s_alarms[i].audio_mp3;
                snap[i].len  = s_alarms[i].audio_mp3_len;
                snap[i].used = true;
                firing_count++;
                if (s_alarms[i].fired_at_us < earliest_us) earliest_us = s_alarms[i].fired_at_us;
            }
            xSemaphoreGive(s_mutex);

            if (firing_count == 0) { session_active = false; break; }

            int64_t age_us = esp_timer_get_time() - earliest_us;
            if (age_us > (int64_t)HARD_STOP_MS * 1000) {
                ESP_LOGW(TAG, "alarm hard-stop cap reached, stopping ring");
                xSemaphoreTake(s_mutex, portMAX_DELAY);
                for (int i = 0; i < ALARM_MAX; ++i) {
                    if (!snap[i].entry) continue;
                    if (snap[i].entry->ring_refs > 0) snap[i].entry->ring_refs--;
                    if (snap[i].entry->ring_refs == 0 && snap[i].entry->delete_pending) {
                        free_entry_locked(snap[i].entry);
                    }
                }
                xSemaphoreGive(s_mutex);
                alarm_stop(NULL);
                break;
            }

            ESP_LOGI(TAG, "alarm ring cycle (firing=%d, age=%lldms)",
                     firing_count, (long long)(age_us / 1000));

            // Chime + each firing alarm's TTS, in sequence within this cycle.
            // Post-reboot alarms have no MP3 (audio cache is RAM-only); they
            // ring chime-only — user infers context.
            play_chime();
            for (int i = 0; i < ALARM_MAX; ++i) {
                if (snap[i].used && snap[i].mp3 && snap[i].len) {
                    play_mp3_buf(snap[i].mp3, snap[i].len);
                }
            }

            xSemaphoreTake(s_mutex, portMAX_DELAY);
            for (int i = 0; i < ALARM_MAX; ++i) {
                if (!snap[i].entry) continue;
                if (snap[i].entry->ring_refs > 0) snap[i].entry->ring_refs--;
                if (snap[i].entry->ring_refs == 0 && snap[i].entry->delete_pending) {
                    free_entry_locked(snap[i].entry);
                }
            }
            xSemaphoreGive(s_mutex);

            // play_chime + play_mp3_buf queue PCM into the audio_io ring-
            // buffer and return immediately; DMA drains it in parallel with
            // this delay. Alarm-clock cadence: chime is ~250 ms, cycle 600 ms,
            // so ~350 ms of silence between chimes (≈1.67 Hz). With audio
            // disabled (chime-only alarms) the TTS playback considerations
            // no longer constrain us. If a future caller starts feeding TTS
            // via play_mp3_buf again, bump this back up to 4000 ms.
            vTaskDelay(pdMS_TO_TICKS(600));
        }

        ring_session_end();
    }
}

// Background task: if NTP wasn't ready at boot, persisted alarms got loaded
// into s_alarms[] but couldn't be scheduled. Wait for wallclock_ready() and
// then schedule them all.
static void deferred_schedule_task(void *arg)
{
    (void)arg;
    while (!wallclock_ready()) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
    ESP_LOGI(TAG, "wall-clock acquired; scheduling persisted alarms");
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < ALARM_MAX; ++i) {
        if (s_alarms[i].used && !s_alarms[i].delete_pending &&
            s_alarms[i].state == ALARM_STATE_ARMED && !s_alarms[i].timer) {
            schedule_locked(&s_alarms[i]);
        }
    }
    xSemaphoreGive(s_mutex);
    vTaskDelete(NULL);
}

esp_err_t alarm_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    s_ring_signal = xSemaphoreCreateBinary();
    s_event_q = xQueueCreate(8, sizeof(alarm_event_t));
    if (!s_mutex || !s_ring_signal || !s_event_q) return ESP_ERR_NO_MEM;

    build_chime_pcm();
    // If a user-uploaded chime is on disk, decode + install it (replaces the
    // procedural one). Failures fall through silently — procedural chime is
    // already installed.
    try_load_custom_chime();
    nvs_load();

    xTaskCreate(ring_task, "alarm_ring", 4096, NULL, 5, NULL);
    xTaskCreate(alarm_event_task, "alarm_evt", 4096, NULL, 4, NULL);
    xTaskCreate(deferred_schedule_task, "alarm_sched", 3072, NULL, 4, NULL);

    int armed = 0;
    for (int i = 0; i < ALARM_MAX; ++i) if (s_alarms[i].used) armed++;
    ESP_LOGI(TAG, "alarm_init: %d persisted alarm(s) loaded", armed);
    return ESP_OK;
}

esp_err_t alarm_arm(const char *id, const char *label,
                    int64_t trigger_at_ms,
                    uint8_t *audio_mp3, size_t audio_mp3_len,
                    const char *type)
{
    if (!id || !label || !type) {
        if (audio_mp3) heap_caps_free(audio_mp3);
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    alarm_entry_t *e = find_by_id_locked(id);
    if (e) free_entry_locked(e);
    e = alloc_slot_locked();
    if (!e) {
        xSemaphoreGive(s_mutex);
        ESP_LOGW(TAG, "alarm_arm: no free slot (max=%d)", ALARM_MAX);
        if (audio_mp3) heap_caps_free(audio_mp3);
        return ESP_ERR_NO_MEM;
    }
    e->used = true;
    strncpy(e->id, id, sizeof(e->id) - 1);
    strncpy(e->label, label, sizeof(e->label) - 1);
    strncpy(e->type, type, sizeof(e->type) - 1);
    e->trigger_at_ms = trigger_at_ms;
    e->audio_mp3 = audio_mp3;
    e->audio_mp3_len = audio_mp3_len;
    e->state = ALARM_STATE_ARMED;
    e->timer = NULL;

    esp_err_t er = schedule_locked(e);
    xSemaphoreGive(s_mutex);

    nvs_save();
    if (er == ESP_OK) {
        ESP_LOGI(TAG, "alarm armed: id=%s label=%s trigger_at_ms=%lld",
                 id, label, (long long)trigger_at_ms);
    } else if (er == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "alarm armed (deferred — NTP not ready): id=%s", id);
        er = ESP_OK;  // not an error — deferred scheduler will pick up
    } else {
        ESP_LOGE(TAG, "alarm_arm schedule failed: %d", er);
    }
    return er;
}

esp_err_t alarm_disarm(const char *id)
{
    if (!id) return ESP_ERR_INVALID_ARG;
    char ack_id[40] = {0};
    bool was_firing = false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    alarm_entry_t *e = find_by_id_locked(id);
    if (!e) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }
    was_firing = e->state == ALARM_STATE_FIRING;
    strncpy(ack_id, e->id, sizeof(ack_id) - 1);
    free_entry_locked(e);
    xSemaphoreGive(s_mutex);
    nvs_save();
    if (was_firing) oe_ws_send_alarm_acked(ack_id);
    ESP_LOGI(TAG, "alarm disarmed: id=%s", id);
    return ESP_OK;
}

void alarm_stop(const char *id)
{
    bool changed = false;
    char ack_ids[ALARM_MAX][40] = {0};
    int ack_count = 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < ALARM_MAX; ++i) {
        if (!s_alarms[i].used) continue;
        if (s_alarms[i].delete_pending) continue;
        if (s_alarms[i].state != ALARM_STATE_FIRING) continue;
        if (id && strcmp(s_alarms[i].id, id) != 0) continue;
        ESP_LOGI(TAG, "alarm_stop: id=%s", s_alarms[i].id);
        strncpy(ack_ids[ack_count++], s_alarms[i].id, sizeof(ack_ids[0]) - 1);
        free_entry_locked(&s_alarms[i]);
        changed = true;
    }
    xSemaphoreGive(s_mutex);
    if (changed) nvs_save();
    for (int i = 0; i < ack_count; ++i) oe_ws_send_alarm_acked(ack_ids[i]);
}

bool alarm_is_firing(void)
{
    bool firing = false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < ALARM_MAX; ++i) {
        if (s_alarms[i].used && !s_alarms[i].delete_pending &&
            s_alarms[i].state == ALARM_STATE_FIRING) { firing = true; break; }
    }
    xSemaphoreGive(s_mutex);
    return firing;
}

// Re-report every currently-ringing alarm to the server. alarm_fired is
// fire-and-forget (sent once from the timer path), so if the WS was down at
// the fire instant the server never learned the alarm rang and later
// false-alerts "didn't fire". main.c calls this on WS (re)connect; it's
// idempotent server-side (markAlarmFired). We snapshot under the lock and
// enqueue onto s_event_q — the WS send happens in alarm_event_task, never
// under the mutex (same rule as the fire path).
void alarm_resend_fired(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < ALARM_MAX; ++i) {
        if (s_alarms[i].used && !s_alarms[i].delete_pending &&
            s_alarms[i].state == ALARM_STATE_FIRING) {
            alarm_event_t ev = { .type = ALARM_EVT_FIRED };
            strncpy(ev.id, s_alarms[i].id, sizeof(ev.id) - 1);
            ev.id[sizeof(ev.id) - 1] = '\0';
            if (s_event_q) xQueueSend(s_event_q, &ev, 0);
        }
    }
    xSemaphoreGive(s_mutex);
}

bool alarm_handle_local_dismiss(void)
{
    bool any = false;
    char ack_ids[ALARM_MAX][40] = {0};
    int ack_count = 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < ALARM_MAX; ++i) {
        if (!s_alarms[i].used || s_alarms[i].delete_pending ||
            s_alarms[i].state != ALARM_STATE_FIRING) continue;
        any = true;
        ESP_LOGI(TAG, "alarm local dismiss: id=%s", s_alarms[i].id);
        strncpy(ack_ids[ack_count++], s_alarms[i].id, sizeof(ack_ids[0]) - 1);
        free_entry_locked(&s_alarms[i]);
    }
    xSemaphoreGive(s_mutex);
    if (any) nvs_save();
    for (int i = 0; i < ack_count; ++i) oe_ws_send_alarm_acked(ack_ids[i]);
    return any;
}

esp_err_t alarm_set_custom_chime(uint8_t *mp3, size_t mp3_len)
{
    ensure_storage_mounted();
    if (!s_storage_mounted) return ESP_ERR_INVALID_STATE;

    // Hard cap consistent with try_load_custom_chime (boot-time loader).
    // Reject up front so a misconfigured server-side cap can't blow up
    // PSRAM via decode_mp3_to_pcm.
    if (mp3 && mp3_len > MAX_CHIME_MP3_BYTES) {
        ESP_LOGW(TAG, "alarm_set_custom_chime: %u bytes exceeds %d KB cap — rejecting",
                 (unsigned)mp3_len, MAX_CHIME_MP3_BYTES / 1024);
        heap_caps_free(mp3);
        return ESP_ERR_INVALID_SIZE;
    }

    // mp3=NULL → revert to procedural chime + remove the persisted file.
    if (!mp3 || !mp3_len) {
        unlink(CUSTOM_CHIME_PATH);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        free_chime_buf_locked(s_chime_pcm);
        s_chime_pcm = NULL;
        s_chime_pcm_total = 0;
        build_chime_pcm();
        xSemaphoreGive(s_mutex);
        ESP_LOGI(TAG, "custom chime cleared; reverted to procedural");
        return ESP_OK;
    }

    // Decode FIRST — no point persisting a buffer that doesn't decode.
    int16_t *pcm = NULL;
    size_t samples = 0;
    uint32_t rate = 16000;
    esp_err_t e = decode_mp3_to_pcm(mp3, mp3_len, &pcm, &samples, &rate);
    if (e != ESP_OK) {
        heap_caps_free(mp3);
        ESP_LOGW(TAG, "alarm_set_custom_chime: decode failed (%s)", esp_err_to_name(e));
        return e;
    }

    // Persist for next boot. Use a temp file + rename so a power loss
    // mid-write doesn't leave a half-baked chime on disk.
    const char *tmp_path = CUSTOM_CHIME_PATH ".tmp";
    FILE *f = fopen(tmp_path, "wb");
    if (!f) {
        heap_caps_free(mp3);
        heap_caps_free(pcm);
        return ESP_FAIL;
    }
    size_t wrote = fwrite(mp3, 1, mp3_len, f);
    fclose(f);
    heap_caps_free(mp3);  // we have the PCM; raw MP3 no longer needed in RAM
    if (wrote != mp3_len) {
        unlink(tmp_path);
        heap_caps_free(pcm);
        ESP_LOGW(TAG, "custom chime fwrite short: %u/%u", (unsigned)wrote, (unsigned)mp3_len);
        return ESP_FAIL;
    }
    unlink(CUSTOM_CHIME_PATH);
    rename(tmp_path, CUSTOM_CHIME_PATH);

    // Swap into live state under the mutex. free_chime_buf_locked defers the
    // old buffer's free if the ring task is mid-play on it, so a chime upload
    // during an active ring can't pull PCM out from under play_chime().
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    free_chime_buf_locked(s_chime_pcm);
    s_chime_pcm = pcm;
    s_chime_pcm_total = samples;
    s_chime_rate = rate;
    s_chime_is_custom = true;
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "custom chime installed: %u samples @ %u Hz (%u MP3 bytes)",
             (unsigned)samples, (unsigned)rate, (unsigned)mp3_len);
    return ESP_OK;
}
