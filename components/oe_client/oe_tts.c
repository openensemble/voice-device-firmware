#include "oe_client.h"
#include "mp3_decode.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "cJSON.h"

static const char *TAG = "oe_tts";

static void build_api_url(char *out, size_t out_len, const char *server_url, const char *path)
{
    size_t n = strnlen(server_url, OE_URL_BUF);
    while (n > 0 && server_url[n - 1] == '/') n--;
    snprintf(out, out_len, "%.*s%s", (int)n, server_url, path);
}

// Monotonic byte counter across every oe_tts_post invocation. The heartbeat
// ambient-stats log samples this each tick and computes the delta to derive
// a per-interval byte rate — gives you a live view of whether the network
// stream is actually flowing during a dropout investigation, without
// instrumenting individual HTTP chunks.
static volatile uint32_t s_tts_bytes_total = 0;
uint32_t oe_tts_get_total_bytes_received(void) { return s_tts_bytes_total; }

// Streaming response handler. Server returns raw audio/mpeg body; we feed
// HTTP_EVENT_ON_DATA chunks straight into libhelix instead of buffering
// the whole response. This drops peak memory from ~4 MB (a music-length
// MP3 + base64 JSON wrapper) to one libhelix frame, AND lets playback
// start within ~200 ms of the first byte rather than after the full
// download completes. The old JSON+base64 path is gone — server uses
// `Content-Type: audio/mpeg` uniformly now.
typedef struct {
    mp3_dec_t *dec;
    oe_tts_pcm_cb_t user_cb;
    void *user_user;
    size_t bytes_received;
    volatile bool *abort;   // caller's stop flag; NULL = never abort early
    volatile bool *pause;   // caller's pause flag; when set, read+discard (no decode)
} stream_state_t;

static void pcm_relay_cb(const int16_t *pcm, size_t samples, uint32_t rate, void *u)
{
    stream_state_t *r = (stream_state_t *)u;
    if (r->user_cb) r->user_cb(pcm, samples, rate, r->user_user);
}

static esp_err_t http_evt(esp_http_client_event_t *evt)
{
    stream_state_t *s = (stream_state_t *)evt->user_data;
    if (!s) return ESP_OK;
    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->data && evt->data_len > 0) {
        // Real stop (s_ambient_stop) — abort the perform so the fetch tears down.
        if (s->abort && *(s->abort)) return ESP_FAIL;
        // Count bytes even while paused so [ambient-stats] shows the stream is
        // still alive during a command; only DECODE/play when not paused.
        s->bytes_received += evt->data_len;
        s_tts_bytes_total += evt->data_len;
        // Paused (s_ambient_paused) — keep the socket warm but discard the audio
        // instead of decoding it (frees the speaker for the command/reply). The
        // stream is never torn down, so there is nothing to reconnect.
        if (s->pause && *(s->pause)) return ESP_OK;
        if (s->dec) mp3_dec_feed(s->dec, (const uint8_t *)evt->data, evt->data_len);
    }
    return ESP_OK;
}

esp_err_t oe_tts_post(const char *server_url, const char *token,
                      const char *text, const char *voice, int wake_slot,
                      oe_tts_pcm_cb_t cb, void *user,
                      volatile bool *abort, volatile bool *pause)
{
    // Build the JSON request body (text + optional wake_slot). The server
    // expects this unchanged; only the RESPONSE format is now raw audio.
    cJSON *body = cJSON_CreateObject();
    cJSON_AddStringToObject(body, "text", text);
    if (wake_slot >= 0) cJSON_AddNumberToObject(body, "wake_slot", wake_slot);
    char *body_str = cJSON_PrintUnformatted(body);
    cJSON_Delete(body);
    (void)voice;

    // Set up the MP3 decoder BEFORE the HTTP request so the streaming
    // event handler can feed chunks to it as they arrive. The decoder
    // owns its own working buffer (1152 stereo frames); we don't need to
    // buffer the full network response anywhere.
    stream_state_t state = {
        .dec = NULL,
        .user_cb = cb,
        .user_user = user,
        .bytes_received = 0,
        .abort = abort,
        .pause = pause,
    };
    state.dec = mp3_dec_create(pcm_relay_cb, &state);
    if (!state.dec) { free(body_str); return ESP_ERR_NO_MEM; }

    char url[OE_URL_BUF + 16];
    build_api_url(url, sizeof(url), server_url, "/api/tts");

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .event_handler = http_evt,
        .user_data = &state,
        // Generous timeout because /api/tts on a long input (music test
        // MP3) can stream for tens of seconds before the body completes.
        // For ambient (loop=true) streams the response runs indefinitely;
        // timeout_ms applies to the idle gap between bytes, not total
        // duration, so the keep-alive probes below + steady mp3 flow
        // keep it from tripping.
        .timeout_ms = 60000,
        // TCP keepalive on the HTTP socket. Forces a probe every minute
        // even when the application is just decoding bytes — defeats
        // home-router NAT conntrack timeouts (commonly 30 min) that
        // silently evicted long-lived ambient connections, leaving the
        // device stuck on a dead socket until the next 60s idle timeout.
        // 60s idle → probe every 30s → 3 missed probes → declare dead.
        // Total worst-case detection: 60 + 3*30 = 150s, well inside
        // typical NAT timeout windows.
        .keep_alive_enable = true,
        .keep_alive_idle = 60,
        .keep_alive_interval = 30,
        .keep_alive_count = 3,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);

    char auth[OE_TOKEN_BUF + 16];
    snprintf(auth, sizeof(auth), "Bearer %s", token);
    esp_http_client_set_header(c, "Authorization", auth);
    esp_http_client_set_header(c, "Content-Type", "application/json");
    esp_http_client_set_post_field(c, body_str, strlen(body_str));

    esp_err_t e = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);
    free(body_str);

    mp3_dec_destroy(state.dec);

    if (e != ESP_OK || status != 200) {
        ESP_LOGE(TAG, "tts perform=%d HTTP=%d bytes=%u", (int)e, status, (unsigned)state.bytes_received);
        return e != ESP_OK ? e : ESP_FAIL;
    }

    ESP_LOGI(TAG, "tts: streamed %u mp3 bytes", (unsigned)state.bytes_received);
    return ESP_OK;
}

// Raw-collect variant: stash all body bytes into a growing PSRAM buffer
// instead of streaming through mp3_decode. Used by oe_alarm_fetch_audio
// so the alarm subsystem can hold onto the MP3 across ring cycles without
// hitting the network on every fire.
// Cap collected response body at 512 KB — well above any reasonable chime
// or per-alarm announcement (~12 KB typical), well under PSRAM budget. The
// HTTP perform loop aborts the read once we reach this and reports failure
// upstream so the caller falls back to chime-only / no audio.
#define OE_FETCH_MAX_BYTES (512 * 1024)

typedef struct {
    uint8_t *buf;
    size_t   cap;
    size_t   len;
    bool     overflow;
} raw_collect_t;

static esp_err_t http_raw_evt(esp_http_client_event_t *evt)
{
    raw_collect_t *r = (raw_collect_t *)evt->user_data;
    if (!r) return ESP_OK;
    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->data && evt->data_len > 0) {
        size_t need = r->len + evt->data_len;
        if (need > OE_FETCH_MAX_BYTES) {
            r->overflow = true;
            return ESP_FAIL;  // aborts the http perform loop
        }
        if (need > r->cap) {
            size_t new_cap = r->cap ? r->cap : 8192;
            while (new_cap < need) new_cap *= 2;
            if (new_cap > OE_FETCH_MAX_BYTES) new_cap = OE_FETCH_MAX_BYTES;
            uint8_t *nb = heap_caps_realloc(r->buf, new_cap,
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!nb) nb = realloc(r->buf, new_cap);
            if (!nb) return ESP_FAIL;
            r->buf = nb;
            r->cap = new_cap;
        }
        memcpy(r->buf + r->len, evt->data, evt->data_len);
        r->len += evt->data_len;
    }
    return ESP_OK;
}

esp_err_t oe_alarm_fetch_audio(const char *server_url, const char *token,
                               const char *marker,
                               uint8_t **out_buf, size_t *out_len)
{
    if (!server_url || !token || !marker || !out_buf || !out_len) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_buf = NULL;
    *out_len = 0;

    cJSON *body = cJSON_CreateObject();
    cJSON_AddStringToObject(body, "text", marker);
    char *body_str = cJSON_PrintUnformatted(body);
    cJSON_Delete(body);
    if (!body_str) return ESP_ERR_NO_MEM;

    raw_collect_t r = {0};
    char url[OE_URL_BUF + 16];
    build_api_url(url, sizeof(url), server_url, "/api/tts");

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .event_handler = http_raw_evt,
        .user_data = &r,
        .timeout_ms = 30000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    char auth[OE_TOKEN_BUF + 16];
    snprintf(auth, sizeof(auth), "Bearer %s", token);
    esp_http_client_set_header(c, "Authorization", auth);
    esp_http_client_set_header(c, "Content-Type", "application/json");
    esp_http_client_set_post_field(c, body_str, strlen(body_str));

    esp_err_t e = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);
    free(body_str);

    if (r.overflow) {
        ESP_LOGW(TAG, "alarm fetch exceeded %d KB cap — aborting",
                 OE_FETCH_MAX_BYTES / 1024);
        if (r.buf) heap_caps_free(r.buf);
        return ESP_ERR_INVALID_SIZE;
    }
    if (e != ESP_OK || status != 200 || r.len == 0) {
        ESP_LOGE(TAG, "alarm fetch perform=%d HTTP=%d bytes=%u",
                 (int)e, status, (unsigned)r.len);
        if (r.buf) heap_caps_free(r.buf);
        return e != ESP_OK ? e : ESP_FAIL;
    }

    ESP_LOGI(TAG, "alarm fetch: %u mp3 bytes", (unsigned)r.len);
    *out_buf = r.buf;
    *out_len = r.len;
    return ESP_OK;
}
