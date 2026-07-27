#include "oe_client.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "oe_stt";

static void build_api_url(char *out, size_t out_len, const char *server_url, const char *path)
{
    size_t n = strnlen(server_url, OE_URL_BUF);
    while (n > 0 && server_url[n - 1] == '/') n--;
    snprintf(out, out_len, "%.*s%s", (int)n, server_url, path);
}

static bool private_ipv4_authority(const char *authority, size_t len)
{
    size_t host_len = len;
    for (size_t i = 0; i < len; ++i) {
        if (authority[i] != ':') continue;
        host_len = i;
        if (++i >= len) return false;
        unsigned port = 0;
        for (; i < len; ++i) {
            if (authority[i] < '0' || authority[i] > '9') return false;
            const unsigned digit = (unsigned)(authority[i] - '0');
            if (port > 6553u || (port == 6553u && digit > 5u)) return false;
            port = port * 10u + digit;
        }
        if (port == 0) return false;
        break;
    }

    unsigned octets[4] = {0};
    size_t pos = 0;
    for (size_t part = 0; part < 4; ++part) {
        if (pos >= host_len) return false;
        // Require canonical dotted-decimal octets. lwIP's IPv4 parser treats
        // a leading zero as octal, so accepting "010.0.0.1" here as decimal
        // 10/8 would let the transport resolve the supposedly-private origin
        // as public 8.0.0.1.
        if (authority[pos] == '0' && pos + 1 < host_len &&
            authority[pos + 1] >= '0' && authority[pos + 1] <= '9') {
            return false;
        }
        unsigned value = 0;
        size_t digits = 0;
        while (pos < host_len && authority[pos] >= '0' &&
               authority[pos] <= '9') {
            value = value * 10u + (unsigned)(authority[pos] - '0');
            if (value > 255u || ++digits > 3) return false;
            pos++;
        }
        if (digits == 0) return false;
        octets[part] = value;
        if (part < 3) {
            if (pos >= host_len || authority[pos++] != '.') return false;
        } else if (pos != host_len) {
            return false;
        }
    }
    return octets[0] == 10 ||
           (octets[0] == 172 && octets[1] >= 16 && octets[1] <= 31) ||
           (octets[0] == 192 && octets[1] == 168);
}

bool oe_verify_gate_origin_allowed(const char *server_url)
{
    if (!server_url) return false;
    size_t prefix_len;
    bool private_http = false;
    if (strncmp(server_url, "https://", 8) == 0) {
        prefix_len = 8;
    } else if (strncmp(server_url, "http://", 7) == 0) {
        prefix_len = 7;
        private_http = true;
    } else {
        return false;
    }
    size_t n = strnlen(server_url, OE_URL_BUF);
    if (n <= prefix_len || n >= OE_URL_BUF) return false;
    while (n > prefix_len && server_url[n - 1] == '/') n--;
    if (n == prefix_len) return false;

    // The paired value must be an origin, not a second path or a URL whose
    // authority could capture the bearer through userinfo.
    for (size_t i = prefix_len; i < n; ++i) {
        const unsigned char c = (unsigned char)server_url[i];
        if (c <= 0x20 || c == 0x7f || c == '/' || c == '?' ||
            c == '#' || c == '@' || c == '\\') {
            return false;
        }
    }
    return !private_http ||
           private_ipv4_authority(server_url + prefix_len, n - prefix_len);
}

// Build the verifier URL only from the paired OE origin and the single
// protocol-defined path. Validation runs before allocating/copying audio or
// constructing Authorization. Plain HTTP is limited to numeric private-LAN
// IPv4, so a stale public/free-form HTTP origin cannot receive the bearer.
static bool build_verify_url(char *out, size_t out_len,
                             const char *server_url, const char *gate_path)
{
    if (!out || out_len == 0 || !server_url || !gate_path ||
        strcmp(gate_path, OE_VERIFY_GATE_PATH) != 0 ||
        !oe_verify_gate_origin_allowed(server_url)) {
        return false;
    }
    size_t n = strnlen(server_url, OE_URL_BUF);
    while (n > 0 && server_url[n - 1] == '/') n--;
    int written = snprintf(out, out_len, "%.*s%s",
                           (int)n, server_url, OE_VERIFY_GATE_PATH);
    return written > 0 && (size_t)written < out_len;
}

#define MP_BOUNDARY "----oevdfilewavboundary7f3"

static void write_wav_header(uint8_t *hdr, uint32_t pcm_bytes)
{
    uint32_t sample_rate = 16000;
    uint16_t channels = 1;
    uint16_t bits = 16;
    uint32_t byte_rate = sample_rate * channels * (bits / 8);
    uint16_t block_align = channels * (bits / 8);

    memcpy(hdr +  0, "RIFF", 4);
    uint32_t total = 36 + pcm_bytes;
    memcpy(hdr +  4, &total, 4);
    memcpy(hdr +  8, "WAVE", 4);
    memcpy(hdr + 12, "fmt ", 4);
    uint32_t fmt_size = 16;
    memcpy(hdr + 16, &fmt_size, 4);
    uint16_t fmt_type = 1;
    memcpy(hdr + 20, &fmt_type, 2);
    memcpy(hdr + 22, &channels, 2);
    memcpy(hdr + 24, &sample_rate, 4);
    memcpy(hdr + 28, &byte_rate, 4);
    memcpy(hdr + 32, &block_align, 2);
    memcpy(hdr + 34, &bits, 2);
    memcpy(hdr + 36, "data", 4);
    memcpy(hdr + 40, &pcm_bytes, 4);
}

typedef struct {
    char *buf;
    size_t len;
    size_t cap;
    bool overflow;
} resp_buf_t;

static esp_err_t http_evt(esp_http_client_event_t *evt)
{
    resp_buf_t *r = (resp_buf_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->data && evt->data_len > 0) {
        if (r->len + evt->data_len + 1 > r->cap) {
            r->overflow = true;
            return ESP_OK;
        }
        memcpy(r->buf + r->len, evt->data, evt->data_len);
        r->len += evt->data_len;
        r->buf[r->len] = 0;
    }
    return ESP_OK;
}

esp_err_t oe_stt_post(const char *server_url, const char *token,
                      const int16_t *pcm_16k_mono, size_t n_samples,
                      char *out_text, size_t out_len)
{
    if (!server_url || !token || !pcm_16k_mono || !out_text || out_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    out_text[0] = 0;

    uint32_t pcm_bytes = (uint32_t)(n_samples * sizeof(int16_t));
    const char *prelude_fmt =
        "--" MP_BOUNDARY "\r\n"
        "Content-Disposition: form-data; name=\"audio\"; filename=\"speech.wav\"\r\n"
        "Content-Type: audio/wav\r\n\r\n";
    const char *epilogue = "\r\n--" MP_BOUNDARY "--\r\n";
    uint8_t wav_hdr[44];
    write_wav_header(wav_hdr, pcm_bytes);

    size_t prelude_len = strlen(prelude_fmt);
    size_t epilogue_len = strlen(epilogue);
    size_t total = prelude_len + sizeof(wav_hdr) + pcm_bytes + epilogue_len;

    uint8_t *body = heap_caps_malloc(total, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!body) return ESP_ERR_NO_MEM;
    size_t off = 0;
    memcpy(body + off, prelude_fmt, prelude_len);          off += prelude_len;
    memcpy(body + off, wav_hdr, sizeof(wav_hdr));          off += sizeof(wav_hdr);
    memcpy(body + off, pcm_16k_mono, pcm_bytes);           off += pcm_bytes;
    memcpy(body + off, epilogue, epilogue_len);            off += epilogue_len;

    char url[OE_URL_BUF + 16];
    build_api_url(url, sizeof(url), server_url, "/api/stt");

    char resp[1024];
    resp_buf_t rb = { .buf = resp, .len = 0, .cap = sizeof(resp) };

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .event_handler = http_evt,
        .user_data = &rb,
        .timeout_ms = 30000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) {
        free(body);
        return ESP_ERR_NO_MEM;
    }

    char auth[OE_TOKEN_BUF + 16];
    snprintf(auth, sizeof(auth), "Bearer %s", token);
    esp_http_client_set_header(c, "Authorization", auth);
    esp_http_client_set_header(c, "Content-Type", "multipart/form-data; boundary=" MP_BOUNDARY);
    esp_http_client_set_post_field(c, (const char *)body, total);

    esp_err_t e = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);
    free(body);

    if (e != ESP_OK) { ESP_LOGE(TAG, "stt perform: %s", esp_err_to_name(e)); return e; }
    if (status != 200) { ESP_LOGE(TAG, "stt HTTP %d", status); return ESP_FAIL; }

    cJSON *j = cJSON_ParseWithLength(rb.buf, rb.len);
    if (!j) return ESP_FAIL;
    const cJSON *jt = cJSON_GetObjectItem(j, "transcript");
    if (!jt || !cJSON_IsString(jt)) jt = cJSON_GetObjectItem(j, "text");
    if (jt && cJSON_IsString(jt)) {
        snprintf(out_text, out_len, "%s", jt->valuestring);
    }
    cJSON_Delete(j);
    ESP_LOGI(TAG, "stt: \"%s\"", out_text);
    return ESP_OK;
}

// ── Wake-word verify gate upload ─────────────────────────────────────────────
// Adapted from the removed diagnostic oe_wake_capture_post (oe_stt.c.bak-
// rmwakecap-20260622-195049): same WAV-in-PSRAM build + esp_http_client POST,
// but retargeted to OE's authenticated verify-proxy contract and made
// verdict-returning + non-persisting (no /api/wake-capture, no disk).
#define VG_BOUNDARY "----oevdverifygateboundary9k2"

// --- verify-gate POST abort (self-heal for a wedged gate request) ----------
// A gate POST that wedges on a half-open socket (e.g. the OE server mid-
// restart) would otherwise block verify_worker_task forever, pinning
// s_verify_inflight and deafening the device to every wake. The capture task's
// watchdog and the WS-disconnect handler call oe_verify_gate_abort() to close
// the in-flight connection, which unblocks esp_http_client_perform() so the
// worker can unwind and clear the flag IN ORDER (it still owns the wake-window
// buffer). The mutex serializes close() against the worker's cleanup(); the
// perform() itself runs WITHOUT the mutex so the aborter can take it.
static SemaphoreHandle_t s_vg_client_mtx = NULL;
static esp_http_client_handle_t s_vg_client = NULL;

void oe_verify_gate_init(void)
{
    if (!s_vg_client_mtx) s_vg_client_mtx = xSemaphoreCreateMutex();
}

void oe_verify_gate_abort(void)
{
    if (!s_vg_client_mtx) return;
    xSemaphoreTake(s_vg_client_mtx, portMAX_DELAY);
    if (s_vg_client) esp_http_client_close(s_vg_client);
    xSemaphoreGive(s_vg_client_mtx);
}

esp_err_t oe_verify_gate_post(const char *server_url, const char *gate_path,
                              const char *token,
                              const char *session_id, const char *device_id,
                              const char *wake_words_json, float detector_score,
                              const char *fired_at,
                              const int16_t *pcm_16k_mono, size_t n_samples,
                              int timeout_ms, oe_verify_result_t *out_effective)
{
    if (out_effective) *out_effective = OE_VERIFY_ERROR;
    if (!server_url || !server_url[0] || !gate_path || !gate_path[0] ||
        !token || !token[0] ||
        !session_id || !session_id[0] || !device_id || !device_id[0] ||
        !wake_words_json || !wake_words_json[0] ||
        !pcm_16k_mono || n_samples == 0 || !out_effective) {
        return ESP_ERR_INVALID_ARG;
    }

    char url[OE_URL_BUF + sizeof(OE_VERIFY_GATE_PATH)];
    if (!build_verify_url(url, sizeof(url), server_url, gate_path)) {
        ESP_LOGW(TAG, "verify: paired OE origin/path is not allowed (fail-closed)");
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t pcm_bytes = (uint32_t)(n_samples * sizeof(int16_t));
    uint8_t wav_hdr[44];
    write_wav_header(wav_hdr, pcm_bytes);

    // Text form fields (session_id, device_id, wake_words, optional
    // detector_score + fired_at) as one contiguous multipart prelude.
    char fields[768];
    int fl = 0;
    fl += snprintf(fields + fl, sizeof(fields) - fl,
        "--" VG_BOUNDARY "\r\nContent-Disposition: form-data; name=\"session_id\"\r\n\r\n%s\r\n", session_id);
    fl += snprintf(fields + fl, sizeof(fields) - fl,
        "--" VG_BOUNDARY "\r\nContent-Disposition: form-data; name=\"device_id\"\r\n\r\n%s\r\n", device_id);
    fl += snprintf(fields + fl, sizeof(fields) - fl,
        "--" VG_BOUNDARY "\r\nContent-Disposition: form-data; name=\"wake_words\"\r\n\r\n%s\r\n", wake_words_json);
    if (detector_score >= 0.0f) {
        fl += snprintf(fields + fl, sizeof(fields) - fl,
            "--" VG_BOUNDARY "\r\nContent-Disposition: form-data; name=\"detector_score\"\r\n\r\n%.4f\r\n",
            (double)detector_score);
    }
    if (fired_at && fired_at[0]) {
        fl += snprintf(fields + fl, sizeof(fields) - fl,
            "--" VG_BOUNDARY "\r\nContent-Disposition: form-data; name=\"fired_at\"\r\n\r\n%s\r\n", fired_at);
    }
    if (fl < 0 || (size_t)fl >= sizeof(fields)) {
        ESP_LOGW(TAG, "verify: form fields overflow — fail-closed");
        return ESP_ERR_INVALID_SIZE;
    }

    const char *file_hdr =
        "--" VG_BOUNDARY "\r\n"
        "Content-Disposition: form-data; name=\"file\"; filename=\"wake.wav\"\r\n"
        "Content-Type: audio/wav\r\n\r\n";
    const char *epilogue = "\r\n--" VG_BOUNDARY "--\r\n";
    size_t fields_len = (size_t)fl;
    size_t fhdr_len   = strlen(file_hdr);
    size_t epi_len    = strlen(epilogue);
    size_t total = fields_len + fhdr_len + sizeof(wav_hdr) + pcm_bytes + epi_len;

    uint8_t *body = heap_caps_malloc(total, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!body) return ESP_ERR_NO_MEM;
    size_t off = 0;
    memcpy(body + off, fields, fields_len);        off += fields_len;
    memcpy(body + off, file_hdr, fhdr_len);        off += fhdr_len;
    memcpy(body + off, wav_hdr, sizeof(wav_hdr));   off += sizeof(wav_hdr);
    memcpy(body + off, pcm_16k_mono, pcm_bytes);    off += pcm_bytes;
    memcpy(body + off, epilogue, epi_len);          off += epi_len;

    char resp[1024] = "";
    resp_buf_t rb = { .buf = resp, .len = 0, .cap = sizeof(resp) };

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .event_handler = http_evt,
        .user_data = &rb,
        .timeout_ms = timeout_ms > 0 ? timeout_ms : 1200,
        .crt_bundle_attach = esp_crt_bundle_attach,
        // The fixed same-origin endpoint is part of the bearer boundary.
        // A redirect must fail closed rather than forwarding credentials or
        // wake audio to a Location chosen by an intermediary/server.
        .disable_auto_redirect = true,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) { free(body); return ESP_ERR_NO_MEM; }

    char auth[OE_TOKEN_BUF + 16];
    snprintf(auth, sizeof(auth), "Bearer %s", token ? token : "");
    esp_http_client_set_header(c, "Authorization", auth);
    esp_http_client_set_header(c, "Content-Type", "multipart/form-data; boundary=" VG_BOUNDARY);
    esp_http_client_set_post_field(c, (const char *)body, total);

    // Publish the handle so oe_verify_gate_abort() can close it if this POST
    // wedges. perform() runs WITHOUT the mutex (so the aborter can take it);
    // cleanup() runs UNDER the mutex so it can never race a concurrent close().
    if (s_vg_client_mtx) {
        xSemaphoreTake(s_vg_client_mtx, portMAX_DELAY);
        s_vg_client = c;
        xSemaphoreGive(s_vg_client_mtx);
    }
    esp_err_t e = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    if (s_vg_client_mtx) {
        xSemaphoreTake(s_vg_client_mtx, portMAX_DELAY);
        s_vg_client = NULL;
        esp_http_client_cleanup(c);
        xSemaphoreGive(s_vg_client_mtx);
    } else {
        esp_http_client_cleanup(c);
    }
    free(body);

    if (e != ESP_OK) { ESP_LOGW(TAG, "verify perform: %s (fail-closed)", esp_err_to_name(e)); return e; }
    if (status < 200 || status >= 300) { ESP_LOGW(TAG, "verify HTTP %d (fail-closed)", status); return ESP_FAIL; }
    if (rb.overflow) {
        ESP_LOGW(TAG, "verify: response too large (fail-closed)");
        return ESP_ERR_INVALID_SIZE;
    }

    cJSON *j = cJSON_ParseWithLength(rb.buf, rb.len);
    if (!j) { ESP_LOGW(TAG, "verify: body parse failed (fail-closed)"); return ESP_FAIL; }
    const cJSON *je = cJSON_GetObjectItem(j, "effective");
    const cJSON *jv = cJSON_GetObjectItem(j, "verdict");
    const cJSON *jm = cJSON_GetObjectItem(j, "mode");
    const cJSON *js = cJSON_GetObjectItem(j, "session_id");
    const cJSON *jd = cJSON_GetObjectItem(j, "device_id");
    esp_err_t ret = ESP_OK;
    const bool session_matches =
        cJSON_IsString(js) && js->valuestring &&
        strcmp(js->valuestring, session_id) == 0;
    const bool device_matches =
        cJSON_IsString(jd) && jd->valuestring &&
        strcmp(jd->valuestring, device_id) == 0;
    if (!session_matches || !device_matches) {
        // A verdict is owned by the composite (device, session) identity.
        // Checking only session_id lets another device with the same per-boot
        // turn id consume a cached ACCEPT that was computed for this request.
        ESP_LOGW(TAG,
                 "verify: response identity mismatch (session=%s device=%s; fail-closed)",
                 session_matches ? "match" : "mismatch",
                 device_matches ? "match" : "mismatch");
        *out_effective = OE_VERIFY_ERROR;
        ret = ESP_ERR_INVALID_RESPONSE;
    } else {
        const bool verdict_valid =
            cJSON_IsString(jv) && jv->valuestring &&
            (strcmp(jv->valuestring, "accept") == 0 ||
             strcmp(jv->valuestring, "reject") == 0);
        const bool mode_valid =
            cJSON_IsString(jm) && jm->valuestring &&
            (strcmp(jm->valuestring, "shadow") == 0 ||
             strcmp(jm->valuestring, "enforce") == 0);
        const char *expected_effective =
            mode_valid && strcmp(jm->valuestring, "shadow") == 0
                ? "accept"
                : (verdict_valid ? jv->valuestring : NULL);
        const bool effective_valid =
            cJSON_IsString(je) && je->valuestring && expected_effective &&
            strcmp(je->valuestring, expected_effective) == 0;
        if (!verdict_valid || !mode_valid || !effective_valid) {
            ESP_LOGW(TAG, "verify: inconsistent verdict tuple (fail-closed)");
            *out_effective = OE_VERIFY_ERROR;
            ret = ESP_ERR_INVALID_RESPONSE;
        } else {
            *out_effective = strcmp(je->valuestring, "accept") == 0
                                 ? OE_VERIFY_ACCEPT
                                 : OE_VERIFY_REJECT;
        }
    }
    // Log verdict metadata only — NEVER any transcript (the body carries none).
    ESP_LOGI(TAG, "verify: effective=%s verdict=%s mode=%s",
             (cJSON_IsString(je) ? je->valuestring : "?"),
             (cJSON_IsString(jv) ? jv->valuestring : "?"),
             (cJSON_IsString(jm) ? jm->valuestring : "?"));
    cJSON_Delete(j);
    return ret;
}
