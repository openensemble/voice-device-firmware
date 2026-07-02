#include "oe_client.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "cJSON.h"

static const char *TAG = "oe_stt";

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
} resp_buf_t;

static esp_err_t http_evt(esp_http_client_event_t *evt)
{
    resp_buf_t *r = (resp_buf_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->data && evt->data_len > 0) {
        if (r->len + evt->data_len + 1 > r->cap) return ESP_OK;
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
    snprintf(url, sizeof(url), "%s/api/stt", server_url);

    char resp[1024];
    resp_buf_t rb = { .buf = resp, .len = 0, .cap = sizeof(resp) };

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .event_handler = http_evt,
        .user_data = &rb,
        .timeout_ms = 30000,
        .skip_cert_common_name_check = true,
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
