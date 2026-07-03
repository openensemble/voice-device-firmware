#include "oe_client.h"

#include <string.h>
#include <stdio.h>
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "cJSON.h"

static const char *TAG = "oe_pair";

static void build_api_url(char *out, size_t out_len, const char *server_url, const char *path)
{
    size_t n = strnlen(server_url, OE_URL_BUF);
    while (n > 0 && server_url[n - 1] == '/') n--;
    snprintf(out, out_len, "%.*s%s", (int)n, server_url, path);
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
        size_t want = r->len + evt->data_len + 1;
        if (want > r->cap) return ESP_OK;
        memcpy(r->buf + r->len, evt->data, evt->data_len);
        r->len += evt->data_len;
        r->buf[r->len] = 0;
    }
    return ESP_OK;
}

esp_err_t oe_pair_redeem(const char *server_url, const char *pair_code,
                         const char *device_name, oe_pair_result_t *out)
{
    char url[OE_URL_BUF + 32];
    build_api_url(url, sizeof(url), server_url, "/api/devices/redeem");

    cJSON *body = cJSON_CreateObject();
    cJSON_AddStringToObject(body, "code", pair_code);
    cJSON_AddStringToObject(body, "device_name", device_name ? device_name : "voice-device");
    char *body_str = cJSON_PrintUnformatted(body);
    cJSON_Delete(body);

    char resp[1024];
    resp_buf_t rb = { .buf = resp, .len = 0, .cap = sizeof(resp) };

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .event_handler = http_evt,
        .user_data = &rb,
        .timeout_ms = 10000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    esp_http_client_set_header(c, "Content-Type", "application/json");
    esp_http_client_set_post_field(c, body_str, strlen(body_str));

    esp_err_t e = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);
    free(body_str);

    if (e != ESP_OK) { ESP_LOGE(TAG, "redeem perform: %s", esp_err_to_name(e)); return e; }
    if (status != 200) {
        ESP_LOGE(TAG, "redeem HTTP %d body=%.*s", status, (int)rb.len, rb.buf);
        return ESP_FAIL;
    }

    cJSON *j = cJSON_ParseWithLength(rb.buf, rb.len);
    if (!j) return ESP_FAIL;
    const cJSON *jt = cJSON_GetObjectItem(j, "token");
    const cJSON *ju = cJSON_GetObjectItem(j, "userId");
    const cJSON *jh = cJSON_GetObjectItem(j, "server_hint");
    if (jt && cJSON_IsString(jt)) strncpy(out->token, jt->valuestring, sizeof(out->token) - 1);
    if (ju && cJSON_IsString(ju)) strncpy(out->user_id, ju->valuestring, sizeof(out->user_id) - 1);
    if (jh && cJSON_IsString(jh)) strncpy(out->server_hint, jh->valuestring, sizeof(out->server_hint) - 1);
    cJSON_Delete(j);

    if (out->token[0] == 0) {
        ESP_LOGE(TAG, "redeem returned no token");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "redeem ok");
    return ESP_OK;
}
