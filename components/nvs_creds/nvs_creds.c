#include "nvs_creds.h"

#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "nvs_creds";

esp_err_t nvs_creds_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

static esp_err_t set_str(const char *key, const char *value)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_CREDS_NAMESPACE, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_str(h, key, value);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

static esp_err_t get_str(const char *key, char *out, size_t out_len)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_CREDS_NAMESPACE, NVS_READONLY, &h);
    if (e != ESP_OK) return e;
    size_t needed = out_len;
    e = nvs_get_str(h, key, out, &needed);
    nvs_close(h);
    return e;
}

static esp_err_t set_u8(const char *key, uint8_t v)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_CREDS_NAMESPACE, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_u8(h, key, v);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

static esp_err_t get_u8(const char *key, uint8_t *v)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_CREDS_NAMESPACE, NVS_READONLY, &h);
    if (e != ESP_OK) return e;
    e = nvs_get_u8(h, key, v);
    nvs_close(h);
    return e;
}

esp_err_t nvs_creds_set_wifi(const char *ssid, const char *password)
{
    esp_err_t e = set_str("wifi_ssid", ssid);
    if (e != ESP_OK) return e;
    return set_str("wifi_pwd", password);
}

esp_err_t nvs_creds_get_wifi(char *ssid, size_t ssid_len, char *password, size_t password_len)
{
    esp_err_t e = get_str("wifi_ssid", ssid, ssid_len);
    if (e != ESP_OK) return e;
    return get_str("wifi_pwd", password, password_len);
}

bool nvs_creds_has_wifi(void)
{
    char ssid[64];
    return get_str("wifi_ssid", ssid, sizeof(ssid)) == ESP_OK;
}

bool nvs_creds_normalize_server_url(char *url)
{
    if (!url) return false;
    bool changed = false;
    // RFC 3986: the scheme is case-insensitive and should be normalized to
    // lowercase. Only the scheme is touched — the authority is left exactly as
    // provisioned. A value typed as "HTTP://" is otherwise persisted verbatim,
    // leaving every consumer of server_url obliged to remember a
    // case-insensitive compare; the one that forgot (the verify-gate origin
    // check) silently gated a device off for nine days.
    for (size_t i = 0; url[i]; ++i) {
        if (url[i] == ':') break;              // end of scheme
        if (url[i] >= 'A' && url[i] <= 'Z') {
            url[i] = (char)(url[i] - 'A' + 'a');
            changed = true;
        }
    }
    return changed;
}

esp_err_t nvs_creds_set_server(const char *url)
{
    if (!url) return ESP_ERR_INVALID_ARG;
    char norm[NVS_CREDS_SERVER_URL_MAX];
    if (strlen(url) >= sizeof(norm)) return ESP_ERR_INVALID_SIZE;
    strcpy(norm, url);
    nvs_creds_normalize_server_url(norm);
    return set_str("server", norm);
}
esp_err_t nvs_creds_get_server(char *url, size_t n) { return get_str("server", url, n); }

esp_err_t nvs_creds_set_verify_gate_path(const char *path)     { return set_str("vgate_url", path); }
esp_err_t nvs_creds_get_verify_gate_path(char *path, size_t n) { return get_str("vgate_url", path, n); }

esp_err_t nvs_creds_set_device_id(const char *device_id)     { return set_str("device_id", device_id); }
esp_err_t nvs_creds_get_device_id(char *device_id, size_t n) { return get_str("device_id", device_id, n); }

esp_err_t nvs_creds_set_verify_gate_config(const char *device_id, const char *path)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_CREDS_NAMESPACE, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    // NVS updates are not a transactional pair. Write policy first: if power
    // fails while enabling, the next boot sees an enabled path with a missing
    // or stale identity and therefore blocks. Writing identity first could
    // leave the prior empty path and silently bypass verification.
    e = nvs_set_str(h, "vgate_url", path);
    if (e == ESP_OK) e = nvs_set_str(h, "device_id", device_id);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

esp_err_t nvs_creds_set_token(const char *token)     { return set_str("token", token); }
esp_err_t nvs_creds_get_token(char *token, size_t n) { return get_str("token", token, n); }

esp_err_t nvs_creds_set_device_name(const char *name)     { return set_str("dev_name", name); }
esp_err_t nvs_creds_get_device_name(char *name, size_t n) { return get_str("dev_name", name, n); }

esp_err_t nvs_creds_set_default_agent(const char *a)     { return set_str("def_agent", a); }
esp_err_t nvs_creds_get_default_agent(char *a, size_t n) { return get_str("def_agent", a, n); }

esp_err_t nvs_creds_set_wake_slot(uint8_t s)  { return set_u8("wake_slot", s); }
esp_err_t nvs_creds_get_wake_slot(uint8_t *s) { return get_u8("wake_slot", s); }

esp_err_t nvs_creds_set_tts_voice(uint8_t v)  { return set_u8("tts_voice", v); }
esp_err_t nvs_creds_get_tts_voice(uint8_t *v) { return get_u8("tts_voice", v); }

esp_err_t nvs_creds_set_volume(uint8_t pct)  { return set_u8("volume", pct); }
esp_err_t nvs_creds_get_volume(uint8_t *pct) { return get_u8("volume", pct); }

esp_err_t nvs_creds_set_headphone_mode(uint8_t e)  { return set_u8("hp_mode", e); }
esp_err_t nvs_creds_get_headphone_mode(uint8_t *e) { return get_u8("hp_mode", e); }

esp_err_t nvs_creds_set_ota_retry_state(uint8_t state)
{
    if (state > 2) return ESP_ERR_INVALID_ARG;
    if (state != 0) return set_u8("ota_retry", state);

    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_CREDS_NAMESPACE, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_erase_key(h, "ota_retry");
    if (e == ESP_ERR_NVS_NOT_FOUND) e = ESP_OK;
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

esp_err_t nvs_creds_get_ota_retry_state(uint8_t *state)
{
    if (!state) return ESP_ERR_INVALID_ARG;
    esp_err_t e = get_u8("ota_retry", state);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        *state = 0;
        return ESP_OK;
    }
    return e;
}

esp_err_t nvs_creds_set_server_cert(const uint8_t *pem, size_t pem_len)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_CREDS_NAMESPACE, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_blob(h, "srv_cert", pem, pem_len);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

esp_err_t nvs_creds_get_server_cert(uint8_t *pem, size_t *pem_len)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_CREDS_NAMESPACE, NVS_READONLY, &h);
    if (e != ESP_OK) return e;
    e = nvs_get_blob(h, "srv_cert", pem, pem_len);
    nvs_close(h);
    return e;
}

bool nvs_creds_is_provisioned(void)
{
    // Use the size-probe form (NULL value + size_t *length): nvs_get_str
    // returns ESP_OK if the key exists and writes the required size into
    // *length. We don't care about the actual values here — just whether
    // both keys are present. The old form used 8-byte stack buffers, which
    // were smaller than the stored values (64-hex token + URL), so
    // nvs_get_str returned ESP_ERR_NVS_INVALID_LENGTH and is_provisioned
    // always reported false even after a successful pair.
    nvs_handle_t h;
    if (nvs_open(NVS_CREDS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;
    size_t tok_len = 0, srv_len = 0;
    esp_err_t et = nvs_get_str(h, "token",  NULL, &tok_len);
    esp_err_t es = nvs_get_str(h, "server", NULL, &srv_len);
    nvs_close(h);
    return et == ESP_OK && es == ESP_OK && tok_len > 1 && srv_len > 1;
}

esp_err_t nvs_creds_factory_reset(void)
{
    ESP_LOGW(TAG, "factory reset — wiping NVS credentials");
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_CREDS_NAMESPACE, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
    return ESP_OK;
}
