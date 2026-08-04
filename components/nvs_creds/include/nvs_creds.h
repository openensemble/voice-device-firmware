#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#define NVS_CREDS_NAMESPACE "oe_voice"

esp_err_t nvs_creds_init(void);

esp_err_t nvs_creds_set_wifi(const char *ssid, const char *password);
esp_err_t nvs_creds_get_wifi(char *ssid, size_t ssid_len, char *password, size_t password_len);
bool      nvs_creds_has_wifi(void);

// Matches OE_URL_BUF in oe_client.h. Kept as its own constant so this
// low-level component does not have to depend on the client header.
#define NVS_CREDS_SERVER_URL_MAX 256

// Lowercase the URL scheme in place (RFC 3986 makes it case-insensitive and
// calls for lowercase); the authority is left untouched. Returns true if the
// value changed. nvs_creds_set_server() applies this before writing, so every
// consumer of server_url can rely on a lowercase scheme regardless of how it
// was typed at provisioning. Exposed so boot can heal a value stored before
// this normalization existed.
bool nvs_creds_normalize_server_url(char *url);

esp_err_t nvs_creds_set_server(const char *url);
esp_err_t nvs_creds_get_server(char *url, size_t url_len);

// Exact OE verify-proxy path (empty/absent = gate disabled). The legacy
// "vgate_url" NVS key is intentionally retained so upgrades can migrate a
// previously provisioned direct verifier URL without silently bypassing it.
esp_err_t nvs_creds_set_verify_gate_path(const char *path);
esp_err_t nvs_creds_get_verify_gate_path(char *path, size_t path_len);

// Canonical OE voice-device id. Gate path + id are persisted in fail-closed
// order when they arrive together in server_caps (NVS is not transactional).
esp_err_t nvs_creds_set_device_id(const char *device_id);
esp_err_t nvs_creds_get_device_id(char *device_id, size_t device_id_len);
esp_err_t nvs_creds_set_verify_gate_config(const char *device_id, const char *path);

esp_err_t nvs_creds_set_token(const char *token);
esp_err_t nvs_creds_get_token(char *token, size_t token_len);

esp_err_t nvs_creds_set_device_name(const char *name);
esp_err_t nvs_creds_get_device_name(char *name, size_t name_len);

esp_err_t nvs_creds_set_default_agent(const char *agent_id);
esp_err_t nvs_creds_get_default_agent(char *agent_id, size_t agent_id_len);

esp_err_t nvs_creds_set_wake_slot(uint8_t slot);
esp_err_t nvs_creds_get_wake_slot(uint8_t *slot);

esp_err_t nvs_creds_set_tts_voice(uint8_t voice);
esp_err_t nvs_creds_get_tts_voice(uint8_t *voice);

esp_err_t nvs_creds_set_volume(uint8_t pct);
esp_err_t nvs_creds_get_volume(uint8_t *pct);

esp_err_t nvs_creds_set_headphone_mode(uint8_t enabled);
esp_err_t nvs_creds_get_headphone_mode(uint8_t *enabled);

// One-shot OTA memory-recovery latch. State is persisted because the recovery
// deliberately crosses esp_restart(): 0=idle, 1=pending reboot resume,
// 2=recovery attempt consumed. Writing 0 erases the key.
esp_err_t nvs_creds_set_ota_retry_state(uint8_t state);
esp_err_t nvs_creds_get_ota_retry_state(uint8_t *state);

esp_err_t nvs_creds_set_server_cert(const uint8_t *pem, size_t pem_len);
esp_err_t nvs_creds_get_server_cert(uint8_t *pem, size_t *pem_len);

bool nvs_creds_is_provisioned(void);
esp_err_t nvs_creds_factory_reset(void);
