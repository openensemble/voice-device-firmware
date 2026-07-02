#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#define NVS_CREDS_NAMESPACE "oe_voice"

esp_err_t nvs_creds_init(void);

esp_err_t nvs_creds_set_wifi(const char *ssid, const char *password);
esp_err_t nvs_creds_get_wifi(char *ssid, size_t ssid_len, char *password, size_t password_len);
bool      nvs_creds_has_wifi(void);

esp_err_t nvs_creds_set_server(const char *url);
esp_err_t nvs_creds_get_server(char *url, size_t url_len);

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

esp_err_t nvs_creds_set_server_cert(const uint8_t *pem, size_t pem_len);
esp_err_t nvs_creds_get_server_cert(uint8_t *pem, size_t *pem_len);

bool nvs_creds_is_provisioned(void);
esp_err_t nvs_creds_factory_reset(void);
