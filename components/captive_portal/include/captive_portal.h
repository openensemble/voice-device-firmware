#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    char ssid[64];
    char password[64];
    char server_url[256];
    char pair_code[16];
    char device_name[64];
} captive_form_result_t;

typedef void (*captive_submit_callback_t)(const captive_form_result_t *r, void *user);

esp_err_t captive_portal_start(const char *ap_ssid, captive_submit_callback_t cb, void *user);
esp_err_t captive_portal_stop(void);
bool      captive_portal_running(void);
