/**
 * Firmware OTA flow for the OE voice device.
 *
 * Triggered by a server WS message {type:'ota_check'} (see main.c
 * OE_WS_EVT_OTA_CHECK handler). Flow:
 *
 *   1. GET <server>/firmware/voice-device/manifest.json
 *   2. Compare manifest.version (e.g. "0.2.3") to the running app's version
 *      from esp_app_get_description(). If equal-or-older, emit "up_to_date"
 *      and return — never thrash the flash.
 *   3. esp_https_ota_begin against <server>/firmware/voice-device/oe_voice_device.bin,
 *      pull chunks via esp_https_ota_perform until ESP_OK, periodically
 *      reporting progress over WS (so the browser UI can show %).
 *   4. esp_https_ota_finish writes the boot-sector switch and we
 *      esp_restart() into the new app.
 *   5. Next boot: oe_ota_mark_running_valid (called from main.c once Wi-Fi +
 *      WS are alive) cancels the pending rollback. If the new app crashes
 *      before that ever happens, IDF reverts to the previous slot on the
 *      following boot — that's our automatic safety net.
 *
 * Public/no-auth: the firmware fetches over plain HTTP (or HTTPS if the
 * pairing-time server_url uses https://) without sending the device token.
 * The manifest + binary live in OE's public/ tree, which the server serves
 * without auth (see server.mjs:300 firmware static handler).
 */
#include "oe_client.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "esp_partition.h"
#include "mbedtls/sha256.h"

static const char *TAG = "oe_ota";

// One-at-a-time guard. Both the WS event and any future auto-check entry
// would otherwise race two parallel esp_https_ota sessions against the same
// ota partition.
static volatile bool s_in_flight = false;

// Snapshot of the server URL captured by oe_ota_start_check, used by the
// worker task. Sized large enough for OE_URL_BUF.
static char s_server_url[OE_URL_BUF];

// Strip trailing slashes and append the firmware bundle path. server_url
// can be "https://host" or "https://host/" — both should produce
// "https://host/firmware/voice-device/<suffix>".
static void build_firmware_url(char *out, size_t out_len,
                               const char *server_url, const char *suffix)
{
    size_t n = strnlen(server_url, OE_URL_BUF);
    while (n > 0 && server_url[n - 1] == '/') n--;
    snprintf(out, out_len, "%.*s/firmware/voice-device/%s",
             (int) n, server_url, suffix);
}

// Compare two dotted version strings "MAJOR.MINOR.PATCH". Returns:
//   <0 if a < b, 0 if equal, >0 if a > b. Unknown/missing components are 0.
//
// Stay tolerant of suffixes ("0.2.3-dirty") — we just stop comparing at the
// first non-digit/non-dot and treat the rest as equal. That keeps "dirty"
// dev builds from looking older than a release manifest at the same triplet.
static int version_cmp(const char *a, const char *b)
{
    while (*a || *b) {
        int va = 0, vb = 0;
        while (*a >= '0' && *a <= '9') { va = va * 10 + (*a - '0'); a++; }
        while (*b >= '0' && *b <= '9') { vb = vb * 10 + (*b - '0'); b++; }
        if (va != vb) return va - vb;
        if (*a == '.') a++; else if (*a) break;
        if (*b == '.') b++; else if (*b) break;
    }
    return 0;
}

// Fetch the manifest JSON into a freshly-allocated buffer. Caller frees.
// Returns NULL on any error (network, http status, oversized).
static char *fetch_manifest(const char *server_url)
{
    char url[OE_URL_BUF + 64];
    build_firmware_url(url, sizeof(url), server_url, "manifest.json");

    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 8000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) return NULL;

    esp_err_t e = esp_http_client_open(c, 0);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "manifest open failed: %s", esp_err_to_name(e));
        esp_http_client_cleanup(c);
        return NULL;
    }
    int content_len = esp_http_client_fetch_headers(c);
    int status = esp_http_client_get_status_code(c);
    if (status != 200) {
        ESP_LOGW(TAG, "manifest http %d", status);
        esp_http_client_close(c);
        esp_http_client_cleanup(c);
        return NULL;
    }
    // 4 KB is generous for our manifest (<1 KB) and protects against a
    // server-side surprise without an upfront content-length.
    const int MAX_BODY = 4096;
    int cap = content_len > 0 ? content_len + 1 : 1024;
    if (cap > MAX_BODY + 1) cap = MAX_BODY + 1;
    char *body = malloc(cap);
    if (!body) {
        esp_http_client_close(c);
        esp_http_client_cleanup(c);
        return NULL;
    }
    int total = 0;
    while (total < cap - 1) {
        int r = esp_http_client_read(c, body + total, cap - 1 - total);
        if (r <= 0) break;
        total += r;
    }
    body[total] = 0;
    esp_http_client_close(c);
    esp_http_client_cleanup(c);
    return body;
}

// Pull the "version" string and the firmware app's "file" field out of the
// manifest. Returns true on success; *out_version and *out_file are
// strdup'd, caller frees both.
static bool parse_manifest(const char *body, char **out_version, char **out_file,
                           char **out_sha256)
{
    cJSON *j = cJSON_Parse(body);
    if (!j) return false;
    bool ok = false;
    const cJSON *jver = cJSON_GetObjectItem(j, "version");
    const cJSON *jparts = cJSON_GetObjectItem(j, "parts");
    if (!cJSON_IsString(jver) || !cJSON_IsArray(jparts)) goto out;
    char *appfile = NULL;
    char *appsha = NULL;
    cJSON *p = NULL;
    cJSON_ArrayForEach(p, jparts) {
        const cJSON *jname = cJSON_GetObjectItem(p, "name");
        const cJSON *jfile = cJSON_GetObjectItem(p, "file");
        if (cJSON_IsString(jname) && cJSON_IsString(jfile) &&
            strcmp(jname->valuestring, "app") == 0) {
            appfile = strdup(jfile->valuestring);
            const cJSON *jsha = cJSON_GetObjectItem(p, "sha256");
            if (cJSON_IsString(jsha)) appsha = strdup(jsha->valuestring);
            break;
        }
    }
    if (!appfile) goto out;
    *out_version = strdup(jver->valuestring);
    *out_file = appfile;
    *out_sha256 = appsha;   // may be NULL (older manifest w/o hashes → skip verify)
    ok = true;
out:
    cJSON_Delete(j);
    return ok;
}

// Compute SHA-256 over the first `len` bytes of `part` and compare to the
// 64-char hex `expected_hex`. Returns true on match. If `expected_hex` is
// NULL or not 64 hex chars, returns true — an older manifest without a hash
// keeps the prior (unverified) behavior. A definitive mismatch or a read
// failure returns false so the caller aborts the OTA: we never bless a boot
// image whose bytes we couldn't confirm.
static bool ota_partition_sha256_ok(const esp_partition_t *part, size_t len,
                                    const char *expected_hex)
{
    if (!expected_hex || strlen(expected_hex) != 64) return true;  // no hash → skip
    uint8_t want[32];
    for (int i = 0; i < 32; ++i) {
        char c1 = expected_hex[i * 2], c2 = expected_hex[i * 2 + 1];
        int hi = (c1 >= '0' && c1 <= '9') ? c1 - '0' :
                 (c1 >= 'a' && c1 <= 'f') ? c1 - 'a' + 10 :
                 (c1 >= 'A' && c1 <= 'F') ? c1 - 'A' + 10 : -1;
        int lo = (c2 >= '0' && c2 <= '9') ? c2 - '0' :
                 (c2 >= 'a' && c2 <= 'f') ? c2 - 'a' + 10 :
                 (c2 >= 'A' && c2 <= 'F') ? c2 - 'A' + 10 : -1;
        if (hi < 0 || lo < 0) return false;   // non-hex → treat as mismatch
        want[i] = (uint8_t)((hi << 4) | lo);
    }
    uint8_t *buf = malloc(4096);
    if (!buf) return false;
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);   // 0 = SHA-256 (not SHA-224)
    bool ok = true;
    for (size_t off = 0; off < len; ) {
        size_t chunk = len - off;
        if (chunk > 4096) chunk = 4096;
        if (esp_partition_read(part, off, buf, chunk) != ESP_OK) { ok = false; break; }
        mbedtls_sha256_update(&ctx, buf, chunk);
        off += chunk;
    }
    uint8_t got[32];
    if (ok) mbedtls_sha256_finish(&ctx, got);
    mbedtls_sha256_free(&ctx);
    free(buf);
    if (!ok) return false;
    return memcmp(got, want, sizeof(want)) == 0;
}

static void ota_task(void *arg)
{
    char *manifest_body = NULL;
    char *target_version = NULL;
    char *app_file = NULL;
    char *app_sha256 = NULL;
    const esp_app_desc_t *running = esp_app_get_description();
    const char *running_ver = running && running->version[0] ? running->version : "0.0.0";

    oe_ws_send_ota_progress("checking", 0, 0, NULL, NULL);

    manifest_body = fetch_manifest(s_server_url);
    if (!manifest_body) {
        oe_ws_send_ota_progress("error", 0, 0, NULL, "manifest_fetch_failed");
        goto done;
    }
    if (!parse_manifest(manifest_body, &target_version, &app_file, &app_sha256)) {
        oe_ws_send_ota_progress("error", 0, 0, NULL, "manifest_parse_failed");
        goto done;
    }

    ESP_LOGI(TAG, "running=%s manifest=%s app=%s", running_ver, target_version, app_file);
    if (version_cmp(target_version, running_ver) <= 0) {
        oe_ws_send_ota_progress("up_to_date", 0, 0, target_version, NULL);
        goto done;
    }

    char bin_url[OE_URL_BUF + 64];
    build_firmware_url(bin_url, sizeof(bin_url), s_server_url, app_file);

    esp_http_client_config_t http_cfg = {
        .url = bin_url,
        .timeout_ms = 30000,
        .keep_alive_enable = true,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };
    esp_https_ota_handle_t handle = NULL;
    esp_err_t e = esp_https_ota_begin(&ota_cfg, &handle);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "https_ota_begin: %s", esp_err_to_name(e));
        oe_ws_send_ota_progress("error", 0, 0, target_version, esp_err_to_name(e));
        goto done;
    }

    int total = esp_https_ota_get_image_size(handle);
    oe_ws_send_ota_progress("downloading", 0, (uint32_t) (total > 0 ? total : 0), target_version, NULL);

    // Throttle progress events so we don't drown the WS in 60+ messages on a
    // 2 MB download. 100 KB step ≈ 22 events for a 2.2 MB image.
    const int PROGRESS_STEP = 100 * 1024;
    int last_emit = 0;
    while (1) {
        e = esp_https_ota_perform(handle);
        if (e != ESP_ERR_HTTPS_OTA_IN_PROGRESS) break;
        int done = esp_https_ota_get_image_len_read(handle);
        if (done - last_emit >= PROGRESS_STEP) {
            oe_ws_send_ota_progress("downloading", (uint32_t) done,
                                    (uint32_t) (total > 0 ? total : 0),
                                    target_version, NULL);
            last_emit = done;
        }
    }
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "https_ota_perform: %s", esp_err_to_name(e));
        oe_ws_send_ota_progress("error", 0, 0, target_version, esp_err_to_name(e));
        esp_https_ota_abort(handle);
        goto done;
    }
    if (!esp_https_ota_is_complete_data_received(handle)) {
        oe_ws_send_ota_progress("error", 0, 0, target_version, "incomplete");
        esp_https_ota_abort(handle);
        goto done;
    }

    // Verify the written image against the manifest SHA-256 BEFORE finish()
    // switches the boot partition. Mismatch → abort, boot partition untouched
    // (fail-safe). Skipped only if the manifest carries no hash (back-compat).
    {
        const esp_partition_t *upd = esp_ota_get_next_update_partition(NULL);
        size_t img_len = (size_t) esp_https_ota_get_image_len_read(handle);
        if (!upd || !ota_partition_sha256_ok(upd, img_len, app_sha256)) {
            ESP_LOGE(TAG, "OTA image sha256 verification failed — aborting");
            oe_ws_send_ota_progress("error", 0, 0, target_version, "sha256_mismatch");
            esp_https_ota_abort(handle);
            goto done;
        }
        if (app_sha256) ESP_LOGI(TAG, "OTA image sha256 verified");
    }

    oe_ws_send_ota_progress("applying", (uint32_t)(total > 0 ? total : 0),
                            (uint32_t)(total > 0 ? total : 0), target_version, NULL);
    e = esp_https_ota_finish(handle);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "https_ota_finish: %s", esp_err_to_name(e));
        oe_ws_send_ota_progress("error", 0, 0, target_version, esp_err_to_name(e));
        goto done;
    }

    oe_ws_send_ota_progress("rebooting", 0, 0, target_version, NULL);
    ESP_LOGI(TAG, "OTA complete; rebooting into %s", target_version);
    // Small delay so the WS frame has a chance to flush before we yank power.
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();

done:
    free(manifest_body);
    free(target_version);
    free(app_file);
    free(app_sha256);
    s_in_flight = false;
    vTaskDelete(NULL);
}

esp_err_t oe_ota_start_check(const char *server_url)
{
    if (s_in_flight) return ESP_ERR_INVALID_STATE;
    if (!server_url || !server_url[0]) return ESP_ERR_INVALID_ARG;
    s_in_flight = true;
    strncpy(s_server_url, server_url, sizeof(s_server_url) - 1);
    s_server_url[sizeof(s_server_url) - 1] = 0;
    // 8 KB stack — esp_https_ota + cJSON parse + mbedtls TLS combined sit
    // around 6 KB peak in our usage. Pinned to the protocol core so it
    // doesn't compete with audio inference for CPU0.
    BaseType_t r = xTaskCreatePinnedToCore(ota_task, "oe_ota", 8192, NULL,
                                           tskIDLE_PRIORITY + 3, NULL, 0);
    if (r != pdPASS) {
        s_in_flight = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void oe_ota_mark_running_valid(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (!running) return;
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) != ESP_OK) return;
    if (state == ESP_OTA_IMG_PENDING_VERIFY) {
        // Cancel the auto-rollback that IDF armed when we booted from a
        // freshly-OTA'd image. From here on, the new image is "blessed".
        esp_err_t e = esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(TAG, "marked running app valid: %s", esp_err_to_name(e));
    }
}
