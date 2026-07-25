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
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "esp_partition.h"
#include "mbedtls/sha256.h"
#include "nvs_creds.h"

static const char *TAG = "oe_ota";

// OTA's task must be able to run after months of heap churn. Keeping both its
// stack and TCB in internal DRAM removes the late, contiguous 8 KB allocation
// that previously failed on fragmented devices. A PSRAM stack is unsafe here:
// flash writes can make external RAM temporarily inaccessible.
#define OTA_TASK_STACK_BYTES 8192
// Ordinary component .bss is internal DRAM; only EXT_RAM_BSS_ATTR would place
// this in PSRAM. Keeping the zeroed storage in .bss also avoids embedding an
// unnecessary 8 KB run of zeros in every OTA image.
static StackType_t s_ota_stack[OTA_TASK_STACK_BYTES]
    __attribute__((aligned(16)));
static StaticTask_t s_ota_tcb;
static TaskHandle_t s_ota_task = NULL;

// One-at-a-time state and the request snapshot are shared by the WS callback
// and persistent worker. Volatile alone did not make the old check/set atomic.
static portMUX_TYPE s_ota_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_in_flight = false;
static bool s_current_is_recovery = false;

// Snapshot of the server URL captured by oe_ota_start_check, used by the
// worker task. Sized large enough for OE_URL_BUF.
static char s_server_url[OE_URL_BUF];

// Persisted one-reboot cap. PENDING is written before the first OOM reboot;
// CONSUMED is committed before the post-reboot attempt is scheduled. A stale
// CONSUMED latch at boot means that recovery itself reset, so it is cleared
// rather than retried again.
enum {
    OTA_RETRY_IDLE = 0,
    OTA_RETRY_PENDING = 1,
    OTA_RETRY_CONSUMED = 2,
    OTA_RETRY_ACTIVATING = 3,  // in-memory only while NVS 1 -> 2 is committed
};
static uint8_t s_retry_state = OTA_RETRY_IDLE;

typedef enum {
    OTA_ATTEMPT_DONE = 0,
    OTA_ATTEMPT_FAILED,
    OTA_ATTEMPT_NO_MEM,
} ota_attempt_result_t;

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
// Keep explicit allocation failures distinct from network/server errors so
// only a proven OOM can arm the one-shot reboot recovery.
static esp_err_t fetch_manifest(const char *server_url, char **out_body)
{
    if (!out_body) return ESP_ERR_INVALID_ARG;
    *out_body = NULL;
    char url[OE_URL_BUF + 64];
    build_firmware_url(url, sizeof(url), server_url, "manifest.json");

    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 8000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    // IDF returns NULL when one of the client's transport/parser/buffer
    // allocations fails. This URL/config is constructed locally, so retain
    // the useful OOM classification instead of collapsing it into a generic
    // manifest fetch error.
    if (!c) return ESP_ERR_NO_MEM;

    esp_err_t e = esp_http_client_open(c, 0);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "manifest open failed: %s", esp_err_to_name(e));
        esp_http_client_cleanup(c);
        return e;
    }
    int content_len = esp_http_client_fetch_headers(c);
    int status = esp_http_client_get_status_code(c);
    if (status != 200) {
        ESP_LOGW(TAG, "manifest http %d", status);
        esp_http_client_close(c);
        esp_http_client_cleanup(c);
        return ESP_ERR_INVALID_RESPONSE;
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
        return ESP_ERR_NO_MEM;
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
    *out_body = body;
    return ESP_OK;
}

// Pull the "version" string and the firmware app's "file" field out of the
// manifest. Output strings are freshly allocated and owned by the caller.
// A malformed manifest and an allocation failure intentionally have different
// return codes; bad public content must never cause a recovery reboot.
static esp_err_t parse_manifest(const char *body, char **out_version,
                                char **out_file, char **out_sha256)
{
    if (!body || !out_version || !out_file || !out_sha256) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_version = NULL;
    *out_file = NULL;
    *out_sha256 = NULL;
    cJSON *j = cJSON_Parse(body);
    if (!j) return ESP_ERR_INVALID_RESPONSE;
    esp_err_t result = ESP_ERR_INVALID_RESPONSE;
    const cJSON *jver = cJSON_GetObjectItem(j, "version");
    const cJSON *jparts = cJSON_GetObjectItem(j, "parts");
    if (!cJSON_IsString(jver) || !cJSON_IsArray(jparts)) goto out;
    const char *appfile = NULL;
    const char *appsha = NULL;
    cJSON *p = NULL;
    cJSON_ArrayForEach(p, jparts) {
        const cJSON *jname = cJSON_GetObjectItem(p, "name");
        const cJSON *jfile = cJSON_GetObjectItem(p, "file");
        if (cJSON_IsString(jname) && cJSON_IsString(jfile) &&
            strcmp(jname->valuestring, "app") == 0) {
            appfile = jfile->valuestring;
            const cJSON *jsha = cJSON_GetObjectItem(p, "sha256");
            if (cJSON_IsString(jsha)) appsha = jsha->valuestring;
            break;
        }
    }
    if (!appfile) goto out;
    *out_version = strdup(jver->valuestring);
    *out_file = strdup(appfile);
    if (appsha) *out_sha256 = strdup(appsha);
    if (!*out_version || !*out_file || (appsha && !*out_sha256)) {
        free(*out_version);
        free(*out_file);
        free(*out_sha256);
        *out_version = NULL;
        *out_file = NULL;
        *out_sha256 = NULL;
        result = ESP_ERR_NO_MEM;
        goto out;
    }
    result = ESP_OK;
out:
    cJSON_Delete(j);
    return result;
}

// Compute SHA-256 over the first `len` bytes of `part`. Missing hashes retain
// backward compatibility, while malformed hashes/read failures remain hard
// failures. Return allocation errors separately for bounded OOM recovery.
static esp_err_t ota_partition_sha256_check(const esp_partition_t *part,
                                            size_t len,
                                            const char *expected_hex,
                                            bool *out_match)
{
    if (!part || !out_match) return ESP_ERR_INVALID_ARG;
    *out_match = false;
    if (!expected_hex || strlen(expected_hex) != 64) {
        *out_match = true;  // older manifest without a usable hash
        return ESP_OK;
    }
    uint8_t want[32];
    for (int i = 0; i < 32; ++i) {
        char c1 = expected_hex[i * 2], c2 = expected_hex[i * 2 + 1];
        int hi = (c1 >= '0' && c1 <= '9') ? c1 - '0' :
                 (c1 >= 'a' && c1 <= 'f') ? c1 - 'a' + 10 :
                 (c1 >= 'A' && c1 <= 'F') ? c1 - 'A' + 10 : -1;
        int lo = (c2 >= '0' && c2 <= '9') ? c2 - '0' :
                 (c2 >= 'a' && c2 <= 'f') ? c2 - 'a' + 10 :
                 (c2 >= 'A' && c2 <= 'F') ? c2 - 'A' + 10 : -1;
        if (hi < 0 || lo < 0) return ESP_OK;  // non-hex → mismatch
        want[i] = (uint8_t)((hi << 4) | lo);
    }
    uint8_t *buf = malloc(4096);
    if (!buf) return ESP_ERR_NO_MEM;
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);   // 0 = SHA-256 (not SHA-224)
    esp_err_t result = ESP_OK;
    for (size_t off = 0; off < len; ) {
        size_t chunk = len - off;
        if (chunk > 4096) chunk = 4096;
        result = esp_partition_read(part, off, buf, chunk);
        if (result != ESP_OK) break;
        mbedtls_sha256_update(&ctx, buf, chunk);
        off += chunk;
    }
    uint8_t got[32];
    if (result == ESP_OK) {
        mbedtls_sha256_finish(&ctx, got);
        *out_match = memcmp(got, want, sizeof(want)) == 0;
    }
    mbedtls_sha256_free(&ctx);
    free(buf);
    return result;
}

static ota_attempt_result_t ota_check_once(void)
{
    char *manifest_body = NULL;
    char *target_version = NULL;
    char *app_file = NULL;
    char *app_sha256 = NULL;
    ota_attempt_result_t result = OTA_ATTEMPT_FAILED;
    const esp_app_desc_t *running = esp_app_get_description();
    const char *running_ver = running && running->version[0] ? running->version : "0.0.0";

    oe_ws_send_ota_progress("checking", 0, 0, NULL, NULL);

    esp_err_t e = fetch_manifest(s_server_url, &manifest_body);
    if (e != ESP_OK) {
        if (e == ESP_ERR_NO_MEM) {
            result = OTA_ATTEMPT_NO_MEM;
            goto done;
        }
        oe_ws_send_ota_progress("error", 0, 0, NULL, "manifest_fetch_failed");
        goto done;
    }
    e = parse_manifest(manifest_body, &target_version, &app_file, &app_sha256);
    if (e != ESP_OK) {
        if (e == ESP_ERR_NO_MEM) {
            result = OTA_ATTEMPT_NO_MEM;
            goto done;
        }
        oe_ws_send_ota_progress("error", 0, 0, NULL, "manifest_parse_failed");
        goto done;
    }

    ESP_LOGI(TAG, "running=%s manifest=%s app=%s", running_ver, target_version, app_file);
    if (version_cmp(target_version, running_ver) <= 0) {
        oe_ws_send_ota_progress("up_to_date", 0, 0, target_version, NULL);
        result = OTA_ATTEMPT_DONE;
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
    e = esp_https_ota_begin(&ota_cfg, &handle);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "https_ota_begin: %s", esp_err_to_name(e));
        if (e == ESP_ERR_NO_MEM) {
            result = OTA_ATTEMPT_NO_MEM;
            goto done;
        }
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
        esp_https_ota_abort(handle);
        if (e == ESP_ERR_NO_MEM) {
            result = OTA_ATTEMPT_NO_MEM;
            goto done;
        }
        oe_ws_send_ota_progress("error", 0, 0, target_version, esp_err_to_name(e));
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
        bool sha_match = false;
        esp_err_t sha_e = upd
            ? ota_partition_sha256_check(upd, img_len, app_sha256, &sha_match)
            : ESP_ERR_NOT_FOUND;
        if (sha_e == ESP_ERR_NO_MEM) {
            esp_https_ota_abort(handle);
            result = OTA_ATTEMPT_NO_MEM;
            goto done;
        }
        if (sha_e != ESP_OK || !sha_match) {
            ESP_LOGE(TAG, "OTA image sha256 verification failed — aborting");
            oe_ws_send_ota_progress("error", 0, 0, target_version,
                                    sha_e == ESP_OK ? "sha256_mismatch"
                                                    : "sha256_read_failed");
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
        if (e == ESP_ERR_NO_MEM) {
            result = OTA_ATTEMPT_NO_MEM;
            goto done;
        }
        oe_ws_send_ota_progress("error", 0, 0, target_version, esp_err_to_name(e));
        goto done;
    }

    // A recovery attempt was consumed before it started. Clear that latch
    // before rebooting into the new image so the successful OTA boot cannot
    // be mistaken for another memory-recovery boot.
    if (s_current_is_recovery) {
        esp_err_t clear_e = nvs_creds_set_ota_retry_state(OTA_RETRY_IDLE);
        if (clear_e != ESP_OK) {
            ESP_LOGW(TAG, "OTA retry latch clear before reboot failed: %s",
                     esp_err_to_name(clear_e));
        }
        portENTER_CRITICAL(&s_ota_mux);
        s_retry_state = OTA_RETRY_IDLE;
        portEXIT_CRITICAL(&s_ota_mux);
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
    return result;
}

static void ota_log_internal_heap(const char *reason)
{
    const uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    ESP_LOGW(TAG, "%s: internal_free=%lu internal_largest=%lu",
             reason,
             (unsigned long)heap_caps_get_free_size(caps),
             (unsigned long)heap_caps_get_largest_free_block(caps));
}

static void ota_finish_attempt_state(bool was_recovery)
{
    if (was_recovery) {
        esp_err_t e = nvs_creds_set_ota_retry_state(OTA_RETRY_IDLE);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "OTA retry latch clear failed: %s", esp_err_to_name(e));
        }
    }
    portENTER_CRITICAL(&s_ota_mux);
    if (was_recovery) s_retry_state = OTA_RETRY_IDLE;
    s_current_is_recovery = false;
    s_in_flight = false;
    portEXIT_CRITICAL(&s_ota_mux);
}

static void ota_task(void *arg)
{
    (void)arg;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        bool was_recovery;
        portENTER_CRITICAL(&s_ota_mux);
        was_recovery = s_current_is_recovery;
        portEXIT_CRITICAL(&s_ota_mux);

        ota_attempt_result_t result = ota_check_once();
        if (result == OTA_ATTEMPT_NO_MEM) {
            ota_log_internal_heap(was_recovery ? "OTA retry OOM" : "OTA OOM");
            if (!was_recovery) {
                // Persist first. If NVS itself cannot commit under memory
                // pressure, fail without rebooting: an unrecorded retry could
                // otherwise turn into an unbounded restart loop.
                esp_err_t e = nvs_creds_set_ota_retry_state(OTA_RETRY_PENDING);
                if (e == ESP_OK) {
                    portENTER_CRITICAL(&s_ota_mux);
                    s_retry_state = OTA_RETRY_PENDING;
                    portEXIT_CRITICAL(&s_ota_mux);
                    oe_ws_send_ota_progress("restarting_for_memory", 0, 0,
                                            NULL, "out_of_memory");
                    ESP_LOGW(TAG, "rebooting once to defragment OTA runtime allocations");
                    vTaskDelay(pdMS_TO_TICKS(800));
                    esp_restart();
                }
                ESP_LOGE(TAG, "cannot persist OTA memory retry: %s",
                         esp_err_to_name(e));
                oe_ws_send_ota_progress("error", 0, 0, NULL,
                                        "out_of_memory_retry_persist_failed");
            } else {
                // The persisted state was moved to CONSUMED before this
                // attempt. Never reboot twice for the same OTA request.
                oe_ws_send_ota_progress("error", 0, 0, NULL,
                                        "out_of_memory_after_reboot");
            }
        }
        ota_finish_attempt_state(was_recovery);
    }
}

esp_err_t oe_ota_init(void)
{
    if (s_ota_task) return ESP_OK;

    uint8_t persisted = OTA_RETRY_IDLE;
    esp_err_t e = nvs_creds_get_ota_retry_state(&persisted);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "OTA retry latch read failed: %s", esp_err_to_name(e));
        persisted = OTA_RETRY_IDLE;
    } else if (persisted == OTA_RETRY_CONSUMED || persisted > OTA_RETRY_CONSUMED) {
        // Recovery started but this firmware booted again without completing
        // it. Clear the latch instead of re-entering a reboot loop.
        ESP_LOGW(TAG, "clearing stale consumed OTA retry latch");
        e = nvs_creds_set_ota_retry_state(OTA_RETRY_IDLE);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "stale OTA retry latch clear failed: %s",
                     esp_err_to_name(e));
        }
        persisted = OTA_RETRY_IDLE;
    }
    portENTER_CRITICAL(&s_ota_mux);
    s_retry_state = persisted;
    portEXIT_CRITICAL(&s_ota_mux);

    // ESP-IDF specifies stack depth in bytes, and StackType_t is uint8_t on
    // this ESP32-S3 Xtensa target. The static buffer and TCB are linked into
    // internal DRAM and never touch the runtime heap.
    TaskHandle_t task = xTaskCreateStaticPinnedToCore(
        ota_task, "oe_ota", OTA_TASK_STACK_BYTES, NULL,
        tskIDLE_PRIORITY + 3, s_ota_stack, &s_ota_tcb, 0);
    if (!task) return ESP_ERR_NO_MEM;
    s_ota_task = task;
    ESP_LOGI(TAG, "persistent OTA worker ready (static %u-byte internal stack)",
             (unsigned)OTA_TASK_STACK_BYTES);
    return ESP_OK;
}

static esp_err_t ota_schedule_normal(const char *server_url)
{
    portENTER_CRITICAL(&s_ota_mux);
    if (s_in_flight) {
        portEXIT_CRITICAL(&s_ota_mux);
        return ESP_ERR_INVALID_STATE;
    }
    s_in_flight = true;
    s_current_is_recovery = false;
    strncpy(s_server_url, server_url, sizeof(s_server_url) - 1);
    s_server_url[sizeof(s_server_url) - 1] = 0;
    TaskHandle_t task = s_ota_task;
    portEXIT_CRITICAL(&s_ota_mux);
    xTaskNotifyGive(task);
    return ESP_OK;
}

esp_err_t oe_ota_resume_pending(const char *server_url)
{
    if (!server_url || !server_url[0]) return ESP_ERR_INVALID_ARG;
    esp_err_t e = oe_ota_init();
    if (e != ESP_OK) return e;

    // Reserve the worker slot before the NVS commit so two callers (the
    // authenticated caps callback and end-of-boot latch) cannot both consume
    // the same pending recovery.
    portENTER_CRITICAL(&s_ota_mux);
    if (s_in_flight) {
        portEXIT_CRITICAL(&s_ota_mux);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_retry_state != OTA_RETRY_PENDING) {
        portEXIT_CRITICAL(&s_ota_mux);
        return ESP_ERR_NOT_FOUND;
    }
    s_in_flight = true;
    s_current_is_recovery = true;
    s_retry_state = OTA_RETRY_ACTIVATING;
    strncpy(s_server_url, server_url, sizeof(s_server_url) - 1);
    s_server_url[sizeof(s_server_url) - 1] = 0;
    portEXIT_CRITICAL(&s_ota_mux);

    // Consume before notify. A power loss or crash after this commit cannot
    // make the next boot retry again.
    e = nvs_creds_set_ota_retry_state(OTA_RETRY_CONSUMED);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "cannot consume OTA retry latch: %s", esp_err_to_name(e));
        (void)nvs_creds_set_ota_retry_state(OTA_RETRY_IDLE);
        portENTER_CRITICAL(&s_ota_mux);
        s_retry_state = OTA_RETRY_IDLE;
        s_current_is_recovery = false;
        s_in_flight = false;
        portEXIT_CRITICAL(&s_ota_mux);
        return e;
    }

    portENTER_CRITICAL(&s_ota_mux);
    s_retry_state = OTA_RETRY_CONSUMED;
    TaskHandle_t task = s_ota_task;
    portEXIT_CRITICAL(&s_ota_mux);
    ESP_LOGI(TAG, "resuming OTA after one-shot memory recovery reboot");
    xTaskNotifyGive(task);
    return ESP_OK;
}

esp_err_t oe_ota_start_check(const char *server_url)
{
    if (!server_url || !server_url[0]) return ESP_ERR_INVALID_ARG;
    esp_err_t e = oe_ota_init();
    if (e != ESP_OK) return e;

    portENTER_CRITICAL(&s_ota_mux);
    bool pending_recovery = s_retry_state == OTA_RETRY_PENDING;
    portEXIT_CRITICAL(&s_ota_mux);
    // Only main.c's authenticated-caps + boot-ready latch may consume a
    // reboot recovery. A server OTA_CHECK can arrive during asynchronous boot;
    // letting it consume PENDING here would bypass that readiness barrier.
    if (pending_recovery) return ESP_ERR_INVALID_STATE;
    return ota_schedule_normal(server_url);
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
