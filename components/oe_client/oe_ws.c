#include "oe_client.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_websocket_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "cJSON.h"

static const char *TAG = "oe_ws";

static esp_websocket_client_handle_t s_ws = NULL;
static oe_ws_callback_t s_cb = NULL;
static void *s_cb_user = NULL;
static char s_token[OE_TOKEN_BUF];

static char *s_msg_accum = NULL;
static size_t s_msg_accum_len = 0;
static size_t s_msg_accum_cap = 0;

static void emit(oe_ws_event_t t, const char *text, size_t len)
{
    if (!s_cb) return;
    oe_ws_payload_t p = { .type = t, .text = text, .text_len = len, .turn_id = NULL };
    s_cb(&p, s_cb_user);
}

// emit + the message's turn_id (NULL when the server didn't include one).
static void emit_turn(oe_ws_event_t t, const char *text, size_t len, const cJSON *j)
{
    if (!s_cb) return;
    const cJSON *jt = cJSON_GetObjectItem(j, "turn_id");
    oe_ws_payload_t p = {
        .type = t, .text = text, .text_len = len,
        .turn_id = (cJSON_IsString(jt) && jt->valuestring[0]) ? jt->valuestring : NULL,
    };
    s_cb(&p, s_cb_user);
}

// ── WS transmit path ────────────────────────────────────────────────────────
// INVARIANT: no finite timeout may ever reach esp_websocket_client_send_*.
//
// In this client a send timeout is NOT a soft failure. esp_transport_ws_send_raw
// returns 0 when its poll expires, and the client's reaction to a zero-length
// write is esp_websocket_client_abort_connection() — it tears the socket down,
// the server sees a 1006, and every in-flight turn dies with it. A merely
// congested link (host loaded, whisper backed up) would therefore strand the
// device rather than delay a frame. That is what dropped gate-accepted wakes:
// oe_ws_send_stt_backlog bursts the whole buffered wake window, the TCP window
// fills after ~1 frame, the next 20 ms send returns 0, and the socket dies.
//
// So all sends are issued from ONE dedicated TX task using portMAX_DELAY — the
// client maps that to an infinite transport poll, which can never time out, so
// only a genuine socket error can abort. Callers hand their payload to a
// bounded queue instead, and their timeout now bounds QUEUE ADMISSION only: a
// full queue is a soft ESP_ERR_TIMEOUT that the existing buffered-HTTP
// fallbacks already handle, and it never touches the socket. Hot paths (the
// capture loop, whose ring holds only ~512 ms) still never block on the network.
//
// A genuinely dead peer is still caught: LWIP TCP keepalive (idle 10 /
// interval 5 / count 3, set in oe_ws_start) errors the blocked write within
// ~25 s and drives the normal auto-reconnect. Waiting is correct; teardown is
// reserved for a socket that is actually gone.
//
// The single TX task also preserves frame ordering, which a per-caller send
// could not guarantee once sends are allowed to block.
#define OE_WS_TX_QUEUE_DEPTH  48
#define OE_WS_TX_MAX_BACKLOG  (160 * 1024)   // ≈2× a 2.5 s gate replay burst

typedef struct {
    uint8_t *data;
    size_t   len;
    bool     is_text;
} ws_tx_msg_t;

static QueueHandle_t     s_tx_q     = NULL;
static SemaphoreHandle_t s_tx_mtx   = NULL;   // guards s_tx_bytes only
static TaskHandle_t      s_tx_task  = NULL;
static size_t            s_tx_bytes = 0;
static volatile bool     s_tx_run   = false;

// Reserve/release the queued-byte budget. Reserving under the mutex keeps the
// check and the increment atomic, so a burst can't race past the cap.
static bool tx_reserve(size_t n)
{
    if (!s_tx_mtx) return false;
    bool ok = false;
    xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
    if (s_tx_bytes + n <= OE_WS_TX_MAX_BACKLOG) { s_tx_bytes += n; ok = true; }
    xSemaphoreGive(s_tx_mtx);
    return ok;
}

static void tx_release(size_t n)
{
    if (!s_tx_mtx) return;
    xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
    s_tx_bytes = (s_tx_bytes > n) ? s_tx_bytes - n : 0;
    xSemaphoreGive(s_tx_mtx);
}

// Drop everything still queued. Called on disconnect so a reconnected socket
// never receives audio frames belonging to the previous session.
static void tx_drain(void)
{
    if (!s_tx_q) return;
    ws_tx_msg_t m;
    while (xQueueReceive(s_tx_q, &m, 0) == pdTRUE) {
        tx_release(m.len);
        free(m.data);
    }
}

static void ws_tx_task(void *arg)
{
    (void)arg;
    ws_tx_msg_t m;
    while (s_tx_run) {
        // Bounded receive so shutdown is noticed even with an idle queue.
        if (xQueueReceive(s_tx_q, &m, pdMS_TO_TICKS(200)) != pdTRUE) continue;
        if (s_ws && esp_websocket_client_is_connected(s_ws)) {
            // portMAX_DELAY is the whole point: block for as long as the peer
            // needs. Anything finite would abort the connection instead.
            int rc = m.is_text
                ? esp_websocket_client_send_text(s_ws, (const char *)m.data, (int)m.len, portMAX_DELAY)
                : esp_websocket_client_send_bin (s_ws, (const char *)m.data, (int)m.len, portMAX_DELAY);
            if (rc != (int)m.len) {
                // Only a real transport error lands here; the client has
                // already aborted and auto-reconnect is scheduled.
                ESP_LOGW(TAG, "tx: send returned %d for %u bytes", rc, (unsigned)m.len);
            }
        }
        tx_release(m.len);
        free(m.data);
    }
    s_tx_task = NULL;
    vTaskDelete(NULL);
}

// Queue one frame, optionally assembled from a header + payload so callers
// don't need a staging buffer. `admit` bounds queue admission only.
static esp_err_t ws_tx_submit2(const void *a, size_t alen,
                               const void *b, size_t blen,
                               bool is_text, TickType_t admit)
{
    const size_t len = alen + blen;
    if (!s_tx_q || !oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    if (len == 0) return ESP_ERR_INVALID_ARG;
    // Budget first: a rejected frame must not allocate.
    if (!tx_reserve(len)) return ESP_ERR_TIMEOUT;
    // PSRAM by preference — a 2.5 s replay burst is ~80 KB and internal SRAM
    // is the scarce pool. Falls back to internal if PSRAM is unavailable.
    uint8_t *copy = heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!copy) copy = malloc(len);
    if (!copy) { tx_release(len); return ESP_ERR_NO_MEM; }
    memcpy(copy, a, alen);
    if (blen) memcpy(copy + alen, b, blen);
    ws_tx_msg_t m = { .data = copy, .len = len, .is_text = is_text };
    if (xQueueSend(s_tx_q, &m, admit) != pdTRUE) {
        free(copy);
        tx_release(len);
        return ESP_ERR_TIMEOUT;   // soft: caller falls back, socket untouched
    }
    return ESP_OK;
}

static esp_err_t ws_send_json(cJSON *o, TickType_t timeout)
{
    if (!o) return ESP_ERR_NO_MEM;
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    char *s = cJSON_PrintUnformatted(o);
    if (!s) return ESP_ERR_NO_MEM;
    esp_err_t err = ws_tx_submit2(s, strlen(s), NULL, 0, true, timeout);
    free(s);
    return err;
}

static void handle_message(const char *data, size_t len)
{
    cJSON *j = cJSON_ParseWithLength(data, len);
    if (!j) return;
    const cJSON *jtype = cJSON_GetObjectItem(j, "type");
    if (!jtype || !cJSON_IsString(jtype)) { cJSON_Delete(j); return; }
    const char *type = jtype->valuestring;

    if (strcmp(type, "agent_list") == 0) {
        emit(OE_WS_EVT_AGENT_LIST, data, len);
    } else if (strcmp(type, "token") == 0) {
        const cJSON *jtxt = cJSON_GetObjectItem(j, "text");
        if (jtxt && cJSON_IsString(jtxt)) {
            emit_turn(OE_WS_EVT_CHAT_TOKEN, jtxt->valuestring, strlen(jtxt->valuestring), j);
        }
    } else if (strcmp(type, "done") == 0) {
        emit_turn(OE_WS_EVT_CHAT_DONE, NULL, 0, j);
    } else if (strcmp(type, "tts_audio_begin") == 0) {
        emit_turn(OE_WS_EVT_TTS_AUDIO_BEGIN, NULL, 0, j);
    } else if (strcmp(type, "tts_audio") == 0) {
        const cJSON *jp = cJSON_GetObjectItem(j, "pcm_b64");
        if (cJSON_IsString(jp) && jp->valuestring)
            emit_turn(OE_WS_EVT_TTS_AUDIO, jp->valuestring, strlen(jp->valuestring), j);
    } else if (strcmp(type, "tts_audio_end") == 0) {
        // Raw JSON up to main.c — it parses the optional `pending` flag
        // (burst-close on an open turn → waiting-LED instead of idle).
        emit_turn(OE_WS_EVT_TTS_AUDIO_END, data, len, j);
    } else if (strcmp(type, "duplicate_suppressed") == 0) {
        emit(OE_WS_EVT_DUPLICATE_SUPPRESSED, NULL, 0);
    } else if (strcmp(type, "error") == 0) {
        const cJSON *jmsg = cJSON_GetObjectItem(j, "message");
        if (jmsg && cJSON_IsString(jmsg)) {
            emit_turn(OE_WS_EVT_ERROR, jmsg->valuestring, strlen(jmsg->valuestring), j);
        }
    } else if (strcmp(type, "server_caps") == 0) {
        emit(OE_WS_EVT_SERVER_CAPS, data, len);
    } else if (strcmp(type, "set_conversation_mode") == 0) {
        emit(OE_WS_EVT_SET_CONVERSATION_MODE, data, len);
    } else if (strcmp(type, "ui_wait") == 0) {
        // Raw JSON up to main.c — it parses `on` and owns the LED state.
        emit(OE_WS_EVT_UI_WAIT, data, len);
    } else if (strcmp(type, "ww_upload") == 0) {
        // Pass the raw JSON message up to main.c — it owns the SPIFFS
        // writer + the wakeword_t array. Keeping the parse + b64 decode
        // there avoids dragging wakeword/spiffs deps into oe_ws.c.
        emit(OE_WS_EVT_WW_UPLOAD, data, len);
    } else if (strcmp(type, "ww_clear") == 0) {
        // { type:'ww_clear', slot:N } — main.c unlinks the slot files +
        // unloads the live detector. Raw JSON up to main.c for the slot index.
        emit(OE_WS_EVT_WW_CLEAR, data, len);
    } else if (strcmp(type, "set_volume") == 0) {
        // Voice-control intent: { type:'set_volume', pct:<0-100> } or
        // { type:'set_volume', delta:<-100..+100> }. main.c parses + applies.
        emit(OE_WS_EVT_SET_VOLUME, data, len);
    } else if (strcmp(type, "pause_playback") == 0) {
        emit(OE_WS_EVT_PAUSE_PLAYBACK, NULL, 0);
    } else if (strcmp(type, "resume_playback") == 0) {
        emit(OE_WS_EVT_RESUME_PLAYBACK, NULL, 0);
    } else if (strcmp(type, "ota_check") == 0) {
        emit(OE_WS_EVT_OTA_CHECK, NULL, 0);
    } else if (strcmp(type, "enter_ap_mode") == 0) {
        emit(OE_WS_EVT_ENTER_AP, NULL, 0);
    } else if (strcmp(type, "reboot") == 0) {
        // Raw JSON; main.c logs the optional reason before esp_restart().
        emit(OE_WS_EVT_REBOOT, data, len);
    } else if (strcmp(type, "alarm_arm") == 0) {
        // Raw JSON; main.c parses id/label/triggerAtMs/audioMarker/alarmType.
        emit(OE_WS_EVT_ALARM_ARM, data, len);
    } else if (strcmp(type, "alarm_disarm") == 0) {
        emit(OE_WS_EVT_ALARM_DISARM, data, len);
    } else if (strcmp(type, "alarm_stop") == 0) {
        emit(OE_WS_EVT_ALARM_STOP, data, len);
    } else if (strcmp(type, "chime_upload") == 0) {
        emit(OE_WS_EVT_CHIME_UPLOAD, data, len);
    } else if (strcmp(type, "await_followup") == 0) {
        emit(OE_WS_EVT_AWAIT_FOLLOWUP, data, len);
    } else if (strcmp(type, "play_ambient") == 0) {
        // Raw JSON; main.c parses audioMarker + loop + volume and spawns
        // ambient_worker_task. Looped playback re-fetches the same /api/tts
        // marker on EOF until a stop signal is set device-side.
        emit(OE_WS_EVT_PLAY_AMBIENT, data, len);
    } else if (strcmp(type, "stop_ambient") == 0) {
        emit(OE_WS_EVT_STOP_AMBIENT, NULL, 0);
    } else if (strcmp(type, "set_device_name") == 0) {
        // Raw JSON; main.c parses `name` and persists to NVS.
        emit(OE_WS_EVT_SET_DEVICE_NAME, data, len);
    } else if (strcmp(type, "set_headphone_mode") == 0) {
        // Raw JSON; main.c parses `enabled` and persists + applies.
        emit(OE_WS_EVT_SET_HEADPHONE_MODE, data, len);
    } else if (strcmp(type, "airplay_stop") == 0) {
        emit(OE_WS_EVT_AIRPLAY_STOP, NULL, 0);
    } else if (strcmp(type, "airplay_next") == 0) {
        emit(OE_WS_EVT_AIRPLAY_NEXT, NULL, 0);
    } else if (strcmp(type, "airplay_prev") == 0) {
        emit(OE_WS_EVT_AIRPLAY_PREV, NULL, 0);
    }
    cJSON_Delete(j);
}

static void send_auth(void)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "auth");
    cJSON_AddStringToObject(o, "token", s_token);
    // PROJECT_VER from CMakeLists.txt -> baked into esp_app_desc -> here.
    // Server stores this on voice-devices.json and uses it to decide whether
    // an OTA is needed (and to display the current version in the UI).
    const esp_app_desc_t *app = esp_app_get_description();
    if (app && app->version[0]) {
        cJSON_AddStringToObject(o, "firmware_version", app->version);
    }
    esp_err_t err = ws_send_json(o, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) ESP_LOGW(TAG, "auth send failed: %s", esp_err_to_name(err));
    cJSON_Delete(o);
}

static void build_ws_url(char *out, size_t out_len, const char *server_url)
{
    const char *base = server_url;
    const char *scheme = "";
    if (strncasecmp(server_url, "https://", 8) == 0) {
        scheme = "wss://";
        base = server_url + 8;
    } else if (strncasecmp(server_url, "http://", 7) == 0) {
        scheme = "ws://";
        base = server_url + 7;
    }
    size_t n = strnlen(base, OE_URL_BUF);
    while (n > 0 && base[n - 1] == '/') n--;
    snprintf(out, out_len, "%s%.*s/ws", scheme, (int)n, base);
}

static void ws_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *d = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "ws connected");
            send_auth();
            emit(OE_WS_EVT_CONNECTED, NULL, 0);
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "ws disconnected");
            s_msg_accum_len = 0;
            // Drop queued frames: they belong to the session that just died,
            // and replaying them onto a reconnected socket would inject stale
            // audio into whatever turn comes next.
            tx_drain();
            emit(OE_WS_EVT_DISCONNECTED, NULL, 0);
            break;
        case WEBSOCKET_EVENT_DATA:
            if (d->op_code == 0x01 || d->op_code == 0x00) {
                if (d->payload_len > d->data_len) {
                    size_t needed = s_msg_accum_len + d->data_len + 1;
                    if (needed > s_msg_accum_cap) {
                        size_t new_cap = s_msg_accum_cap ? s_msg_accum_cap * 2 : 2048;
                        while (new_cap < needed) new_cap *= 2;
                        char *nb = realloc(s_msg_accum, new_cap);
                        if (!nb) { s_msg_accum_len = 0; return; }
                        s_msg_accum = nb;
                        s_msg_accum_cap = new_cap;
                    }
                    memcpy(s_msg_accum + s_msg_accum_len, d->data_ptr, d->data_len);
                    s_msg_accum_len += d->data_len;
                    if (d->payload_offset + d->data_len >= d->payload_len) {
                        s_msg_accum[s_msg_accum_len] = 0;
                        handle_message(s_msg_accum, s_msg_accum_len);
                        s_msg_accum_len = 0;
                    }
                } else {
                    handle_message((const char *)d->data_ptr, d->data_len);
                }
            }
            break;
        case WEBSOCKET_EVENT_ERROR:
            emit(OE_WS_EVT_ERROR, NULL, 0);
            break;
        default: break;
    }
}

esp_err_t oe_ws_start(const char *server_url, const char *token,
                      oe_ws_callback_t cb, void *user)
{
    if (s_ws) return ESP_OK;
    s_cb = cb;
    s_cb_user = user;
    strncpy(s_token, token, sizeof(s_token) - 1);

    // URL scheme is case-insensitive per RFC 3986 §3.1. Users who type
    // HTTP:// in the captive portal would otherwise hit the fallback branch
    // and produce a malformed ws_url (HTTP://.../ws), which esp_websocket
    // silently can't parse and the connection never opens.
    char ws_url[OE_URL_BUF + 8];
    build_ws_url(ws_url, sizeof(ws_url), server_url);
    ESP_LOGI(TAG, "ws uri: %s", ws_url);

    // The two flags that matter for reconnect across an OE server restart:
    //
    //   enable_close_reconnect: schedules a reconnect after the server sends
    //   a clean close-1001 frame. Defaults to FALSE in esp_websocket_client,
    //   which means the worker task logs "Did not get TCP close within
    //   expected delay" and then exits without ever trying again. This is
    //   THE bug that left us stuck after every OE restart.
    //
    //   keep_alive_*: LWIP-layer TCP keepalive. Runs in the kernel, immune
    //   to wake-word audio starvation (which broke WS-level ping/pong
    //   experiments — bug #8545). Detects half-dead sockets (where the
    //   server died before sending FIN) within ~25 s and fires a transport
    //   error, which triggers the normal auto-reconnect path.
    //
    // See research notes in project_xvf3800_voice_device.md memory entry
    // (gate at esp_websocket_client.c:1398, flag at .h:117).
    esp_websocket_client_config_t cfg = {
        .uri = ws_url,
        // One complete OEA1 audio message must fit in the WS client's own
        // buffer. Its 1024-byte default split every 2568-byte STT frame into
        // three separately blocking transport writes, long enough to overflow
        // the ~512 ms mic capture ring on a congested link.
        .buffer_size = 8 + OE_STT_FRAME_MAX_SAMPLES * sizeof(int16_t),
        // The WS event callback runs the entire streamed-TTS write path on
        // this task's stack: cJSON parse + base64 decode + audio_io_write_pcm
        // (which keeps a 1.5 KB resample buffer). The library default of
        // 4 KB overflowed the moment TTS frames arrived (0.2.60 panic loop —
        // "stack overflow in task websocket_task", 2026-07-02). 8 KB gives
        // ~2× worst-case headroom; the [hb] stack-hwm dump watches it.
        .task_stack             = 8192,
        .reconnect_timeout_ms   = 2000,
        .network_timeout_ms     = 10000,
        .enable_close_reconnect = true,
        .keep_alive_enable      = true,
        .keep_alive_idle        = 10,
        .keep_alive_interval    = 5,
        .keep_alive_count       = 3,
        .crt_bundle_attach      = esp_crt_bundle_attach,
    };
    // TX plumbing must exist before the client can connect: send_auth() runs
    // from the CONNECTED event and goes through the queue like everything else.
    if (!s_tx_mtx) s_tx_mtx = xSemaphoreCreateMutex();
    if (!s_tx_q)   s_tx_q   = xQueueCreate(OE_WS_TX_QUEUE_DEPTH, sizeof(ws_tx_msg_t));
    if (!s_tx_mtx || !s_tx_q) {
        ESP_LOGE(TAG, "tx: queue alloc failed");
        return ESP_ERR_NO_MEM;
    }
    // Re-arm before the existence check: a restart can land here while a prior
    // task is still draining its way out of oe_ws_stop. Setting the flag first
    // lets that task simply keep serving instead of exiting into a dead queue.
    s_tx_run = true;
    if (!s_tx_task) {
        if (xTaskCreate(ws_tx_task, "oe_ws_tx", 3072, NULL, 5, &s_tx_task) != pdPASS) {
            s_tx_run = false;
            s_tx_task = NULL;
            ESP_LOGE(TAG, "tx: task create failed");
            return ESP_ERR_NO_MEM;
        }
    }

    s_ws = esp_websocket_client_init(&cfg);
    if (!s_ws) return ESP_FAIL;
    esp_websocket_register_events(s_ws, WEBSOCKET_EVENT_ANY, ws_event_handler, NULL);
    return esp_websocket_client_start(s_ws);
}

esp_err_t oe_ws_stop(void)
{
    if (!s_ws) return ESP_OK;
    // Stop the client FIRST: it aborts the transport, which unblocks a TX task
    // parked in an unbounded send so it can observe s_tx_run and exit.
    esp_websocket_client_stop(s_ws);
    s_tx_run = false;
    for (int i = 0; i < 100 && s_tx_task; ++i) vTaskDelay(pdMS_TO_TICKS(10));
    tx_drain();
    // Only reclaim the queue once the task is provably gone — tearing it down
    // underneath a live consumer would be a use-after-free.
    if (!s_tx_task) {
        if (s_tx_q)   { vQueueDelete(s_tx_q);      s_tx_q = NULL; }
        if (s_tx_mtx) { vSemaphoreDelete(s_tx_mtx); s_tx_mtx = NULL; }
    } else {
        ESP_LOGW(TAG, "tx: task still running at stop — queue retained");
    }
    esp_websocket_client_destroy(s_ws);
    s_ws = NULL;
    free(s_msg_accum);
    s_msg_accum = NULL;
    s_msg_accum_len = 0;
    s_msg_accum_cap = 0;
    return ESP_OK;
}

bool oe_ws_connected(void)
{
    return s_ws && esp_websocket_client_is_connected(s_ws);
}

static TickType_t timeout_ms_to_ticks(uint32_t timeout_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    return ticks > 0 ? ticks : 1;
}

esp_err_t oe_ws_send_chat(const char *agent_id, const char *text, uint8_t wake_slot, uint8_t wake_avg_prob, const char *turn_id, bool barge_in)
{
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "chat");
    // Omitted when empty — the server then routes to the acting user's
    // coordinator (and a wake-slot assignment overrides it anyway).
    if (agent_id && agent_id[0]) cJSON_AddStringToObject(o, "agent", agent_id);
    cJSON_AddStringToObject(o, "text", text);
    cJSON_AddNumberToObject(o, "wake_slot", wake_slot);
    cJSON_AddNumberToObject(o, "wake_avg_prob", wake_avg_prob);
    cJSON_AddStringToObject(o, "source", "voice-device");
    // Device-minted turn correlation id. Older servers ignore it; newer ones
    // echo it on every event of this turn so stale-turn events are droppable.
    if (turn_id && turn_id[0]) cJSON_AddStringToObject(o, "turn_id", turn_id);
    // Speech-barge turn: transcript may be prefixed with reply bleed.
    if (barge_in) cJSON_AddBoolToObject(o, "barge", true);
    // Opt into server-side TTS streaming: the server segments + synthesizes +
    // pushes PCM audio frames instead of raw tokens (this firmware plays them).
    cJSON_AddBoolToObject(o, "tts_stream", true);
    esp_err_t err = ws_send_json(o, pdMS_TO_TICKS(2000));
    cJSON_Delete(o);
    return err;
}

static esp_err_t send_stop(const char *agent_id, const char *turn_id,
                           const char *hold_id, uint32_t timeout_ms)
{
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "stop");
    // Omitted when empty — the server falls back to the user's coordinator.
    if (agent_id && agent_id[0]) cJSON_AddStringToObject(o, "agent", agent_id);
    // Id of the turn being stopped (NOT a new turn's id) so the server can
    // ignore a stale stop that races a newer turn on the same socket.
    if (turn_id && turn_id[0]) cJSON_AddStringToObject(o, "turn_id", turn_id);
    if (hold_id && hold_id[0]) cJSON_AddStringToObject(o, "hold_id", hold_id);
    esp_err_t err = ws_send_json(o, timeout_ms_to_ticks(timeout_ms));
    cJSON_Delete(o);
    return err;
}

esp_err_t oe_ws_send_stop_timeout(const char *agent_id, const char *turn_id,
                                  uint32_t timeout_ms)
{
    return send_stop(agent_id, turn_id, NULL, timeout_ms);
}

esp_err_t oe_ws_send_stop_hold_timeout(const char *agent_id,
                                       const char *turn_id,
                                       const char *hold_id,
                                       uint32_t timeout_ms)
{
    return send_stop(agent_id, turn_id, hold_id, timeout_ms);
}

esp_err_t oe_ws_send_stop(const char *agent_id, const char *turn_id)
{
    return oe_ws_send_stop_timeout(agent_id, turn_id, 1000);
}

static esp_err_t send_tts_flow(const char *type, const char *turn_id,
                               const char *hold_id, uint32_t timeout_ms)
{
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", type);
    if (turn_id && turn_id[0]) cJSON_AddStringToObject(o, "turn_id", turn_id);
    if (hold_id && hold_id[0]) cJSON_AddStringToObject(o, "hold_id", hold_id);
    esp_err_t err = ws_send_json(o, timeout_ms_to_ticks(timeout_ms));
    cJSON_Delete(o);
    return err;
}

esp_err_t oe_ws_send_tts_pause_timeout(const char *turn_id,
                                       uint32_t timeout_ms)
{
    return send_tts_flow("tts_pause", turn_id, NULL, timeout_ms);
}

esp_err_t oe_ws_send_tts_resume_timeout(const char *turn_id,
                                        uint32_t timeout_ms)
{
    return send_tts_flow("tts_resume", turn_id, NULL, timeout_ms);
}

esp_err_t oe_ws_send_tts_pause_hold_timeout(const char *turn_id,
                                            const char *hold_id,
                                            uint32_t timeout_ms)
{
    return send_tts_flow("tts_pause", turn_id, hold_id, timeout_ms);
}

esp_err_t oe_ws_send_tts_resume_hold_timeout(const char *turn_id,
                                             const char *hold_id,
                                             uint32_t timeout_ms)
{
    return send_tts_flow("tts_resume", turn_id, hold_id, timeout_ms);
}

esp_err_t oe_ws_send_tts_pause(const char *turn_id)
{
    return oe_ws_send_tts_pause_timeout(turn_id, 1000);
}

esp_err_t oe_ws_send_tts_resume(const char *turn_id)
{
    return oe_ws_send_tts_resume_timeout(turn_id, 1000);
}

static esp_err_t send_stt_begin_timeout(const char *turn_id, uint8_t wake_slot,
                                        uint8_t wake_avg_prob,
                                        const char *agent_id,
                                        TickType_t timeout)
{
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "stt_begin");
    if (turn_id && turn_id[0]) cJSON_AddStringToObject(o, "turn_id", turn_id);
    cJSON_AddNumberToObject(o, "wake_slot", wake_slot);
    cJSON_AddNumberToObject(o, "wake_avg_prob", wake_avg_prob);
    if (agent_id && agent_id[0]) cJSON_AddStringToObject(o, "agent", agent_id);
    esp_err_t err = ws_send_json(o, timeout);
    cJSON_Delete(o);
    return err;
}

esp_err_t oe_ws_send_stt_begin(const char *turn_id, uint8_t wake_slot,
                               uint8_t wake_avg_prob, const char *agent_id)
{
    return send_stt_begin_timeout(turn_id, wake_slot, wake_avg_prob, agent_id,
                                  pdMS_TO_TICKS(1000));
}

// Binary frame: 'OEA1' + u32 LE seq + payload. Assembled straight into the
// queue allocation, so there is no shared staging buffer to serialize on.
static esp_err_t send_stt_frame_timeout(const int16_t *samples,
                                        size_t n_samples, uint32_t seq,
                                        TickType_t timeout)
{
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    if (!samples || n_samples == 0 || n_samples > OE_STT_FRAME_MAX_SAMPLES) return ESP_ERR_INVALID_ARG;
    uint8_t hdr[8];
    hdr[0] = 'O'; hdr[1] = 'E'; hdr[2] = 'A'; hdr[3] = '1';
    hdr[4] = (uint8_t)(seq & 0xFF);
    hdr[5] = (uint8_t)((seq >> 8) & 0xFF);
    hdr[6] = (uint8_t)((seq >> 16) & 0xFF);
    hdr[7] = (uint8_t)((seq >> 24) & 0xFF);
    return ws_tx_submit2(hdr, sizeof(hdr), samples,
                         n_samples * sizeof(int16_t), false, timeout);
}

esp_err_t oe_ws_send_stt_frame(const int16_t *samples, size_t n_samples, uint32_t seq)
{
    // Short timeout, but it now bounds QUEUE ADMISSION rather than the socket
    // write: at the 80 ms frame cadence a backed-up TX queue must fail fast so
    // the caller can flip to the buffered-HTTP fallback rather than stalling
    // the capture loop (the 16 KB capture ring only holds ~0.5 s). The frame
    // itself is then sent with an unbounded wait, so congestion delays audio
    // instead of tearing the socket down.
    return send_stt_frame_timeout(samples, n_samples, seq, pdMS_TO_TICKS(20));
}

esp_err_t oe_ws_send_stt_backlog(const char *turn_id, uint8_t wake_slot,
                                 uint8_t wake_avg_prob, const char *agent_id,
                                 const int16_t *samples, size_t n_samples,
                                 uint32_t budget_ms, uint32_t *out_next_seq)
{
    if (!samples || n_samples == 0 || budget_ms == 0 || !out_next_seq) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_next_seq = 0;

    const int64_t deadline_us =
        esp_timer_get_time() + (int64_t)budget_ms * 1000;
    TickType_t begin_ticks = pdMS_TO_TICKS(budget_ms < 20 ? budget_ms : 20);
    if (begin_ticks == 0) begin_ticks = 1;
    esp_err_t err = send_stt_begin_timeout(
        turn_id, wake_slot, wake_avg_prob, agent_id, begin_ticks);
    if (err != ESP_OK) return err;

    uint32_t seq = 0;
    for (size_t off = 0; off < n_samples; off += OE_STT_FRAME_MAX_SAMPLES) {
        int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0) return ESP_ERR_TIMEOUT;
        uint32_t remaining_ms = (uint32_t)((remaining_us + 999) / 1000);
        uint32_t send_ms = remaining_ms < 20 ? remaining_ms : 20;
        TickType_t send_ticks = pdMS_TO_TICKS(send_ms);
        if (send_ticks == 0) send_ticks = 1;

        size_t chunk = n_samples - off;
        if (chunk > OE_STT_FRAME_MAX_SAMPLES) {
            chunk = OE_STT_FRAME_MAX_SAMPLES;
        }
        err = send_stt_frame_timeout(samples + off, chunk, seq, send_ticks);
        if (err != ESP_OK) return err;
        seq++;
    }
    *out_next_seq = seq;
    return ESP_OK;
}

esp_err_t oe_ws_send_stt_end(const char *turn_id, uint32_t total_samples)
{
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "stt_end");
    if (turn_id && turn_id[0]) cJSON_AddStringToObject(o, "turn_id", turn_id);
    cJSON_AddNumberToObject(o, "samples", total_samples);
    esp_err_t err = ws_send_json(o, pdMS_TO_TICKS(2000));
    cJSON_Delete(o);
    return err;
}

esp_err_t oe_ws_send_stt_abort(const char *turn_id)
{
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "stt_abort");
    if (turn_id && turn_id[0]) cJSON_AddStringToObject(o, "turn_id", turn_id);
    esp_err_t err = ws_send_json(o, pdMS_TO_TICKS(1000));
    cJSON_Delete(o);
    return err;
}

esp_err_t oe_ws_send_ambient_stopped(const char *reason)
{
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "ambient_stopped");
    if (reason && reason[0]) cJSON_AddStringToObject(o, "reason", reason);
    esp_err_t err = ws_send_json(o, pdMS_TO_TICKS(1000));
    cJSON_Delete(o);
    return err;
}

esp_err_t oe_ws_send_ww_ack(int slot, bool ok, const char *err)
{
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "ww_upload_ack");
    cJSON_AddNumberToObject(o, "slot", slot);
    cJSON_AddBoolToObject(o, "ok", ok);
    if (!ok && err) cJSON_AddStringToObject(o, "err", err);
    esp_err_t send_err = ws_send_json(o, pdMS_TO_TICKS(1000));
    cJSON_Delete(o);
    return send_err;
}

static esp_err_t send_alarm_ack(const char *type, const char *alarm_id)
{
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    if (!alarm_id) return ESP_ERR_INVALID_ARG;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", type);
    cJSON_AddStringToObject(o, "id", alarm_id);
    esp_err_t err = ws_send_json(o, pdMS_TO_TICKS(1000));
    cJSON_Delete(o);
    return err;
}

esp_err_t oe_ws_send_alarm_fired(const char *alarm_id)
{
    return send_alarm_ack("alarm_fired", alarm_id);
}

esp_err_t oe_ws_send_alarm_acked(const char *alarm_id)
{
    return send_alarm_ack("alarm_acked", alarm_id);
}

esp_err_t oe_ws_send_ota_progress(const char *phase, uint32_t bytes_done,
                                  uint32_t total, const char *target_version,
                                  const char *err)
{
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "ota_progress");
    cJSON_AddStringToObject(o, "phase", phase ? phase : "");
    if (bytes_done) cJSON_AddNumberToObject(o, "bytes_done", bytes_done);
    if (total)      cJSON_AddNumberToObject(o, "total",      total);
    if (target_version) cJSON_AddStringToObject(o, "target_version", target_version);
    if (err)            cJSON_AddStringToObject(o, "err",            err);
    esp_err_t send_err = ws_send_json(o, pdMS_TO_TICKS(1000));
    cJSON_Delete(o);
    return send_err;
}
