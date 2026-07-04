#include "oe_client.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_websocket_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_app_desc.h"
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

static esp_err_t ws_send_json(cJSON *o, TickType_t timeout)
{
    if (!o) return ESP_ERR_NO_MEM;
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    char *s = cJSON_PrintUnformatted(o);
    if (!s) return ESP_ERR_NO_MEM;
    int rc = esp_websocket_client_send_text(s_ws, s, strlen(s), timeout);
    free(s);
    return rc < 0 ? ESP_FAIL : ESP_OK;
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
    s_ws = esp_websocket_client_init(&cfg);
    if (!s_ws) return ESP_FAIL;
    esp_websocket_register_events(s_ws, WEBSOCKET_EVENT_ANY, ws_event_handler, NULL);
    return esp_websocket_client_start(s_ws);
}

esp_err_t oe_ws_stop(void)
{
    if (!s_ws) return ESP_OK;
    esp_websocket_client_stop(s_ws);
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

esp_err_t oe_ws_send_stop(const char *agent_id, const char *turn_id)
{
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "stop");
    // Omitted when empty — the server falls back to the user's coordinator.
    if (agent_id && agent_id[0]) cJSON_AddStringToObject(o, "agent", agent_id);
    // Id of the turn being stopped (NOT a new turn's id) so the server can
    // ignore a stale stop that races a newer turn on the same socket.
    if (turn_id && turn_id[0]) cJSON_AddStringToObject(o, "turn_id", turn_id);
    esp_err_t err = ws_send_json(o, pdMS_TO_TICKS(1000));
    cJSON_Delete(o);
    return err;
}

static esp_err_t send_tts_flow(const char *type, const char *turn_id)
{
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", type);
    if (turn_id && turn_id[0]) cJSON_AddStringToObject(o, "turn_id", turn_id);
    esp_err_t err = ws_send_json(o, pdMS_TO_TICKS(1000));
    cJSON_Delete(o);
    return err;
}

esp_err_t oe_ws_send_tts_pause(const char *turn_id)  { return send_tts_flow("tts_pause",  turn_id); }
esp_err_t oe_ws_send_tts_resume(const char *turn_id) { return send_tts_flow("tts_resume", turn_id); }

esp_err_t oe_ws_send_stt_begin(const char *turn_id, uint8_t wake_slot,
                               uint8_t wake_avg_prob, const char *agent_id)
{
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "stt_begin");
    if (turn_id && turn_id[0]) cJSON_AddStringToObject(o, "turn_id", turn_id);
    cJSON_AddNumberToObject(o, "wake_slot", wake_slot);
    cJSON_AddNumberToObject(o, "wake_avg_prob", wake_avg_prob);
    if (agent_id && agent_id[0]) cJSON_AddStringToObject(o, "agent", agent_id);
    esp_err_t err = ws_send_json(o, pdMS_TO_TICKS(1000));
    cJSON_Delete(o);
    return err;
}

// Binary frame: 'OEA1' + u32 LE seq + payload. Static buffer is safe — only
// the capture/drive task streams frames, one at a time.
esp_err_t oe_ws_send_stt_frame(const int16_t *samples, size_t n_samples, uint32_t seq)
{
    static uint8_t buf[8 + OE_STT_FRAME_MAX_SAMPLES * sizeof(int16_t)];
    if (!oe_ws_connected()) return ESP_ERR_INVALID_STATE;
    if (!samples || n_samples == 0 || n_samples > OE_STT_FRAME_MAX_SAMPLES) return ESP_ERR_INVALID_ARG;
    buf[0] = 'O'; buf[1] = 'E'; buf[2] = 'A'; buf[3] = '1';
    buf[4] = (uint8_t)(seq & 0xFF);
    buf[5] = (uint8_t)((seq >> 8) & 0xFF);
    buf[6] = (uint8_t)((seq >> 16) & 0xFF);
    buf[7] = (uint8_t)((seq >> 24) & 0xFF);
    memcpy(buf + 8, samples, n_samples * sizeof(int16_t));
    // Short timeout: at the 80 ms frame cadence a congested socket must fail
    // fast so the caller can flip to the buffered-HTTP fallback rather than
    // stalling the capture loop (the 16 KB capture ring only holds ~0.5 s).
    int rc = esp_websocket_client_send_bin(s_ws, (const char *)buf,
                                           8 + n_samples * sizeof(int16_t),
                                           pdMS_TO_TICKS(150));
    return rc < 0 ? ESP_FAIL : ESP_OK;
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
