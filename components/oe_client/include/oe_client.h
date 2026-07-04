#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#define OE_TOKEN_BUF 128
#define OE_AGENT_ID_BUF 64
#define OE_URL_BUF 256

typedef struct {
    char token[OE_TOKEN_BUF];
    char user_id[OE_AGENT_ID_BUF];
    char server_hint[OE_URL_BUF];
} oe_pair_result_t;

esp_err_t oe_pair_redeem(const char *server_url, const char *pair_code,
                         const char *device_name, oe_pair_result_t *out);

typedef enum {
    OE_WS_EVT_CONNECTED,
    OE_WS_EVT_DISCONNECTED,
    OE_WS_EVT_AGENT_LIST,
    OE_WS_EVT_CHAT_TOKEN,
    OE_WS_EVT_CHAT_DONE,
    OE_WS_EVT_DUPLICATE_SUPPRESSED,
    OE_WS_EVT_ERROR,
    // Server pushed a wake-word OTA update for one of the SPIFFS slots.
    // Payload is the raw JSON text of the ww_upload message; the handler
    // in main.c parses it (slot index + tflite_b64 + manifest), writes
    // the file pair to /ww/slot{N}.{tflite,json}, and calls
    // wakeword_load_slot to hot-reload that slot's model.
    OE_WS_EVT_WW_UPLOAD,
    // Server cleared a wake-word slot whose user was removed from the voice
    // config: { type:'ww_clear', slot:N }. Payload is the raw JSON text;
    // main.c deletes /ww/slot{N}.{tflite,json} and unloads the live detector
    // so the slot stops firing. Acked with the same ww_upload_ack the server
    // uses to serialize per-slot pushes.
    OE_WS_EVT_WW_CLEAR,
    // Voice-control intents from the server-side regex router. Payload
    // is the raw JSON text; main.c parses pct/delta and routes to
    // audio_io_set_volume / pause / resume. See chat-dispatch.mjs
    // classifyVoiceIntent for the protocol shape.
    OE_WS_EVT_SET_VOLUME,
    OE_WS_EVT_PAUSE_PLAYBACK,
    OE_WS_EVT_RESUME_PLAYBACK,
    // Server requested a firmware OTA check: { type:'ota_check' }. No
    // payload. Device fetches /firmware/voice-device/manifest.json, compares
    // version against the running app, downloads + applies if newer, then
    // reboots. Reports progress via oe_ws_send_ota_progress.
    OE_WS_EVT_OTA_CHECK,
    // Server requested Wi-Fi re-provisioning: { type:'enter_ap_mode' }. No
    // payload. Device wipes stored credentials (NVS) and reboots into the
    // captive-portal AP (oe-voice-XXXX) so the user can join it and enter new
    // Wi-Fi / re-pair — move a device to a different network with no computer
    // or physical button. (Wake-word models in SPIFFS are preserved.)
    OE_WS_EVT_ENTER_AP,
    // Server requested a plain reboot: { type:'reboot', reason?:'mic_dead' }.
    // Credentials, pairing, and wake-word models all survive — this is the
    // recovery primitive the server-side health loop sends when telemetry
    // says the device is broken-but-heartbeating (e.g. cap_sps=0). Raw JSON
    // payload so main.c can log the server's reason before restarting.
    OE_WS_EVT_REBOOT,
    // Alarm protocol — server → device. Raw JSON payload; main.c parses
    // id/label/trigger_at_ms/audio_marker/type and delegates to the alarm
    // component. See lib/alarms.mjs (server) for the message shapes.
    OE_WS_EVT_ALARM_ARM,
    OE_WS_EVT_ALARM_DISARM,
    OE_WS_EVT_ALARM_STOP,
    // { type:'chime_upload', audioMarker } — main.c fetches the marker
    // via oe_alarm_fetch_audio and hands the MP3 to alarm_set_custom_chime.
    OE_WS_EVT_CHIME_UPLOAD,
    // { type:'await_followup', windowMs } — server's signal that the
    // last reply ended with a question. Device opens a brief listen
    // window where any voice activity bypasses the wake-word check.
    OE_WS_EVT_AWAIT_FOLLOWUP,
    // { type:'play_ambient', audioMarker, loop, volume? } — start a
    // (looped) ambient playback fetched via /api/tts using the marker
    // text. main.c spawns ambient_worker_task which loops on EOF until
    // a stop signal is set (wake fire OR explicit stop_ambient).
    OE_WS_EVT_PLAY_AMBIENT,
    // { type:'stop_ambient' } — halt any in-flight ambient loop.
    OE_WS_EVT_STOP_AMBIENT,
    // { type:'set_device_name', name:'Kitchen' } — user renamed the
    // device in Settings → Voice devices. Payload is the raw JSON;
    // main.c parses `name`, persists via nvs_creds_set_device_name,
    // and updates g_dev_config.device_name in place. The AirPlay
    // mDNS service name only refreshes on next boot (raop has the
    // name baked into the RAOP context at create time).
    OE_WS_EVT_SET_DEVICE_NAME,
    // { type:'set_headphone_mode', enabled:true|false } — toggles the
    // amp-enable suppression so wake-word stays sensitive while audio
    // is routed through the 3.5 mm jack. main.c persists via NVS +
    // calls xvf3800_set_headphone_mode.
    OE_WS_EVT_SET_HEADPHONE_MODE,
    // AirPlay session control. No payload — main.c calls the corresponding
    // airplay_*() function; the firmware-side helpers no-op when no AirPlay
    // session is streaming so the server can fire these unconditionally.
    OE_WS_EVT_AIRPLAY_STOP,
    OE_WS_EVT_AIRPLAY_NEXT,
    OE_WS_EVT_AIRPLAY_PREV,
    // Server-side TTS streaming: the server segments + synthesizes + pushes
    // 16 kHz mono s16le PCM frames; the device just plays them. See
    // lib/voice-tts-stream.mjs. BEGIN starts playback, AUDIO carries one
    // base64 PCM frame (payload in .text), END signals drain-then-idle.
    OE_WS_EVT_TTS_AUDIO_BEGIN,
    OE_WS_EVT_TTS_AUDIO,
    OE_WS_EVT_TTS_AUDIO_END,
    // { type:'server_caps', turn_ids, tts_pause, stt_stream } — sent once
    // after auth. Raw JSON payload; main.c stores the capability flags so
    // newer device→server messages are only sent to servers that understand
    // them. Flags reset on disconnect (the next server may be older).
    OE_WS_EVT_SERVER_CAPS,
    // { type:'set_conversation_mode', enabled } — per-device toggle from
    // Settings. Enables the speech barge-in VAD during SPEAKING (main.c
    // pause-then-verify state machine). Raw JSON payload. Not persisted:
    // the server reconciles it on every connect, and conversation features
    // are meaningless without a live server anyway.
    OE_WS_EVT_SET_CONVERSATION_MODE,
    // { type:'ui_wait', on } — server hint that background work for this
    // device is in flight OUTSIDE any turn (delegated task after the ack
    // reply ended; result arrives later as an announcement). LED-only:
    // main.c shows the WAITING rainbow while otherwise idle, mic untouched.
    // Raw JSON payload. Re-sent ~10s by the server while work is pending.
    OE_WS_EVT_UI_WAIT,
} oe_ws_event_t;

typedef struct {
    oe_ws_event_t type;
    const char *text;
    size_t text_len;
    // Optional turn correlation id echoed by the server on token/done/
    // tts_audio_*/await_followup/error. NULL or empty when the server didn't
    // send one (older server) — treat as "accept" for compatibility. main.c
    // drops events whose turn_id mismatches the device's current turn, which
    // kills the whole family of aborted-turn-events-race-the-next-turn bugs.
    const char *turn_id;
} oe_ws_payload_t;

typedef void (*oe_ws_callback_t)(const oe_ws_payload_t *evt, void *user);

esp_err_t oe_ws_start(const char *server_url, const char *token,
                      oe_ws_callback_t cb, void *user);
esp_err_t oe_ws_stop(void);
bool      oe_ws_connected(void);

// turn_id: device-minted correlation id for this turn (NULL/"" = omit —
// pre-turn-id behavior). The server adopts it and echoes it on every event
// belonging to the turn; a stop must carry the id of the turn being stopped.
// barge_in: true when this utterance interrupted our own reply (stage-C
// commit). The transcript may carry a prefix of reply bleed from the barge
// pre-roll, so the server relaxes its bare-word intent anchors ("stop").
esp_err_t oe_ws_send_chat(const char *agent_id, const char *text, uint8_t wake_slot, uint8_t wake_avg_prob, const char *turn_id, bool barge_in);
esp_err_t oe_ws_send_stop(const char *agent_id, const char *turn_id);

// Speech barge-in flow control (send ONLY when server_caps.tts_pause).
// tts_pause stalls the server's PCM pacer while the device verifies a
// barge-in candidate; tts_resume un-stalls it after a false alarm. The
// server auto-aborts the stream ~20 s after an unresumed pause, so a lost
// resume degrades to a truncated reply, never a wedged pacer.
esp_err_t oe_ws_send_tts_pause(const char *turn_id);
esp_err_t oe_ws_send_tts_resume(const char *turn_id);

// ── Streaming STT (send ONLY when server_caps.stt_stream) ──────────────────
// Instead of buffering the whole utterance and blocking on one HTTP POST,
// the capture loop streams each 80 ms frame as a binary WS frame ('OEA1'
// magic + u32 LE seq + s16le mono 16 kHz PCM) between stt_begin and stt_end.
// The server transcribes at stt_end and dispatches the turn itself — the
// device goes straight to THINKING and the reply arrives over the normal
// token / tts_audio_* events. stt_abort drops a no-speech capture. Any send
// failure mid-utterance → caller falls back to the buffered oe_stt_post path
// (the capture buffer is still filled in parallel precisely for this).
#define OE_STT_FRAME_MAX_SAMPLES 1280
esp_err_t oe_ws_send_stt_begin(const char *turn_id, uint8_t wake_slot,
                               uint8_t wake_avg_prob, const char *agent_id);
esp_err_t oe_ws_send_stt_frame(const int16_t *samples, size_t n_samples, uint32_t seq);
esp_err_t oe_ws_send_stt_end(const char *turn_id, uint32_t total_samples);
esp_err_t oe_ws_send_stt_abort(const char *turn_id);

// Tell the server the device tore down ambient playback on its own (e.g.
// the user hit the physical mute button). Without this the server keeps its
// ambient session marker + ffmpeg stream alive and the wake-mid-ambient
// logic resurrects the "stopped" ambient after the next turn. reason is a
// short tag for the server log ("mute").
esp_err_t oe_ws_send_ambient_stopped(const char *reason);

// Acknowledge a server-pushed ww_upload. Server uses these to serialize
// per-slot pushes (won't send slot N+1 until N is acked) and to gate the
// voice_config_pushed_version bump on every slot actually landing. err
// is optional and only logged server-side when ok=false.
esp_err_t oe_ws_send_ww_ack(int slot, bool ok, const char *err);

// Device → server alarm acknowledgements. Called from the alarm
// component when an alarm fires (chime/TTS ring loop starts) and when
// the user dismisses (wake-while-firing). Server uses these to cancel
// the ack-timeout watchdog (no email/telegram fallback needed) and to
// clean up its own registry entry.
esp_err_t oe_ws_send_alarm_fired(const char *alarm_id);
esp_err_t oe_ws_send_alarm_acked(const char *alarm_id);

// Stream OTA progress to the server while esp_https_ota runs. `phase` is a
// short tag ("checking" | "downloading" | "applying" | "rebooting" |
// "up_to_date" | "error"). `bytes_done`/`total` are 0 when not applicable;
// `err` may be NULL. Server fans these to the user's browser tabs so the UI
// can show a progress bar without polling.
esp_err_t oe_ws_send_ota_progress(const char *phase, uint32_t bytes_done,
                                  uint32_t total, const char *target_version,
                                  const char *err);

esp_err_t oe_stt_post(const char *server_url, const char *token,
                      const int16_t *pcm_16k_mono, size_t n_samples,
                      char *out_text, size_t out_len);

typedef void (*oe_tts_pcm_cb_t)(const int16_t *pcm_mono, size_t samples, uint32_t rate, void *user);

// `abort` (optional, may be NULL): flip true to abort the streaming read
// promptly mid-response (real teardown). Ambient passes &s_ambient_stop.
// `pause` (optional, may be NULL): flip true to keep the socket open and
// reading but DISCARD incoming audio instead of decoding/playing it. Ambient
// passes &s_ambient_paused so a wake frees the speaker for the command/reply
// WITHOUT tearing the stream down — we just lose the audio for that moment and
// resume playing the live stream when the turn ends. NULL = always decode.
esp_err_t oe_tts_post(const char *server_url, const char *token,
                      const char *text, const char *voice, int wake_slot,
                      oe_tts_pcm_cb_t cb, void *user,
                      volatile bool *abort, volatile bool *pause);

// Monotonic counter of mp3 body bytes received across all oe_tts_post calls
// (ambient + TTS). Heartbeat ambient-stats reads this + computes a delta
// to derive byte_rate over the heartbeat interval. Useful for confirming
// the network stream is actually flowing during a dropout investigation.
uint32_t oe_tts_get_total_bytes_received(void);

// Fetch the raw MP3 bytes for a one-shot TTS marker (server-side cached
// audio for an alarm announcement). Allocates a PSRAM buffer on success;
// caller owns it and must heap_caps_free() when done. Returns ESP_OK with
// *out_buf and *out_len populated, or an error code with *out_buf NULL.
esp_err_t oe_alarm_fetch_audio(const char *server_url, const char *token,
                               const char *marker,
                               uint8_t **out_buf, size_t *out_len);

size_t oe_b64_decoded_len(const char *b64, size_t b64_len);
size_t oe_b64_decode(const char *b64, size_t b64_len, uint8_t *out, size_t out_max);

// UDP diagnostic forwarder (oe_udplog.c). oe_udplog_init parses the host from
// server_url and targets <host>:OE_UDPLOG_PORT. oe_udplog_send is best-effort,
// non-blocking, and safe to call from any normal task context — it does not
// block, allocate, or log. Lets a Wi-Fi-only device be observed like serial,
// and the datagrams survive WS drops. No-op until init succeeds.
#define OE_UDPLOG_PORT 47269
esp_err_t oe_udplog_init(const char *server_url, uint16_t port);
void      oe_udplog_send(const char *line);

// Spawn an OTA check task. Fetches <server_url>/firmware/voice-device/manifest.json,
// compares against the running app's version (from esp_app_get_description),
// and if the manifest version is newer, downloads the app binary via
// esp_https_ota and reboots. Streams ota_progress events over WS for UI.
// Idempotent: a no-op if a check is already in flight.
esp_err_t oe_ota_start_check(const char *server_url);

// Marks the running app as valid after a successful boot, cancelling any
// pending rollback. Call once Wi-Fi is associated and the WS is connected —
// that proves the new image isn't fundamentally broken. Without this,
// IDF will boot the previous slot next time. Safe to call repeatedly.
void oe_ota_mark_running_valid(void);
