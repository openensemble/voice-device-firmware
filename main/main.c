#include "state.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>       // fabsf/M_PI for the DoA direction gate
#include <sys/stat.h>   // stat() for ww_file_matches (skip identical ww re-push)
#include <time.h>       // gmtime_r/strftime for the verify-gate fired_at stamp

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_spiffs.h"

#include "audio_io.h"
#include "xvf3800_ctrl.h"
#include "wakeword.h"
#include "mbedtls/base64.h"   // server-side TTS streaming: decode pushed PCM frames
#include "vad.h"
#include "mp3_decode.h"
#include "oe_client.h"
#include "alarm.h"
#include "airplay.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "captive_portal.h"
#include "cJSON.h"
#include "esp_heap_caps.h"
#include "nvs_creds.h"
#include "leds_buttons.h"
#include "nvs.h"
static const char *TAG = "main";

#include "esp_system.h"
#include "esp_random.h"

// Embedded known-good XVF firmware (formatBCE HA v1.0.7 — what was working
// yesterday morning before today's regression chasing). Pushed to the XMOS
// via I²C DFU on first boot if the migration NVS flag is absent. Single
// flash for end users — they don't need dfu-util.
extern const uint8_t xvf_ha_v1_0_7_bin_start[] asm("_binary_xvf_ha_v1_0_7_bin_start");
extern const uint8_t xvf_ha_v1_0_7_bin_end[]   asm("_binary_xvf_ha_v1_0_7_bin_end");

// NVS flag — set after a successful DFU. Fresh key name so devices that
// were previously migrated to i2s (or any earlier flag) re-run this DFU
// and end up on HA.
#define NVS_KEY_XVF_HA_REVERT "xvf_ha_v2"

static bool xvf_ha_v2_done(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_CREDS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;
    uint8_t v = 0;
    esp_err_t e = nvs_get_u8(h, NVS_KEY_XVF_HA_REVERT, &v);
    nvs_close(h);
    return e == ESP_OK && v == 1;
}

static void xvf_ha_v2_mark_done(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_CREDS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, NVS_KEY_XVF_HA_REVERT, 1);
    nvs_commit(h);
    nvs_close(h);
}

// One-shot XVF firmware migration. On first boot of this ESP firmware, push
// the embedded HA v1.0.7 firmware to the XMOS over I²C, then esp_restart so
// audio_io re-inits cleanly. After success the NVS flag suppresses future
// runs.
static void xvf_migration_task(void *arg)
{
    if (xvf_ha_v2_done()) {
        vTaskDelete(NULL);
        return;
    }

    // Let XVF finish its own internal boot before touching I²C.
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Verify the chip is on the bus before attempting DFU.
    if (xvf3800_xmos_write(XVF_RESID_DFU_CONTROLLER, XVF_CMD_DFU_GETVERSION, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "xvf_migration: XVF not on I²C bus; deferring DFU to next boot");
        vTaskDelete(NULL);
        return;
    }

    // Skip the DFU if the XVF is already running the embedded version. The
    // formatBCE HA v1.0.7 blob reports {1, 0, 7} via XCORE-VOICE GETVERSION;
    // any other value (or a read failure) means we need to (re-)flash it.
    // This is what saves a redundant ~3-min DFU when a user web-flashes both
    // the XVF (via Stage 1) and the ESP (via Stage 2) — the chip already has
    // the right firmware, no need to push it again.
    {
        uint8_t ver[3] = {0};
        if (xvf3800_get_dfu_version(ver) == ESP_OK && ver[0] == 1 && ver[1] == 0 && ver[2] == 7) {
            ESP_LOGI(TAG, "xvf_migration: XVF already on HA v%u.%u.%u — skipping DFU, marking flag done",
                     ver[0], ver[1], ver[2]);
            xvf_ha_v2_mark_done();
            vTaskDelete(NULL);
            return;
        }
        ESP_LOGI(TAG, "xvf_migration: XVF version is not the embedded HA v1.0.7 (got %u.%u.%u) — DFU needed",
                 ver[0], ver[1], ver[2]);
    }

    size_t bin_len = xvf_ha_v1_0_7_bin_end - xvf_ha_v1_0_7_bin_start;
    ESP_LOGI(TAG, "xvf_migration: pushing embedded HA v1.0.7 firmware (%u bytes)...", (unsigned)bin_len);

    // Skip post-DFU version verify; DFU manifest completion + chip reboot
    // is sufficient proof of success.
    esp_err_t e = xvf3800_dfu_apply(xvf_ha_v1_0_7_bin_start, bin_len, 0, 0, 0);
    if (e == ESP_OK) {
        ESP_LOGI(TAG, "xvf_migration: complete — marking done and rebooting ESP for clean audio init");
        xvf_ha_v2_mark_done();
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    } else {
        ESP_LOGE(TAG, "xvf_migration: failed (%s) — leaving flag clear, will retry on next boot",
                 esp_err_to_name(e));
    }
    vTaskDelete(NULL);
}

// Boot-ready indicator — 1 s green flash on the XVF LED ring once the audio
// engine has had time to settle. Doubles as proof that the XVF is reachable
// on I²C from the ESP. Per the original file comment: boot-time I²C
// transactions disrupt the wake-detect path, so we wait 3 s before poking.
static void boot_indicator_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(3000));
    xvf3800_set_led_brightness(120);
    xvf3800_set_led_color(XVF_RGB(0x00, 0xC0, 0x40));   // green
    xvf3800_set_led_effect(XVF_LED_EFFECT_SINGLE);
    vTaskDelay(pdMS_TO_TICKS(1000));
    xvf3800_set_led_brightness(0);
    xvf3800_set_led_effect(XVF_LED_EFFECT_OFF);
    vTaskDelete(NULL);
}

// One-shot AGC freeze with wait-for-quiet (2026-05-15 — see memory
// project_xvf3800_agc_quiet_wait for revert path).
//
// HA-variant default AGC re-shapes the mic spectrum in a way the Piper-
// trained wake-word models don't recognize, so we write PP_AGCONOFF=0 to
// halt adaptation. BUT — that's a *freeze*, not a reset: whatever gain
// coefficient the AGC last computed stays locked. If the room is noisy
// (music, TV, talk) when the freeze fires, AGC has just ramped gain down
// to tame the loud audio, so the locked gain is too low for normal
// speech later → wake-word features starve (`audio_lvl ~5000, feat_max
// ~115, prob=0/255`).
//
// Defer the freeze until we observe sustained acoustic quiet via
// audio_io_get_capture_rms_1s(). Caps total wait so a permanently noisy
// room still gets a freeze rather than no freeze.
static void agc_freeze_task(void *arg)
{
    // Tuned by ear on a quiet bedroom + music-playing test:
    //   - quiet room peak-RMS sits ~500–1500
    //   - speech bursts hit 8k–20k
    //   - music at moderate volume saturates near 30k
    const uint32_t QUIET_RMS_MAX       = 2000;
    const int      QUIET_SECONDS_NEEDED = 3;
    const int      MAX_WAIT_SECONDS    = 60;

    // Initial settle so the I²S engine + capture task are producing frames
    // before we start polling RMS (RMS reads 0 until the first 1-s window
    // rolls over inside capture_task).
    vTaskDelay(pdMS_TO_TICKS(2000));

    int quiet_seconds = 0;
    int total_seconds = 0;
    while (quiet_seconds < QUIET_SECONDS_NEEDED && total_seconds < MAX_WAIT_SECONDS) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        // While AirPlay is actively streaming, the speaker is loud,
        // the XVF AGC has attenuated mic gain to compensate, and any
        // freeze captured now would lock in the wrong (low) gain —
        // starving the wake-word once music stops. Defer instead:
        // don't count this second toward MAX_WAIT_SECONDS and don't
        // touch quiet_seconds. When the user stops casting, the
        // XVF AGC takes ~1-2 s to recover to quiet-room gain, which
        // shows up as residual "noise" on the RMS meter and naturally
        // resets quiet_seconds via the else branch below — so by the
        // time we successfully count 3 quiet seconds, the AGC is
        // settled and the freeze captures the correct high gain.
        if (airplay_is_streaming()) continue;
        // Same reasoning for the device's OWN output (TTS reply, routine
        // announcement, auto-resumed ambient): a boot that lands into
        // playback would otherwise burn the wait cap against speaker noise
        // and freeze the AGC at the gain it adapted DOWN to — the exact
        // starved-wake-word state this task exists to prevent. Don't count
        // these seconds toward the cap either; like the AirPlay case, the
        // AGC's own ~1-2 s recovery after playback ends resets quiet_seconds
        // naturally, so the freeze always captures settled quiet-room gain.
        if (audio_io_playback_active()) continue;
        total_seconds++;
        uint32_t rms = audio_io_get_capture_rms_1s();
        if (rms > 0 && rms < QUIET_RMS_MAX) {
            quiet_seconds++;
            ESP_LOGI(TAG, "agc_freeze: quiet sec %d/%d (rms=%u)",
                     quiet_seconds, QUIET_SECONDS_NEEDED, (unsigned)rms);
        } else {
            if (quiet_seconds > 0) {
                ESP_LOGI(TAG, "agc_freeze: noise (rms=%u), reset", (unsigned)rms);
            }
            quiet_seconds = 0;
        }
    }

    uint8_t zero_int32[4] = {0, 0, 0, 0};
    esp_err_t e = xvf3800_xmos_write(XVF_RESID_PP, XVF_CMD_PP_AGCONOFF, zero_int32, 4);
    if (e == ESP_OK) {
        ESP_LOGI(TAG, "agc_freeze: PP_AGCONOFF=0 OK after %ds (quiet=%d/%d, cap=%ds)",
                 total_seconds, quiet_seconds, QUIET_SECONDS_NEEDED, MAX_WAIT_SECONDS);
    } else {
        ESP_LOGW(TAG, "agc_freeze: PP_AGCONOFF write failed (%s)", esp_err_to_name(e));
    }
    vTaskDelete(NULL);
}

dev_config_t g_dev_config = {0};

// Gate path + canonical device id form one runtime configuration pair. The WS
// task can reconcile them while the capture/verify tasks are active, so publish
// and snapshot the pair under one lock. NVS persists policy before identity so
// an interrupted update cannot turn an enabled gate into a silent bypass.
static portMUX_TYPE s_verify_config_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_verify_gate_config_persisted = false;
// Missing NVS state is not the same as an explicit remote disable. Until a
// server_caps from an allowed paired origin supplies the fixed or empty path,
// genuine wakes must remain blocked.
static bool s_verify_gate_config_known = false;

static void verify_gate_config_snapshot(char path[OE_URL_MAX],
                                        char device_id[OE_DEVICE_ID_MAX])
{
    portENTER_CRITICAL(&s_verify_config_mux);
    memcpy(path, g_dev_config.verify_gate_path, OE_URL_MAX);
    memcpy(device_id, g_dev_config.device_id, OE_DEVICE_ID_MAX);
    portEXIT_CRITICAL(&s_verify_config_mux);
}

static bool server_device_id_valid(const char *device_id)
{
    if (!device_id) return false;
    size_t n = strnlen(device_id, OE_DEVICE_ID_MAX);
    if (n == 0 || n >= OE_DEVICE_ID_MAX) return false;
    for (size_t i = 0; i < n; ++i) {
        unsigned char c = (unsigned char)device_id[i];
        if (c <= 0x20 || c == 0x7f) return false;
    }
    return true;
}

static bool server_verify_gate_path_valid(const char *path)
{
    if (!path) return false;
    return path[0] == '\0' || strcmp(path, OE_VERIFY_GATE_PATH) == 0;
}

// Follow-up and conversation-mode barge intentionally open capture without a
// wake word. UNKNOWN policy never authorizes them. Enabled policy requires an
// allowed paired origin: HTTPS, or numeric private-LAN HTTP by deployment
// policy. Explicit-disabled policy preserves the legacy behavior.
static bool server_control_may_open_ungated_capture(void)
{
    bool known;
    bool gate_enabled;
    portENTER_CRITICAL(&s_verify_config_mux);
    known = s_verify_gate_config_known;
    gate_enabled = g_dev_config.verify_gate_path[0] != '\0';
    portEXIT_CRITICAL(&s_verify_config_mux);
    return known &&
           (!gate_enabled ||
            oe_verify_gate_origin_allowed(g_dev_config.server_url));
}

static void apply_verify_gate_server_caps(const cJSON *j)
{
    // Gate policy arrives on the operational WebSocket. Plaintext is accepted
    // only when the paired target is a numeric private-LAN IPv4 origin.
    if (!oe_verify_gate_origin_allowed(g_dev_config.server_url)) {
        ESP_LOGW(TAG, "server_caps: gate config ignored on disallowed OE origin");
        return;
    }

    const cJSON *jid = cJSON_GetObjectItemCaseSensitive(j, "device_id");
    const cJSON *jpath = cJSON_GetObjectItemCaseSensitive(j, "verify_gate_path");
    const cJSON *jlegacy =
        cJSON_GetObjectItemCaseSensitive(j, "verify_gate_url");
    bool apply_id = false;
    bool apply_path = false;
    const char *path_value = NULL;

    if (jid) {
        apply_id = cJSON_IsString(jid) && server_device_id_valid(jid->valuestring);
        if (!apply_id) ESP_LOGW(TAG, "server_caps: ignoring invalid device_id");
    }
    if (jpath) {
        apply_path = cJSON_IsString(jpath) &&
                     server_verify_gate_path_valid(jpath->valuestring);
        if (apply_path) path_value = jpath->valuestring;
        else ESP_LOGW(TAG, "server_caps: ignoring invalid verify_gate_path");
    } else if (jlegacy) {
        // OE sends an empty legacy field as a kill switch for old firmware.
        // New firmware may honor only that empty value. A non-empty direct URL
        // is never accepted because it would receive the voice-device bearer.
        apply_path = cJSON_IsString(jlegacy) &&
                     jlegacy->valuestring &&
                     jlegacy->valuestring[0] == '\0';
        if (apply_path) path_value = "";
        else ESP_LOGW(TAG, "server_caps: refusing legacy direct verify_gate_url");
    }
    // Missing fields come from an older server and intentionally preserve the
    // current values. Empty device ids are never authoritative; an explicitly
    // empty path is valid and disables the gate.
    if (!apply_id && !apply_path) return;

    char next_path[OE_URL_MAX];
    char next_device_id[OE_DEVICE_ID_MAX];
    bool next_known;
    verify_gate_config_snapshot(next_path, next_device_id);
    portENTER_CRITICAL(&s_verify_config_mux);
    next_known = s_verify_gate_config_known;
    portEXIT_CRITICAL(&s_verify_config_mux);
    if (apply_id) snprintf(next_device_id, sizeof(next_device_id), "%s", jid->valuestring);
    if (apply_path) {
        snprintf(next_path, sizeof(next_path), "%s", path_value);
        next_known = true;
    }

    bool changed;
    portENTER_CRITICAL(&s_verify_config_mux);
    changed = strcmp(next_path, g_dev_config.verify_gate_path) != 0 ||
              strcmp(next_device_id, g_dev_config.device_id) != 0 ||
              next_known != s_verify_gate_config_known;
    portEXIT_CRITICAL(&s_verify_config_mux);
    if (!changed && s_verify_gate_config_persisted) return;

    if (next_known && next_path[0]) {
        // Publish a blocked transition state before the slower NVS operation.
        // Otherwise disabled->enabled leaves a window in which capture still
        // observes known+empty and admits an ungated wake.
        portENTER_CRITICAL(&s_verify_config_mux);
        s_verify_gate_config_known = false;
        portEXIT_CRITICAL(&s_verify_config_mux);
    }

    esp_err_t e;
    if (next_known) {
        // Persistence writes an enabling path before identity, so interruption
        // can leave a blocked missing/stale identity but never an empty-path
        // bypass. See nvs_creds_set_verify_gate_config().
        e = nvs_creds_set_verify_gate_config(next_device_id, next_path);
        s_verify_gate_config_persisted = e == ESP_OK;
    } else {
        // An identity-only update must not materialize an empty gate key:
        // absence remains UNKNOWN/BLOCKED until an explicit path field arrives.
        e = nvs_creds_set_device_id(next_device_id);
        s_verify_gate_config_persisted = false;
    }
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "server_caps: gate config NVS write failed: %s",
                 esp_err_to_name(e));
    }

    portENTER_CRITICAL(&s_verify_config_mux);
    memcpy(g_dev_config.device_id, next_device_id, sizeof(g_dev_config.device_id));
    memcpy(g_dev_config.verify_gate_path, next_path, sizeof(g_dev_config.verify_gate_path));
    s_verify_gate_config_known = next_known;
    portEXIT_CRITICAL(&s_verify_config_mux);
    ESP_LOGI(TAG, "server_caps: verify gate %s (device_id=%s%s)",
             !next_known ? "pending"
                 : (next_path[0] && next_device_id[0]
                        ? "configured"
                        : "disabled"),
             next_device_id[0] ? next_device_id : "(pending)",
             e == ESP_OK ? "" : ", volatile");
}

// Wake-word slots loaded concurrently. The wakewords SPIFFS partition is built
// from firmware/wakewords/slot{0..N}.tflite + slot{0..N}.json — fresh flash
// ships slots 0 + 1 (hey_ensemble + hey_computer); slots 2-5 start empty and are
// populated via the server's ww_upload WS message when the user assigns a
// wake word to that slot in OE.
//
// All loaded slots run in parallel; whichever fires first wins per audio
// frame. The slot index is included in the chat message ("wake_slot": N) so
// the server can route to slot_assignments[N] (a per-device map managed in
// Settings → Voice devices).
//
// The wakewords partition is 0x80000 (512 KB) per partitions.csv. Measured
// with esp-idf spiffsgen.py at this project's SPIFFS geometry (2026-07-06):
// 459,364 B of slot files fit, 476,034 B did not. Six ~63 KB (v1) pairs fit
// comfortably; the retrained ~79 KB pairs (tensor_arena_size 78240) max out
// at FIVE — six of those overflow. The OE server refuses over-budget pushes
// up front (lib/voice-config.mjs, WW_SPIFFS_BUDGET_BYTES = 460000), which
// matters because apply_ww_upload below unlinks the old slot file before
// writing — a failed write would leave that slot empty. Bumping capacity
// would require resizing the partition (USB re-flash) or shrinking the
// SPIFFS image somehow.
#define WW_NUM_SLOTS 6
static wakeword_t *s_ww[WW_NUM_SLOTS] = { NULL };
static uint8_t     s_active_slot      = 0;  // which slot fired the current utterance
static uint8_t     s_active_wake_prob = 0;  // sliding-window avg prob (0..255) of the wake that fired
// Manifest-derived identity snapshotted with the winning detection. It must
// travel with the fire: reading the slot manifest later would let a concurrent
// hot-swap relabel an already-fired model.
static char        s_active_wake_slug[WW_WAKE_SLUG_MAX] = "";

// Per-slot manifest (default) probability cutoff, mirrored at file scope so the
// playback-aware cutoff task can restore the CURRENT cutoff after TTS/AirPlay
// ends. Seeded from the loaded models at wake-task start AND refreshed on every
// hot-swap in apply_ww_upload. Previously this lived as a task-local snapshot
// captured once at boot: a pushed cutoff hot-swapped the model fine, but the
// next playback edge reverted it to the stale boot value — so a cutoff change
// only stuck after a reboot. Keeping it here + refreshing on hot-swap makes a
// pushed cutoff durable without a reboot. (uint8 writes are atomic on ESP32, so
// the cross-task read from the wake task needs no lock.)
static uint8_t     s_default_cutoff[WW_NUM_SLOTS] = {0};

// Server-side TTS streaming (push model). The server pushes PCM frames over the
// WS; the device plays them. Declared here so the WS callback (above the player
// task) can reference them. See stream_finalize_task / stream_abort_local.
static volatile bool s_stream_active  = false;   // between tts_audio_begin and finalize
static volatile bool s_stream_end_req = false;   // tts_audio_end received; finalize task drains
// Legacy HTTP TTS can overlap the pushed stream's teardown; the WS disconnect
// path needs this ownership bit early enough to preserve its speech output.
static volatile bool s_legacy_tts_active = false;
// Exact server tag of the active pushed stream. A verify hold requires this
// to match s_turn_id; untagged announcements cannot be safely paused/resumed
// through the turn-scoped server protocol and are therefore fail-closed.
static char          s_stream_turn_id[24] = "";
// ACCEPT quarantines turnless announcement frames until the old announcement
// proves terminal or correctly tagged content from the promoted turn begins.
// This covers frames already queued on TCP even after the server hold/stop.
static bool          s_drop_untagged_tts = false;
static uint8_t       s_pcm_frame[4096];          // base64 decode scratch (WS callback is single-threaded)
// Liveness timestamp for the streamed-TTS path: last tts_audio_begin/tts_audio
// arrival (esp_timer us). stream_finalize_task's stall watchdog uses it to
// tear down a SPEAKING state whose tts_audio_end never arrives (server crash
// mid-stream on a healthy socket). Without this, s_stream_active had NO
// timeout: amp stayed on (AEC suppressing the mic) and every non-owner wake
// slot stayed gated forever.
static volatile int64_t s_last_tts_frame_us = 0;
// Burst-close signalling: a tts_audio_end carrying pending:true means "this
// turn ISN'T over — the server just paused output while slow work grinds"
// (delegation, long tool call). The device then shows the THINKING pattern
// while the mic stays fully open — the LEDs say "working on it" without
// costing any listening. Auto-clears after a deadline so an abandoned turn
// can't leave the ring spinning forever.
static volatile bool    s_stream_end_pending = false;
static volatile int64_t s_wait_led_until_us  = 0;
#define WAIT_LED_TIMEOUT_US (120LL * 1000 * 1000)
// 20 s, not lower: a slow cloned-voice sentence can legitimately gap frames
// for several seconds with the ring drained (synth latency), and firing early
// truncates the reply — frames that arrive after teardown are dropped because
// s_stream_active is false. Dead-Pocket is already handled promptly by the
// server's fatal-synth bail; this is the backstop for a crashed/hung server.
#define TTS_STREAM_STALL_TIMEOUT_MS 20000
// True while capture_and_drive_task is recording a command utterance. File
// scope (not a task-local) so ws_event_cb can refuse to treat a stale `done`
// from an aborted turn as turn-terminal while the user is mid-command — the
// old behavior IDLE'd the UI and airplay_resume()'d over the user's speech.
static volatile bool s_in_utterance = false;

// ── Turn correlation ids ─────────────────────────────────────────────────────
// Minted at every wake/follow-up commit, sent in `chat`, echoed back by the
// server on token/done/tts_audio_*/await_followup/error. Events carrying a
// DIFFERENT turn_id belong to an aborted/prior turn and are dropped. Events
// with NO turn_id (older server) are always accepted — never drop on missing.
// Written only by capture_and_drive_task; read by websocket_task. 16-byte
// writes aren't atomic, but the reader tolerates a torn read as at worst one
// mis-dropped/mis-accepted frame during the commit instant.
static char     s_turn_id[24] = "";
static char     s_turn_prefix[5] = "0000";   // 4 hex chars from esp_random at boot
static uint32_t s_turn_counter = 0;

static void mint_turn_id_into(char *out, size_t out_len)
{
    if (!out || out_len == 0) return;
    snprintf(out, out_len, "%s-%lu",
             s_turn_prefix, (unsigned long)(++s_turn_counter));
}

static void mint_turn_id(void)
{
    mint_turn_id_into(s_turn_id, sizeof(s_turn_id));
}

// A verify-gated wake is not the current turn until the gate accepts it.
// Keep its id separate so the active reply's tagged token/TTS/end events
// continue to be accepted (and can be resumed after a reject).
static char          s_prov_turn_id[24] = "";
static volatile bool s_verify_playback_hold_active = false;
static volatile bool s_verify_cancelled = false;
// Alarm start is serialized with gate release. If its amp callback arrives
// while a provisional hold owns physical silence, the release path consumes
// this handoff; if it arrives after release, the callback enables playback
// directly. This closes the final snapshot-to-release race.
static bool          s_alarm_amp_deferred = false;
// Monotonic ownership token for the reversible hold. A delayed RESUME retry
// may act only while this is still the generation that requested it; the next
// provisional wake invalidates that retry before issuing its own PAUSE.
static volatile uint32_t s_verify_hold_generation = 0;
// A turnless announcement can be active before the hold or begin behind the
// server-side hold-id latch. Track that ownership explicitly so REJECT restores
// it even at boot, where there is no established turn id to compare.
static bool s_verify_hold_untagged_stream_owned = false;
// Serializes acquisition/release/commit of the provisional playback hold
// against mute, alarm, disconnect, and the final state transition of each
// TTS producer.  The hold spans several independent components, so publishing
// a bare boolean without this mutex leaves an acquisition window where a mute
// can release flags that have not been set yet (then begin sets them forever).
static SemaphoreHandle_t s_verify_lifecycle_mutex = NULL;
// ACCEPT closes admission before waiting for any already-admitted speech-ring
// writer to finish.  This prevents an old WS frame that passed correlation
// just before promotion from refilling the ring after the destructive flush.
static volatile bool     s_speech_writes_blocked = false;
static volatile uint32_t s_speech_writers = 0;

// Identity/UI-adjacent state changed by detector arbitration before a verdict.
// These declarations must precede ws_event_cb: a tagged await_followup event
// received during verification still belongs to this previous slot.
static bool              s_prov_identity_valid = false;
static uint8_t           s_prov_prev_slot = 0;
static uint8_t           s_prov_prev_prob = 0;
static char              s_prov_prev_slug[WW_WAKE_SLUG_MAX] = "";
static int64_t           s_prov_prev_followup_until_us = 0;
static bool              s_prov_bearing_pending = false;
static float             s_prov_bearing = -1.0f;

static bool vg_abort_for_mute(void);
static void vg_request_cancel(void);

static bool vg_lifecycle_take(void)
{
    return s_verify_lifecycle_mutex &&
           xSemaphoreTake(s_verify_lifecycle_mutex, portMAX_DELAY) == pdTRUE;
}

static void vg_lifecycle_give(void)
{
    xSemaphoreGive(s_verify_lifecycle_mutex);
}

// True when the event names a turn that is NOT the device's current one.
// Missing turn_id (NULL/empty, i.e. older server) always passes.
static bool evt_turn_stale(const oe_ws_payload_t *evt)
{
    if (!evt->turn_id || !evt->turn_id[0]) return false;
    if (!s_turn_id[0]) return true;   // we have no live turn — event is from a past life
    return strcmp(evt->turn_id, s_turn_id) != 0;
}

// PCM BEGIN establishes one stream identity; every AUDIO/END must match both
// its tag value and its tagged-vs-untagged class. Otherwise a late untagged
// announcement can append into (or terminate) a newer tagged reply.
// Callers hold s_verify_lifecycle_mutex.
static bool evt_matches_active_stream(const oe_ws_payload_t *evt)
{
    if (!s_stream_active) return false;
    const bool evt_tagged = evt->turn_id && evt->turn_id[0];
    const bool stream_tagged = s_stream_turn_id[0];
    if (evt_tagged != stream_tagged) return false;
    return !evt_tagged ||
           strcmp(evt->turn_id, s_stream_turn_id) == 0;
}

// Server capability flags, learned from `server_caps` after auth and reset on
// disconnect (the next server may be older). Gate every NEW device→server
// message type on these so a firmware upgrade never breaks against an old
// server. turn_id fields on chat/stop are NOT gated — old servers ignore
// unknown fields harmlessly.
static volatile bool s_caps_tts_pause  = false;
static volatile bool s_caps_stt_stream = false;
static volatile bool s_caps_turn_ids   = false;
static volatile bool s_caps_tts_hold_ids = false;

// ── Speech barge-in (conversation mode): pause-then-verify ──────────────────
// While the device speaks a streamed reply, a burst of mic energy PAUSES
// playback locally (one call — the speaker mutes before any network hop) and
// drops amp_en so the XVF AEC stops suppressing the mic. With the room now
// quiet and the mic hot, we require sustained speech to confirm; a false
// alarm (dish clatter, TV transient) resumes the reply where it paused. A
// confirmed barge captures a normal utterance — and even then the reply is
// only flushed once STT proves the interjection real (a cough or "um" costs
// a ~1-3 s pause, never the rest of the reply). Every state has a local
// deadline: this converges with zero network.
//
// Tunables — retuned 2026-07-04 from the first live Kitchen session:
//  - CONFIRM 350→160 + SETTLE 150→80: a crisp single-word "stop" (~300-400 ms)
//    couldn't accumulate 350 ms of speech after a 150 ms settle — observed as
//    a "false alarm (80ms speech)" that swallowed a stop. Stage C (transcript
//    check) is the designed false-positive gate, so stage B can be permissive:
//    a wrong confirm now costs a ~1.5 s pause-and-resume, not a lost command.
//  - CONSEC 2→3 + WARMUP: three self-triggered candidates in one reply
//    (fe 4-22× floor, 0 ms speech) — loud TTS peaks early in the reply while
//    the EMA floor was still seeding from the quiet lead-in. Candidates are
//    disabled for the first WARMUP frames of each reply while the floor learns.
#define BARGE_CONSEC_FRAMES     3      // 80 ms frames above trigger to become a candidate
#define BARGE_FLOOR_MULT        4      // trigger = max(floor*MULT, BARGE_TRIGGER_MIN)
// Absolute candidate minimum — deliberately BELOW the 800k speech threshold:
// during a reply the amp is on and the XVF AEC suppresses the user's voice,
// so normal-volume speech reads far quieter than it does idle (field: "have
// to talk louder to barge in"). The 3-frame streak + floor×4 + warmup +
// false-alarm cooldown carry the false-positive load; a wrong candidate
// costs a ~1s pause-and-resume, a missed one costs the user their turn.
#define BARGE_TRIGGER_MIN       400000
// Verify progress is counted in FRAMES PROCESSED, not wall time. The
// candidate branch sends tts_pause over the WS, which can block this task
// ~1 s when the socket is busy (paced TTS + ambient chatter) — with a wall
// clock, the whole verify window expired during the send and the user's
// queued speech frames were judged against a dead deadline (field:
// candidate→false-alarm in 15 ms with "80ms speech"). Frames are real time
// by construction (80 ms each off the capture ring) and immune to stalls.
#define BARGE_SETTLE_FRAMES        1   // skip 1st verify frame (amp-off gain settle)
#define BARGE_CONFIRM_SPEECH_MS    160 // cumulative speech to confirm a real barge
#define BARGE_VERIFY_WINDOW_FRAMES 13  // ~1 s of audio, then resume
#define BARGE_WARMUP_FRAMES     8      // ~640 ms of floor learning at reply start, no candidates
// How far BEFORE the candidate the barge prepend reaches: covers the trigger
// frames plus the sub-threshold onset ramp ("Wh-" of "Who"). Kept modest so
// the prepend stays mostly user speech — audio earlier than this is dominated
// by our own reply bleed (speaker was still on), which would prefix the
// transcript with assistant words.
#define BARGE_PREROLL_LEAD_MS   320
typedef enum { BARGE_NONE = 0, BARGE_VERIFYING } barge_state_t;
static barge_state_t     s_barge_state = BARGE_NONE;
static int               s_barge_consec = 0;
static int               s_barge_verify_frames = 0;
static int               s_barge_warmup = 0;   // frames of floor seeding this reply
// After a false alarm (verify or stage-C says "not real"), hold off new
// candidates briefly. Without this, sustained loud non-speech (TV burst,
// music passage) re-candidates the instant the reply resumes — a 1 s
// pause-resume stutter loop. The floor keeps absorbing the loud frames
// during the cooldown, so the baseline usually catches up by expiry.
#define BARGE_FALSE_ALARM_COOLDOWN_MS 2000
static int64_t           s_barge_cooldown_until_us = 0;
static int64_t           s_barge_started_us = 0;
static uint32_t          s_barge_speech_ms = 0;
static uint32_t          s_speak_floor = 0;         // EMA of frame energy during SPEAKING
static bool              s_barge_capture = false;   // current utterance came from a confirmed barge
// True between our audio_io_pause_playback() and the resume/commit. Suspends
// the tts stall watchdog (the server pacer is deliberately stalled) and marks
// that WE own the pause flag (AirPlay's user-pause must not be clobbered).
static volatile bool     s_paused_for_barge = false;
static volatile bool     s_conversation_mode = false;

// ── Streaming STT session (gated on server_caps.stt_stream) ─────────────────
// Wake/follow-up captures stream frames to the server as they arrive (upload
// overlaps speech; no 30 s blocking POST on this task). The capture buffer
// keeps filling in parallel as the fallback: any send failure flips
// s_stt_send_failed and VAD-end takes the buffered oe_stt_post road instead.
// Barge captures NEVER stream — stage C needs the transcript on-device to
// decide resume-vs-commit, so they always use the local HTTP path.
static bool     s_stt_streaming = false;
static bool     s_stt_send_failed = false;
static uint32_t s_stt_seq = 0;

// Mean-square frame energy (same metric as the VAD's threshold comparisons).
static inline uint32_t frame_energy(const int16_t *samples, size_t n)
{
    if (n == 0) return 0;
    uint64_t sum_sq = 0;
    for (size_t i = 0; i < n; ++i) {
        int32_t s = samples[i];
        sum_sq += (uint64_t)(s * s);
    }
    return (uint32_t)(sum_sq / n);
}
static volatile bool s_ws_connected = false;
// Retries are scoped to one authenticated websocket lifetime. An old STOP or
// RESUME must never migrate onto a replacement connection after reconnect.
static volatile uint32_t s_ws_connection_epoch = 0;
static volatile bool s_ota_marked_valid = false;
// A memory-recovery OTA may resume only after both the authenticated
// server_caps message and the tail of operational boot. oe_ws_start is
// asynchronous, so caps can otherwise race the remaining task allocations.
static volatile bool s_authenticated_caps_seen = false;
static volatile bool s_operational_boot_ready = false;

static void maybe_resume_pending_ota(void)
{
    if (!s_authenticated_caps_seen || !s_operational_boot_ready) return;
    esp_err_t e = oe_ota_resume_pending(g_dev_config.server_url);
    if (e == ESP_OK) {
        ESP_LOGI(TAG, "pending OTA memory recovery resumed");
    } else if (e != ESP_ERR_NOT_FOUND && e != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "pending OTA memory recovery not resumed: %s",
                 esp_err_to_name(e));
    }
}

// THINKING-state gate. Set when VAD ends and we ship the utterance to STT/LLM;
// cleared the instant the TTS worker enters SPEAKING (so barge-in during TTS
// still works) or when the reply path errors back to IDLE. While true, the
// audio loop stops feeding wake-word inference so a false positive during the
// brief THINKING window can't re-arm LISTENING and consume TTS leakage as a
// new utterance. See git blame for the 2026-05-15 false-wake-during-THINKING
// incident this guards.
static volatile bool s_awaiting_reply = false;
// Timestamp (esp_timer us) when s_awaiting_reply was armed. The capture loop's
// THINKING gate uses it to re-open the wake feed if the server reply never
// arrives (WS dropped mid-turn / server hang) instead of staying deaf to every
// wake until reboot. 0 = not armed.
static volatile int64_t s_awaiting_since_us = 0;
static portMUX_TYPE s_time_mux = portMUX_INITIALIZER_UNLOCKED;

// Current servers intentionally leave background announcements untagged. They
// may start behind a reversible verify hold (so REJECT can still play them),
// but once a command has committed, an untagged late event must not mutate or
// mix into that correlated turn. Normal turn replies are always tagged when
// verify gating is enabled (s_caps_turn_ids is mandatory).
static bool evt_untagged_conflicts_with_busy_turn(
    const oe_ws_payload_t *evt)
{
    if (evt->turn_id && evt->turn_id[0]) return false;
    if (s_drop_untagged_tts) return true;
    if (s_verify_playback_hold_active) return false;
    return s_in_utterance || s_awaiting_reply ||
           (s_stream_active && s_stream_turn_id[0]);
}
// Watchdog ceiling. Generous so a slow-but-valid delegated LLM reply (observed
// up to ~50s) is never cut off; the OE_WS_EVT_DISCONNECTED handler clears the
// gate instantly in the common dead-socket case, so this only backstops a live
// socket that goes silent (server crash mid-turn).
#define AWAITING_REPLY_TIMEOUT_MS 90000

// Follow-up listen window: when set non-zero, the capture loop accepts a
// VAD-start as wake fire (bypasses the wake-word) until the timestamp
// expires. Server sets this via OE_WS_EVT_AWAIT_FOLLOWUP when its last
// reply ended with a question — so the user can answer without saying
// the wake word again. esp_timer_get_time() microseconds.
static volatile int64_t s_followup_until_us = 0;
// Deferred follow-up: when await_followup arrives while TTS is still streaming,
// we stash the window length here and only start the countdown once playback
// actually drains (see stream_finalize_task). Otherwise the window would tick
// down during the spoken reply and expire before the user can answer.
static volatile int     s_followup_pending_ms = 0;
#define FOLLOWUP_WINDOW_DEFAULT_MS 5000
#define FOLLOWUP_WINDOW_MAX_MS    15000
// Slot the conversation was on when follow-up was armed. Any utterance
// captured during the window — whether a VAD-start or a (potentially
// false) wake fire on a different slot — gets routed back to THIS slot
// so the answer always lands on the user who asked the original turn.
static volatile uint8_t s_followup_slot = 0;
// Single speech-energy threshold (mean-square of int16 samples, ~-31 dBFS)
// shared by the VAD config and frame_is_speech. These used to be two
// literals ("800000, same as VAD") that could drift apart.
#define VOICE_ENERGY_THRESHOLD 800000
// Follow-up window trigger runs LOWER than the VAD threshold: the window is
// the least sensitive listener in the system (a blunt full-frame energy gate
// vs the wake model's trained sensitivity), and a soft-spoken answer that
// only grazes 800k triggers a word late. A too-eager trigger is cheap — the
// capture just ends as no-speech / an empty transcript apology.
#define FOLLOWUP_TRIGGER_ENERGY 300000
// …cheap in a QUIET room. Near speakers the fixed 300k answered the TV:
// dialog/music bleed sits above 300k, every window fired on it, and STT
// happily transcribed the TV line as the user's answer. So the effective
// trigger is floor-relative like the barge detector's: an EMA of idle-room
// energy (mic hot, nothing playing) raises the bar in a noisy placement and
// leaves quiet rooms untouched (floor≈0 → the max() keeps 300k). Frames at or
// above VOICE_ENERGY_THRESHOLD never feed the EMA — firm speech and TV peaks
// must not shape the baseline they're judged against; steady sub-800k bleed
// is exactly what we want absorbed. Accepted limit: dialog PEAKS above the
// raised trigger still fire and transcribe — rejecting other-people's-speech
// outright needs speaker-ID upstream, not an energy gate.
#define FOLLOWUP_FLOOR_MULT 3
static uint32_t s_room_floor = 0;   // EMA (α=1/8) of idle-room frame energy

// ── DoA direction gate (0.2.75) ──────────────────────────────────────────────
// Field data 2026-07-07 (Kitchen, music on nearby speakers): the user sat at
// azimuth ~1.08 rad all session; the speakers tracked at ~0.45 and ~4.72.
// Both "she stopped mid-reply to listen" incidents were barge captures whose
// speech energy sat entirely at a speaker bearing — while during the user's
// real utterances the auto-select beam pointed at HIM even with music
// playing (the XVF's auto-select prefers speech-like sources). So: anchor
// the turn's bearing when a wake word commits (the wake phrase is
// proof-of-user; barge/follow-up captures are NOT and never move the
// anchor), then let barge candidates and follow-up fires proceed only if a
// tracked beam points near that bearing.
//
// Fail-open everywhere: no anchor, or no sufficiently fresh reading → behave
// exactly like 0.2.74. The gate only REJECTS on positive evidence (fresh
// reading, every considered beam off-axis). A rejected candidate costs
// nothing audible; a wrongly allowed one still faces stage B/C.
//
// The free-running beam (index 2) is deliberately NOT consulted: it parks at
// exactly π/2 between transients (an idle default in the field logs), which
// would leak through the gate whenever the anchor lands near 1.57.
#define DOA_GATE_TOLERANCE_RAD      0.35f     // user jitter ±0.01; speakers sat 0.6+ away
#define DOA_GATE_MAX_AGE_US         1500000   // cache older than this = fail open
#define DOA_GATE_REJECT_COOLDOWN_US 1000000   // barge streak holdoff after a rejection
static portMUX_TYPE s_doa_mux = portMUX_INITIALIZER_UNLOCKED;
static float   s_doa_az[4] = {0};        // beam1, beam2, free-running, auto-select
static int64_t s_doa_fresh_us = 0;       // 0 = no fresh reading yet
static float   s_turn_bearing = -1.0f;   // <0 = unset; anchored at wake commit
// Defined with the probe task near boot_operational; the wake-commit anchor
// calls it directly for a fresh reading (the idle-cadence cache can be ~2 s
// old, which would fail the anchor open in exactly the music-playing case
// the gate exists for).
static bool doa_read4(uint8_t cmd, float vals[4], uint8_t *last_st);

static inline float doa_ang_dist(float a, float b)
{
    float d = fabsf(a - b);
    return d > (float)M_PI ? (2.0f * (float)M_PI - d) : d;
}

// True = proceed. Logs rejections (throttled — a loud speaker can re-trip the
// energy triggers every frame during a follow-up window).
static bool doa_gate_allows(const char *what)
{
    float az[4], bearing;
    int64_t fresh;
    taskENTER_CRITICAL(&s_doa_mux);
    memcpy(az, (const void *)s_doa_az, sizeof(az));
    fresh = s_doa_fresh_us;
    bearing = s_turn_bearing;
    taskEXIT_CRITICAL(&s_doa_mux);
    if (bearing < 0 || fresh == 0) return true;
    if (esp_timer_get_time() - fresh > DOA_GATE_MAX_AGE_US) return true;
    if (doa_ang_dist(az[0], bearing) <= DOA_GATE_TOLERANCE_RAD ||
        doa_ang_dist(az[1], bearing) <= DOA_GATE_TOLERANCE_RAD ||
        doa_ang_dist(az[3], bearing) <= DOA_GATE_TOLERANCE_RAD) return true;
    static int64_t s_last_reject_log_us = 0;
    int64_t now_us = esp_timer_get_time();
    if (now_us - s_last_reject_log_us > 1000000) {
        s_last_reject_log_us = now_us;
        char gl[128];
        snprintf(gl, sizeof(gl), "[doagate] reject %s az=%.2f,%.2f,%.2f,%.2f turn=%.2f",
                 what, (double)az[0], (double)az[1], (double)az[2], (double)az[3],
                 (double)bearing);
        ESP_LOGI(TAG, "%s", gl); oe_udplog_send(gl);
    }
    return false;
}
static vad_state_t *s_vad = NULL;
static QueueHandle_t s_sentence_q = NULL;

// Pre-roll: a rolling window of the most recent mic audio, kept while idle so
// a follow-up answer captured via VAD-start (which by definition triggers
// AFTER speech has begun) can prepend the onset instead of clipping the first
// syllable. Reset every time a follow-up window arms so it can never contain
// the tail of our own TTS. PSRAM; feature silently disabled if alloc fails.
//
// 1.2 s (was 400 ms): field data 2026-07-04 showed barge captures losing the
// first word ("Who started…" → "started…") because onset→confirm spans
// ~700 ms+ and the ring couldn't reach back far enough. Barge prepends a
// computed span (candidate age + BARGE_PREROLL_LEAD_MS); follow-up prepends
// whatever accumulated since the window armed.
//
// 2.6 s (was 1.2 s), 2026-07-24: the verify gate needs the full wake window
// [fire−2.0 s, fire+0.5 s], so the ring must hold ≥2.0 s of pre-fire audio
// with margin. At fire the frozen ring (preroll_append stops once
// s_in_utterance) supplies the [fire−2.0 s, fire] slice; the +0.5 s tail is
// captured over the next frames into the wake-window buffer. ~83 KB PSRAM.
#define PREROLL_SAMPLES (16000 * 2600 / 1000)   // 2.6 s @ 16 kHz mono
static int16_t *s_preroll_buf = NULL;
static size_t   s_preroll_head = 0;     // next write index (circular)
static size_t   s_preroll_filled = 0;   // valid samples, caps at PREROLL_SAMPLES

static void preroll_reset(void)
{
    s_preroll_head = 0;
    s_preroll_filled = 0;
}

static void preroll_append(const int16_t *samples, size_t n)
{
    if (!s_preroll_buf || n == 0) return;
    for (size_t i = 0; i < n; ++i) {
        s_preroll_buf[s_preroll_head] = samples[i];
        s_preroll_head = (s_preroll_head + 1) % PREROLL_SAMPLES;
    }
    s_preroll_filled += n;
    if (s_preroll_filled > PREROLL_SAMPLES) s_preroll_filled = PREROLL_SAMPLES;
}

// Copy the pre-roll (oldest → newest) into dst. Returns samples copied.
static size_t preroll_copy_out(int16_t *dst, size_t max)
{
    if (!s_preroll_buf || s_preroll_filled == 0) return 0;
    size_t n = s_preroll_filled < max ? s_preroll_filled : max;
    size_t start = (s_preroll_head + PREROLL_SAMPLES - n) % PREROLL_SAMPLES;
    for (size_t i = 0; i < n; ++i) {
        dst[i] = s_preroll_buf[(start + i) % PREROLL_SAMPLES];
    }
    return n;
}

// Fast frame-level "is this speech?" check for follow-up start detection.
// Mirrors the VAD's RMS comparison but stateless — we just need to know
// if THIS frame is above the speech threshold.
static inline bool frame_is_speech(const int16_t *samples, size_t n)
{
    return frame_energy(samples, n) > VOICE_ENERGY_THRESHOLD;
}

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static EventGroupHandle_t s_wifi_evt = NULL;

// Must hold a full VAD-bounded utterance: max_utterance_ms is 15 s, so 16 s
// gives the ceiling plus margin. 12 s (pre-0.2.62) silently truncated the
// longest questions — the append gate below just stops copying when full, so
// STT got a cut-off utterance with no error anywhere. 512 KB, lands in PSRAM.
#define CAPTURE_BUFFER_SAMPLES (16000 * 16)
static int16_t *s_capture_buf = NULL;
static size_t s_capture_used = 0;

#define SENTENCE_MAX 512

typedef struct {
    char text[SENTENCE_MAX];
    // Snapshot the reply turn at enqueue time. A sentence dequeued after a
    // verify ACCEPT must not resurrect audio from the reply we just stopped.
    char turn_id[24];
} sentence_t;

static char s_token_accum[2048];
static size_t s_token_accum_len = 0;
static char s_token_accum_turn[24] = "";
static SemaphoreHandle_t s_token_mutex = NULL;

static int64_t get_awaiting_since_us(void)
{
    portENTER_CRITICAL(&s_time_mux);
    int64_t v = s_awaiting_since_us;
    portEXIT_CRITICAL(&s_time_mux);
    return v;
}

static void set_awaiting_since_us(int64_t v)
{
    portENTER_CRITICAL(&s_time_mux);
    s_awaiting_since_us = v;
    portEXIT_CRITICAL(&s_time_mux);
}

static int64_t get_followup_until_us(void)
{
    portENTER_CRITICAL(&s_time_mux);
    int64_t v = s_followup_until_us;
    portEXIT_CRITICAL(&s_time_mux);
    return v;
}

static void set_followup_until_us(int64_t v)
{
    portENTER_CRITICAL(&s_time_mux);
    s_followup_until_us = v;
    portEXIT_CRITICAL(&s_time_mux);
}

static void set_ui_state(ui_state_t s) { leds_buttons_set_state(s); }

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "wifi disconnected, retrying");
        xEventGroupSetBits(s_wifi_evt, WIFI_FAIL_BIT);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "got IP");
        // Re-assert PS_NONE on every reconnect. The Wi-Fi stack resets the
        // PS mode to MIN_MODEM on disconnect (and CONFIG_ESP_WIFI_STA_
        // DISCONNECTED_PM_ENABLE silently puts the radio in low-power mode
        // while down), so without this the device responds sluggishly to
        // incoming AirPlay TCP / mDNS queries after any transient Wi-Fi
        // blip — looks identical to "device fell asleep".
        esp_wifi_set_ps(WIFI_PS_NONE);
        xEventGroupSetBits(s_wifi_evt, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_sta_start(const char *ssid, const char *password)
{
    s_wifi_evt = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));
    esp_event_handler_instance_t inst_any, inst_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, &inst_any);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, &inst_ip);

    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid, ssid, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, password, sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    // Wake on every DTIM beacon, not every 3rd (the IDF default). Belt-and-
    // suspenders with WIFI_PS_NONE — even if PS ever flips on, we want the
    // shortest possible wake interval so incoming AirPlay TCP / mDNS PTR
    // queries aren't held up.
    wc.sta.listen_interval = 1;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Voice devices are always mains-powered; the default MIN_MODEM
    // power-save lets the radio sleep between DTIM beacons (~100 ms),
    // which delivers UDP audio (AirPlay) and our WS frames in bursts.
    // Disable it once and leave it off — improves wake latency, OE WS
    // round-trip, and AirPlay clock sync alike. esp_wifi_set_ps must
    // run AFTER esp_wifi_start.
    esp_wifi_set_ps(WIFI_PS_NONE);

    EventBits_t b = xEventGroupWaitBits(s_wifi_evt, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
    return (b & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

// Pairing worker — runs in its own task spawned from portal_submit_cb. The
// callback can't do this work inline: it executes inside the captive portal's
// httpd request handler, and the first thing we need to do is stop that
// httpd (and the Wi-Fi AP it lives on). Tearing the handler out from under
// itself causes a LoadProhibited panic in uxListRemove (FreeRTOS list of
// active worker contexts).
static void pair_worker_task(void *arg)
{
    captive_form_result_t *r = (captive_form_result_t *) arg;

    // Give the httpd worker that called us a moment to finish flushing the
    // DONE_HTML response and tear down its socket cleanly. ~500 ms is the
    // same delay the original inline path used.
    vTaskDelay(pdMS_TO_TICKS(500));

    captive_portal_stop();
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_wifi_stop();
    esp_wifi_deinit();

    if (wifi_sta_start(r->ssid, r->password) != ESP_OK) {
        ESP_LOGE(TAG, "STA failed, rebooting to retry");
        free(r);
        esp_restart();
    }

    oe_pair_result_t pr = {0};
    if (oe_pair_redeem(r->server_url, r->pair_code, r->device_name, &pr) == ESP_OK) {
        nvs_creds_set_token(pr.token);
        if (pr.server_hint[0]) nvs_creds_set_server(pr.server_hint);
        if (pr.device_id[0]) {
            esp_err_t e = nvs_creds_set_device_id(pr.device_id);
            if (e != ESP_OK) {
                ESP_LOGW(TAG, "paired device_id NVS write failed: %s",
                         esp_err_to_name(e));
            }
        }
        ESP_LOGI(TAG, "paired — rebooting into operational mode");
        free(r);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    } else {
        ESP_LOGE(TAG, "pair redeem failed");
        set_ui_state(UI_STATE_ERROR);
        free(r);
        vTaskDelete(NULL);
    }
}

static void portal_submit_cb(const captive_form_result_t *r, void *user)
{
    ESP_LOGI(TAG, "portal: ssid=%s server=%s code=%s name=%s",
             r->ssid, r->server_url, r->pair_code, r->device_name);
    nvs_creds_set_wifi(r->ssid, r->password);
    nvs_creds_set_server(r->server_url);
    nvs_creds_set_device_name(r->device_name[0] ? r->device_name : "voice-device");

    // Hand off the heavy work (stop captive portal, switch to STA, redeem)
    // to a separate task so the httpd handler that called us can return
    // cleanly. Doing it inline tears down the httpd from inside its own
    // worker thread → LoadProhibited in uxListRemove.
    captive_form_result_t *copy = (captive_form_result_t *) malloc(sizeof(*copy));
    if (!copy) {
        ESP_LOGE(TAG, "pair_worker malloc failed");
        return;
    }
    *copy = *r;
    if (xTaskCreate(pair_worker_task, "pair_worker", 6144, copy, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "pair_worker task spawn failed");
        free(copy);
    }
}

static void token_lock(void)
{
    if (s_token_mutex) xSemaphoreTake(s_token_mutex, portMAX_DELAY);
}

static void token_unlock(void)
{
    if (s_token_mutex) xSemaphoreGive(s_token_mutex);
}

static void flush_token_to_sentence_queue_locked(void)
{
    if (s_token_accum_len == 0) return;
    if (!s_sentence_q) {
        s_token_accum_len = 0;
        s_token_accum[0] = 0;
        s_token_accum_turn[0] = 0;
        return;
    }
    sentence_t s = {0};
    size_t take = s_token_accum_len < SENTENCE_MAX - 1 ? s_token_accum_len : SENTENCE_MAX - 1;
    memcpy(s.text, s_token_accum, take);
    s.text[take] = 0;
    snprintf(s.turn_id, sizeof(s.turn_id), "%s",
             s_token_accum_turn[0] ? s_token_accum_turn : s_turn_id);
    xQueueSend(s_sentence_q, &s, pdMS_TO_TICKS(100));
    s_token_accum_len = 0;
    s_token_accum[0] = 0;
    s_token_accum_turn[0] = 0;
}

static void flush_token_to_sentence_queue(void)
{
    token_lock();
    flush_token_to_sentence_queue_locked();
    token_unlock();
}

static void reset_token_accum(void)
{
    token_lock();
    s_token_accum_len = 0;
    s_token_accum[0] = 0;
    s_token_accum_turn[0] = 0;
    token_unlock();
}

static bool token_accum_empty(void)
{
    token_lock();
    bool empty = s_token_accum_len == 0;
    token_unlock();
    return empty;
}

static void accumulate_token(const char *tok, size_t len, const char *turn_id)
{
    if (!tok || len == 0) return;
    if (len >= sizeof(s_token_accum)) len = sizeof(s_token_accum) - 1;

    token_lock();
    // Re-check correlation under the same lock used by ACCEPT's accumulator
    // reset. An old event can pass evt_turn_stale(), lose the race to turn
    // promotion, then arrive here; without this check it would be mislabeled
    // as the new turn and spoken later.
    if (turn_id && turn_id[0] &&
        (!s_turn_id[0] || strcmp(turn_id, s_turn_id) != 0)) {
        token_unlock();
        return;
    }
    const char *event_turn = (turn_id && turn_id[0]) ? turn_id : s_turn_id;
    if (s_token_accum_len > 0 && s_token_accum_turn[0] &&
        event_turn[0] && strcmp(s_token_accum_turn, event_turn) != 0) {
        flush_token_to_sentence_queue_locked();
    }
    if (s_token_accum_len == 0) {
        snprintf(s_token_accum_turn, sizeof(s_token_accum_turn), "%s",
                 event_turn);
    }
    if (s_token_accum_len + len + 1 > sizeof(s_token_accum)) {
        flush_token_to_sentence_queue_locked();
        snprintf(s_token_accum_turn, sizeof(s_token_accum_turn), "%s",
                 event_turn);
    }
    memcpy(s_token_accum + s_token_accum_len, tok, len);
    s_token_accum_len += len;
    s_token_accum[s_token_accum_len] = 0;

    for (size_t i = 0; i < s_token_accum_len; ++i) {
        char c = s_token_accum[i];
        if ((c == '.' || c == '!' || c == '?') && i + 1 < s_token_accum_len &&
            (s_token_accum[i + 1] == ' ' || s_token_accum[i + 1] == '\n')) {
            sentence_t s = {0};
            size_t take = i + 1;
            if (take > SENTENCE_MAX - 1) take = SENTENCE_MAX - 1;
            memcpy(s.text, s_token_accum, take);
            s.text[take] = 0;
            snprintf(s.turn_id, sizeof(s.turn_id), "%s",
                     s_token_accum_turn[0] ? s_token_accum_turn : s_turn_id);
            xQueueSend(s_sentence_q, &s, pdMS_TO_TICKS(100));
            size_t rest = s_token_accum_len >= (i + 2) ? s_token_accum_len - (i + 2) : 0;
            memmove(s_token_accum, s_token_accum + i + 2, rest);
            s_token_accum_len = rest;
            s_token_accum[s_token_accum_len] = 0;
            if (rest == 0) s_token_accum_turn[0] = 0;
            i = (size_t)-1;
        }
    }
    token_unlock();
}

// Helper for apply_ww_upload — open/write/close a SPIFFS file in one
// call. Returns false on open failure OR short write so the caller can
// decide whether to force a GC pass and retry.
static bool ww_write_file(const char *path, const void *data, size_t len)
{
    FILE *f = fopen(path, "wb");
    if (!f) { ESP_LOGE(TAG, "ww_upload: open %s failed", path); return false; }
    size_t w = fwrite(data, 1, len, f);
    fclose(f);
    if (w != len) {
        ESP_LOGE(TAG, "ww_upload: short write %s (%u/%u)", path, (unsigned)w, (unsigned)len);
        return false;
    }
    return true;
}

// True iff the file at `path` exists and its bytes exactly equal `data[0..len)`.
// Lets apply_ww_upload skip a needless SPIFFS rewrite + model reload when OE
// re-pushes an identical wake-word (the device-side guard against re-push
// reboot storms). Streamed compare so we never hold a second 62 KB copy.
static bool ww_file_matches(const char *path, const void *data, size_t len)
{
    struct stat st;
    if (stat(path, &st) != 0 || (size_t) st.st_size != len) return false;
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    bool match = true;
    uint8_t buf[512];
    const uint8_t *p = (const uint8_t *) data;
    size_t off = 0;
    while (off < len) {
        size_t want = len - off;
        if (want > sizeof(buf)) want = sizeof(buf);
        size_t got = fread(buf, 1, want, f);
        if (got != want || memcmp(buf, p + off, want) != 0) { match = false; break; }
        off += got;
    }
    fclose(f);
    return match;
}

// Apply an OTA wake-word upload pushed from OE. Writes /ww/slot{N}.tflite
// + /ww/slot{N}.json to SPIFFS, then calls wakeword_load_slot to swap the
// model live without rebooting. Called from ws_event_cb on OE_WS_EVT_WW_UPLOAD.
//
// Sends {type:'ww_upload_ack', slot, ok, err?} at every exit path. The
// server uses acks both to serialize sequential per-slot pushes (so the
// device's WS RX never sees more than one ~85KB ww_upload frame in flight
// at a time — back-to-back bursts overran the recv path) and to gate the
// voice_config_pushed_version bump until every slot has actually landed.
static void apply_ww_upload(const char *json_text, size_t json_len)
{
    int ack_slot = -1;
    cJSON *j = cJSON_ParseWithLength(json_text, json_len);
    if (!j) {
        ESP_LOGE(TAG, "ww_upload: bad JSON");
        oe_ws_send_ww_ack(-1, false, "bad_json");
        return;
    }
    const cJSON *jslot     = cJSON_GetObjectItem(j, "slot");
    const cJSON *jtflite   = cJSON_GetObjectItem(j, "tflite_b64");
    const cJSON *jmanifest = cJSON_GetObjectItem(j, "manifest");
    if (!cJSON_IsNumber(jslot) || !cJSON_IsString(jtflite) || !cJSON_IsString(jmanifest)) {
        ESP_LOGE(TAG, "ww_upload: missing slot/tflite_b64/manifest");
        if (cJSON_IsNumber(jslot)) ack_slot = jslot->valueint;
        cJSON_Delete(j);
        oe_ws_send_ww_ack(ack_slot, false, "bad_fields");
        return;
    }
    int slot = jslot->valueint;
    ack_slot = slot;
    if (slot < 0 || slot >= WW_NUM_SLOTS) {
        ESP_LOGE(TAG, "ww_upload: slot %d out of range [0,%d)", slot, WW_NUM_SLOTS);
        cJSON_Delete(j);
        oe_ws_send_ww_ack(ack_slot, false, "slot_oor");
        return;
    }

    // Decode base64 → tflite bytes. oe_b64_decoded_len gives an upper bound
    // (it doesn't account for padding), so the actual length is whatever
    // oe_b64_decode returns. The 256 KB cap matches load_model_file in
    // wakeword.cpp — if a bigger payload arrives we reject before alloc.
    const char *b64 = jtflite->valuestring;
    size_t b64_len  = strlen(b64);
    size_t max_bin  = oe_b64_decoded_len(b64, b64_len);
    if (max_bin == 0 || max_bin > 256 * 1024) {
        ESP_LOGE(TAG, "ww_upload: tflite size %u out of range", (unsigned)max_bin);
        cJSON_Delete(j);
        oe_ws_send_ww_ack(ack_slot, false, "size_oor");
        return;
    }
    uint8_t *tflite = heap_caps_malloc(max_bin, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!tflite) tflite = heap_caps_malloc(max_bin, MALLOC_CAP_8BIT);
    if (!tflite) {
        ESP_LOGE(TAG, "ww_upload: tflite alloc failed");
        cJSON_Delete(j);
        oe_ws_send_ww_ack(ack_slot, false, "alloc");
        return;
    }
    size_t bin_len = oe_b64_decode(b64, b64_len, tflite, max_bin);
    if (bin_len == 0) {
        ESP_LOGE(TAG, "ww_upload: base64 decode failed");
        heap_caps_free(tflite);
        cJSON_Delete(j);
        oe_ws_send_ww_ack(ack_slot, false, "b64");
        return;
    }

    // Serialize manifest. The server sends it as a string field that's
    // already been JSON-stringified; we write that text verbatim so the
    // file matches what's bundled at build time and the same parser reads.
    const char *manifest_text = jmanifest->valuestring;
    size_t manifest_len = strlen(manifest_text);

    char tflite_path[64], manifest_path[64];
    snprintf(tflite_path, sizeof(tflite_path),   "/ww/slot%d.tflite", slot);
    snprintf(manifest_path, sizeof(manifest_path), "/ww/slot%d.json",   slot);

    // Skip an identical re-push. OE re-sends the saved config "in case NVS was
    // reset" on every reconnect; on a flapping link that became an all-night
    // reboot loop, because each push runs GC + a ~62 KB rewrite + a
    // wakeword_load_slot rebuild (which esp_restart()s if the reload fails). If
    // the bytes already on flash match exactly, the live model is already
    // correct — ack and return without touching SPIFFS or the model. (The
    // server-side re-push debounce in ws-handler.mjs is the other half.)
    if (ww_file_matches(tflite_path, tflite, bin_len) &&
        ww_file_matches(manifest_path, manifest_text, manifest_len)) {
        ESP_LOGI(TAG, "ww_upload: slot %d unchanged (%u bytes) — skip rewrite/reload", slot, (unsigned)bin_len);
        heap_caps_free(tflite);
        cJSON_Delete(j);
        oe_ws_send_ww_ack(ack_slot, true, NULL);
        return;
    }

    // Write order: tflite first, manifest second. If we crash between writes
    // the model file matches the slot index but the manifest is stale —
    // wakeword_load_slot will use the OLD probability_cutoff with the NEW
    // model, which usually still works (cutoffs are similar across v2 models).
    // The reverse order would mean a manifest pointing at a stale .tflite,
    // which is identical to the pre-upload state. Neither is great; the
    // chosen order biases toward the upload taking effect.
    // SPIFFS quirk: overwriting an existing file doesn't immediately free
    // the old pages — they're marked deleted but stay reserved until GC
    // runs. unlink() alone isn't enough: SPIFFS GC is opportunistic, so a
    // back-to-back rewrite can still fwrite() 0 bytes. Force GC with
    // headroom for both the tflite and the manifest, then write. If the
    // first write still short-writes, run GC again with a larger budget
    // and retry once. CONFIG_SPIFFS_GC_MAX_RUNS=32 in sdkconfig.defaults
    // ensures each GC pass can reclaim enough to land a ~62 KB tflite.
    unlink(tflite_path);
    unlink(manifest_path);
    esp_spiffs_gc("wakewords", bin_len + manifest_len + 4096);

    bool ok = true;
    if (!ww_write_file(tflite_path, tflite, bin_len)) {
        ESP_LOGW(TAG, "ww_upload: tflite short-write — forcing GC and retrying");
        esp_spiffs_gc("wakewords", bin_len * 2);
        if (!ww_write_file(tflite_path, tflite, bin_len)) {
            ESP_LOGE(TAG, "ww_upload: tflite short write after GC retry (%u bytes)", (unsigned)bin_len);
            ok = false;
        }
    }
    heap_caps_free(tflite);

    if (ok && !ww_write_file(manifest_path, manifest_text, manifest_len)) {
        ESP_LOGW(TAG, "ww_upload: manifest short-write — forcing GC and retrying");
        esp_spiffs_gc("wakewords", manifest_len + 4096);
        if (!ww_write_file(manifest_path, manifest_text, manifest_len)) {
            ESP_LOGE(TAG, "ww_upload: manifest short write after GC retry");
            ok = false;
        }
    }

    cJSON_Delete(j);
    if (!ok) {
        oe_ws_send_ww_ack(ack_slot, false, "spiffs");
        return;
    }

    // Hot-reload the slot — wakeword_load_slot already handles cleanup of
    // the previous model and re-reads the manifest. Skip if the slot's
    // wakeword_t never created (e.g. boot-time alloc failure). Files
    // landed on disk in this case, so the next reboot will pick them up;
    // we still ack ok=true to let the server mark this slot complete.
    if (!s_ww[slot]) {
        // No live detector to hot-swap (boot-time alloc failure). Files are on
        // flash; reboot so the next boot loads them instead of leaving the slot
        // running nothing until a manual power-cycle.
        ESP_LOGW(TAG, "ww_upload: slot %d has no wakeword_t — rebooting to pick up", slot);
        oe_ws_send_ww_ack(ack_slot, true, NULL);
        vTaskDelay(pdMS_TO_TICKS(800));
        esp_restart();
    }
    esp_err_t e = wakeword_load_slot(s_ww[slot], slot);
    if (e != ESP_OK) {
        // Hot-swap failed but the .tflite + manifest are already on flash, so a
        // clean boot WILL load them. Reboot rather than limp along on a stale /
        // half-torn-down model (the cause of "had to power-cycle to get the new
        // wake word to work"). Ack ok=true since the reboot completes the swap.
        ESP_LOGE(TAG, "ww_upload: slot %d reload failed: %s — rebooting to load from flash", slot, esp_err_to_name(e));
        oe_ws_send_ww_ack(ack_slot, true, NULL);
        vTaskDelay(pdMS_TO_TICKS(800));  // let the ack WS frame flush first
        esp_restart();
    } else {
        ESP_LOGI(TAG, "ww_upload: slot %d hot-swapped (%u bytes)", slot, (unsigned)bin_len);
        // Refresh the playback-aware cutoff task's per-slot default from the
        // just-loaded manifest. Without this its boot-time value would clobber
        // this freshly-pushed cutoff on the next TTS/AirPlay edge (the "cutoff
        // change only sticks after a reboot" bug).
        s_default_cutoff[slot] = wakeword_get_default_cutoff(s_ww[slot]);
        oe_ws_send_ww_ack(ack_slot, true, NULL);
    }
}

// Clear a wake-word slot. The server sends { type:'ww_clear', slot:N } for
// every slot index that has no assignment in the user's voice-config — e.g.
// after a user is removed and the remaining users repack into lower slots,
// leaving the old tail slot orphaned. We delete /ww/slot{N}.{tflite,json}
// from SPIFFS (so a reboot doesn't reload it) and unload the live detector
// (so it stops firing immediately, no reboot needed). The wakeword_t is kept
// so the slot can be reused later via a plain ww_upload.
//
// Acks via oe_ws_send_ww_ack — the server keys its per-slot pending-ack on
// {deviceId, slot} the same way it does for ww_upload, so a clear participates
// in the same sequential throttle. Idempotent: clearing an already-empty slot
// (no files, no live model) still acks ok=true.
static void apply_ww_clear(const char *json_text, size_t json_len)
{
    int ack_slot = -1;
    cJSON *j = cJSON_ParseWithLength(json_text, json_len);
    if (!j) {
        ESP_LOGE(TAG, "ww_clear: bad JSON");
        oe_ws_send_ww_ack(-1, false, "bad_json");
        return;
    }
    const cJSON *jslot = cJSON_GetObjectItem(j, "slot");
    if (!cJSON_IsNumber(jslot)) {
        ESP_LOGE(TAG, "ww_clear: missing slot");
        cJSON_Delete(j);
        oe_ws_send_ww_ack(-1, false, "bad_fields");
        return;
    }
    int slot = jslot->valueint;
    ack_slot = slot;
    cJSON_Delete(j);
    if (slot < 0 || slot >= WW_NUM_SLOTS) {
        ESP_LOGE(TAG, "ww_clear: slot %d out of range [0,%d)", slot, WW_NUM_SLOTS);
        oe_ws_send_ww_ack(ack_slot, false, "slot_oor");
        return;
    }

    // Unload the live detector first so the audio task stops feeding it the
    // moment the files go away. Safe when the slot was never loaded (NULL).
    if (s_ww[slot]) wakeword_unload_slot(s_ww[slot]);

    char tflite_path[64], manifest_path[64];
    snprintf(tflite_path, sizeof(tflite_path),   "/ww/slot%d.tflite", slot);
    snprintf(manifest_path, sizeof(manifest_path), "/ww/slot%d.json",   slot);
    // unlink returns -1/ENOENT when the file's already gone — that's the
    // idempotent no-op case, not an error. Force a GC pass so the freed
    // pages are reclaimed promptly for the next upload rather than lingering.
    unlink(tflite_path);
    unlink(manifest_path);
    esp_spiffs_gc("wakewords", 4096);

    ESP_LOGI(TAG, "ww_clear: slot %d cleared", slot);
    oe_ws_send_ww_ack(ack_slot, true, NULL);
}

// Forward declarations — alarm_arm_worker + alarm_arm_req_t are defined
// below near boot_operational so they can call alarm.h functions cleanly;
// ws_event_cb uses them and needs the types in scope here.
typedef struct {
    char id[64];
    char label[64];
    char type[16];
    char marker[64];
    int64_t trigger_at_ms;
} alarm_arm_req_t;
static void alarm_arm_worker(void *arg);

typedef struct { char marker[64]; } chime_upload_req_t;
static void chime_upload_worker(void *arg);

// Looped ambient playback (e.g. thunderstorm.mp3 from a "goodnight" routine).
// A SINGLE persistent ambient task (created at boot) owns audio_io for ambient;
// play_ambient posts an ambient_req_t to its queue and sets s_ambient_stop to
// interrupt any current playback. Serializing through one task means two
// ambient fetchers can never race over s_ambient_active / s_ambient_stop /
// audio_io — that race was the barge-in→restore "ambient went silent after a
// command" bug. Wake-word fire also sets the stop flag so the wake word cuts
// the ambient like Alexa does.
typedef struct {
    char marker[64];
    bool loop;
    int  volume;          // -1 = leave volume alone
} ambient_req_t;
static void ambient_task(void *arg);
static void ambient_resume(void);   // un-pause ambient audio once a turn is fully over
static bool ambient_resume_locked(bool gate_rollback_owner);
static void heartbeat_task(void *arg);
static volatile bool s_ambient_active = false;
static volatile bool s_ambient_stop   = false;
// Pause (wake/barge-in) vs stop (teardown). Pause keeps the HTTP stream open and
// reading but discards the audio so the speaker is free for the command/reply;
// resume just plays the live stream again. No reconnect = no "ambient went
// silent after a command" failure. s_ambient_stop is reserved for real teardown
// (stop_ambient, switching markers, mute).
static volatile bool s_ambient_paused = false;
static int           s_pre_ambient_volume = -1;
static QueueHandle_t s_ambient_req_q = NULL;  // requests for the single ambient task
static char          s_ambient_cur_marker[64] = {0};  // marker of the live session ("" = none)
// Sample-rate handling for the ambient stream. Two layers:
//
//   s_last_ambient_rate (seed = 44100): cached rate used when libhelix hands
//   up a buffer with rate=0 (post-error recovery frame).
//
//   s_ambient_stable_rate: locked once we see the first non-zero rate on the
//   stream. After that, every buffer plays at the locked rate, even if a
//   later misparse reports something different. mp3 doesn't change rate
//   mid-stream in practice, and the alternative — trusting the decoder's
//   per-frame report — lets corrupt-header misparses (mp3 err -6/-2) leak
//   fake rates like 22050/32000 into audio_io, producing garbage PCM at
//   the wrong pitch.
//
// Both reset at the start of each ambient_worker_task so a new file can
// have a different rate.
static uint32_t      s_last_ambient_rate = 44100;
static uint32_t      s_ambient_stable_rate = 0;

static void ws_event_cb(const oe_ws_payload_t *evt, void *user)
{
    switch (evt->type) {
        case OE_WS_EVT_CONNECTED:
            // Treat CONNECTED→SERVER_CAPS as an untrusted compatibility
            // window. A reconnect to an older server must not inherit the
            // previous socket's capabilities for even one wake frame.
            s_caps_tts_pause = false;
            s_caps_stt_stream = false;
            s_caps_turn_ids = false;
            s_caps_tts_hold_ids = false;
            s_authenticated_caps_seen = false;
            s_ws_connected = true;
            __sync_add_and_fetch(&s_ws_connection_epoch, 1);
            // Bring AirPlay 1 receiver online now that we have Wi-Fi + a
            // paired account. airplay_init is internally idempotent so
            // reconnects are safe. Service name uses the configured device
            // name so multiple devices show distinct entries in iOS.
            airplay_init(g_dev_config.device_name[0] ? g_dev_config.device_name : "OE Voice");
            // Re-report any alarm still ringing. oe_ws.c sends {type:'auth'}
            // before this event fires, so the session is established first.
            alarm_resend_fired();
            break;
        case OE_WS_EVT_CHAT_TOKEN: {
            if (!vg_lifecycle_take()) break;
            if (s_drop_untagged_tts &&
                evt->turn_id && evt->turn_id[0] &&
                !evt_turn_stale(evt)) {
                s_drop_untagged_tts = false;
            }
            if (!evt_turn_stale(evt) &&
                !evt_untagged_conflicts_with_busy_turn(evt) &&
                !s_speech_writes_blocked) {
                accumulate_token(evt->text, evt->text_len, evt->turn_id);
            }
            vg_lifecycle_give();
            break;
        }
        case OE_WS_EVT_CHAT_DONE:
            if (!vg_lifecycle_take()) break;
            if (s_drop_untagged_tts &&
                (!evt->turn_id || !evt->turn_id[0])) {
                s_drop_untagged_tts = false;
                vg_lifecycle_give();
                break;
            }
            if (evt_turn_stale(evt) ||
                evt_untagged_conflicts_with_busy_turn(evt) ||
                s_speech_writes_blocked) {
                vg_lifecycle_give();
                break;
            }
            // A `done` while the user is mid-command is NOT ours to act on —
            // it's the tail of an aborted turn (stop-ack, superseded streamer)
            // racing the new turn. Acting on it used to IDLE the UI and
            // airplay_resume() over the user's speech (repro: wake during
            // AirPlay → barge-in pauses music + sends stop → server acks with
            // a bare done → music resumed mid-dictation).
            if (s_in_utterance && !s_verify_playback_hold_active) {
                vg_lifecycle_give();
                break;
            }
            s_wait_led_until_us = 0;
            flush_token_to_sentence_queue();
            // If the reply produced no audio (e.g. server-side voice-intent
            // router short-circuited a "volume up" / "pause" without
            // invoking the LLM), there's nothing for tts_worker_task to
            // play and it will sit on xQueueReceive forever — leaving the
            // UI stuck in THINKING. Drop to IDLE here when the queue is
            // empty so the device looks responsive after a control intent.
            if (uxQueueMessagesWaiting(s_sentence_q) == 0 && token_accum_empty()) {
                // No audio coming — re-open the wake-word feed now rather
                // than waiting on a SPEAKING transition that will never
                // happen. Keeps the device responsive after a voice-intent
                // short-circuit or empty/error reply.
                s_awaiting_reply = false;
                set_awaiting_since_us(0);
                // A follow-up window deferred behind this (audio-less) reply
                // arms now — there is no drain event coming to arm it.
                if (s_followup_pending_ms > 0) {
                    set_followup_until_us(esp_timer_get_time() +
                                          (int64_t)s_followup_pending_ms * 1000);
                    s_followup_pending_ms = 0;
                    preroll_reset();
                    set_ui_state(UI_STATE_LISTENING);
                } else {
                    set_ui_state(UI_STATE_IDLE);
                }
                airplay_resume();
            }
            vg_lifecycle_give();
            break;
        case OE_WS_EVT_TTS_AUDIO_BEGIN:
            // Server-side streaming: about to receive synthesized PCM frames.
            // Enter SPEAKING once; the legacy per-sentence path isn't used.
            // Stale-turn begin (aborted reply racing a barge-in) must not
            // re-enter SPEAKING mid-capture of the new turn.
            if (!vg_lifecycle_take()) break;
            if (s_drop_untagged_tts &&
                evt->turn_id && evt->turn_id[0] &&
                !evt_turn_stale(evt)) {
                s_drop_untagged_tts = false;
            }
            if (evt_turn_stale(evt) ||
                evt_untagged_conflicts_with_busy_turn(evt) ||
                s_speech_writes_blocked) {
                vg_lifecycle_give();
                break;
            }
            if (!leds_buttons_is_muted() && !s_stream_active) {
                if (s_verify_playback_hold_active &&
                    (!evt->turn_id || !evt->turn_id[0])) {
                    s_verify_hold_untagged_stream_owned = true;
                }
                s_stream_active  = true;
                s_stream_end_req = false;
                snprintf(s_stream_turn_id, sizeof(s_stream_turn_id), "%s",
                         (evt->turn_id && evt->turn_id[0])
                             ? evt->turn_id : "");
                s_awaiting_reply = false;
                s_last_tts_frame_us = esp_timer_get_time();
                s_wait_led_until_us = 0;
                set_ui_state(UI_STATE_SPEAKING);
                for (uint8_t _i = 0; _i < WW_NUM_SLOTS; ++_i)
                    if (s_ww[_i]) wakeword_notify_speaking_began(s_ww[_i]);
                // A provisional wake owns an independent local playback hold.
                // Preserve the old stream state and buffer its tagged frames,
                // but do not re-enable the amp while verification is listening.
                if (!s_verify_playback_hold_active) {
                    xvf3800_enable_amplifier(true);
                }
                audio_io_start_playback();
            }
            vg_lifecycle_give();
            break;
        case OE_WS_EVT_TTS_AUDIO: {
            // One base64 PCM frame → write straight to I²S. Payload contract
            // with lib/voice-tts-stream.mjs: 16 kHz STEREO interleaved s16le
            // (CHANNELS=2, ffmpeg -ac 2), so the frame goes to write_pcm
            // as-is. Do NOT re-wrap it in a mono→stereo expander: 0.2.60 did
            // and it (a) double-expanded the already-stereo frames (half-
            // speed audio) and (b) blew the 4 KB websocket_task stack — the
            // panic-on-every-reply bug fixed in 0.2.61.
            bool admitted = false;
            if (!vg_lifecycle_take()) break;
            if (!evt_turn_stale(evt) &&
                !evt_untagged_conflicts_with_busy_turn(evt) &&
                !s_speech_writes_blocked &&
                evt_matches_active_stream(evt) &&
                evt->text && evt->text_len) {
                __sync_add_and_fetch(&s_speech_writers, 1);
                admitted = true;
            }
            vg_lifecycle_give();
            if (admitted) {
                s_last_tts_frame_us = esp_timer_get_time();
                size_t olen = 0;
                if (mbedtls_base64_decode(s_pcm_frame, sizeof(s_pcm_frame), &olen,
                        (const unsigned char *)evt->text, evt->text_len) == 0 && olen >= 2) {
                    // SPEECH lane: mixes OVER (ducks) any ambient/AirPlay bed
                    // instead of fighting it for one ring.
                    audio_io_write_speech_pcm((const int16_t *)s_pcm_frame, olen / 2, 16000);
                }
                __sync_sub_and_fetch(&s_speech_writers, 1);
            }
            break;
        }
        case OE_WS_EVT_TTS_AUDIO_END: {
            // All audio sent — finalize task drains the ring, then idles.
            if (!vg_lifecycle_take()) break;
            if (s_drop_untagged_tts &&
                (!evt->turn_id || !evt->turn_id[0])) {
                s_drop_untagged_tts = false;
                vg_lifecycle_give();
                break;
            }
            if (evt_turn_stale(evt) ||
                evt_untagged_conflicts_with_busy_turn(evt) ||
                s_speech_writes_blocked ||
                !evt_matches_active_stream(evt)) {
                vg_lifecycle_give();
                break;
            }
            if (s_stream_active) {
                bool pending = false;
                if (evt->text && evt->text_len) {
                    cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
                    if (j) {
                        pending = cJSON_IsTrue(cJSON_GetObjectItem(j, "pending"));
                        cJSON_Delete(j);
                    }
                }
                s_stream_end_pending = pending;
                s_stream_end_req = true;
            }
            vg_lifecycle_give();
            break;
        }
        case OE_WS_EVT_DUPLICATE_SUPPRESSED:
            // Server suppressed the chat as a duplicate — no reply is coming.
            // Clearing the THINKING gate here used to be missing, leaving the
            // wake feed closed for the full 90 s watchdog.
            if (s_awaiting_reply) {
                s_awaiting_reply = false;
                set_awaiting_since_us(0);
                set_ui_state(UI_STATE_IDLE);
                airplay_resume();
            }
            break;
        case OE_WS_EVT_ERROR:
            // Server-side turn error (or transport error). If a reply was
            // pending, it is not coming — re-open the wake feed instead of
            // sitting deaf until the 90 s watchdog. The streaming path
            // usually converts errors to spoken fallback + done server-side;
            // this handles the bare-error paths (validation, caps, shutdown).
            if (!vg_lifecycle_take()) break;
            if (evt_turn_stale(evt) ||
                evt_untagged_conflicts_with_busy_turn(evt) ||
                s_speech_writes_blocked) {
                vg_lifecycle_give();
                break;
            }
            if (s_awaiting_reply && !s_stream_active) {
                ESP_LOGW(TAG, "server error while awaiting reply%s%.*s — back to IDLE",
                         evt->text ? ": " : "",
                         evt->text ? (int)(evt->text_len < 96 ? evt->text_len : 96) : 0,
                         evt->text ? evt->text : "");
                s_awaiting_reply = false;
                set_awaiting_since_us(0);
                s_followup_pending_ms = 0;
                set_ui_state(UI_STATE_IDLE);
                airplay_resume();
            }
            vg_lifecycle_give();
            break;
        case OE_WS_EVT_SERVER_CAPS: {
            // Capability flags plus production verify-gate provisioning. The
            // URL field is authoritative even when empty; omitted fields
            // preserve state for compatibility with older servers.
            cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
            if (j) {
                s_caps_tts_pause  = cJSON_IsTrue(cJSON_GetObjectItem(j, "tts_pause"));
                s_caps_stt_stream = cJSON_IsTrue(cJSON_GetObjectItem(j, "stt_stream"));
                s_caps_turn_ids   = cJSON_IsTrue(cJSON_GetObjectItem(j, "turn_ids"));
                s_caps_tts_hold_ids =
                    cJSON_IsTrue(cJSON_GetObjectItem(j, "tts_hold_ids"));
                apply_verify_gate_server_caps(j);
                ESP_LOGI(TAG, "server caps: turn_ids=%d tts_pause=%d tts_hold_ids=%d stt_stream=%d",
                         (int)s_caps_turn_ids, (int)s_caps_tts_pause,
                         (int)s_caps_tts_hold_ids,
                         (int)s_caps_stt_stream);
                cJSON_Delete(j);
                s_authenticated_caps_seen = true;
                maybe_resume_pending_ota();
            }
            break;
        }
        case OE_WS_EVT_SET_CONVERSATION_MODE: {
            cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
            if (j) {
                cJSON *je = cJSON_GetObjectItem(j, "enabled");
                bool en = cJSON_IsTrue(je) || (cJSON_IsNumber(je) && je->valueint != 0);
                if (en && !server_control_may_open_ungated_capture()) {
                    ESP_LOGW(TAG,
                             "conversation mode enable ignored: secure gate control unavailable");
                    en = false;
                }
                if (en != s_conversation_mode) {
                    s_conversation_mode = en;
                    ESP_LOGI(TAG, "conversation mode: %s", en ? "on" : "off");
                }
                cJSON_Delete(j);
            }
            break;
        }
        case OE_WS_EVT_UI_WAIT: {
            // { type:'ui_wait', on } — background work is running for this
            // device outside any turn (e.g. a delegated task after the ack
            // reply finished; the result arrives later as an announcement).
            // LED-only, exactly like the burst-close WAITING state: the mic
            // stays fully open and an active turn's UI always wins — only
            // flip the ring when the device is otherwise idle. The server
            // re-asserts every ~10s while work is pending, so a wake/reply
            // that clears this locally gets the spinner back afterwards.
            cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
            if (j) {
                bool on = cJSON_IsTrue(cJSON_GetObjectItem(j, "on"));
                cJSON_Delete(j);
                bool turn_busy = s_stream_active || s_awaiting_reply ||
                                 s_in_utterance || get_followup_until_us() != 0;
                if (on) {
                    s_wait_led_until_us = esp_timer_get_time() + WAIT_LED_TIMEOUT_US;
                    if (!turn_busy) set_ui_state(UI_STATE_WAITING);
                } else if (s_wait_led_until_us != 0) {
                    s_wait_led_until_us = 0;
                    if (!turn_busy) set_ui_state(UI_STATE_IDLE);
                }
            }
            break;
        }
        case OE_WS_EVT_DISCONNECTED:
            s_ws_connected = false;
            s_authenticated_caps_seen = false;
            __sync_add_and_fetch(&s_ws_connection_epoch, 1);
            s_drop_untagged_tts = false;
            // Turn ownership is socket-scoped. Never carry an old id onto a
            // replacement connection, where a retry/next wake could target a
            // chat this socket never owned.
            s_turn_id[0] = 0;
            // The gate cannot safely accept after losing the control socket.
            // The capture task will roll back its gate-owned hold on the next
            // frame; keeping cleanup there preserves single-owner ordering.
            vg_request_cancel();
            s_wait_led_until_us = 0;
            s_stream_end_pending = false;
            // Forget capabilities — the socket may reconnect to an older
            // server (rollback) that doesn't understand the newer messages.
            s_caps_tts_pause  = false;
            s_caps_stt_stream = false;
            s_caps_turn_ids   = false;
            s_caps_tts_hold_ids = false;
            // A barge verify in flight loses its server: release OUR pause on
            // the playback engine so the stream teardown below fully cleans up.
            if (s_paused_for_barge) {
                s_paused_for_barge = false;
                s_barge_state = BARGE_NONE;
                audio_io_resume_playback();
            }
            // If a chat reply was pending when the socket died it will never
            // arrive — clear the THINKING gate now so the wake feed re-opens
            // instead of the device staying deaf until reboot.
            if (s_awaiting_reply) {
                s_awaiting_reply = false;
                set_awaiting_since_us(0);
                set_ui_state(UI_STATE_IDLE);
            }
            s_followup_pending_ms = 0;
            if (s_stream_active || s_stream_end_req) {
                for (uint8_t _i = 0; _i < WW_NUM_SLOTS; ++_i)
                    if (s_ww[_i]) wakeword_notify_speaking_ended(s_ww[_i]);
                s_stream_active = false;
                s_stream_end_req = false;
                s_stream_turn_id[0] = 0;
                // The dead socket owns only its speech lane. A global engine
                // stop/flush here used to erase retained ambient/AirPlay/alarm
                // PCM, including audio behind a provisional verify hold.
                audio_io_flush_speech();
                const bool other_audio =
                    s_legacy_tts_active ||
                    (s_ambient_active && !s_ambient_paused) ||
                    airplay_is_playing() || alarm_is_firing();
                if (!other_audio || s_verify_playback_hold_active) {
                    xvf3800_enable_amplifier(false);
                    airplay_note_amp_forced_off();
                }
                if (!alarm_is_firing()) set_ui_state(UI_STATE_IDLE);
            }
            airplay_resume();
            break;
        case OE_WS_EVT_WW_UPLOAD:
            apply_ww_upload(evt->text, evt->text_len);
            break;
        case OE_WS_EVT_WW_CLEAR:
            apply_ww_clear(evt->text, evt->text_len);
            break;
        case OE_WS_EVT_SET_VOLUME: {
            // Payload is the raw set_volume JSON. Accept absolute `pct`
            // or relative `delta`; clamp to 0-100; persist via NVS.
            cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
            if (j) {
                int target = audio_io_get_volume();
                cJSON *jp = cJSON_GetObjectItem(j, "pct");
                cJSON *jd = cJSON_GetObjectItem(j, "delta");
                if (cJSON_IsNumber(jp)) target = jp->valueint;
                else if (cJSON_IsNumber(jd)) target += jd->valueint;
                if (target < 0)   target = 0;
                if (target > 100) target = 100;
                audio_io_set_volume((uint8_t) target);
                nvs_creds_set_volume((uint8_t) target);
                // Invalidate the ambient-restore baseline: user-initiated
                // volume changes must stick even when ambient stops, otherwise
                // ambient_worker_task's cleanup at line ~1431 silently reverts
                // to the pre-ambient value and overwrites the user's intent.
                s_pre_ambient_volume = -1;
                ESP_LOGI(TAG, "set_volume: %d%%", target);
                cJSON_Delete(j);
            }
            break;
        }
        case OE_WS_EVT_PAUSE_PLAYBACK:
            ESP_LOGI(TAG, "pause_playback");
            // DACP pause to iOS first — that's what actually stops the
            // music. The local audio_io stall is the safety net for when
            // pause hits while the ringbuffer still has queued PCM that
            // shouldn't drain past this point.
            airplay_user_pause();
            audio_io_pause_playback();
            break;
        case OE_WS_EVT_RESUME_PLAYBACK:
            ESP_LOGI(TAG, "resume_playback");
            audio_io_resume_playback();
            airplay_user_resume();
            break;
        case OE_WS_EVT_ALARM_ARM: {
            // { type:'alarm_arm', id, label, triggerAtMs, alarmType, audioMarker }
            // Marker resolves to the server's pre-synthesized announcement MP3
            // (oneShotMp3Cache, 60 s TTL). Fetch + arm happens on a worker
            // task — the WS task can't block on HTTP.
            cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
            if (j) {
                const cJSON *jid     = cJSON_GetObjectItem(j, "id");
                const cJSON *jlabel  = cJSON_GetObjectItem(j, "label");
                const cJSON *jts     = cJSON_GetObjectItem(j, "triggerAtMs");
                const cJSON *jtype   = cJSON_GetObjectItem(j, "alarmType");
                const cJSON *jmarker = cJSON_GetObjectItem(j, "audioMarker");
                if (cJSON_IsString(jid) && cJSON_IsString(jlabel) && cJSON_IsNumber(jts)) {
                    alarm_arm_req_t *req = calloc(1, sizeof(*req));
                    if (req) {
                        strncpy(req->id, jid->valuestring, sizeof(req->id) - 1);
                        strncpy(req->label, jlabel->valuestring, sizeof(req->label) - 1);
                        strncpy(req->type,
                                cJSON_IsString(jtype) ? jtype->valuestring : "timer",
                                sizeof(req->type) - 1);
                        if (cJSON_IsString(jmarker)) {
                            strncpy(req->marker, jmarker->valuestring, sizeof(req->marker) - 1);
                        }
                        req->trigger_at_ms = (int64_t) jts->valuedouble;
                        xTaskCreate(alarm_arm_worker, "alarm_arm_w", 4096, req, 4, NULL);
                    }
                }
                cJSON_Delete(j);
            }
            break;
        }
        case OE_WS_EVT_ALARM_DISARM: {
            cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
            if (j) {
                const cJSON *jid = cJSON_GetObjectItem(j, "id");
                if (cJSON_IsString(jid)) alarm_disarm(jid->valuestring);
                cJSON_Delete(j);
            }
            break;
        }
        case OE_WS_EVT_AWAIT_FOLLOWUP: {
            // { type:'await_followup', windowMs } — open a brief listening
            // window AFTER TTS drains. If the reply audio is still streaming/
            // playing, DEFER the countdown to stream_finalize_task (drain
            // complete) via s_followup_pending_ms — arming it now would let the
            // window expire during the spoken reply. If nothing is playing (a
            // short reply that already drained), arm immediately.
            cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
            if (j) {
                if (!server_control_may_open_ungated_capture()) {
                    ESP_LOGW(TAG,
                             "follow-up ignored: secure gate control unavailable");
                    cJSON_Delete(j);
                    break;
                }
                if (!vg_lifecycle_take()) {
                    cJSON_Delete(j);
                    break;
                }
                // Raw-JSON message — turn check happens here, not in oe_ws.c.
                // A follow-up window for an aborted/prior turn must not open.
                const cJSON *jturn = cJSON_GetObjectItem(j, "turn_id");
                if (cJSON_IsString(jturn) && jturn->valuestring[0] &&
                    (!s_turn_id[0] || strcmp(jturn->valuestring, s_turn_id) != 0)) {
                    vg_lifecycle_give();
                    cJSON_Delete(j);
                    break;
                }
                const cJSON *jw = cJSON_GetObjectItem(j, "windowMs");
                int window_ms =
                    (cJSON_IsNumber(jw) && jw->valueint > 0)
                        ? jw->valueint
                        : FOLLOWUP_WINDOW_DEFAULT_MS;
                if (window_ms > FOLLOWUP_WINDOW_MAX_MS) {
                    ESP_LOGW(TAG, "follow-up window clamped from %d to %d ms",
                             window_ms, FOLLOWUP_WINDOW_MAX_MS);
                    window_ms = FOLLOWUP_WINDOW_MAX_MS;
                }
                // Lock the slot of the turn that opened this follow-up so a
                // false-fire on a different wake-word can't reroute the
                // answer to a different user.
                // During provisional verification s_active_slot already names
                // the candidate wake, while this tagged event still belongs to
                // the old published reply. Preserve that reply's owner.
                s_followup_slot =
                    (s_verify_playback_hold_active &&
                     s_prov_identity_valid)
                        ? s_prov_prev_slot : s_active_slot;
                // Defer whenever ANY part of the reply is still in flight —
                // not just when PCM is already streaming. The old
                // s_stream_active-only check armed the window immediately for
                // short replies (synth hadn't produced the first frame yet)
                // and for the legacy token/done path (sentences still queued),
                // so the window burned down DURING the spoken reply and could
                // expire before the user was even asked the question.
                bool reply_in_flight = s_stream_active || s_awaiting_reply ||
                                       uxQueueMessagesWaiting(s_sentence_q) > 0 ||
                                       !token_accum_empty();
                if (reply_in_flight) {
                    s_followup_pending_ms = window_ms;
                    ESP_LOGI(TAG, "follow-up window pending: %d ms (starts when reply drains), slot=%u",
                             window_ms, (unsigned) s_followup_slot);
                } else {
                    set_followup_until_us(esp_timer_get_time() + (int64_t)window_ms * 1000);
                    preroll_reset();
                    set_ui_state(UI_STATE_LISTENING);
                    ESP_LOGI(TAG, "follow-up window armed: %d ms, slot=%u",
                             window_ms, (unsigned) s_followup_slot);
                }
                vg_lifecycle_give();
                cJSON_Delete(j);
            }
            break;
        }
        case OE_WS_EVT_CHIME_UPLOAD: {
            // { type:'chime_upload', audioMarker } — fetch MP3 via marker
            // (one-shot, 60s TTL on server) and install as the custom
            // chime. Empty/missing audioMarker means revert to the built-in
            // procedural chime (server's DELETE /api/voice-chime path).
            // Fetch + install spawned as a task so the WS loop doesn't
            // block on HTTP.
            cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
            if (j) {
                const cJSON *jm = cJSON_GetObjectItem(j, "audioMarker");
                if (cJSON_IsString(jm) && jm->valuestring[0]) {
                    chime_upload_req_t *req = calloc(1, sizeof(*req));
                    if (req) {
                        strncpy(req->marker, jm->valuestring, sizeof(req->marker) - 1);
                        xTaskCreate(chime_upload_worker, "chime_up", 4096, req, 4, NULL);
                    }
                } else {
                    alarm_set_custom_chime(NULL, 0);
                }
                cJSON_Delete(j);
            }
            break;
        }
        case OE_WS_EVT_ALARM_STOP: {
            // { type:'alarm_stop', ids:null|[...] } — null kills every firing alarm
            cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
            if (j) {
                const cJSON *jids = cJSON_GetObjectItem(j, "ids");
                if (cJSON_IsArray(jids)) {
                    int n = cJSON_GetArraySize(jids);
                    for (int i = 0; i < n; ++i) {
                        const cJSON *it = cJSON_GetArrayItem(jids, i);
                        if (cJSON_IsString(it)) alarm_stop(it->valuestring);
                    }
                } else {
                    alarm_stop(NULL);
                }
                cJSON_Delete(j);
            }
            break;
        }
        case OE_WS_EVT_PLAY_AMBIENT: {
            // { type:'play_ambient', audioMarker, loop, volume? } — start
            // looped ambient playback (e.g. thunderstorm.mp3 from a
            // "goodnight" routine). Handler is non-blocking: spawn a worker
            // that owns the HTTP stream + audio_io_write_pcm for the
            // session's duration.
            cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
            if (j) {
                const cJSON *jm = cJSON_GetObjectItem(j, "audioMarker");
                const cJSON *jl = cJSON_GetObjectItem(j, "loop");
                const cJSON *jv = cJSON_GetObjectItem(j, "volume");
                if (cJSON_IsString(jm) && jm->valuestring[0] && s_ambient_req_q) {
                    const char *mk = jm->valuestring;
                    if (s_ambient_active &&
                        strncmp(mk, s_ambient_cur_marker, sizeof(s_ambient_cur_marker) - 1) == 0) {
                        // Same marker we already have open. We pause/resume this
                        // stream locally around each turn, so we don't re-fetch.
                        // But if we're currently PAUSED, treat the server's
                        // restore as a recovery nudge and resume — this is the
                        // only way the server can un-wedge ambient if a turn-end
                        // failed to resume it. ambient_resume() no-ops if we're
                        // not actually paused, so an in-flight live stream is
                        // never disturbed.
                        if (s_ambient_paused) {
                            // Only resume when the turn is COMPLETELY idle.
                            // The server's 3s auto-restore backstop can land
                            // mid-reply (long replies keep pacing well past
                            // the LLM turn) — resuming here played the rain
                            // UNDER the reply. When busy, stay paused: the
                            // capture loop's idle check resumes it later.
                            if (!s_verify_playback_hold_active &&
                                !leds_buttons_is_muted() && !alarm_is_firing() &&
                                !s_in_utterance && !s_awaiting_reply && !s_stream_active &&
                                get_followup_until_us() == 0 &&
                                uxQueueMessagesWaiting(s_sentence_q) == 0 && token_accum_empty()) {
                                oe_udplog_send("[ambient] same-marker restore -> resume");
                                ambient_resume();
                            } else {
                                oe_udplog_send("[ambient] same-marker restore deferred (turn busy)");
                            }
                        } else {
                            oe_udplog_send("[ambient] play_ambient ignored (same marker live)");
                        }
                    } else {
                        // New / different marker → hand it to the single ambient
                        // task and interrupt any current playback to switch.
                        ambient_req_t req = {0};
                        strncpy(req.marker, mk, sizeof(req.marker) - 1);
                        req.loop = cJSON_IsBool(jl) ? cJSON_IsTrue(jl) : true;
                        req.volume = cJSON_IsNumber(jv) ? jv->valueint : -1;
                        // Interrupt BEFORE queueing: the ambient task clears
                        // s_ambient_stop only AFTER it dequeues, so ordering it
                        // first both aborts an in-flight fetch and can't race past
                        // the task's reset to abort the session it's starting.
                        s_ambient_stop = true;
                        if (xQueueSend(s_ambient_req_q, &req, 0) != pdTRUE) {
                            ambient_req_t drop;                   // queue full → drop oldest
                            xQueueReceive(s_ambient_req_q, &drop, 0);
                            xQueueSend(s_ambient_req_q, &req, 0);
                        }
                    }
                }
                cJSON_Delete(j);
            }
            break;
        }
        case OE_WS_EVT_SET_DEVICE_NAME: {
            // { type:'set_device_name', name:'Kitchen' } — user-edited name
            // from Settings → Voice devices. Persist + update in-memory
            // + live-refresh the AirPlay mDNS instance label so iOS sees
            // the new name immediately (no reboot, active stream survives
            // — raop doesn't reference the name post-create).
            cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
            if (j) {
                cJSON *jn = cJSON_GetObjectItem(j, "name");
                if (cJSON_IsString(jn) && jn->valuestring && jn->valuestring[0]) {
                    char clean[OE_DEVICE_NAME_MAX];
                    strncpy(clean, jn->valuestring, sizeof(clean) - 1);
                    clean[sizeof(clean) - 1] = '\0';
                    // No-op if unchanged. The server reconciles device state on
                    // every reconnect (re-sends set_device_name), so an
                    // unconditional NVS write would wear flash on a device that
                    // reconnects often (e.g. flapping Wi-Fi). Only persist +
                    // refresh mDNS on an actual change.
                    if (strcmp(clean, g_dev_config.device_name) != 0) {
                        nvs_creds_set_device_name(clean);
                        strncpy(g_dev_config.device_name, clean, sizeof(g_dev_config.device_name) - 1);
                        g_dev_config.device_name[sizeof(g_dev_config.device_name) - 1] = '\0';
                        airplay_set_name(clean);
                        ESP_LOGI(TAG, "device renamed to \"%s\"", clean);
                    }
                }
                cJSON_Delete(j);
            }
            break;
        }
        case OE_WS_EVT_SET_HEADPHONE_MODE: {
            cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
            if (j) {
                cJSON *je = cJSON_GetObjectItem(j, "enabled");
                bool enabled = cJSON_IsTrue(je) ||
                               (cJSON_IsNumber(je) && je->valueint != 0);
                // Apply always (cheap GPIO/state, no flash), but only persist on
                // an actual change: the server re-sends this on every reconnect
                // (state reconcile), so an unconditional NVS write would wear
                // flash on a device that reconnects often.
                bool hp_changed = (enabled != xvf3800_get_headphone_mode());
                xvf3800_set_headphone_mode(enabled);
                if (hp_changed) nvs_creds_set_headphone_mode(enabled ? 1 : 0);
                ESP_LOGI(TAG, "headphone mode: %s%s", enabled ? "on" : "off",
                         hp_changed ? "" : " (unchanged)");
                cJSON_Delete(j);
            }
            break;
        }
        case OE_WS_EVT_AIRPLAY_STOP:
            airplay_stop();
            ESP_LOGI(TAG, "airplay_stop (server)");
            break;
        case OE_WS_EVT_AIRPLAY_NEXT:
            airplay_next();
            ESP_LOGI(TAG, "airplay_next (server)");
            break;
        case OE_WS_EVT_AIRPLAY_PREV:
            airplay_prev();
            ESP_LOGI(TAG, "airplay_prev (server)");
            break;
        case OE_WS_EVT_STOP_AMBIENT:
            // Server-initiated stop ("<wake word>, stop" voice command, or
            // user clicked Stop in the web UI). Set the stop flag, drop
            // any in-flight audio, and let the ambient worker exit.
            ESP_LOGI(TAG, "stop_ambient (server)");
            s_ambient_stop = true;
            // Ambient owns only the music lane. A global stop/flush here can
            // destroy speech retained behind a provisional verify hold.
            audio_io_flush_music();
            break;
        case OE_WS_EVT_OTA_CHECK: {
            // Server-driven OTA. Worker task fans the whole flow (fetch
            // manifest, compare version, esp_https_ota, reboot) so this
            // event handler returns immediately and the WS keeps draining.
            ESP_LOGI(TAG, "ota_check requested");
            esp_err_t oe = oe_ota_start_check(g_dev_config.server_url);
            if (oe == ESP_ERR_INVALID_STATE) {
                // Already running; tell the UI rather than silently dropping.
                oe_ws_send_ota_progress("error", 0, 0, NULL, "already_in_flight");
            } else if (oe != ESP_OK) {
                oe_ws_send_ota_progress("error", 0, 0, NULL, esp_err_to_name(oe));
            }
            break;
        }
        case OE_WS_EVT_ENTER_AP: {
            // Server asked the device to re-provision Wi-Fi. Wipe stored creds
            // and reboot — the next boot finds itself unprovisioned and brings
            // up the captive-portal AP (oe-voice-XXXX). Brief delay so this log
            // and the WS frame flush before the reset. Wake-word models live in
            // SPIFFS, not NVS, so they survive.
            ESP_LOGW(TAG, "enter_ap_mode: wiping credentials + rebooting to captive portal");
            nvs_creds_factory_reset();
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
            break;
        }
        case OE_WS_EVT_REBOOT: {
            // Plain restart, everything preserved — the server health loop's
            // recovery primitive for broken-but-heartbeating states (deaf mic
            // etc.). Log the server's reason so the serial/UDP trail explains
            // the [boot] line that follows. Brief delay to flush this log.
            char reason[32] = "unspecified";
            cJSON *j = evt->text ? cJSON_ParseWithLength(evt->text, evt->text_len) : NULL;
            if (j) {
                cJSON *jr = cJSON_GetObjectItem(j, "reason");
                if (cJSON_IsString(jr) && jr->valuestring && jr->valuestring[0]) {
                    strncpy(reason, jr->valuestring, sizeof(reason) - 1);
                    reason[sizeof(reason) - 1] = '\0';
                }
                cJSON_Delete(j);
            }
            ESP_LOGW(TAG, "reboot (server, reason=%s)", reason);
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
            break;
        }
        default:
            break;
    }
}

// Per-sentence rate-lock for TTS — same vulnerability as the ambient path:
// libhelix can misparse a corrupted frame header and report a fake rate
// (22050/32000 instead of the real 24000/44100), feeding audio_io a wrong-
// rate buffer that plays at the wrong pitch. Lock to the first valid rate
// reported on this TTS sentence and ignore deviating reports thereafter.
// Reset by tts_worker_task before each oe_tts_post call so different TTS
// providers (Piper @ 22050, OpenAI @ 24000, ElevenLabs @ 44100, etc.) each
// get to set their own rate.
static uint32_t s_tts_stable_rate = 0;
// Legacy per-sentence TTS runs a blocking HTTP stream on tts_worker_task.
// Verification stalls its decoder callback (TCP backpressure preserves the
// response) and ACCEPT flips abort so old audio cannot refill the speech ring
// after teardown. The active turn tag is also checked at every PCM callback.
static volatile bool s_legacy_tts_verify_hold = false;
static volatile bool s_legacy_tts_abort = false;
static volatile bool s_legacy_tts_was_verify_held = false;
static char          s_legacy_tts_turn[24] = "";

static void tts_pcm_cb(const int16_t *pcm, size_t samples, uint32_t rate, void *user)
{
    (void)user;
    for (;;) {
        while (s_legacy_tts_verify_hold && !s_legacy_tts_abort) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (!vg_lifecycle_take()) return;
        // Close the wait→write race with gate acquisition: begin/commit/mute
        // mutate these flags under the same mutex. Register the writer before
        // releasing it so ACCEPT can close admission and wait for this exact
        // callback before flushing.
        if (s_legacy_tts_verify_hold && !s_legacy_tts_abort) {
            vg_lifecycle_give();
            continue;
        }
        bool stale = s_speech_writes_blocked || s_legacy_tts_abort ||
                     (s_legacy_tts_turn[0] &&
                      (!s_turn_id[0] ||
                       strcmp(s_legacy_tts_turn, s_turn_id) != 0));
        if (!stale) __sync_add_and_fetch(&s_speech_writers, 1);
        vg_lifecycle_give();
        if (stale) {
            // Make oe_tts_post's next HTTP event abort promptly too; returning
            // here prevents every remaining decoded frame in the current chunk.
            s_legacy_tts_abort = true;
            return;
        }
        break;
    }
    if (s_tts_stable_rate == 0 && rate > 0) {
        s_tts_stable_rate = rate;
        ESP_LOGI(TAG, "tts: rate locked at %u Hz", (unsigned)rate);
    }
    uint32_t effective_rate = s_tts_stable_rate > 0 ? s_tts_stable_rate : rate;
    audio_io_write_speech_pcm(pcm, samples, effective_rate);
    __sync_sub_and_fetch(&s_speech_writers, 1);
}

// ── Server-side TTS streaming (push model) ──────────────────────────────────
// (state flags s_stream_active/s_stream_end_req/s_pcm_frame declared earlier so
// the WS callback can use them.) Waits for the ring to drain after the last
// pushed frame, then tears
// down speaking state. Keys idle off the actual buffer (not a timer), which is
// what fixes the "last sentence clipped" race.
static void stream_finalize_task(void *arg)
{
    while (1) {
        if (s_stream_end_req && s_stream_active &&
            !s_verify_playback_hold_active) {
            s_stream_end_req = false;
            bool deferred = false;
            for (int i = 0; i < 300; ++i) {           // up to ~15 s safety cap
                // A verify hold can begin after this task observed end_req.
                // Hand ownership back to the gate instead of finalizing the
                // old reply underneath a potentially-rejected wake.
                if (s_verify_playback_hold_active || !s_stream_active) {
                    deferred = true;
                    break;
                }
                uint32_t used = 0, cap = 0;
                audio_io_get_playback_buf_stats(&used, &cap);
                if (used == 0) break;
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            if (deferred || s_verify_playback_hold_active ||
                !s_stream_active) {
                if (s_stream_active) s_stream_end_req = true;
                vTaskDelay(pdMS_TO_TICKS(30));
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(120));           // I²S DMA tail margin
            if (!vg_lifecycle_take()) continue;
            if (s_verify_playback_hold_active || !s_stream_active) {
                if (s_stream_active) s_stream_end_req = true;
                vg_lifecycle_give();
                continue;
            }
            // Publish the terminal stream state while begin/commit/mute are
            // excluded. A gate that wins this mutex sees a live stream and
            // holds it; a finalizer that wins first makes the old reply fully
            // terminal before that gate snapshots playback.
            s_stream_active = false;
            s_stream_turn_id[0] = 0;
            vg_lifecycle_give();
            for (uint8_t _i = 0; _i < WW_NUM_SLOTS; ++_i)
                if (s_ww[_i]) wakeword_notify_speaking_ended(s_ww[_i]);
            xvf3800_enable_amplifier(false);
            airplay_note_amp_forced_off();
            int64_t now_us = esp_timer_get_time();
            int64_t followup_until = get_followup_until_us();
            // Playback has drained — START any deferred follow-up window now, at
            // the instant the assistant stops talking, so the full answer window
            // is available regardless of how long the streamed reply ran.
            if (s_followup_pending_ms > 0) {
                followup_until = now_us + (int64_t)s_followup_pending_ms * 1000;
                set_followup_until_us(followup_until);
                s_followup_pending_ms = 0;
            }
            if (followup_until > now_us) {
                // Window opens on clean mic audio only — the reply's tail
                // must not ride into the answer capture as pre-roll.
                preroll_reset();
                set_ui_state(UI_STATE_LISTENING);
            } else if (s_stream_end_pending) {
                // Burst closed but the turn is still open server-side (slow
                // delegation/tool). Rotating rainbow ring = "working on it"
                // — mic is fully open the whole time.
                set_ui_state(UI_STATE_WAITING);
                s_wait_led_until_us = now_us + WAIT_LED_TIMEOUT_US;
            } else {
                set_ui_state(UI_STATE_IDLE);
            }
            s_stream_end_pending = false;
            airplay_resume();
        } else if (s_stream_active && !s_stream_end_req &&
                   !s_paused_for_barge &&
                   !s_verify_playback_hold_active) {
            // Stall watchdog: tts_audio_begin arrived but the stream went
            // silent with no tts_audio_end and the ring has fully drained.
            // Without this there was NO timeout on SPEAKING — a server crash
            // mid-stream on a healthy socket left the amp on (AEC suppressing
            // the mic) and every non-owner wake slot gated forever. Use the
            // same teardown as the WS-disconnect path. Suspended while a
            // barge or wake verification holds the pacer paused (frames stop
            // on purpose).
            int64_t last = s_last_tts_frame_us;
            uint32_t used = 0, cap = 0;
            audio_io_get_playback_buf_stats(&used, &cap);
            if (last != 0 && used == 0 &&
                esp_timer_get_time() - last > (int64_t)TTS_STREAM_STALL_TIMEOUT_MS * 1000) {
                if (!vg_lifecycle_take()) {
                    vTaskDelay(pdMS_TO_TICKS(30));
                    continue;
                }
                // Revalidate under the lifecycle lock; a gate may have begun
                // after the watchdog's outer snapshot and intentionally
                // stopped frame arrival.
                if (s_verify_playback_hold_active || !s_stream_active ||
                    s_stream_end_req || s_paused_for_barge) {
                    vg_lifecycle_give();
                    continue;
                }
                s_stream_active = false;
                s_stream_turn_id[0] = 0;
                vg_lifecycle_give();
                ESP_LOGW(TAG, "tts stream stalled (%d ms, ring empty, no end) — tearing down SPEAKING",
                         TTS_STREAM_STALL_TIMEOUT_MS);
                oe_udplog_send("[tts] stream stall watchdog fired");
                for (uint8_t _i = 0; _i < WW_NUM_SLOTS; ++_i)
                    if (s_ww[_i]) wakeword_notify_speaking_ended(s_ww[_i]);
                xvf3800_enable_amplifier(false);
                airplay_note_amp_forced_off();
                audio_io_stop_playback();
                audio_io_flush_playback();
                s_followup_pending_ms = 0;
                set_ui_state(UI_STATE_IDLE);
                airplay_resume();
            }
        }
        // Wait-LED expiry: the turn never continued (abandoned delegation,
        // lost frames). Drop the spinner back to idle — mic was never gated.
        if (s_wait_led_until_us != 0 && esp_timer_get_time() > s_wait_led_until_us &&
            !s_stream_active) {
            s_wait_led_until_us = 0;
            if (!s_in_utterance && !s_awaiting_reply && get_followup_until_us() == 0) {
                set_ui_state(UI_STATE_IDLE);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

static void tts_worker_task(void *arg)
{
    sentence_t s;
    while (1) {
        if (xQueueReceive(s_sentence_q, &s, portMAX_DELAY) != pdTRUE) continue;
        if (leds_buttons_is_muted()) continue;
        // Publish ownership before waiting: a gate that starts after dequeue
        // can now hold/abort this exact sentence even though no PCM exists yet.
        if (!vg_lifecycle_take()) continue;
        s_legacy_tts_abort = false;
        s_legacy_tts_was_verify_held = false;
        snprintf(s_legacy_tts_turn, sizeof(s_legacy_tts_turn), "%s",
                 s.turn_id[0] ? s.turn_id : s_turn_id);
        s_legacy_tts_active = true;
        vg_lifecycle_give();
        while (s_legacy_tts_verify_hold && !s_legacy_tts_abort) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (s_legacy_tts_abort ||
            (s_legacy_tts_turn[0] &&
             (!s_turn_id[0] || strcmp(s_legacy_tts_turn, s_turn_id) != 0)) ||
            leds_buttons_is_muted()) {
            ESP_LOGI(TAG, "dropping stale/aborted TTS sentence (turn=%s current=%s)",
                     s_legacy_tts_turn, s_turn_id);
            s_legacy_tts_abort = true;
            s_legacy_tts_active = false;
            continue;
        }
        ESP_LOGI(TAG, "tts sentence: \"%s\"", s.text);
        set_ui_state(UI_STATE_SPEAKING);
        // Re-open the wake-word feed for barge-in: THINKING is over now that
        // we're actually about to play audio. The wakeword module's own
        // speaking-state machinery handles AEC-residual suppression during
        // playback (see wakeword_notify_speaking_began below).
        s_awaiting_reply = false;
        // Notify ALL loaded slots so each one's speaking-state gate holds
        // off detection during TTS playback (otherwise the AEC residual on
        // a non-active slot could still false-trigger).
        for (uint8_t _i = 0; _i < WW_NUM_SLOTS; ++_i) {
            if (s_ww[_i]) wakeword_notify_speaking_began(s_ww[_i]);
        }
        xvf3800_enable_amplifier(true);  // turn speaker on for playback
        // Close the narrow race where a gate acquired its hold between the
        // wait/check above and this amp write.
        if (s_legacy_tts_verify_hold) {
            xvf3800_enable_amplifier(false);
            airplay_note_amp_forced_off();
        }
        audio_io_start_playback();
        // Reset TTS rate-lock for this sentence — different providers can
        // stream at different rates (Piper 22050, OpenAI 24000, ElevenLabs
        // 44100) and the per-slot ttsVoice may route to a different one
        // each turn. First valid frame of this sentence sets the lock.
        s_tts_stable_rate = 0;
        // Pass the wake slot that fired this turn so the server can pick
        // the per-slot ttsVoice (set in Settings → Voice devices). The
        // slot stays valid for the whole turn — utterance capture + STT +
        // chat + TTS — so reusing it here is correct.
        esp_err_t te = oe_tts_post(g_dev_config.server_url, g_dev_config.token,
                                    s.text, NULL, (int)s_active_slot, tts_pcm_cb,
                                    NULL, &s_legacy_tts_abort, NULL);
        ESP_LOGI(TAG, "tts post -> %s, queue_remaining=%u",
                 esp_err_to_name(te), (unsigned)uxQueueMessagesWaiting(s_sentence_q));
        // The HTTP stream can finish just before a verify hold starts, leaving
        // its tail entirely in the retained speech ring. Defer this worker's
        // amp/UI finalizer until the verdict, then (on rollback) wait for that
        // preserved tail to actually drain.
legacy_finalize_retry:
        while (s_legacy_tts_verify_hold && !s_legacy_tts_abort) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (s_legacy_tts_was_verify_held && !s_legacy_tts_abort) {
            for (int i = 0; i < 300; ++i) {
                uint32_t used = 0, cap = 0;
                audio_io_get_playback_buf_stats(&used, &cap);
                if (used == 0 || s_legacy_tts_abort) break;
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            if (!s_legacy_tts_abort) vTaskDelay(pdMS_TO_TICKS(120));
        } else {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (!vg_lifecycle_take()) continue;
        // Close the final delay→teardown window. If begin acquired the hold
        // while we slept, hand finalization back to the verdict path instead
        // of clearing SPEAKING/UI state underneath retained audio.
        if (s_legacy_tts_verify_hold && !s_legacy_tts_abort) {
            s_legacy_tts_was_verify_held = true;
            vg_lifecycle_give();
            goto legacy_finalize_retry;
        }
        bool aborted = s_legacy_tts_abort ||
                       (s_legacy_tts_turn[0] &&
                        (!s_turn_id[0] ||
                         strcmp(s_legacy_tts_turn, s_turn_id) != 0));
        s_legacy_tts_active = false;
        bool queue_empty = uxQueueMessagesWaiting(s_sentence_q) == 0;
        vg_lifecycle_give();
        if (aborted) {
            ESP_LOGI(TAG, "legacy TTS teardown superseded (turn=%s)",
                     s_legacy_tts_turn);
            continue;
        }
        if (queue_empty) {
            for (uint8_t _i = 0; _i < WW_NUM_SLOTS; ++_i) {
                if (s_ww[_i]) wakeword_notify_speaking_ended(s_ww[_i]);
            }
            xvf3800_enable_amplifier(false);
            airplay_note_amp_forced_off();
            // If the server armed a follow-up window before/during TTS,
            // sit in LISTENING instead of IDLE so capture_and_drive_task's
            // VAD-start path can fire without requiring the wake word.
            int64_t now_us = esp_timer_get_time();
            int64_t followup_until = get_followup_until_us();
            // Deferred window (reply was still in flight when await_followup
            // arrived) starts NOW, at drain — mirror of stream_finalize_task.
            if (s_followup_pending_ms > 0) {
                followup_until = now_us + (int64_t)s_followup_pending_ms * 1000;
                set_followup_until_us(followup_until);
                s_followup_pending_ms = 0;
            }
            if (followup_until > now_us) {
                ESP_LOGI(TAG, "tts drained -> LISTENING (follow-up window: %lldms left)",
                         (long long)((followup_until - now_us) / 1000));
                preroll_reset();
                set_ui_state(UI_STATE_LISTENING);
            } else {
                ESP_LOGI(TAG, "tts queue drained -> idle");
                set_ui_state(UI_STATE_IDLE);
            }
            // Reply finished — let the AirPlay session resume pushing
            // PCM. No-op if there's no active stream.
            airplay_resume();
        }
    }
}

static void mute_change_cb(bool muted)
{
    g_dev_config.muted = muted;
    if (muted) {
        // Capture stops consuming frames while muted, so it cannot perform
        // the usual provisional rollback itself. The helper serializes with
        // begin/commit, fences speech writers, performs local destruction
        // while the verify holds remain asserted, then releases them last.
        bool ambient_was_active = vg_abort_for_mute();
        // DACP connect/send/recv can block on a dead controller. The local
        // source, amp and writer teardown above is complete and the lifecycle
        // mutex is released before this best-effort remote stop.
        airplay_send_stop();
        // Ambient is a "real teardown" case (see s_ambient_stop's comment) —
        // before 0.2.62 this callback skipped it, leaving the ambient task's
        // HTTP stream alive and, worse, the server's ambient session marker
        // intact, so the wake-mid-ambient resume logic would resurrect the
        // "muted away" ambient after the next turn. Stop the worker AND tell
        // the server so both halves of the session die together.
        if (ambient_was_active) {
            oe_ws_send_ambient_stopped("mute");
        }
        // Cancel any in-flight server turn after local mute is already
        // complete. The turn-scoped server guard makes a stale id a no-op.
        // Modern firmware never sends an unscoped fallback: with no owned
        // turn id it could abort an unrelated browser/device coordinator chat.
        if (s_turn_id[0]) {
            oe_ws_send_stop(g_dev_config.default_agent_id, s_turn_id);
        }
    } else {
        // Physical mute owns durable producer fences. Re-open them only after
        // the button state has published unmuted; any still-pending lane flush
        // remains its own admission fence until playback_task services it.
        if (vg_lifecycle_take()) {
            s_speech_writes_blocked = false;
            audio_io_allow_speech_writes();
            audio_io_allow_music_writes();
            // The alarm's amp callback is one-shot per firing session. If it
            // arrived while mute owned physical silence, restore from durable
            // alarm state now rather than leaving that session inaudible.
            if (alarm_is_firing()) {
                audio_io_start_playback();
                xvf3800_enable_amplifier(true);
            }
            vg_lifecycle_give();
        }
    }
}

// Stage-C gate for a confirmed speech barge: is this transcript an actual
// interjection, or just a vocal tic / breath the local verify let through?
// Fillers resume the paused reply instead of killing it.
static bool transcript_is_filler(const char *t)
{
    // Normalize: keep letters only, lowercase.
    char norm[24];
    size_t k = 0;
    for (const char *p = t; *p && k < sizeof(norm) - 1; ++p) {
        char c = *p;
        if (c >= 'A' && c <= 'Z') c += 32;
        if (c >= 'a' && c <= 'z') norm[k++] = c;
    }
    norm[k] = 0;
    if (k <= 2) return true;   // "uh", "mm", stray phonemes
    static const char *kFillers[] = {
        "umm", "uhh", "hmm", "mhm", "huh", "ahh", "ohh", "err", "hmmm",
    };
    for (size_t i = 0; i < sizeof(kFillers) / sizeof(kFillers[0]); ++i) {
        if (strcmp(norm, kFillers[i]) == 0) return true;
    }
    return false;
}

// ── Wake-word verify gate: device-owned provisional session (Option A) ───────
// On a genuine wake fire, when OE's verify-gate proxy path is configured, we do NOT
// immediately ack (no LED flip, no chime, no stt_begin). Instead we snapshot
// the wake window [fire−2.0 s, fire+0.5 s] from the enlarged pre-roll ring plus
// the first 0.5 s of command capture, POST it to the paired OE fixed gate path
// on a worker task, and keep buffering the command locally. Only an explicit
// effective=="accept" may commit the turn. A reject, transport/provider error,
// malformed response, local deadline, missing identity/resource, disallowed OE
// origin, or busy worker drops the fire. An explicitly empty provisioned path
// is the one disabled state and preserves the legacy immediate path. Everything
// keys on turn_id so a late verdict for a superseded fire is ignored. Follow-up
// / speech-barge captures never enter this path (they carry no wake word). See
// oe-design-docs/verify-gate-integration-plan.md §2.2.
#define VG_PRE_SAMPLES   (16000 * 2000 / 1000)              // 2.0 s pre-fire  (32000)
#define VG_TAIL_SAMPLES  (16000 *  500 / 1000)              // 0.5 s post-fire  (8000)
#define VG_WIN_SAMPLES   (VG_PRE_SAMPLES + VG_TAIL_SAMPLES) // 2.5 s window    (40000)
#define VG_DEADLINE_MS   1200                               // client long-poll deadline
#define VG_STT_REPLAY_BUDGET_MS 160                         // ample margin under 512 ms ring
#define VG_CONTROL_SEND_TIMEOUT_MS 20                       // never starve capture

typedef enum {
    VG_CONTROL_RETRY_RESUME = 1,
    VG_CONTROL_RETRY_STOP,
} vg_control_retry_kind_t;

typedef struct {
    vg_control_retry_kind_t kind;
    char turn_id[24];
    char hold_id[24];
    char agent_id[OE_AGENT_ID_MAX];
    uint32_t hold_generation;
    uint32_t connection_epoch;
} vg_control_retry_t;

// A bounded gate send normally succeeds immediately. If the socket TX lock was
// briefly busy, retry off the capture task so a REJECT cannot leave the old
// server pacer paused, and ACCEPT cannot leave an old turn producing forever.
// Both messages are turn-scoped; a late retry is a safe no-op after supersede.
static void vg_control_retry_task(void *arg)
{
    vg_control_retry_t *job = (vg_control_retry_t *)arg;
    for (int attempt = 0; attempt < 8; ++attempt) {
        if (!oe_ws_connected() ||
            __sync_fetch_and_add(&s_ws_connection_epoch, 0) !=
                job->connection_epoch) {
            break;
        }
        if (job->kind == VG_CONTROL_RETRY_RESUME) {
            if (!vg_lifecycle_take()) break;
            const bool still_current =
                s_verify_hold_generation == job->hold_generation;
            vg_lifecycle_give();
            if (!still_current) break;
        }
        esp_err_t err =
            job->kind == VG_CONTROL_RETRY_RESUME
                ? oe_ws_send_tts_resume_hold_timeout(
                      job->turn_id, job->hold_id, 100)
                : oe_ws_send_stop_hold_timeout(
                      job->agent_id, job->turn_id, job->hold_id, 100);
        if (err == ESP_OK) {
            free(job);
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    ESP_LOGW(TAG, "verify control retry exhausted (%s turn=%s)",
             job->kind == VG_CONTROL_RETRY_RESUME ? "resume" : "stop",
             job->turn_id);
    free(job);
    vTaskDelete(NULL);
}

static void vg_schedule_control_retry(vg_control_retry_kind_t kind,
                                      const char *turn_id,
                                      const char *hold_id,
                                      const char *agent_id,
                                      uint32_t hold_generation,
                                      uint32_t connection_epoch)
{
    if ((!turn_id || !turn_id[0]) &&
        (!hold_id || !hold_id[0])) {
        return;
    }
    if (!oe_ws_connected() ||
        __sync_fetch_and_add(&s_ws_connection_epoch, 0) !=
            connection_epoch) {
        return;
    }
    vg_control_retry_t *job =
        (vg_control_retry_t *)calloc(1, sizeof(*job));
    if (!job) {
        ESP_LOGW(TAG, "verify control retry alloc failed");
        return;
    }
    job->kind = kind;
    snprintf(job->turn_id, sizeof(job->turn_id), "%s",
             turn_id ? turn_id : "");
    snprintf(job->hold_id, sizeof(job->hold_id), "%s",
             hold_id ? hold_id : "");
    snprintf(job->agent_id, sizeof(job->agent_id), "%s",
             agent_id ? agent_id : "");
    job->hold_generation = hold_generation;
    job->connection_epoch = connection_epoch;
    if (xTaskCreate(vg_control_retry_task, "vg_ctrl_retry", 3072, job, 5,
                    NULL) != pdPASS) {
        ESP_LOGW(TAG, "verify control retry task create failed");
        free(job);
    }
}

typedef enum { PROV_NONE = 0, PROV_FILL_TAIL, PROV_AWAIT_VERDICT } prov_state_t;
static prov_state_t s_prov_state   = PROV_NONE;   // capture-task-owned
static vad_end_reason_t s_prov_vad_end = VAD_END_NONE; // first terminal command boundary
static int16_t     *s_verify_wav   = NULL;        // PSRAM, holds the assembled wake window
static size_t       s_verify_win_len = 0;         // samples assembled so far
static size_t       s_verify_pre_len = 0;         // pre-fire samples snapshotted (<=VG_PRE_SAMPLES)
static int64_t      s_prov_deadline_us = 0;

// Capture-task → verify-worker handshake. Dispatch fills s_verify_wav + the
// job params WHILE the worker is idle (s_verify_inflight==false), then gives
// the binary semaphore. The worker reads s_verify_wav in place (no copy); the
// capture task must not reuse s_verify_wav until s_verify_inflight clears — it
// never does during a live provisional/command. A new genuine wake that finds
// the worker busy is rejected; it must never fall into the ungated command path.
// The result is published as (s_verify_effective, s_verify_res_turn) with
// s_verify_done set last; the capture task ignores a result whose turn does not
// match the live turn_id.
static SemaphoreHandle_t s_verify_job_sem  = NULL;
static TaskHandle_t      s_verify_worker_handle = NULL;
static volatile bool     s_verify_inflight = false;
static volatile bool     s_verify_done     = false;
static volatile int      s_verify_effective = (int)OE_VERIFY_ERROR;
static char              s_verify_res_turn[24] = "";   // turn the published result belongs to
static char              s_verify_job_turn[24] = "";   // turn the dispatched job is for
static char              s_verify_job_server_url[OE_URL_MAX] = "";
static char              s_verify_job_gate_path[OE_URL_MAX] = "";
static char              s_verify_job_device_id[OE_DEVICE_ID_MAX] = "";
static char              s_verify_job_slug[WW_WAKE_SLUG_MAX] = "";
static float             s_verify_job_score  = -1.0f;
static size_t            s_verify_job_samples = 0;

// Playback hold snapshot. The gate owns an independent audio_io/AirPlay hold;
// these fields track only source-side changes that must be rolled back.
static bool              s_verify_hold_stream_flow_paused = false;
static bool              s_verify_hold_ambient_owned = false;
static bool              s_verify_hold_transferred_barge = false;
static char              s_verify_hold_old_turn[24] = "";
static char              s_verify_hold_ambient_marker[64] = "";

typedef enum {
    VG_DISABLED = 0,  // explicit empty path: preserve the legacy wake path
    VG_READY,
    VG_BLOCKED,       // configured, but cannot prove this wake: drop it
} verify_gate_state_t;

// Configuration and readiness are deliberately separate. Once the proxy path is
// configured, a missing canonical id, capture resource, worker, token, or an
// already-busy worker is a fail-closed condition—not a reason to bypass.
static verify_gate_state_t verify_gate_state(const char **blocked_reason)
{
    bool known;
    bool has_path;
    bool has_device_id;
    portENTER_CRITICAL(&s_verify_config_mux);
    known = s_verify_gate_config_known;
    has_path = g_dev_config.verify_gate_path[0] != 0;
    has_device_id = g_dev_config.device_id[0] != 0;
    portEXIT_CRITICAL(&s_verify_config_mux);
    if (!known) {
        if (blocked_reason) *blocked_reason = "gate_policy_pending";
        return VG_BLOCKED;
    }
    if (!has_path) return VG_DISABLED;
    if (!has_device_id) {
        if (blocked_reason) *blocked_reason = "missing_device_id";
        return VG_BLOCKED;
    }
    if (!g_dev_config.token[0]) {
        if (blocked_reason) *blocked_reason = "missing_token";
        return VG_BLOCKED;
    }
    if (!oe_verify_gate_origin_allowed(g_dev_config.server_url)) {
        if (blocked_reason) *blocked_reason = "disallowed_oe_origin";
        return VG_BLOCKED;
    }
    // Correlation is part of the safety boundary, not an optional
    // optimization. A persisted gate URL may outlive a server rollback; on a
    // server that does not echo turn ids, late old TTS frames could cross the
    // ACCEPT promotion and refill the freshly flushed speech lane.
    if (!s_caps_turn_ids || !s_caps_tts_pause ||
        !s_caps_tts_hold_ids) {
        if (blocked_reason) {
            *blocked_reason = "server_gate_protocol_unavailable";
        }
        return VG_BLOCKED;
    }
    if (!s_preroll_buf || !s_verify_wav || !s_verify_job_sem ||
        !s_verify_worker_handle) {
        if (blocked_reason) *blocked_reason = "resources_unavailable";
        return VG_BLOCKED;
    }
    if (s_verify_inflight) {
        if (blocked_reason) *blocked_reason = "worker_busy";
        return VG_BLOCKED;
    }
    return VG_READY;
}

static void vg_restore_rejected_identity(void)
{
    if (!s_prov_identity_valid) return;
    s_active_slot = s_prov_prev_slot;
    s_active_wake_prob = s_prov_prev_prob;
    snprintf(s_active_wake_slug, sizeof(s_active_wake_slug), "%s",
             s_prov_prev_slug);
    // Do not overwrite a newer follow-up event that arrived during the gate.
    if (get_followup_until_us() == 0 &&
        s_prov_prev_followup_until_us > esp_timer_get_time()) {
        set_followup_until_us(s_prov_prev_followup_until_us);
    }
    s_prov_identity_valid = false;
    s_prov_bearing_pending = false;
}

static void vg_accept_identity(void)
{
    if (s_prov_bearing_pending) {
        taskENTER_CRITICAL(&s_doa_mux);
        s_turn_bearing = s_prov_bearing;
        taskEXIT_CRITICAL(&s_doa_mux);
    }
    s_prov_identity_valid = false;
    s_prov_bearing_pending = false;
}

static void vg_clear_playback_hold_locked(void)
{
    s_verify_playback_hold_active = false;
    s_verify_hold_stream_flow_paused = false;
    s_verify_hold_ambient_owned = false;
    s_verify_hold_transferred_barge = false;
    s_verify_hold_untagged_stream_owned = false;
    s_verify_hold_old_turn[0] = 0;
    s_verify_hold_ambient_marker[0] = 0;
}

static bool vg_wait_playback_writers_locked(uint32_t timeout_ms)
{
    const int64_t deadline =
        esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while ((__sync_fetch_and_add(&s_speech_writers, 0) != 0 ||
            !audio_io_speech_writers_idle() ||
            !audio_io_music_writers_idle()) &&
           esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return __sync_fetch_and_add(&s_speech_writers, 0) == 0 &&
           audio_io_speech_writers_idle() &&
           audio_io_music_writers_idle();
}

static bool vg_wait_speech_writers_locked(uint32_t timeout_ms)
{
    const int64_t deadline =
        esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while ((__sync_fetch_and_add(&s_speech_writers, 0) != 0 ||
            !audio_io_speech_writers_idle()) &&
           esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return __sync_fetch_and_add(&s_speech_writers, 0) == 0 &&
           audio_io_speech_writers_idle();
}

static bool vg_service_gate_flushes_locked(uint32_t timeout_ms)
{
    const int64_t deadline =
        esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    do {
        const bool music_clean =
            audio_io_gate_service_pending_music_flush();
        const bool speech_clean =
            audio_io_gate_service_pending_speech_flush();
        if (music_clean && speech_clean) return true;
        vTaskDelay(pdMS_TO_TICKS(5));
    } while (esp_timer_get_time() < deadline);
    return false;
}

static bool vg_service_gate_speech_flush_locked(uint32_t timeout_ms)
{
    const int64_t deadline =
        esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    do {
        if (audio_io_gate_service_pending_speech_flush()) return true;
        vTaskDelay(pdMS_TO_TICKS(5));
    } while (esp_timer_get_time() < deadline);
    return false;
}

// Called with s_verify_lifecycle_mutex held immediately before releasing the
// audio gate. alarm_amp_cb uses the same mutex, so an alarm start is either
// represented by this latch or observes the fully released gate and performs
// its own handoff; it cannot fall between the two.
static void vg_release_deferred_alarm_locked(void)
{
    if (!s_alarm_amp_deferred) return;
    s_alarm_amp_deferred = false;
    if (leds_buttons_is_muted() || !alarm_is_firing()) return;
    audio_io_start_playback();
    xvf3800_enable_amplifier(true);
}

// External priority/destructive events request cancellation through the same
// lifecycle state used by begin/commit. The normal path takes the mutex; the
// busy path publishes the one-way cancellation flag without waiting so a
// synchronous WebSocket error callback cannot recursively deadlock its sender.
static void vg_request_cancel(void)
{
    if (!s_verify_lifecycle_mutex) {
        s_verify_cancelled = true;
        return;
    }
    // A failed esp_websocket_client send can synchronously dispatch the
    // DISCONNECTED callback on the very task that currently owns this mutex.
    // Never block recursively here: publish the cancellation lock-free when
    // the lifecycle is busy; begin/commit/rollback recheck it before crossing
    // or releasing their boundary.
    if (xSemaphoreTake(s_verify_lifecycle_mutex, 0) != pdTRUE) {
        s_verify_cancelled = true;
        return;
    }
    if (s_verify_playback_hold_active) s_verify_cancelled = true;
    xSemaphoreGive(s_verify_lifecycle_mutex);
}

// Release a provisional hold without destroying any playback. Source
// producers are re-opened while the independent audio gate is still held;
// the final release then makes the preserved rings audible atomically.
static void vg_playback_hold_finish_rollback_locked(void)
{
    if (!s_verify_playback_hold_active) return;

    // The mute callback has already published the physical mute state and is
    // waiting for this mutex. It owns destructive teardown; do not resume a
    // server pacer or clear its legacy abort while the speaker is muting.
    if (leds_buttons_is_muted()) {
        s_verify_cancelled = true;
        return;
    }

    const bool same_old_turn =
        strcmp(s_verify_hold_old_turn, s_turn_id) == 0;
    const bool stream_live =
        s_stream_active &&
        (same_old_turn ||
         (s_verify_hold_untagged_stream_owned &&
          !s_stream_turn_id[0]));

    s_legacy_tts_verify_hold = false;
    // begin never asserts the legacy abort flag, so rollback must not clear
    // it. A concurrent destructive owner (mute/disconnect/explicit stop) may
    // have asserted it for reasons unrelated to this reversible hold.

    // A source that independently stopped during verification may have queued
    // a destructive lane flush. Service it while playback is still
    // acknowledged-held; the request itself already canceled/fenced old
    // writers, so this does not truncate an otherwise live REJECT path.
    if (!vg_service_gate_flushes_locked(250)) {
        ESP_LOGE(TAG, "verify rollback could not settle pending lane flush");
        oe_udplog_send("[verify] rollback flush invariant failed — rebooting");
        esp_restart();
        return;
    }

    bool ambient_resumed = false;
    if (s_verify_hold_ambient_owned &&
        strncmp(s_ambient_cur_marker, s_verify_hold_ambient_marker,
                sizeof(s_verify_hold_ambient_marker)) == 0) {
        // The lifecycle mutex is already held. This variant bypasses only the
        // provisional hold/busy checks; mute, alarm, stop and source identity
        // still veto restoration.
        ambient_resumed = ambient_resume_locked(true);
    }

    // Do not pre-enable the amp from an AirPlay state snapshot: RAOP STOP can
    // race that snapshot while its amp latch is still marked forced-off. A
    // genuinely live transport receives another PCM callback immediately
    // after verify release, and that callback owns the amp enable.
    const bool source_will_play =
        stream_live || s_legacy_tts_active || ambient_resumed ||
        alarm_is_firing();
    if (!audio_io_is_paused() && source_will_play) {
        xvf3800_enable_amplifier(true);
        audio_io_start_playback();
    }

    // AirPlay can begin feeding its music lane again while the audio gate is
    // still closed; releasing the shared gate last makes rollback audible as
    // one transition and preserves the queued speech lane.
    airplay_verify_release();
    vg_release_deferred_alarm_locked();
    audio_io_gate_release_playback();
    if (s_verify_hold_transferred_barge && stream_live) {
        s_barge_cooldown_until_us =
            esp_timer_get_time() +
            (int64_t)BARGE_FALSE_ALARM_COOLDOWN_MS * 1000;
        set_ui_state(UI_STATE_SPEAKING);
    }

    vg_clear_playback_hold_locked();
}

static void vg_playback_hold_rollback(void)
{
    if (!vg_lifecycle_take()) return;
    if (!s_verify_playback_hold_active) {
        vg_lifecycle_give();
        return;
    }
    if (leds_buttons_is_muted()) {
        s_verify_cancelled = true;
        vg_lifecycle_give();
        return;
    }

    const bool need_resume =
        s_verify_hold_stream_flow_paused &&
        s_caps_tts_pause && s_caps_tts_hold_ids &&
        oe_ws_connected();
    char resume_turn[sizeof(s_verify_hold_old_turn)];
    char resume_hold[sizeof(s_prov_turn_id)];
    snprintf(resume_turn, sizeof(resume_turn), "%s",
             s_verify_hold_old_turn);
    snprintf(resume_hold, sizeof(resume_hold), "%s",
             s_prov_turn_id);
    const uint32_t resume_generation =
        __sync_fetch_and_add(&s_verify_hold_generation, 0);
    const uint32_t resume_epoch =
        __sync_fetch_and_add(&s_ws_connection_epoch, 0);

    // Never call esp_websocket_client while holding the lifecycle mutex. A
    // send error synchronously dispatches ERROR then DISCONNECTED callbacks;
    // both must be able to enter this lifecycle without self-deadlocking.
    if (need_resume) {
        vg_lifecycle_give();
        esp_err_t re = ESP_FAIL;
        for (int attempt = 0; attempt < 2 && re != ESP_OK; ++attempt) {
            if (__sync_fetch_and_add(&s_ws_connection_epoch, 0) !=
                resume_epoch) {
                break;
            }
            re = oe_ws_send_tts_resume_hold_timeout(
                resume_turn, resume_hold, 50);
        }
        if (re != ESP_OK) {
            ESP_LOGW(TAG, "verify rollback tts_resume failed after retry: %s",
                     esp_err_to_name(re));
            oe_udplog_send("[verify] rollback resume send failed");
            vg_schedule_control_retry(
                VG_CONTROL_RETRY_RESUME, resume_turn, resume_hold, NULL,
                resume_generation, resume_epoch);
        }
        if (!vg_lifecycle_take()) return;
    }

    // Mute may have won while the bounded resume was outside the lock.
    // Its local teardown already released/cleared the holds.
    if (!s_verify_playback_hold_active) {
        vg_lifecycle_give();
        return;
    }
    vg_playback_hold_finish_rollback_locked();
    vg_lifecycle_give();
}

// Acquire every reversible source hold before gate audio capture starts.
// A tagged streamed reply additionally needs a bounded server-pacer pause;
// inability to establish that pause drops the wake with playback restored.
static bool vg_playback_hold_begin(void)
{
    if (!vg_lifecycle_take()) return false;
    if (s_verify_playback_hold_active || leds_buttons_is_muted() ||
        alarm_is_firing() || !oe_ws_connected() ||
        !s_caps_turn_ids || !s_caps_tts_pause ||
        !s_caps_tts_hold_ids || !s_prov_turn_id[0]) {
        vg_lifecycle_give();
        return false;
    }

    const bool stream_now = s_stream_active;
    if (stream_now && s_stream_turn_id[0] &&
        (!s_turn_id[0] ||
         strcmp(s_stream_turn_id, s_turn_id) != 0)) {
        oe_udplog_send("[verify] active TTS is not safely pausable — fire dropped");
        vg_lifecycle_give();
        return false;
    }

    snprintf(s_verify_hold_old_turn, sizeof(s_verify_hold_old_turn), "%s",
             s_turn_id);
    s_verify_hold_stream_flow_paused = false;
    s_verify_hold_ambient_owned = false;
    s_verify_hold_transferred_barge = s_paused_for_barge;
    s_verify_hold_untagged_stream_owned =
        stream_now && !s_stream_turn_id[0];
    s_verify_hold_ambient_marker[0] = 0;
    s_verify_cancelled = false;

    // Acquire every component hold first, then publish ACTIVE while still
    // holding the lifecycle mutex. A mute/alarm/disconnect requester can no
    // longer observe a half-acquired set and strand the later flags.
    esp_err_t hold_err = audio_io_gate_hold_playback();
    if (hold_err != ESP_OK) {
        ESP_LOGW(TAG, "verify playback hold could not quiesce audio task: %s",
                 esp_err_to_name(hold_err));
        oe_udplog_send("[verify] playback hold unavailable — fire dropped");
        vg_lifecycle_give();
        return false;
    }
    __sync_add_and_fetch(&s_verify_hold_generation, 1);
    airplay_verify_hold();
    s_legacy_tts_verify_hold = true;
    if (s_legacy_tts_active) s_legacy_tts_was_verify_held = true;
    s_verify_playback_hold_active = true;

    if (s_ambient_active && !s_ambient_paused) {
        s_verify_hold_ambient_owned = true;
        snprintf(s_verify_hold_ambient_marker,
                 sizeof(s_verify_hold_ambient_marker), "%s",
                 s_ambient_cur_marker);
        s_ambient_paused = true;
        oe_udplog_send("[ambient] HOLD (wake verification)");
    }

    // Transfer a speech-barge pause only after the independent gate hold is
    // established, so there is no unpaused frame between the two owners.
    if (s_paused_for_barge) {
        s_paused_for_barge = false;
        s_barge_state = BARGE_NONE;
        s_barge_capture = false;
        audio_io_resume_playback();
    }

    // The playback task acknowledgement proves no more I2S writes can begin,
    // but the TX DMA may still contain the tail of its last chunk even when
    // every source/ring snapshot above is empty. Silence the physical output
    // unconditionally; rollback restores it only from live source ownership.
    xvf3800_enable_amplifier(false);
    airplay_note_amp_forced_off();

    char pause_turn[sizeof(s_verify_hold_old_turn)];
    char pause_hold[sizeof(s_prov_turn_id)];
    snprintf(pause_turn, sizeof(pause_turn), "%s",
             s_verify_hold_old_turn);
    snprintf(pause_hold, sizeof(pause_hold), "%s", s_prov_turn_id);
    // Establish the server-side latch for EVERY provisional wake, including
    // a first-ever wake with no prior turn/stream. This closes the window in
    // which a turnless announcement could start after local hold acquisition.
    // Mark it optimistically: if the frame queued just before a reported send
    // failure, rollback's matching hold-id resume remains the safe inverse.
    s_verify_hold_stream_flow_paused = true;
    vg_lifecycle_give();
    esp_err_t pe = oe_ws_send_tts_pause_hold_timeout(
        pause_turn, pause_hold, VG_CONTROL_SEND_TIMEOUT_MS);
    if (!vg_lifecycle_take()) return false;
    // Mute may have completed the entire local teardown while the bounded
    // send ran outside the lock.
    if (!s_verify_playback_hold_active) {
        vg_lifecycle_give();
        return false;
    }
    if (pe != ESP_OK) {
        ESP_LOGW(TAG, "verify tts_pause failed: %s — restoring reply",
                 esp_err_to_name(pe));
        // Restore the old reply's routing slot before releasing a legacy
        // worker that may be waiting on the hold.
        vg_restore_rejected_identity();
        vg_lifecycle_give();
        vg_playback_hold_rollback();
        return false;
    }

    // The physical/network state can change while the bounded pause send
    // yields. Revalidate before exposing a usable provisional session.
    if (s_verify_cancelled || leds_buttons_is_muted() ||
        alarm_is_firing() || !oe_ws_connected()) {
        vg_restore_rejected_identity();
        vg_lifecycle_give();
        vg_playback_hold_rollback();
        return false;
    }
    vg_lifecycle_give();
    return true;
}

// ACCEPT was explicit, but a priority owner or socket loss arrived while the
// hold-scoped STOP send was outside the lifecycle mutex. Delivery is ambiguous,
// so the old speech turn cannot be safely resumed; equally, a disconnected
// provisional id must never be promoted onto the next socket. Retire only the
// old speech ownership, restore independent live sources, and abandon capture.
// Caller owns s_verify_lifecycle_mutex.
static bool vg_cancel_accept_after_stop_locked(void)
{
    const bool had_stream = s_stream_active;
    const bool had_legacy = s_legacy_tts_active;

    s_speech_writes_blocked = true;
    audio_io_block_speech_writes();
    if (!vg_wait_speech_writers_locked(1000)) {
        ESP_LOGE(TAG, "cancelled accept could not quiesce speech writers");
        oe_udplog_send("[verify] cancelled accept invariant failed — rebooting");
        esp_restart();
        return false;
    }

    s_legacy_tts_abort = true;
    s_stream_active = false;
    s_stream_end_req = false;
    s_stream_end_pending = false;
    s_stream_turn_id[0] = 0;
    s_turn_id[0] = 0;
    s_awaiting_reply = false;
    set_awaiting_since_us(0);
    s_followup_pending_ms = 0;
    set_followup_until_us(0);
    s_wait_led_until_us = 0;
    audio_io_flush_speech();
    xQueueReset(s_sentence_q);
    reset_token_accum();
    s_legacy_tts_verify_hold = false;

    if (!vg_service_gate_speech_flush_locked(250)) {
        ESP_LOGE(TAG, "cancelled accept could not service speech flush");
        oe_udplog_send("[verify] cancelled accept flush failed — rebooting");
        esp_restart();
        return false;
    }

    if (had_stream || had_legacy) {
        for (uint8_t i = 0; i < WW_NUM_SLOTS; ++i)
            if (s_ww[i]) wakeword_notify_speaking_ended(s_ww[i]);
    }

    // The accepted command will not run, so retire its/its predecessor's
    // whole-turn AirPlay latch. The independent verify hold still prevents
    // PCM admission until the final release below.
    airplay_resume();
    const bool ambient_resumed = ambient_resume_locked(true);
    if (!audio_io_is_paused() &&
        (ambient_resumed || alarm_is_firing())) {
        audio_io_start_playback();
        xvf3800_enable_amplifier(true);
    }

    airplay_verify_release();
    vg_release_deferred_alarm_locked();
    audio_io_gate_release_playback();
    vg_clear_playback_hold_locked();
    s_speech_writes_blocked = false;
    audio_io_allow_speech_writes();
    return false;
}

// Explicit ACCEPT is the sole destructive boundary. Stop uses the old reply
// id; only after that frame is queued do we publish the provisional id, making
// all late old reply events stale.
static bool vg_playback_hold_commit(void)
{
    if (!vg_lifecycle_take()) return false;
    if (!s_verify_playback_hold_active || s_verify_cancelled ||
        leds_buttons_is_muted() || alarm_is_firing() ||
        !oe_ws_connected() || !s_caps_turn_ids ||
        !s_caps_tts_pause || !s_caps_tts_hold_ids ||
        !s_prov_turn_id[0]) {
        vg_lifecycle_give();
        return false;
    }

    char stop_turn[sizeof(s_verify_hold_old_turn)];
    char stop_hold[sizeof(s_prov_turn_id)];
    snprintf(stop_turn, sizeof(stop_turn), "%s",
             s_verify_hold_old_turn);
    snprintf(stop_hold, sizeof(stop_hold), "%s", s_prov_turn_id);
    const uint32_t stop_generation =
        __sync_fetch_and_add(&s_verify_hold_generation, 0);
    const uint32_t stop_epoch =
        __sync_fetch_and_add(&s_ws_connection_epoch, 0);

    // From this point an explicit ACCEPT owns the transition. Quarantine any
    // already-queued turnless announcement frames before the matching
    // hold-scoped STOP crosses the socket. Local writer cancellation remains
    // after the send, so every pre-boundary failure above was reversible.
    s_drop_untagged_tts = true;
    vg_lifecycle_give();

    // Stop the exact server hold even when there is no old turn id (first wake
    // during a turnless announcement). OE uses hold_id as socket ownership and
    // never falls through to the legacy user+agent abort in that case.
    esp_err_t se = ESP_FAIL;
    for (int attempt = 0; attempt < 2 && se != ESP_OK; ++attempt) {
        if (__sync_fetch_and_add(&s_ws_connection_epoch, 0) !=
            stop_epoch) {
            break;
        }
        se = oe_ws_send_stop_hold_timeout(
            g_dev_config.default_agent_id, stop_turn, stop_hold, 50);
    }
    if (se != ESP_OK) {
        ESP_LOGW(TAG, "verify accept stop send failed after retry: %s",
                 esp_err_to_name(se));
        vg_schedule_control_retry(
            VG_CONTROL_RETRY_STOP, stop_turn, stop_hold,
            g_dev_config.default_agent_id,
            stop_generation, stop_epoch);
    }
    if (!vg_lifecycle_take()) return false;
    // Mute can win while the bounded stop is outside the lock. It already
    // performed the accepted turn's destructive local teardown; report the
    // boundary as crossed so the caller only suppresses new capture.
    if (!s_verify_playback_hold_active) {
        vg_lifecycle_give();
        return true;
    }
    // The STOP send deliberately ran without the lifecycle mutex. Do not
    // publish this provisional id after its socket epoch died, nor install
    // whole-turn source pauses after an alarm/cancellation already won.
    if (leds_buttons_is_muted()) {
        // Physical mute has published and is waiting for this mutex; leave the
        // held resources for its destructive owner.
        vg_lifecycle_give();
        return false;
    }
    const bool socket_lost =
        __sync_fetch_and_add(&s_ws_connection_epoch, 0) != stop_epoch ||
        !oe_ws_connected() || !s_caps_turn_ids ||
        !s_caps_tts_pause || !s_caps_tts_hold_ids;
    const bool hold_changed =
        __sync_fetch_and_add(&s_verify_hold_generation, 0) != stop_generation ||
        strcmp(s_prov_turn_id, stop_hold) != 0;
    if (socket_lost || hold_changed || s_verify_cancelled ||
        alarm_is_firing()) {
        ESP_LOGW(TAG,
                 "verify accept abandoned after stop attempt "
                 "(socket=%d hold=%d cancel=%d alarm=%d)",
                 socket_lost ? 1 : 0, hold_changed ? 1 : 0,
                 s_verify_cancelled ? 1 : 0,
                 alarm_is_firing() ? 1 : 0);
        if (socket_lost) s_drop_untagged_tts = false;
        bool committed = vg_cancel_accept_after_stop_locked();
        vg_lifecycle_give();
        return committed;
    }

    // The accepted boundary is now crossed. Cancel and close both lane
    // admissions, then quiesce callbacks that passed correlation just before
    // promotion. Cancellation-aware ring sends leave within ~5 ms.
    s_speech_writes_blocked = true;
    audio_io_block_speech_writes();
    audio_io_block_music_writes();
    if (!vg_wait_playback_writers_locked(1000)) {
        ESP_LOGE(TAG, "accepted verify could not quiesce playback writers");
        oe_udplog_send("[verify] accepted teardown invariant failed — rebooting");
        vg_lifecycle_give();
        esp_restart();
        return true;
    }

    snprintf(s_turn_id, sizeof(s_turn_id), "%s", s_prov_turn_id);
    // Snapshot after the bounded send: an already-in-flight old BEGIN may
    // have made the stream live while the send yielded. Promotion plus the
    // lifecycle mutex now makes every later old event stale.
    const bool had_stream = s_stream_active;
    const bool had_legacy = s_legacy_tts_active;
    s_legacy_tts_abort = true;
    s_stream_active = false;
    s_stream_end_req = false;
    s_stream_end_pending = false;
    s_stream_turn_id[0] = 0;
    s_followup_pending_ms = 0;
    set_followup_until_us(0);
    s_wait_led_until_us = 0;

    // Convert reversible source holds into the established whole-turn pauses.
    if (s_ambient_active) s_ambient_paused = true;
    // Session-independent latch: a session created after this instant must
    // also stay paused for the accepted command/reply.
    airplay_pause();

    if (had_stream || had_legacy) {
        for (uint8_t i = 0; i < WW_NUM_SLOTS; ++i)
            if (s_ww[i]) wakeword_notify_speaking_ended(s_ww[i]);
    }
    xvf3800_enable_amplifier(false);
    airplay_note_amp_forced_off();
    // Keep the gate hold asserted until the destructive stop/flush is done;
    // this prevents a one-tick audible gap during ownership conversion.
    audio_io_stop_playback();
    audio_io_flush_playback();
    xQueueReset(s_sentence_q);
    reset_token_accum();
    s_legacy_tts_verify_hold = false;

    // Complete both destructive lane flushes synchronously while the
    // acknowledged gate and writer fences are still owned. Releasing first
    // would let a new accepted-turn frame queue behind an asynchronous old
    // flush and be discarded with it.
    if (!vg_service_gate_flushes_locked(250)) {
        ESP_LOGE(TAG, "accepted verify could not service gated flushes");
        oe_udplog_send("[verify] accepted flush invariant failed — rebooting");
        vg_lifecycle_give();
        esp_restart();
        return true;
    }

    // Disconnect/cancel callbacks cannot take this mutex recursively from a
    // failed WebSocket send, so reconcile once more after all destructive
    // work and before releasing source holds. Nothing below can reinstall a
    // pause after this check; a later disconnect's own resume/turn clear wins.
    const bool late_socket_lost =
        __sync_fetch_and_add(&s_ws_connection_epoch, 0) != stop_epoch ||
        !oe_ws_connected();
    if (late_socket_lost || s_verify_cancelled || alarm_is_firing()) {
        if (late_socket_lost) s_drop_untagged_tts = false;
        s_turn_id[0] = 0;
        airplay_resume();
        const bool ambient_resumed = ambient_resume_locked(true);
        if (!audio_io_is_paused() &&
            (ambient_resumed || alarm_is_firing())) {
            audio_io_start_playback();
            xvf3800_enable_amplifier(true);
        }
        airplay_verify_release();
        vg_release_deferred_alarm_locked();
        audio_io_gate_release_playback();
        vg_clear_playback_hold_locked();
        s_speech_writes_blocked = false;
        audio_io_allow_speech_writes();
        audio_io_allow_music_writes();
        vg_lifecycle_give();
        return false;
    }

    airplay_verify_release();
    vg_release_deferred_alarm_locked();
    audio_io_gate_release_playback();

    vg_clear_playback_hold_locked();
    s_speech_writes_blocked = false;
    audio_io_allow_speech_writes();
    audio_io_allow_music_writes();
    vg_lifecycle_give();
    return true;
}

// Mute is intentionally destructive and also stops frame consumption, so a
// provisional session cannot wait for the capture task to roll itself back.
static bool vg_abort_for_mute(void)
{
    if (!vg_lifecycle_take()) return false;
    const bool had_hold = s_verify_playback_hold_active;
    const bool had_ambient = s_ambient_active;
    if (had_hold) s_verify_cancelled = true;
    s_alarm_amp_deferred = false;

    // Fence source writers before the local destructive teardown. The amp is
    // disabled immediately; waiting only protects the post-flush ring state.
    s_speech_writes_blocked = true;
    audio_io_block_speech_writes();
    audio_io_block_music_writes();
    s_legacy_tts_abort = true;
    if (s_ambient_active) s_ambient_stop = true;
    s_stream_active = false;
    s_stream_end_req = false;
    s_stream_end_pending = false;
    s_stream_turn_id[0] = 0;
    // Publish the AirPlay source as stopped under its state mutex before the
    // shared physical force-off. Otherwise an already-admitted PCM callback
    // could finish its ring write between note_amp_forced_off() and the local
    // stop, still see streaming=true, and briefly re-enable the amplifier.
    airplay_mute_local();
    xvf3800_enable_amplifier(false);
    airplay_note_amp_forced_off();
    audio_io_stop_playback();
    if (!vg_wait_playback_writers_locked(1000)) {
        ESP_LOGE(TAG, "mute could not quiesce playback writers");
        oe_udplog_send("[audio] mute teardown invariant failed — rebooting");
        vg_lifecycle_give();
        esp_restart();
        return had_ambient;
    }
    // Flush again after admitted writers have left, then release provisional
    // holds last. No AirPlay callback can reopen the amp in between.
    audio_io_flush_playback();
    if (had_hold) {
        if (!vg_service_gate_flushes_locked(250)) {
            ESP_LOGE(TAG, "mute could not service gated flushes");
            oe_udplog_send("[audio] mute flush invariant failed — rebooting");
            vg_lifecycle_give();
            esp_restart();
            return had_ambient;
        }
        airplay_verify_release();
        audio_io_gate_release_playback();
        vg_clear_playback_hold_locked();
    }
    s_legacy_tts_verify_hold = false;
    vg_lifecycle_give();
    // Leave candidate/state publication to capture_and_drive_task. It may be
    // inside vg_provisional_step right now; clearing the id/state here would
    // let that task cross ACCEPT with an empty/torn candidate. On unmute, the
    // next frame observes s_verify_cancelled and performs the normal reject.
    return had_ambient;
}

// Build the one-element wake_words array from the exact identity snapshotted
// when the winning detector fired. wakeword.cpp already normalized and bounded
// it from manifest.wake_word; revalidate the invariant before creating JSON.
// There is intentionally no model-filename or all-loaded-slots fallback.
static bool vg_build_wake_words(const char *slug, char *out, size_t out_len)
{
    if (!slug || !slug[0] || !out || out_len == 0) return false;
    size_t n = strnlen(slug, WW_WAKE_SLUG_MAX);
    if (n == 0 || n >= WW_WAKE_SLUG_MAX ||
        slug[0] == '_' || slug[n - 1] == '_') {
        return false;
    }
    for (size_t i = 0; i < n; ++i) {
        char c = slug[i];
        if (!((c >= 'a' && c <= 'z') ||
              (c >= '0' && c <= '9') || c == '_') ||
            (c == '_' && i > 0 && slug[i - 1] == '_')) {
            return false;
        }
    }
    int written = snprintf(out, out_len, "[\"%s\"]", slug);
    return written > 0 && (size_t)written < out_len;
}

// Persistent worker that runs the (blocking) gate POST off the capture task so
// the command buffer keeps filling during PROVISIONAL. One job at a time.
static void verify_worker_task(void *arg)
{
    (void)arg;
    for (;;) {
        xSemaphoreTake(s_verify_job_sem, portMAX_DELAY);
        // Job params are stable: dispatch set them before giving the sem and
        // won't touch s_verify_wav/params again until s_verify_inflight clears.
        char wake_words[WW_WAKE_SLUG_MAX + 5];
        char fired_at[32] = "";
        time_t now = time(NULL);
        if (now > 1700000000) {   // SNTP has a real wall clock (post-2023)
            struct tm tmv;
            gmtime_r(&now, &tmv);
            strftime(fired_at, sizeof(fired_at), "%Y-%m-%dT%H:%M:%SZ", &tmv);
        }
        oe_verify_result_t eff = OE_VERIFY_ERROR;
        if (!vg_build_wake_words(s_verify_job_slug,
                                 wake_words, sizeof(wake_words))) {
            oe_udplog_send("[verify] invalid fired wake slug — fail-closed");
        } else {
            oe_verify_gate_post(s_verify_job_server_url,
                                s_verify_job_gate_path,
                                g_dev_config.token,
                                s_verify_job_turn, s_verify_job_device_id,
                                wake_words, s_verify_job_score, fired_at,
                                s_verify_wav, s_verify_job_samples,
                                VG_DEADLINE_MS, &eff);
        }
        s_verify_effective = (int)eff;
        snprintf(s_verify_res_turn, sizeof(s_verify_res_turn), "%s", s_verify_job_turn);
        __sync_synchronize();      // publish params/result before the done flag
        s_verify_done = true;
        s_verify_inflight = false;
    }
}

// Advance the provisional (verify-gate) session for one captured frame. Runs
// ONLY on the capture task while s_prov_state != PROV_NONE. Keeps filling the
// local command buffer + the wake-window tail, dispatches the gate POST once the
// +0.5 s tail is complete, then acts on the verdict. Command frames feed VAD
// immediately even though all STT/turn side effects remain deferred. The first
// terminal VAD boundary is latched and freezes the command buffer while gate
// polling continues. On explicit accept, return that boundary so the caller can
// finalize immediately; all other outcomes return VAD_END_NONE.
static vad_end_reason_t vg_provisional_step(const int16_t *frame, size_t n)
{
    bool local_cancel = false;
    // Mirror normal capture through the first terminal VAD frame. Once VAD
    // ends, later frames are gate-polling time, not part of this utterance.
    if (s_prov_vad_end == VAD_END_NONE) {
        if (s_capture_used + n < CAPTURE_BUFFER_SAMPLES) {
            memcpy(s_capture_buf + s_capture_used, frame, n * sizeof(int16_t));
            s_capture_used += n;
        }
        vad_end_reason_t observed = VAD_END_NONE;
        vad_feed(s_vad, frame, n, &observed);
        if (observed != VAD_END_NONE) {
            s_prov_vad_end = observed;
            oe_udplog_send("[verify] command boundary held pending verdict");
        }
    }

    local_cancel = s_verify_cancelled || alarm_is_firing() ||
                   leds_buttons_is_muted() || !oe_ws_connected();
    if (local_cancel) {
        // External teardown/priority work wins. An eventual worker result for
        // this candidate is ignored after the local fail-closed rollback.
        s_prov_state = PROV_AWAIT_VERDICT;
    }

    if (s_prov_state == PROV_FILL_TAIL) {
        // Append into the wake-window tail until we have fire+0.5 s.
        if (s_verify_wav && s_verify_win_len < VG_WIN_SAMPLES) {
            size_t room = VG_WIN_SAMPLES - s_verify_win_len;
            size_t take = n < room ? n : room;
            memcpy(s_verify_wav + s_verify_win_len, frame, take * sizeof(int16_t));
            s_verify_win_len += take;
        }
        if (s_verify_win_len >= s_verify_pre_len + VG_TAIL_SAMPLES) {
            // Tail complete → dispatch the gate POST on the worker task.
            s_verify_job_samples = s_verify_win_len;
            s_verify_job_score   = (float)s_active_wake_prob / 255.0f;
            snprintf(s_verify_job_turn, sizeof(s_verify_job_turn), "%s",
                     s_prov_turn_id);
            snprintf(s_verify_job_slug, sizeof(s_verify_job_slug), "%s",
                     s_active_wake_slug);
            // server_caps can update the live path/id pair on the WS task.
            // Snapshot it atomically; the paired OE origin is immutable for
            // this operational boot and is copied into the same worker job.
            snprintf(s_verify_job_server_url,
                     sizeof(s_verify_job_server_url), "%s",
                     g_dev_config.server_url);
            verify_gate_config_snapshot(s_verify_job_gate_path,
                                        s_verify_job_device_id);
            s_verify_done     = false;
            s_verify_inflight = true;
            s_prov_deadline_us = esp_timer_get_time() + (int64_t)(VG_DEADLINE_MS + 300) * 1000;
            s_prov_state = PROV_AWAIT_VERDICT;
            if (xSemaphoreGive(s_verify_job_sem) != pdTRUE) {
                // Should be impossible with an idle binary semaphore, but a
                // failed dispatch cannot be treated as an ungated wake.
                s_verify_effective = (int)OE_VERIFY_ERROR;
                snprintf(s_verify_res_turn, sizeof(s_verify_res_turn), "%s",
                         s_verify_job_turn);
                s_verify_inflight = false;
                s_verify_done = true;
                oe_udplog_send("[verify] worker dispatch failed — fail-closed");
            }
        }
        return VAD_END_NONE;
    }

    // PROV_AWAIT_VERDICT — the command buffer keeps filling while we wait.
    oe_verify_result_t eff;
    bool have = false;
    if (local_cancel) {
        eff = OE_VERIFY_ERROR;
        have = true;
    } else if (s_verify_done) {
        __sync_synchronize();  // pair with worker's publish barrier
        have = strcmp(s_verify_res_turn, s_prov_turn_id) == 0;
    }
    if (have) {
        if (!local_cancel) {
            eff = (oe_verify_result_t) s_verify_effective;
            s_verify_done = false;
        }
    } else if (esp_timer_get_time() > s_prov_deadline_us) {
        eff = OE_VERIFY_ERROR;
        oe_udplog_send("[verify] deadline elapsed — fail-closed");
    } else {
        return VAD_END_NONE;        // still waiting
    }

    // Re-check priority/destructive state after consuming the verdict. An
    // alarm or mute can race the worker publication within this same frame;
    // it must win before we cross the irreversible ACCEPT boundary.
    if (eff == OE_VERIFY_ACCEPT &&
        (s_verify_cancelled || alarm_is_firing() ||
         leds_buttons_is_muted() || !oe_ws_connected())) {
        eff = OE_VERIFY_ERROR;
    }

    if (eff != OE_VERIFY_ACCEPT) {
        oe_udplog_send(eff == OE_VERIFY_REJECT
                           ? "[verify] reject — fire dropped silently"
                           : "[verify] error — fire dropped fail-closed");
        s_capture_used = 0;
        s_in_utterance = false;
        s_prov_state = PROV_NONE;
        s_prov_vad_end = VAD_END_NONE;
        vg_restore_rejected_identity();
        vg_playback_hold_rollback();
        s_prov_turn_id[0] = 0;
        s_verify_cancelled = false;
        return VAD_END_NONE;
    }

    // Explicit ACCEPT: this is the sole irreversible boundary. Stop the old
    // reply with its old id, promote the provisional id, then acknowledge the
    // wake and try to preserve streaming STT.
    if (!vg_playback_hold_commit()) {
        // A pre-STOP veto is fully reversible. A priority/socket change after
        // the STOP attempt may already have retired old speech, but commit
        // clears its hold before returning false; rollback is intentionally
        // idempotent in both cases.
        s_capture_used = 0;
        s_in_utterance = false;
        s_prov_state = PROV_NONE;
        s_prov_vad_end = VAD_END_NONE;
        vg_restore_rejected_identity();
        vg_playback_hold_rollback();
        s_prov_turn_id[0] = 0;
        s_verify_cancelled = false;
        return VAD_END_NONE;
    }
    // A destructive owner may have arrived immediately after commit released
    // the lifecycle mutex. It is allowed to tear down the now-accepted old
    // playback, but must not let this task re-arm LISTENING/STT while muted,
    // under an alarm, or on a dead control socket.
    if (leds_buttons_is_muted() || alarm_is_firing() ||
        !oe_ws_connected()) {
        vg_restore_rejected_identity();
        if (vg_lifecycle_take()) {
            s_turn_id[0] = 0;
            if (!leds_buttons_is_muted()) {
                // No chat will be sent for this accepted-but-abandoned wake;
                // do not strand independent sources behind its turn latch.
                airplay_resume();
                ambient_resume_locked(true);
            }
            vg_lifecycle_give();
        }
        s_capture_used = 0;
        s_in_utterance = false;
        s_prov_state = PROV_NONE;
        s_prov_vad_end = VAD_END_NONE;
        s_prov_turn_id[0] = 0;
        s_verify_cancelled = false;
        if (!leds_buttons_is_muted() && !alarm_is_firing()) {
            set_ui_state(UI_STATE_IDLE);
        }
        oe_udplog_send("[verify] accepted but external teardown won before capture");
        return VAD_END_NONE;
    }
    vg_accept_identity();
    // Its 160 ms send budget leaves ample margin under the ~512 ms capture
    // ring; a congested socket fails over to the complete local buffer instead
    // of starving capture and truncating a command still being spoken.
    set_ui_state(UI_STATE_LISTENING);
    s_stt_streaming = s_caps_stt_stream && oe_ws_connected();
    s_stt_send_failed = false;
    s_stt_seq = 0;
    if (s_stt_streaming &&
        oe_ws_send_stt_backlog(
            s_turn_id, s_active_slot, s_active_wake_prob,
            g_dev_config.default_agent_id, s_capture_buf, s_capture_used,
            VG_STT_REPLAY_BUDGET_MS, &s_stt_seq) != ESP_OK) {
        s_stt_send_failed = true;
        oe_udplog_send("[stt] bounded gate replay failed — buffered fallback armed");
    }
    vad_end_reason_t accepted_end = s_prov_vad_end;
    s_prov_state = PROV_NONE;
    s_prov_vad_end = VAD_END_NONE;
    s_prov_turn_id[0] = 0;
    s_verify_cancelled = false;
    oe_udplog_send("[verify] accept — turn proceeds");
    return accepted_end;
}

static void capture_and_drive_task(void *arg)
{
    int16_t frame[WW_FRAME_SAMPLES];
    audio_io_start_capture();
    set_ui_state(UI_STATE_IDLE);

    // VAD tuning notes (2026-05-12):
    //   energy_threshold = 800000 — RMS² of incoming int16 frames. Raised
    //     from 250k after observing 9 s LISTENING tails on a barge-in test,
    //     where post-TTS amp decay + ambient kept the mic above the old
    //     threshold and VAD never saw any silence. 800k corresponds to
    //     RMS amplitude ~900 / int16 max ~32k, i.e. roughly -31 dBFS —
    //     still well below normal speech (~-15 dBFS) but above quiet rooms
    //     (~-50 dBFS) and AEC settling residual.
    //   silence_ms_to_end = 500 — half second of below-threshold audio ends
    //     the utterance. Was 1000; faster cut-off feels markedly snappier
    //     and matches what Alexa/Google use.
    //   no_speech_ms_to_end = 5000 — if the wake word fires and no command
    //     speech follows, close LISTENING without posting empty audio to STT.
    //   max_utterance_ms unchanged at 15 s — hard ceiling.
    vad_config_t vcfg = {
        .energy_threshold = VOICE_ENERGY_THRESHOLD,
        .silence_ms_to_end = 500,
        .no_speech_ms_to_end = 5000,
        .max_utterance_ms = 15000,
        .sample_rate = 16000,
    };
    s_vad = vad_create(&vcfg);

    s_in_utterance = false;
    // Log-once guard for the capture-buffer saturation warning below —
    // without it a saturated utterance would warn on every remaining frame.
    bool capture_sat_logged = false;

    // Wait-one-frame slot arbitration. When two wake-word models cover
    // overlapping phrases ("hey korra" vs "hey computer") the first slot to
    // fire is not necessarily the better match — feeding all slots, then
    // holding the wake decision for one additional 80 ms frame, lets the
    // second slot get a chance to fire too. We then commit to whichever
    // had the higher avg probability. Adds ~80 ms wake latency in exchange
    // for not mis-routing overlapping wake-words to the wrong account.
    int     pending_slot = -1;
    uint8_t pending_prob = 0;
    int     pending_age  = 0;
    char    pending_wake_slug[WW_WAKE_SLUG_MAX] = "";

    // Wake-word cutoff is CONSTANT — we do NOT lower it during playback.
    // The old playback-aware drop (-30) made first-try barge-in easier, but it
    // also let the device's OWN TTS reply bleed into the mic and false-trigger
    // wakes on other users' slots, kicking off spurious turns that kept ambient
    // paused. Decision 2026-06-22: keep the manifest cutoff no matter what.
    // (s_default_cutoff is still maintained by apply_ww_upload for the
    // cutoff-persist hot-swap path; we just never deviate from it here.)
    for (uint8_t i = 0; i < WW_NUM_SLOTS; ++i) {
        if (s_ww[i]) s_default_cutoff[i] = wakeword_get_default_cutoff(s_ww[i]);
    }

    while (1) {
        if (leds_buttons_is_muted()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        size_t n = audio_io_read_frame(frame, WW_FRAME_SAMPLES, 200);
        if (n < WW_FRAME_SAMPLES) continue;

        // Keep the pre-roll ring warm while not capturing, so a follow-up
        // VAD-start can prepend the speech onset it (by definition) missed.
        // Reset at every window-arm point, so by the time a window is open
        // this only ever holds post-reply room audio, never our own TTS.
        if (!s_in_utterance) preroll_append(frame, n);

        // Resume paused ambient once the turn is COMPLETELY over: not capturing
        // a command, not waiting on a reply, no TTS streaming/queued, and no
        // follow-up window open. This single check covers every turn-end path
        // (reply spoken, STT failed, empty reply, watchdog) — they all land
        // back here at full idle, where the rain should pick back up.
        if (s_ambient_paused && !s_verify_playback_hold_active &&
            !s_in_utterance && !s_awaiting_reply && !s_stream_active &&
            get_followup_until_us() == 0 &&
            !alarm_is_firing() &&
            uxQueueMessagesWaiting(s_sentence_q) == 0 && token_accum_empty()) {
            ambient_resume();
        }

        // THINKING gate: don't run wake inference between VAD-end and TTS
        // start. Without this, the mic stays hot while we wait on STT + LLM,
        // and any model with a non-trivial false-positive rate (notably the
        // stock hey_computer) re-arms LISTENING on near-silence — see the
        // 28754 ms entry in the 2026-05-15 trace. Resumes feeding the moment
        // tts_worker_task transitions to SPEAKING.
        if (s_awaiting_reply && !s_in_utterance) {
            // Watchdog: a normal reply clears s_awaiting_reply (TTS begin /
            // chat done / STT fail). If the WS dropped mid-turn or the server
            // went silent, nothing clears it and this gate would deafen the
            // device to every wake until reboot. After a bounded wait, give up
            // and re-open the wake feed.
            int64_t now_us = esp_timer_get_time();
            int64_t awaiting_since = get_awaiting_since_us();
            if (awaiting_since != 0 &&
                now_us - awaiting_since > (int64_t) AWAITING_REPLY_TIMEOUT_MS * 1000) {
                ESP_LOGW(TAG, "awaiting-reply watchdog fired (%dms, no reply) — re-opening wake feed",
                         AWAITING_REPLY_TIMEOUT_MS);
                s_awaiting_reply = false;
                set_awaiting_since_us(0);
                s_followup_pending_ms = 0;   // reply died — no window to defer
                set_ui_state(UI_STATE_IDLE);
                airplay_resume();
                // fall through — resume wake inference this frame
            } else {
                // Drain the frame so it doesn't pile up in the I²S DMA buffer,
                // but skip inference. pending_* state is irrelevant here — any
                // mid-flight pending was committed or expired before the gate.
                pending_slot = -1; pending_prob = 0; pending_age = 0;
                pending_wake_slug[0] = '\0';
                continue;
            }
        }

        if (!s_in_utterance) {
            // Set when THIS frame's fire came from the follow-up VAD-start
            // path — the only fire type that prepends pre-roll (a wake-word
            // fire must NOT: pre-roll would include the wake phrase itself).
            bool fired_from_followup = false;
            // Idle-room noise floor for the follow-up trigger. Learn only
            // while the mic is hot and we're not hearing ourselves: during
            // playback amp_en is HIGH and the XVF AEC reads the whole room
            // ~250× quiet — folding those frames in would crater the floor
            // right before a window opens on a hot mic.
            uint32_t fe_room = frame_energy(frame, n);
            if (!s_stream_active && !audio_io_playback_active() &&
                fe_room < VOICE_ENERGY_THRESHOLD) {
                s_room_floor = s_room_floor
                    ? s_room_floor - s_room_floor / 8 + fe_room / 8 : fe_room;
            }
            // Follow-up listening: if the server armed a window after its
            // last reply ended with a "?", treat any voice activity in this
            // frame as a wake fire so the user can answer without saying
            // the wake word again. Window expires silently if no speech detected.
            int64_t followup_until = get_followup_until_us();
            if (followup_until > 0) {
                int64_t now_us = esp_timer_get_time();
                uint64_t ftrig = (uint64_t)s_room_floor * FOLLOWUP_FLOOR_MULT;
                if (ftrig < FOLLOWUP_TRIGGER_ENERGY) ftrig = FOLLOWUP_TRIGGER_ENERGY;
                if (now_us >= followup_until) {
                    set_followup_until_us(0);
                    // Window expired without speech — drop back to IDLE. The
                    // floor/trig line is the tuning signal for the deaf-window
                    // failure mode ("I answered and it ignored me"): it shows
                    // the bar the answer needed to beat.
                    set_ui_state(UI_STATE_IDLE);
                    char fl[96];
                    snprintf(fl, sizeof(fl), "[followup] window expired floor=%u trig=%u",
                             (unsigned)s_room_floor, (unsigned)ftrig);
                    ESP_LOGI(TAG, "%s", fl); oe_udplog_send(fl);
                } else if ((uint64_t)fe_room > ftrig &&
                           doa_gate_allows("followup")) {
                    // Direction gate on the fire (not the window): a rejected
                    // frame leaves the window ARMED. If it's really the user
                    // answering from a new spot, the beamformer re-tasks onto
                    // them within a probe tick or two and a later frame
                    // passes — the pre-roll prepend covers the delayed onset.
                    // Speakers at their fixed bearing just never fire it.
                    char fl[112];
                    snprintf(fl, sizeof(fl), "[followup] fire fe=%u floor=%u trig=%u slot=%u",
                             (unsigned)fe_room, (unsigned)s_room_floor, (unsigned)ftrig,
                             (unsigned)s_followup_slot);
                    ESP_LOGI(TAG, "%s", fl); oe_udplog_send(fl);
                    s_active_slot = s_followup_slot;
                    set_followup_until_us(0);
                    // Fall through into the wake-fire actions below by
                    // emulating a committed pending_slot. The block right
                    // after sets fired/in_utterance based on this state.
                    pending_slot = s_followup_slot;
                    pending_prob = 255;
                    pending_age = 1;
                    pending_wake_slug[0] = '\0';
                    fired_from_followup = true;
                }
            }

            // ── Speech barge-in (conversation mode): pause-then-verify ──────
            // Runs only while OUR streamed reply plays. Ambient/AirPlay keep
            // the wake word as their barge path — their audio isn't a
            // conversation. See the barge_state_t block at file scope for the
            // three-stage design (candidate → local verify → STT commit).
            if (s_conversation_mode && s_stream_active &&
                server_control_may_open_ungated_capture()) {
                if (s_barge_state == BARGE_NONE && !s_paused_for_barge) {
                    uint32_t fe = frame_energy(frame, n);
                    // Rolling floor: EMA (α=1/8) of the reply+room energy as
                    // heard by the AEC-suppressed mic. The trigger is
                    // relative to it so pre-existing noise (TV) is absorbed
                    // into the baseline. CRITICAL: post-warmup, only
                    // SUB-TRIGGER frames feed the EMA — folding the loud
                    // frames of a building streak into the floor let the
                    // trigger outrun the user's own voice by frame 3, and no
                    // candidate ever fired (the 0.2.66 "can't barge in at
                    // all" regression). Classic noise-floor rule: the signal
                    // you're trying to detect must not shape the baseline.
                    // Warm-up: no candidates while the floor is still learning
                    // this reply's level (unconditional EMA there) — seeding
                    // from the quiet lead-in made the first loud TTS peaks
                    // look like interjections. NOTE: no `continue` anywhere
                    // here — the wake-word feed below must keep running.
                    uint64_t trigger = (uint64_t)s_speak_floor * BARGE_FLOOR_MULT;
                    if (trigger < BARGE_TRIGGER_MIN) trigger = BARGE_TRIGGER_MIN;
                    if (s_barge_warmup < BARGE_WARMUP_FRAMES) {
                        s_speak_floor = s_speak_floor
                            ? s_speak_floor - s_speak_floor / 8 + fe / 8 : fe;
                        s_barge_warmup++;
                        s_barge_consec = 0;
                    } else if ((uint64_t)fe > trigger &&
                               esp_timer_get_time() >= s_barge_cooldown_until_us) {
                        if (++s_barge_consec >= BARGE_CONSEC_FRAMES) {
                            s_barge_consec = 0;
                            // Direction gate: a candidate whose fresh beams
                            // all point away from the turn's talker is the
                            // speakers, not an interjection — skip it with
                            // ZERO audible cost (no pause, no stutter). The
                            // cooldown keeps sustained off-axis audio from
                            // re-running the gate every 3rd frame, and its
                            // frames land in the else-branch below, feeding
                            // the floor like any other cooldown audio.
                            if (!doa_gate_allows("barge")) {
                                s_barge_cooldown_until_us = esp_timer_get_time()
                                    + DOA_GATE_REJECT_COOLDOWN_US;
                                // NO continue (wake feed below must still run
                                // this frame) — just decline the candidate.
                            } else {
                                // Mute FIRST (local, ~instant): pausing
                                // playback + dropping amp_en is what
                                // un-suppresses the mic for the verify. Then
                                // stall the server pacer. (The WS send can
                                // block up to 1 s worst-case on this task —
                                // the audible pause already happened, and
                                // verify timing keys off s_barge_started_us.)
                                audio_io_pause_playback();
                                xvf3800_enable_amplifier(false);
                                s_paused_for_barge = true;
                                s_barge_state = BARGE_VERIFYING;
                                s_barge_started_us = esp_timer_get_time();
                                s_barge_verify_frames = 0;
                                s_barge_speech_ms = 0;
                                set_ui_state(UI_STATE_LISTENING);
                                if (s_caps_tts_pause) oe_ws_send_tts_pause(s_turn_id);
                                char bl[96];
                                snprintf(bl, sizeof(bl), "[barge] candidate fe=%u floor=%u",
                                         (unsigned)fe, (unsigned)s_speak_floor);
                                ESP_LOGI(TAG, "%s", bl); oe_udplog_send(bl);
                            }
                        }
                    } else {
                        // Sub-trigger frame (or cooldown): this is what shapes
                        // the floor. Sustained loud noise during a cooldown
                        // also lands here, so the baseline absorbs it instead
                        // of re-candidating the moment cooldown expires.
                        s_speak_floor = s_speak_floor
                            ? s_speak_floor - s_speak_floor / 8 + fe / 8 : fe;
                        s_barge_consec = 0;
                    }
                } else if (s_barge_state == BARGE_VERIFYING) {
                    int64_t now_us = esp_timer_get_time();
                    s_barge_verify_frames++;
                    // Skip the first verify frame: the mic gain steps up when
                    // amp_en drops and it reads artificially hot/cold.
                    if (s_barge_verify_frames > BARGE_SETTLE_FRAMES && frame_is_speech(frame, n)) {
                        s_barge_speech_ms += (uint32_t)((n * 1000) / 16000);
                    }
                    if (s_barge_speech_ms >= BARGE_CONFIRM_SPEECH_MS) {
                        // Confirmed speech — capture it as an utterance. The
                        // paused reply is DELIBERATELY kept (not flushed):
                        // stage C below only kills it if STT proves the
                        // interjection real, so a cough costs a short pause,
                        // never the rest of the answer.
                        oe_udplog_send("[barge] speech confirmed — capturing");
                        s_barge_state = BARGE_NONE;
                        s_barge_capture = true;
                        vad_reset(s_vad);
                        // Prepend everything from just BEFORE the candidate
                        // (BARGE_PREROLL_LEAD_MS covers the trigger frames +
                        // the sub-threshold onset ramp) through now. Field
                        // data: the fixed 400 ms prepend lost the first word
                        // ("Who started…" → "started…") because onset→confirm
                        // spans ~700 ms. Reaching further back than the lead
                        // would mostly add our own reply bleed — the server
                        // gets a `barge` flag instead so its stop-matcher
                        // tolerates a bled prefix.
                        {
                            size_t want = ((size_t)s_barge_verify_frames * 80 + BARGE_PREROLL_LEAD_MS) * 16;
                            if (want > PREROLL_SAMPLES) want = PREROLL_SAMPLES;
                            s_capture_used = preroll_copy_out(s_capture_buf, want);
                        }
                        capture_sat_logged = false;
                        s_in_utterance = true;
                        // Slot stays s_active_slot (the turn owner). Turn id
                        // is minted at stage-C commit; until then pause/
                        // resume flow control still names the CURRENT turn.
                    } else if (s_barge_verify_frames >= BARGE_VERIFY_WINDOW_FRAMES) {
                        // False alarm — resume the reply where it paused.
                        //
                        // TODO(short-word barge): a single bare "stop" lands
                        // here as "0ms speech" — the word ENDS right as the
                        // candidate triggers, so the verify window opens onto
                        // silence and the reply resumes (field-confirmed
                        // 2026-07-04). The word itself is already sitting in
                        // the pre-roll ring at this point: instead of
                        // instantly resuming, ship the pre-roll snippet to
                        // STT and let the transcript decide (same contract as
                        // stage C). Cost: TV-noise false candidates hold the
                        // pause ~1.5s instead of 1s. Deferred — multi-word
                        // phrases ("that's enough", "you can stop") and
                        // "<wake> stop" cover it today.
                        char bl[64];
                        snprintf(bl, sizeof(bl), "[barge] false alarm (%ums speech) — resume",
                                 (unsigned)s_barge_speech_ms);
                        ESP_LOGI(TAG, "%s", bl); oe_udplog_send(bl);
                        s_barge_state = BARGE_NONE;
                        s_paused_for_barge = false;
                        s_barge_cooldown_until_us = now_us + (int64_t)BARGE_FALSE_ALARM_COOLDOWN_MS * 1000;
                        xvf3800_enable_amplifier(true);
                        audio_io_resume_playback();
                        set_ui_state(UI_STATE_SPEAKING);
                        if (s_caps_tts_pause) oe_ws_send_tts_resume(s_turn_id);
                    }
                }
            } else if (s_barge_state != BARGE_NONE || s_paused_for_barge || s_speak_floor != 0) {
                // Stream ended (teardown, server pause-abort, mode toggle)
                // while the machine was engaged — reset, and critically
                // release OUR playback pause: a stranded s_paused flag would
                // silently stall every future TTS reply (ring fills, drops).
                s_barge_state = BARGE_NONE;
                s_barge_consec = 0;
                s_speak_floor = 0;
                s_barge_warmup = 0;   // next reply re-learns its floor first
                s_barge_cooldown_until_us = 0;
                if (s_paused_for_barge) {
                    s_paused_for_barge = false;
                    audio_io_resume_playback();
                }
            }

            // Feed every loaded slot. Track this frame's highest-prob wake,
            // then merge into a (slot, prob) pending decision that we commit
            // one frame later — see comment by pending_slot above.
            int     frame_best_slot = -1;
            uint8_t frame_best_prob = 0;
            char    frame_best_wake_slug[WW_WAKE_SLUG_MAX] = "";
            for (uint8_t i = 0; i < WW_NUM_SLOTS; ++i) {
                if (!s_ww[i]) continue;
                // While the device is speaking its OWN reply (streamed TTS),
                // only the slot that owns this turn may barge in. A different
                // user's wake word (user B's during user A's turn) must not
                // interrupt, and — since the reply bleeds into the mic — this
                // is also what stops the other model from self-triggering on
                // the reply voice. The owning slot stays live so the asker can
                // still interrupt. (Ambient/idle: s_stream_active is false, so
                // every slot is fed normally.)
                if (s_stream_active && (int)i != (int)s_active_slot) continue;
                if (wakeword_feed(s_ww[i], frame, n)) {
                    uint8_t p = wakeword_last_wake_prob(s_ww[i]);
                    if (frame_best_slot < 0 || p > frame_best_prob) {
                        frame_best_slot = (int) i;
                        frame_best_prob = p;
                        if (!wakeword_last_wake_slug(
                                s_ww[i], frame_best_wake_slug,
                                sizeof(frame_best_wake_slug))) {
                            frame_best_wake_slug[0] = '\0';
                        }
                    }
                }
            }
            if (frame_best_slot >= 0 &&
                (pending_slot < 0 || frame_best_prob > pending_prob)) {
                pending_slot = frame_best_slot;
                pending_prob = frame_best_prob;
                snprintf(pending_wake_slug, sizeof(pending_wake_slug), "%s",
                         frame_best_wake_slug);
            }

            bool fired = false;
            if (pending_slot >= 0) {
                if (pending_age >= 1) {
                    // Held one full frame past first fire; commit.
                    s_prov_prev_slot = s_active_slot;
                    s_prov_prev_prob = s_active_wake_prob;
                    snprintf(s_prov_prev_slug, sizeof(s_prov_prev_slug), "%s",
                             s_active_wake_slug);
                    s_prov_prev_followup_until_us = get_followup_until_us();
                    s_prov_identity_valid = true;
                    s_active_slot = (uint8_t) pending_slot;
                    s_active_wake_prob = pending_prob;
                    snprintf(s_active_wake_slug, sizeof(s_active_wake_slug), "%s",
                             pending_wake_slug);
                    fired = true;
                    // If a follow-up window is active, lock the slot back
                    // to the originating turn — a false-fire (or even a
                    // genuine fire) on a different wake-word during the
                    // window should still route the answer to the user
                    // who asked the question.
                    if (s_prov_prev_followup_until_us > esp_timer_get_time() &&
                        s_active_slot != s_followup_slot) {
                        ESP_LOGI(TAG, "follow-up: slot %u wake fired, overriding to slot %u",
                                 (unsigned) s_active_slot, (unsigned) s_followup_slot);
                        s_active_slot = s_followup_slot;
                    }
                    set_followup_until_us(0);  // window closes once we commit
                    pending_slot = -1; pending_prob = 0; pending_age = 0;
                    pending_wake_slug[0] = '\0';
                } else {
                    pending_age++;
                }
            }

            if (fired) {
                // Follow-up VAD, speech-barge captures, and a wake used only to
                // dismiss a locally-ringing alarm are intentional non-command
                // paths. Every other detector fire is governed by the
                // configured gate. Evaluate readiness before any LED/playback/
                // WS side effect so a blocked fire remains invisible.
                bool local_alarm_dismiss = alarm_is_firing();
                bool vg_on = false;
                if (fired_from_followup &&
                    !server_control_may_open_ungated_capture()) {
                    ESP_LOGW(TAG,
                             "[verify] untrusted follow-up capture dropped");
                    vg_restore_rejected_identity();
                    continue;
                }
                if (!fired_from_followup && !local_alarm_dismiss) {
                    const char *blocked_reason = NULL;
                    verify_gate_state_t vg_state =
                        verify_gate_state(&blocked_reason);
                    if (vg_state == VG_READY) {
                        char wake_words_probe[WW_WAKE_SLUG_MAX + 5];
                        if (!vg_build_wake_words(
                                s_active_wake_slug, wake_words_probe,
                                sizeof(wake_words_probe))) {
                            vg_state = VG_BLOCKED;
                            blocked_reason = "invalid_wake_slug";
                        }
                    }
                    if (vg_state == VG_BLOCKED) {
                        char line[96];
                        snprintf(line, sizeof(line),
                                 "[verify] blocked (%s) — fire dropped",
                                 blocked_reason ? blocked_reason : "not_ready");
                        ESP_LOGW(TAG, "%s", line);
                        oe_udplog_send(line);
                        vg_restore_rejected_identity();
                        continue;
                    }
                    vg_on = vg_state == VG_READY;
                }

                if (vg_on) {
                    mint_turn_id_into(s_prov_turn_id,
                                      sizeof(s_prov_turn_id));
                    if (!vg_playback_hold_begin()) {
                        oe_udplog_send("[verify] reversible hold unavailable — fire dropped");
                        s_prov_turn_id[0] = 0;
                        vg_restore_rejected_identity();
                        continue;
                    }
                }

                // A wake fire supersedes any speech-barge verify in flight:
                // the ungated path releases it before destructive cleanup.
                // The gated path transfers it inside vg_playback_hold_begin()
                // only after the independent gate hold owns the engine.
                if (!vg_on && s_paused_for_barge) {
                    s_paused_for_barge = false;
                    s_barge_state = BARGE_NONE;
                    s_barge_capture = false;
                    audio_io_resume_playback();
                }
                if (!vg_on) s_wait_led_until_us = 0;
                // Visual ack FIRST. Everything below this point — barge-in
                // cleanup, WS stop send, ringbuffer flush — has variable
                // latency (the WS send is the main offender, ~50-500 ms
                // depending on socket state). Flipping the LED before any of
                // it gives the user immediate "we heard you" feedback so
                // they're not left staring at IDLE while we tidy up. The
                // alarm-dismiss branch below overrides this back to a
                // short LISTENING-then-IDLE flash on its own.
                //
                // With the verify gate active we DEFER this ack until the gate
                // returns an explicit accept (vg_provisional_step), so a
                // rejected/error fire dies invisibly. Alarm dismiss still
                // flashes below.
                if (!vg_on) set_ui_state(UI_STATE_LISTENING);

                // Alarm dismiss takes precedence over normal wake flow: if
                // any alarm is currently firing, treat the wake as a local
                // ack — stop the ring, send alarm_acked, skip STT/utterance
                // capture for this wake. No STT roundtrip means dismiss
                // still works when the server is unreachable.
                if (local_alarm_dismiss) {
                    // Alarm dismiss is a confirmed LOCAL action (works offline)
                    // and is NOT gated — show the LISTENING flash now even when
                    // the verify gate deferred the ack above.
                    if (s_ambient_active) {
                        s_ambient_paused = true;
                        oe_udplog_send("[ambient] PAUSE (alarm dismiss)");
                    }
                    if (audio_io_playback_active()) {
                        audio_io_stop_playback();
                        audio_io_flush_playback();
                    }
                    alarm_handle_local_dismiss();
                    // 500 ms is long enough to register visually, short
                    // enough to feel "instant."
                    vTaskDelay(pdMS_TO_TICKS(500));
                    set_ui_state(UI_STATE_IDLE);
                    s_prov_identity_valid = false;
                    continue;
                }

                // Anchor the direction gate to this turn's talker. Only a
                // WAKE fire may move the anchor — the wake phrase is
                // proof-of-user, while follow-up fires reuse the previous
                // anchor and barge captures never touch it. Fresh blocking
                // read (≤~50 ms) rather than the cache: at idle cadence the
                // cache can be ~2 s stale, which would fail the anchor open
                // in exactly the music-playing case the gate exists for.
                // The WS stop send just below costs up to 500 ms, so this
                // adds nothing perceptible.
                if (!fired_from_followup) {
                    float anchor_az[4];
                    uint8_t anchor_st = 0;
                    if (doa_read4(XVF_CMD_AEC_AZIMUTH_VALUES, anchor_az, &anchor_st)) {
                        taskENTER_CRITICAL(&s_doa_mux);
                        memcpy(s_doa_az, anchor_az, sizeof(anchor_az));
                        s_doa_fresh_us = esp_timer_get_time();
                        if (!vg_on) s_turn_bearing = anchor_az[3];
                        taskEXIT_CRITICAL(&s_doa_mux);
                        if (vg_on) {
                            s_prov_bearing = anchor_az[3];
                            s_prov_bearing_pending = true;
                        }
                        char al[96];
                        snprintf(al, sizeof(al),
                                 "[doagate] %sbearing=%.2f (wake)",
                                 vg_on ? "provisional " : "turn ",
                                 (double)anchor_az[3]);
                        ESP_LOGI(TAG, "%s", al); oe_udplog_send(al);
                    } else {
                        // No fresh localization of the wake phrase — don't
                        // reuse a stale anchor from an earlier turn; run
                        // this conversation ungated (0.2.74 behavior).
                        if (vg_on) {
                            s_prov_bearing = -1.0f;
                            s_prov_bearing_pending = true;
                        } else {
                            taskENTER_CRITICAL(&s_doa_mux);
                            s_turn_bearing = -1.0f;
                            taskEXIT_CRITICAL(&s_doa_mux);
                        }
                        oe_udplog_send("[doagate] no fresh bearing at wake — gate open this turn");
                    }
                }
                if (!vg_on && audio_io_playback_active()) {
                    ESP_LOGI(TAG, "barge-in");
                    // Wake-during-ambient: PAUSE the ambient (don't tear it
                    // down). The single ambient task keeps its HTTP stream open
                    // and reading; oe_tts.c discards the audio while paused so
                    // the speaker is free for the command/reply. We flush the
                    // queued ambient PCM below so the rain stops instantly, then
                    // the capture loop resumes it once the turn is fully idle.
                    if (s_ambient_active) {
                        s_ambient_paused = true;
                        oe_udplog_send("[ambient] PAUSE (wake)");
                    }
                    // Pause AirPlay (if any) for the duration of the user's
                    // utterance + reply. RTSP session stays open so iOS
                    // resumes from the same spot after airplay_resume().
                    airplay_pause();
                    audio_io_stop_playback();
                    audio_io_flush_playback();
                    // Cancel any in-flight server-pushed TTS stream so the
                    // finalize task doesn't re-enter SPEAKING after barge-in.
                    // The server halts the push on the stop below.
                    s_stream_active  = false;
                    s_stream_end_req = false;
                    s_stream_turn_id[0] = 0;
                    oe_ws_send_stop(g_dev_config.default_agent_id, s_turn_id);
                    // Drain queued TTS sentences and partial-token accumulator.
                    // Without this, tts_worker_task will pull whatever was
                    // buffered from the just-aborted reply and start speaking
                    // it again right after the user's new utterance — making
                    // barge-in look broken (the original reply keeps going).
                    xQueueReset(s_sentence_q);
                    reset_token_accum();
                }
                // New turn starts here. Mint AFTER the barge-in block above —
                // its stop frame must carry the OLD turn's id (the turn being
                // stopped), not this new one's.
                if (!vg_on) {
                    mint_turn_id();
                    s_prov_identity_valid = false;
                }
                vad_reset(s_vad);
                s_capture_used = 0;
                capture_sat_logged = false;
                if (fired_from_followup) {
                    // Prepend the pre-roll (oldest→newest) so the onset of the
                    // answer — which necessarily happened BEFORE this frame
                    // crossed the energy threshold — reaches STT instead of
                    // being clipped. Buffer headroom is guaranteed: capture is
                    // 16 s for a 15 s VAD ceiling, pre-roll is 0.4 s.
                    s_capture_used = preroll_copy_out(s_capture_buf, PREROLL_SAMPLES);
                }
                if (vg_on) {
                    // PROVISIONAL: snapshot the [fire−2.0 s, fire] slice from
                    // the now-frozen pre-roll ring (preroll_append stops once
                    // s_in_utterance is set below). The +0.5 s tail fills over
                    // the next frames in the capture branch, then we POST +
                    // await the verdict. No LED, no stt_begin until commit.
                    s_verify_pre_len = preroll_copy_out(s_verify_wav, VG_PRE_SAMPLES);
                    s_verify_win_len = s_verify_pre_len;
                    s_prov_vad_end = VAD_END_NONE;
                    s_prov_state = PROV_FILL_TAIL;
                    oe_udplog_send("[verify] provisional — capturing wake window");
                } else {
                    // Gate explicitly disabled, or intentional follow-up/alarm
                    // bypass: original immediate path.
                    // Streaming STT: open the server-side session and ship any
                    // pre-rolled onset immediately. Failure at any point just
                    // falls back to the buffered HTTP path at VAD-end.
                    s_stt_streaming = s_caps_stt_stream && oe_ws_connected();
                    s_stt_send_failed = false;
                    s_stt_seq = 0;
                    if (s_stt_streaming) {
                        if (oe_ws_send_stt_begin(s_turn_id, s_active_slot,
                                                 s_active_wake_prob,
                                                 g_dev_config.default_agent_id) != ESP_OK) {
                            s_stt_streaming = false;
                        } else {
                            for (size_t off = 0; off < s_capture_used && !s_stt_send_failed; off += WW_FRAME_SAMPLES) {
                                size_t chunk = s_capture_used - off;
                                if (chunk > WW_FRAME_SAMPLES) chunk = WW_FRAME_SAMPLES;
                                if (oe_ws_send_stt_frame(s_capture_buf + off, chunk, s_stt_seq++) != ESP_OK) {
                                    s_stt_send_failed = true;
                                }
                            }
                        }
                    }
                }
                s_in_utterance = true;
            }
        } else {
            // Provisional (verify gate): keep buffering the command locally but
            // run VAD locally while holding ack/STT until the gate verdict.
            // If the command already ended, an accept returns its latched
            // boundary and falls directly into the normal finalizer below.
            vad_end_reason_t end_reason = VAD_END_NONE;
            if (s_prov_state != PROV_NONE) {
                end_reason = vg_provisional_step(frame, n);
                if (end_reason == VAD_END_NONE) continue;
            } else {
                if (s_capture_used + n < CAPTURE_BUFFER_SAMPLES) {
                    memcpy(s_capture_buf + s_capture_used, frame, n * sizeof(int16_t));
                    s_capture_used += n;
                    // Streaming STT: ship this frame now so the upload overlaps
                    // the user's speech. One failure flips to the HTTP fallback
                    // for the rest of the utterance (buffer keeps accumulating).
                    if (s_stt_streaming && !s_stt_send_failed) {
                        if (oe_ws_send_stt_frame(frame, n, s_stt_seq++) != ESP_OK) {
                            s_stt_send_failed = true;
                            oe_udplog_send("[stt] frame send failed — buffered HTTP fallback armed");
                        }
                    }
                } else if (!capture_sat_logged) {
                    // Saturated mid-utterance: STT will get a truncated question.
                    // Should be unreachable now that the buffer (16 s) exceeds
                    // the VAD ceiling (max_utterance_ms 15 s) — log loudly if it
                    // ever happens instead of silently cutting the user off.
                    ESP_LOGW(TAG, "capture buffer full at %u samples — utterance tail dropped",
                             (unsigned)s_capture_used);
                    capture_sat_logged = true;
                }
                vad_feed(s_vad, frame, n, &end_reason);
            }
            if (end_reason != VAD_END_NONE) {
                s_in_utterance = false;

                if (end_reason == VAD_END_NO_SPEECH) {
                    if (s_stt_streaming) {
                        // Nothing worth transcribing — tell the server to
                        // drop the accumulated session (it would TTL out
                        // anyway; this is just prompt cleanup).
                        s_stt_streaming = false;
                        oe_ws_send_stt_abort(s_turn_id);
                    }
                    if (s_barge_capture) {
                        // Confirmed energy but no sustained speech followed —
                        // resume the paused reply where it left off.
                        s_barge_capture = false;
                        oe_udplog_send("[barge] no speech after confirm — resuming reply");
                        s_paused_for_barge = false;
                        s_barge_cooldown_until_us = esp_timer_get_time() + (int64_t)BARGE_FALSE_ALARM_COOLDOWN_MS * 1000;
                        xvf3800_enable_amplifier(true);
                        audio_io_resume_playback();
                        set_ui_state(UI_STATE_SPEAKING);
                        if (s_caps_tts_pause) oe_ws_send_tts_resume(s_turn_id);
                        s_capture_used = 0;
                        continue;
                    }
                    ESP_LOGI(TAG, "wake capture ended after %ums with no speech; skipping STT",
                             (unsigned)vad_elapsed_ms(s_vad));
                    oe_udplog_send("[voice] no speech after wake; skipping STT");
                    s_capture_used = 0;
                    set_ui_state(UI_STATE_IDLE);
                    airplay_resume();
                    continue;
                }

                set_ui_state(UI_STATE_THINKING);
                // Close the wake-word feed until SPEAKING starts (or the
                // reply path errors back to IDLE) — see s_awaiting_reply.
                s_awaiting_reply = true;
                set_awaiting_since_us(esp_timer_get_time());  // arm watchdog

                // Streaming STT hand-off: every frame already reached the
                // server; stt_end makes IT transcribe + dispatch the turn.
                // The reply then arrives over the normal token/tts events
                // (guarded by the same 90 s THINKING watchdog). Any earlier
                // send failure — or a failed stt_end itself — falls through
                // to the buffered HTTP path below; the server side of the
                // half-sent session just TTLs out.
                if (s_stt_streaming) {
                    bool stream_clean = !s_stt_send_failed;
                    s_stt_streaming = false;
                    if (stream_clean &&
                        oe_ws_send_stt_end(s_turn_id, (uint32_t)s_capture_used) == ESP_OK) {
                        s_capture_used = 0;
                        continue;
                    }
                    if (!stream_clean) {
                        // A bounded backlog/live-frame failure may have left a
                        // partial server session. Drop it before posting the
                        // complete local buffer so it cannot linger to TTL.
                        oe_ws_send_stt_abort(s_turn_id);
                    }
                    oe_udplog_send("[stt] stream fallback — posting buffered utterance");
                }

                char transcript[512] = {0};
                bool stt_ok = oe_stt_post(g_dev_config.server_url, g_dev_config.token,
                                          s_capture_buf, s_capture_used,
                                          transcript, sizeof(transcript)) == ESP_OK &&
                              transcript[0];
                const bool barge = s_barge_capture;
                s_barge_capture = false;

                if (barge && (!stt_ok || transcript_is_filler(transcript))) {
                    // Stage C says NOT a real interjection (empty transcript /
                    // vocal tic) — the reply we kept paused resumes intact.
                    // Total cost of the false positive: a few seconds' pause.
                    char bl[96];
                    snprintf(bl, sizeof(bl), "[barge] not real (\"%.32s\") — resuming reply",
                             stt_ok ? transcript : "");
                    ESP_LOGI(TAG, "%s", bl); oe_udplog_send(bl);
                    s_awaiting_reply = false;
                    set_awaiting_since_us(0);
                    s_paused_for_barge = false;
                    s_barge_cooldown_until_us = esp_timer_get_time() + (int64_t)BARGE_FALSE_ALARM_COOLDOWN_MS * 1000;
                    xvf3800_enable_amplifier(true);
                    audio_io_resume_playback();
                    set_ui_state(UI_STATE_SPEAKING);
                    if (s_caps_tts_pause) oe_ws_send_tts_resume(s_turn_id);
                    s_capture_used = 0;
                    continue;
                }

                if (stt_ok) {
                    if (barge) {
                        // Stage-C commit: the interjection is real — NOW kill
                        // the paused reply and stop its turn server-side. The
                        // stop names the OLD turn; the chat below carries a
                        // freshly minted one.
                        oe_udplog_send("[barge] interjection real — interrupting reply");
                        s_paused_for_barge = false;
                        audio_io_resume_playback();   // release pause flag before stop
                        audio_io_stop_playback();
                        audio_io_flush_playback();
                        s_stream_active  = false;
                        s_stream_end_req = false;
                        s_stream_turn_id[0] = 0;
                        for (uint8_t _i = 0; _i < WW_NUM_SLOTS; ++_i)
                            if (s_ww[_i]) wakeword_notify_speaking_ended(s_ww[_i]);
                        oe_ws_send_stop(g_dev_config.default_agent_id, s_turn_id);
                        mint_turn_id();
                        // amp stays off (dropped at the pause); the next
                        // tts_audio_begin re-enables it for the new reply.
                    }
                    // Final-second cleanup: tokens from the just-aborted
                    // chat can keep arriving for up to ~2 s after barge-in
                    // (server's WS send buffer + Node's net layer). Without
                    // this clear, those stale tokens get concatenated with
                    // the new reply ("Inside was a" + "okay." → "Inside was
                    // aokay."). Clearing right before send means anything
                    // received from the prior chat is discarded; the new
                    // chat's tokens populate a clean accumulator.
                    xQueueReset(s_sentence_q);
                    reset_token_accum();
                    // s_active_slot was set when this utterance's wake word
                    // fired. Server uses it to look up slot_agent_map[N].
                    esp_err_t ce = oe_ws_send_chat(g_dev_config.default_agent_id, transcript,
                                                   s_active_slot, s_active_wake_prob, s_turn_id, barge);
                    if (ce != ESP_OK) {
                        // Send failed (WS send timeout / socket flapping):
                        // no reply is coming, so re-open the wake feed NOW.
                        // Ignoring this return used to leave the THINKING
                        // gate armed for the full 90 s watchdog — and the
                        // user's utterance silently vanished with the UI
                        // stuck on THINKING.
                        ESP_LOGE(TAG, "chat send failed (%s) — turn dropped", esp_err_to_name(ce));
                        oe_udplog_send("[voice] chat send failed — turn dropped, back to IDLE");
                        s_awaiting_reply = false;
                        set_awaiting_since_us(0);
                        set_ui_state(UI_STATE_IDLE);
                        airplay_resume();
                    }
                } else {
                    // STT failed / empty transcript — no reply is coming, so
                    // re-open the wake-word feed immediately rather than
                    // leaving the mic gated until something clears the flag.
                    s_awaiting_reply = false;
                    set_awaiting_since_us(0);
                    set_ui_state(UI_STATE_IDLE);
                    airplay_resume();
                }
                s_capture_used = 0;
            }
        }
    }
}

// Background worker for alarm_arm: fetches the cached TTS MP3 from the
// server (one-shot marker → /api/tts → raw audio body) on its own task so
// the WS event loop doesn't stall on the HTTP roundtrip. Then arms the
// alarm. Caller heap-allocates the request struct; worker frees it.
// (Type + forward decl live above ws_event_cb so the dispatch can spawn
// this without a circular include.)
static void alarm_arm_worker(void *arg)
{
    alarm_arm_req_t *req = (alarm_arm_req_t *)arg;
    uint8_t *audio = NULL;
    size_t audio_len = 0;
    if (req->marker[0]) {
        esp_err_t fe = oe_alarm_fetch_audio(g_dev_config.server_url, g_dev_config.token,
                                             req->marker, &audio, &audio_len);
        if (fe != ESP_OK) {
            ESP_LOGW(TAG, "alarm audio fetch failed: %s — chime-only fallback",
                     esp_err_to_name(fe));
            audio = NULL; audio_len = 0;
        }
    }
    alarm_arm(req->id, req->label, req->trigger_at_ms, audio, audio_len, req->type);
    free(req);
    vTaskDelete(NULL);
}

// Ambient playback worker — streams the routine's looped MP3 from /api/tts
// through libhelix to audio_io until either the stop flag is set (wake fire
// or stop_ambient WS) or the HTTP stream EOFs (network blip / server drop).
//
// The server holds the response open with ffmpeg `-stream_loop -1`, so one
// oe_tts_post call covers the entire ambient session — no per-iteration
// re-fetch, no silent gap at loop seams. When the user says wake or stop,
// audio_io_stop_playback is called from elsewhere (capture_and_drive_task
// barge-in / OE_WS_EVT_STOP_AMBIENT handler); the bytes-already-in-flight
// drain silently, oe_tts_post eventually returns, this task exits.
static void ambient_pcm_cb(const int16_t *pcm, size_t samples, uint32_t rate, void *user)
{
    (void)user;
    if (s_ambient_stop || s_ambient_paused) return;  // stop/pause — drop PCM (http_evt already discards when paused; belt)

    // Lock to the first non-zero rate the decoder reports. After lock, every
    // PCM buffer plays at that rate regardless of what the decoder claims —
    // this filters out the corrupt-header misparses (22050/32000 reads of a
    // 44100 file caused by a flipped MPEG-version bit).
    if (s_ambient_stable_rate == 0 && rate > 0) {
        s_ambient_stable_rate = rate;
        s_last_ambient_rate = rate;
        ESP_LOGI(TAG, "ambient: rate locked at %u Hz", (unsigned)rate);
    }
    // Effective rate for this buffer:
    //   1. After lock: always the stable rate (decoder misparses ignored).
    //   2. Before lock, rate>0: use it + update cache.
    //   3. Before lock, rate=0: use cached seed (44100 by default).
    uint32_t effective_rate;
    if (s_ambient_stable_rate > 0) {
        effective_rate = s_ambient_stable_rate;
    } else if (rate > 0) {
        s_last_ambient_rate = rate;
        effective_rate = rate;
    } else {
        effective_rate = s_last_ambient_rate;
    }
    audio_io_write_pcm(pcm, samples, effective_rate);
}

// Helper: does the request queue have a newer ambient request waiting? Used to
// preempt the current session cleanly (loop back and pick up the newest).
static inline bool ambient_preempted(void)
{
    return s_ambient_req_q && uxQueueMessagesWaiting(s_ambient_req_q) > 0;
}

// THE single ambient task (created once at boot). Blocks idle on its request
// queue, plays one marker at a time, and switches markers / stops only between
// sessions — so there is never more than one ambient fetcher touching
// s_ambient_active / s_ambient_stop / audio_io. play_ambient enqueues a request
// + sets s_ambient_stop (to abort an in-flight fetch); barge-in / stop_ambient
// just set s_ambient_stop. Logs key transitions over UDP so a Wi-Fi-only device
// can be diagnosed without serial.
static void ambient_task(void *arg)
{
    (void)arg;
    ambient_req_t req;
    char ll[112];
    while (1) {
        // Idle: block until a play_ambient request arrives.
        if (xQueueReceive(s_ambient_req_q, &req, portMAX_DELAY) != pdTRUE) continue;
        // Coalesce a burst (rapid restores) down to the most recent request.
        ambient_req_t tmp;
        while (xQueueReceive(s_ambient_req_q, &tmp, 0) == pdTRUE) req = tmp;

        // We own playback now — clear the interrupt flag that play_ambient set
        // to wake us. A barge-in / stop / newer request during the waits below
        // re-sets it (or queues) and we bail before touching audio_io.
        s_ambient_stop = false;

        // Wait for any TTS sentence queue to drain so we don't fight
        // tts_worker_task for the audio_io ringbuffer. Capped; bail on a
        // stop or a newer request.
        for (int i = 0; i < 50; ++i) {
            if (uxQueueMessagesWaiting(s_sentence_q) == 0 && token_accum_empty()) break;
            if (s_ambient_stop || ambient_preempted()) break;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (s_ambient_stop || ambient_preempted()) continue;  // stopped/preempted before start

        // ── Begin a playback session ────────────────────────────────────────
        if (!vg_lifecycle_take()) continue;
        if (leds_buttons_is_muted()) {
            vg_lifecycle_give();
            // Physical mute is destructive for ambient sessions. Do not open
            // a new long-lived HTTP stream behind the mute fence.
            oe_ws_send_ambient_stopped("mute");
            continue;
        }
        s_ambient_active = true;
        strncpy(s_ambient_cur_marker, req.marker, sizeof(s_ambient_cur_marker) - 1);
        s_ambient_cur_marker[sizeof(s_ambient_cur_marker) - 1] = 0;
        // Attach a replacement session to either the provisional hold or the
        // established command/reply pause. In particular, ACCEPT's background
        // pause must outlive a point-in-time snapshot: a new session created
        // one tick later cannot play over the user's utterance.
        bool turn_busy =
            s_verify_playback_hold_active || s_in_utterance ||
            s_awaiting_reply || s_stream_active ||
            get_followup_until_us() != 0 ||
            uxQueueMessagesWaiting(s_sentence_q) > 0 ||
            !token_accum_empty() || alarm_is_firing() ||
            leds_buttons_is_muted();
        s_ambient_paused = turn_busy;
        if (s_verify_playback_hold_active) {
            s_verify_hold_ambient_owned = true;
            snprintf(s_verify_hold_ambient_marker,
                     sizeof(s_verify_hold_ambient_marker), "%s",
                     s_ambient_cur_marker);
        }
        s_ambient_stable_rate = 0;       // re-arm rate lock for this stream
        s_last_ambient_rate = 44100;
        if (req.volume >= 0 && req.volume <= 100) {
            s_pre_ambient_volume = audio_io_get_volume();
            audio_io_set_volume((uint8_t)req.volume);
        } else {
            s_pre_ambient_volume = -1;
        }
        if (!s_ambient_paused) {
            set_ui_state(UI_STATE_AMBIENT);
            xvf3800_enable_amplifier(true);
        }
        audio_io_start_playback();
        vg_lifecycle_give();
        for (uint8_t i = 0; i < WW_NUM_SLOTS; ++i)
            if (s_ww[i]) wakeword_notify_speaking_began(s_ww[i]);
        snprintf(ll, sizeof(ll), "[ambient] START marker=%s loop=%d vol=%d", req.marker, req.loop, req.volume);
        ESP_LOGI(TAG, "%s", ll); oe_udplog_send(ll);

        // Auto-reconnect loop (same backoff as before). oe_tts_post aborts
        // promptly on s_ambient_stop (barge-in/stop/newer request) via the
        // &s_ambient_stop hook; between attempts we also bail on a queued
        // newer request so a restore switches markers cleanly.
        const int retry_delays_ms[] = { 0, 1000, 2000, 5000, 10000, 30000 };
        const int retry_count = sizeof(retry_delays_ms) / sizeof(retry_delays_ms[0]);
        for (int attempt = 0; attempt < retry_count; ++attempt) {
            if (s_ambient_stop || ambient_preempted()) break;
            if (retry_delays_ms[attempt] > 0) {
                int slept = 0;
                while (slept < retry_delays_ms[attempt] && !s_ambient_stop && !ambient_preempted()) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    slept += 100;
                }
                if (s_ambient_stop || ambient_preempted()) break;
            }
            esp_err_t e = oe_tts_post(g_dev_config.server_url, g_dev_config.token,
                                      req.marker, NULL, -1, ambient_pcm_cb, NULL,
                                      &s_ambient_stop, &s_ambient_paused);
            snprintf(ll, sizeof(ll), "[ambient] fetch end e=%s stop=%d attempt=%d/%d",
                     esp_err_to_name(e), s_ambient_stop ? 1 : 0, attempt + 1, retry_count);
            ESP_LOGI(TAG, "%s", ll); oe_udplog_send(ll);
            if (s_ambient_stop || ambient_preempted()) break;
            // ESP_OK with !stop = server closed the stream → reconnect.
        }

        // ── Tear down this session ──────────────────────────────────────────
        // This worker owns only MUSIC. Keep the shared engine and speech lane
        // alive for any overlapping assistant reply.
        audio_io_flush_music();
        if (!vg_lifecycle_take()) continue;
        s_ambient_active = false;
        s_ambient_paused = false;
        s_ambient_cur_marker[0] = 0;
        const bool other_speech =
            s_stream_active || s_legacy_tts_active;
        const bool other_audio =
            other_speech || airplay_is_playing() ||
            alarm_is_firing();
        const bool gate_active = s_verify_playback_hold_active;
        vg_lifecycle_give();
        if (!other_speech) {
            for (uint8_t i = 0; i < WW_NUM_SLOTS; ++i)
                if (s_ww[i]) wakeword_notify_speaking_ended(s_ww[i]);
        }
        if (!other_audio || gate_active) {
            xvf3800_enable_amplifier(false);
            airplay_note_amp_forced_off();
        }
        if (s_pre_ambient_volume >= 0) {
            audio_io_set_volume((uint8_t)s_pre_ambient_volume);
            s_pre_ambient_volume = -1;
        }
        if (!other_audio && !gate_active && !s_in_utterance &&
            !s_awaiting_reply) {
            set_ui_state(UI_STATE_IDLE);
        }
        oe_udplog_send("[ambient] STOP");
        // Loop: a queued (preempting) request is picked up immediately next
        // iteration; otherwise we block idle in xQueueReceive.
    }
}

// Un-pause ambient: the live stream stayed open + reading (discarding) while a
// command ran; now play it again. Re-enables the amp + playback (the barge-in
// flushed/stopped it, and a TTS reply may have stopped it). The ambient task is
// still sitting in oe_tts_post — clearing s_ambient_paused makes its http_evt
// start decoding the live bytes again, so the rain picks up from "now".
// Caller owns s_verify_lifecycle_mutex.
static bool ambient_resume_locked(bool gate_rollback_owner)
{
    if (!s_ambient_paused || !s_ambient_active || s_ambient_stop ||
        leds_buttons_is_muted() || alarm_is_firing() ||
        audio_io_is_paused()) {
        return false;
    }
    if (!gate_rollback_owner &&
        (s_verify_playback_hold_active || s_in_utterance ||
         s_awaiting_reply || s_stream_active ||
         get_followup_until_us() != 0 ||
         uxQueueMessagesWaiting(s_sentence_q) > 0 ||
         !token_accum_empty())) {
        return false;
    }
    xvf3800_enable_amplifier(true);
    audio_io_start_playback();
    s_ambient_paused = false;
    oe_udplog_send("[ambient] RESUME");
    return true;
}

// Normal callers do not already own the lifecycle mutex. Re-check every
// source/priority predicate under it so a mute or new gate cannot slip between
// the final check and amplifier enable.
static void ambient_resume(void)
{
    if (!vg_lifecycle_take()) return;
    ambient_resume_locked(false);
    vg_lifecycle_give();
}

// Custom-chime worker: fetch the MP3 via the one-shot marker, hand off to
// alarm.c which decodes + persists + installs. alarm.c takes ownership of
// the MP3 buffer on success; on failure it frees it for us.
static void chime_upload_worker(void *arg)
{
    chime_upload_req_t *req = (chime_upload_req_t *)arg;
    uint8_t *mp3 = NULL;
    size_t mp3_len = 0;
    esp_err_t fe = oe_alarm_fetch_audio(g_dev_config.server_url, g_dev_config.token,
                                         req->marker, &mp3, &mp3_len);
    if (fe == ESP_OK && mp3 && mp3_len) {
        esp_err_t se = alarm_set_custom_chime(mp3, mp3_len);
        if (se != ESP_OK) {
            ESP_LOGW(TAG, "alarm_set_custom_chime failed: %s", esp_err_to_name(se));
        }
    } else {
        ESP_LOGW(TAG, "chime upload fetch failed: %s", esp_err_to_name(fe));
        if (mp3) heap_caps_free(mp3);
    }
    free(req);
    vTaskDelete(NULL);
}

// Bridges the alarm subsystem to the wake-word slots + XVF amplifier so
// alarm.c doesn't need to know about either. Wake-speaking notifications
// are AEC-residual hints — wake still fires during alarm audio (that's the
// dismiss path), they just steady the slot's internal state.
static void alarm_speaking_cb(bool speaking)
{
    if (speaking) {
        // Alarm priority wins over an unverified wake. The capture task drops
        // the provisional session on its next frame and releases only the
        // gate-owned holds; alarm audio queued meanwhile remains intact.
        vg_request_cancel();
    }
    if (speaking && s_ambient_active) {
        s_ambient_paused = true;
        oe_udplog_send("[ambient] PAUSE (alarm)");
    }
    for (uint8_t i = 0; i < WW_NUM_SLOTS; ++i) {
        if (!s_ww[i]) continue;
        if (speaking) wakeword_notify_speaking_began(s_ww[i]);
        else if (!s_stream_active && !s_legacy_tts_active)
            wakeword_notify_speaking_ended(s_ww[i]);
    }
}

static void alarm_amp_cb(bool enable)
{
    if (!vg_lifecycle_take()) return;
    if (enable) {
        // Alarm PCM may be queued behind a provisional gate or rejected by a
        // destructive mute fence. Serialize with the gate's final release:
        // either leave a deferred handoff for that path, or observe the gate
        // fully clear and enable playback here.
        if (leds_buttons_is_muted()) {
            s_alarm_amp_deferred = false;
        } else if (s_verify_playback_hold_active) {
            s_alarm_amp_deferred = true;
        } else {
            s_alarm_amp_deferred = false;
            audio_io_start_playback();
            xvf3800_enable_amplifier(true);
        }
        vg_lifecycle_give();
        return;
    }
    s_alarm_amp_deferred = false;
    // Alarm owns the music it queued, not the shared amplifier. Preserve amp
    // ownership for an overlapping reply/ambient/AirPlay source; each source's
    // own finalizer will turn it off when that source actually ends.
    bool another_source =
        !audio_io_is_paused() &&
        (s_stream_active || s_legacy_tts_active ||
         (s_ambient_active && !s_ambient_paused) ||
         airplay_is_playing());
    if (!another_source || s_verify_playback_hold_active ||
        leds_buttons_is_muted()) {
        xvf3800_enable_amplifier(false);
        airplay_note_amp_forced_off();
    }
    vg_lifecycle_give();
}

// Human-readable last-reset cause, sent in the [boot] UDP line. PANIC /
// INT_WDT / TASK_WDT / BROWNOUT distinguish a crash-reboot (what we suspect
// drives the overnight reconnect loop) from a clean power-cycle or OTA.
static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

// ── DoA probe (0.2.74, experimental — strictly read-only) ───────────────────
// RESOLVED by the first probe flash (2026-07-07): the formatBCE HA variant
// DOES expose the AEC beamformer over device control —
//   AEC_AZIMUTH_VALUES  (33, 75): 4 floats = beam1, beam2, free-running,
//                                 auto-select — azimuth in radians
//   AEC_SPENERGY_VALUES (33, 80): 4 floats — speech energy per beam
// but reads answer status 0x40 (SERVICER_COMMAND_RETRY, XMOS sln_voice)
// whenever the servicer has no fresh estimate — which is ALWAYS true in a
// silent room, so the original one-shot discovery sweep misread "exposed but
// quiet" as "not exposed". The host contract (verified against formatBCE's
// ESPHome component, which runs this same firmware) is: retry the whole
// write+read up to ~8× with a short pause; retries exhausting during silence
// is itself signal (no source to localize). Wrong lengths return 0x42, and
// cmd 74 returns 0x42 at every length — it does not exist on this variant
// (the gillespinault map it came from was for a different firmware).
//
// This task streams the values over the UDP log so a human walking around
// the device can verify the auto-select beam (index 3) tracks the talker.
// If it does, next step is direction-gating the conversation windows —
// possibly with the AEC beam-lock writes formatBCE uses (cmds 37/81), but
// NOT in this build: reads only, and never SAVE_CONFIGURATION.
#define DOA_STATUS_DONE  0x00
#define DOA_STATUS_WAIT  0x01   // CTRL_WAIT: transport busy, retry
#define DOA_STATUS_RETRY 0x40   // SERVICER_COMMAND_RETRY: no fresh data yet
// Reads one 4-float AEC value set. True = fresh data in vals[4]; false =
// silence (retry status exhausted) or error, with the last status byte in
// *last_st (0xEE = I²C transport failure).
static bool doa_read4(uint8_t cmd, float vals[4], uint8_t *last_st)
{
    for (int attempt = 0; attempt < 8; ++attempt) {
        uint8_t st = 0xEE, buf[16] = {0};
        if (xvf3800_xmos_read_raw(XVF_RESID_AEC, cmd, &st, buf, 16) != ESP_OK) {
            if (last_st) *last_st = 0xEE;
            return false;
        }
        if (last_st) *last_st = st;
        if (st == DOA_STATUS_DONE) {
            memcpy(vals, buf, 16);
            return true;
        }
        if (st != DOA_STATUS_WAIT && st != DOA_STATUS_RETRY) return false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

static void doa_probe_task(void *arg)
{
    // Let the XVF boot and agc_freeze_task settle before poking the bus.
    vTaskDelay(pdMS_TO_TICKS(10000));

    for (;;) {
        // Poll faster whenever the direction data would actually matter: an
        // utterance being captured, an armed follow-up window, or our own
        // reply streaming (barge candidates).
        bool active = s_in_utterance || s_stream_active ||
                      get_followup_until_us() > 0;
        vTaskDelay(pdMS_TO_TICKS(active ? 400 : 2000));

        float az[4] = {0}, spe[4] = {0};
        uint8_t az_st = 0, spe_st = 0;
        bool az_ok  = doa_read4(XVF_CMD_AEC_AZIMUTH_VALUES,  az,  &az_st);
        bool spe_ok = doa_read4(XVF_CMD_AEC_SPENERGY_VALUES, spe, &spe_st);

        // Publish fresh azimuths for the direction gate (0.2.75). The gate
        // treats readings older than DOA_GATE_MAX_AGE_US as absent, so the
        // 2 s idle cadence naturally fails it open outside conversations.
        if (az_ok) {
            taskENTER_CRITICAL(&s_doa_mux);
            memcpy(s_doa_az, az, sizeof(az));
            s_doa_fresh_us = esp_timer_get_time();
            taskEXIT_CRITICAL(&s_doa_mux);
        }

        // Silent room at idle cadence = nothing to say; skip the line so the
        // udplog isn't 40k no-op rows a day. While "active" we always emit —
        // even a stale read proves the plumbing during the windows we care
        // about, and fresh lines appear exactly when someone is audible.
        if (!az_ok && !spe_ok && !active) continue;

        // snprintf returns would-have-written length — clamp off after every
        // append or `sizeof(line) - off` underflows (size_t) if garbage data
        // decodes into very wide floats.
        char line[160];
        int off = snprintf(line, sizeof(line), "[doa]");
        #define DOA_APPEND(...) do { \
                if (off < (int)sizeof(line) - 1) \
                    off += snprintf(line + off, sizeof(line) - off, __VA_ARGS__); \
                if (off > (int)sizeof(line) - 1) off = (int)sizeof(line) - 1; \
            } while (0)
        if (az_ok)  DOA_APPEND(" az=%.2f,%.2f,%.2f,%.2f",
                               (double)az[0], (double)az[1], (double)az[2], (double)az[3]);
        else        DOA_APPEND(" az=st%02x", az_st);
        if (spe_ok) DOA_APPEND(" spe=%g,%g,%g,%g",
                               (double)spe[0], (double)spe[1], (double)spe[2], (double)spe[3]);
        else        DOA_APPEND(" spe=st%02x", spe_st);
        DOA_APPEND(" u=%d s=%d", s_in_utterance ? 1 : 0, s_stream_active ? 1 : 0);
        #undef DOA_APPEND
        oe_udplog_send(line);
    }
}

static void boot_operational(void)
{
    // Create this before alarm/WS/TTS tasks can call into the verify-gate
    // lifecycle. A configured gate fails closed if its cross-component hold
    // cannot be serialized safely.
    s_verify_lifecycle_mutex = xSemaphoreCreateMutex();
    if (!s_verify_lifecycle_mutex) {
        ESP_LOGE(TAG, "verify lifecycle mutex alloc");
        esp_restart();
    }

    nvs_creds_get_server(g_dev_config.server_url, sizeof(g_dev_config.server_url));
    nvs_creds_get_token(g_dev_config.token, sizeof(g_dev_config.token));
    nvs_creds_get_device_name(g_dev_config.device_name, sizeof(g_dev_config.device_name));

    // Verify-gate config. Both the fixed proxy path and OE's canonical device
    // id are required. A non-empty legacy direct URL proves gating was enabled,
    // so migrate it to the fixed OE proxy path in memory instead of silently
    // bypassing wakes while waiting for server_caps. Only a persisted,
    // explicitly empty value is disabled; a missing/unreadable key is pending
    // and blocks wakes until server_caps from an allowed origin makes policy
    // explicit.
    esp_err_t gate_path_e =
        nvs_creds_get_verify_gate_path(g_dev_config.verify_gate_path,
                                       sizeof(g_dev_config.verify_gate_path));
    esp_err_t device_id_e =
        nvs_creds_get_device_id(g_dev_config.device_id,
                                sizeof(g_dev_config.device_id));
    bool gate_path_loaded =
        gate_path_e == ESP_OK &&
        server_verify_gate_path_valid(g_dev_config.verify_gate_path);
    bool migrate_legacy_gate =
        (gate_path_e == ESP_OK && !gate_path_loaded &&
         g_dev_config.verify_gate_path[0] != 0) ||
        gate_path_e == ESP_ERR_NVS_INVALID_LENGTH;
    bool gate_policy_known = gate_path_loaded || migrate_legacy_gate;
    bool device_id_loaded = device_id_e == ESP_OK &&
                            server_device_id_valid(g_dev_config.device_id);
    if (migrate_legacy_gate) {
        snprintf(g_dev_config.verify_gate_path,
                 sizeof(g_dev_config.verify_gate_path), "%s",
                 OE_VERIFY_GATE_PATH);
        ESP_LOGW(TAG, "verify gate: migrated legacy direct endpoint to OE proxy");
    } else if (!gate_path_loaded) {
        g_dev_config.verify_gate_path[0] = 0;
    }
    if (!device_id_loaded) g_dev_config.device_id[0] = 0;
    s_verify_gate_config_known = gate_policy_known;
    s_verify_gate_config_persisted = gate_path_loaded && device_id_loaded;
    if (migrate_legacy_gate && device_id_loaded) {
        esp_err_t migrate_e =
            nvs_creds_set_verify_gate_config(
                g_dev_config.device_id, g_dev_config.verify_gate_path);
        s_verify_gate_config_persisted = migrate_e == ESP_OK;
        if (migrate_e != ESP_OK) {
            ESP_LOGW(TAG, "verify gate: legacy migration is volatile: %s",
                     esp_err_to_name(migrate_e));
        }
    }
    if (!s_verify_gate_config_known) {
        ESP_LOGW(TAG, "verify gate: policy pending allowed-origin server_caps");
    } else if (g_dev_config.verify_gate_path[0] && g_dev_config.device_id[0]) {
        ESP_LOGI(TAG, "verify gate: OE proxy configured (device_id %s)",
                 g_dev_config.device_id);
    } else if (g_dev_config.verify_gate_path[0]) {
        ESP_LOGI(TAG, "verify gate: pending canonical device_id");
    } else {
        ESP_LOGI(TAG, "verify gate: disabled (no verify_gate_path)");
    }
    if (nvs_creds_get_default_agent(g_dev_config.default_agent_id,
                                    sizeof(g_dev_config.default_agent_id)) != ESP_OK ||
        g_dev_config.default_agent_id[0] == 0) {
        // No default agent configured — leave it empty. oe_ws_send_chat/stop
        // omit the agent field when empty and the server routes the turn to
        // the acting user's coordinator (a wake-slot assignment overrides
        // the default anyway). Hardcoding a "likely" agent id here breaks
        // fresh installs where that agent doesn't exist.
        g_dev_config.default_agent_id[0] = 0;
    }
    ESP_LOGI(TAG, "default agent: %s",
             g_dev_config.default_agent_id[0] ? g_dev_config.default_agent_id : "(coordinator)");
    uint8_t slot = 0;
    nvs_creds_get_wake_slot(&slot);
    g_dev_config.wake_word_slot = slot;

    // Restore last-set playback volume from NVS. Falls back to the
    // audio_io default (80 %) if no value persisted yet — e.g. first
    // boot after factory reset.
    uint8_t saved_vol = 0;
    if (nvs_creds_get_volume(&saved_vol) == ESP_OK && saved_vol > 0 && saved_vol <= 100) {
        audio_io_set_volume(saved_vol);
        ESP_LOGI(TAG, "restored volume from NVS: %u%%", saved_vol);
    }

    // Restore headphone-mode flag. When set, amp_en stays LOW during
    // playback so the XVF AEC doesn't suppress wake-word mic level
    // (audio still reaches the 3.5 mm jack which taps the DAC before
    // the speaker amp).
    uint8_t saved_hp = 0;
    esp_err_t hp_e = nvs_creds_get_headphone_mode(&saved_hp);
    if (hp_e == ESP_OK) {
        xvf3800_set_headphone_mode(saved_hp != 0);
        ESP_LOGI(TAG, "headphone mode restored: %s", saved_hp ? "on" : "off");
    } else {
        // First boot, no server-sent preference yet. Default is a build-time
        // choice: line-out/headphone ON for external-amp boards (mics stay
        // live because amp_en is never asserted), OFF for onboard-speaker
        // boards (amp must be asserted to hear anything). A disabled bool
        // Kconfig emits no #define, so this must be #ifdef, not a plain read
        // of CONFIG_OE_DEFAULT_HEADPHONE_MODE — the latter fails to compile
        // when the option is off.
#ifdef CONFIG_OE_DEFAULT_HEADPHONE_MODE
        const bool default_hp = true;
#else
        const bool default_hp = false;
#endif
        xvf3800_set_headphone_mode(default_hp);
        nvs_creds_set_headphone_mode(default_hp ? 1 : 0);
        ESP_LOGI(TAG, "headphone mode defaulted: %s", default_hp ? "on" : "off");
    }

    char ssid[64] = {0};
    char password[64] = {0};
    nvs_creds_get_wifi(ssid, sizeof(ssid), password, sizeof(password));
    if (wifi_sta_start(ssid, password) != ESP_OK) {
        // Just reboot and try again — DO NOT factory-reset NVS. A 30 s
        // join failure is most often a transient DHCP/router hiccup, and
        // wiping the pairing token forces the user to re-do captive-portal
        // setup, which is brutal UX. If the router is genuinely gone the
        // device will loop until it's back. Future enhancement: count
        // consecutive failures and only factory-reset after N attempts.
        ESP_LOGE(TAG, "wifi join failed — rebooting (creds preserved)");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    }

    // SNTP: required by the alarm subsystem to schedule esp_timers against
    // wall-clock epoch. Fire and forget — alarm_init's deferred_schedule_task
    // polls time() until it acquires (typically <2s on a working network),
    // so a slow NTP server doesn't block the rest of boot.
    //
    // 30 s poll interval doubles as a network keepalive — without periodic
    // outbound traffic, some APs / Wi-Fi stacks let a device drift into a
    // silent-RX state after 3-5 min idle (heartbeat task confirms CPU is
    // awake; the modem just stops forwarding incoming TCP/mDNS for our
    // address). A small NTP packet every 30 s keeps the AP-side forwarding
    // table warm AND benefits the alarm subsystem.
    {
        esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        esp_netif_sntp_init(&sntp_cfg);
        sntp_set_sync_interval(30000);
        sntp_restart();
        ESP_LOGI(TAG, "sntp init -> pool.ntp.org (30 s poll, doubles as Wi-Fi keepalive)");
    }

    // Alarm subsystem: loads persisted alarms from NVS and schedules them
    // once SNTP acquires. Must run after nvs_creds_init (done in app_main)
    // and after WiFi is up (so the deferred scheduler can succeed).
    alarm_set_speaking_callback(alarm_speaking_cb);
    alarm_set_amp_callback(alarm_amp_cb);
    alarm_init();

    // Amplifier is enabled lazily, only around TTS playback (in
    // tts_worker_task). Leaving it enabled at boot suppressed wake-word
    // detection — the XVF3800's AEC treats "amp on" as "speaker active"
    // and reduces effective mic level, dropping our wake probability
    // from ~254/255 to ~1/255 on the same utterance.

    wakeword_mount_partition("wakewords", "/ww");

    // Load every available slot in the SPIFFS partition concurrently. A slot
    // that fails to load (missing tflite, corrupt manifest, OOM) is skipped;
    // the firmware keeps running with the slots that did load. With zero
    // slots loaded the device stops responding to wake words — that's a hard
    // error worth logging but not crashing on.
    uint8_t loaded = 0;
    for (uint8_t i = 0; i < WW_NUM_SLOTS; ++i) {
        wakeword_config_t wcfg = {
            .slot = i,
            .threshold = 0.7f,
            .cooldown_ms = 1500,
            .refractory_after_speak_ms = 400,
        };
        s_ww[i] = wakeword_create(&wcfg);
        if (!s_ww[i]) {
            ESP_LOGE(TAG, "wakeword slot %u: create failed", i);
            continue;
        }
        esp_err_t e = wakeword_load_slot(s_ww[i], i);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "wakeword slot %u: load failed (%s) — skipping",
                     i, esp_err_to_name(e));
            wakeword_destroy(s_ww[i]);
            s_ww[i] = NULL;
            continue;
        }
        loaded++;
    }
    if (loaded == 0) {
        ESP_LOGE(TAG, "no wake-word slots loaded — device will not respond to wake words");
    } else {
        ESP_LOGI(TAG, "wake-word slots loaded: %u/%u", loaded, WW_NUM_SLOTS);
    }

    s_sentence_q = xQueueCreate(16, sizeof(sentence_t));
    s_token_mutex = xSemaphoreCreateMutex();
    if (!s_token_mutex) { ESP_LOGE(TAG, "token mutex alloc"); esp_restart(); }
    s_capture_buf = malloc(CAPTURE_BUFFER_SAMPLES * sizeof(int16_t));
    if (!s_capture_buf) { ESP_LOGE(TAG, "capture buf alloc"); esp_restart(); }
    // Pre-roll ring (12.8 KB). PSRAM preferred; internal fallback; NULL is
    // fine — preroll_* helpers no-op and follow-up capture just loses onsets.
    s_preroll_buf = heap_caps_malloc(PREROLL_SAMPLES * sizeof(int16_t),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_preroll_buf) s_preroll_buf = malloc(PREROLL_SAMPLES * sizeof(int16_t));
    if (!s_preroll_buf) {
        ESP_LOGW(TAG, "preroll alloc failed — follow-up onset disabled; configured verify gate will fail-closed");
    }

    // Verify-gate wake-window buffer (~80 KB) + its worker task. PSRAM; if
    // setup fails, an explicitly disabled gate still uses the legacy path, but
    // a configured gate drops wakes until a reboot restores the resources.
    // The worker is created unconditionally (it idles on the semaphore) so
    // server_caps can enable or disable the gate without a firmware change.
    s_verify_wav = heap_caps_malloc(VG_WIN_SAMPLES * sizeof(int16_t),
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_verify_wav) {
        ESP_LOGW(TAG, "verify-gate window alloc failed — configured gate will fail-closed");
    } else {
        s_verify_job_sem = xSemaphoreCreateBinary();
        if (!s_verify_job_sem) {
            ESP_LOGW(TAG, "verify-gate sem alloc failed — configured gate will fail-closed");
            heap_caps_free(s_verify_wav);
            s_verify_wav = NULL;
        } else if (xTaskCreate(verify_worker_task, "verify_gate", 6144, NULL, 5,
                               &s_verify_worker_handle) != pdPASS) {
            ESP_LOGW(TAG, "verify-gate worker create failed — configured gate will fail-closed");
            vSemaphoreDelete(s_verify_job_sem);
            s_verify_job_sem = NULL;
            heap_caps_free(s_verify_wav);
            s_verify_wav = NULL;
        }
    }

    // Per-boot turn-id prefix: distinguishes this boot's turns from a
    // pre-reboot turn's stale events still queued server-side.
    snprintf(s_turn_prefix, sizeof(s_turn_prefix), "%04x",
             (unsigned)(esp_random() & 0xFFFF));

    oe_ws_start(g_dev_config.server_url, g_dev_config.token, ws_event_cb, NULL);

    // Diagnostic UDP forwarder: lets a Wi-Fi-only device be watched like serial
    // (the [hb]/[ambient-stats] heartbeat below + this [boot] line), and the
    // datagrams keep arriving while the WS is dropping. Best-effort; a failed
    // init just leaves oe_udplog_send a no-op. The [boot] line's reset reason
    // is the key signal for whether the device is crash-rebooting under load.
    oe_udplog_init(g_dev_config.server_url, OE_UDPLOG_PORT);
    {
        const uint32_t int_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
        char bl[160];
        snprintf(bl, sizeof(bl),
                 "[boot] reset=%s heap_int=%luKB heap_int_largest=%luKB heap_psram=%luKB",
                 reset_reason_str(esp_reset_reason()),
                 (unsigned long)(heap_caps_get_free_size(int_caps) / 1024),
                 (unsigned long)(heap_caps_get_largest_free_block(int_caps) / 1024),
                 (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
        ESP_LOGI(TAG, "%s", bl);
        oe_udplog_send(bl);
    }

    // 0.2.34: tts_worker, capture_and_drive, and ambient_w (created on demand
    // elsewhere) each carry libhelix mp3 decode + audio_io resampler stack
    // frames during playback. 8 KB was occasionally tight enough to trigger
    // vApplicationStackOverflowHook after a long uptime under load — bumped
    // to 12 KB for breathing room. Heartbeat below now logs per-task
    // high-water marks so we can tell empirically which task gets closest.

    // Single persistent ambient task + request queue (replaces the old
    // per-play_ambient task spawning that let two ambient workers race).
    s_ambient_req_q = xQueueCreate(4, sizeof(ambient_req_t));
    if (s_ambient_req_q) xTaskCreate(ambient_task, "ambient", 12288, NULL, 4, NULL);
    else ESP_LOGE(TAG, "ambient queue alloc failed — ambient playback disabled");

    xTaskCreate(tts_worker_task, "tts_worker", 12288, NULL, 6, NULL);
    xTaskCreate(stream_finalize_task, "stream_fin", 4096, NULL, 6, NULL);
    xTaskCreatePinnedToCore(capture_and_drive_task, "drive", 12288, NULL, 7, NULL, 0);
    xTaskCreate(xvf_migration_task, "xvf_migrate", 3072, NULL, 3, NULL);
    xTaskCreate(boot_indicator_task, "boot_ind", 3072, NULL, 4, NULL);
    xTaskCreate(agc_freeze_task, "agc_freeze", 3072, NULL, 4, NULL);
    // 0.2.74 experiment — remove (or promote to a real direction gate) once
    // the DoA question is answered. Low priority: purely diagnostic traffic.
    // DISABLED 2026-07-24 for the verify-gate test image: keep this pre-existing
    // WIP out of the flashed build so on-device gate validation isn't muddied by
    // the probe's I2C sweep. Re-enable to resume the DoA investigation.
    // xTaskCreate(doa_probe_task, "doa_probe", 4096, NULL, 2, NULL);
    (void)doa_probe_task;   // keep the function referenced so it still compiles
    // hb itself needs ~4 KB now that it runs the per-minute stack-hwm
    // dump: 256-byte line buffer + ESP_LOGI (vprintf) + xTaskGetHandle +
    // uxTaskGetStackHighWaterMark traversal added enough stack pressure
    // to overflow the original 2 KB allocation (observed 0.2.34 boot).
    xTaskCreate(heartbeat_task, "hb", 6144, NULL, 1, NULL);

    s_operational_boot_ready = true;
    maybe_resume_pending_ota();
}

// Heartbeat — logs every 10s so we can tell from serial whether the CPU is
// actually running. If users report "the device went to sleep", presence or
// absence of [hb] lines in the log answers the question instantly.
//
// Every 6th tick (~once a minute) we also dump per-task stack high-water
// marks. A task whose remaining stack ever dips near zero is the one to
// blame on the next vApplicationStackOverflowHook panic. We can't iterate
// every task without a static list (FreeRTOS' trace-facility runtime
// enumeration is heavy), so we look up the long-lived ones by name.
static void heartbeat_task(void *arg)
{
    (void) arg;
    uint32_t n = 0;
    static const char *kWatchTasks[] = {
        "tts_worker", "drive", "ambient", "audio_play", "audio_cap",
        "oe_ota", "agc_freeze", "boot_ind", "hb",
        // esp_websocket_client's task. Its stack carries the whole streamed-
        // TTS write path (base64 decode + 16k→48k resample); the 4 KB default
        // overflowed in 0.2.60 (panic on every reply). Now 8 KB via
        // .task_stack in oe_ws.c — watch it here so creep is visible.
        "websocket_task",
    };
    // Ambient-stats delta state. Track totals at the prior heartbeat so
    // each line shows per-interval rates (bytes/sec, decode errs/sec)
    // alongside instantaneous gauges (RSSI, buffer fills, heap).
    uint32_t prev_tts_bytes = 0;
    uint32_t prev_decode_errs = 0;
    uint32_t prev_tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    uint32_t prev_cap_samples = audio_io_get_capture_samples_total();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        {
            // 10s pulse with link + heap, sent over UDP too. If this pulse
            // stops arriving server-side the device crashed/rebooted — the
            // next [boot] line then reports the reset reason. Falling rssi or
            // shrinking heap_int across the night is the leak/RF smoking gun.
            int rssi_now = 0;
            wifi_ap_record_t apr;
            if (esp_wifi_sta_get_ap_info(&apr) == ESP_OK) rssi_now = apr.rssi;
            // Mic-liveness: 16 kHz samples captured over this interval,
            // normalized to per-second. ~16000 = mic path healthy; 0 = the
            // capture pipeline is dead even though the CPU/Wi-Fi are fine
            // (the 0.2.60 deaf-device failure mode). Cheap unsigned delta.
            const uint32_t cap_total = audio_io_get_capture_samples_total();
            const uint32_t cap_sps = (cap_total - prev_cap_samples) / 10u;
            prev_cap_samples = cap_total;
            if (!s_ota_marked_valid && s_ws_connected && cap_sps > 0) {
                oe_ota_mark_running_valid();
                s_ota_marked_valid = true;
            }
            const uint32_t int_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
            char hbline[192];
            // pcm_drop is the running total of playback samples lost to a
            // full ring (audio_io_write_pcm). Nonzero = audible skip
            // happened; steadily climbing = server pacing outrunning the
            // ring. Cumulative on purpose — a rare drop stays visible.
            snprintf(hbline, sizeof(hbline),
                     "[hb] alive tick=%lu rssi=%d heap_int=%luKB "
                     "heap_int_largest=%luKB cap_sps=%lu pcm_drop=%lu",
                     (unsigned long)n++, rssi_now,
                     (unsigned long)(heap_caps_get_free_size(int_caps) / 1024),
                     (unsigned long)(heap_caps_get_largest_free_block(int_caps) / 1024),
                     (unsigned long)cap_sps,
                     (unsigned long)audio_io_get_playback_drop_samples());
            ESP_LOGI(TAG, "%s", hbline);
            oe_udplog_send(hbline);
        }
        if ((n % 6) == 0) {
            char line[256];
            int off = snprintf(line, sizeof(line), "[hb] stack hwm (bytes remaining):");
            for (size_t i = 0; i < sizeof(kWatchTasks) / sizeof(kWatchTasks[0]); ++i) {
                TaskHandle_t h = xTaskGetHandle(kWatchTasks[i]);
                if (!h) continue;
                UBaseType_t words = uxTaskGetStackHighWaterMark(h);
                int written = snprintf(line + off, sizeof(line) - off,
                    " %s=%u", kWatchTasks[i], (unsigned)(words * sizeof(StackType_t)));
                if (written <= 0 || (size_t)written >= sizeof(line) - off) break;
                off += written;
            }
            ESP_LOGI(TAG, "%s", line);
            oe_udplog_send(line);
        }

        // ── Ambient streaming telemetry ──────────────────────────────────
        // Per-heartbeat (10s) line showing Wi-Fi, network, decoder, and
        // memory state — the data needed to diagnose audio dropouts. Only
        // fires while ambient is active so it doesn't spam during normal
        // operation. Single grep-friendly line, key=value pairs:
        //   rssi  : current AP RSSI in dBm (link quality; <-75 is poor)
        //   bytes/s: HTTP byte rate over the interval (should ≈ 20000 for
        //            160kbps CBR; a drop indicates network slowdown)
        //   dec_errs/s: per-second mp3 decode-error count (resyncs)
        //   mp3_inbuf: bytes queued waiting for the decoder
        //   pcm_rb: bytes queued waiting for I²S to drain — IF THIS GOES
        //           NEAR 0 you have an underrun and the speaker pops
        //   heap_int: free internal SRAM in KB
        //   heap_int_largest: largest allocatable internal block in KB
        //   heap_psram: free PSRAM in KB
        //   heap_int_min: lowest free internal ever (low-water mark)
        const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        const uint32_t interval_ms = (now_ms > prev_tick_ms) ? (now_ms - prev_tick_ms) : 10000;
        const uint32_t tts_bytes = oe_tts_get_total_bytes_received();
        const uint32_t decode_errs = mp3_dec_get_total_errors();
        const uint32_t bytes_delta = (tts_bytes >= prev_tts_bytes) ? (tts_bytes - prev_tts_bytes) : 0;
        const uint32_t errs_delta = (decode_errs >= prev_decode_errs) ? (decode_errs - prev_decode_errs) : 0;
        prev_tts_bytes = tts_bytes;
        prev_decode_errs = decode_errs;
        prev_tick_ms = now_ms;

        if (s_ambient_active) {
            int rssi = 0;
            wifi_ap_record_t ap_info;
            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                rssi = ap_info.rssi;
            }
            uint32_t pcm_used = 0, pcm_cap = 0;
            audio_io_get_music_buf_stats(&pcm_used, &pcm_cap);
            // bytes/sec and errs/sec scaled from the actual interval so a
            // late wake-up of the hb task doesn't lie about rates.
            const uint32_t bytes_per_sec = (interval_ms > 0) ? (bytes_delta * 1000u / interval_ms) : 0;
            const uint32_t errs_per_sec_x10 = (interval_ms > 0) ? (errs_delta * 10000u / interval_ms) : 0;
            const uint32_t int_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
            char as[256];
            snprintf(as, sizeof(as),
                "[ambient-stats] rssi=%d bytes/s=%lu dec_errs/s=%lu.%lu "
                "pcm_rb=%lu/%lu heap_int=%lu heap_int_largest=%lu "
                "heap_psram=%lu heap_int_min=%lu",
                rssi,
                (unsigned long)bytes_per_sec,
                (unsigned long)(errs_per_sec_x10 / 10),
                (unsigned long)(errs_per_sec_x10 % 10),
                (unsigned long)pcm_used,
                (unsigned long)pcm_cap,
                (unsigned long)(heap_caps_get_free_size(int_caps) / 1024),
                (unsigned long)(heap_caps_get_largest_free_block(int_caps) / 1024),
                (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
                (unsigned long)(heap_caps_get_minimum_free_size(int_caps) / 1024));
            ESP_LOGI(TAG, "%s", as);
            oe_udplog_send(as);
        }
    }
}

static void boot_provisioning(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    char ssid[32];
    snprintf(ssid, sizeof(ssid), "oe-voice-%02X%02X", mac[4], mac[5]);
    set_ui_state(UI_STATE_PROVISION);
    captive_portal_start(ssid, portal_submit_cb, NULL);
}

void app_main(void)
{
    nvs_creds_init();
    // Reserve OTA's task stack/TCB in internal DRAM at boot. The persistent
    // worker sleeps until notified and never asks a fragmented runtime heap
    // for the contiguous 8 KB block that failed on long-running devices.
    esp_err_t ota_init_e = oe_ota_init();
    if (ota_init_e != ESP_OK) {
        ESP_LOGE(TAG, "persistent OTA worker init failed: %s",
                 esp_err_to_name(ota_init_e));
    }
    esp_netif_init();
    esp_event_loop_create_default();

    // Init order matters: I²C bus + XVF acknowledgement BEFORE we start
    // clocking on I²S. The Seeed reSpeaker XVF3800 i2s firmware needs the
    // host to be reachable on I²C before it'll stream audio on DIN — the
    // working third-party project (gillespinault/respeaker-xvf3800-vad)
    // sequences I²C init → version read → I²S init, in that order.
    xvf3800_init();
    leds_buttons_init(mute_change_cb);
    esp_err_t audio_err = audio_io_init();
    if (audio_err != ESP_OK) {
        // In particular, surface PSRAM exhaustion from the enlarged
        // reversible speech ring. Continuing would look like a successful
        // boot but leave capture/playback (and therefore gating) dead.
        ESP_LOGE(TAG, "audio_io_init failed: %s", esp_err_to_name(audio_err));
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    if (nvs_creds_is_provisioned()) {
        ESP_LOGI(TAG, "provisioned — operational boot");
        boot_operational();
    } else {
        ESP_LOGI(TAG, "unprovisioned — provisioning boot");
        boot_provisioning();
    }
}
