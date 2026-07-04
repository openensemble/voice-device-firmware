#include "state.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>   // stat() for ww_file_matches (skip identical ww re-push)

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
// The wakewords partition is 0x80000 (512 KB) per partitions.csv — at ~63 KB
// per slot pair that fits 6 comfortably with overhead. Bumping past 6 would
// require either resizing the partition (USB re-flash) or shrinking the
// SPIFFS image somehow.
#define WW_NUM_SLOTS 6
static wakeword_t *s_ww[WW_NUM_SLOTS] = { NULL };
static uint8_t     s_active_slot      = 0;  // which slot fired the current utterance
static uint8_t     s_active_wake_prob = 0;  // sliding-window avg prob (0..255) of the wake that fired

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
static uint8_t       s_pcm_frame[4096];          // base64 decode scratch (WS callback is single-threaded)
// Liveness timestamp for the streamed-TTS path: last tts_audio_begin/tts_audio
// arrival (esp_timer us). stream_finalize_task's stall watchdog uses it to
// tear down a SPEAKING state whose tts_audio_end never arrives (server crash
// mid-stream on a healthy socket). Without this, s_stream_active had NO
// timeout: amp stayed on (AEC suppressing the mic) and every non-owner wake
// slot stayed gated forever.
static volatile int64_t s_last_tts_frame_us = 0;
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

static void mint_turn_id(void)
{
    snprintf(s_turn_id, sizeof(s_turn_id), "%s-%lu",
             s_turn_prefix, (unsigned long)(++s_turn_counter));
}

// True when the event names a turn that is NOT the device's current one.
// Missing turn_id (NULL/empty, i.e. older server) always passes.
static bool evt_turn_stale(const oe_ws_payload_t *evt)
{
    if (!evt->turn_id || !evt->turn_id[0]) return false;
    if (!s_turn_id[0]) return true;   // we have no live turn — event is from a past life
    return strcmp(evt->turn_id, s_turn_id) != 0;
}

// Server capability flags, learned from `server_caps` after auth and reset on
// disconnect (the next server may be older). Gate every NEW device→server
// message type on these so a firmware upgrade never breaks against an old
// server. turn_id fields on chat/stop are NOT gated — old servers ignore
// unknown fields harmlessly.
static volatile bool s_caps_tts_pause  = false;
static volatile bool s_caps_stt_stream = false;

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
// Tunables. BARGE_SETTLE_MS covers the unknown mic-gain step response after
// amp-off — the [barge] UDP telemetry is the measuring instrument; adjust
// after reading real kitchen logs.
#define BARGE_CONSEC_FRAMES     2      // 80 ms frames above trigger to become a candidate
#define BARGE_FLOOR_MULT        4      // trigger = max(floor*MULT, VOICE_ENERGY_THRESHOLD)
#define BARGE_SETTLE_MS         150    // ignore first ms after amp-off (gain settling)
#define BARGE_CONFIRM_SPEECH_MS 350    // cumulative speech to confirm a real barge
#define BARGE_VERIFY_WINDOW_MS  1000   // candidate → confirm deadline, else resume
typedef enum { BARGE_NONE = 0, BARGE_VERIFYING } barge_state_t;
static barge_state_t     s_barge_state = BARGE_NONE;
static int               s_barge_consec = 0;
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
static volatile bool s_ota_marked_valid = false;

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
// Slot the conversation was on when follow-up was armed. Any utterance
// captured during the window — whether a VAD-start or a (potentially
// false) wake fire on a different slot — gets routed back to THIS slot
// so the answer always lands on the user who asked the original turn.
static volatile uint8_t s_followup_slot = 0;
// Single speech-energy threshold (mean-square of int16 samples, ~-31 dBFS)
// shared by the VAD config and frame_is_speech. These used to be two
// literals ("800000, same as VAD") that could drift apart.
#define VOICE_ENERGY_THRESHOLD 800000
static vad_state_t *s_vad = NULL;
static QueueHandle_t s_sentence_q = NULL;

// Pre-roll: a rolling window of the most recent mic audio, kept while idle so
// a follow-up answer captured via VAD-start (which by definition triggers
// AFTER speech has begun) can prepend the onset instead of clipping the first
// syllable. Reset every time a follow-up window arms so it can never contain
// the tail of our own TTS. PSRAM; feature silently disabled if alloc fails.
#define PREROLL_SAMPLES (16000 * 400 / 1000)   // 400 ms @ 16 kHz mono
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
} sentence_t;

static char s_token_accum[2048];
static size_t s_token_accum_len = 0;
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
        return;
    }
    sentence_t s;
    size_t take = s_token_accum_len < SENTENCE_MAX - 1 ? s_token_accum_len : SENTENCE_MAX - 1;
    memcpy(s.text, s_token_accum, take);
    s.text[take] = 0;
    xQueueSend(s_sentence_q, &s, pdMS_TO_TICKS(100));
    s_token_accum_len = 0;
    s_token_accum[0] = 0;
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
    token_unlock();
}

static bool token_accum_empty(void)
{
    token_lock();
    bool empty = s_token_accum_len == 0;
    token_unlock();
    return empty;
}

static void accumulate_token(const char *tok, size_t len)
{
    if (!tok || len == 0) return;
    if (len >= sizeof(s_token_accum)) len = sizeof(s_token_accum) - 1;

    token_lock();
    if (s_token_accum_len + len + 1 > sizeof(s_token_accum)) flush_token_to_sentence_queue_locked();
    memcpy(s_token_accum + s_token_accum_len, tok, len);
    s_token_accum_len += len;
    s_token_accum[s_token_accum_len] = 0;

    for (size_t i = 0; i < s_token_accum_len; ++i) {
        char c = s_token_accum[i];
        if ((c == '.' || c == '!' || c == '?') && i + 1 < s_token_accum_len &&
            (s_token_accum[i + 1] == ' ' || s_token_accum[i + 1] == '\n')) {
            sentence_t s;
            size_t take = i + 1;
            if (take > SENTENCE_MAX - 1) take = SENTENCE_MAX - 1;
            memcpy(s.text, s_token_accum, take);
            s.text[take] = 0;
            xQueueSend(s_sentence_q, &s, pdMS_TO_TICKS(100));
            size_t rest = s_token_accum_len >= (i + 2) ? s_token_accum_len - (i + 2) : 0;
            memmove(s_token_accum, s_token_accum + i + 2, rest);
            s_token_accum_len = rest;
            s_token_accum[s_token_accum_len] = 0;
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
            s_ws_connected = true;
            // Bring AirPlay 1 receiver online now that we have Wi-Fi + a
            // paired account. airplay_init is internally idempotent so
            // reconnects are safe. Service name uses the configured device
            // name so multiple devices show distinct entries in iOS.
            airplay_init(g_dev_config.device_name[0] ? g_dev_config.device_name : "OE Voice");
            // Re-report any alarm still ringing. oe_ws.c sends {type:'auth'}
            // before this event fires, so the session is established first.
            alarm_resend_fired();
            break;
        case OE_WS_EVT_CHAT_TOKEN:
            if (evt_turn_stale(evt)) break;
            accumulate_token(evt->text, evt->text_len);
            break;
        case OE_WS_EVT_CHAT_DONE:
            if (evt_turn_stale(evt)) break;
            // A `done` while the user is mid-command is NOT ours to act on —
            // it's the tail of an aborted turn (stop-ack, superseded streamer)
            // racing the new turn. Acting on it used to IDLE the UI and
            // airplay_resume() over the user's speech (repro: wake during
            // AirPlay → barge-in pauses music + sends stop → server acks with
            // a bare done → music resumed mid-dictation).
            if (s_in_utterance) break;
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
            break;
        case OE_WS_EVT_TTS_AUDIO_BEGIN:
            // Server-side streaming: about to receive synthesized PCM frames.
            // Enter SPEAKING once; the legacy per-sentence path isn't used.
            // Stale-turn begin (aborted reply racing a barge-in) must not
            // re-enter SPEAKING mid-capture of the new turn.
            if (evt_turn_stale(evt)) break;
            if (!leds_buttons_is_muted() && !s_stream_active) {
                s_stream_active  = true;
                s_stream_end_req = false;
                s_awaiting_reply = false;
                s_last_tts_frame_us = esp_timer_get_time();
                set_ui_state(UI_STATE_SPEAKING);
                for (uint8_t _i = 0; _i < WW_NUM_SLOTS; ++_i)
                    if (s_ww[_i]) wakeword_notify_speaking_began(s_ww[_i]);
                xvf3800_enable_amplifier(true);
                audio_io_start_playback();
            }
            break;
        case OE_WS_EVT_TTS_AUDIO:
            // One base64 PCM frame → write straight to I²S. Payload contract
            // with lib/voice-tts-stream.mjs: 16 kHz STEREO interleaved s16le
            // (CHANNELS=2, ffmpeg -ac 2), so the frame goes to write_pcm
            // as-is. Do NOT re-wrap it in a mono→stereo expander: 0.2.60 did
            // and it (a) double-expanded the already-stereo frames (half-
            // speed audio) and (b) blew the 4 KB websocket_task stack — the
            // panic-on-every-reply bug fixed in 0.2.61.
            if (evt_turn_stale(evt)) break;
            if (s_stream_active && evt->text && evt->text_len) {
                s_last_tts_frame_us = esp_timer_get_time();
                size_t olen = 0;
                if (mbedtls_base64_decode(s_pcm_frame, sizeof(s_pcm_frame), &olen,
                        (const unsigned char *)evt->text, evt->text_len) == 0 && olen >= 2) {
                    audio_io_write_pcm((const int16_t *)s_pcm_frame, olen / 2, 16000);
                }
            }
            break;
        case OE_WS_EVT_TTS_AUDIO_END:
            // All audio sent — finalize task drains the ring, then idles.
            if (evt_turn_stale(evt)) break;
            if (s_stream_active) s_stream_end_req = true;
            break;
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
            if (evt_turn_stale(evt)) break;
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
            break;
        case OE_WS_EVT_SERVER_CAPS: {
            // { type:'server_caps', turn_ids, tts_pause, stt_stream } — what
            // the connected server understands. Gates every NEW device→server
            // message so this firmware stays compatible with older servers.
            cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
            if (j) {
                s_caps_tts_pause  = cJSON_IsTrue(cJSON_GetObjectItem(j, "tts_pause"));
                s_caps_stt_stream = cJSON_IsTrue(cJSON_GetObjectItem(j, "stt_stream"));
                ESP_LOGI(TAG, "server caps: tts_pause=%d stt_stream=%d",
                         (int)s_caps_tts_pause, (int)s_caps_stt_stream);
                cJSON_Delete(j);
            }
            break;
        }
        case OE_WS_EVT_SET_CONVERSATION_MODE: {
            cJSON *j = cJSON_ParseWithLength(evt->text, evt->text_len);
            if (j) {
                cJSON *je = cJSON_GetObjectItem(j, "enabled");
                bool en = cJSON_IsTrue(je) || (cJSON_IsNumber(je) && je->valueint != 0);
                if (en != s_conversation_mode) {
                    s_conversation_mode = en;
                    ESP_LOGI(TAG, "conversation mode: %s", en ? "on" : "off");
                }
                cJSON_Delete(j);
            }
            break;
        }
        case OE_WS_EVT_DISCONNECTED:
            s_ws_connected = false;
            // Forget capabilities — the socket may reconnect to an older
            // server (rollback) that doesn't understand the newer messages.
            s_caps_tts_pause  = false;
            s_caps_stt_stream = false;
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
                xvf3800_enable_amplifier(false);
                airplay_note_amp_forced_off();
                audio_io_stop_playback();
                audio_io_flush_playback();
                s_stream_active = false;
                s_stream_end_req = false;
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
                // Raw-JSON message — turn check happens here, not in oe_ws.c.
                // A follow-up window for an aborted/prior turn must not open.
                const cJSON *jturn = cJSON_GetObjectItem(j, "turn_id");
                if (cJSON_IsString(jturn) && jturn->valuestring[0] &&
                    (!s_turn_id[0] || strcmp(jturn->valuestring, s_turn_id) != 0)) {
                    cJSON_Delete(j);
                    break;
                }
                const cJSON *jw = cJSON_GetObjectItem(j, "windowMs");
                int window_ms = (cJSON_IsNumber(jw) && jw->valueint > 0) ? jw->valueint : 5000;
                // Lock the slot of the turn that opened this follow-up so a
                // false-fire on a different wake-word can't reroute the
                // answer to a different user.
                s_followup_slot = s_active_slot;
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
                            oe_udplog_send("[ambient] same-marker restore -> resume");
                            ambient_resume();
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
            audio_io_stop_playback();
            audio_io_flush_playback();
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

static void tts_pcm_cb(const int16_t *pcm, size_t samples, uint32_t rate, void *user)
{
    (void)user;
    if (s_tts_stable_rate == 0 && rate > 0) {
        s_tts_stable_rate = rate;
        ESP_LOGI(TAG, "tts: rate locked at %u Hz", (unsigned)rate);
    }
    uint32_t effective_rate = s_tts_stable_rate > 0 ? s_tts_stable_rate : rate;
    audio_io_write_pcm(pcm, samples, effective_rate);
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
        if (s_stream_end_req && s_stream_active) {
            s_stream_end_req = false;
            for (int i = 0; i < 300; ++i) {           // up to ~15 s safety cap
                uint32_t used = 0, cap = 0;
                audio_io_get_playback_buf_stats(&used, &cap);
                if (used == 0) break;
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            vTaskDelay(pdMS_TO_TICKS(120));           // I²S DMA tail margin
            for (uint8_t _i = 0; _i < WW_NUM_SLOTS; ++_i)
                if (s_ww[_i]) wakeword_notify_speaking_ended(s_ww[_i]);
            xvf3800_enable_amplifier(false);
            airplay_note_amp_forced_off();
            s_stream_active = false;
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
            } else {
                set_ui_state(UI_STATE_IDLE);
            }
            airplay_resume();
        } else if (s_stream_active && !s_stream_end_req && !s_paused_for_barge) {
            // Stall watchdog: tts_audio_begin arrived but the stream went
            // silent with no tts_audio_end and the ring has fully drained.
            // Without this there was NO timeout on SPEAKING — a server crash
            // mid-stream on a healthy socket left the amp on (AEC suppressing
            // the mic) and every non-owner wake slot gated forever. Use the
            // same teardown as the WS-disconnect path. Suspended while a
            // barge verify holds the pacer paused (frames stop on purpose).
            int64_t last = s_last_tts_frame_us;
            uint32_t used = 0, cap = 0;
            audio_io_get_playback_buf_stats(&used, &cap);
            if (last != 0 && used == 0 &&
                esp_timer_get_time() - last > (int64_t)TTS_STREAM_STALL_TIMEOUT_MS * 1000) {
                ESP_LOGW(TAG, "tts stream stalled (%d ms, ring empty, no end) — tearing down SPEAKING",
                         TTS_STREAM_STALL_TIMEOUT_MS);
                oe_udplog_send("[tts] stream stall watchdog fired");
                for (uint8_t _i = 0; _i < WW_NUM_SLOTS; ++_i)
                    if (s_ww[_i]) wakeword_notify_speaking_ended(s_ww[_i]);
                xvf3800_enable_amplifier(false);
                airplay_note_amp_forced_off();
                audio_io_stop_playback();
                audio_io_flush_playback();
                s_stream_active = false;
                s_followup_pending_ms = 0;
                set_ui_state(UI_STATE_IDLE);
                airplay_resume();
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
                                    s.text, NULL, (int)s_active_slot, tts_pcm_cb, NULL, NULL, NULL);
        ESP_LOGI(TAG, "tts post -> %s, queue_remaining=%u",
                 esp_err_to_name(te), (unsigned)uxQueueMessagesWaiting(s_sentence_q));
        vTaskDelay(pdMS_TO_TICKS(200));
        if (uxQueueMessagesWaiting(s_sentence_q) == 0) {
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
        // Ambient is a "real teardown" case (see s_ambient_stop's comment) —
        // before 0.2.62 this callback skipped it, leaving the ambient task's
        // HTTP stream alive and, worse, the server's ambient session marker
        // intact, so the wake-mid-ambient resume logic would resurrect the
        // "muted away" ambient after the next turn. Stop the worker AND tell
        // the server so both halves of the session die together.
        if (s_ambient_active) {
            s_ambient_stop = true;
            oe_ws_send_ambient_stopped("mute");
        }
        audio_io_stop_playback();
        audio_io_flush_playback();
        // Cancel any in-flight server-pushed TTS stream (server halts on stop).
        s_stream_active  = false;
        s_stream_end_req = false;
        oe_ws_send_stop(g_dev_config.default_agent_id, s_turn_id);
        // Hard-drop any AirPlay session — physically muting a speaker
        // should disconnect the iOS sender, not just gag the output.
        airplay_stop();
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
        if (s_ambient_paused && !s_in_utterance && !s_awaiting_reply && !s_stream_active &&
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
                continue;
            }
        }

        if (!s_in_utterance) {
            // Set when THIS frame's fire came from the follow-up VAD-start
            // path — the only fire type that prepends pre-roll (a wake-word
            // fire must NOT: pre-roll would include the wake phrase itself).
            bool fired_from_followup = false;
            // Follow-up listening: if the server armed a window after its
            // last reply ended with a "?", treat any voice activity in this
            // frame as a wake fire so the user can answer without saying
            // the wake word again. Window expires silently if no speech detected.
            int64_t followup_until = get_followup_until_us();
            if (followup_until > 0) {
                int64_t now_us = esp_timer_get_time();
                if (now_us >= followup_until) {
                    set_followup_until_us(0);
                    // Window expired without speech — drop back to IDLE.
                    set_ui_state(UI_STATE_IDLE);
                } else if (frame_is_speech(frame, n)) {
                    ESP_LOGI(TAG, "follow-up: VAD-start (slot=%u), bypassing wake-word",
                             (unsigned) s_followup_slot);
                    s_active_slot = s_followup_slot;
                    set_followup_until_us(0);
                    // Fall through into the wake-fire actions below by
                    // emulating a committed pending_slot. The block right
                    // after sets fired/in_utterance based on this state.
                    pending_slot = s_followup_slot;
                    pending_prob = 255;
                    pending_age = 1;
                    fired_from_followup = true;
                }
            }

            // ── Speech barge-in (conversation mode): pause-then-verify ──────
            // Runs only while OUR streamed reply plays. Ambient/AirPlay keep
            // the wake word as their barge path — their audio isn't a
            // conversation. See the barge_state_t block at file scope for the
            // three-stage design (candidate → local verify → STT commit).
            if (s_conversation_mode && s_stream_active) {
                if (s_barge_state == BARGE_NONE && !s_paused_for_barge) {
                    uint32_t fe = frame_energy(frame, n);
                    // Rolling floor: EMA (α=1/8) of the reply+room energy as
                    // heard by the AEC-suppressed mic. The trigger is
                    // relative to it so pre-existing noise (TV) is absorbed
                    // into the baseline instead of firing candidates.
                    s_speak_floor = s_speak_floor
                        ? s_speak_floor - s_speak_floor / 8 + fe / 8 : fe;
                    uint64_t trigger = (uint64_t)s_speak_floor * BARGE_FLOOR_MULT;
                    if (trigger < VOICE_ENERGY_THRESHOLD) trigger = VOICE_ENERGY_THRESHOLD;
                    if ((uint64_t)fe > trigger) {
                        if (++s_barge_consec >= BARGE_CONSEC_FRAMES) {
                            s_barge_consec = 0;
                            // Mute FIRST (local, ~instant): pausing playback
                            // + dropping amp_en is what un-suppresses the mic
                            // for the verify. Then stall the server pacer.
                            // (The WS send can block up to 1 s worst-case on
                            // this task — the audible pause already happened,
                            // and verify timing keys off s_barge_started_us.)
                            audio_io_pause_playback();
                            xvf3800_enable_amplifier(false);
                            s_paused_for_barge = true;
                            s_barge_state = BARGE_VERIFYING;
                            s_barge_started_us = esp_timer_get_time();
                            s_barge_speech_ms = 0;
                            set_ui_state(UI_STATE_LISTENING);
                            if (s_caps_tts_pause) oe_ws_send_tts_pause(s_turn_id);
                            char bl[96];
                            snprintf(bl, sizeof(bl), "[barge] candidate fe=%u floor=%u",
                                     (unsigned)fe, (unsigned)s_speak_floor);
                            ESP_LOGI(TAG, "%s", bl); oe_udplog_send(bl);
                        }
                    } else {
                        s_barge_consec = 0;
                    }
                } else if (s_barge_state == BARGE_VERIFYING) {
                    int64_t now_us = esp_timer_get_time();
                    uint32_t since_ms = (uint32_t)((now_us - s_barge_started_us) / 1000);
                    // Skip the settle window: the mic gain steps up when
                    // amp_en drops and the first frames read artificially
                    // hot/cold. BARGE_SETTLE_MS is provisional — the [barge]
                    // logs measure the real step response in the field.
                    if (since_ms > BARGE_SETTLE_MS && frame_is_speech(frame, n)) {
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
                        // Pre-roll = the verify-window audio (all post-pause,
                        // speaker silent) so STT hears the interjection from
                        // its first word. Confirm takes ≥500 ms and pre-roll
                        // holds 400 ms — never includes pre-pause reply bleed.
                        s_capture_used = preroll_copy_out(s_capture_buf, PREROLL_SAMPLES);
                        capture_sat_logged = false;
                        s_in_utterance = true;
                        // Slot stays s_active_slot (the turn owner). Turn id
                        // is minted at stage-C commit; until then pause/
                        // resume flow control still names the CURRENT turn.
                    } else if (since_ms >= BARGE_VERIFY_WINDOW_MS) {
                        // False alarm — resume the reply where it paused.
                        char bl[64];
                        snprintf(bl, sizeof(bl), "[barge] false alarm (%ums speech) — resume",
                                 (unsigned)s_barge_speech_ms);
                        ESP_LOGI(TAG, "%s", bl); oe_udplog_send(bl);
                        s_barge_state = BARGE_NONE;
                        s_paused_for_barge = false;
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
                    }
                }
            }
            if (frame_best_slot >= 0 &&
                (pending_slot < 0 || frame_best_prob > pending_prob)) {
                pending_slot = frame_best_slot;
                pending_prob = frame_best_prob;
            }

            bool fired = false;
            if (pending_slot >= 0) {
                if (pending_age >= 1) {
                    // Held one full frame past first fire; commit.
                    s_active_slot = (uint8_t) pending_slot;
                    s_active_wake_prob = pending_prob;
                    fired = true;
                    // If a follow-up window is active, lock the slot back
                    // to the originating turn — a false-fire (or even a
                    // genuine fire) on a different wake-word during the
                    // window should still route the answer to the user
                    // who asked the question.
                    if (get_followup_until_us() > esp_timer_get_time() &&
                        s_active_slot != s_followup_slot) {
                        ESP_LOGI(TAG, "follow-up: slot %u wake fired, overriding to slot %u",
                                 (unsigned) s_active_slot, (unsigned) s_followup_slot);
                        s_active_slot = s_followup_slot;
                    }
                    set_followup_until_us(0);  // window closes once we commit
                    pending_slot = -1; pending_prob = 0; pending_age = 0;
                } else {
                    pending_age++;
                }
            }

            if (fired) {
                // A wake fire supersedes any speech-barge verify in flight:
                // release OUR playback pause so the barge-in cleanup below
                // (stop/flush) owns the audio engine outright.
                if (s_paused_for_barge) {
                    s_paused_for_barge = false;
                    s_barge_state = BARGE_NONE;
                    s_barge_capture = false;
                    audio_io_resume_playback();
                }
                // Visual ack FIRST. Everything below this point — barge-in
                // cleanup, WS stop send, ringbuffer flush — has variable
                // latency (the WS send is the main offender, ~50-500 ms
                // depending on socket state). Flipping the LED before any of
                // it gives the user immediate "we heard you" feedback so
                // they're not left staring at IDLE while we tidy up. The
                // alarm-dismiss branch below overrides this back to a
                // short LISTENING-then-IDLE flash on its own.
                set_ui_state(UI_STATE_LISTENING);

                // Alarm dismiss takes precedence over normal wake flow: if
                // any alarm is currently firing, treat the wake as a local
                // ack — stop the ring, send alarm_acked, skip STT/utterance
                // capture for this wake. No STT roundtrip means dismiss
                // still works when the server is unreachable.
                if (alarm_is_firing()) {
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
                    continue;
                }
                if (audio_io_playback_active()) {
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
                mint_turn_id();
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
                s_in_utterance = true;
            }
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
            vad_end_reason_t end_reason = VAD_END_NONE;
            vad_feed(s_vad, frame, n, &end_reason);
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
                                                   s_active_slot, s_active_wake_prob, s_turn_id);
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
        s_ambient_active = true;
        s_ambient_paused = false;        // fresh session starts playing
        strncpy(s_ambient_cur_marker, req.marker, sizeof(s_ambient_cur_marker) - 1);
        s_ambient_cur_marker[sizeof(s_ambient_cur_marker) - 1] = 0;
        s_ambient_stable_rate = 0;       // re-arm rate lock for this stream
        s_last_ambient_rate = 44100;
        if (req.volume >= 0 && req.volume <= 100) {
            s_pre_ambient_volume = audio_io_get_volume();
            audio_io_set_volume((uint8_t)req.volume);
        } else {
            s_pre_ambient_volume = -1;
        }
        set_ui_state(UI_STATE_AMBIENT);
        xvf3800_enable_amplifier(true);
        audio_io_start_playback();
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
        audio_io_stop_playback();
        audio_io_flush_playback();
        for (uint8_t i = 0; i < WW_NUM_SLOTS; ++i)
            if (s_ww[i]) wakeword_notify_speaking_ended(s_ww[i]);
        xvf3800_enable_amplifier(false);
        airplay_note_amp_forced_off();
        if (s_pre_ambient_volume >= 0) {
            audio_io_set_volume((uint8_t)s_pre_ambient_volume);
            s_pre_ambient_volume = -1;
        }
        s_ambient_active = false;
        s_ambient_paused = false;
        s_ambient_cur_marker[0] = 0;
        set_ui_state(UI_STATE_IDLE);
        oe_udplog_send("[ambient] STOP");
        // Loop: a queued (preempting) request is picked up immediately next
        // iteration; otherwise we block idle in xQueueReceive.
    }
}

// Un-pause ambient: the live stream stayed open + reading (discarding) while a
// command ran; now play it again. Re-enables the amp + playback (the barge-in
// flushed/stopped it, and a TTS reply may have stopped it). The ambient task is
// still sitting in oe_tts_post — clearing s_ambient_paused makes its http_evt
// start decoding the live bytes again, so the rain picks up from "now". No-op
// unless we're actually paused on a live session.
static void ambient_resume(void)
{
    if (!s_ambient_paused || !s_ambient_active) { s_ambient_paused = false; return; }
    xvf3800_enable_amplifier(true);
    audio_io_start_playback();
    s_ambient_paused = false;
    oe_udplog_send("[ambient] RESUME");
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
    if (speaking && s_ambient_active) {
        s_ambient_paused = true;
        oe_udplog_send("[ambient] PAUSE (alarm)");
    }
    for (uint8_t i = 0; i < WW_NUM_SLOTS; ++i) {
        if (!s_ww[i]) continue;
        if (speaking) wakeword_notify_speaking_began(s_ww[i]);
        else          wakeword_notify_speaking_ended(s_ww[i]);
    }
}

static void alarm_amp_cb(bool enable)
{
    xvf3800_enable_amplifier(enable);
    if (!enable) airplay_note_amp_forced_off();
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

static void boot_operational(void)
{
    nvs_creds_get_server(g_dev_config.server_url, sizeof(g_dev_config.server_url));
    nvs_creds_get_token(g_dev_config.token, sizeof(g_dev_config.token));
    nvs_creds_get_device_name(g_dev_config.device_name, sizeof(g_dev_config.device_name));
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
    if (!s_preroll_buf) ESP_LOGW(TAG, "preroll alloc failed — follow-up onset capture disabled");

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
        char bl[128];
        snprintf(bl, sizeof(bl), "[boot] reset=%s heap_int=%luKB heap_psram=%luKB",
                 reset_reason_str(esp_reset_reason()),
                 (unsigned long)(esp_get_free_heap_size() / 1024),
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
    // hb itself needs ~4 KB now that it runs the per-minute stack-hwm
    // dump: 256-byte line buffer + ESP_LOGI (vprintf) + xTaskGetHandle +
    // uxTaskGetStackHighWaterMark traversal added enough stack pressure
    // to overflow the original 2 KB allocation (observed 0.2.34 boot).
    xTaskCreate(heartbeat_task, "hb", 6144, NULL, 1, NULL);
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
            char hbline[160];
            // pcm_drop is the running total of playback samples lost to a
            // full ring (audio_io_write_pcm). Nonzero = audible skip
            // happened; steadily climbing = server pacing outrunning the
            // ring. Cumulative on purpose — a rare drop stays visible.
            snprintf(hbline, sizeof(hbline), "[hb] alive tick=%lu rssi=%d heap_int=%luKB cap_sps=%lu pcm_drop=%lu",
                     (unsigned long)n++, rssi_now,
                     (unsigned long)(esp_get_free_heap_size() / 1024),
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
            audio_io_get_playback_buf_stats(&pcm_used, &pcm_cap);
            // bytes/sec and errs/sec scaled from the actual interval so a
            // late wake-up of the hb task doesn't lie about rates.
            const uint32_t bytes_per_sec = (interval_ms > 0) ? (bytes_delta * 1000u / interval_ms) : 0;
            const uint32_t errs_per_sec_x10 = (interval_ms > 0) ? (errs_delta * 10000u / interval_ms) : 0;
            char as[256];
            snprintf(as, sizeof(as),
                "[ambient-stats] rssi=%d bytes/s=%lu dec_errs/s=%lu.%lu "
                "pcm_rb=%lu/%lu heap_int=%lu heap_psram=%lu heap_int_min=%lu",
                rssi,
                (unsigned long)bytes_per_sec,
                (unsigned long)(errs_per_sec_x10 / 10),
                (unsigned long)(errs_per_sec_x10 % 10),
                (unsigned long)pcm_used,
                (unsigned long)pcm_cap,
                (unsigned long)(esp_get_free_heap_size() / 1024),
                (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
                (unsigned long)(esp_get_minimum_free_heap_size() / 1024));
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
    esp_netif_init();
    esp_event_loop_create_default();

    // Init order matters: I²C bus + XVF acknowledgement BEFORE we start
    // clocking on I²S. The Seeed reSpeaker XVF3800 i2s firmware needs the
    // host to be reachable on I²C before it'll stream audio on DIN — the
    // working third-party project (gillespinault/respeaker-xvf3800-vad)
    // sequences I²C init → version read → I²S init, in that order.
    xvf3800_init();
    leds_buttons_init(mute_change_cb);
    audio_io_init();

    if (nvs_creds_is_provisioned()) {
        ESP_LOGI(TAG, "provisioned — operational boot");
        boot_operational();
    } else {
        ESP_LOGI(TAG, "unprovisioned — provisioning boot");
        boot_provisioning();
    }
}
