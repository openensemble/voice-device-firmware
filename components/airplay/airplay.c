/*
 * airplay.c — OE voice-device AirPlay 1 driver glue.
 *
 *   - Owns the boot sequence: mdns_init → raop_sink_init → fetch
 *     netif IP/MAC → raop_sink_start_now → done.
 *   - Bridges raop's PCM callback to audio_io_write_pcm at 44.1 kHz.
 *   - Bridges raop's cmd callback to FreeRTOS event bits so main.c
 *     can suspend the wake-word loop only while audio is actually
 *     streaming (no need to gate when iOS isn't even paired).
 *   - Implements `_gettime_ms_()` for raop's platform.h.
 *
 * License: MIT.
 */

#include <string.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_random.h"
#include "esp_heap_caps.h"
#include "mdns.h"

#include "airplay.h"
#include "audio_io.h"
#include "xvf3800_ctrl.h"

#include "raop/platform.h"
#include "raop/raop.h"
#include "raop/raop_sink.h"

static const char *TAG = "airplay";

static atomic_bool s_streaming = ATOMIC_VAR_INIT(false);
// Source transport state, distinct from an open RTSP session. SETUP keeps the
// session alive even while iOS is paused; STREAM/PLAY/PCM mark it playing.
static atomic_bool s_playing = ATOMIC_VAR_INIT(false);
static atomic_bool s_paused_for_wake = ATOMIC_VAR_INIT(false);
// Voice-pause is local-only — we keep accepting iOS RTP but drop it in
// on_pcm. iOS never sees a DACP pause, so its RTSP session stays alive
// indefinitely and the user can always resume via voice. With DACP pause
// (the prior approach), iOS would tear down RTSP after a few minutes →
// airplay_user_resume hit `!s_streaming` and silently no-op'd. iPhone
// Music app shows "Playing" while voice-paused; acceptable trade-off
// for reliable voice control.
static atomic_bool s_paused_by_user = ATOMIC_VAR_INIT(false);
// Provisional verify-gate ownership. This is intentionally separate from the
// established wake/user pause flags and their lifecycle paths: only
// airplay_verify_release() (or full component teardown) may clear this one.
// No flush or DACP command is associated with the verify hold.
static atomic_bool s_verify_held = ATOMIC_VAR_INIT(false);
// AirPlay-owned amplifier state. Without this, the amp's enabled/disabled
// state is whatever the last TTS / alarm / ambient call left it in — which
// is "off" anytime a wake-word reply finished (main.c:932 disables on TTS
// drain). First AirPlay session after a TTS conversation then plays into
// a muted speaker; user perceives "device asleep / no sound".
static atomic_bool s_amp_on = ATOMIC_VAR_INIT(false);
// Serializes the final source-owner check with the physical XVF amp write.
// Atomics alone leave a check→enable window where verify/mute/RAOP STOP can
// publish after on_pcm's check but before its I2C enable.
static SemaphoreHandle_t s_state_mutex = NULL;

// Jitter ring buffer allocated at RAOP_SETUP and freed at RAOP_STOP.
// rtp.c sizes each frame slot at 1408 bytes (352 samples * 4 bytes
// stereo s16) and caps BUFFER_FRAMES_MAX at ~1252 = ~10 s of audio.
// We allocate 1.5 MB ≈ 1100 frames ≈ 8.8 s so clock skew between
// iOS's 44.1 kHz and the XVF3800-mastered 48 kHz I²S has plenty of
// headroom to absorb before silence-insertion ("robotic") kicks in.
// Lives in PSRAM — internal RAM doesn't have anywhere near this room.
static uint8_t *s_rtp_buffer      = NULL;
static size_t   s_rtp_buffer_size = 0;
#define OE_AIRPLAY_RTP_BUFFER_BYTES (1536 * 1024)

static bool state_lock(void)
{
    return !s_state_mutex ||
           xSemaphoreTake(s_state_mutex, portMAX_DELAY) == pdTRUE;
}

static void state_unlock(void)
{
    if (s_state_mutex) xSemaphoreGive(s_state_mutex);
}

static void amp_disable_locked(void)
{
    if (atomic_load(&s_amp_on)) {
        xvf3800_enable_amplifier(false);
    }
    atomic_store(&s_amp_on, false);
}

/* ------------------------------------------------------------------------ */
/* platform.h requires this symbol — wall-clock ms since boot is fine for   */
/* raop's NTP-ish timing. esp_timer is monotonic-microseconds since boot.   */
/* ------------------------------------------------------------------------ */
u32_t _gettime_ms_(void)
{
    return (u32_t)(esp_timer_get_time() / 1000ULL);
}

/* ------------------------------------------------------------------------ */
/* mbedTLS 3.x RNG callback adapter — used by rsa_apply() in raop.c for     */
/* blinding during the AirPlay 1 RSA key-wrap dance. Real entropy from the */
/* SoC's hardware RNG; signature shaped to match `mbedtls_*` f_rng arg.    */
/* ------------------------------------------------------------------------ */
int esp_random_wrap(void *p, unsigned char *buf, size_t len)
{
    (void)p;
    esp_fill_random(buf, len);
    return 0;
}

/* ------------------------------------------------------------------------ */
/* PCM data path: raop hands us decoded 44.1 kHz s16 stereo frames. We     */
/* drop them while any AirPlay pause/verify owner holds the source.         */
/* audio_io's write_pcm needs to learn the 44100 source rate; the 147:160  */
/* upsampler lives in audio_io.c.                                          */
/* ------------------------------------------------------------------------ */
static void on_pcm(const u8_t *data, size_t len, u32_t playtime)
{
    (void)playtime;
    // An arriving PCM frame is definitive evidence that the source transport
    // is playing, even when a local hold below intentionally drops it.
    if (!atomic_load(&s_streaming)) return;
    atomic_store(&s_playing, true);
    // airplay_pause() and airplay_verify_hold() silence music without
    // tearing down the RTSP session. The verify flag is checked separately
    // so no normal wake/user/RAOP resume can release a provisional gate.
    if (atomic_load(&s_verify_held))      return;
    if (atomic_load(&s_paused_for_wake)) return;
    if (atomic_load(&s_paused_by_user)) return;
    // len is total bytes of int16 interleaved L/R PCM.
    size_t written = audio_io_write_pcm(
        (const int16_t *)data, len / sizeof(int16_t), RAOP_SAMPLE_RATE);
    if (written == 0) return;

    // A verify/command pause or destructive writer fence can arrive while the
    // ring send above is blocked. Revalidate every owner only after a frame was
    // actually admitted; a stale callback must never re-enable the amplifier
    // after ACCEPT/mute has fenced and flushed it.
    if (!atomic_load(&s_streaming) ||
        atomic_load(&s_verify_held) ||
        atomic_load(&s_paused_for_wake) ||
        atomic_load(&s_paused_by_user) ||
        audio_io_is_paused()) {
        return;
    }
    if (!atomic_load(&s_amp_on) && state_lock()) {
        // Re-check under the same mutex used by verify hold, destructive mute,
        // forced-off notification and RAOP STOP. The physical write and source
        // latch now form one transition.
        if (!atomic_load(&s_amp_on) &&
            atomic_load(&s_streaming) &&
            !atomic_load(&s_verify_held) &&
            !atomic_load(&s_paused_for_wake) &&
            !atomic_load(&s_paused_by_user) &&
            !audio_io_is_paused() &&
            xvf3800_enable_amplifier(true) == ESP_OK) {
            atomic_store(&s_amp_on, true);
        }
        state_unlock();
    }
}

static void stop_local(bool clear_wake_pause)
{
    if (state_lock()) {
        atomic_store(&s_streaming, false);
        atomic_store(&s_playing, false);
        if (clear_wake_pause) {
            atomic_store(&s_paused_for_wake, false);
        }
        atomic_store(&s_paused_by_user, false);
        amp_disable_locked();
        state_unlock();
    }
    audio_io_flush_music();
}

/* ------------------------------------------------------------------------ */
/* Control event path: SETUP/PLAY/STOP transitions flip our streaming flag */
/* (airplay_is_streaming), which main.c and agc_freeze_task poll directly. */
/* ------------------------------------------------------------------------ */
static bool on_cmd(raop_event_t event, va_list args)
{
    switch (event) {
        case RAOP_SETUP: {
            // Upstream contract: caller passes `(buffer_out, size_out)` so
            // the cmd_cb hands back the jitter-buffer storage rtp_init will
            // chop into per-frame slots. If we don't fill these, rtp.c
            // falls through to per-frame malloc() of ~375 × 1.4 KB from
            // the default heap, which on this build's internal heap
            // pushes ALACDecoder::Init's calloc into kALAC_MemFullError.
            uint8_t **buf_out = va_arg(args, uint8_t **);
            size_t   *size_out = va_arg(args, size_t *);
            if (!s_rtp_buffer) {
                s_rtp_buffer = (uint8_t *)heap_caps_malloc(
                    OE_AIRPLAY_RTP_BUFFER_BYTES,
                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                s_rtp_buffer_size = s_rtp_buffer ? OE_AIRPLAY_RTP_BUFFER_BYTES : 0;
            }
            if (buf_out)  *buf_out  = s_rtp_buffer;
            if (size_out) *size_out = s_rtp_buffer_size;
            ESP_LOGI(TAG, "RAOP_SETUP: handed %zu B PSRAM jitter buffer at %p",
                     s_rtp_buffer_size, s_rtp_buffer);
            atomic_store(&s_streaming, true);
            atomic_store(&s_playing, false);
            audio_io_start_playback();
            break;
        }
        case RAOP_STREAM:
            ESP_LOGI(TAG, "event=%d -> stream armed", event);
            atomic_store(&s_streaming, true);
            atomic_store(&s_playing, false);
            audio_io_start_playback();
            // Defensive: if a prior voice "pause" command left any local pause flag
            // stuck on, clear it now. The whole-turn wake latch is owned by
            // airplay_pause()/airplay_resume() and must survive a replacement
            // RTSP session arriving while the accepted turn is still active.
            audio_io_resume_playback();
            atomic_store(&s_paused_by_user, false);
            break;

        case RAOP_PLAY:
        case RAOP_RESUME:
            ESP_LOGI(TAG, "event=%d -> playing", event);
            atomic_store(&s_streaming, true);
            atomic_store(&s_playing, true);
            audio_io_start_playback();
            // Same stale normal/user-pause cleanup as STREAM. Neither the
            // whole-turn wake latch nor independent verify hold is ours.
            audio_io_resume_playback();
            atomic_store(&s_paused_by_user, false);
            break;

        case RAOP_PAUSE:
        case RAOP_FLUSH:
            ESP_LOGI(TAG, "event=%d -> pause/flush", event);
            atomic_store(&s_playing, false);
            // iOS-side pause (user tapped pause in iPhone Music app, or DACP
            // pause we just sent came back as FLUSH). Flush our local music
            // PCM ringbuffer so the speaker silences within ~10 ms instead
            // of finishing the already-decoded audio.
            // Keep s_streaming true so the RTSP session stays open.
            audio_io_flush_music();
            break;

        case RAOP_STOP:
        case RAOP_STALLED:
            ESP_LOGI(TAG, "event=%d -> stop", event);
            // Keep the whole-turn wake pause: a replacement AirPlay session
            // arriving before the assistant turn ends must still be silent.
            stop_local(false);
            break;

        case RAOP_VOLUME: {
            // raop.c maps iOS dB (-30..0, -144=mute) to 0.0..1.0 before
            // firing the callback. Variadic floats are promoted to double.
            // Deliberately not persisted to NVS: AirPlay volume is a
            // session-level override; the NVS value tracks the voice/web
            // "intent" volume the device boots back to.
            float v = (float)va_arg(args, double);
            int pct = (int)(v * 100.0f + 0.5f);
            if (pct < 0) pct = 0;
            if (pct > 100) pct = 100;
            ESP_LOGI(TAG, "RAOP_VOLUME -> %d%%", pct);
            audio_io_set_volume((uint8_t)pct);
            break;
        }

        default:
            // METADATA / PROGRESS / etc — log only.
            ESP_LOGD(TAG, "event=%d (ignored)", event);
            break;
    }
    return true;
}

/* ------------------------------------------------------------------------ */
/* mDNS initialization — IDF's `espressif/mdns` is the canonical service.  */
/* raop.c calls mdns_service_add(..., "_raop", "_tcp", ...) itself, so we  */
/* just need mdns_init + hostname before raop_create runs.                 */
/* ------------------------------------------------------------------------ */
static esp_err_t airplay_mdns_bootstrap(const char *hostname)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(err));
        return err;
    }
    char host[64];
    snprintf(host, sizeof(host), "%s", hostname ? hostname : "oe-voice");
    // Strip whitespace/punctuation that breaks mDNS hostnames.
    for (char *p = host; *p; p++) {
        if (*p == ' ' || *p == '.' || *p == '/') *p = '-';
    }
    mdns_hostname_set(host);
    mdns_instance_name_set(hostname ? hostname : "OE Voice");
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Public API                                                               */
/* ------------------------------------------------------------------------ */
esp_err_t airplay_init(const char *service_name)
{
    static bool inited = false;
    if (inited) return ESP_OK;

    if (!s_state_mutex) s_state_mutex = xSemaphoreCreateMutex();
    if (!s_state_mutex) return ESP_ERR_NO_MEM;

    raop_sink_init(on_cmd, on_pcm);

    if (airplay_mdns_bootstrap(service_name) != ESP_OK) {
        // Non-fatal — keep going, mdns_service_add inside raop.c will
        // surface its own error.
    }

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        ESP_LOGE(TAG, "no STA netif yet — call airplay_init AFTER Wi-Fi up");
        return ESP_ERR_INVALID_STATE;
    }
    esp_netif_ip_info_t ip_info = {0};
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK || ip_info.ip.addr == 0) {
        ESP_LOGE(TAG, "no IP yet — call airplay_init AFTER DHCP");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, mac);

    raop_sink_start_now(ip_info.ip.addr, mac,
                        service_name ? service_name : "OE Voice (AirPlay)");
    inited = true;
    ESP_LOGI(TAG, "airplay_init done");
    return ESP_OK;
}

void airplay_deinit(void)
{
    raop_sink_deinit();
    if (state_lock()) {
        atomic_store(&s_streaming, false);
        atomic_store(&s_playing, false);
        atomic_store(&s_paused_for_wake, false);
        atomic_store(&s_paused_by_user, false);
        atomic_store(&s_verify_held, false);
        amp_disable_locked();
        state_unlock();
    }
    if (s_rtp_buffer) {
        heap_caps_free(s_rtp_buffer);
        s_rtp_buffer = NULL;
        s_rtp_buffer_size = 0;
    }
}

void airplay_pause(void)
{
    // This is a whole-turn latch, not session state. Set it before checking
    // the current session so an AirPlay connection that arrives mid-turn is
    // silent until the matching resume.
    if (state_lock()) {
        atomic_store(&s_paused_for_wake, true);
        amp_disable_locked();
        state_unlock();
    }
    if (!atomic_load(&s_streaming)) return;
    audio_io_flush_music();
    ESP_LOGI(TAG, "paused for wake");
}

void airplay_resume(void)
{
    // Always release the whole-turn latch, even if the old session ended.
    // A future session must not inherit a completed conversation's pause.
    if (state_lock()) {
        atomic_store(&s_paused_for_wake, false);
        // Force a fresh owner check on the next successfully queued PCM.
        atomic_store(&s_amp_on, false);
        state_unlock();
    }
    if (!atomic_load(&s_streaming)) return;
    audio_io_start_playback();
    ESP_LOGI(TAG, "resumed after wake");
}

void airplay_verify_hold(void)
{
    if (state_lock()) {
        atomic_store(&s_verify_held, true);
        // Close on_pcm's check→enable race inside the component. Main also
        // forces the shared amp off for non-AirPlay DMA tails.
        amp_disable_locked();
        state_unlock();
    }
    ESP_LOGI(TAG, "held for wake verification");
}

void airplay_verify_release(void)
{
    if (state_lock()) {
        atomic_store(&s_verify_held, false);
        state_unlock();
    }
    ESP_LOGI(TAG, "released after wake verification");
}

bool airplay_verify_is_held(void)
{
    return atomic_load(&s_verify_held);
}

void airplay_note_amp_forced_off(void)
{
    if (!state_lock()) return;
    // Main calls this only when it owns a physical force-off. Repeat the
    // disable if AirPlay raced that write and re-enabled in between.
    amp_disable_locked();
    state_unlock();
}

void airplay_send_stop(void)
{
    struct raop_ctx_s *ctx = raop_sink_ctx();
    if (ctx) raop_cmd(ctx, RAOP_STOP, NULL);
}

void airplay_mute_local(void)
{
    // Physical mute retires both user- and whole-turn pause latches. A future
    // session after unmute must not inherit a turn that mute destroyed.
    stop_local(true);
}

void airplay_stop(void)
{
    // Silence locally before a potentially slow/unreachable DACP controller.
    // Keep the whole-turn pause for the normal voice "stop music" path.
    stop_local(false);
    airplay_send_stop();
}

void airplay_next(void)
{
    if (!atomic_load(&s_streaming)) return;
    struct raop_ctx_s *ctx = raop_sink_ctx();
    if (ctx) raop_cmd(ctx, RAOP_NEXT, NULL);
    // raop_cmd's RAOP_NEXT path also clears any audio in-flight, but flush
    // ours too so the brief silence on track-change isn't filled with the
    // tail of the previous track from our jitter buffer.
    audio_io_flush_music();
    // The wake that brought us here ran the barge-in path which set
    // s_paused_for_wake — clear it so the next RTP from iOS actually reaches
    // the speaker. Without this, on_pcm drops every packet of the new track.
    atomic_store(&s_paused_for_wake, false);
}

void airplay_prev(void)
{
    if (!atomic_load(&s_streaming)) return;
    struct raop_ctx_s *ctx = raop_sink_ctx();
    if (ctx) raop_cmd(ctx, RAOP_PREV, NULL);
    audio_io_flush_music();
    atomic_store(&s_paused_for_wake, false);
}

void airplay_user_pause(void)
{
    if (!atomic_load(&s_streaming)) return;
    // Hybrid pause: DACP-tell iOS to pause (so the iPhone Music app UI
    // shows "Paused", and iPhone-side tap-pause/tap-play stay meaningful
    // to the user), AND set the local mute so audio stops instantly
    // regardless of RTP timing. The local flag is also a safety net for
    // stuck wake-pause states (s_paused_for_wake never clearing on fast
    // paths — see voice-fastpath-replaces-state memory).
    if (state_lock()) {
        atomic_store(&s_paused_by_user, true);
        amp_disable_locked();
        state_unlock();
    }
    audio_io_flush_music();
    struct raop_ctx_s *ctx = raop_sink_ctx();
    if (ctx) raop_cmd(ctx, RAOP_PAUSE, NULL);
    ESP_LOGI(TAG, "user-paused (DACP pause sent + local mute)");
}

void airplay_user_resume(void)
{
    if (!atomic_load(&s_streaming)) return;
    // Clear both pause flags. s_paused_for_wake is set by the wake barge-in
    // path; its usual clearer (post-TTS in main.c) doesn't fire for fast-path
    // voice intents like "play" because they short-circuit the chat pipeline
    // (replaces=true). s_paused_by_user is set by airplay_user_pause above.
    if (state_lock()) {
        atomic_store(&s_paused_for_wake, false);
        atomic_store(&s_paused_by_user, false);
        atomic_store(&s_amp_on, false);
        state_unlock();
    }
    // Belt-and-suspenders DACP nudge — covers the legacy case where iOS
    // genuinely paused (e.g. user paused via iPhone Music app then said
    // "<wake word>, play"). Safe no-op when iOS is already streaming.
    struct raop_ctx_s *ctx = raop_sink_ctx();
    if (!ctx) return;
    raop_cmd(ctx, RAOP_RESUME, NULL);  // "playresume"
    raop_cmd(ctx, RAOP_PLAY, NULL);    // "play"
    ESP_LOGI(TAG, "user-resumed");
}

bool airplay_is_streaming(void)
{
    return atomic_load(&s_streaming);
}

bool airplay_is_playing(void)
{
    return atomic_load(&s_streaming) &&
           atomic_load(&s_playing) &&
           !atomic_load(&s_paused_for_wake) &&
           !atomic_load(&s_paused_by_user) &&
           !atomic_load(&s_verify_held);
}

void airplay_set_name(const char *name)
{
    if (!name || !name[0]) return;
    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    // Match the instance-name format raop.c uses at raop_create:
    //   "<MAC>@<friendly>" — keeps Bonjour resource pathing stable.
    char inst[96];
    snprintf(inst, sizeof(inst), "%02X%02X%02X%02X%02X%02X@%s",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], name);

    // Goodbye + hello sequence. mdns_service_instance_name_set alone
    // only refreshes the SRV/TXT records — iOS treats that as a cache
    // touch and keeps showing the previously-cached instance label
    // until the record TTL expires (often 60+ min). Removing and
    // re-adding the service broadcasts a TTL=0 "goodbye" for the old
    // name, which invalidates iOS's mDNS cache immediately, then a
    // fresh "hello" for the new name. Picker updates within ~5 s.
    // The TCP RTSP/RAOP socket is independent of this mDNS dance, so
    // an active music stream is unaffected.
    mdns_service_remove("_raop", "_tcp");
    const mdns_txt_item_t txt[] = {
        {"am", "airesp32"},
        {"tp", "UDP"},
        {"sm","false"}, {"sv","false"},
        {"ek","1"},     {"et","0,1"},
        {"md","0,1,2"}, {"cn","0,1"},
        {"ch","2"},     {"ss","16"},
        {"sr","44100"}, {"vn","3"},
        {"txtvers","1"},
    };
    esp_err_t e = mdns_service_add(
        inst, "_raop", "_tcp", 5000,
        (mdns_txt_item_t *)txt, sizeof(txt) / sizeof(txt[0]));
    if (e == ESP_OK) {
        ESP_LOGI(TAG, "mDNS _raop._tcp re-advertised as \"%s\"", inst);
    } else {
        ESP_LOGW(TAG, "mdns_service_add (rename) failed: %s",
                 esp_err_to_name(e));
    }
    // Refresh the host's instance label too so iOS shows the new name
    // anywhere else it's surfaced (e.g. hostname-based lookups).
    mdns_instance_name_set(name);
}
