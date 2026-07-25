/*
 * airplay.h — public API for the OE voice-device AirPlay 1 receiver.
 *
 * Lifecycle:
 *   airplay_init(name)        — once, after Wi-Fi+WS auth succeed.
 *                               Reads the netif IP/MAC and brings up
 *                               the RTSP listener + _raop._tcp mDNS
 *                               service. Audio packets land in
 *                               audio_io_write_pcm via an internal
 *                               callback.
 *   airplay_pause()           — temporarily mute output (wake-word
 *                               fired, OE is about to TTS).
 *   airplay_resume()          — resume after TTS-done.
 *   airplay_stop()            — explicit stop (mute switch, "stop
 *                               music" intent). Drops the RTSP session.
 *   airplay_is_streaming()    — true while an iOS RTSP session is active.
 *   airplay_is_playing()      — true only while that source is playing and
 *                               no AirPlay-local pause/hold masks it.
 *   airplay_deinit()          — full tear-down.
 *
 * Session and audible-source state are observable via airplay_is_streaming()
 * and airplay_is_playing(); there is no event-group side channel (the old
 * g_dev_events layer was removed 2026-07-04).
 *
 * License: MIT.
 */
#ifndef OE_AIRPLAY_H_
#define OE_AIRPLAY_H_

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t airplay_init(const char *service_name);
void      airplay_deinit(void);

void airplay_pause(void);
void airplay_resume(void);
void airplay_stop(void);
// Immediate, network-free destructive half used while physical mute owns the
// firmware lifecycle. Clears both wake/user pause latches and drops local PCM.
void airplay_mute_local(void);
// Potentially blocking DACP request. Call only after releasing lifecycle
// mutexes; local mute/stop must already have made the speaker safe.
void airplay_send_stop(void);

// Non-destructive provisional wake hold. It only gates incoming AirPlay PCM:
// no ring flush and no DACP command. The flag is independent of wake/user
// pause state, so RAOP PLAY/RESUME, airplay_resume(), and
// airplay_user_resume() cannot release it.
void airplay_verify_hold(void);
void airplay_verify_release(void);
bool airplay_verify_is_held(void);

// Main firmware may force the shared XVF amp off after TTS/alarm/ambient audio.
// Tell AirPlay so its lazy amp latch re-enables the speaker on the next RTP PCM.
void airplay_note_amp_forced_off(void);

// Send a DACP remote-control command back to the source (iPhone / iPad / Mac
// running the AirPlay session). The raop_sink library uses the active-remote
// token captured at RTSP setup to POST /ctrl-int/1/<cmd> at the source's
// DACP HTTP port. No-op if no session is active. Useful for "skip" / "back"
// voice commands.
void airplay_next(void);
void airplay_prev(void);

// User-initiated pause/resume (as opposed to airplay_pause/resume which are
// the wake-handling transient pause). Sends DACP pause/play so iOS shows the
// session as paused in Music.app and stops pushing RTP — otherwise iOS
// keeps streaming and packets pile up in the local ringbuffer. No-op when
// no AirPlay session is streaming.
void airplay_user_pause(void);
void airplay_user_resume(void);

bool airplay_is_streaming(void);
bool airplay_is_playing(void);

/**
 * @brief     Refresh the mDNS instance name without tearing down the
 *            RAOP context. iOS picks up the new label within ~5 s.
 *            Safe to call mid-stream; the active AirPlay session is
 *            unaffected because raop doesn't reference the name at
 *            runtime — it was only used to seed the mDNS announce.
 */
void airplay_set_name(const char *name);

#ifdef __cplusplus
}
#endif

#endif
