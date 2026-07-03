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
 *   airplay_is_streaming()    — true while iOS is actively pushing.
 *   airplay_deinit()          — full tear-down.
 *
 * Events propagated to FreeRTOS event group `g_dev_events`:
 *   DEV_EVT_AIRPLAY_PLAY   — iOS started a stream
 *   DEV_EVT_AIRPLAY_STOP   — iOS stopped a stream
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
