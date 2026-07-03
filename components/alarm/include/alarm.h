#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * Device-side alarm system.
 *
 * Owns the local alarm registry (in PSRAM + persisted in NVS), schedules
 * esp_timers against wall-clock epoch ms (requires SNTP-synced time), and
 * plays the chime+TTS ring cycle until dismissed.
 *
 * v1 scope:
 *   - Timer alarms (server pushes a trigger_at_ms + pre-synthesized
 *     announcement MP3 cached server-side; device fetches once and stores
 *     in PSRAM).
 *   - Audio buffer is RAM-only; reboot loses it but NVS metadata survives,
 *     so post-reboot alarms ring chime-only ("user infers from context").
 *
 * Out of scope for v1 (deferred):
 *   - Wall-clock alarms ("wake me at 7am") — same machinery, no TTS, future.
 *   - User-uploaded chime via LittleFS — defaults to built-in tone for now.
 *   - Multi-alarm combined audio sequencing — v1 plays each alarm's audio
 *     in sequence within a ring cycle.
 */

// Called once at boot, AFTER nvs_flash_init. Loads any alarms persisted in
// NVS. Scheduling happens lazily once SNTP-synced wall-clock is available;
// if SNTP isn't ready, this function returns and an internal retry task
// schedules persisted alarms as soon as the clock acquires.
esp_err_t alarm_init(void);

// Arm a new alarm. Called from main.c's WS event dispatcher when an
// alarm_arm payload arrives. Stores in NVS + schedules esp_timer. The audio
// buffer is copied into PSRAM and owned by the alarm subsystem from here
// on; the caller must NOT free it.
//
// trigger_at_ms is wall-clock epoch milliseconds. If it's already past
// when this is called (e.g. reboot replay), the alarm fires immediately.
esp_err_t alarm_arm(const char *id,
                    const char *label,
                    int64_t trigger_at_ms,
                    uint8_t *audio_mp3,
                    size_t audio_mp3_len,
                    const char *type);

// Cancel a pending alarm by id. No-op if not found. Stops the ring loop
// if this alarm was currently firing.
esp_err_t alarm_disarm(const char *id);

// Stop currently-firing alarm(s). If id is non-NULL, stops only that one;
// NULL stops every currently-firing alarm on this device.
void alarm_stop(const char *id);

// True if any alarm's ring loop is currently active. Wake handler checks
// this to decide whether to treat a wake fire as a local dismiss.
bool alarm_is_firing(void);

// Re-send alarm_fired for every currently-ringing alarm. Call on WS
// (re)connect: alarm_fired is fire-and-forget, so a WS blip at the fire
// instant otherwise leaves the server thinking the alarm never rang.
// Idempotent server-side.
void alarm_resend_fired(void);

// Local dismiss path — invoked by the wake handler when wake fires while
// alarm_is_firing() is true. Stops the ring, sends alarm_acked over WS,
// removes the alarm(s) from NVS, and suppresses the normal STT/utterance
// pipeline for this wake event (returns true if a dismiss happened so the
// caller knows to suppress the rest of the wake flow).
bool alarm_handle_local_dismiss(void);

// Coordination hooks the alarm subsystem invokes around its audio cycle.
// `speaking` callback is called with true when the ring is about to start
// playing audio (so main.c can wakeword_notify_speaking_began for AEC
// residual handling) and false when the ring session ends.
//
// `amp` callback toggles the speaker amp (xvf3800_enable_amplifier) — the
// alarm component shouldn't have to know about the specific carrier.
//
// Registering NULL is allowed (no-op); if either is unset the corresponding
// hook simply isn't called. Wake-word slots and amp state then stay as
// whatever the rest of the firmware left them.
typedef void (*alarm_speaking_cb_t)(bool speaking);
typedef void (*alarm_amp_cb_t)(bool enable);
void alarm_set_speaking_callback(alarm_speaking_cb_t cb);
void alarm_set_amp_callback(alarm_amp_cb_t cb);

// Install a user-uploaded chime MP3 as the alarm tone. Decodes to PCM,
// replaces the built-in procedural chime, and persists the MP3 bytes to
// the `storage` SPIFFS partition so it survives reboots. Pass `mp3=NULL`
// to revert to the built-in chime (also removes the persisted file).
//
// Takes ownership of the `mp3` buffer on success — caller must NOT
// heap_caps_free it.
esp_err_t alarm_set_custom_chime(uint8_t *mp3, size_t mp3_len);
