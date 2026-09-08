#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * Device-side alarm system.
 *
 * Owns a registry persisted in NVS and schedules esp_timers. Server-provided
 * relative delays use the monotonic clock immediately; restoring deadlines
 * after reboot requires SNTP. Audio is RAM-only, so restored alarms use the
 * built-in or uploaded chime. Completed IDs are retained for at least two
 * hours after their deadline/dismissal to make reconnect replay idempotent.
 */

// Called once at boot, AFTER nvs_flash_init. Loads any alarms persisted in
// NVS. Scheduling happens lazily once SNTP-synced wall-clock is available;
// if SNTP isn't ready, this function returns and an internal retry task
// schedules persisted alarms as soon as the clock acquires.
esp_err_t alarm_init(void);

// Arm a new alarm. Called from main.c's WS event dispatcher when an
// alarm_arm payload arrives. Stores in NVS + schedules esp_timer. The audio
// buffer's ownership transfers on every return path, including failures;
// the caller must NOT free it. Repeated IDs preserve their original countdown.
//
// trigger_at_ms is wall-clock epoch milliseconds. If it's already past
// when this is called (e.g. reboot replay), the alarm fires immediately.
esp_err_t alarm_arm(const char *id,
                    const char *label,
                    int64_t trigger_at_ms,
                    uint8_t *audio_mp3,
                    size_t audio_mp3_len,
                    const char *type);

// Relative delay from authenticated server time; -1 retains the legacy
// wall-clock path. Ownership of audio is transferred even on failure.
esp_err_t alarm_arm_with_delay(const char *id, const char *label, int64_t trigger_at_ms,
                    uint8_t *audio_mp3, size_t audio_mp3_len, const char *type, int64_t delay_ms);

// Cancel an alarm by id, including one whose arm worker has not arrived yet.
// Stops its ring loop and persists a completion record before acknowledging.
// Returns an error if persistence or completion-record capacity is unavailable.
esp_err_t alarm_disarm(const char *id);

// Stop currently-firing alarm(s). If id is non-NULL, stops only that one;
// NULL stops every currently-firing alarm on this device.
void alarm_stop(const char *id);

// True if any alarm's ring loop is currently active. Wake handler checks
// this to decide whether to treat a wake fire as a local dismiss.
bool alarm_is_firing(void);

// Re-send alarm_fired and durable local dismissals after authenticated WS
// (re)connect: alarm_fired is fire-and-forget, so a WS blip at the fire
// instant otherwise leaves the server thinking the alarm never rang.
// Idempotent server-side.
void alarm_resend_fired(void);

// Local dismiss path — invoked by the wake handler when wake fires while
// alarm_is_firing() is true. Stops the ring, sends alarm_acked after committing
// the dismissal to NVS, and suppresses the normal STT/utterance
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
