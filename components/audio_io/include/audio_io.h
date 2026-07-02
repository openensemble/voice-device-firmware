#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

// I²S bus runs at 48 kHz stereo 32-bit. This matches the formatBCE Home
// Assistant variant of the XVF3800 firmware (application_xvf3800_inthost-
// lr48-sqr-i2c-v1.0.7-release.bin), which is what we DFU-flash onto the
// XMOS for music+voice simultaneous playback support.
//
// In this firmware variant:
//   - XVF is I²S PRIMARY (master) — generates BCLK/WS/MCLK from its own
//     crystal. ESP32 is SECONDARY (slave), follows those clocks.
//   - Both slots carry useful audio: ESP32 RX gets the XVF-processed mic
//     (channel 1 = mono speech, post-AEC/AGC), ESP32 TX feeds the speaker
//     amp via the XVF (which still does AEC reference internally).
//   - 48 kHz is the only "high-rate" the XVF supports; 16 kHz was the
//     other option but loses music quality. App layer (wake-word, VAD,
//     STT, TTS) still runs at 16 kHz — audio_io resamples at the boundary.
#define AUDIO_BUS_SAMPLE_RATE 48000
#define AUDIO_BUS_BITS        32
#define AUDIO_BUS_CHANNELS    2

// App-side rate stays 16 kHz so the rest of the firmware (wake-word,
// VAD, STT capture, TTS playback) doesn't need to change. audio_io
// resamples 48k↔16k internally — 3:1 ratio so a fixed integer decimator
// (mic side) + linear-interp upsampler (speaker side) keeps it cheap.
#define AUDIO_APP_SAMPLE_RATE 16000
#define AUDIO_APP_FRAME_SAMPLES 1280

#define AUDIO_PIN_BCLK   GPIO_NUM_8
#define AUDIO_PIN_LRCLK  GPIO_NUM_7
#define AUDIO_PIN_DIN_RX GPIO_NUM_43
#define AUDIO_PIN_DOUT_TX GPIO_NUM_44

esp_err_t audio_io_init(void);
esp_err_t audio_io_start_capture(void);
esp_err_t audio_io_stop_capture(void);

size_t audio_io_read_frame(int16_t *out, size_t max_samples, uint32_t timeout_ms);

esp_err_t audio_io_start_playback(void);
esp_err_t audio_io_stop_playback(void);

// PCM input is INTERLEAVED stereo L/R int16 samples. `samples` is the total
// int16 count (frames * 2). Mono callers should duplicate L=R before calling
// (mp3_decode does this internally for mono MP3s). source_rate is the input
// sample rate (Hz) — function upsamples to 48 kHz internally for the bus.
size_t audio_io_write_pcm(const int16_t *pcm_stereo, size_t samples, uint32_t source_rate);
void audio_io_flush_playback(void);
bool audio_io_playback_active(void);

// Software playback volume (0-100 %, linear). Applied per-sample inside
// playback_task before the int16→int32 shift. Persists across boots via
// NVS; main.c loads on boot. Default 80 %.
void    audio_io_set_volume(uint8_t pct);
uint8_t audio_io_get_volume(void);

// Pause/resume of TTS / music playback WITHOUT dropping queued audio.
// Unlike audio_io_stop_playback (which also flushes the ringbuffer),
// pause leaves everything buffered and resume picks up where we stopped.
// Used by the voice-control intent router for "<wake word>, pause"/"resume"
// without losing the in-flight reply.
esp_err_t audio_io_pause_playback(void);
esp_err_t audio_io_resume_playback(void);
bool      audio_io_is_paused(void);

// Peak RMS of the captured 16 kHz mono stream over the most recent ~1 s
// window. 0 until at least one window has rolled over. Used by main.c's
// agc_freeze_task to wait for sustained acoustic quiet before locking the
// XVF AGC gain — XVF AGC freeze captures whatever gain was active, so
// freezing in noise locks a low gain that later starves the wake-word
// model. See memory project_xvf3800_agc_quiet_wait for revert details.
uint32_t audio_io_get_capture_rms_1s(void);

// Monotonic count of 16 kHz mono samples captured since boot. The heartbeat
// prints the per-interval delta as cap_sps: ~16000 = mic path alive, 0 = mic
// path dead (I²S RX stalled / channel gone). Unsigned wrap is fine — callers
// take deltas.
uint32_t audio_io_get_capture_samples_total(void);

// Playback ringbuffer telemetry. *used returns bytes currently queued
// (decoder-produced PCM waiting for I²S to drain), *capacity returns the
// ringbuffer's total size. Used by the ambient-stats heartbeat to
// surface "is the speaker about to underrun" without exposing internal
// xRingbuffer state.
void audio_io_get_playback_buf_stats(uint32_t *used_bytes, uint32_t *capacity_bytes);
