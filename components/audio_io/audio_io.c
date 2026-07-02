#include "audio_io.h"

#include <math.h>
#include <string.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

static const char *TAG = "audio_io";

static i2s_chan_handle_t s_rx = NULL;
static i2s_chan_handle_t s_tx = NULL;

static RingbufHandle_t s_capture_rb = NULL;
static RingbufHandle_t s_playback_rb = NULL;

static volatile bool s_playback_active = false;
static volatile bool s_capture_active = false;

// Monotonic count of 16 kHz mono samples pushed into the capture ringbuffer.
// The [hb] heartbeat in main.c prints the per-interval delta as cap_sps —
// ~16000 means the mic path is alive, 0 means it is dead. Added after the
// 0.2.60 regression where the device kept heartbeating while completely
// deaf; this makes that failure class visible from the log line alone.
static volatile uint32_t s_capture_samples_total = 0;

// Software playback volume. Linear gain applied in the int16→int32 shift
// inside playback_task. 0 = silent, 100 = unity. 80 by default (mild
// headroom for the XVF DAC stage so we don't clip on loud peaks).
// Persists via NVS (nvs_creds_get_volume / nvs_creds_set_volume); loaded
// by main.c on boot.
static volatile uint8_t s_volume_pct = 80;

// Pause flag — when true, playback_task stalls without draining the
// ringbuffer or writing to I²S. Used by the voice control flow so a
// "<wake word>, pause" doesn't lose the rest of the TTS queue (resume picks
// up where we left off).
static volatile bool s_paused = false;

static TaskHandle_t s_capture_task = NULL;
static TaskHandle_t s_playback_task = NULL;

// Rolling 1-second peak-RMS of captured mono16k audio. Exposed via
// audio_io_get_capture_rms_1s() so main.c's agc_freeze_task can wait
// for a quiet acoustic environment before locking the XVF AGC gain.
// See memory project_xvf3800_agc_quiet_wait for revert path.
static uint32_t s_rms_current_max  = 0;
static uint32_t s_rms_last_window  = 0;
static int64_t  s_rms_window_start = 0;

// 44.1 → 48 kHz fractional resampler state (16.16 phase accumulator +
// previous-sample interpolation). File-scope so audio_io_flush_playback
// can reset it on pause — see write_pcm's 44100 branch and the flush
// function below for why. Only AirPlay uses these (16k/24k callers
// keep stack-local state per call).
static int16_t  s_44k_prev_l = 0, s_44k_prev_r = 0;
static uint32_t s_44k_phase  = 0;
static uint32_t s_44k_last_rate_witness = 0;

// DMA sizing was tuned for 16 kHz. At 48 kHz a frame = 480 samples ≈ 10 ms
// (still); 6 frames = 60 ms of buffering, plenty for interrupt latency.
// RX_DMA_SAMPLES is in *bus frames*. Each frame = stereo @ 32-bit = 8 bytes.
#define RX_DMA_FRAMES 4
#define RX_DMA_SAMPLES 480
#define TX_DMA_FRAMES 6
#define TX_DMA_SAMPLES 480

// Bus→app and app→bus rate ratio. 48000 / 16000 = 3. Hardcoded so the
// decimator/interpolator stays branchless and the compiler can unroll
// the inner loops.
#define BUS_TO_APP_DECIM 3

#define CAPTURE_RB_BYTES (16 * 1024)
// 32 KB was fine for ~21 frames of TTS at 48 kHz stereo, but AirPlay
// arrives in bursts that exceed that window and cause write_pcm to
// block — and while blocked the I²S DMA drains and you hear static.
// 128 KB → 256 KB on 2026-06-04: ambient telemetry showed pcm_rb dipping
// to 0 during marginal-Wi-Fi byte-rate dips (~68% of 160k target for one
// 10s interval), audibly glitching as I²S ran dry. 256 KB doubles the
// cushion to ~1.3s of decoded PCM, enough to ride out a single bursty
// Wi-Fi interval. Allocated from PSRAM via xRingbufferCreateWithCaps
// below — internal SRAM is too tight to absorb another 128 KB at this
// stage of boot (wake-word model + decoder + drive/capture tasks).
#define PLAYBACK_RB_BYTES (256 * 1024)

static void capture_task(void *arg)
{
    const size_t bus_chunk_samples = RX_DMA_SAMPLES * AUDIO_BUS_CHANNELS;
    const size_t bus_chunk_bytes = bus_chunk_samples * sizeof(int32_t);
    int32_t *bus_buf = heap_caps_malloc(bus_chunk_bytes, MALLOC_CAP_DMA);
    if (!bus_buf) {
        ESP_LOGE(TAG, "capture_task: malloc fail");
        vTaskDelete(NULL);
        return;
    }
    // mono16k holds the decimated output. 480 bus frames @ /3 = 160 app
    // samples; size for the worst case (full frame) at /1 just in case.
    int16_t mono16k[RX_DMA_SAMPLES];

    // Carry-over state for the 3-tap box averager: when a bus chunk ends
    // mid-triplet (frames not a multiple of 3), we accumulate the partial
    // triplet's running sum + count and finish it on the next chunk.
    // Skipping this would create periodic clicks at the wrap.
    int32_t carry_sum = 0;
    int     carry_count = 0;

    while (1) {
        if (!s_capture_active || !s_rx) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        size_t br = 0;
        esp_err_t e = i2s_channel_read(s_rx, bus_buf, bus_chunk_bytes, &br, pdMS_TO_TICKS(200));
        if (e != ESP_OK || br == 0) continue;

        // One-shot debug: dump the first 16 raw int32 samples after boot
        // so we can verify the XVF's bit alignment. Expected for 24-in-32
        // Philips: top 24 bits are signed sample, bottom 8 are zero. If
        // we see huge int32 values (close to ±INT32_MAX), data is
        // right-aligned or different format.
        static bool dumped_once = false;
        if (!dumped_once && br >= 16 * sizeof(int32_t)) {
            ESP_LOGI(TAG, "raw bus int32 samples (first 16):");
            for (int k = 0; k < 16; k += 2) {
                ESP_LOGI(TAG, "  [%d] L=0x%08x (%d)  R=0x%08x (%d)",
                         k/2,
                         (unsigned)bus_buf[k], (int)bus_buf[k],
                         (unsigned)bus_buf[k+1], (int)bus_buf[k+1]);
            }
            dumped_once = true;
        }

        // Bus is 48 kHz stereo 32-bit. Decimate by 3 to get 16 kHz mono.
        // Take channel 1 (right slot) — the HA-variant firmware puts the
        // ASR-processed mic output here at full gain. Channel 0 (left)
        // carries a much-quieter reference signal in this firmware
        // (verified via raw-bus dump: L peaks ~1.5M, R peaks ~42M for
        // the same audio event). The old i2s_v1.0.7 firmware mirrored
        // mono to both slots, so channel 0 worked there.
        //
        // Anti-alias: simple 3-tap box average — kills high freq above
        // ~5 kHz, well below the Nyquist of the 16 kHz target. Voice
        // bandwidth is mostly under 4 kHz so this is acceptable; a real
        // polyphase FIR would be cleaner but adds CPU + code.
        const size_t frames = br / sizeof(int32_t) / AUDIO_BUS_CHANNELS;
        size_t out_idx = 0;
        for (size_t i = 0; i < frames; ++i) {
            int32_t s = bus_buf[i * AUDIO_BUS_CHANNELS + 1] >> 16; // R slot, s24 → s16
            carry_sum += s;
            if (++carry_count >= BUS_TO_APP_DECIM) {
                int32_t avg = carry_sum / BUS_TO_APP_DECIM;
                if (avg >  INT16_MAX) avg = INT16_MAX;
                if (avg <  INT16_MIN) avg = INT16_MIN;
                mono16k[out_idx++] = (int16_t) avg;
                carry_sum = 0;
                carry_count = 0;
                if (out_idx >= sizeof(mono16k)/sizeof(mono16k[0])) break;
            }
        }
        if (out_idx > 0 && s_capture_rb) {
            // Track peak RMS for audio_io_get_capture_rms_1s() — agc_freeze_task
            // polls this to wait for quiet before locking the XVF AGC gain.
            // Cheap: one int64 mac per output sample, sqrt once per chunk.
            int64_t sum_sq = 0;
            for (size_t i = 0; i < out_idx; ++i) {
                sum_sq += (int64_t)mono16k[i] * mono16k[i];
            }
            uint32_t rms = (uint32_t) sqrtf((float)(sum_sq / (int64_t)out_idx));
            if (rms > s_rms_current_max) s_rms_current_max = rms;
            int64_t now = esp_timer_get_time();
            if (s_rms_window_start == 0) s_rms_window_start = now;
            if (now - s_rms_window_start >= 1000000) {
                s_rms_last_window  = s_rms_current_max;
                s_rms_current_max  = 0;
                s_rms_window_start = now;
            }

            xRingbufferSend(s_capture_rb, mono16k, out_idx * sizeof(int16_t), 0);
            s_capture_samples_total += out_idx;
        }
    }
}

uint32_t audio_io_get_capture_rms_1s(void) {
    return s_rms_last_window;
}

uint32_t audio_io_get_capture_samples_total(void) {
    return s_capture_samples_total;
}

static void playback_task(void *arg)
{
    const size_t bus_chunk_samples = TX_DMA_SAMPLES * AUDIO_BUS_CHANNELS;
    const size_t bus_chunk_bytes = bus_chunk_samples * sizeof(int32_t);
    int32_t *bus_buf = heap_caps_malloc(bus_chunk_bytes, MALLOC_CAP_DMA);
    if (!bus_buf) {
        ESP_LOGE(TAG, "playback_task: malloc fail");
        vTaskDelete(NULL);
        return;
    }

    // Ringbuffer now holds interleaved stereo samples at BUS rate (48 kHz int16).
    // The
    // upsample/downsample of source-rate audio happens at the call-site in
    // audio_io_write_pcm — that way music sources at 48 kHz can push 1:1
    // without going through a quality-lossy 16 kHz intermediate, while
    // 16 kHz TTS sources still play correctly. playback_task here just
    // scales each L/R sample and pushes it to I2S at 1:1 bus rate.
    while (1) {
        if (!s_playback_active || !s_tx) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // Pause: stall here without consuming the ringbuffer. Resuming
        // simply clears the flag and we pick up at the next ringbuffer
        // slot. Sleep at the wake-word frame cadence so resume latency
        // is sub-100 ms.
        if (s_paused) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        size_t item_size = 0;
        int16_t *pcm = (int16_t *)xRingbufferReceiveUpTo(
            s_playback_rb, &item_size, pdMS_TO_TICKS(50), TX_DMA_SAMPLES * sizeof(int16_t));

        if (!pcm || item_size == 0) {
            if (pcm) vRingbufferReturnItem(s_playback_rb, pcm);
            continue;
        }

        // Ringbuffer is interleaved L/R int16 at bus rate. Each int16
        // becomes one int32 bus slot (shift << 16 to fill the upper bits
        // for 24-bit-in-32 MSB-justified Philips). Alternation of L/R
        // in the rb maps 1:1 to alternation of L/R slots on the bus.
        // No mono mirroring — true stereo reaches the 3.5 mm jack.
        //
        // Software volume: scale each int16 by s_volume_pct / 100 before
        // the int32 shift. Linear scaling — not perceptual, but the
        // XVF DAC + downstream amp curve already imposes a non-linear
        // response so the perceived loudness vs slider position is
        // close enough to natural. Sub-10% will start to sound quantized
        // (loss of effective bit depth), which is fine since at that
        // level you almost can't hear it anyway.
        const uint8_t vol = s_volume_pct;  // snapshot — runtime updates race-safe
        const size_t in_samples = item_size / sizeof(int16_t);
        size_t out_idx = 0;
        for (size_t i = 0; i < in_samples && out_idx < bus_chunk_samples; ++i) {
            int32_t v = ((int32_t)pcm[i] * vol) / 100;
            bus_buf[out_idx++] = v << 16;
        }
        vRingbufferReturnItem(s_playback_rb, pcm);

        size_t bw;
        i2s_channel_write(s_tx, bus_buf, out_idx * sizeof(int32_t), &bw, pdMS_TO_TICKS(100));
        s_playback_active = true;
    }
}

// Full-duplex SLAVE bring-up — BOTH channels created in one i2s_new_channel
// call and enabled for the lifetime of the process, exactly like every build
// through 0.2.59. The XVF is the I²S master and clocks both directions
// continuously, so RX (mic) keeps flowing while TX (speaker) plays — that is
// what makes barge-in and wake-during-ambient possible at all.
//
// DO NOT reintroduce the 0.2.60 "half-duplex flip" (teardown RX around
// playback). Validated live 2026-07-02: the flip left the mic torn down for
// the rest of the session after the first playback (reply paths never call
// stop_playback), so the device went permanently deaf — and deleting the RX
// channel out from under capture_task's blocking 200 ms i2s_channel_read is
// a use-after-free. TX-while-RX wake degradation on this carrier is handled
// upstream by the XVF AEC plus the speaking-state gates in wakeword.cpp.
static esp_err_t bring_up_full_duplex(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    chan_cfg.dma_desc_num = RX_DMA_FRAMES;
    chan_cfg.dma_frame_num = RX_DMA_SAMPLES;
    chan_cfg.auto_clear = true;
    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx, &s_rx);
    if (err != ESP_OK) return err;
    i2s_std_config_t cfg = {
        // clk_cfg.sample_rate is informational in slave mode (the driver
        // follows external BCLK/WS); set to the XVF rate for sane logs.
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_BUS_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,   // XVF generates MCLK internally
            .bclk = AUDIO_PIN_BCLK,
            .ws   = AUDIO_PIN_LRCLK,
            .dout = AUDIO_PIN_DOUT_TX,
            .din  = AUDIO_PIN_DIN_RX,
            .invert_flags = {0},
        },
    };
    err = i2s_channel_init_std_mode(s_rx, &cfg);
    if (err == ESP_OK) err = i2s_channel_init_std_mode(s_tx, &cfg);
    if (err == ESP_OK) err = i2s_channel_enable(s_rx);
    if (err == ESP_OK) err = i2s_channel_enable(s_tx);
    return err;
}

esp_err_t audio_io_init(void)
{
    // I2S half-duplex SLAVE. The HA-variant XVF firmware acts as the
    // I²S primary (master): it generates BCLK/WS from its own crystal
    // and drives the data lines. ESP32 just follows. No MCLK needed
    // on the ESP32 side — XVF derives MCLK internally.
    //
    // The 16 kHz master mode (i2s_v1.0.7.bin firmware) is the revert
    // path if anything breaks: change I2S_ROLE_SLAVE → I2S_ROLE_MASTER
    // and AUDIO_BUS_SAMPLE_RATE 48000 → 16000 in audio_io.h.
    s_capture_rb = xRingbufferCreate(CAPTURE_RB_BYTES, RINGBUF_TYPE_BYTEBUF);
    // Playback ringbuffer lives in PSRAM — at 256 KB it would otherwise
    // squeeze internal SRAM (~150-200 KB free at this point in boot).
    // PSRAM read bandwidth is far above the playback drain rate (~192 KB/s
    // for 48k stereo 16-bit), so the latency penalty is negligible.
    s_playback_rb = xRingbufferCreateWithCaps(PLAYBACK_RB_BYTES, RINGBUF_TYPE_BYTEBUF,
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_capture_rb || !s_playback_rb) return ESP_ERR_NO_MEM;

    esp_err_t err = bring_up_full_duplex();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S full-duplex init failed: %s", esp_err_to_name(err));
        return err;
    }

    // 0.2.34: bumped from 4 KB → 6 KB. The audio_io_write_pcm resampler
    // family (called by tts_worker / ambient_w) keeps a stack-local
    // `int16_t out[1024]` (2 KB) plus prev-state locals — combined with
    // ESP-IDF logging frames inside the deep call chain, 4 KB was
    // occasionally too tight. heartbeat_task now logs high-water marks
    // so we can tell empirically how close we get.
    xTaskCreatePinnedToCore(capture_task, "audio_cap", 6144, NULL, 10, &s_capture_task, 1);
    xTaskCreatePinnedToCore(playback_task, "audio_play", 6144, NULL, 9, &s_playback_task, 1);

    ESP_LOGI(TAG, "I2S slave full-duplex: BCLK=%d LRCLK=%d DIN=%d DOUT=%d @ %dHz/%dch/%dbit (XVF is master)",
             AUDIO_PIN_BCLK, AUDIO_PIN_LRCLK, AUDIO_PIN_DIN_RX, AUDIO_PIN_DOUT_TX,
             AUDIO_BUS_SAMPLE_RATE, AUDIO_BUS_CHANNELS, AUDIO_BUS_BITS);
    return ESP_OK;
}

esp_err_t audio_io_start_capture(void) { s_capture_active = true; return ESP_OK; }
esp_err_t audio_io_stop_capture(void)  { s_capture_active = false; return ESP_OK; }

size_t audio_io_read_frame(int16_t *out, size_t max_samples, uint32_t timeout_ms)
{
    if (!s_capture_rb) return 0;
    size_t got = 0;
    const size_t want_bytes = max_samples * sizeof(int16_t);
    while (got < want_bytes) {
        size_t item_size = 0;
        int16_t *p = (int16_t *)xRingbufferReceiveUpTo(
            s_capture_rb, &item_size, pdMS_TO_TICKS(timeout_ms), want_bytes - got);
        if (!p || item_size == 0) break;
        memcpy(((uint8_t *)out) + got, p, item_size);
        got += item_size;
        vRingbufferReturnItem(s_capture_rb, p);
    }
    return got / sizeof(int16_t);
}

esp_err_t audio_io_start_playback(void) { s_playback_active = true; return ESP_OK; }
esp_err_t audio_io_stop_playback(void)  { s_playback_active = false; audio_io_flush_playback(); return ESP_OK; }

void audio_io_set_volume(uint8_t pct) {
    if (pct > 100) pct = 100;
    s_volume_pct = pct;
}
uint8_t audio_io_get_volume(void) { return s_volume_pct; }

esp_err_t audio_io_pause_playback(void)  { s_paused = true;  return ESP_OK; }
esp_err_t audio_io_resume_playback(void) { s_paused = false; return ESP_OK; }
bool      audio_io_is_paused(void)       { return s_paused; }

size_t audio_io_write_pcm(const int16_t *pcm_stereo, size_t samples, uint32_t source_rate)
{
    if (!s_playback_rb) return 0;
    // Ringbuffer holds STEREO INTERLEAVED L/R int16 samples at BUS rate
    // (48 kHz). Callers (mp3_decode, etc.) always pass interleaved stereo
    // — for mono sources mp3_decode duplicates L=R before calling here.
    // The `samples` arg is the TOTAL int16 count (L+R interleaved), so
    // frames = samples / 2.
    //
    // This function converts source-rate audio to 48 kHz, preserving the
    // L/R distinction so the 3.5 mm jack outputs real stereo while the
    // internal speaker hears the XVF's L+R sum. Four supported rates:
    //   48000  — already bus-rate (music files, raw audio): push 1:1
    //   16000  — TTS replies (server-side ffmpeg targets 16k mono): /3 upsample per channel
    //   24000  — F5-TTS native output before ffmpeg: /2 upsample per channel
    //   44100  — AirPlay 1 receiver (CD-rate ALAC): 147:160 fractional
    //            upsample via 16.16 phase accumulator + linear interp.
    // Linear interpolation for upsampling — voice quality. For music a
    // polyphase FIR would sound cleaner; revisit if music quality is poor.

    if (source_rate == AUDIO_BUS_SAMPLE_RATE) {
        // 48 kHz native stereo — push straight to bus rb.
        BaseType_t r = xRingbufferSend(s_playback_rb, pcm_stereo, samples * sizeof(int16_t), pdMS_TO_TICKS(200));
        return r == pdTRUE ? samples : 0;
    }

    if (source_rate == 16000) {
        // 1:3 linear-interp upsample to 48k. Each L and R channel is
        // upsampled independently. 128 input frames (256 samples) → 384
        // output frames (768 samples) per chunk.
        int16_t buf[768];
        size_t total_written = 0;
        int16_t prev_l = pcm_stereo[0];
        int16_t prev_r = pcm_stereo[1];
        const size_t in_frames = samples / 2;
        for (size_t in_base = 0; in_base < in_frames; ) {
            const size_t chunk_in = (in_frames - in_base) > 128 ? 128 : (in_frames - in_base);
            size_t bi = 0;
            for (size_t i = 0; i < chunk_in; ++i) {
                int16_t cur_l = pcm_stereo[2 * (in_base + i)];
                int16_t cur_r = pcm_stereo[2 * (in_base + i) + 1];
                int32_t step_l = (int32_t)cur_l - prev_l;
                int32_t step_r = (int32_t)cur_r - prev_r;
                buf[bi++] = prev_l;
                buf[bi++] = prev_r;
                buf[bi++] = (int16_t)(prev_l + step_l / 3);
                buf[bi++] = (int16_t)(prev_r + step_r / 3);
                buf[bi++] = (int16_t)(prev_l + (2 * step_l) / 3);
                buf[bi++] = (int16_t)(prev_r + (2 * step_r) / 3);
                prev_l = cur_l;
                prev_r = cur_r;
            }
            xRingbufferSend(s_playback_rb, buf, bi * sizeof(int16_t), pdMS_TO_TICKS(200));
            total_written += bi;
            in_base += chunk_in;
        }
        return total_written;
    }

    if (source_rate == 44100) {
        // AirPlay 1 sources push s16 stereo at 44.1 kHz. 44100/48000 is
        // 147/160 — not an integer ratio — so we use a 16.16 fixed-point
        // phase accumulator with linear interpolation between adjacent
        // input frames. State persists across calls (chunk boundaries)
        // to avoid the per-8ms clicks you'd hear if prev_l/prev_r reset
        // to 0 every chunk. Reset state when the source rate changes
        // so a TTS→music transition starts cleanly. State is now at
        // file scope so audio_io_flush_playback can also reset it on
        // an AirPlay pause (where source_rate stays 44100 but the
        // continuity between last-pre-pause sample and first-post-
        // resume sample is otherwise broken — would emit a click).
        const uint32_t  PHASE_STEP_44K = 60211u; // (44100<<16)/48000 rounded
        if (s_44k_last_rate_witness != 44100u) {
            s_44k_prev_l = 0;
            s_44k_prev_r = 0;
            s_44k_phase  = 0;
            s_44k_last_rate_witness = 44100u;
        }
        if (samples < 2) return 0;
        int16_t out[1024];               // ~512 frames per ringbuffer push
        size_t  out_idx = 0;
        size_t  total_written = 0;
        const size_t in_frames = samples / 2;

        int16_t  prev_l = s_44k_prev_l;
        int16_t  prev_r = s_44k_prev_r;
        uint32_t phase  = s_44k_phase;

        for (size_t in_idx = 0; in_idx < in_frames; ++in_idx) {
            int16_t cur_l = pcm_stereo[in_idx * 2];
            int16_t cur_r = pcm_stereo[in_idx * 2 + 1];
            // Emit every output sample whose phase still falls between
            // the prev input frame and this cur one (phase < 65536).
            while (phase < 65536u) {
                uint32_t frac = phase;
                int32_t  step_l = (int32_t)cur_l - prev_l;
                int32_t  step_r = (int32_t)cur_r - prev_r;
                int16_t  o_l = (int16_t)(prev_l + (int32_t)(((int64_t)step_l * (int32_t)frac) >> 16));
                int16_t  o_r = (int16_t)(prev_r + (int32_t)(((int64_t)step_r * (int32_t)frac) >> 16));
                out[out_idx++] = o_l;
                out[out_idx++] = o_r;
                phase += PHASE_STEP_44K;
                if (out_idx >= (sizeof(out) / sizeof(out[0]))) {
                    xRingbufferSend(s_playback_rb, out, out_idx * sizeof(int16_t), pdMS_TO_TICKS(200));
                    total_written += out_idx;
                    out_idx = 0;
                }
            }
            prev_l = cur_l;
            prev_r = cur_r;
            phase -= 65536u;     // consume one input frame
        }

        if (out_idx > 0) {
            xRingbufferSend(s_playback_rb, out, out_idx * sizeof(int16_t), pdMS_TO_TICKS(200));
            total_written += out_idx;
        }
        s_44k_prev_l = prev_l;
        s_44k_prev_r = prev_r;
        s_44k_phase  = phase;
        return total_written;
    }

    if (source_rate == 24000) {
        // 1:2 linear-interp upsample to 48k, per channel.
        int16_t buf[512];
        size_t total_written = 0;
        int16_t prev_l = pcm_stereo[0];
        int16_t prev_r = pcm_stereo[1];
        const size_t in_frames = samples / 2;
        for (size_t in_base = 0; in_base < in_frames; ) {
            const size_t chunk_in = (in_frames - in_base) > 128 ? 128 : (in_frames - in_base);
            size_t bi = 0;
            for (size_t i = 0; i < chunk_in; ++i) {
                int16_t cur_l = pcm_stereo[2 * (in_base + i)];
                int16_t cur_r = pcm_stereo[2 * (in_base + i) + 1];
                buf[bi++] = prev_l;
                buf[bi++] = prev_r;
                buf[bi++] = (int16_t)(((int32_t)prev_l + cur_l) / 2);
                buf[bi++] = (int16_t)(((int32_t)prev_r + cur_r) / 2);
                prev_l = cur_l;
                prev_r = cur_r;
            }
            xRingbufferSend(s_playback_rb, buf, bi * sizeof(int16_t), pdMS_TO_TICKS(200));
            total_written += bi;
            in_base += chunk_in;
        }
        return total_written;
    }

    // Unknown source rate — fall back to 1:1 push (will sound wrong unless
    // it happens to be 48 kHz). Log so future-us notices.
    ESP_LOGW(TAG, "audio_io_write_pcm: unhandled source_rate=%u, pushing 1:1", (unsigned) source_rate);
    BaseType_t r = xRingbufferSend(s_playback_rb, pcm_stereo, samples * sizeof(int16_t), pdMS_TO_TICKS(200));
    return r == pdTRUE ? samples : 0;
}

void audio_io_flush_playback(void)
{
    if (!s_playback_rb) return;
    size_t item_size;
    void *p;
    while ((p = xRingbufferReceive(s_playback_rb, &item_size, 0)) != NULL) {
        vRingbufferReturnItem(s_playback_rb, p);
    }
    // OE 2026-05-27: reset the 44.1→48 kHz fractional resampler state
    // so a fresh AirPlay stream (or post-pause resume) doesn't linear-
    // interpolate between a stale prev_l/prev_r and the new first
    // sample — that interpolation across a gap produces an audible
    // click. The rate-change guard inside write_pcm only fires when
    // source_rate changes; on pause-then-resume the source stays 44100
    // so without this reset the statics carry forward. The 16k/24k
    // branches use stack-local state and aren't affected.
    s_44k_prev_l = 0;
    s_44k_prev_r = 0;
    s_44k_phase = 0;
    s_44k_last_rate_witness = 0;
}

bool audio_io_playback_active(void) { return s_playback_active; }

void audio_io_get_playback_buf_stats(uint32_t *used_bytes, uint32_t *capacity_bytes) {
    if (capacity_bytes) *capacity_bytes = PLAYBACK_RB_BYTES;
    if (used_bytes) {
        if (!s_playback_rb) { *used_bytes = 0; return; }
        size_t free_bytes = xRingbufferGetCurFreeSize(s_playback_rb);
        *used_bytes = (free_bytes > PLAYBACK_RB_BYTES) ? 0 : (uint32_t)(PLAYBACK_RB_BYTES - free_bytes);
    }
}
