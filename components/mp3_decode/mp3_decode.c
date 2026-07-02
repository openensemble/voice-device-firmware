#include "mp3_decode.h"

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "mp3dec.h"

static const char *TAG = "mp3";

// Monotonic decode-error counter (all mp3_dec_t instances contribute).
// Read by the heartbeat ambient telemetry to derive per-interval error
// rate without instrumenting every decode call site.
static volatile uint32_t s_decode_errors_total = 0;
uint32_t mp3_dec_get_total_errors(void) { return s_decode_errors_total; }

// 128 KB input ringbuffer for MP3 bytes between TCP recv and libhelix.
// At 160 kbps stereo (~20 KB/s) this holds ~6 seconds of pre-decode data,
// giving the decoder room to ride out Wi-Fi packet jitter without
// processing partial frames. Allocated from PSRAM, no SRAM impact.
// Was 16 KB which only held ~0.8 sec — too tight for typical Wi-Fi
// retransmit windows; brief stalls forced the decoder to work with
// borderline-complete frames and produce -6 / -9 / -2 errors.
#define MP3_INPUT_BUF_BYTES   (128 * 1024)
#define MP3_PCM_FRAME_SAMPLES 1152

struct mp3_dec_s {
    HMP3Decoder helix;
    uint8_t *inbuf;
    size_t inbuf_used;
    int16_t *pcm;
    mp3_pcm_callback_t cb;
    void *user;
};

mp3_dec_t *mp3_dec_create(mp3_pcm_callback_t cb, void *user)
{
    mp3_dec_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;
    d->helix = MP3InitDecoder();
    if (!d->helix) { free(d); return NULL; }
    d->inbuf = heap_caps_malloc(MP3_INPUT_BUF_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    d->pcm   = heap_caps_malloc(MP3_PCM_FRAME_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!d->inbuf || !d->pcm) {
        if (d->inbuf) free(d->inbuf);
        if (d->pcm) free(d->pcm);
        MP3FreeDecoder(d->helix);
        free(d);
        return NULL;
    }
    d->cb = cb;
    d->user = user;
    return d;
}

void mp3_dec_destroy(mp3_dec_t *d)
{
    if (!d) return;
    if (d->helix) MP3FreeDecoder(d->helix);
    free(d->inbuf);
    free(d->pcm);
    free(d);
}

static esp_err_t drain(mp3_dec_t *d)
{
    while (d->inbuf_used > 0) {
        int off = MP3FindSyncWord(d->inbuf, d->inbuf_used);
        if (off < 0) {
            d->inbuf_used = 0;
            return ESP_OK;
        }
        if (off > 0) {
            memmove(d->inbuf, d->inbuf + off, d->inbuf_used - off);
            d->inbuf_used -= off;
        }

        unsigned char *bytes = d->inbuf;
        int bytes_left = (int)d->inbuf_used;
        int rc = MP3Decode(d->helix, &bytes, &bytes_left, d->pcm, 0);
        if (rc == ERR_MP3_INDATA_UNDERFLOW) {
            return ESP_OK;
        }
        if (rc != ERR_MP3_NONE) {
            s_decode_errors_total++;
            ESP_LOGW(TAG, "decode err %d, skipping byte", rc);
            if (d->inbuf_used > 1) {
                memmove(d->inbuf, d->inbuf + 1, d->inbuf_used - 1);
                d->inbuf_used -= 1;
            } else {
                d->inbuf_used = 0;
            }
            continue;
        }
        size_t consumed = d->inbuf_used - (size_t)bytes_left;
        memmove(d->inbuf, d->inbuf + consumed, d->inbuf_used - consumed);
        d->inbuf_used -= consumed;

        MP3FrameInfo fi;
        MP3GetLastFrameInfo(d->helix, &fi);

        // Always deliver interleaved stereo to the callback so downstream
        // (audio_io_write_pcm) can push true L/R to the 48 kHz stereo bus
        // and the 3.5 mm jack outputs real stereo. Mono MP3s get the
        // single channel mirrored to L=R; stereo MP3s pass through
        // untouched.
        //
        // The callback's `samples` arg is total int16 element count
        // (L+R interleaved), so frames = samples/2. The previous mono-
        // downmixing behavior is gone; both callers (TTS + test-MP3)
        // now receive stereo.
        if (fi.nChans == 2) {
            // Already interleaved L/R — pass through. fi.outputSamps is
            // total L+R count (frames * 2).
            if (d->cb) d->cb(d->pcm, fi.outputSamps, fi.samprate, d->user);
        } else {
            // Mono source — expand to stereo in-place by duplicating each
            // sample as (L, R). Walk back-to-front so source and dest
            // don't overlap-destroy each other.
            int16_t *p = d->pcm;
            int mono_samps = fi.outputSamps;
            for (int i = mono_samps - 1; i >= 0; --i) {
                int16_t s = p[i];
                p[2*i]     = s;
                p[2*i + 1] = s;
            }
            if (d->cb) d->cb(p, mono_samps * 2, fi.samprate, d->user);
        }
    }
    return ESP_OK;
}

esp_err_t mp3_dec_feed(mp3_dec_t *d, const uint8_t *bytes, size_t n)
{
    if (!d || !bytes || n == 0) return ESP_OK;
    while (n > 0) {
        size_t room = MP3_INPUT_BUF_BYTES - d->inbuf_used;
        size_t take = n < room ? n : room;
        memcpy(d->inbuf + d->inbuf_used, bytes, take);
        d->inbuf_used += take;
        bytes += take;
        n -= take;
        drain(d);
    }
    return ESP_OK;
}

void mp3_dec_flush(mp3_dec_t *d) { if (d) d->inbuf_used = 0; }
