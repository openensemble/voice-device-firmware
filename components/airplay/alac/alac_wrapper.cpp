/*
 * alac_wrapper.cpp — C bridge to Apple's ALACDecoder C++ class.
 *
 * Written for OpenEnsemble voice-device firmware (2026-05-27).
 * License: MIT (this wrapper). The Apple ALAC sources are Apache-2.0.
 *
 * Mirrors the API used by squeezelite-esp32's RAOP component
 * (vendored verbatim in ../raop/rtp.c) so no changes are needed to
 * the upstream RTP / RAOP code.
 *
 * The RAOP "magic cookie" is built by rtp.c::alac_init from the SDP
 * fmtp= attributes that Apple's RTSP ANNOUNCE provides. Layout is a
 * packed big-endian ALACSpecificConfig (see ALACAudioTypes.h:165).
 * Apple's ALACDecoder::Init() expects exactly this format and does
 * its own endian-swap to host byte order internally.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>

#include "esp_log.h"
#include "esp_heap_caps.h"

#include "ALACAudioTypes.h"
#include "ALACBitUtilities.h"
#include "ALACDecoder.h"
#include "alac_wrapper.h"

static const char *ALAC_TAG = "alac_wrap";

struct alac_codec_s {
    ALACDecoder *decoder;
    uint32_t     frame_length;   // samples per channel per ALAC frame
    uint32_t     channels;
    uint32_t     bit_depth;
    uint32_t     sample_rate;
};

extern "C" struct alac_codec_s *alac_create_decoder(int magic_cookie_size,
                                                    unsigned char *magic_cookie,
                                                    unsigned char *sample_size,
                                                    unsigned *sample_rate,
                                                    unsigned char *channels,
                                                    unsigned int *block_size)
{
    ESP_LOGI(ALAC_TAG, "create_decoder: cookie_size=%d, expected>=%u",
             magic_cookie_size, (unsigned)sizeof(ALACSpecificConfig));
    if (magic_cookie && magic_cookie_size >= 24) {
        ESP_LOGI(ALAC_TAG, "cookie bytes 0..11: %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x",
                 magic_cookie[0], magic_cookie[1], magic_cookie[2], magic_cookie[3],
                 magic_cookie[4], magic_cookie[5], magic_cookie[6], magic_cookie[7],
                 magic_cookie[8], magic_cookie[9], magic_cookie[10], magic_cookie[11]);
        ESP_LOGI(ALAC_TAG, "cookie bytes 12..23: %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x",
                 magic_cookie[12], magic_cookie[13], magic_cookie[14], magic_cookie[15],
                 magic_cookie[16], magic_cookie[17], magic_cookie[18], magic_cookie[19],
                 magic_cookie[20], magic_cookie[21], magic_cookie[22], magic_cookie[23]);
    }
    if (!magic_cookie || magic_cookie_size < (int)sizeof(ALACSpecificConfig)) {
        ESP_LOGE(ALAC_TAG, "cookie too small or NULL");
        return nullptr;
    }

    struct alac_codec_s *codec =
        (struct alac_codec_s *)calloc(1, sizeof(*codec));
    if (!codec) return nullptr;

    codec->decoder = new ALACDecoder();
    if (!codec->decoder) { free(codec); return nullptr; }

    ESP_LOGI(ALAC_TAG, "pre-Init heap: internal_free=%u psram_free=%u largest_internal=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    int32_t status = codec->decoder->Init(magic_cookie, magic_cookie_size);
    if (status != ALAC_noErr) {
        ESP_LOGE(ALAC_TAG, "ALACDecoder::Init status=%ld (kALAC_ParamError=-50, kALAC_MemFullError=-108)",
                 (long)status);
        ESP_LOGE(ALAC_TAG, "post-fail heap: internal_free=%u psram_free=%u largest_internal=%u",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        delete codec->decoder;
        free(codec);
        return nullptr;
    }
    ESP_LOGI(ALAC_TAG, "ALACDecoder::Init OK — frameLength=%lu bitDepth=%u channels=%u sampleRate=%lu",
             (unsigned long)codec->decoder->mConfig.frameLength,
             (unsigned)codec->decoder->mConfig.bitDepth,
             (unsigned)codec->decoder->mConfig.numChannels,
             (unsigned long)codec->decoder->mConfig.sampleRate);

    // mConfig holds host-byte-order values after Init() byte-swaps the
    // cookie. Mirror them back to the caller in the layout Philippe44's
    // wrapper uses.
    codec->frame_length = codec->decoder->mConfig.frameLength;
    codec->channels     = codec->decoder->mConfig.numChannels;
    codec->bit_depth    = codec->decoder->mConfig.bitDepth;
    codec->sample_rate  = codec->decoder->mConfig.sampleRate;

    if (sample_size) *sample_size = (unsigned char)codec->bit_depth;
    if (sample_rate) *sample_rate = codec->sample_rate;
    if (channels)    *channels    = (unsigned char)codec->channels;
    // block_size is decoded PCM bytes per frame (frame_length * channels *
    // bytes/sample). rtp.c uses this to size its decode-output buffers.
    if (block_size)  *block_size  =
        codec->frame_length * codec->channels * (codec->bit_depth / 8);

    return codec;
}

extern "C" void alac_delete_decoder(struct alac_codec_s *codec)
{
    if (!codec) return;
    if (codec->decoder) delete codec->decoder;
    free(codec);
}

extern "C" bool alac_to_pcm(struct alac_codec_s *codec,
                            unsigned char *input,
                            unsigned char *output,
                            char channels,
                            unsigned *out_frames)
{
    if (!codec || !codec->decoder || !input || !output) return false;

    BitBuffer bits;
    // Worst-case bound for the input buffer — Apple's BitBuffer does
    // its own bounds checking against `byteSize`. The caller (rtp.c)
    // sizes input to maxFrameBytes from the SDP, well above any real
    // frame.
    BitBufferInit(&bits, input, codec->frame_length * codec->channels * 4);

    uint32_t decoded_samples = 0;
    int32_t  status = codec->decoder->Decode(
        &bits, output, codec->frame_length,
        (uint32_t)(channels ? channels : codec->channels),
        &decoded_samples);

    if (status != ALAC_noErr) {
        if (out_frames) *out_frames = 0;
        return false;
    }
    if (out_frames) *out_frames = decoded_samples;
    return true;
}
