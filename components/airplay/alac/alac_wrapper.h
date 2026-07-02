/*
 * alac_wrapper.h — C API bridge for Apple's C++ ALACDecoder
 *
 * Written for OpenEnsemble voice-device firmware (2026-05-27). API
 * signatures match Philippe44's wrapper (used by squeezelite-esp32's
 * RAOP component) so rtp.c can be vendored verbatim.
 *
 * License: MIT (this wrapper). The underlying Apple ALAC sources in
 * this same directory are Apache-2.0.
 */
#ifndef OE_ALAC_WRAPPER_H_
#define OE_ALAC_WRAPPER_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct alac_codec_s;

// magic_cookie is the 24-byte big-endian ALACSpecificConfig built by
// raop's rtp.c::alac_init from the SDP fmtp= attributes. On success
// returns a heap-allocated codec handle and writes back the parsed
// per-stream values. Returns NULL on failure.
struct alac_codec_s *alac_create_decoder(int magic_cookie_size,
                                         unsigned char *magic_cookie,
                                         unsigned char *sample_size,
                                         unsigned *sample_rate,
                                         unsigned char *channels,
                                         unsigned int *block_size);

void alac_delete_decoder(struct alac_codec_s *codec);

// Decode one ALAC frame from `input` into `output` (interleaved int16
// when sample_size == 16, which is the AirPlay-1 / RAOP case). Writes
// the number of decoded PCM frames (not bytes, not samples) into
// *out_frames. Returns true on success.
bool alac_to_pcm(struct alac_codec_s *codec,
                 unsigned char *input,
                 unsigned char *output,
                 char channels,
                 unsigned *out_frames);

#ifdef __cplusplus
}
#endif

#endif
