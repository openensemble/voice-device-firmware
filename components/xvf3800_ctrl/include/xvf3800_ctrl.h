#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// XVF3800 I²C bus and pins on Seeed reSpeaker XVF3800 + XIAO ESP32-S3 board.
#define XVF_I2C_ADDR 0x2C
#define XVF_I2C_SDA  GPIO_NUM_5
#define XVF_I2C_SCL  GPIO_NUM_6

// XMOS XCORE-VOICE device-control protocol IDs as exposed by the Seeed
// I2S firmware variant. Values cross-referenced with the working
// gillespinault/respeaker-xvf3800-vad and formatBCE/Respeaker-XVF3800-
// ESPHome-integration projects — the older XCORE-200 Device Control
// protocol (service/resource/cmd-byte encoding) is NOT what this firmware
// uses. New framing:
//   write: [resid:8][cmd:8][len:8][data ...]
//   read:  write [resid, cmd | 0x80, len] then read [status:8][data ...]
// The 0x80 bit in the cmd byte distinguishes a read from a write — without
// it the XVF treats the transaction as a malformed write and rejects with
// a non-zero status byte on the subsequent read.
#define XVF_CMD_READ_BIT 0x80

// Resource IDs
#define XVF_RESID_GPO            20   // LEDs, amplifier enable, GPO writes
#define XVF_RESID_AEC            33   // Echo cancellation, speech energy
#define XVF_RESID_PP             17   // Post-processing: AGC, NS, limiter, etc.
#define XVF_RESID_DFU_CONTROLLER 240  // DFU control (firmware version + DFU ops)

// Post-processing commands (resid 17)
#define XVF_CMD_PP_AGCONOFF      10   // int32: 0=off (freeze gain), 1=on (adapt)

// Command IDs (for resid 20 = GPO)
#define XVF_CMD_GPO_WRITE      1
#define XVF_CMD_LED_EFFECT     12
#define XVF_CMD_LED_BRIGHTNESS 13
#define XVF_CMD_LED_SPEED      15
#define XVF_CMD_LED_COLOR      16
#define XVF_CMD_LED_DOA_COLOR  17

// Command IDs (for resid 33 = AEC)
#define XVF_CMD_AEC_SPENERGY   74

// Command IDs (for resid 240 = DFU_CONTROLLER)
#define XVF_CMD_DFU_DNLOAD        1
#define XVF_CMD_DFU_GETSTATUS     3
#define XVF_CMD_DFU_SETALTERNATE 64
#define XVF_CMD_DFU_GETVERSION   88
#define XVF_CMD_DFU_REBOOT       89

// DFU alt-setting IDs
#define XVF_DFU_ALT_UPGRADE 1

// DFU state machine values (match DFU 1.1 spec / XMOS host_xvf_control)
#define XVF_DFU_STATE_IDLE                2
#define XVF_DFU_STATE_DNLOAD_IDLE         5
#define XVF_DFU_STATE_MANIFEST_WAIT_RESET 8
#define XVF_DFU_STATE_ERROR               10

// XVF3800 LED ring effects
typedef enum {
    XVF_LED_EFFECT_OFF     = 0,
    XVF_LED_EFFECT_BREATH  = 1,
    XVF_LED_EFFECT_RAINBOW = 2,
    XVF_LED_EFFECT_SINGLE  = 3,
    XVF_LED_EFFECT_DOA     = 4,
} xvf_led_effect_t;

// 24-bit RGB colors used by the LED ring.
#define XVF_RGB(r, g, b) (((uint32_t)(r) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(b))

// Higher-level "what is the device doing" states. main.c emits these via
// leds_buttons_set_state() and the implementation maps them to LED effects
// + colors below.
typedef enum {
    XVF_LED_PATTERN_OFF = 0,
    XVF_LED_PATTERN_IDLE,
    XVF_LED_PATTERN_LISTENING,
    XVF_LED_PATTERN_THINKING,
    XVF_LED_PATTERN_SPEAKING,
    XVF_LED_PATTERN_MUTE,
    XVF_LED_PATTERN_ERROR,
} xvf_led_pattern_t;

esp_err_t xvf3800_init(void);

// Low-level XMOS device-control transactions.
esp_err_t xvf3800_xmos_write(uint8_t resid, uint8_t cmd, const uint8_t *data, uint8_t len);
esp_err_t xvf3800_xmos_read(uint8_t resid, uint8_t cmd, uint8_t *out, uint8_t len);

// Convenience wrappers built on the low-level ops.
esp_err_t xvf3800_set_led_effect(xvf_led_effect_t effect);
esp_err_t xvf3800_set_led_brightness(uint8_t brightness);
esp_err_t xvf3800_set_led_color(uint32_t rgb);
esp_err_t xvf3800_set_led_doa_colors(uint32_t base_rgb, uint32_t doa_rgb);
esp_err_t xvf3800_enable_amplifier(bool enable);

// Headphone-mode override. When true, xvf3800_enable_amplifier(true) becomes
// a no-op (forces amp_en LOW). The XVF reads GPIO 31 as a "speaker active"
// AEC hint and drops mic sensitivity while it's HIGH — which kills wake-word
// during any playback. With headphones in the 3.5 mm jack (which taps the
// stereo DAC before the speaker amp), suppressing amp_en preserves wake-word
// sensitivity while audio still reaches the listener. Flipping ON also forces
// any in-flight amp HIGH back to LOW immediately so an already-running stream
// recovers without waiting for the next toggle.
void xvf3800_set_headphone_mode(bool enabled);
bool xvf3800_get_headphone_mode(void);

// Read the XVF's firmware version. Out is 3 bytes: [major, minor, patch].
esp_err_t xvf3800_get_dfu_version(uint8_t version[3]);

// One-shot I²C DFU client. Blocking — ~3 minutes for an 870 KB image.
// Returns ESP_OK on full success (image transferred + XVF rebooted +
// version verified). On any failure the XVF falls back to its Factory
// partition on next boot, so the device is recoverable.
//
// Pass expected_major/minor/patch to verify the chip reports that exact
// version after the post-DFU reboot, or {0,0,0} to skip verification.
esp_err_t xvf3800_dfu_apply(const uint8_t *bin, size_t len,
                            uint8_t expected_major, uint8_t expected_minor, uint8_t expected_patch);

// Pattern → effect+color mapping used by leds_buttons.c.
esp_err_t xvf3800_set_led_pattern(xvf_led_pattern_t pattern);

#ifdef __cplusplus
}
#endif
