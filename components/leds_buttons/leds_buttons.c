#include "leds_buttons.h"
#include "xvf3800_ctrl.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "leds_btn";
static ui_mute_callback_t s_mute_cb = NULL;
static bool s_muted = false;
static ui_state_t s_last_state = UI_STATE_BOOT;
// Gate: stays false until the first wake-word-driven state transition
// (LISTENING). I²C writes to the XVF before that point disturb the audio
// engine and suppress wake-word detection — but writes during/after wake
// appear to be safe. The first LISTENING transition arms the gate, and
// from then on every state change drives LED feedback as designed.
static bool s_armed = false;

static xvf_led_pattern_t ui_to_xvf(ui_state_t s)
{
    switch (s) {
        case UI_STATE_BOOT:      return XVF_LED_PATTERN_OFF;
        case UI_STATE_PROVISION: return XVF_LED_PATTERN_IDLE;
        case UI_STATE_IDLE:      return XVF_LED_PATTERN_IDLE;
        case UI_STATE_LISTENING: return XVF_LED_PATTERN_LISTENING;
        case UI_STATE_THINKING:  return XVF_LED_PATTERN_THINKING;
        case UI_STATE_SPEAKING:  return XVF_LED_PATTERN_SPEAKING;
        case UI_STATE_AMBIENT:   return XVF_LED_PATTERN_OFF;
        case UI_STATE_MUTED:     return XVF_LED_PATTERN_MUTE;
        case UI_STATE_ERROR:     return XVF_LED_PATTERN_ERROR;
    }
    return XVF_LED_PATTERN_OFF;
}

esp_err_t leds_buttons_init(ui_mute_callback_t on_mute_change)
{
    s_mute_cb = on_mute_change;
    // No mute polling. The XVF3800 i2s firmware exposes physical button
    // state via a path we haven't mapped yet (the older XCORE-200
    // service/resource we tried doesn't exist). Mute is currently
    // software-only via leds_buttons_force_mute() if needed; the LED ring
    // will reflect the state on the next leds_buttons_set_state() call.
    (void) s_mute_cb;
    return leds_buttons_set_state(UI_STATE_BOOT);
}

static const char *state_name(ui_state_t s) {
    switch (s) {
        case UI_STATE_BOOT:      return "BOOT";
        case UI_STATE_PROVISION: return "PROVISION";
        case UI_STATE_IDLE:      return "IDLE";
        case UI_STATE_LISTENING: return "LISTENING";
        case UI_STATE_THINKING:  return "THINKING";
        case UI_STATE_SPEAKING:  return "SPEAKING";
        case UI_STATE_AMBIENT:   return "AMBIENT";
        case UI_STATE_MUTED:     return "MUTED";
        case UI_STATE_ERROR:     return "ERROR";
    }
    return "?";
}

esp_err_t leds_buttons_set_state(ui_state_t state)
{
    ESP_LOGI(TAG, "state -> %s (armed=%d, muted=%d)",
             state_name(state), (int)s_armed, (int)s_muted);
    s_last_state = state;
    if (state == UI_STATE_LISTENING) s_armed = true;
    if (!s_armed) return ESP_OK;
    xvf_led_pattern_t pat = s_muted ? XVF_LED_PATTERN_MUTE : ui_to_xvf(state);
    return xvf3800_set_led_pattern(pat);
}

bool leds_buttons_is_muted(void) { return s_muted; }
