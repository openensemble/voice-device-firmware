#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef enum {
    UI_STATE_BOOT = 0,
    UI_STATE_PROVISION,
    UI_STATE_IDLE,
    UI_STATE_LISTENING,
    UI_STATE_THINKING,
    UI_STATE_SPEAKING,
    // Background ambient playback (sleep sounds, etc.). Distinct from
    // SPEAKING because ambient is meant to be unobtrusive — bright green
    // ring is wrong for "I'm playing thunderstorm sounds while you sleep."
    // Rendered as LEDs off; ambient is its own audible feedback.
    UI_STATE_AMBIENT,
    UI_STATE_MUTED,
    UI_STATE_ERROR,
} ui_state_t;

typedef void (*ui_mute_callback_t)(bool muted);

esp_err_t leds_buttons_init(ui_mute_callback_t on_mute_change);
esp_err_t leds_buttons_set_state(ui_state_t state);
bool      leds_buttons_is_muted(void);
