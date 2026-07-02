#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define OE_WAKEWORD_SLOTS 4
#define OE_AGENT_ID_MAX   64
#define OE_TOKEN_MAX      128
#define OE_URL_MAX        256
#define OE_DEVICE_NAME_MAX 64

typedef enum {
    DEV_STATE_BOOT = 0,
    DEV_STATE_PROVISION,
    DEV_STATE_CONNECTING,
    DEV_STATE_IDLE,
    DEV_STATE_WAKE,
    DEV_STATE_CAPTURE,
    DEV_STATE_STT,
    DEV_STATE_CHAT,
    DEV_STATE_TTS,
    DEV_STATE_ERROR,
} dev_state_t;

typedef struct {
    char token[OE_TOKEN_MAX];
    char server_url[OE_URL_MAX];
    char device_name[OE_DEVICE_NAME_MAX];
    char default_agent_id[OE_AGENT_ID_MAX];
    uint8_t wake_word_slot;
    uint8_t tts_voice;
    bool muted;
} dev_config_t;

extern dev_state_t g_dev_state;
extern dev_config_t g_dev_config;

#define DEV_EVT_WAKE_DETECTED    (1 << 0)
#define DEV_EVT_VAD_END          (1 << 1)
#define DEV_EVT_STT_DONE         (1 << 2)
#define DEV_EVT_CHAT_TOKEN       (1 << 3)
#define DEV_EVT_CHAT_DONE        (1 << 4)
#define DEV_EVT_TTS_DONE         (1 << 5)
#define DEV_EVT_MUTE_TOGGLE      (1 << 6)
#define DEV_EVT_BARGE_IN         (1 << 7)
#define DEV_EVT_NET_DOWN         (1 << 8)
#define DEV_EVT_DUPLICATE_LOST   (1 << 9)

extern EventGroupHandle_t g_dev_events;
