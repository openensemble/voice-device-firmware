#pragma once

#include <stdint.h>
#include <stdbool.h>

#define OE_WAKEWORD_SLOTS 4
#define OE_AGENT_ID_MAX   64
#define OE_TOKEN_MAX      128
#define OE_URL_MAX        256
#define OE_DEVICE_NAME_MAX 64

// NOTE: this header used to also declare a dev_state_t/g_dev_state enum and a
// g_dev_events FreeRTOS event group with DEV_EVT_* bits. Nothing ever waited
// on them — the live state machine is the ui_state_t LED state plus the
// volatile flags in main.c (s_awaiting_reply, s_stream_active, s_in_utterance,
// s_followup_until_us). Removed 2026-07-04; don't reintroduce a parallel
// state layer here.

typedef struct {
    char token[OE_TOKEN_MAX];
    char server_url[OE_URL_MAX];
    char device_name[OE_DEVICE_NAME_MAX];
    char default_agent_id[OE_AGENT_ID_MAX];
    uint8_t wake_word_slot;
    uint8_t tts_voice;
    bool muted;
} dev_config_t;

extern dev_config_t g_dev_config;
