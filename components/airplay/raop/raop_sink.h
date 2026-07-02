/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef RAOP_SINK_H
#define RAOP_SINK_H

#include <stdint.h>
#include <stdarg.h>

#define RAOP_SAMPLE_RATE	44100

typedef enum { 	RAOP_SETUP, RAOP_STREAM, RAOP_PLAY, RAOP_FLUSH, RAOP_METADATA, RAOP_ARTWORK, RAOP_PROGRESS, RAOP_PAUSE, RAOP_STOP, RAOP_STALLED, 
				RAOP_VOLUME, RAOP_TIMING, RAOP_PREV, RAOP_NEXT, RAOP_REW, RAOP_FWD, 
				RAOP_VOLUME_UP, RAOP_VOLUME_DOWN, RAOP_RESUME, RAOP_TOGGLE } raop_event_t ;

typedef bool (*raop_cmd_cb_t)(raop_event_t event, ...);
typedef bool (*raop_cmd_vcb_t)(raop_event_t event, va_list args);
typedef void (*raop_data_cb_t)(const u8_t *data, size_t len, u32_t playtime);

/**
 * @brief     init sink mode (need to be provided)
 */
void raop_sink_init(raop_cmd_vcb_t cmd_cb, raop_data_cb_t data_cb);

/**
 * @brief     deinit sink mode (need to be provided)
 */
void raop_sink_deinit(void);

/**
 * @brief     force disconnection
 */
void raop_disconnect(void);

/* ===== OE additions ====================================================
 * Hooks the OE voice-device firmware uses to drive the receiver from
 * its own event loop (we don't use squeezelite's wifi-manager).
 * ===================================================================== */

struct raop_ctx_s;

/**
 * @brief     Bring the RTSP listener + _raop._tcp mDNS service up.
 *            Call after Wi-Fi has an IP. host_ip is in network byte
 *            order (struct in_addr.s_addr).
 */
void raop_sink_start_now(uint32_t host_ip, const unsigned char mac[6], const char *name);

/**
 * @brief     Active context handle, for issuing raop_cmd().
 */
struct raop_ctx_s *raop_sink_ctx(void);

#endif /* RAOP_SINK_H*/