/*
 * raop_sink.c — OE voice-device adapter (REWRITTEN 2026-05-27).
 *
 * Replaces the upstream squeezelite-esp32 raop_sink.c, which pulled in
 * platform_config / audio_controls / display / accessors /
 * network_services. We only need a thin callback registry plus a
 * start/stop pair our airplay.c driver can call once we have an IP
 * address — the wifi-up event chain is owned by main.c, not this
 * component.
 *
 * The raop_sink_init / raop_disconnect entry points keep the names
 * raop.c expects so the upstream raop.c can stay verbatim.
 *
 * License: MIT (this file). Underlying RAOP source is MIT (philippe44).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <arpa/inet.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "platform.h"
#include "raop.h"
#include "raop_sink.h"
#include "log_util.h"

static const char *TAG = "raop_sink";

/* ----------------------------------------------------------------------- */
/* logtime / logprint — declared in log_util.h, expected to be provided    */
/* by the application. Squeezelite ships its own; we wrap esp_log so       */
/* RAOP chatter lands in the normal monitor stream and respects the IDF    */
/* log-level filter.                                                       */
/* ----------------------------------------------------------------------- */
const char *logtime(void) {
    static char buf[24];
    uint64_t us = esp_timer_get_time();
    uint64_t s  = us / 1000000ULL;
    uint64_t ms = (us / 1000ULL) % 1000ULL;
    snprintf(buf, sizeof(buf), "[%llu.%03llu]", (unsigned long long)s, (unsigned long long)ms);
    return buf;
}

void logprint(const char *fmt, ...) {
    char line[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    // Strip any trailing newline the format already added — ESP_LOGI
    // appends its own.
    if (n > 0 && line[n - 1] == '\n') line[n - 1] = 0;
    ESP_LOGI(TAG, "%s", line);
}

log_level debug2level(char *level) {
    if (!level) return lINFO;
    if (!strcmp(level, "error")) return lERROR;
    if (!strcmp(level, "warn"))  return lWARN;
    if (!strcmp(level, "info"))  return lINFO;
    if (!strcmp(level, "debug")) return lDEBUG;
    if (!strcmp(level, "sdebug")) return lSDEBUG;
    return lINFO;
}

char *level2debug(log_level level) {
    switch (level) {
        case lERROR: return "error";
        case lWARN:  return "warn";
        case lINFO:  return "info";
        case lDEBUG: return "debug";
        case lSDEBUG: return "sdebug";
        default:     return "info";
    }
}

// Caller-installed handlers — airplay.c registers our own data/cmd
// callbacks here at init time.
static struct {
    raop_cmd_vcb_t cmd;
    raop_data_cb_t data;
} s_raop_cbs;

// Loglevel symbols referenced by raop.c / rtp.c / util.c via
// `extern log_level raop_loglevel` (and `extern log_level
// util_loglevel`). MUST be non-static — these are the canonical
// definitions the linker resolves the externs to.
//
// Default = lWARN: at lINFO the per-packet chatter blasts UART at
// 115k baud and starves the audio path (underrun → static during
// AirPlay playback). Bump to lINFO temporarily when debugging.
log_level raop_loglevel = lWARN;
log_level util_loglevel = lWARN;

static struct raop_ctx_s *s_raop_ctx = NULL;

// Dispatch: raop.c's RTSP/RTP threads fire this for every event. We
// just hand the varargs straight to whatever airplay.c registered —
// no display layer, no button hooks.
static bool cmd_dispatch(raop_event_t event, ...) {
    if (!s_raop_cbs.cmd) return true;
    va_list args;
    va_start(args, event);
    bool rc = s_raop_cbs.cmd(event, args);
    va_end(args);
    return rc;
}

void raop_sink_init(raop_cmd_vcb_t cmd_cb, raop_data_cb_t data_cb) {
    s_raop_cbs.cmd  = cmd_cb;
    s_raop_cbs.data = data_cb;
    ESP_LOGI(TAG, "raop_sink_init (cmd=%p data=%p)", cmd_cb, data_cb);
}

// Called by airplay.c once Wi-Fi is up and we have an IP. Brings the
// RTSP listener online and announces _raop._tcp via IDF mdns.
void raop_sink_start_now(uint32_t host_ip, const unsigned char mac[6], const char *name) {
    if (s_raop_ctx) {
        ESP_LOGW(TAG, "raop_sink_start_now called while already running — ignoring");
        return;
    }
    ESP_LOGI(TAG, "starting AirPlay on %s as \"%s\"",
             inet_ntoa(*(struct in_addr*)&host_ip), name ? name : "(null)");
    s_raop_ctx = raop_create(host_ip, (char *)(name ? name : "OE-AirPlay"),
                             (unsigned char *)mac, 0, cmd_dispatch, s_raop_cbs.data);
    if (!s_raop_ctx) {
        ESP_LOGE(TAG, "raop_create failed");
    }
}

struct raop_ctx_s *raop_sink_ctx(void) { return s_raop_ctx; }

void raop_sink_deinit(void) {
    if (!s_raop_ctx) return;
    raop_delete(s_raop_ctx);
    s_raop_ctx = NULL;
}

void raop_disconnect(void) {
    if (!s_raop_ctx) return;
    ESP_LOGI(TAG, "forced disconnection");
    if (!raop_cmd(s_raop_ctx, RAOP_STOP, NULL)) {
        // raop_cmd returns false when there's no live session — emit
        // STALLED so airplay.c can clean up its own state.
        cmd_dispatch(RAOP_STALLED);
    }
}
