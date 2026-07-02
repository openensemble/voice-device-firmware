// oe_udplog.c — best-effort UDP diagnostic forwarder.
//
// Ships short diagnostic lines (boot/reset reason, the [hb] heartbeat with
// RSSI + heap + stack high-water marks, [ambient-stats]) to the OE host over
// connectionless UDP so a Wi-Fi-only device can be watched like a serial
// console. Unlike forwarding over the control WebSocket, UDP datagrams keep
// getting through while the WS is dropping/reconnecting — which is the exact
// moment we need visibility. Best-effort by design: non-blocking sendto, every
// failure dropped silently. It NEVER logs on its own send path (a future
// esp_log vprintf hook would otherwise recurse).
#include "oe_client.h"
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"

static const char *TAG = "udplog";
static int                s_sock  = -1;
static struct sockaddr_in s_dest;
static volatile bool      s_ready = false;

esp_err_t oe_udplog_init(const char *server_url, uint16_t port)
{
    if (!server_url || !*server_url || port == 0) return ESP_ERR_INVALID_ARG;

    // Extract the bare host from server_url: strip scheme, then stop at the
    // first ':' (port) or '/' (path). Same shape oe_ws.c assumes.
    const char *h = server_url;
    if      (strncasecmp(h, "https://", 8) == 0) h += 8;
    else if (strncasecmp(h, "http://",  7) == 0) h += 7;
    char host[64];
    size_t i = 0;
    while (h[i] && h[i] != ':' && h[i] != '/' && i < sizeof(host) - 1) { host[i] = h[i]; i++; }
    host[i] = '\0';
    if (!host[0]) return ESP_ERR_INVALID_ARG;

    char portstr[8];
    snprintf(portstr, sizeof(portstr), "%u", (unsigned)port);
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_DGRAM };
    struct addrinfo *res = NULL;
    if (getaddrinfo(host, portstr, &hints, &res) != 0 || !res) {
        ESP_LOGW(TAG, "init: cannot resolve %s", host);
        return ESP_FAIL;
    }
    memcpy(&s_dest, res->ai_addr, sizeof(struct sockaddr_in));
    freeaddrinfo(res);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { ESP_LOGW(TAG, "init: socket() failed"); return ESP_FAIL; }
    // Non-blocking so a full lwIP send buffer can never stall the caller task
    // (heartbeat / boot path run on modest stacks).
    int fl = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, fl | O_NONBLOCK);
    s_sock  = sock;
    s_ready = true;
    ESP_LOGI(TAG, "forwarding diagnostics to %s:%u over UDP", host, (unsigned)port);
    return ESP_OK;
}

void oe_udplog_send(const char *line)
{
    if (!s_ready || s_sock < 0 || !line) return;
    size_t len = strlen(line);
    if (len == 0) return;
    if (len > 1400) len = 1400;  // stay within one datagram / typical MTU
    // Best-effort: EWOULDBLOCK or any error just drops this line. No logging
    // here — this runs on the log path and must never recurse or block.
    sendto(s_sock, line, len, 0, (struct sockaddr *)&s_dest, sizeof(s_dest));
}
