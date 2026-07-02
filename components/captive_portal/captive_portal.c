#include "captive_portal.h"

#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "cp";

static httpd_handle_t s_httpd = NULL;
static captive_submit_callback_t s_cb = NULL;
static void *s_cb_user = NULL;
static bool s_running = false;
static TaskHandle_t s_dns_task = NULL;

static const char INDEX_HTML[] =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>OE Voice Setup</title>"
"<style>body{font-family:system-ui;max-width:380px;margin:1em auto;padding:0 1em}"
"label{display:block;margin-top:1em;font-size:.9em;color:#555}"
"input{display:block;width:100%;padding:.6em;font-size:1em;box-sizing:border-box}"
"button{margin-top:1.5em;width:100%;padding:.8em;font-size:1em;background:#1769ff;color:#fff;border:0;border-radius:4px}"
"h1{font-size:1.2em}</style></head><body>"
"<h1>Pair this voice device with OpenEnsemble</h1>"
"<form action='/submit' method='POST'>"
"<label>Wi-Fi network<input name=ssid required></label>"
"<label>Wi-Fi password<input name=password type=password></label>"
"<label>OE server URL<input name=server_url placeholder='https://your-oe.example' required></label>"
"<label>Pairing code (from OE Settings)<input name=pair_code required maxlength=8></label>"
"<label>Device name<input name=device_name placeholder='Kitchen speaker'></label>"
"<button>Pair</button></form></body></html>";

static const char DONE_HTML[] =
"<!doctype html><body style='font-family:system-ui;padding:2em'>"
"<h2>OK — pairing…</h2><p>This device will reboot and join your Wi-Fi.</p></body>";

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, sizeof(INDEX_HTML) - 1);
}

static esp_err_t captive_redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    return httpd_resp_send(req, NULL, 0);
}

static int find_field(const char *body, const char *key, char *out, size_t out_len)
{
    char needle[32];
    snprintf(needle, sizeof(needle), "%s=", key);
    const char *p = strstr(body, needle);
    if (!p) return -1;
    p += strlen(needle);
    size_t i = 0;
    while (*p && *p != '&' && i + 1 < out_len) {
        if (*p == '+') { out[i++] = ' '; p++; continue; }
        if (*p == '%' && p[1] && p[2]) {
            char hex[3] = { p[1], p[2], 0 };
            out[i++] = (char)strtol(hex, NULL, 16);
            p += 3;
            continue;
        }
        out[i++] = *p++;
    }
    out[i] = 0;
    return (int)i;
}

static esp_err_t submit_handler(httpd_req_t *req)
{
    char body[1024];
    int total = 0;
    while (total < (int)sizeof(body) - 1) {
        int got = httpd_req_recv(req, body + total, sizeof(body) - 1 - total);
        if (got <= 0) break;
        total += got;
    }
    body[total] = 0;

    captive_form_result_t r = {0};
    find_field(body, "ssid",        r.ssid,        sizeof(r.ssid));
    find_field(body, "password",    r.password,    sizeof(r.password));
    find_field(body, "server_url",  r.server_url,  sizeof(r.server_url));
    find_field(body, "pair_code",   r.pair_code,   sizeof(r.pair_code));
    find_field(body, "device_name", r.device_name, sizeof(r.device_name));

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, DONE_HTML, sizeof(DONE_HTML) - 1);

    if (s_cb) s_cb(&r, s_cb_user);
    return ESP_OK;
}

static void dns_hijack_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { vTaskDelete(NULL); return; }
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(53), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) { close(sock); vTaskDelete(NULL); return; }

    while (s_running) {
        uint8_t buf[512];
        struct sockaddr_in peer; socklen_t peer_len = sizeof(peer);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&peer, &peer_len);
        if (n < 12) continue;
        buf[2] |= 0x80; buf[3] |= 0x80;
        buf[6] = 0; buf[7] = 1; buf[8] = 0; buf[9] = 0; buf[10] = 0; buf[11] = 0;
        int qend = 12;
        while (qend < n && buf[qend] != 0) qend += buf[qend] + 1;
        qend += 5;
        if (qend + 16 > (int)sizeof(buf)) continue;
        uint8_t *a = buf + qend;
        *a++ = 0xc0; *a++ = 0x0c;
        *a++ = 0; *a++ = 1;
        *a++ = 0; *a++ = 1;
        *a++ = 0; *a++ = 0; *a++ = 0; *a++ = 60;
        *a++ = 0; *a++ = 4;
        *a++ = 192; *a++ = 168; *a++ = 4; *a++ = 1;
        sendto(sock, buf, qend + 16, 0, (struct sockaddr *)&peer, peer_len);
    }
    close(sock);
    vTaskDelete(NULL);
}

esp_err_t captive_portal_start(const char *ap_ssid, captive_submit_callback_t cb, void *user)
{
    if (s_running) return ESP_OK;
    s_cb = cb;
    s_cb_user = user;

    esp_netif_create_default_wifi_ap();
    wifi_init_config_t wic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wic));
    wifi_config_t apc = {0};
    strncpy((char *)apc.ap.ssid, ap_ssid, sizeof(apc.ap.ssid) - 1);
    apc.ap.ssid_len = strlen(ap_ssid);
    apc.ap.channel = 6;
    apc.ap.max_connection = 4;
    apc.ap.authmode = WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &apc));
    ESP_ERROR_CHECK(esp_wifi_start());

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.max_uri_handlers = 12;
    ESP_ERROR_CHECK(httpd_start(&s_httpd, &cfg));

    httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_handler };
    httpd_uri_t sub  = { .uri = "/submit", .method = HTTP_POST, .handler = submit_handler };
    httpd_uri_t cap1 = { .uri = "/generate_204", .method = HTTP_GET, .handler = captive_redirect_handler };
    httpd_uri_t cap2 = { .uri = "/hotspot-detect.html", .method = HTTP_GET, .handler = captive_redirect_handler };
    httpd_uri_t cap3 = { .uri = "/connectivity-check.html", .method = HTTP_GET, .handler = captive_redirect_handler };
    httpd_uri_t wild = { .uri = "/*", .method = HTTP_GET, .handler = captive_redirect_handler };
    httpd_register_uri_handler(s_httpd, &root);
    httpd_register_uri_handler(s_httpd, &sub);
    httpd_register_uri_handler(s_httpd, &cap1);
    httpd_register_uri_handler(s_httpd, &cap2);
    httpd_register_uri_handler(s_httpd, &cap3);
    httpd_register_uri_handler(s_httpd, &wild);

    s_running = true;
    xTaskCreate(dns_hijack_task, "cp_dns", 3072, NULL, 4, &s_dns_task);
    ESP_LOGI(TAG, "captive portal up: SSID=%s", ap_ssid);
    return ESP_OK;
}

esp_err_t captive_portal_stop(void)
{
    s_running = false;
    if (s_httpd) { httpd_stop(s_httpd); s_httpd = NULL; }
    return ESP_OK;
}

bool captive_portal_running(void) { return s_running; }
