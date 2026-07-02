// XVF3800 device control over I²C.
//
// Implements the XMOS XCORE-VOICE protocol used by the Seeed reSpeaker
// XVF3800 + XIAO ESP32-S3 board's I2S firmware variant. Cross-referenced
// with the working gillespinault/respeaker-xvf3800-vad reference project.
//
// Wire format:
//   write: [resid:8][cmd:8][len:8][data ...]      (3 + len bytes total)
//   read:  write [resid, cmd, 0] then read [status:8][data ...]
//
// `status` byte: 0 = CONTROL_SUCCESS, non-zero = error (see XMOS lib_device_control).
//
// Earlier code used the XCORE-200 service/resource/CMD_GET-or-SET protocol
// (services 0x37/0x3A/0x34 etc., LED resources 0x01/0x02). That protocol is
// not what this firmware exposes; commands using it returned status 0x05
// (BAD_RESOURCE) and the LED never reacted. This file replaces all of that.

#include "xvf3800_ctrl.h"

#include <string.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "xvf_ctrl";

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;

// ── Low-level I²C transactions ─────────────────────────────────────────────

esp_err_t xvf3800_xmos_write(uint8_t resid, uint8_t cmd, const uint8_t *data, uint8_t len)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    if (len > 32) return ESP_ERR_INVALID_SIZE;
    uint8_t buf[3 + 32];
    buf[0] = resid;
    buf[1] = cmd;
    buf[2] = len;
    if (len > 0 && data) memcpy(buf + 3, data, len);
    return i2c_master_transmit(s_dev, buf, 3 + len, pdMS_TO_TICKS(200));
}

esp_err_t xvf3800_xmos_read(uint8_t resid, uint8_t cmd, uint8_t *out, uint8_t len)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    // Bit 0x80 in the cmd byte marks this as a read; without it the XVF
    // treats the I²C write as a malformed write-command and rejects the
    // subsequent read with a non-zero status byte. The length byte is the
    // TOTAL bytes the host will read back, which is len data + 1 status byte.
    uint8_t tx[3] = { resid, (uint8_t)(cmd | XVF_CMD_READ_BIT), (uint8_t)(len + 1) };
    esp_err_t e = i2c_master_transmit(s_dev, tx, sizeof(tx), pdMS_TO_TICKS(200));
    if (e != ESP_OK) return e;
    // Short delay lets the XMOS firmware queue the response. Mirrors the
    // 5 ms wait in the third-party reference; without it the read can race
    // and return stale data.
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t rx[1 + 32];
    if ((size_t)len + 1 > sizeof(rx)) return ESP_ERR_INVALID_SIZE;
    e = i2c_master_receive(s_dev, rx, 1 + len, pdMS_TO_TICKS(200));
    if (e != ESP_OK) return e;
    if (rx[0] != 0) {
        ESP_LOGW(TAG, "xmos read resid=%u cmd=%u status=0x%02x (non-zero = command rejected)",
                 resid, cmd, rx[0]);
        return ESP_FAIL;
    }
    if (out && len) memcpy(out, rx + 1, len);
    return ESP_OK;
}

// ── Bus init ──────────────────────────────────────────────────────────────

esp_err_t xvf3800_init(void)
{
    // The Seeed XVF3800+XIAO carrier doesn't expose a reset GPIO to the
    // XIAO, so we don't manipulate one. Bus alone.
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = XVF_I2C_SCL,
        .sda_io_num = XVF_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = XVF_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev));

    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "XVF3800 I²C bus up: SDA=%d SCL=%d addr=0x%02x",
             XVF_I2C_SDA, XVF_I2C_SCL, XVF_I2C_ADDR);
    // Intentionally NO I²C transactions here. Sending any command at boot
    // (even a read-version) appears to disturb the XVF's audio engine and
    // wake-word detection collapses. We diagnose mic-safe commands later.
    return ESP_OK;
}

// ── High-level commands ───────────────────────────────────────────────────

esp_err_t xvf3800_get_dfu_version(uint8_t version[3])
{
    // Version comes from the DFU_CONTROLLER service (resid 240).
    // Response is 4 bytes: [status, major, minor, patch]. xvf3800_xmos_read
    // strips the status byte, so we ask for 3 bytes of data.
    return xvf3800_xmos_read(XVF_RESID_DFU_CONTROLLER, XVF_CMD_DFU_GETVERSION, version, 3);
}

esp_err_t xvf3800_set_led_effect(xvf_led_effect_t effect)
{
    uint8_t v = (uint8_t) effect;
    return xvf3800_xmos_write(XVF_RESID_GPO, XVF_CMD_LED_EFFECT, &v, 1);
}

esp_err_t xvf3800_set_led_brightness(uint8_t brightness)
{
    return xvf3800_xmos_write(XVF_RESID_GPO, XVF_CMD_LED_BRIGHTNESS, &brightness, 1);
}

esp_err_t xvf3800_set_led_color(uint32_t rgb)
{
    // The XVF3800 expects BGR byte order with a padding byte, per the
    // reference project: { B, G, R, 0 }.
    uint8_t data[4] = {
        (uint8_t)(rgb & 0xFF),         // B
        (uint8_t)((rgb >> 8) & 0xFF),  // G
        (uint8_t)((rgb >> 16) & 0xFF), // R
        0x00,
    };
    return xvf3800_xmos_write(XVF_RESID_GPO, XVF_CMD_LED_COLOR, data, 4);
}

esp_err_t xvf3800_set_led_doa_colors(uint32_t base_rgb, uint32_t doa_rgb)
{
    uint8_t data[8] = {
        (uint8_t)(base_rgb & 0xFF),
        (uint8_t)((base_rgb >> 8) & 0xFF),
        (uint8_t)((base_rgb >> 16) & 0xFF),
        0x00,
        (uint8_t)(doa_rgb & 0xFF),
        (uint8_t)((doa_rgb >> 8) & 0xFF),
        (uint8_t)((doa_rgb >> 16) & 0xFF),
        0x00,
    };
    return xvf3800_xmos_write(XVF_RESID_GPO, XVF_CMD_LED_DOA_COLOR, data, 8);
}

static volatile bool s_headphone_mode = false;

esp_err_t xvf3800_enable_amplifier(bool enable)
{
    // Speaker amp enable on the XVF3800 carrier is GPIO 31 on the XMOS
    // (NOT a GPIO on the XIAO). Reference: GPIO_AMP_ENABLE in the third-
    // party VAD project. LOW = enabled, HIGH = disabled.
    //
    // The XVF firmware reads this same GPIO as a "speaker active" hint for
    // its AEC and drops mic sensitivity while it's asserted, which kills
    // wake-word during playback. In headphone mode we never assert the
    // enable so the AEC keeps mic level normal — audio still reaches the
    // 3.5 mm jack since it taps the DAC before the speaker amp.
    if (enable && s_headphone_mode) return ESP_OK;
    uint8_t data[2] = { 31, enable ? 0 : 1 };
    return xvf3800_xmos_write(XVF_RESID_GPO, XVF_CMD_GPO_WRITE, data, 2);
}

void xvf3800_set_headphone_mode(bool enabled)
{
    s_headphone_mode = enabled;
    if (enabled) {
        // If the amp is currently HIGH (e.g. a TTS or AirPlay session enabled
        // it before the toggle), force it back LOW now so the running stream
        // gets mic sensitivity restored without waiting for the next gate.
        uint8_t data[2] = { 31, 1 };
        xvf3800_xmos_write(XVF_RESID_GPO, XVF_CMD_GPO_WRITE, data, 2);
    }
}

bool xvf3800_get_headphone_mode(void) { return s_headphone_mode; }

// ── I²C DFU client ───────────────────────────────────────────────────────
//
// Pushes an XVF firmware image to the XMOS Upgrade partition over I²C and
// reboots the chip into it. State machine follows the DFU 1.1 spec as
// implemented by XMOS XCORE-VOICE:
//   1. SETALTERNATE → Upgrade partition
//   2. Loop {DNLOAD 128-byte block; GETSTATUS until DNLOAD_IDLE}
//   3. Final empty DNLOAD → MANIFEST_WAIT_RESET
//   4. REBOOT → chip restarts into new firmware
//   5. Verify by reading back version

#define XVF_DFU_BLOCK_MAX          128
#define XVF_DFU_STATUS_TIMEOUT_MS  30000
#define XVF_DFU_VERIFY_TIMEOUT_MS  8000

// Per-block DNLOAD has a non-standard 5-byte header (XCORE protocol's
// "payload length" byte hard-coded to 130 + 2-byte LE data length).
static esp_err_t xvf_dfu_dnload_block(const uint8_t *data, uint8_t len)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    if (len > XVF_DFU_BLOCK_MAX) return ESP_ERR_INVALID_SIZE;
    uint8_t buf[5 + XVF_DFU_BLOCK_MAX];
    memset(buf, 0, sizeof(buf));
    buf[0] = XVF_RESID_DFU_CONTROLLER;
    buf[1] = XVF_CMD_DFU_DNLOAD;
    buf[2] = XVF_DFU_BLOCK_MAX + 2;
    buf[3] = len;
    buf[4] = 0;
    if (data && len > 0) memcpy(&buf[5], data, len);
    return i2c_master_transmit(s_dev, buf, sizeof(buf), pdMS_TO_TICKS(500));
}

static esp_err_t xvf_dfu_get_state(uint8_t *state_out)
{
    // GETSTATUS returns 5 data bytes after status: dfu_status, poll_lo, poll_mid, poll_hi, state.
    uint8_t resp[5] = {0};
    esp_err_t e = xvf3800_xmos_read(XVF_RESID_DFU_CONTROLLER, XVF_CMD_DFU_GETSTATUS, resp, 5);
    if (e != ESP_OK) return e;
    if (state_out) *state_out = resp[4];
    return ESP_OK;
}

static esp_err_t xvf_dfu_wait_idle(uint32_t timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        uint8_t state = 0xFF;
        if (xvf_dfu_get_state(&state) == ESP_OK) {
            if (state == XVF_DFU_STATE_IDLE ||
                state == XVF_DFU_STATE_DNLOAD_IDLE ||
                state == XVF_DFU_STATE_MANIFEST_WAIT_RESET) {
                return ESP_OK;
            }
            if (state == XVF_DFU_STATE_ERROR) {
                ESP_LOGE(TAG, "DFU state machine ERROR");
                return ESP_FAIL;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t xvf3800_dfu_apply(const uint8_t *bin, size_t len,
                            uint8_t expected_major, uint8_t expected_minor, uint8_t expected_patch)
{
    if (!bin || len == 0) return ESP_ERR_INVALID_ARG;
    if (!s_dev) return ESP_ERR_INVALID_STATE;

    ESP_LOGI(TAG, "DFU: starting (%u bytes)", (unsigned)len);

    uint8_t alt = XVF_DFU_ALT_UPGRADE;
    esp_err_t e = xvf3800_xmos_write(XVF_RESID_DFU_CONTROLLER, XVF_CMD_DFU_SETALTERNATE, &alt, 1);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "DFU: SETALTERNATE failed: %s", esp_err_to_name(e));
        return e;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    size_t offset = 0;
    int64_t progress_last_us = esp_timer_get_time();
    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > XVF_DFU_BLOCK_MAX) chunk = XVF_DFU_BLOCK_MAX;

        e = xvf_dfu_dnload_block(bin + offset, (uint8_t)chunk);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "DFU: DNLOAD failed at offset %u: %s", (unsigned)offset, esp_err_to_name(e));
            return e;
        }
        offset += chunk;

        e = xvf_dfu_wait_idle(XVF_DFU_STATUS_TIMEOUT_MS);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "DFU: wait-idle failed at offset %u: %s", (unsigned)offset, esp_err_to_name(e));
            return e;
        }

        int64_t now = esp_timer_get_time();
        if (now - progress_last_us > 1000000 || offset == len) {
            progress_last_us = now;
            ESP_LOGI(TAG, "DFU: %u / %u bytes (%.1f%%)",
                     (unsigned)offset, (unsigned)len, 100.0f * offset / len);
        }
    }

    e = xvf_dfu_dnload_block(NULL, 0);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "DFU: final empty DNLOAD failed: %s", esp_err_to_name(e));
        return e;
    }
    e = xvf_dfu_wait_idle(XVF_DFU_STATUS_TIMEOUT_MS);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "DFU: manifest wait failed: %s", esp_err_to_name(e));
        return e;
    }
    ESP_LOGI(TAG, "DFU: manifest complete, rebooting XVF...");

    // REBOOT may NACK because the XVF resets the bus before ACKing — harmless.
    uint8_t zero = 0;
    (void) xvf3800_xmos_write(XVF_RESID_DFU_CONTROLLER, XVF_CMD_DFU_REBOOT, &zero, 1);

    if (expected_major || expected_minor || expected_patch) {
        ESP_LOGI(TAG, "DFU: waiting for XVF to come back online...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        uint8_t ver[3] = {0};
        int64_t verify_deadline = esp_timer_get_time() + (int64_t)XVF_DFU_VERIFY_TIMEOUT_MS * 1000;
        while (esp_timer_get_time() < verify_deadline) {
            if (xvf3800_get_dfu_version(ver) == ESP_OK && (ver[0] || ver[1] || ver[2])) break;
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        if (ver[0] != expected_major || ver[1] != expected_minor || ver[2] != expected_patch) {
            ESP_LOGE(TAG, "DFU: version mismatch: got %u.%u.%u expected %u.%u.%u",
                     ver[0], ver[1], ver[2], expected_major, expected_minor, expected_patch);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "DFU: success, XVF now running %u.%u.%u", ver[0], ver[1], ver[2]);
    }
    return ESP_OK;
}

// ── UI-pattern → effect+color mapping ─────────────────────────────────────

esp_err_t xvf3800_set_led_pattern(xvf_led_pattern_t pattern)
{
    // Picked to be visually distinct without being annoying. BREATH pulses
    // softly; DOA mode lights the LED nearest the speaker for "we're
    // listening to you" feedback. Tune to taste.
    switch (pattern) {
        case XVF_LED_PATTERN_OFF:
            return xvf3800_set_led_effect(XVF_LED_EFFECT_OFF);

        case XVF_LED_PATTERN_IDLE:
            // XVF3800 i2s firmware has a built-in breathing-blue "ready"
            // indicator that LED_EFFECT_OFF alone doesn't fully suppress.
            // Zero brightness + OFF effect forces the ring dark.
            xvf3800_set_led_brightness(0);
            return xvf3800_set_led_effect(XVF_LED_EFFECT_OFF);

        case XVF_LED_PATTERN_LISTENING:
            xvf3800_set_led_brightness(200);
            xvf3800_set_led_doa_colors(XVF_RGB(0x00, 0x00, 0x80),  // base navy
                                       XVF_RGB(0xFF, 0xFF, 0xFF)); // pointer white
            return xvf3800_set_led_effect(XVF_LED_EFFECT_DOA);

        case XVF_LED_PATTERN_THINKING:
            xvf3800_set_led_brightness(180);
            xvf3800_set_led_color(XVF_RGB(0x80, 0x00, 0xFF));   // purple
            return xvf3800_set_led_effect(XVF_LED_EFFECT_BREATH);

        case XVF_LED_PATTERN_SPEAKING:
            xvf3800_set_led_brightness(200);
            xvf3800_set_led_color(XVF_RGB(0x00, 0xC0, 0x40));   // green
            return xvf3800_set_led_effect(XVF_LED_EFFECT_SINGLE);

        case XVF_LED_PATTERN_MUTE:
            xvf3800_set_led_brightness(120);
            xvf3800_set_led_color(XVF_RGB(0xFF, 0x00, 0x00));   // red
            return xvf3800_set_led_effect(XVF_LED_EFFECT_SINGLE);

        case XVF_LED_PATTERN_ERROR:
            xvf3800_set_led_brightness(180);
            xvf3800_set_led_color(XVF_RGB(0xFF, 0x40, 0x00));   // orange/red
            return xvf3800_set_led_effect(XVF_LED_EFFECT_BREATH);
    }
    return ESP_ERR_INVALID_ARG;
}
