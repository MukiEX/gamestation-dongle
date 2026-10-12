// SPDX-License-Identifier: GPL-3.0-or-later
#include "settings.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

static const char *TAG = "settings";

static out_mode_t s_mode = OUT_MODE_XINPUT;

void settings_init(void)
{
    nvs_handle_t h;
    if (nvs_open("dongle", NVS_READONLY, &h) == ESP_OK) {
        uint8_t v;
        if (nvs_get_u8(h, "mode", &v) == ESP_OK && v < OUT_MODE_COUNT) {
            s_mode = (out_mode_t)v;
        }
        nvs_close(h);
    }
    ESP_LOGI(TAG, "output mode: %s", settings_out_mode_name(s_mode));
}

out_mode_t settings_out_mode(void)
{
    return s_mode;
}

const char *settings_out_mode_name(out_mode_t mode)
{
    switch (mode) {
    case OUT_MODE_XINPUT:     return "XInput (Xbox 360)";
    case OUT_MODE_DS4:        return "DualShock 4";
    case OUT_MODE_SWITCH_PRO: return "Switch Pro Controller";
    case OUT_MODE_HID:        return "generic HID gamepad";
    default:                  return "?";
    }
}

out_mode_t settings_next_out_mode(void)
{
    out_mode_t next = (out_mode_t)((s_mode + 1) % OUT_MODE_COUNT);
    nvs_handle_t h;
    if (nvs_open("dongle", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "mode", (uint8_t)next);
        nvs_commit(h);
        nvs_close(h);
    }
    s_mode = next;
    return next;
}
