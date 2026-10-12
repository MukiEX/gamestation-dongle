// SPDX-License-Identifier: GPL-3.0-or-later
#include "settings.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"

static const char *TAG = "settings";

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static settings_t s_cur = {
    .out_mode = OUT_MODE_XINPUT,
    .log_port = true,
    .rumble_strength = 5,
    .layout = LAYOUT_POSITIONAL,
    .deadzone_pct = 3,
    .mode_hotkeys = true,
};

static uint8_t get_u8(nvs_handle_t h, const char *key, uint8_t def, uint8_t max)
{
    uint8_t v;
    if (nvs_get_u8(h, key, &v) == ESP_OK && v <= max) {
        return v;
    }
    return def;
}

void settings_init(void)
{
    nvs_handle_t h;
    if (nvs_open("dongle", NVS_READONLY, &h) == ESP_OK) {
        settings_t s = s_cur;
        s.out_mode = (out_mode_t)get_u8(h, "mode", s.out_mode, OUT_MODE_COUNT - 1);
        s.log_port = get_u8(h, "logport", s.log_port, 1);
        s.rumble_strength = get_u8(h, "rumble", s.rumble_strength, SETTINGS_RUMBLE_MAX);
        s.layout = (button_layout_t)get_u8(h, "layout", s.layout, LAYOUT_LABELS);
        s.deadzone_pct = get_u8(h, "deadzone", s.deadzone_pct, SETTINGS_DEADZONE_MAX);
        s.mode_hotkeys = get_u8(h, "hotkeys", s.mode_hotkeys, 1);
        s_cur = s;
        nvs_close(h);
    }
    ESP_LOGI(TAG, "output mode: %s; log port %s; rumble %u/10; %s buttons; deadzone %u%%",
             settings_out_mode_name(s_cur.out_mode), s_cur.log_port ? "on" : "off", s_cur.rumble_strength,
             s_cur.layout == LAYOUT_LABELS ? "label" : "positional", s_cur.deadzone_pct);
}

settings_t settings_get(void)
{
    portENTER_CRITICAL(&s_mux);
    settings_t s = s_cur;
    portEXIT_CRITICAL(&s_mux);
    return s;
}

void settings_set(const settings_t *in)
{
    settings_t s = *in;
    if (s.out_mode >= OUT_MODE_COUNT) s.out_mode = OUT_MODE_XINPUT;
    if (s.rumble_strength > SETTINGS_RUMBLE_MAX) s.rumble_strength = SETTINGS_RUMBLE_MAX;
    if (s.layout > LAYOUT_LABELS) s.layout = LAYOUT_POSITIONAL;
    if (s.deadzone_pct > SETTINGS_DEADZONE_MAX) s.deadzone_pct = SETTINGS_DEADZONE_MAX;
    portENTER_CRITICAL(&s_mux);
    s_cur = s;
    portEXIT_CRITICAL(&s_mux);

    nvs_handle_t h;
    if (nvs_open("dongle", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "mode", (uint8_t)s.out_mode);
        nvs_set_u8(h, "logport", s.log_port);
        nvs_set_u8(h, "rumble", s.rumble_strength);
        nvs_set_u8(h, "layout", (uint8_t)s.layout);
        nvs_set_u8(h, "deadzone", s.deadzone_pct);
        nvs_set_u8(h, "hotkeys", s.mode_hotkeys);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "saved: %s; log port %s; rumble %u/10; %s buttons; deadzone %u%%; hotkeys %s",
             settings_out_mode_name(s.out_mode), s.log_port ? "on" : "off", s.rumble_strength,
             s.layout == LAYOUT_LABELS ? "label" : "positional", s.deadzone_pct, s.mode_hotkeys ? "on" : "off");
}

out_mode_t settings_out_mode(void)
{
    return settings_get().out_mode;
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

const char *settings_out_mode_key(out_mode_t mode)
{
    switch (mode) {
    case OUT_MODE_XINPUT:     return "xinput";
    case OUT_MODE_DS4:        return "ds4";
    case OUT_MODE_SWITCH_PRO: return "switchpro";
    case OUT_MODE_HID:        return "hid";
    default:                  return "?";
    }
}

out_mode_t settings_next_out_mode(void)
{
    settings_t s = settings_get();
    s.out_mode = (out_mode_t)((s.out_mode + 1) % OUT_MODE_COUNT);
    settings_set(&s);
    return s.out_mode;
}
