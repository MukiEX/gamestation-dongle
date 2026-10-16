// SPDX-License-Identifier: GPL-3.0-or-later
#include "settings.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"

static void log_pad(const settings_t *s, const char *what);

static const char *TAG = "settings";

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static settings_t s_cur = {
    .out_mode = OUT_MODE_XINPUT,
    .log_port = true,
    .switch_plain = true,
    .rumble_strength = 5,
    .layout = LAYOUT_POSITIONAL,
    .deadzone_pct = 3,
    .mode_hotkeys = true,
    .grip_gl = GRIP_AUTO,
    .grip_gr = GRIP_AUTO,
    .pad_speed = 3,
    .mouse_speed = 12,
    .pad_hscroll = true,
    .ble_hid = true,
    .display_rot = 3,  // upright, turned round: plugged facing down into a dock
    .display_bright = 6,
    .scroll_speed = 5,
    .pad_lift = 0,
    .pad_ds_touch = true,
    .pad_grip = true,
    .xinput_generic = true,
};

static const struct { const char *key, *name; } s_grips[GRIP_COUNT] = {
    [GRIP_AUTO] = { "auto", "Auto" },        [GRIP_OFF] = { "off", "Nothing" },
    [GRIP_A] = { "a", "A" },                 [GRIP_B] = { "b", "B" },
    [GRIP_X] = { "x", "X" },                 [GRIP_Y] = { "y", "Y" },
    [GRIP_L] = { "l", "L" },                 [GRIP_R] = { "r", "R" },
    [GRIP_ZL] = { "zl", "ZL" },              [GRIP_ZR] = { "zr", "ZR" },
    [GRIP_LSTICK] = { "lstick", "L stick click" }, [GRIP_RSTICK] = { "rstick", "R stick click" },
    [GRIP_MINUS] = { "minus", "\xe2\x88\x92" }, [GRIP_PLUS] = { "plus", "+" },
    [GRIP_HOME] = { "home", "Home" },        [GRIP_CAPTURE] = { "capture", "Capture" },
    [GRIP_UP] = { "up", "D-pad up" },        [GRIP_DOWN] = { "down", "D-pad down" },
    [GRIP_LEFT] = { "left", "D-pad left" },  [GRIP_RIGHT] = { "right", "D-pad right" },
    [GRIP_TP_LEFT] = { "tp_left", "Touchpad left (DS4)" },
    [GRIP_TP_RIGHT] = { "tp_right", "Touchpad right (DS4)" },
};

const char *settings_grip_key(grip_map_t m)
{
    return m < GRIP_COUNT ? s_grips[m].key : "auto";
}

const char *settings_grip_name(grip_map_t m)
{
    return m < GRIP_COUNT ? s_grips[m].name : "Auto";
}

const char *settings_flip_code(uint8_t m, char *buf)
{
    snprintf(buf, 20, "gyro %c%c%c accel %c%c%c", (m & MOTION_FLIP_GX) ? '-' : '+', (m & MOTION_FLIP_GY) ? '-' : '+',
             (m & MOTION_FLIP_GZ) ? '-' : '+', (m & MOTION_FLIP_AX) ? '-' : '+', (m & MOTION_FLIP_AY) ? '-' : '+',
             (m & MOTION_FLIP_AZ) ? '-' : '+');
    return buf;
}

grip_map_t settings_grip_from_key(const char *key)
{
    for (int i = 0; i < GRIP_COUNT; i++) {
        if (strcmp(key, s_grips[i].key) == 0) return (grip_map_t)i;
    }
    return GRIP_COUNT;
}

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
        s.switch_plain = get_u8(h, "swplain", s.switch_plain, 1);
        s.rumble_strength = get_u8(h, "rumble", s.rumble_strength, SETTINGS_RUMBLE_MAX);
        s.layout = (button_layout_t)get_u8(h, "layout", s.layout, LAYOUT_LABELS);
        s.deadzone_pct = get_u8(h, "deadzone", s.deadzone_pct, SETTINGS_DEADZONE_MAX);
        s.mode_hotkeys = get_u8(h, "hotkeys", s.mode_hotkeys, 1);
        s.grip_gl = (grip_map_t)get_u8(h, "gl", s.grip_gl, GRIP_COUNT - 1);
        s.grip_gr = (grip_map_t)get_u8(h, "gr", s.grip_gr, GRIP_COUNT - 1);
        s.flip_ds4 = get_u8(h, "flipds4", s.flip_ds4, MOTION_FLIP_ALL);
        s.flip_switch = get_u8(h, "flipsw", s.flip_switch, MOTION_FLIP_ALL);
        // "padspd3": 0.6.7 moved the default to 0.3x, so older saved speeds
        // (key "padspd2") are left behind on purpose.
        s.pad_speed = get_u8(h, "padspd3", s.pad_speed, SETTINGS_PAD_SPEED_MAX);
        if (s.pad_speed == 0) s.pad_speed = 1;
        // "mousespd2": 0.6.9 moved the default to speed 3 (older saves are left behind).
        s.mouse_speed = get_u8(h, "mousespd2", s.mouse_speed, SETTINGS_PAD_SPEED_MAX);
        s.pad_hscroll = get_u8(h, "hscroll", s.pad_hscroll, 1);
        s.ble_hid = get_u8(h, "blehid", s.ble_hid, 1);
        s.m64_nso = get_u8(h, "m64nso", s.m64_nso, 1);
        s.display_rot = get_u8(h, "disprot", s.display_rot, 3);
        s.display_bright = get_u8(h, "dispbright", s.display_bright, 10);
        s.scroll_speed = get_u8(h, "scrollspd", s.scroll_speed, 10);
        if (s.scroll_speed == 0) s.scroll_speed = 1;
        // "...2" keys: 1.0.1 changed these defaults (on), so older saves are left behind.
        s.xinput_generic = get_u8(h, "xigeneric2", s.xinput_generic, 1);
        if (s.mouse_speed == 0) s.mouse_speed = 1;
        s.pad_lift = get_u8(h, "padlift", s.pad_lift, SETTINGS_PAD_LIFT_MAX);
        s.pad_grip = get_u8(h, "padgrip2", s.pad_grip, 1);
        s.pad_ds_touch = get_u8(h, "padds", s.pad_ds_touch, 1);
        s.pad_mouse = get_u8(h, "padmouse", s.pad_mouse, PAD_MOUSE_MAX);
        s_cur = s;
        nvs_close(h);
    }
    ESP_LOGI(TAG, "output mode: %s; log port %s; rumble %u/10; %s buttons; deadzone %u%%",
             settings_out_mode_name(s_cur.out_mode), s_cur.log_port ? "on" : "off", s_cur.rumble_strength,
             s_cur.layout == LAYOUT_LABELS ? "label" : "positional", s_cur.deadzone_pct);
    log_pad(&s_cur, "joy-con sensors");
}

static void speed_name(uint8_t v, char *out, size_t n)
{
    if (v >= 10) snprintf(out, n, "%u", v - 9);
    else snprintf(out, n, "0.%ux", v);
}

static void log_pad(const settings_t *s, const char *what)
{
    static const char *const lift[] = { "low", "medium", "high" };
    static const char *const mouse[] = { "controller", "USB mouse", "USB mouse pointer only" };
    char sp[8], ms[8];
    speed_name(s->pad_speed, sp, sizeof(sp));
    speed_name(s->mouse_speed, ms, sizeof(ms));
    ESP_LOGI(TAG, "%s: mouse sensors -> %s; trackpad speed %s; mouse speed %s; scroll speed %u; sideways scroll %s; lift-off %s; sensor when held: %s; "
                  "DS4/DualSense touchpad %s",
             what, mouse[s->pad_mouse <= PAD_MOUSE_MAX ? s->pad_mouse : 0], sp, ms, s->scroll_speed, s->pad_hscroll ? "on" : "off", lift[s->pad_lift <= 2 ? s->pad_lift : 0],
             s->pad_grip ? "grip sense" : "nothing", s->pad_ds_touch ? "on" : "off");
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
    if (s.grip_gl >= GRIP_COUNT) s.grip_gl = GRIP_AUTO;
    if (s.grip_gr >= GRIP_COUNT) s.grip_gr = GRIP_AUTO;
    s.flip_ds4 &= MOTION_FLIP_ALL;
    s.flip_switch &= MOTION_FLIP_ALL;
    if (s.pad_speed < 1) s.pad_speed = 1;
    if (s.pad_speed > SETTINGS_PAD_SPEED_MAX) s.pad_speed = SETTINGS_PAD_SPEED_MAX;
    if (s.pad_lift > SETTINGS_PAD_LIFT_MAX) s.pad_lift = SETTINGS_PAD_LIFT_MAX;
    if (s.pad_mouse > PAD_MOUSE_MAX) s.pad_mouse = PAD_MOUSE_OFF;
    if (s.display_bright > 10) s.display_bright = 10;
    s.display_rot &= 3;
    if (s.scroll_speed < 1) s.scroll_speed = 1;
    if (s.scroll_speed > 10) s.scroll_speed = 10;
    if (s.mouse_speed < 1) s.mouse_speed = 1;
    if (s.mouse_speed > SETTINGS_PAD_SPEED_MAX) s.mouse_speed = SETTINGS_PAD_SPEED_MAX;
    portENTER_CRITICAL(&s_mux);
    s_cur = s;
    portEXIT_CRITICAL(&s_mux);

    nvs_handle_t h;
    if (nvs_open("dongle", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "mode", (uint8_t)s.out_mode);
        nvs_set_u8(h, "logport", s.log_port);
        nvs_set_u8(h, "swplain", s.switch_plain);
        nvs_set_u8(h, "rumble", s.rumble_strength);
        nvs_set_u8(h, "layout", (uint8_t)s.layout);
        nvs_set_u8(h, "deadzone", s.deadzone_pct);
        nvs_set_u8(h, "hotkeys", s.mode_hotkeys);
        nvs_set_u8(h, "gl", (uint8_t)s.grip_gl);
        nvs_set_u8(h, "gr", (uint8_t)s.grip_gr);
        nvs_set_u8(h, "flipds4", s.flip_ds4);
        nvs_set_u8(h, "flipsw", s.flip_switch);
        nvs_set_u8(h, "padspd3", s.pad_speed);
        nvs_set_u8(h, "mousespd2", s.mouse_speed);
        nvs_set_u8(h, "hscroll", s.pad_hscroll);
        nvs_set_u8(h, "blehid", s.ble_hid);
        nvs_set_u8(h, "m64nso", s.m64_nso);
        nvs_set_u8(h, "disprot", s.display_rot);
        nvs_set_u8(h, "dispbright", s.display_bright);
        nvs_set_u8(h, "scrollspd", s.scroll_speed);
        nvs_set_u8(h, "xigeneric2", s.xinput_generic);
        nvs_set_u8(h, "padlift", s.pad_lift);
        nvs_set_u8(h, "padgrip2", s.pad_grip);
        nvs_set_u8(h, "padds", s.pad_ds_touch);
        nvs_set_u8(h, "padmouse", s.pad_mouse);
        nvs_commit(h);
        nvs_close(h);
    }
    char fd[20], fs[20];
    if (s.xinput_generic) ESP_LOGI(TAG, "saved: XInput uses the dongle's own USB ID");
    ESP_LOGI(TAG, "saved: %s; log port %s; rumble %u/10; %s buttons; deadzone %u%%; hotkeys %s; GL %s, GR %s; "
                  "DS4 motion %s; Switch Pro motion %s",
             settings_out_mode_name(s.out_mode), s.log_port ? "on" : "off", s.rumble_strength,
             s.layout == LAYOUT_LABELS ? "label" : "positional", s.deadzone_pct, s.mode_hotkeys ? "on" : "off",
             settings_grip_key(s.grip_gl), settings_grip_key(s.grip_gr),
             settings_flip_code(s.flip_ds4, fd), settings_flip_code(s.flip_switch, fs));
    log_pad(&s, "saved");
}

const out_mode_t SETTINGS_MODE_ORDER[OUT_MODE_COUNT] = {
    OUT_MODE_XINPUT, OUT_MODE_DS4, OUT_MODE_DS5, OUT_MODE_SWITCH_PRO, OUT_MODE_STEAM, OUT_MODE_HID,
};

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
    case OUT_MODE_DS5:        return "DualSense";
    case OUT_MODE_STEAM:      return "Steam Controller";
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
    case OUT_MODE_DS5:        return "ds5";
    case OUT_MODE_STEAM:      return "steam";
    default:                  return "?";
    }
}

out_mode_t settings_next_out_mode(void)
{
    settings_t s = settings_get();
    int i = 0;
    while (i < OUT_MODE_COUNT && SETTINGS_MODE_ORDER[i] != s.out_mode) i++;
    s.out_mode = SETTINGS_MODE_ORDER[(i + 1) % OUT_MODE_COUNT];
    settings_set(&s);
    return s.out_mode;
}
