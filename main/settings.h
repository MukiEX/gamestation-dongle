// Persistent dongle settings (NVS namespace "dongle"), edited from the web
// configurator, the BOOT button and the controller hotkeys.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    OUT_MODE_XINPUT = 0,   // Xbox 360 controller (default)
    OUT_MODE_DS4,          // DualShock 4
    OUT_MODE_SWITCH_PRO,   // Nintendo Switch Pro Controller
    OUT_MODE_HID,          // generic HID gamepad (step-1 mode)
    OUT_MODE_DS5,          // DualSense (added last so stored modes keep their numbers)
    OUT_MODE_STEAM,        // Steam Controller (2026), with the optical sensors as trackpads
    OUT_MODE_COUNT,
} out_mode_t;

// The order modes are cycled through and listed in.
extern const out_mode_t SETTINGS_MODE_ORDER[OUT_MODE_COUNT];

typedef enum {
    LAYOUT_POSITIONAL = 0,  // bottom face button = Xbox A / PS Cross, whatever its label
    LAYOUT_LABELS,          // Nintendo A = Xbox A, B = B, X = X, Y = Y
} button_layout_t;

// What the GL / GR back buttons (Pro Controller 2, Joy-Con 2 charging grip) press. AUTO picks per
// output mode: the DS4 touchpad's left / right half, the generic HID mode's
// own buttons 20 / 21, nothing in XInput and Switch Pro (no spare buttons).
typedef enum {
    GRIP_AUTO = 0,
    GRIP_OFF,
    GRIP_A, GRIP_B, GRIP_X, GRIP_Y,
    GRIP_L, GRIP_R, GRIP_ZL, GRIP_ZR,
    GRIP_LSTICK, GRIP_RSTICK,
    GRIP_MINUS, GRIP_PLUS, GRIP_HOME, GRIP_CAPTURE,
    GRIP_UP, GRIP_DOWN, GRIP_LEFT, GRIP_RIGHT,
    GRIP_TP_LEFT, GRIP_TP_RIGHT,  // DS4 touchpad left / right half click
    GRIP_COUNT,
} grip_map_t;

#define SETTINGS_GS_KEYS 14  // GS_KEY_COUNT

typedef struct {
    out_mode_t out_mode;
    bool log_port;           // CDC serial log port next to the controller
    bool single_ctrl;        // single controller mode: only the controller on USB (no log port / mouse);
                             // XInput then uses the real Xbox 360 ID. For consoles.
    uint8_t rumble_strength; // 0 (off) .. 10; 5 = Switch2Connect's default
    button_layout_t layout;
    uint8_t deadzone_pct;    // radial stick deadzone, 0..30 %
    bool mode_hotkeys;       // controller hotkeys (- + for mode, Capture + Home for config)
    grip_map_t grip_gl;      // Pro Controller 2 back buttons
    grip_map_t grip_gr;
    // Motion axis flips while the right conventions are pinned down against
    // Steam: one mask per mode, MOTION_FLIP_* bits.
    uint8_t flip_ds4;
    uint8_t flip_switch;
    // Joy-Con optical sensor as a trackpad (Steam Controller mode).
    uint8_t pad_speed;       // 1..19: 1..9 = 0.1..0.9 x speed 1, 10..19 = speeds 1..10 (default 3 = 0.3x)
    uint8_t pad_lift;        // 0..2: how far off the surface still counts as touching
    bool pad_grip;           // held normally (not on its rail side): sensor = grip sensor
    bool pad_ds_touch;       // DS4 / DualSense: the mouse sensors drive the touchpad
    uint8_t mouse_speed;     // USB mouse speed, same 1..19 steps (default 12 = speed 3)
    uint8_t display_rot;     // on-device display: 0/1 landscape, 2/3 portrait (each either way up)
    uint8_t display_bright;  // on-device display backlight 0..10
    bool ble_hid;            // pair with / connect generic Bluetooth LE HID gamepads
    bool m64_nso;            // M64 Pro: Nintendo Switch Online layout (A = A, B = B) instead of positional
    uint8_t scroll_speed;    // USB mouse wheel / sideways scroll from the stick: 1..10, 5 = default
    bool pad_hscroll;        // USB mouse: stick sideways = sideways scroll
    uint8_t pad_mouse;       // PAD_MOUSE_*: the mouse sensors drive a USB mouse instead (all modes)
    // Gamestation Gamepad (gs.h): where the paddle and trackball go.
    uint8_t gs_paddle;       // gs_paddle_target_t
    uint8_t gs_paddle_sens;  // 1..16: 0.25x .. 4x (4 = 1x)
    bool gs_paddle_inv;      // turn the other way
    uint8_t gs_paddle_range; // stick-position modes: steps from centre to full deflection (5..160)
    uint8_t gs_ball;         // gs_ball_target_t
    uint8_t gs_ball_sens;    // 1..16, as the paddle
    bool gs_ball_inv_y;
    uint8_t gs_jp3_lines;    // gs_jp3_lines_t
    uint8_t gs_jp3_hold;     // ms each encoder state (and each d-pad tap) is held, 1..16
    bool gs_jp3_yield;       // a d-pad press hands the JP3 lines back until the next spinner movement
    uint8_t gs_keys[SETTINGS_GS_KEYS];  // grip_map_t per keypad key / Credit / Settings
} settings_t;

#define SETTINGS_PAD_SPEED_MAX 19

#define PAD_MOUSE_OFF     0  // sensors feed the controller (trackpads / touchpad)
#define PAD_MOUSE_FULL    1  // USB mouse; the Joy-Con lying there lends its buttons as clicks / wheel
#define PAD_MOUSE_POINTER 2  // USB mouse pointer only; every button stays on the controller
#define PAD_MOUSE_MAX     2
#define SETTINGS_PAD_LIFT_MAX  2

#define MOTION_FLIP_GX 0x01  // gyro pitch (X)
#define MOTION_FLIP_GY 0x02  // gyro yaw (Y)
#define MOTION_FLIP_GZ 0x04  // gyro roll (Z)
#define MOTION_FLIP_AX 0x08  // accelerometer X
#define MOTION_FLIP_AY 0x10  // accelerometer Y
#define MOTION_FLIP_AZ 0x20  // accelerometer Z
#define MOTION_FLIP_ALL 0x3F

// "gyro +-+ accel +++" style code for a flip mask (buf >= 20 bytes).
const char *settings_flip_code(uint8_t mask, char *buf);

#define SETTINGS_RUMBLE_MAX   10
#define SETTINGS_DEADZONE_MAX 30

void settings_init(void);  // call after nvs_flash_init()

// Current settings (a snapshot; cheap to call).
settings_t settings_get(void);
// Validates, stores and publishes new settings.
void settings_set(const settings_t *s);

out_mode_t settings_out_mode(void);
const char *settings_out_mode_name(out_mode_t mode);
const char *settings_out_mode_key(out_mode_t mode);  // "xinput", "ds4", ...

const char *settings_grip_key(grip_map_t m);   // "auto", "off", "a", ..., "tp_left"
const char *settings_grip_name(grip_map_t m);  // for people: "Auto", "A", "Touchpad left"...
grip_map_t settings_grip_from_key(const char *key);  // GRIP_COUNT if unknown

// Saves the next output mode in the cycle and returns it.
out_mode_t settings_next_out_mode(void);
