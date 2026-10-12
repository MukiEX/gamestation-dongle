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

typedef struct {
    out_mode_t out_mode;
    bool log_port;           // CDC serial log port next to the controller
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
    uint8_t pad_speed;       // 1..10, 5 = default
    uint8_t pad_lift;        // 0..2: how far off the surface still counts as touching
} settings_t;

#define SETTINGS_PAD_SPEED_MAX 10
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
