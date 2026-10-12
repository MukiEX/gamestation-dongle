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
    OUT_MODE_COUNT,
} out_mode_t;

typedef enum {
    LAYOUT_POSITIONAL = 0,  // bottom face button = Xbox A / PS Cross, whatever its label
    LAYOUT_LABELS,          // Nintendo A = Xbox A, B = B, X = X, Y = Y
} button_layout_t;

typedef struct {
    out_mode_t out_mode;
    bool log_port;           // CDC serial log port next to the controller
    uint8_t rumble_strength; // 0 (off) .. 10; 5 = Switch2Connect's default
    button_layout_t layout;
    uint8_t deadzone_pct;    // radial stick deadzone, 0..30 %
    bool mode_hotkeys;       // controller hotkeys (- + for mode, Capture + Home for config)
} settings_t;

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

// Saves the next output mode in the cycle and returns it.
out_mode_t settings_next_out_mode(void);
