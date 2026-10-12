// Persistent dongle settings (NVS namespace "dongle").
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdint.h>

typedef enum {
    OUT_MODE_XINPUT = 0,   // Xbox 360 controller (default)
    OUT_MODE_DS4,          // DualShock 4
    OUT_MODE_SWITCH_PRO,   // Nintendo Switch Pro Controller
    OUT_MODE_HID,          // generic HID gamepad (step-1 mode)
    OUT_MODE_COUNT,
} out_mode_t;

void settings_init(void);           // call after nvs_flash_init()
out_mode_t settings_out_mode(void);
const char *settings_out_mode_name(out_mode_t mode);

// Saves the next output mode and restarts the dongle so the PC sees the new
// USB device. Does not return.
void settings_cycle_out_mode(void);
