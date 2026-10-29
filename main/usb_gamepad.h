// USB side of the dongle: one gamepad in the selected output mode (XInput,
// DS4, Switch Pro or generic HID) plus a CDC serial port mirroring the log.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "settings.h"

void usb_gamepad_start(void);

// The output button(s) a grip_map_t choice presses in the current mode
// (shared by the GL / GR back buttons and the Gamestation keypad).
uint32_t usb_gamepad_grip_bits(grip_map_t m, bool left);

// Wakes the report task; safe to call from any task (not from an ISR).
void usb_gamepad_poke(void);

// Switches to the next output mode (saved for the next power-up too). The
// dongle re-enumerates over USB; Bluetooth connections stay up.
void usb_gamepad_next_mode(void);

// Web configurator (USB network at 192.168.7.1). While it is open the
// controllers stay connected but nothing is sent to the PC as a gamepad.
void usb_gamepad_enter_config(void);
void usb_gamepad_exit_config(void);
bool usb_gamepad_in_config(void);

// The PC has set up the current USB device.
bool usb_gamepad_mounted(void);

// Re-enumerates in the current mode, e.g. after the log port setting changed.
void usb_gamepad_reapply(void);

void usb_gamepad_set_firmware_version(const char *v);
const char *usb_gamepad_firmware_version(void);
// True if the current USB device includes the mouse interface.
bool usb_gamepad_has_mouse(void);

// How far the host got with the USB device (shown on the T-Dongle screen,
// since a console setup has no log port).
typedef enum {
    USB_STAGE_NONE, USB_STAGE_DEVICE, USB_STAGE_CONFIG, USB_STAGE_HID, USB_STAGE_MOUNTED,
    USB_STAGE_HELLO, USB_STAGE_STREAMING,
} usb_stage_t;
void usb_gamepad_note_stage(usb_stage_t st);
usb_stage_t usb_gamepad_stage(void);
