// USB side of the dongle: one gamepad in the selected output mode (XInput,
// DS4, Switch Pro or generic HID) plus a CDC serial port mirroring the log.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

void usb_gamepad_start(void);

// Wakes the report task; safe to call from any task (not from an ISR).
void usb_gamepad_poke(void);

// Switches to the next output mode (saved for the next power-up too). The
// dongle re-enumerates over USB; Bluetooth connections stay up.
void usb_gamepad_next_mode(void);
