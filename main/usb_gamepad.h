// Step-1 USB output: a generic HID gamepad plus a CDC serial port that mirrors
// the firmware log. Later steps replace the gamepad with selectable modes
// (XInput, DS4, DualSense, Switch Pro, Steam Controller...).
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

void usb_gamepad_start(void);

// Wakes the report task; safe to call from any task (not from an ISR).
void usb_gamepad_poke(void);
