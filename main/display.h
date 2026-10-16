// On-device display (boards with one: LILYGO T-Dongle-S3, Waveshare ESP32-S3-LCD-1.47).
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdbool.h>

#if defined(DONGLE_BOARD_TDONGLE) || defined(DONGLE_BOARD_LCD147)
#define DISPLAY_PRESENT 1
#else
#define DISPLAY_PRESENT 0
#endif

// Starts the display task; does nothing on boards without a display.
void display_start(void);
