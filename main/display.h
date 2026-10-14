// On-device display (boards with one, e.g. the LILYGO T-Dongle-S3).
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdbool.h>

#ifdef DONGLE_BOARD_TDONGLE
#define DISPLAY_PRESENT 1
#else
#define DISPLAY_PRESENT 0
#endif

// Starts the display task; does nothing on boards without a display.
void display_start(void);
