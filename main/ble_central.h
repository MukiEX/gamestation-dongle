// BLE central for Switch 2 controllers: finds them, connects, runs Nintendo's
// application-level pairing and init sequence, and feeds calibrated input into
// the pad merge layer.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdbool.h>

void ble_central_start(void);

// Drops every connection and forgets every paired controller.
void ble_central_forget_all(void);
