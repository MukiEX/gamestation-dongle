// BLE central for Switch 2 controllers: finds them, connects, runs Nintendo's
// application-level pairing and init sequence, and feeds calibrated input into
// the pad merge layer.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pad.h"

void ble_central_start(void);

// Drops every connection and forgets every paired controller.
void ble_central_forget_all(void);

// Forgets one controller (MAC little-endian, as stored) and drops its link.
void ble_central_forget(const uint8_t mac[6]);

typedef struct {
    uint8_t mac[6];   // little-endian
    pad_role_t role;
    bool ready;       // fully set up and sending input
    uint16_t rate;    // input reports per second
} ble_link_info_t;

int ble_central_links(ble_link_info_t *out, int max);
