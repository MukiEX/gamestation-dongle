// Remembers which controllers have been paired to this dongle (MAC -> product
// ID), so a controller that reconnects with a *directed* advertisement (which
// carries no manufacturer data) can still be identified before connecting.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdbool.h>
#include <stdint.h>

void bonds_init(void);
// Returns the product ID for a paired MAC (little-endian, as in ble_addr_t.val),
// or 0 if unknown.
uint16_t bonds_lookup(const uint8_t mac[6]);
void bonds_remember(const uint8_t mac[6], uint16_t pid);
void bonds_clear(void);
int bonds_count(void);
