// Web configurator served over a USB network link (see config_server.c).
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// String descriptor index carrying the USB network MAC address. esp_tinyusb's
// network driver writes its MAC string to this index, so it has to match.
#define CONFIG_MAC_STRING_INDEX 6

// The PC-side MAC of the configurator's USB network link, as 12 hex digits.
const char *config_server_host_mac_string(void);
