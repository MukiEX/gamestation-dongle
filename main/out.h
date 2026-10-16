// USB output modes. Each mode supplies its USB identity, its gamepad
// interface descriptors and how it turns the merged controller state into
// reports. usb_gamepad.c adds the CDC log port and runs the report task.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pad.h"
#include "tusb.h"

typedef struct {
    const char *name;
    uint16_t vid, pid, bcd_device;
    const char *manufacturer;
    const char *product;
    const char *serial;          // fixed serial number string (NULL: from the MAC)

    // Gamepad interface(s), starting at interface 0, without the configuration
    // header. The log port's interfaces are appended after them.
    const uint8_t *itf_desc;
    uint16_t itf_desc_len;
    uint8_t itf_count;
    // HID report descriptor (NULL for XInput, which is not HID).
    const uint8_t *hid_report_desc;

    // Called from the report task whenever the controller state may have
    // changed and at least every `period_ms`. Returns false if the endpoint
    // was busy and the call should be repeated shortly.
    bool (*send)(const pad_out_t *o);
    // Clears per-connection state before the mode (re)starts; may be NULL.
    void (*reset)(void);
    uint16_t period_ms;

    // Optional hooks around the USB stack's (re)installation.
    void (*started)(void);
    void (*stopping)(void);
    bool is_config;            // the web configurator, not a controller
    // Device class for the descriptor when there is no log port (0 = per interface).
    uint8_t dev_class, dev_subclass, dev_protocol;

    // HID GET_REPORT / SET_REPORT (and interrupt OUT) handlers; may be NULL.
    uint16_t (*get_report)(uint8_t report_id, hid_report_type_t type, uint8_t *buf, uint16_t len);
    void (*set_report)(uint8_t report_id, hid_report_type_t type, const uint8_t *buf, uint16_t len);
} out_driver_t;

extern const out_driver_t out_xinput;
extern const out_driver_t out_ds4;
extern const out_driver_t out_ds5;
extern const out_driver_t out_steam;
extern const out_driver_t out_switchpro;
extern const out_driver_t out_hid;
extern const out_driver_t out_config;

// Shared helpers for the drivers.
void usb_gamepad_poke(void);
void out_get_mac(uint8_t mac[6]);  // the dongle's Bluetooth address, MSB first

static inline float out_clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// Hat switch from the d-pad bits: 0 = N, 1 = NE ... 7 = NW, 8 = centred.
uint8_t out_hat(uint32_t buttons);

// Motion with the configurator's per-mode test flips applied (MOTION_FLIP_*).
static inline void out_motion_flipped(const pad_motion_t *m, uint8_t flips, float g[3], float a[3])
{
    for (int i = 0; i < 3; i++) {
        g[i] = (flips & (1u << i)) ? -m->gyro[i] : m->gyro[i];
        a[i] = (flips & (8u << i)) ? -m->accel[i] : m->accel[i];
    }
}

// DS4 / DualSense touchpad: fills the two 4-byte finger slots (left half from
// the left Joy-Con's mouse sensor or the left half-click, right likewise) and
// returns whether the pad is clicked. w x h is the pad's resolution.
bool out_touchpad(const pad_out_t *o, uint16_t w, uint16_t h, uint8_t fingers[8]);
void out_touchpad_reset(void);
