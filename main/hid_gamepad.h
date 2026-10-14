// Generic HID gamepad support for Bluetooth LE controllers (HID over GATT):
// parses the controller's report map and decodes its input reports into a
// neutral gamepad state. Pure code, no Bluetooth or ESP-IDF dependencies.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HID_MAX_FIELDS 96

typedef struct {
    uint8_t report_id;   // 0 = the device uses no report IDs
    uint16_t bit_off;    // within the report, after the ID byte
    uint8_t size;        // bits
    uint16_t page;
    uint16_t usage;      // for arrays: the first usage of the range
    uint16_t usage_max;  // arrays only
    int32_t lmin, lmax;
    bool array;
} hid_field_t;

typedef struct {
    hid_field_t f[HID_MAX_FIELDS];
    int n;
    bool truncated;
} hid_layout_t;

// Neutral gamepad state decoded from input reports.
typedef struct {
    uint32_t buttons;      // Button page usages 1..32 -> bits 0..31
    float x, y, z, rz, rx, ry;  // -1..1 (y as reported: + = down)
    float brake, accel;    // 0..1 (Simulation page triggers)
    int8_t hat;            // 0 = up, clockwise to 7; -1 = centred
    bool has[8];           // which of x, y, z, rz, rx, ry, brake, accel exist
    bool home, back, menu; // Consumer page AC Home / AC Back / Menu
} hid_state_t;

enum { HID_AX_X, HID_AX_Y, HID_AX_Z, HID_AX_RZ, HID_AX_RX, HID_AX_RY, HID_AX_BRAKE, HID_AX_ACCEL };

bool hid_parse(const uint8_t *map, size_t len, hid_layout_t *out);

// Applies one input report (without the report ID byte) to `st`. Fields
// belonging to other report IDs are left as they are. Returns true if the
// report contained any gamepad field.
bool hid_decode(const hid_layout_t *lay, uint8_t report_id, const uint8_t *data, size_t len, hid_state_t *st);

// Marks which axes exist (st->has) and centres everything.
void hid_state_init(const hid_layout_t *lay, hid_state_t *st);

// Human-readable name for a usage, for the log ("Button 3", "X", "Hat"...).
const char *hid_usage_name(uint16_t page, uint16_t usage, char *buf, size_t n);
