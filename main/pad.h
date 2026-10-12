// The single virtual controller the dongle exposes, built from whatever
// Switch 2 controllers are currently connected.
//
// Rules:
//   * A Pro Controller 2 is a complete controller on its own (role FULL).
//   * A left and a right Joy-Con 2 merge into one controller.
//   * Only one controller per role: a second Joy-Con of the same side, or any
//     Joy-Con while a Pro Controller is connected, is ignored.
//   * Once a pair has formed it stays a pair until both halves are gone; if one
//     half drops out its inputs read neutral until it reconnects.
//   * A lone Joy-Con (no pair formed yet) is mapped as a sideways mini-controller.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PAD_ROLE_NONE = 0,
    PAD_ROLE_LEFT,
    PAD_ROLE_RIGHT,
    PAD_ROLE_FULL,
    PAD_ROLE_COUNT,
} pad_role_t;

typedef enum {
    PAD_MODE_NONE = 0,     // nothing connected
    PAD_MODE_FULL,         // Pro Controller 2
    PAD_MODE_PAIR,         // merged left + right Joy-Con
    PAD_MODE_SINGLE_LEFT,  // lone left Joy-Con, held sideways
    PAD_MODE_SINGLE_RIGHT, // lone right Joy-Con, held sideways
} pad_mode_t;

// Motion uses the PlayStation / SDL axis convention for a controller held
// normally: +X = right, +Y = out of the face (up when lying flat), +Z = toward
// the player. Gyro in degrees/second (right-hand rule: +X = top tilts up,
// +Y = turn left, +Z = roll left), accelerometer in g (lying flat reads
// Y = +1).
typedef struct {
    float gyro[3];
    float accel[3];
    bool valid;
} pad_motion_t;

// A virtual trackpad finger made from a Joy-Con 2's optical (mouse) sensor:
// x right, y up, +-32767 across the pad.
typedef struct {
    bool valid;   // this controller has a sensor
    bool touch;   // finger down (the Joy-Con is on a surface and moving)
    int16_t x, y;
    bool grip;    // grip-sense mode: the sensor sees the hand holding this Joy-Con
    bool grip_mode;  // grip-sense mode is active (held normally, setting on)
    bool mouse_pose; // lying on its rail side, as a mouse
} pad_trackpad_t;

// Input from one physical controller, after stick calibration. Motion is in
// the controller's own upright frame.
typedef struct {
    uint32_t buttons;  // NS2_BTN_* bits
    float lx, ly;      // [-1, 1], +y = up
    float rx, ry;
    pad_motion_t motion;
    pad_trackpad_t tp;
} pad_side_t;

// The merged virtual controller, in Switch 2 button naming with *positional*
// meaning: A = east, B = south, X = north, Y = west.
typedef struct {
    pad_mode_t mode;
    uint32_t buttons;
    float lx, ly, rx, ry;
    pad_motion_t motion;  // in the frame of the controller as the player holds it
    pad_trackpad_t tp[2]; // [0] left trackpad (left Joy-Con), [1] right (right Joy-Con)
} pad_out_t;

pad_role_t pad_role_for_pid(uint16_t pid);
const char *pad_role_name(pad_role_t role);

// Called from any task; `notify` is invoked (outside the lock) whenever the
// output may have changed.
void pad_init(void (*notify)(void));
void pad_link_up(pad_role_t role);
void pad_link_down(pad_role_t role);
void pad_update(pad_role_t role, const pad_side_t *side);
void pad_get_output(pad_out_t *out);
