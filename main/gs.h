// My Arcade Atari Gamestation Gamepad: report decoding and the paddle /
// trackball mapping (mouse, sticks, mouse wheel, d-pad steps, or a JP3-style
// quadrature encoder on two d-pad lines).
//
// The pad is a Bluetooth LE HID-over-GATT controller (VID 0x000E, PID 0x3412)
// with three input reports (payloads as notified, without the report ID):
//   ID 1 (12 bytes): 0 X, 1 Y, 2 Z = paddle (absolute, 0..80, wraps),
//                    3 Rz, 4 hat (unused), 5-6 buttons 1-16,
//                    7-9 keyboard-page bits (d-pad, Settings, Home, keypad),
//                    10 brake, 11 accelerator (unused)
//   ID 2 (8 bytes):  keyboard (never sent)
//   ID 3 (4 bytes):  0 buttons, 1 X, 2 Y (relative int8), 3 wheel = trackball
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pad.h"

#define GS_VID 0x000E
#define GS_PID 0x3412
#define GS_PADDLE_STEPS 81   // paddle positions per turn (0..80)

// Where the paddle goes.
typedef enum {
    GS_PADDLE_OFF = 0,
    GS_PADDLE_MOUSE_X,      // USB mouse, sideways
    GS_PADDLE_WHEEL,        // USB mouse wheel
    GS_PADDLE_LSTICK_POS,   // left stick X follows the knob's position (Atari paddle style)
    GS_PADDLE_RSTICK_POS,   // right stick X, likewise
    GS_PADDLE_LSTICK_SPIN,  // left stick X deflects with turning speed, springs back
    GS_PADDLE_RSTICK_SPIN,  // right stick X, likewise
    GS_PADDLE_DPAD,         // one d-pad left / right tap per step
    GS_PADDLE_JP3,          // quadrature encoder on two d-pad lines
    GS_PADDLE_COUNT,
} gs_paddle_target_t;

// Where the trackball goes.
typedef enum {
    GS_BALL_OFF = 0,
    GS_BALL_MOUSE,          // USB mouse
    GS_BALL_LSTICK,         // left stick deflects with rolling speed
    GS_BALL_RSTICK,         // right stick, likewise
    GS_BALL_JP3_X,          // left / right rolling drives the JP3 encoder
    GS_BALL_JP3_Y,          // up / down rolling drives the JP3 encoder
    GS_BALL_COUNT,
} gs_ball_target_t;

// The two d-pad lines carrying JP3's phases. Left / Right is the real F3
// board; Left / Up survives drivers that cancel Left + Right (Linux xpad and
// hid-nintendo, and every hat-switch output: DS4, DualSense, generic HID).
typedef enum {
    GS_JP3_LEFT_RIGHT = 0,
    GS_JP3_LEFT_UP,
    GS_JP3_COUNT,
} gs_jp3_lines_t;

// Buttons with no standard equivalent, each mappable to any output button.
enum {
    GS_KEY_KP1 = 0, GS_KEY_KP2, GS_KEY_KP3, GS_KEY_KP4, GS_KEY_KP5, GS_KEY_KP6,
    GS_KEY_KP7, GS_KEY_KP8, GS_KEY_KP9, GS_KEY_STAR, GS_KEY_KP0, GS_KEY_HASH,
    GS_KEY_CREDIT, GS_KEY_SETTINGS,
    GS_KEY_COUNT,
};

const char *gs_paddle_key(gs_paddle_target_t t);
const char *gs_paddle_name(gs_paddle_target_t t);
gs_paddle_target_t gs_paddle_from_key(const char *k);   // GS_PADDLE_COUNT if unknown
const char *gs_ball_key(gs_ball_target_t t);
const char *gs_ball_name(gs_ball_target_t t);
gs_ball_target_t gs_ball_from_key(const char *k);       // GS_BALL_COUNT if unknown
const char *gs_key_id(int key);      // "kp1", ..., "credit", "settings"
const char *gs_key_label(int key);   // "1", ..., "Credit", "Settings"

// Connection state (BLE host task / ctrl task).
void gs_link_up(void);
void gs_link_down(void);
bool gs_connected(void);

// One input report from the pad. Fills `side` with the pad's own buttons in
// the dongle's naming (positional: A east, B south, X north, Y west) and
// returns true if anything a controller shows may have changed.
bool gs_report(uint8_t report_id, const uint8_t *data, uint16_t len, pad_side_t *side);

// USB report task, right after pad_get_output(): adds the mapped keys, the
// paddle / trackball sticks and d-pad lines, and the mouse movement. `now_us`
// is esp_timer_get_time().
void gs_apply(pad_out_t *o, int64_t now_us);

// True while JP3 or d-pad steps are still queued: the report task should come
// back within a millisecond instead of waiting for its period.
bool gs_busy(void);

// Whether the current settings send anything to a USB mouse.
bool gs_wants_mouse(void);

// For the configurator's live view.
typedef struct {
    bool connected;
    int paddle_raw;        // -1 = no report yet
    int32_t paddle_total;  // unwrapped steps since connecting
    int32_t ball_x, ball_y;
    uint32_t ball_clipped, ball_reports;
    uint16_t keys;         // GS_KEY_* bits held
    uint8_t jp3_phase;     // 0..3 (00, 10, 11, 01 as A B)
    int32_t jp3_backlog;   // whole steps still to send
} gs_live_t;
void gs_live(gs_live_t *out);
