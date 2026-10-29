// SPDX-License-Identifier: GPL-3.0-or-later
#include "gs.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "ns2_proto.h"
#include "settings.h"
#include "usb_gamepad.h"

static const char *TAG = "gs";

// ---------------------------------------------------------------------------
// Names (keys for the configurator's JSON, names for people)
// ---------------------------------------------------------------------------

static const struct { const char *key, *name; } s_paddle_names[GS_PADDLE_COUNT] = {
    [GS_PADDLE_OFF] = { "off", "Nothing" },
    [GS_PADDLE_MOUSE_X] = { "mouse_x", "USB mouse (sideways)" },
    [GS_PADDLE_WHEEL] = { "wheel", "USB mouse wheel" },
    [GS_PADDLE_LSTICK_POS] = { "lstick_pos", "Left stick X, knob position" },
    [GS_PADDLE_RSTICK_POS] = { "rstick_pos", "Right stick X, knob position" },
    [GS_PADDLE_LSTICK_SPIN] = { "lstick_spin", "Left stick X, turning speed" },
    [GS_PADDLE_RSTICK_SPIN] = { "rstick_spin", "Right stick X, turning speed" },
    [GS_PADDLE_DPAD] = { "dpad", "D-pad left / right taps" },
    [GS_PADDLE_JP3] = { "jp3", "JP3 spinner (quadrature on the d-pad)" },
};

static const struct { const char *key, *name; } s_ball_names[GS_BALL_COUNT] = {
    [GS_BALL_OFF] = { "off", "Nothing" },
    [GS_BALL_MOUSE] = { "mouse", "USB mouse" },
    [GS_BALL_LSTICK] = { "lstick", "Left stick (rolling speed)" },
    [GS_BALL_RSTICK] = { "rstick", "Right stick (rolling speed)" },
    [GS_BALL_JP3_X] = { "jp3_x", "JP3 spinner, left / right rolling" },
    [GS_BALL_JP3_Y] = { "jp3_y", "JP3 spinner, up / down rolling" },
};

static const char *const s_key_ids[GS_KEY_COUNT] = {
    "kp1", "kp2", "kp3", "kp4", "kp5", "kp6", "kp7", "kp8", "kp9", "star", "kp0", "hash", "credit", "settings",
};
static const char *const s_key_labels[GS_KEY_COUNT] = {
    "1", "2", "3", "4", "5", "6", "7", "8", "9", "*", "0", "#", "Credit", "Settings",
};

const char *gs_paddle_key(gs_paddle_target_t t) { return t < GS_PADDLE_COUNT ? s_paddle_names[t].key : "off"; }
const char *gs_paddle_name(gs_paddle_target_t t) { return t < GS_PADDLE_COUNT ? s_paddle_names[t].name : "?"; }
const char *gs_ball_key(gs_ball_target_t t) { return t < GS_BALL_COUNT ? s_ball_names[t].key : "off"; }
const char *gs_ball_name(gs_ball_target_t t) { return t < GS_BALL_COUNT ? s_ball_names[t].name : "?"; }
const char *gs_key_id(int k) { return k >= 0 && k < GS_KEY_COUNT ? s_key_ids[k] : "?"; }
const char *gs_key_label(int k) { return k >= 0 && k < GS_KEY_COUNT ? s_key_labels[k] : "?"; }

gs_paddle_target_t gs_paddle_from_key(const char *k)
{
    for (int i = 0; i < GS_PADDLE_COUNT; i++) {
        if (strcmp(k, s_paddle_names[i].key) == 0) return (gs_paddle_target_t)i;
    }
    return GS_PADDLE_COUNT;
}

gs_ball_target_t gs_ball_from_key(const char *k)
{
    for (int i = 0; i < GS_BALL_COUNT; i++) {
        if (strcmp(k, s_ball_names[i].key) == 0) return (gs_ball_target_t)i;
    }
    return GS_BALL_COUNT;
}

// ---------------------------------------------------------------------------
// State. Written by the BLE side (gs_report), read and drained by the USB
// report task (gs_apply); everything shared is behind s_mux.
// ---------------------------------------------------------------------------

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_connected;
static int s_paddle_raw = -1;     // last paddle position, -1 before the first report
static int32_t s_paddle_total;    // unwrapped position since connecting (live view)
static int32_t s_paddle_new;      // steps not yet taken by gs_apply
static int32_t s_ball_dx, s_ball_dy;          // trackball counts not yet taken
static int32_t s_ball_x, s_ball_y;            // totals (live view)
static uint32_t s_ball_clipped, s_ball_reports;
static uint16_t s_keys;           // GS_KEY_* bits
static uint32_t s_buttons;        // the pad's own buttons, dongle naming

// Owned by the USB report task.
static float s_paddle_pos;        // stick-position modes: steps from centre, clamped
static float s_spin_l, s_spin_r;  // speed-spring stick deflections (x)
static float s_ball_vx, s_ball_vy;
static float s_jp3_acc;           // JP3 steps queued (fractional)
static uint8_t s_jp3_phase;       // 0..3 along 00 -> 10 -> 11 -> 01 (A B)
static int64_t s_jp3_last;
static float s_tap_acc;           // d-pad tap mode: steps queued
static int8_t s_tap_dir;          // the tap being held: -1 left, +1 right, 0 none
static int64_t s_tap_since;
static float s_mouse_frac;
static int64_t s_last_apply;
static volatile bool s_busy;

void gs_link_up(void)
{
    portENTER_CRITICAL(&s_mux);
    s_connected = true;
    s_paddle_raw = -1;
    s_paddle_total = s_paddle_new = 0;
    s_ball_dx = s_ball_dy = s_ball_x = s_ball_y = 0;
    s_ball_clipped = s_ball_reports = 0;
    s_keys = 0;
    portEXIT_CRITICAL(&s_mux);
    ESP_LOGI(TAG, "Gamestation Gamepad connected");
    usb_gamepad_poke();
}

void gs_link_down(void)
{
    portENTER_CRITICAL(&s_mux);
    s_connected = false;
    s_keys = 0;
    s_buttons = 0;
    s_paddle_new = s_ball_dx = s_ball_dy = 0;
    portEXIT_CRITICAL(&s_mux);
    ESP_LOGI(TAG, "Gamestation Gamepad disconnected");
    usb_gamepad_poke();
}

bool gs_connected(void)
{
    return s_connected;
}

// ---------------------------------------------------------------------------
// Input reports (BLE host task)
// ---------------------------------------------------------------------------

bool gs_report(uint8_t id, const uint8_t *d, uint16_t len, pad_side_t *side)
{
    bool changed = false;
    if (id == 1 && len >= 10) {
        uint32_t b = 0;
        // Byte 5: buttons 1-8 = X, A, L1, B, Y, R1, Select, Start. Positional
        // naming (pad.h): A east, B south, X north, Y west, so the pad's A
        // (bottom) is "B", its B (right) "A", its Y (top) "X", its X (left) "Y".
        if (d[5] & 0x01) b |= NS2_BTN_Y;
        if (d[5] & 0x02) b |= NS2_BTN_B;
        if (d[5] & 0x04) b |= NS2_BTN_L;
        if (d[5] & 0x08) b |= NS2_BTN_A;
        if (d[5] & 0x10) b |= NS2_BTN_X;
        if (d[5] & 0x20) b |= NS2_BTN_R;
        if (d[5] & 0x40) b |= NS2_BTN_MINUS;
        if (d[5] & 0x80) b |= NS2_BTN_PLUS;
        // Byte 6: buttons 9-16 = L2, R2, -, -, Credit, -, -, -.
        if (d[6] & 0x01) b |= NS2_BTN_ZL;
        if (d[6] & 0x02) b |= NS2_BTN_ZR;
        // Byte 7: keyboard bits Up, Down, Left, Right, q (Settings), Home, 9 (*), 0 (#).
        if (d[7] & 0x01) b |= NS2_BTN_UP;
        if (d[7] & 0x02) b |= NS2_BTN_DOWN;
        if (d[7] & 0x04) b |= NS2_BTN_LEFT;
        if (d[7] & 0x08) b |= NS2_BTN_RIGHT;
        if (d[7] & 0x20) b |= NS2_BTN_HOME;
        uint16_t k = 0;
        for (int i = 0; i < 8; i++) {
            if (d[8] & (1u << i)) k |= 1u << (GS_KEY_KP1 + i);  // keypad 1-8
        }
        if (d[9] & 0x01) k |= 1u << GS_KEY_KP9;
        if (d[9] & 0x02) k |= 1u << GS_KEY_KP0;
        if (d[7] & 0x40) k |= 1u << GS_KEY_STAR;
        if (d[7] & 0x80) k |= 1u << GS_KEY_HASH;
        if (d[6] & 0x10) k |= 1u << GS_KEY_CREDIT;
        if (d[7] & 0x10) k |= 1u << GS_KEY_SETTINGS;

        int raw = d[2];
        portENTER_CRITICAL(&s_mux);
        if (raw < GS_PADDLE_STEPS) {
            if (s_paddle_raw >= 0) {
                int delta = raw - s_paddle_raw;
                if (delta > GS_PADDLE_STEPS / 2) delta -= GS_PADDLE_STEPS;
                else if (delta < -GS_PADDLE_STEPS / 2) delta += GS_PADDLE_STEPS;
                s_paddle_new += delta;
                s_paddle_total += delta;
                if (delta) changed = true;
            }
            s_paddle_raw = raw;
        }
        if (b != s_buttons || k != s_keys) changed = true;
        s_buttons = b;
        s_keys = k;
        portEXIT_CRITICAL(&s_mux);
    } else if (id == 3 && len >= 3) {
        int dx = (int8_t)d[1], dy = (int8_t)d[2];
        portENTER_CRITICAL(&s_mux);
        s_ball_dx += dx;
        s_ball_dy += dy;
        s_ball_x += dx;
        s_ball_y += dy;
        s_ball_reports++;
        if (dx == 127 || dx == -127 || dy == 127 || dy == -127) s_ball_clipped++;
        portEXIT_CRITICAL(&s_mux);
        changed = dx || dy;
    }
    memset(side, 0, sizeof(*side));
    side->buttons = s_buttons;
    return changed;
}

// ---------------------------------------------------------------------------
// Mapping (USB report task)
// ---------------------------------------------------------------------------

// Sensitivity setting 1..16 -> 0.25x .. 4x (4 = 1x).
static float sens(uint8_t v)
{
    return (v ? v : 4) / 4.0f;
}

bool gs_wants_mouse(void)
{
    settings_t s = settings_get();
    return s.gs_paddle == GS_PADDLE_MOUSE_X || s.gs_paddle == GS_PADDLE_WHEEL || s.gs_ball == GS_BALL_MOUSE;
}

bool gs_busy(void)
{
    return s_busy;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// Speed spring: each count kicks the deflection, which relaxes toward 0 with
// time constant TAU. A steady `full` counts per second holds full deflection.
#define SPRING_TAU 0.06f
static float spring(float v, float counts, float full, float dt)
{
    v *= expf(-dt / SPRING_TAU);
    v += counts / (full * SPRING_TAU);
    return clampf(v, -1.0f, 1.0f);
}

// The JP3 phases as d-pad lines: A on Left, B on Right or Up.
static uint32_t jp3_lines(uint8_t phase, gs_jp3_lines_t which)
{
    static const uint8_t ab[4] = { 0x0, 0x2, 0x3, 0x1 };  // bit 1 = A, bit 0 = B: 00 10 11 01
    uint32_t out = 0;
    if (ab[phase & 3] & 0x2) out |= NS2_BTN_LEFT;
    if (ab[phase & 3] & 0x1) out |= which == GS_JP3_LEFT_UP ? NS2_BTN_UP : NS2_BTN_RIGHT;
    return out;
}

static uint32_t jp3_mask(gs_jp3_lines_t which)
{
    return NS2_BTN_LEFT | (which == GS_JP3_LEFT_UP ? NS2_BTN_UP : NS2_BTN_RIGHT);
}

#define JP3_BACKLOG_MAX 128.0f   // steps; more is dropped rather than replayed for seconds

void gs_apply(pad_out_t *o, int64_t now)
{
    settings_t st = settings_get();
    int32_t paddle, bdx, bdy;
    uint16_t keys;
    bool connected;
    portENTER_CRITICAL(&s_mux);
    paddle = s_paddle_new;
    bdx = s_ball_dx;
    bdy = s_ball_dy;
    s_paddle_new = s_ball_dx = s_ball_dy = 0;
    keys = s_keys;
    connected = s_connected;
    portEXIT_CRITICAL(&s_mux);

    float dt = s_last_apply ? (now - s_last_apply) / 1e6f : 0.0f;
    if (dt > 0.1f) dt = 0.1f;
    s_last_apply = now;
    if (!connected) {
        s_busy = false;
        s_paddle_pos = s_spin_l = s_spin_r = s_ball_vx = s_ball_vy = 0.0f;
        s_jp3_acc = s_tap_acc = 0.0f;
        s_tap_dir = 0;
        return;
    }

    // Mapped keys (keypad, Credit, Settings).
    for (int i = 0; i < GS_KEY_COUNT; i++) {
        if (keys & (1u << i)) o->buttons |= usb_gamepad_grip_bits((grip_map_t)st.gs_keys[i], false);
    }

    float p = paddle * sens(st.gs_paddle_sens) * (st.gs_paddle_inv ? -1.0f : 1.0f);
    float bx = bdx * sens(st.gs_ball_sens);
    float by = bdy * sens(st.gs_ball_sens) * (st.gs_ball_inv_y ? -1.0f : 1.0f);
    float jp3_in = 0.0f;
    float mouse_dx = 0.0f, mouse_dy = 0.0f;

    // Paddle.
    float range = st.gs_paddle_range ? st.gs_paddle_range : 40;
    switch ((gs_paddle_target_t)st.gs_paddle) {
    case GS_PADDLE_MOUSE_X:
        mouse_dx += p * 8.0f;   // 8 pixels per step at 1x: about 650 per turn
        break;
    case GS_PADDLE_WHEEL:
        s_mouse_frac += p;
        {
            int w = (int)s_mouse_frac;
            s_mouse_frac -= w;
            if (w && usb_gamepad_has_mouse()) pad_wheel_add(-w);  // clockwise = scroll down
        }
        break;
    case GS_PADDLE_LSTICK_POS:
    case GS_PADDLE_RSTICK_POS:
        s_paddle_pos = clampf(s_paddle_pos + p, -range, range);
        if (st.gs_paddle == GS_PADDLE_LSTICK_POS) o->lx = s_paddle_pos / range;
        else o->rx = s_paddle_pos / range;
        break;
    case GS_PADDLE_LSTICK_SPIN:
        s_spin_l = spring(s_spin_l, p, 120.0f, dt);  // 120 steps/s (1.5 turns/s) = full
        o->lx = s_spin_l;
        break;
    case GS_PADDLE_RSTICK_SPIN:
        s_spin_r = spring(s_spin_r, p, 120.0f, dt);
        o->rx = s_spin_r;
        break;
    case GS_PADDLE_DPAD:
        s_tap_acc = clampf(s_tap_acc + p, -JP3_BACKLOG_MAX, JP3_BACKLOG_MAX);
        break;
    case GS_PADDLE_JP3:
        jp3_in += p;
        break;
    default:
        break;
    }

    // Trackball.
    switch ((gs_ball_target_t)st.gs_ball) {
    case GS_BALL_MOUSE:
        mouse_dx += bx;
        mouse_dy += by;
        break;
    case GS_BALL_LSTICK:
    case GS_BALL_RSTICK:
        s_ball_vx = spring(s_ball_vx, bx, 1500.0f, dt);  // 1500 counts/s = full
        s_ball_vy = spring(s_ball_vy, -by, 1500.0f, dt); // stick +y is up
        if (st.gs_ball == GS_BALL_LSTICK) { o->lx = s_ball_vx; o->ly = s_ball_vy; }
        else { o->rx = s_ball_vx; o->ry = s_ball_vy; }
        break;
    case GS_BALL_JP3_X:
        jp3_in += bx / 8.0f;   // a trackball counts far finer than a spinner
        break;
    case GS_BALL_JP3_Y:
        jp3_in += by / 8.0f;
        break;
    default:
        break;
    }
    // Only while the USB device has a mouse: otherwise movement would pile up
    // and arrive all at once when one appears.
    if ((mouse_dx != 0.0f || mouse_dy != 0.0f) && usb_gamepad_has_mouse()) pad_mouse_add(mouse_dx, mouse_dy);

    bool jp3 = st.gs_paddle == GS_PADDLE_JP3 || st.gs_ball == GS_BALL_JP3_X || st.gs_ball == GS_BALL_JP3_Y;
    bool busy = false;
    int64_t hold = (int64_t)(st.gs_jp3_hold ? st.gs_jp3_hold : 4) * 1000;
    if (jp3) {
        // The encoder owns its two lines: the pad's own d-pad presses on them
        // would read as phase changes.
        gs_jp3_lines_t which = (gs_jp3_lines_t)st.gs_jp3_lines;
        o->buttons &= ~jp3_mask(which);
        s_jp3_acc = clampf(s_jp3_acc + jp3_in, -JP3_BACKLOG_MAX, JP3_BACKLOG_MAX);
        // One Gray-code step per hold time, so the host never sees both
        // phases change between two of its polls.
        if (now - s_jp3_last >= hold) {
            if (s_jp3_acc >= 1.0f) { s_jp3_phase = (s_jp3_phase + 1) & 3; s_jp3_acc -= 1.0f; s_jp3_last = now; }
            else if (s_jp3_acc <= -1.0f) { s_jp3_phase = (s_jp3_phase + 3) & 3; s_jp3_acc += 1.0f; s_jp3_last = now; }
        }
        o->buttons |= jp3_lines(s_jp3_phase, which);
        busy |= s_jp3_acc >= 1.0f || s_jp3_acc <= -1.0f;
    } else {
        s_jp3_acc = 0.0f;
    }
    if (st.gs_paddle == GS_PADDLE_DPAD) {
        // Each step is a press for one hold time, then a release for one.
        if (s_tap_dir != 0) {
            if (now - s_tap_since >= hold) { s_tap_dir = 0; s_tap_since = now; }
        } else if (now - s_tap_since >= hold) {
            if (s_tap_acc >= 1.0f) { s_tap_dir = 1; s_tap_acc -= 1.0f; s_tap_since = now; }
            else if (s_tap_acc <= -1.0f) { s_tap_dir = -1; s_tap_acc += 1.0f; s_tap_since = now; }
        }
        if (s_tap_dir > 0) o->buttons |= NS2_BTN_RIGHT;
        if (s_tap_dir < 0) o->buttons |= NS2_BTN_LEFT;
        busy |= s_tap_dir != 0 || s_tap_acc >= 1.0f || s_tap_acc <= -1.0f;
    } else {
        s_tap_acc = 0.0f;
        s_tap_dir = 0;
    }
    s_busy = busy;
}

void gs_live(gs_live_t *out)
{
    portENTER_CRITICAL(&s_mux);
    out->connected = s_connected;
    out->paddle_raw = s_paddle_raw;
    out->paddle_total = s_paddle_total;
    out->ball_x = s_ball_x;
    out->ball_y = s_ball_y;
    out->ball_clipped = s_ball_clipped;
    out->ball_reports = s_ball_reports;
    out->keys = s_keys;
    portEXIT_CRITICAL(&s_mux);
    out->jp3_phase = s_jp3_phase;
    out->jp3_backlog = (int32_t)s_jp3_acc;
}
