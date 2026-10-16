// SPDX-License-Identifier: GPL-3.0-or-later
#include "pad.h"

#include "settings.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "ns2_proto.h"
#include "usb_gamepad.h"

static const char *TAG = "pad";

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static pad_side_t s_side[PAD_ROLE_COUNT];
static bool s_present[PAD_ROLE_COUNT];
static bool s_pair_latched;
static void (*s_notify)(void);

pad_role_t pad_role_for_pid(uint16_t pid)
{
    switch (pid) {
    case NS2_PID_JOYCON_L: return PAD_ROLE_LEFT;
    case NS2_PID_JOYCON_R: return PAD_ROLE_RIGHT;
    case NS2_PID_PRO2:     return PAD_ROLE_FULL;
    case PAD_PID_BLE_HID:  return PAD_ROLE_FULL;
    default:               return PAD_ROLE_NONE;
    }
}

const char *pad_role_name(pad_role_t role)
{
    switch (role) {
    case PAD_ROLE_LEFT:  return "Joy-Con L";
    case PAD_ROLE_RIGHT: return "Joy-Con R";
    case PAD_ROLE_FULL:  return "Pro Controller";
    default:             return "?";
    }
}

void pad_init(void (*notify)(void))
{
    s_notify = notify;
}


static const char *mode_name(void)
{
    if (s_present[PAD_ROLE_FULL]) return "Pro Controller";
    if (s_pair_latched) return "Joy-Con pair";
    if (s_present[PAD_ROLE_LEFT]) return "single Joy-Con L (sideways)";
    if (s_present[PAD_ROLE_RIGHT]) return "single Joy-Con R (sideways)";
    return "nothing connected";
}

static void notify(void)
{
    if (s_notify) {
        s_notify();
    }
}

void pad_link_up(pad_role_t role)
{
    if (role <= PAD_ROLE_NONE || role >= PAD_ROLE_COUNT) {
        return;
    }
    bool paired_now = false;
    portENTER_CRITICAL(&s_lock);
    memset(&s_side[role], 0, sizeof(s_side[role]));
    s_present[role] = true;
    if (s_present[PAD_ROLE_LEFT] && s_present[PAD_ROLE_RIGHT] && !s_pair_latched) {
        s_pair_latched = true;
        paired_now = true;
    }
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "%s connected", pad_role_name(role));
    if (paired_now) {
        ESP_LOGI(TAG, "Joy-Con pair merged into one controller");
    }
    ESP_LOGI(TAG, "USB output: %s", mode_name());
    notify();
}

void pad_link_down(pad_role_t role)
{
    if (role <= PAD_ROLE_NONE || role >= PAD_ROLE_COUNT) {
        return;
    }
    bool unpaired = false;
    portENTER_CRITICAL(&s_lock);
    memset(&s_side[role], 0, sizeof(s_side[role]));
    s_present[role] = false;
    if (s_pair_latched && !s_present[PAD_ROLE_LEFT] && !s_present[PAD_ROLE_RIGHT]) {
        s_pair_latched = false;
        unpaired = true;
    }
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "%s disconnected", pad_role_name(role));
    if (unpaired) {
        ESP_LOGI(TAG, "Joy-Con pair dissolved (both halves gone)");
    }
    ESP_LOGI(TAG, "USB output: %s", mode_name());
    notify();
}

void pad_update(pad_role_t role, const pad_side_t *side)
{
    if (role <= PAD_ROLE_NONE || role >= PAD_ROLE_COUNT) {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    if (s_present[role]) {
        s_side[role] = *side;
    }
    portEXIT_CRITICAL(&s_lock);
    notify();
}

static inline uint32_t mapbit(uint32_t in, uint32_t from, uint32_t to)
{
    return (in & from) ? to : 0;
}

// Lone left Joy-Con held sideways: the rail (SL/SR) faces away from the player,
// so the controller is rotated 90 degrees counter-clockwise from upright.
static void map_single_left(const pad_side_t *s, pad_out_t *o)
{
    uint32_t b = s->buttons;
    uint32_t r = 0;
    r |= mapbit(b, NS2_BTN_LEFT,  NS2_BTN_B);   // south
    r |= mapbit(b, NS2_BTN_DOWN,  NS2_BTN_A);   // east
    r |= mapbit(b, NS2_BTN_UP,    NS2_BTN_Y);   // west
    r |= mapbit(b, NS2_BTN_RIGHT, NS2_BTN_X);   // north
    r |= mapbit(b, NS2_BTN_SL_L,  NS2_BTN_L);
    r |= mapbit(b, NS2_BTN_SR_L,  NS2_BTN_R);
    r |= mapbit(b, NS2_BTN_L,     NS2_BTN_ZL);
    r |= mapbit(b, NS2_BTN_ZL,    NS2_BTN_ZR);
    r |= mapbit(b, NS2_BTN_MINUS, NS2_BTN_PLUS);
    r |= mapbit(b, NS2_BTN_LSTICK, NS2_BTN_LSTICK);
    r |= mapbit(b, NS2_BTN_CAPT,  NS2_BTN_CAPT);
    o->buttons = r;
    o->lx = -s->ly;
    o->ly = s->lx;
    o->rx = o->ry = 0.0f;
    // Rotated 90 degrees counter-clockwise about the face normal (Y):
    // the Joy-Con's top points left, its right side away from the player.
    const pad_motion_t *m = &s->motion;
    o->motion.valid = m->valid;
    o->motion.gyro[0] = m->gyro[2];   o->motion.accel[0] = m->accel[2];
    o->motion.gyro[1] = m->gyro[1];   o->motion.accel[1] = m->accel[1];
    o->motion.gyro[2] = -m->gyro[0];  o->motion.accel[2] = -m->accel[0];
}

// Lone right Joy-Con held sideways: rotated 90 degrees clockwise from upright.
static void map_single_right(const pad_side_t *s, pad_out_t *o)
{
    uint32_t b = s->buttons;
    uint32_t r = 0;
    r |= mapbit(b, NS2_BTN_A,     NS2_BTN_B);   // south
    r |= mapbit(b, NS2_BTN_X,     NS2_BTN_A);   // east
    r |= mapbit(b, NS2_BTN_B,     NS2_BTN_Y);   // west
    r |= mapbit(b, NS2_BTN_Y,     NS2_BTN_X);   // north
    r |= mapbit(b, NS2_BTN_SL_R,  NS2_BTN_L);
    r |= mapbit(b, NS2_BTN_SR_R,  NS2_BTN_R);
    r |= mapbit(b, NS2_BTN_R,     NS2_BTN_ZL);
    r |= mapbit(b, NS2_BTN_ZR,    NS2_BTN_ZR);
    r |= mapbit(b, NS2_BTN_PLUS,  NS2_BTN_PLUS);
    r |= mapbit(b, NS2_BTN_RSTICK, NS2_BTN_LSTICK);
    r |= mapbit(b, NS2_BTN_HOME,  NS2_BTN_HOME);
    r |= mapbit(b, NS2_BTN_C,     NS2_BTN_C);
    o->buttons = r;
    // The lone controller's stick drives the virtual left stick.
    o->lx = s->ry;
    o->ly = -s->rx;
    o->rx = o->ry = 0.0f;
    // Rotated 90 degrees clockwise: the top points right, the right side
    // toward the player.
    const pad_motion_t *m = &s->motion;
    o->motion.valid = m->valid;
    o->motion.gyro[0] = -m->gyro[2];  o->motion.accel[0] = -m->accel[2];
    o->motion.gyro[1] = m->gyro[1];   o->motion.accel[1] = m->accel[1];
    o->motion.gyro[2] = m->gyro[0];   o->motion.accel[2] = m->accel[0];
}

static float s_mouse_dx, s_mouse_dy;
static portMUX_TYPE s_mouse_lock = portMUX_INITIALIZER_UNLOCKED;

void pad_mouse_add(float dx, float dy)
{
    portENTER_CRITICAL(&s_mouse_lock);
    s_mouse_dx += dx;
    s_mouse_dy += dy;
    portEXIT_CRITICAL(&s_mouse_lock);
}

void pad_mouse_take(int32_t *dx, int32_t *dy)
{
    portENTER_CRITICAL(&s_mouse_lock);
    *dx = (int32_t)s_mouse_dx;  // whole pixels; the fraction waits for the next report
    *dy = (int32_t)s_mouse_dy;
    s_mouse_dx -= *dx;
    s_mouse_dy -= *dy;
    portEXIT_CRITICAL(&s_mouse_lock);
}

// Mouse setting: a Joy-Con lying on its rail side lends its shoulder (left
// click), trigger (right click), stick press (middle) and stick up/down
// (wheel) to the mouse; they leave the controller while it lies there.
static void take_mouse_controls(pad_side_t *s, bool left, pad_out_t *o)
{
    uint32_t sh = left ? NS2_BTN_L : NS2_BTN_R, tr = left ? NS2_BTN_ZL : NS2_BTN_ZR;
    uint32_t st = left ? NS2_BTN_LSTICK : NS2_BTN_RSTICK;
    if (s->buttons & sh) o->mouse_buttons |= 0x01;
    if (s->buttons & tr) o->mouse_buttons |= 0x02;
    if (s->buttons & st) o->mouse_buttons |= 0x04;
    s->buttons &= ~(sh | tr | st);
    float y = left ? s->ly : s->ry;
    if (y > 0.2f || y < -0.2f) o->mouse_wheel += y;
    float x = left ? s->lx : s->rx;
    if (settings_get().pad_hscroll && (x > 0.2f || x < -0.2f)) o->mouse_pan += x;
    if (left) s->lx = s->ly = 0.0f;
    else s->rx = s->ry = 0.0f;
}

void pad_get_output(pad_out_t *o)
{
    pad_side_t side[PAD_ROLE_COUNT];
    bool present[PAD_ROLE_COUNT];
    bool pair;
    portENTER_CRITICAL(&s_lock);
    memcpy(side, s_side, sizeof(side));
    memcpy(present, s_present, sizeof(present));
    pair = s_pair_latched;
    portEXIT_CRITICAL(&s_lock);

    memset(o, 0, sizeof(*o));
    // Pointer-only mouse setting: every button stays on the controller.
    // (Not when the USB device has no mouse, e.g. Switch Pro mode for a console.)
    if (settings_get().pad_mouse == PAD_MOUSE_FULL && usb_gamepad_has_mouse()) {
        if (present[PAD_ROLE_LEFT] && side[PAD_ROLE_LEFT].tp.mouse_pose) {
            take_mouse_controls(&side[PAD_ROLE_LEFT], true, o);
        }
        if (present[PAD_ROLE_RIGHT] && side[PAD_ROLE_RIGHT].tp.mouse_pose) {
            take_mouse_controls(&side[PAD_ROLE_RIGHT], false, o);
        }
    }
    if (present[PAD_ROLE_FULL]) {
        o->mode = PAD_MODE_FULL;
        o->buttons = side[PAD_ROLE_FULL].buttons;
        o->lx = side[PAD_ROLE_FULL].lx;
        o->ly = side[PAD_ROLE_FULL].ly;
        o->rx = side[PAD_ROLE_FULL].rx;
        o->ry = side[PAD_ROLE_FULL].ry;
        o->motion = side[PAD_ROLE_FULL].motion;
    } else if (pair) {
        // Each Joy-Con only ever sets its own bits, so the merge is a plain OR;
        // an absent half contributes zeros (neutral).
        o->mode = PAD_MODE_PAIR;
        o->buttons = side[PAD_ROLE_LEFT].buttons | side[PAD_ROLE_RIGHT].buttons;
        o->lx = side[PAD_ROLE_LEFT].lx;
        o->ly = side[PAD_ROLE_LEFT].ly;
        o->rx = side[PAD_ROLE_RIGHT].rx;
        o->ry = side[PAD_ROLE_RIGHT].ry;
        // Motion comes from the right Joy-Con (the aiming hand), or the left
        // one while the right is away.
        o->motion = side[PAD_ROLE_RIGHT].motion.valid ? side[PAD_ROLE_RIGHT].motion
                                                      : side[PAD_ROLE_LEFT].motion;
    } else if (present[PAD_ROLE_LEFT]) {
        o->mode = PAD_MODE_SINGLE_LEFT;
        map_single_left(&side[PAD_ROLE_LEFT], o);
    } else if (present[PAD_ROLE_RIGHT]) {
        o->mode = PAD_MODE_SINGLE_RIGHT;
        map_single_right(&side[PAD_ROLE_RIGHT], o);
    }
    // Each Joy-Con's sensor drives the trackpad on its own side.
    if (o->mode != PAD_MODE_FULL) {
        o->tp[0] = side[PAD_ROLE_LEFT].tp;
        o->tp[1] = side[PAD_ROLE_RIGHT].tp;
    }
}
