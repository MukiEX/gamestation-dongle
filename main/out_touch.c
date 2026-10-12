// PlayStation touchpad (DS4 / DualSense) from the Joy-Con mouse sensors and the
// touchpad half-clicks (GL / GR on Auto).
//
// The pad holds two fingers: the left half belongs to the left Joy-Con, the
// right half to the right one. A half's finger is down while that Joy-Con lies
// on its rail side and its sensor touches a surface (it then follows the
// sensor, the same virtual
// finger as Steam Controller mode) or while that half is clicked (resting at
// the half's centre). Active fingers are packed first and every new touch gets
// a new finger ID, as on a real controller.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include "out.h"

#include "ns2_proto.h"
#include "settings.h"

static uint8_t s_ids[2], s_next_id;
static bool s_was[2];

static void put_finger(uint8_t *f, uint8_t id, bool down, uint16_t x, uint16_t y)
{
    f[0] = (uint8_t)((id & 0x7F) | (down ? 0x00 : 0x80));
    f[1] = (uint8_t)x;
    f[2] = (uint8_t)(((x >> 8) & 0x0F) | ((y & 0x0F) << 4));
    f[3] = (uint8_t)(y >> 4);
}

bool out_touchpad(const pad_out_t *o, uint16_t w, uint16_t h, uint8_t fingers[8])
{
    bool sensors = settings_get().pad_ds_touch;
    bool down[2], click = false;
    uint16_t x[2], y[2];
    for (int i = 0; i < 2; i++) {
        bool clicked = (o->buttons & (i == 0 ? NS2_BTN_TP_LEFT : NS2_BTN_TP_RIGHT)) != 0;
        // Only a Joy-Con lying on its rail side (mouse use) touches the pad:
        // held normally its sensor sees the palm, not a surface.
        bool sensor = sensors && o->tp[i].valid && o->tp[i].mouse_pose && o->tp[i].touch;
        click |= clicked;
        down[i] = clicked || sensor;
        float cx = i == 0 ? w / 4.0f : w * 3.0f / 4.0f, cy = h / 2.0f;
        float fx = cx, fy = cy;
        if (sensor) {
            fx += o->tp[i].x / 32767.0f * (w / 4.0f - 8.0f);
            fy -= o->tp[i].y / 32767.0f * (h / 2.0f - 8.0f);  // pad +y is down
        }
        x[i] = (uint16_t)out_clampf(fx, 0.0f, w - 1.0f);
        y[i] = (uint16_t)out_clampf(fy, 0.0f, h - 1.0f);
        if (down[i] && !s_was[i]) {
            s_ids[i] = s_next_id++ & 0x7F;  // a new touch
        }
        s_was[i] = down[i];
    }
    // Active fingers first (some hosts only look at the first one).
    int order[2] = { 0, 1 };
    if (!down[0] && down[1]) {
        order[0] = 1;
        order[1] = 0;
    }
    for (int k = 0; k < 2; k++) {
        int i = order[k];
        put_finger(&fingers[4 * k], s_ids[i], down[i], x[i], y[i]);
    }
    return click;
}

void out_touchpad_reset(void)
{
    s_was[0] = s_was[1] = false;
}
