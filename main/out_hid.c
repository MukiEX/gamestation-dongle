// Generic HID gamepad (the step-1 output): 21 buttons, hat, two 16-bit sticks,
// two 8-bit triggers. No rumble.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include <math.h>
#include <string.h>

#include "esp_timer.h"
#include "ns2_proto.h"
#include "out.h"

#define NUM_BUTTONS 21
#define EP_IN       0x81

typedef struct __attribute__((packed)) {
    uint8_t buttons[3];  // 21 buttons + 3 padding bits
    uint8_t hat;         // low nibble: 0-7 = N, NE, E, ... ; 8 = centred
    int16_t lx, ly;      // X, Y
    int16_t rx, ry;      // Z, Rz
    uint8_t lt, rt;      // Rx, Ry (digital on Joy-Cons: 0 or 255)
} hid_report_t;

_Static_assert(sizeof(hid_report_t) == 14, "report layout");

static const uint8_t report_desc[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (1)
    0x29, NUM_BUTTONS, //   Usage Maximum (21)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, NUM_BUTTONS, //   Report Count (21)
    0x81, 0x02,        //   Input (Data, Var, Abs)
    0x75, 0x01,        //   Report Size (1)
    0x95, 24 - NUM_BUTTONS,
    0x81, 0x03,        //   Input (Const)
    0x05, 0x01,        //   Usage Page (Generic Desktop)
    0x09, 0x39,        //   Usage (Hat switch)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x07,        //   Logical Maximum (7)
    0x35, 0x00,        //   Physical Minimum (0)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x65, 0x14,        //   Unit (Eng Rot: Degrees)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x42,        //   Input (Data, Var, Abs, Null State)
    0x65, 0x00,        //   Unit (None)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x03,        //   Input (Const)
    0x09, 0x30,        //   Usage (X)
    0x09, 0x31,        //   Usage (Y)
    0x09, 0x32,        //   Usage (Z)
    0x09, 0x35,        //   Usage (Rz)
    0x16, 0x01, 0x80,  //   Logical Minimum (-32767)
    0x26, 0xFF, 0x7F,  //   Logical Maximum (32767)
    0x75, 0x10,        //   Report Size (16)
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x02,        //   Input (Data, Var, Abs)
    0x09, 0x33,        //   Usage (Rx)
    0x09, 0x34,        //   Usage (Ry)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x02,        //   Input (Data, Var, Abs)
    0xC0,              // End Collection
};

static const uint8_t itf_desc[] = {
    TUD_HID_DESCRIPTOR(0, 4, HID_ITF_PROTOCOL_NONE, sizeof(report_desc), EP_IN, 16, 1),
};

static int16_t axis16(float v)
{
    return (int16_t)lrintf(out_clampf(v, -1.0f, 1.0f) * 32767.0f);
}

static hid_report_t s_last;
static bool s_have_last;
static int64_t s_last_sent;

static bool send(const pad_out_t *o)
{
    if (!tud_hid_ready()) {
        return false;
    }
    // Positional layout: 1 = south, 2 = east, 3 = west, 4 = north.
    static const uint32_t order[NUM_BUTTONS] = {
        NS2_BTN_B, NS2_BTN_A, NS2_BTN_Y, NS2_BTN_X,
        NS2_BTN_L, NS2_BTN_R, NS2_BTN_ZL, NS2_BTN_ZR,
        NS2_BTN_MINUS, NS2_BTN_PLUS, NS2_BTN_LSTICK, NS2_BTN_RSTICK,
        NS2_BTN_HOME, NS2_BTN_CAPT, NS2_BTN_C,
        NS2_BTN_SL_L, NS2_BTN_SR_L, NS2_BTN_SL_R, NS2_BTN_SR_R,
        NS2_BTN_GL, NS2_BTN_GR,
    };
    hid_report_t r;
    memset(&r, 0, sizeof(r));
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (o->buttons & order[i]) {
            r.buttons[i / 8] |= (uint8_t)(1u << (i % 8));
        }
    }
    r.hat = out_hat(o->buttons);
    r.lx = axis16(o->lx);
    r.ly = axis16(-o->ly);  // HID: +Y is down
    r.rx = axis16(o->rx);
    r.ry = axis16(-o->ry);
    r.lt = (o->buttons & NS2_BTN_ZL) ? 255 : 0;
    r.rt = (o->buttons & NS2_BTN_ZR) ? 255 : 0;

    int64_t now = esp_timer_get_time();
    if (s_have_last && memcmp(&r, &s_last, sizeof(r)) == 0 && now - s_last_sent < 100000) {
        return true;
    }
    if (!tud_hid_report(0, &r, sizeof(r))) {
        return false;
    }
    s_last = r;
    s_have_last = true;
    s_last_sent = now;
    return true;
}

const out_driver_t out_hid = {
    .name = "generic HID gamepad",
    .vid = 0x303A, .pid = 0x8243, .bcd_device = 0x0001,
    .manufacturer = "Joy-Con 2 Dongle",
    .product = "Joy-Con 2 Dongle",
    .itf_desc = itf_desc, .itf_desc_len = sizeof(itf_desc), .itf_count = 1,
    .hid_report_desc = report_desc,
    .send = send,
    .period_ms = 100,
};
