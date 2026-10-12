// DualShock 4 output (Sony 054C:09CC). Report descriptor, feature reports and
// report layout follow GP2040-CE (MIT). Gyro and accelerometer are passed
// through from the controller; the touchpad reads as untouched.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include <math.h>
#include <string.h>

#include "esp_timer.h"
#include "ns2_proto.h"
#include "out.h"
#include "rumble.h"

#define EP_IN  0x81
#define EP_OUT 0x01

static const uint8_t report_desc[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x09, 0x30,        //   Usage (X)
    0x09, 0x31,        //   Usage (Y)
    0x09, 0x32,        //   Usage (Z)
    0x09, 0x35,        //   Usage (Rz)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x09, 0x39,        //   Usage (Hat switch)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x07,        //   Logical Maximum (7)
    0x35, 0x00,        //   Physical Minimum (0)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x65, 0x14,        //   Unit (System: English Rotation, Length: Centimeter)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x42,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)

    0x65, 0x00,        //   Unit (None)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (0x01)
    0x29, 0x0E,        //   Usage Maximum (0x0E)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x0E,        //   Report Count (14)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x06, 0x00, 0xFF,  //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x20,        //   Usage (0x20)
    0x75, 0x06,        //   Report Size (6)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x33,        //   Usage (Rx)
    0x09, 0x34,        //   Usage (Ry)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x06, 0x00, 0xFF,  //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x21,        //   Usage (0x21)
    0x95, 0x36,        //   Report Count (54)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x85, 0x05,        //   Report ID (5)
    0x09, 0x22,        //   Usage (0x22)
    0x95, 0x1F,        //   Report Count (31)
    0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

    0x85, 0x03,        //   Report ID (3)
    0x0A, 0x21, 0x27,  //   Usage (0x2721)
    0x95, 0x2F,        //   Report Count (47)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

    0x85, 0x02,        //   Report ID (2)
    0x09, 0x24,        //   Usage (0x24)
    0x95, 0x24,        //   Report Count (36)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x08,        //   Report ID (8)
    0x09, 0x25,        //   Usage (0x25)
    0x95, 0x03,        //   Report Count (3)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x10,        //   Report ID (16)
    0x09, 0x26,        //   Usage (0x26)
    0x95, 0x04,        //   Report Count (4)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x11,        //   Report ID (17)
    0x09, 0x27,        //   Usage (0x27)
    0x95, 0x02,        //   Report Count (2)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x12,        //   Report ID (18)
    0x06, 0x02, 0xFF,  //   Usage Page (Vendor Defined 0xFF02)
    0x09, 0x21,        //   Usage (0x21)
    0x95, 0x0F,        //   Report Count (15)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x13,        //   Report ID (19)
    0x09, 0x22,        //   Usage (0x22)
    0x95, 0x16,        //   Report Count (22)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x14,        //   Report ID (20)
    0x06, 0x05, 0xFF,  //   Usage Page (Vendor Defined 0xFF05)
    0x09, 0x20,        //   Usage (0x20)
    0x95, 0x10,        //   Report Count (16)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x15,        //   Report ID (21)
    0x09, 0x21,        //   Usage (0x21)
    0x95, 0x2C,        //   Report Count (44)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x06, 0x80, 0xFF,  //   Usage Page (Vendor Defined 0xFF80)
    0x85, 0x80,        //   Report ID (128)
    0x09, 0x20,        //   Usage (0x20)
    0x95, 0x06,        //   Report Count (6)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x81,        //   Report ID (129)
    0x09, 0x21,        //   Usage (0x21)
    0x95, 0x06,        //   Report Count (6)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x82,        //   Report ID (130)
    0x09, 0x22,        //   Usage (0x22)
    0x95, 0x05,        //   Report Count (5)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x83,        //   Report ID (131)
    0x09, 0x23,        //   Usage (0x23)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x84,        //   Report ID (132)
    0x09, 0x24,        //   Usage (0x24)
    0x95, 0x04,        //   Report Count (4)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x85,        //   Report ID (133)
    0x09, 0x25,        //   Usage (0x25)
    0x95, 0x06,        //   Report Count (6)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x86,        //   Report ID (134)
    0x09, 0x26,        //   Usage (0x26)
    0x95, 0x06,        //   Report Count (6)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x87,        //   Report ID (135)
    0x09, 0x27,        //   Usage (0x27)
    0x95, 0x23,        //   Report Count (35)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x88,        //   Report ID (136)
    0x09, 0x28,        //   Usage (0x28)
    0x95, 0x22,        //   Report Count (34)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x89,        //   Report ID (137)
    0x09, 0x29,        //   Usage (0x29)
    0x95, 0x02,        //   Report Count (2)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x90,        //   Report ID (144)
    0x09, 0x30,        //   Usage (0x30)
    0x95, 0x05,        //   Report Count (5)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x91,        //   Report ID (145)
    0x09, 0x31,        //   Usage (0x31)
    0x95, 0x03,        //   Report Count (3)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x92,        //   Report ID (146)
    0x09, 0x32,        //   Usage (0x32)
    0x95, 0x03,        //   Report Count (3)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x93,        //   Report ID (147)
    0x09, 0x33,        //   Usage (0x33)
    0x95, 0x0C,        //   Report Count (12)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xA0,        //   Report ID (160)
    0x09, 0x40,        //   Usage (0x40)
    0x95, 0x06,        //   Report Count (6)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xA1,        //   Report ID (161)
    0x09, 0x41,        //   Usage (0x41)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xA2,        //   Report ID (162)
    0x09, 0x42,        //   Usage (0x42)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xA3,        //   Report ID (163)
    0x09, 0x43,        //   Usage (0x43)
    0x95, 0x30,        //   Report Count (48)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xA4,        //   Report ID (164)
    0x09, 0x44,        //   Usage (0x44)
    0x95, 0x0D,        //   Report Count (13)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xA5,        //   Report ID (165)
    0x09, 0x45,        //   Usage (0x45)
    0x95, 0x15,        //   Report Count (21)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xA6,        //   Report ID (166)
    0x09, 0x46,        //   Usage (0x46)
    0x95, 0x15,        //   Report Count (21)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xA7,        //   Report ID (247)
    0x09, 0x4A,        //   Usage (0x4A)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xA8,        //   Report ID (250)
    0x09, 0x4B,        //   Usage (0x4B)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xA9,        //   Report ID (251)
    0x09, 0x4C,        //   Usage (0x4C)
    0x95, 0x08,        //   Report Count (8)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xAA,        //   Report ID (252)
    0x09, 0x4E,        //   Usage (0x4E)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xAB,        //   Report ID (253)
    0x09, 0x4F,        //   Usage (0x4F)
    0x95, 0x39,        //   Report Count (57)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xAC,        //   Report ID (254)
    0x09, 0x50,        //   Usage (0x50)
    0x95, 0x39,        //   Report Count (57)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xAD,        //   Report ID (255)
    0x09, 0x51,        //   Usage (0x51)
    0x95, 0x0B,        //   Report Count (11)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xAE,        //   Report ID (256)
    0x09, 0x52,        //   Usage (0x52)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xAF,        //   Report ID (175)
    0x09, 0x53,        //   Usage (0x53)
    0x95, 0x02,        //   Report Count (2)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xB0,        //   Report ID (176)
    0x09, 0x54,        //   Usage (0x54)
    0x95, 0x3F,        //   Report Count (63)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,              // End Collection

    0x06, 0xF0, 0xFF,  // Usage Page (Vendor Defined 0xFFF0)
    0x09, 0x40,        // Usage (0x40)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0xF0,        //   Report ID (-16) AUTH F0
    0x09, 0x47,        //   Usage (0x47)
    0x95, 0x3F,        //   Report Count (63)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF1,        //   Report ID (-15) AUTH F1
    0x09, 0x48,        //   Usage (0x48)
    0x95, 0x3F,        //   Report Count (63)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF2,        //   Report ID (-14) AUTH F2
    0x09, 0x49,        //   Usage (0x49)
    0x95, 0x0F,        //   Report Count (15)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF3,        //   Report ID (-13) Auth F3 (Reset)
    0x0A, 0x01, 0x47,  //   Usage (0x4701)
    0x95, 0x07,        //   Report Count (7)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,              // End Collection
};

static const uint8_t itf_desc[] = {
    TUD_HID_INOUT_DESCRIPTOR(0, 4, HID_ITF_PROTOCOL_NONE, sizeof(report_desc), EP_OUT, EP_IN, 64, 1),
};

// Feature 0x02: IMU calibration. Clean, symmetric values instead of one real
// controller's so the scale is exact: zero bias, gyro +-8847 for +-540 dps
// (16.384 LSB per deg/s, as a real DS4), accelerometer +-8192 per g. The
// plus/minus pairs are equal so the USB and Bluetooth layouts read the same.
static const uint8_t feature_02[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             // gyro bias pitch, yaw, roll
    0x8f, 0x22, 0x71, 0xdd, 0x8f, 0x22, 0x71, 0xdd, // gyro +/- (8847 / -8847)
    0x8f, 0x22, 0x71, 0xdd,
    0x1c, 0x02, 0x1c, 0x02,                         // gyro speed +/- 540 dps
    0x00, 0x20, 0x00, 0xe0, 0x00, 0x20, 0x00, 0xe0, // accel +/- 1 g (8192 / -8192)
    0x00, 0x20, 0x00, 0xe0,
    0x0c, 0x00,
};

// Feature 0x03: controller definition (gamepad).
static const uint8_t feature_03[] = {
    0x21, 0x27, 0x04, 0xcf, 0x00, 0x2c, 0x56, 0x08, 0x00, 0x3d, 0x00, 0xe8, 0x03, 0x04, 0x00, 0xff,
    0x7f, 0x0d, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// Feature 0xA3: firmware version and date.
static const uint8_t feature_a3[] = {
    0x4a, 0x75, 0x6e, 0x20, 0x20, 0x39, 0x20, 0x32, 0x30, 0x31, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x31, 0x32, 0x3a, 0x33, 0x36, 0x3a, 0x34, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x08, 0xb4, 0x01, 0x00, 0x00, 0x00, 0x07, 0xa0, 0x10, 0x20, 0x00, 0xa0, 0x02, 0x00,
};

static uint8_t s_report[64];
static uint8_t s_last[64];
static bool s_have_last;
static int64_t s_last_sent;
static uint8_t s_counter;
static uint16_t s_timestamp;
static int64_t s_ts_base;

static uint8_t axis8(float v)
{
    return (uint8_t)lrintf((out_clampf(v, -1.0f, 1.0f) + 1.0f) * 127.5f);
}

static void put16(uint8_t *d, float v)
{
    int16_t x = (int16_t)lrintf(out_clampf(v, -32768.0f, 32767.0f));
    d[0] = (uint8_t)x;
    d[1] = (uint8_t)((uint16_t)x >> 8);
}

// Touchpad half clicks (GL / GR by default): a click plus a finger resting on
// that half, which is how Steam Input and DS4Windows tell left from right.
// The pad is 1920 x 943; each new touch gets a new finger ID, as on a real DS4.
static void put_finger(uint8_t *f, uint8_t id, bool down, uint16_t x, uint16_t y)
{
    f[0] = (uint8_t)((id & 0x7F) | (down ? 0x00 : 0x80));
    f[1] = (uint8_t)x;
    f[2] = (uint8_t)(((x >> 8) & 0x0F) | ((y & 0x0F) << 4));
    f[3] = (uint8_t)(y >> 4);
}

static void touchpad(uint32_t b, uint8_t *r)
{
    static uint8_t ids[2];
    static bool was[2];
    static uint16_t pos[2];
    static uint8_t next_id, packet;
    bool left = (b & NS2_BTN_TP_LEFT) != 0, right = (b & NS2_BTN_TP_RIGHT) != 0;
    // Finger 1 is whichever half was pressed (the left one if both), finger 2 the other.
    bool down[2] = { left || right, left && right };
    uint16_t want[2] = { left ? 480 : 1440, 1440 };
    for (int i = 0; i < 2; i++) {
        if (down[i] && (!was[i] || pos[i] != want[i])) {
            ids[i] = next_id++ & 0x7F;  // a new touch
            pos[i] = want[i];
        }
        was[i] = down[i];
    }
    if (left || right) {
        r[7] |= 0x02;  // touchpad click
        packet++;
    }
    r[33] = 0x01;   // one touch packet
    r[34] = packet;
    put_finger(&r[35], ids[0], down[0], pos[0], 471);
    put_finger(&r[39], ids[1], down[1], pos[1], 471);
}

static bool send(const pad_out_t *o)
{
    if (!tud_hid_ready()) {
        return false;
    }
    uint32_t b = o->buttons;
    uint8_t r[64];
    memset(r, 0, sizeof(r));
    r[0] = 0x01;
    r[1] = axis8(o->lx);
    r[2] = axis8(-o->ly);  // DS4: +Y is down
    r[3] = axis8(o->rx);
    r[4] = axis8(-o->ry);
    // Positional: Square = west (Y), Cross = south (B), Circle = east (A), Triangle = north (X).
    r[5] = out_hat(b) & 0x0F;
    if (b & NS2_BTN_Y) r[5] |= 0x10;
    if (b & NS2_BTN_B) r[5] |= 0x20;
    if (b & NS2_BTN_A) r[5] |= 0x40;
    if (b & NS2_BTN_X) r[5] |= 0x80;
    if (b & NS2_BTN_L)      r[6] |= 0x01;
    if (b & NS2_BTN_R)      r[6] |= 0x02;
    if (b & NS2_BTN_ZL)     r[6] |= 0x04;
    if (b & NS2_BTN_ZR)     r[6] |= 0x08;
    if (b & NS2_BTN_MINUS)  r[6] |= 0x10;  // Share
    if (b & NS2_BTN_PLUS)   r[6] |= 0x20;  // Options
    if (b & NS2_BTN_LSTICK) r[6] |= 0x40;
    if (b & NS2_BTN_RSTICK) r[6] |= 0x80;
    if (b & NS2_BTN_HOME)   r[7] |= 0x01;  // PS
    if (b & NS2_BTN_CAPT)   r[7] |= 0x02;  // touchpad click
    r[8] = (b & NS2_BTN_ZL) ? 255 : 0;
    r[9] = (b & NS2_BTN_ZR) ? 255 : 0;
    if (o->motion.valid) {
        // 16.384 LSB per deg/s and 8192 per g, matching feature report 0x02.
        for (int i = 0; i < 3; i++) {
            put16(&r[13 + 2 * i], o->motion.gyro[i] * 16.384f);
            put16(&r[19 + 2 * i], o->motion.accel[i] * 8192.0f);
        }
    } else {
        put16(&r[21], 8192.0f);  // at rest, lying flat
    }
    r[30] = 0x1B;  // cable connected, battery full
    touchpad(b, r);

    int64_t now = esp_timer_get_time();
    // Only the counters differ when nothing changed: compare without them, but
    // still send every period so timestamps keep moving (as a real DS4 does).
    if (s_have_last && memcmp(r, s_last, sizeof(r)) == 0 && now - s_last_sent < 4000) {
        return true;
    }
    memcpy(s_last, r, sizeof(r));
    s_have_last = true;

    s_timestamp = (uint16_t)((now - s_ts_base) * 3 / 16);  // units of 5.33 us
    r[7] |= (uint8_t)(s_counter << 2);
    r[10] = (uint8_t)s_timestamp;
    r[11] = (uint8_t)(s_timestamp >> 8);
    memcpy(s_report, r, sizeof(r));
    if (!tud_hid_report(0, s_report, sizeof(s_report))) {
        s_have_last = false;
        return false;
    }
    s_counter = (s_counter + 1) & 0x3F;
    s_last_sent = now;
    return true;
}

static uint16_t copy_out(uint8_t *buf, uint16_t len, const uint8_t *src, uint16_t n)
{
    if (n > len) n = len;
    memcpy(buf, src, n);
    return n;
}

static uint16_t get_report(uint8_t id, hid_report_type_t type, uint8_t *buf, uint16_t len)
{
    if (type != HID_REPORT_TYPE_FEATURE) {
        return copy_out(buf, len, s_report + 1, sizeof(s_report) - 1);
    }
    uint8_t mac[6];
    out_get_mac(mac);
    switch (id) {
    case 0x02: return copy_out(buf, len, feature_02, sizeof(feature_02));
    case 0x03: return copy_out(buf, len, feature_03, sizeof(feature_03));
    case 0xA3: return copy_out(buf, len, feature_a3, sizeof(feature_a3));
    case 0x12: {  // device MAC (little-endian), device class, host MAC
        uint8_t r[15] = { mac[5], mac[4], mac[3], mac[2], mac[1], mac[0], 0x08, 0x25, 0x00 };
        return copy_out(buf, len, r, sizeof(r));
    }
    case 0x81: {
        uint8_t r[6] = { mac[5], mac[4], mac[3], mac[2], mac[1], mac[0] };
        return copy_out(buf, len, r, sizeof(r));
    }
    default:
        return 0;
    }
}

static void set_report(uint8_t id, hid_report_type_t type, const uint8_t *buf, uint16_t len)
{
    if (type != HID_REPORT_TYPE_OUTPUT && type != HID_REPORT_TYPE_INVALID) {
        return;
    }
    // Interrupt OUT arrives with id 0 and the report ID as the first byte.
    if (id == 0 && len > 0) {
        id = buf[0];
        buf++;
        len--;
    }
    if (id == 0x05 && len >= 5 && (buf[0] & 0x01)) {
        rumble_set_motors(buf[4], buf[3]);  // [4] = left (strong), [3] = right (weak)
    }
}

static void reset(void)
{
    s_have_last = false;
    s_counter = 0;
}

const out_driver_t out_ds4 = {
    .name = "DualShock 4",
    .vid = 0x054C, .pid = 0x09CC, .bcd_device = 0x0100,
    .manufacturer = "Sony Interactive Entertainment",
    .product = "Wireless Controller",
    .itf_desc = itf_desc, .itf_desc_len = sizeof(itf_desc), .itf_count = 1,
    .hid_report_desc = report_desc,
    .send = send,
    .reset = reset,
    .period_ms = 4,
    .get_report = get_report,
    .set_report = set_report,
};
