// DualSense output (Sony 054C:0CE6), HID part. The report descriptor is a real
// DualSense's (USB, CFI-ZCT1W, dumped by nondebug/dualsense); the input and
// effects layouts follow SDL's PS5 driver and Linux hid-playstation.
//
// Rumble: the effects report's "rumble emulation" motors (what Steam, SDL and
// most PC games send). Audio haptics (the controller's USB audio channels 3/4)
// come later; adaptive triggers and the light bar are ignored.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include <math.h>
#include <string.h>

#include "esp_timer.h"
#include "ns2_proto.h"
#include "out.h"
#include "esp_log.h"
#include "rumble.h"
#include "settings.h"

#define EP_IN  0x81
#define EP_OUT 0x01

static const uint8_t report_desc[] = {
    0x05, 0x01,             // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,             // Usage (Game Pad)
    0xA1, 0x01,             // Collection (Application)
    0x85, 0x01,             // Report ID (1)
    0x09, 0x30,             // Usage (X)
    0x09, 0x31,             // Usage (Y)
    0x09, 0x32,             // Usage (Z)
    0x09, 0x35,             // Usage (Rz)
    0x09, 0x33,             // Usage (Rx)
    0x09, 0x34,             // Usage (Ry)
    0x15, 0x00,             // Logical Minimum (0)
    0x26, 0xFF, 0x00,       // Logical Maximum (255)
    0x75, 0x08,             // Report Size (8)
    0x95, 0x06,             // Report Count (6)
    0x81, 0x02,             // Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x06, 0x00, 0xFF,       // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x20,             // Usage (0x20)
    0x95, 0x01,             // Report Count (1)
    0x81, 0x02,             // Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,             // Usage Page (Generic Desktop Ctrls)
    0x09, 0x39,             // Usage (Hat switch)
    0x15, 0x00,             // Logical Minimum (0)
    0x25, 0x07,             // Logical Maximum (7)
    0x35, 0x00,             // Physical Minimum (0)
    0x46, 0x3B, 0x01,       // Physical Maximum (315)
    0x65, 0x14,             // Unit (System: English Rotation, Length: Centimeter)
    0x75, 0x04,             // Report Size (4)
    0x95, 0x01,             // Report Count (1)
    0x81, 0x42,             // Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
    0x65, 0x00,             // Unit (None)
    0x05, 0x09,             // Usage Page (Button)
    0x19, 0x01,             // Usage Minimum (0x01)
    0x29, 0x0F,             // Usage Maximum (0x0F)
    0x15, 0x00,             // Logical Minimum (0)
    0x25, 0x01,             // Logical Maximum (1)
    0x75, 0x01,             // Report Size (1)
    0x95, 0x0F,             // Report Count (15)
    0x81, 0x02,             // Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x06, 0x00, 0xFF,       // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x21,             // Usage (0x21)
    0x95, 0x0D,             // Report Count (13)
    0x81, 0x02,             // Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x06, 0x00, 0xFF,       // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x22,             // Usage (0x22)
    0x15, 0x00,             // Logical Minimum (0)
    0x26, 0xFF, 0x00,       // Logical Maximum (255)
    0x75, 0x08,             // Report Size (8)
    0x95, 0x34,             // Report Count (52)
    0x81, 0x02,             // Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x85, 0x02,             // Report ID (2)
    0x09, 0x23,             // Usage (0x23)
    0x95, 0x2F,             // Report Count (47)
    0x91, 0x02,             // Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x05,             // Report ID (5)
    0x09, 0x33,             // Usage (0x33)
    0x95, 0x28,             // Report Count (40)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x08,             // Report ID (8)
    0x09, 0x34,             // Usage (0x34)
    0x95, 0x2F,             // Report Count (47)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x09,             // Report ID (9)
    0x09, 0x24,             // Usage (0x24)
    0x95, 0x13,             // Report Count (19)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x0A,             // Report ID (10)
    0x09, 0x25,             // Usage (0x25)
    0x95, 0x1A,             // Report Count (26)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x20,             // Report ID (32)
    0x09, 0x26,             // Usage (0x26)
    0x95, 0x3F,             // Report Count (63)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x21,             // Report ID (33)
    0x09, 0x27,             // Usage (0x27)
    0x95, 0x04,             // Report Count (4)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x22,             // Report ID (34)
    0x09, 0x40,             // Usage (0x40)
    0x95, 0x3F,             // Report Count (63)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x80,             // Report ID (-128)
    0x09, 0x28,             // Usage (0x28)
    0x95, 0x3F,             // Report Count (63)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x81,             // Report ID (-127)
    0x09, 0x29,             // Usage (0x29)
    0x95, 0x3F,             // Report Count (63)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x82,             // Report ID (-126)
    0x09, 0x2A,             // Usage (0x2A)
    0x95, 0x09,             // Report Count (9)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x83,             // Report ID (-125)
    0x09, 0x2B,             // Usage (0x2B)
    0x95, 0x3F,             // Report Count (63)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x84,             // Report ID (-124)
    0x09, 0x2C,             // Usage (0x2C)
    0x95, 0x3F,             // Report Count (63)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x85,             // Report ID (-123)
    0x09, 0x2D,             // Usage (0x2D)
    0x95, 0x02,             // Report Count (2)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xA0,             // Report ID (-96)
    0x09, 0x2E,             // Usage (0x2E)
    0x95, 0x01,             // Report Count (1)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xE0,             // Report ID (-32)
    0x09, 0x2F,             // Usage (0x2F)
    0x95, 0x3F,             // Report Count (63)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF0,             // Report ID (-16)
    0x09, 0x30,             // Usage (0x30)
    0x95, 0x3F,             // Report Count (63)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF1,             // Report ID (-15)
    0x09, 0x31,             // Usage (0x31)
    0x95, 0x3F,             // Report Count (63)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF2,             // Report ID (-14)
    0x09, 0x32,             // Usage (0x32)
    0x95, 0x0F,             // Report Count (15)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF4,             // Report ID (-12)
    0x09, 0x35,             // Usage (0x35)
    0x95, 0x3F,             // Report Count (63)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF5,             // Report ID (-11)
    0x09, 0x36,             // Usage (0x36)
    0x95, 0x03,             // Report Count (3)
    0xB1, 0x02,             // Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,                   // End Collection
};

static const uint8_t itf_desc[] = {
    TUD_HID_INOUT_DESCRIPTOR(0, 4, HID_ITF_PROTOCOL_NONE, sizeof(report_desc), EP_OUT, EP_IN, 64, 1),
};

// Feature 0x05: IMU calibration, same clean values as the DS4 mode (zero bias,
// gyro +-8847 for +-540 dps = 16.384 LSB per deg/s, accelerometer 8192 per g).
static const uint8_t feature_05[40] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             // gyro bias pitch, yaw, roll
    0x8f, 0x22, 0x71, 0xdd, 0x8f, 0x22, 0x71, 0xdd, // gyro +/- per axis
    0x8f, 0x22, 0x71, 0xdd,
    0x1c, 0x02, 0x1c, 0x02,                         // gyro speed +/- 540 deg/s
    0x00, 0x20, 0x00, 0xe0, 0x00, 0x20, 0x00, 0xe0, // accel +/- 1 g per axis
    0x00, 0x20, 0x00, 0xe0,
    0x0b, 0x00,
};

// Feature 0x20: firmware info. Build date/time, then versions; SDL reads the
// update version at [44] (>= 0x0224 means "new" rumble emulation).
static void feature_20(uint8_t *r)
{
    memset(r, 0, 63);
    memcpy(&r[0], "Jun 19 2023", 11);
    memcpy(&r[11], "14:47:34", 8);
    r[19] = 0x03; r[20] = 0x00;                         // firmware type
    r[21] = 0x44; r[22] = 0x00; r[23] = 0x08; r[24] = 0x00;  // software series
    r[25] = 0x13; r[26] = 0x06; r[27] = 0x00; r[28] = 0x01;  // hardware info
    r[29] = 0x2a; r[30] = 0x00; r[31] = 0x10; r[32] = 0x01;  // firmware version
    r[43] = 0x30; r[44] = 0x04;                         // update version 0x0430
}

static uint8_t s_report[64];
static uint8_t s_counter;
static uint32_t s_seq;
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

static void put32(uint8_t *d, uint32_t v)
{
    d[0] = (uint8_t)v; d[1] = (uint8_t)(v >> 8); d[2] = (uint8_t)(v >> 16); d[3] = (uint8_t)(v >> 24);
}

static bool send(const pad_out_t *o)
{
    if (!tud_hid_ready()) {
        return false;
    }
    uint32_t b = o->buttons;
    uint8_t r[64];
    memset(r, 0, sizeof(r));
    uint8_t *p = &r[1];  // the state packet (SDL's PS5StatePacket_t offsets)
    r[0] = 0x01;
    p[0] = axis8(o->lx);
    p[1] = axis8(-o->ly);  // +Y is down
    p[2] = axis8(o->rx);
    p[3] = axis8(-o->ry);
    p[4] = (b & NS2_BTN_ZL) ? 255 : 0;
    p[5] = (b & NS2_BTN_ZR) ? 255 : 0;
    p[6] = s_counter++;
    // Positional: Square = west (Y), Cross = south (B), Circle = east (A), Triangle = north (X).
    p[7] = out_hat(b) & 0x0F;
    if (b & NS2_BTN_Y) p[7] |= 0x10;
    if (b & NS2_BTN_B) p[7] |= 0x20;
    if (b & NS2_BTN_A) p[7] |= 0x40;
    if (b & NS2_BTN_X) p[7] |= 0x80;
    if (b & NS2_BTN_L)      p[8] |= 0x01;
    if (b & NS2_BTN_R)      p[8] |= 0x02;
    if (b & NS2_BTN_ZL)     p[8] |= 0x04;
    if (b & NS2_BTN_ZR)     p[8] |= 0x08;
    if (b & NS2_BTN_MINUS)  p[8] |= 0x10;  // Create
    if (b & NS2_BTN_PLUS)   p[8] |= 0x20;  // Options
    if (b & NS2_BTN_LSTICK) p[8] |= 0x40;
    if (b & NS2_BTN_RSTICK) p[8] |= 0x80;
    if (b & NS2_BTN_HOME)   p[9] |= 0x01;  // PS
    if (b & NS2_BTN_CAPT)   p[9] |= 0x02;  // touchpad click
    if (b & NS2_BTN_C)      p[9] |= 0x04;  // mute (GameChat)
    put32(&p[11], s_seq++);

    if (o->motion.valid) {
        float g[3], a[3];
        out_motion_flipped(&o->motion, settings_get().flip_ds4, g, a);
        for (int i = 0; i < 3; i++) {
            put16(&p[15 + 2 * i], g[i] * 16.384f);
            put16(&p[21 + 2 * i], a[i] * 8192.0f);
        }
    } else {
        put16(&p[23], 8192.0f);  // at rest, lying flat
    }
    int64_t now = esp_timer_get_time();
    put32(&p[27], (uint32_t)((now - s_ts_base) * 3));  // sensor timestamp, 1/3 us units
    p[31] = 0x1F;                                     // temperature
    // Touchpad (1920 x 1080): Joy-Con mouse sensors and GL / GR half-clicks.
    if (out_touchpad(o, 1920, 1080, &p[32])) {
        p[9] |= 0x02;  // touchpad click
    }
    put32(&p[48], (uint32_t)((now - s_ts_base) * 3));
    p[52] = 0x2A;  // battery: full (0x2_), level 10
    p[53] = 0x08;  // connected over USB

    memcpy(s_report, r, sizeof(r));
    return tud_hid_report(0, s_report, sizeof(s_report));
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
    switch (id) {
    case 0x05: return copy_out(buf, len, feature_05, sizeof(feature_05));
    case 0x09: {  // pairing info: device MAC (little-endian), then host MAC
        uint8_t mac[6];
        out_get_mac(mac);
        uint8_t r[19] = { mac[5], mac[4], mac[3], mac[2], mac[1], mac[0], 0x08, 0x25, 0x00 };
        return copy_out(buf, len, r, sizeof(r));
    }
    case 0x20: {
        uint8_t r[63];
        feature_20(r);
        return copy_out(buf, len, r, sizeof(r));
    }
    default: {  // everything else in the descriptor: zeros of the right size
        static const struct { uint8_t id, len; } sizes[] = {
            { 0x08, 47 }, { 0x0A, 26 }, { 0x21, 4 }, { 0x22, 63 }, { 0x80, 63 }, { 0x81, 63 },
            { 0x82, 9 }, { 0x83, 63 }, { 0x84, 63 }, { 0x85, 2 }, { 0xA0, 1 }, { 0xE0, 63 },
            { 0xF0, 63 }, { 0xF1, 63 }, { 0xF2, 15 }, { 0xF4, 63 }, { 0xF5, 3 },
        };
        for (size_t i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
            if (sizes[i].id == id) {
                uint16_t n = sizes[i].len < len ? sizes[i].len : len;
                memset(buf, 0, n);
                return n;
            }
        }
        return 0;
    }
    }
}

static const char *TAG = "ds5";
static int s_out_logs;  // a few unusual output reports are logged per session

// Output report 0x02 (effects). [0] enable bits 1: 0x01 = rumble emulation,
// 0x02 = motors instead of audio haptics; [2] right (weak) motor, [3] left
// (strong) motor.
static void set_report(uint8_t id, hid_report_type_t type, const uint8_t *buf, uint16_t len)
{
    if (type != HID_REPORT_TYPE_OUTPUT && type != HID_REPORT_TYPE_INVALID) {
        return;
    }
    if (id == 0 && len > 0) {  // interrupt OUT: report ID is the first byte
        id = buf[0];
        buf++;
        len--;
    }
    // [38] enable bits 3: 0x04 = "improved rumble emulation" (firmware 2.24+).
    if (id == 0x02 && len >= 4 && ((buf[0] & 0x03) || (len > 38 && (buf[38] & 0x04)))) {
        rumble_set_motors(buf[3], buf[2]);
    } else if (id == 0x02 && len >= 4) {
        if (s_out_logs++ < 4) ESP_LOGI(TAG, "effects report without rumble flags (%02x %02x)", buf[0], buf[1]);
    } else if (id == 0x05 && len >= 5 && (buf[0] & 0x01)) {
        // A host that still takes this for a DualShock 4 sends the DS4 rumble
        // report; play it too ([4] strong, [3] weak).
        rumble_set_motors(buf[4], buf[3]);
    } else if (s_out_logs++ < 4) {
        ESP_LOGI(TAG, "output report %02x (%u bytes) ignored", id, len);
    }
}

static void started(void)
{
    s_ts_base = esp_timer_get_time();
}

static void reset(void)
{
    s_counter = 0;
    s_seq = 0;
    s_out_logs = 0;
    out_touchpad_reset();
}

const out_driver_t out_ds5 = {
    .name = "DualSense",
    .vid = 0x054C, .pid = 0x0CE6, .bcd_device = 0x0100,
    .manufacturer = "Sony Interactive Entertainment",
    .product = "DualSense Wireless Controller",
    .itf_desc = itf_desc, .itf_desc_len = sizeof(itf_desc), .itf_count = 1,
    .hid_report_desc = report_desc,
    .send = send,
    .reset = reset,
    .started = started,
    .period_ms = 4,
    .get_report = get_report,
    .set_report = set_report,
};
