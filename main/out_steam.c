// Steam Controller (2026) output, wired (Valve 28DE:1302), with the Joy-Con 2
// optical sensors as its two trackpads.
//
// Sources:
// * HID report descriptor, USB layout and the command-channel answers Steam
//   checks: HIDMaestro's steam-controller-2 profile (MIT), itself from a real
//   controller and an emulation a live Steam client accepts.
// * Input report 0x42 layout, button bits, IMU scale, rumble: SDL's
//   SDL_hidapi_steam_triton.c and steam/controller_structs.h (Valve, zlib).
// * Haptic output reports 0x81..0x85 (sides, gain in dB): sc2-research's
//   HAPTICS.md (MIT), after iczero/steam-controller-stuff.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "ns2_proto.h"
#include "out.h"
#include "rumble.h"
#include "settings.h"

static const char *TAG = "steam";

#define EP_IN  0x81
#define EP_OUT 0x01

static const uint8_t report_desc[] = {
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x40, 0x09, 0x01, 0xA1, 0x00, 0x05, 0x09, 0x19, 0x01,
    0x29, 0x02, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x02, 0x81, 0x02, 0x75, 0x06, 0x95, 0x01,
    0x81, 0x01, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x02,
    0x81, 0x06, 0x95, 0x01, 0x09, 0x38, 0x81, 0x06, 0x05, 0x0C, 0x0A, 0x38, 0x02, 0x95, 0x01, 0x81,
    0x06, 0xC0, 0xC0, 0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85, 0x41, 0x05, 0x07, 0x19, 0xE0, 0x29,
    0xE7, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x81, 0x01, 0x19, 0x00, 0x29,
    0x65, 0x15, 0x00, 0x25, 0x65, 0x75, 0x08, 0x95, 0x06, 0x81, 0x00, 0xC0, 0x06, 0x00, 0xFF, 0x09,
    0x01, 0xA1, 0x01, 0x85, 0x42, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x35, 0x09, 0x42,
    0x81, 0x02, 0x85, 0x44, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x05, 0x09, 0x44, 0x81,
    0x02, 0x85, 0x79, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x01, 0x09, 0x79, 0x81, 0x02,
    0x85, 0x43, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x0E, 0x09, 0x43, 0x81, 0x02, 0x85,
    0x7B, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x0C, 0x09, 0x7B, 0x81, 0x02, 0x85, 0x45,
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x2D, 0x09, 0x45, 0x81, 0x02, 0x85, 0x80, 0x15,
    0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x09, 0x09, 0x80, 0x91, 0x02, 0x85, 0x81, 0x15, 0x00,
    0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x07, 0x09, 0x81, 0x91, 0x02, 0x85, 0x82, 0x15, 0x00, 0x26,
    0xFF, 0x00, 0x75, 0x08, 0x95, 0x03, 0x09, 0x82, 0x91, 0x02, 0x85, 0x83, 0x15, 0x00, 0x26, 0xFF,
    0x00, 0x75, 0x08, 0x95, 0x09, 0x09, 0x83, 0x91, 0x02, 0x85, 0x84, 0x15, 0x00, 0x26, 0xFF, 0x00,
    0x75, 0x08, 0x95, 0x08, 0x09, 0x84, 0x91, 0x02, 0x85, 0x85, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
    0x08, 0x95, 0x03, 0x09, 0x85, 0x91, 0x02, 0x85, 0x86, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08,
    0x95, 0x03, 0x09, 0x86, 0x91, 0x02, 0x85, 0x87, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95,
    0x3F, 0x09, 0x87, 0x91, 0x02, 0x85, 0x89, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x3F,
    0x09, 0x89, 0x91, 0x02, 0x85, 0x88, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x3F, 0x09,
    0x88, 0x91, 0x02, 0x85, 0x01, 0x95, 0x3F, 0x09, 0x01, 0xB1, 0x02, 0x85, 0x02, 0x95, 0x3F, 0x09,
    0x01, 0xB1, 0x02, 0xC0,
};

static const uint8_t itf_desc[] = {
    TUD_HID_INOUT_DESCRIPTOR(0, 4, HID_ITF_PROTOCOL_NONE, sizeof(report_desc), EP_OUT, EP_IN, 64, 1),
};

// ---- Buttons (SDL's TritonButtons) -----------------------------------------
#define T_A            0x00000001u  // south
#define T_B            0x00000002u  // east
#define T_X            0x00000004u  // west
#define T_Y            0x00000008u  // north
#define T_QAM          0x00000010u
#define T_R3           0x00000020u
#define T_VIEW         0x00000040u  // SDL: Start
#define T_R4           0x00000080u
#define T_R5           0x00000100u
#define T_R            0x00000200u
#define T_DOWN         0x00000400u
#define T_RIGHT        0x00000800u
#define T_LEFT         0x00001000u
#define T_UP           0x00002000u
#define T_MENU         0x00004000u  // SDL: Back
#define T_L3           0x00008000u
#define T_STEAM        0x00010000u
#define T_L4           0x00020000u
#define T_L5           0x00040000u
#define T_L            0x00080000u
#define T_RSTICK_TOUCH 0x00100000u
#define T_RPAD_TOUCH   0x00200000u
#define T_RPAD_CLICK   0x00400000u
#define T_RTRIG_CLICK  0x00800000u
#define T_LSTICK_TOUCH 0x01000000u
#define T_LPAD_TOUCH   0x02000000u
#define T_LPAD_CLICK   0x04000000u
#define T_LTRIG_CLICK  0x08000000u
#define T_RGRIP_TOUCH  0x10000000u
#define T_LGRIP_TOUCH  0x20000000u

// ---- Command channel (feature reports 1 / 2) --------------------------------
// The host writes [cmd][len][payload] and then reads the answer back.

// ID_GET_ATTRIBUTES_VALUES: a real controller's blob, as (tag, u32) records:
// product 0x1302, capabilities, bootloader build, firmware build
// (0x6A18D057, 2026-05-28), board revision. Shared by all units.
static const uint8_t ATTR_83[25] = {
    0x01, 0x02, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x2E, 0xF9,
    0xD2, 0x68, 0x04, 0x57, 0xD0, 0x18, 0x6A, 0x09, 0x48, 0x00, 0x00, 0x00,
};

static uint8_t s_resp[63];
static portMUX_TYPE s_resp_mux = portMUX_INITIALIZER_UNLOCKED;
static char s_unit_serial[16], s_board_serial[16];

static void make_serials(void)
{
    uint8_t mac[6];
    out_get_mac(mac);
    // Valve's format (3 letters + 11 characters); unique per dongle.
    snprintf(s_unit_serial, sizeof(s_unit_serial), "FXA9%02X%02X%02X%02X%02X", mac[1], mac[2], mac[3], mac[4], mac[5]);
    snprintf(s_board_serial, sizeof(s_board_serial), "MXA9%02X%02X%02X%02X%02X", mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void command(const uint8_t *b, uint16_t n)
{
    if (n < 1) {
        return;
    }
    uint8_t cmd = b[0], len = n > 1 ? b[1] : 0;
    const uint8_t *pl = b + 2;
    uint16_t pln = n > 2 ? n - 2 : 0;
    uint8_t r[63] = { 0 };
    r[0] = cmd;
    switch (cmd) {
    case 0x83:  // ID_GET_ATTRIBUTES_VALUES
        r[1] = sizeof(ATTR_83);
        memcpy(&r[2], ATTR_83, sizeof(ATTR_83));
        break;
    case 0xAE: {  // ID_GET_STRING_ATTRIBUTE: [0xAE][0x14][index][string]
        uint8_t idx = pln > 0 ? pl[0] : 0;
        const char *s = idx == 0 ? s_board_serial : idx == 1 ? s_unit_serial : idx == 3 ? "7054257d2da7" : NULL;
        r[1] = 0x14;
        r[2] = s ? idx : 0xFF;  // 0xFF: "not provisioned", as a real unit answers
        if (s) memcpy(&r[3], s, strlen(s));
        break;
    }
    case 0xB4:  // wireless transport state: wired, no radio link
        r[1] = 0x01;
        r[2] = 0x01;
        break;
    default:  // everything else (settings writes, ...) reads back as written
        r[1] = len;
        memcpy(&r[2], pl, pln > 61 ? 61 : pln);
        break;
    }
    portENTER_CRITICAL(&s_resp_mux);
    memcpy(s_resp, r, sizeof(r));
    portEXIT_CRITICAL(&s_resp_mux);
    if (cmd != 0x87) {  // settings writes repeat every few seconds
        ESP_LOGI(TAG, "host command %02x (%u bytes)", cmd, n);
    }
}

// ---- Haptics -> Joy-Con rumble ----------------------------------------------
static int64_t s_rumble_until;  // motor rumble (0x80) expires unless refreshed

static float db_to_amp(int8_t db)
{
    if (db < -23) db = -23;
    if (db > 24) db = 24;
    float a = powf(10.0f, db / 20.0f) * 0.5f;  // 0 dB = half strength
    return a > 1.0f ? 1.0f : a;
}

// Sides: 0x81 uses R, L, both; the others L, R, both; 3..5 are the internal
// motors (same mapping here), 6 / 7 act as 2 / 4.
static uint8_t sides_of(uint8_t id, uint8_t side)
{
    if (side == 6) side = 2;
    if (side == 7) side = 4;
    if (side >= 3) side -= 3;
    if (id == 0x81 && side < 2) side ^= 1;
    return side == 0 ? 1 : side == 1 ? 2 : 3;
}

static uint16_t u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static void haptic(uint8_t id, const uint8_t *p, uint16_t n)
{
    switch (id) {
    case 0x80:  // rumble: type, intensity, left speed/gain, right speed/gain
        if (n >= 9) {
            uint16_t left = u16(&p[3]), right = u16(&p[6]);
            rumble_set_motors((uint8_t)(left >> 8), (uint8_t)(right >> 8));
            s_rumble_until = (left || right) ? esp_timer_get_time() + 150000 : 0;
        }
        break;
    case 0x81:  // pulse: side, on us, off us, repeat
        if (n >= 7) {
            uint32_t on = u16(&p[1]), off = u16(&p[3]), rep = u16(&p[5]);
            uint32_t ms = (uint32_t)(((uint64_t)(on + off) * (rep + 1)) / 1000);
            if (ms < 10) ms = 10;
            if (ms > 400) ms = 400;
            rumble_tone(sides_of(id, p[0]), 170.0f, 0.6f, ms);
        }
        break;
    case 0x82:  // command: side, 0 stop / 1 click / 2 strong click, gain
        if (n >= 3) {
            if (p[1] == 0) rumble_stop();
            else rumble_tone(sides_of(id, p[0]), 320.0f, db_to_amp((int8_t)p[2]) * (p[1] == 2 ? 1.6f : 1.0f), 25);
        }
        break;
    case 0x83:  // LFO tone: side, gain, frequency, duration
        if (n >= 6) {
            uint32_t ms = u16(&p[4]);
            rumble_tone(sides_of(id, p[0]), u16(&p[2]), db_to_amp((int8_t)p[1]), ms > 2000 ? 2000 : ms);
        }
        break;
    case 0x84:  // log sweep: side, gain, duration, start, end
        if (n >= 8) {
            uint32_t ms = u16(&p[2]);
            float hz = sqrtf((float)u16(&p[4]) * (float)u16(&p[6]));
            rumble_tone(sides_of(id, p[0]), hz, db_to_amp((int8_t)p[1]), ms > 2000 ? 2000 : ms);
        }
        break;
    case 0x85:  // script (identify "ping", power on/off sounds...): a short buzz
        if (n >= 3) {
            rumble_tone(sides_of(id, p[0]), 250.0f, db_to_amp((int8_t)p[2]), p[1] == 0x0c ? 300 : 120);
        }
        break;
    default:  // 0x86..0x89 audio streams: not supported
        break;
    }
}

// ---- Reports -----------------------------------------------------------------
static uint8_t s_report[54];
static uint8_t s_seq;
static int64_t s_ts_base, s_last_battery;

static void put16(uint8_t *d, int32_t v)
{
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    d[0] = (uint8_t)v;
    d[1] = (uint8_t)((uint16_t)v >> 8);
}

static int16_t axis(float v)
{
    return (int16_t)lrintf(out_clampf(v, -1.0f, 1.0f) * 32767.0f);
}

static bool send(const pad_out_t *o)
{
    if (!tud_hid_ready()) {
        return false;
    }
    int64_t now = esp_timer_get_time();
    if (s_rumble_until && now > s_rumble_until) {  // the host stopped refreshing
        s_rumble_until = 0;
        rumble_stop();
    }
    // Battery status every ~2.5 s, as the real controller sends it.
    if (now - s_last_battery > 2500000) {
        s_last_battery = now;
        uint8_t bat[15] = { 0x43, 0x04 /* charging done */, 100 };
        put16(&bat[3], 4200);  // battery mV
        put16(&bat[5], 3300);  // system mV
        put16(&bat[7], 5000);  // input mV
        return tud_hid_report(0, bat, sizeof(bat));
    }

    uint32_t b = o->buttons, t = 0;
    // Positional: A = south (Switch B), B = east (A), X = west (Y), Y = north (X).
    if (b & NS2_BTN_B)      t |= T_A;
    if (b & NS2_BTN_A)      t |= T_B;
    if (b & NS2_BTN_Y)      t |= T_X;
    if (b & NS2_BTN_X)      t |= T_Y;
    if (b & NS2_BTN_L)      t |= T_L;
    if (b & NS2_BTN_R)      t |= T_R;
    if (b & NS2_BTN_ZL)     t |= T_LTRIG_CLICK;
    if (b & NS2_BTN_ZR)     t |= T_RTRIG_CLICK;
    if (b & NS2_BTN_MINUS)  t |= T_MENU;   // SDL Back
    if (b & NS2_BTN_PLUS)   t |= T_VIEW;   // SDL Start
    if (b & NS2_BTN_LSTICK) t |= T_L3;
    if (b & NS2_BTN_RSTICK) t |= T_R3;
    if (b & NS2_BTN_HOME)   t |= T_STEAM;
    if (b & NS2_BTN_CAPT)   t |= T_QAM;
    if (b & NS2_BTN_UP)     t |= T_UP;
    if (b & NS2_BTN_DOWN)   t |= T_DOWN;
    if (b & NS2_BTN_LEFT)   t |= T_LEFT;
    if (b & NS2_BTN_RIGHT)  t |= T_RIGHT;
    if (b & NS2_BTN_GL)     t |= T_L4;     // back buttons (Auto)
    if (b & NS2_BTN_GR)     t |= T_R4;
    if (b & (NS2_BTN_SL_L | NS2_BTN_SR_L)) t |= T_L5;
    if (b & (NS2_BTN_SL_R | NS2_BTN_SR_R)) t |= T_R5;
    if (b & NS2_BTN_TP_LEFT)  t |= T_LPAD_CLICK;
    if (b & NS2_BTN_TP_RIGHT) t |= T_RPAD_CLICK;
    // No capacitive sensing on Joy-Cons: a moved stick counts as touched, and
    // the grips as held whenever a controller is connected.
    if (fabsf(o->lx) > 0.1f || fabsf(o->ly) > 0.1f) t |= T_LSTICK_TOUCH;
    if (fabsf(o->rx) > 0.1f || fabsf(o->ry) > 0.1f) t |= T_RSTICK_TOUCH;
    if (o->mode != PAD_MODE_NONE) t |= T_LGRIP_TOUCH | T_RGRIP_TOUCH;
    if (o->tp[0].touch) t |= T_LPAD_TOUCH;
    if (o->tp[1].touch) t |= T_RPAD_TOUCH;

    uint8_t *r = s_report;
    memset(r, 0, sizeof(s_report));
    r[0] = 0x42;
    r[1] = s_seq++;
    r[2] = (uint8_t)t; r[3] = (uint8_t)(t >> 8); r[4] = (uint8_t)(t >> 16); r[5] = (uint8_t)(t >> 24);
    put16(&r[6], (b & NS2_BTN_ZL) ? 32767 : 0);
    put16(&r[8], (b & NS2_BTN_ZR) ? 32767 : 0);
    put16(&r[10], axis(o->lx));
    put16(&r[12], axis(o->ly));  // +Y up
    put16(&r[14], axis(o->rx));
    put16(&r[16], axis(o->ry));
    for (int i = 0; i < 2; i++) {  // trackpads: x, y (+up), pressure
        uint8_t *p = &r[18 + 6 * i];
        if (o->tp[i].touch) {
            put16(&p[0], o->tp[i].x);
            put16(&p[2], o->tp[i].y);
        }
        bool click = (b & (i == 0 ? NS2_BTN_TP_LEFT : NS2_BTN_TP_RIGHT)) != 0;
        put16(&p[4], click ? 32767 : 0);
    }
    uint32_t ts = (uint32_t)(now - s_ts_base);  // IMU timestamp, microseconds
    r[30] = (uint8_t)ts; r[31] = (uint8_t)(ts >> 8); r[32] = (uint8_t)(ts >> 16); r[33] = (uint8_t)(ts >> 24);
    if (o->motion.valid) {
        // Raw IMU axes: X right, Y away from the player, Z up (SDL reads them
        // back as X, Z, -Y). Gyro +-2000 deg/s and accel +-2 g full scale.
        const float G = 32768.0f / 2000.0f, A = 32768.0f / 2.0f;
        const float *g = o->motion.gyro, *a = o->motion.accel;
        put16(&r[34], lrintf(a[0] * A));
        put16(&r[36], lrintf(-a[2] * A));
        put16(&r[38], lrintf(a[1] * A));
        put16(&r[40], lrintf(g[0] * G));
        put16(&r[42], lrintf(-g[2] * G));
        put16(&r[44], lrintf(g[1] * G));
    } else {
        put16(&r[38], 16384);  // at rest, lying flat
    }
    put16(&r[46], 32767);  // orientation quaternion: identity
    return tud_hid_report(0, r, sizeof(s_report));
}

static uint16_t get_report(uint8_t id, hid_report_type_t type, uint8_t *buf, uint16_t len)
{
    if (type != HID_REPORT_TYPE_FEATURE) {
        uint16_t n = len < sizeof(s_report) - 1 ? len : sizeof(s_report) - 1;
        memcpy(buf, s_report + 1, n);
        return n;
    }
    uint16_t n = len < sizeof(s_resp) ? len : sizeof(s_resp);
    portENTER_CRITICAL(&s_resp_mux);
    memcpy(buf, s_resp, n);
    portEXIT_CRITICAL(&s_resp_mux);
    return n;
}

static void set_report(uint8_t id, hid_report_type_t type, const uint8_t *buf, uint16_t len)
{
    if (id == 0 && len > 0) {  // interrupt OUT: the report ID is the first byte
        id = buf[0];
        buf++;
        len--;
    }
    if (type == HID_REPORT_TYPE_FEATURE || id == 0x01 || id == 0x02) {
        command(buf, len);
    } else if (id >= 0x80 && id <= 0x89) {
        haptic(id, buf, len);
    }
}

static void started(void)
{
    s_ts_base = esp_timer_get_time();
    make_serials();
    memset(s_resp, 0, sizeof(s_resp));
}

static void reset(void)
{
    s_seq = 0;
    s_rumble_until = 0;
    s_last_battery = 0;
}

const out_driver_t out_steam = {
    .name = "Steam Controller",
    .vid = 0x28DE, .pid = 0x1302, .bcd_device = 0x0100,
    .manufacturer = "Valve Software",
    .product = "Steam Controller",
    .itf_desc = itf_desc, .itf_desc_len = sizeof(itf_desc), .itf_count = 1,
    .hid_report_desc = report_desc,
    .send = send,
    .reset = reset,
    .started = started,
    .period_ms = 4,
    .get_report = get_report,
    .set_report = set_report,
};
