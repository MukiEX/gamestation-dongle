// SPDX-License-Identifier: GPL-3.0-or-later
#include "ns2_proto.h"

#include <math.h>
#include <string.h>

static inline uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline int16_t  rd_s16(const uint8_t *p) { return (int16_t)rd_u16(p); }
static inline uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void rd_stick(const uint8_t *p, uint16_t *x, uint16_t *y)
{
    uint32_t v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
    *x = v & 0xFFF;
    *y = (v >> 12) & 0xFFF;
}

bool ns2_parse_input(const uint8_t *d, size_t len, ns2_input_t *o)
{
    if (len < 60) {
        return false;
    }
    o->timestamp = rd_u32(&d[0]);
    o->buttons = rd_u32(&d[4]) & NS2_BTN_MASK;
    rd_stick(&d[10], &o->ls_x, &o->ls_y);
    rd_stick(&d[13], &o->rs_x, &o->rs_y);
    o->mouse_x = rd_u16(&d[16]);
    o->mouse_y = rd_u16(&d[18]);
    o->mouse_roughness = rd_u16(&d[20]);
    o->mouse_distance = rd_u16(&d[22]);
    o->mag[0] = rd_s16(&d[25]);
    o->mag[1] = rd_s16(&d[27]);
    o->mag[2] = rd_s16(&d[29]);
    o->battery_mv = rd_u16(&d[31]);
    o->accel[0] = rd_s16(&d[48]);
    o->accel[1] = rd_s16(&d[50]);
    o->accel[2] = rd_s16(&d[52]);
    o->gyro[0] = rd_s16(&d[54]);
    o->gyro[1] = rd_s16(&d[56]);
    o->gyro[2] = rd_s16(&d[58]);
    return true;
}

void ns2_cal_decode(const uint8_t *d, size_t len, ns2_stick_cal_t *c)
{
    c->valid = false;
    c->center_x = c->center_y = 2048;
    c->max_x = c->max_y = c->min_x = c->min_y = 1500;
    if (d == NULL || len < 9) {
        return;
    }
    uint16_t cx, cy, nx, ny, mx, my;
    rd_stick(&d[0], &cx, &cy);
    rd_stick(&d[3], &nx, &ny);  // negative range
    rd_stick(&d[6], &mx, &my);  // positive range

    // Same sanity rules as Switch2Connect: garbage or blank calibration would pin
    // the stick to an extreme, so fall back to centred defaults instead.
    bool bad_center = (cx == 0 && cy == 0 && mx == 0 && my == 0) ||
                      (cx == 4095 && cy == 4095 && mx == 4095 && my == 4095) ||
                      cx < 1024 || cx > 3072 || cy < 1024 || cy > 3072;
    bool bad_range = mx == 0 || my == 0 || nx == 0 || ny == 0 ||
                     mx > 4095 - cx || my > 4095 - cy || nx > cx || ny > cy;
    if (bad_center || bad_range) {
        return;
    }
    c->center_x = cx;
    c->center_y = cy;
    c->max_x = mx;
    c->max_y = my;
    c->min_x = nx;
    c->min_y = ny;
    c->valid = true;
}

static float cal_axis(int raw, int center, int max_abs, int min_abs)
{
    int s = raw - center;
    float v;
    if (s >= 0) {
        v = (float)s / (float)(max_abs > 0 ? max_abs : 1);
        return v > 1.0f ? 1.0f : v;
    }
    v = (float)(-s) / (float)(min_abs > 0 ? min_abs : 1);
    return -(v > 1.0f ? 1.0f : v);
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void ns2_cal_apply(const ns2_stick_cal_t *c, uint16_t rx, uint16_t ry,
                   float gain, float deadzone, float *ox, float *oy)
{
    float x = clampf(cal_axis(rx, c->center_x, c->max_x, c->min_x) * gain, -1.0f, 1.0f);
    float y = clampf(cal_axis(ry, c->center_y, c->max_y, c->min_y) * gain, -1.0f, 1.0f);
    if (sqrtf(x * x + y * y) < deadzone) {
        x = 0.0f;
        y = 0.0f;
    }
    *ox = x;
    *oy = y;
}

void ns2_vib_encode(const ns2_vib_t *v, uint8_t out[5])
{
    uint64_t w = 0;
    w |= (uint64_t)(v->lf_freq & 0x1FF);
    w |= (uint64_t)(v->lf_tone ? 1 : 0) << 9;
    w |= (uint64_t)(v->lf_amp & 0x3FF) << 10;
    w |= (uint64_t)(v->hf_freq & 0x1FF) << 20;
    w |= (uint64_t)(v->hf_tone ? 1 : 0) << 29;
    w |= (uint64_t)(v->hf_amp & 0x3FF) << 30;
    for (int i = 0; i < 5; i++) {
        out[i] = (uint8_t)(w >> (8 * i));
    }
}

size_t ns2_build_rumble(uint8_t *out, uint8_t packet_id, bool is_pro, const ns2_vib_t frames[3])
{
    size_t n = 0;
    out[n++] = 0x00;
    int motors = is_pro ? 2 : 1;  // Pro Controller: same waveform on both motors
    for (int m = 0; m < motors; m++) {
        out[n++] = (uint8_t)(0x50 | (packet_id & 0x0F));
        for (int f = 0; f < 3; f++) {
            ns2_vib_encode(&frames[f], &out[n]);
            n += 5;
        }
    }
    return n;
}
