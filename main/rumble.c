// SPDX-License-Identifier: GPL-3.0-or-later
#include "rumble.h"

#include <math.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "settings.h"

// Joy-Con 2 frequency codes are 9-bit, logarithmic: 256 codes per octave with
// 0x0E1 = 160 Hz and 0x1E1 = 320 Hz (the controllers' default low/high bands).
#define JC_LF_DEFAULT 0x0E1
// Switch2Connect's Xbox-mode high band at frequency setting 10.
#define JC_HF_XBOX    347
#define HD_HOLD_US    150000

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static rumble_frame_t s_frame;
static bool s_active;
static int64_t s_expires;  // 0 = holds until changed

// Rumble strength setting: 5 = Switch2Connect's default, 0 = off, 10 = double.
static float strength(void)
{
    return settings_get().rumble_strength / 5.0f;
}

static uint16_t clamp_amp(float v)
{
    if (v < 0.0f) return 0;
    if (v > 1023.0f) return 1023;
    return (uint16_t)lrintf(v);
}

// Joy-Cons have one actuator; keep the total within its 10-bit range
// (Switch2Connect caps the sum the same way).
static void cap_total(ns2_vib_t *v)
{
    unsigned total = v->lf_amp + v->hf_amp;
    if (total > 1023) {
        v->lf_amp = (uint16_t)(v->lf_amp * 1023u / total);
        v->hf_amp = (uint16_t)(1023u - v->lf_amp);
    }
}

static void publish(const rumble_frame_t *f, bool active, int64_t expires)
{
    portENTER_CRITICAL(&s_mux);
    s_frame = *f;
    s_active = active;
    s_expires = expires;
    portEXIT_CRITICAL(&s_mux);
}

void rumble_init(void)
{
    rumble_stop();
}

void rumble_stop(void)
{
    rumble_frame_t f;
    f.jc[0] = f.jc[1] = f.pro[0] = f.pro[1] = NS2_VIB_SILENT;
    publish(&f, false, 0);
}

void rumble_set_motors(uint8_t strong, uint8_t weak)
{
    if (strong == 0 && weak == 0) {
        rumble_stop();
        return;
    }
    float k = strength();
    if (k <= 0.0f) {
        rumble_stop();
        return;
    }
    float s = strong / 255.0f * k, w = weak / 255.0f * k;
    // Switch2Connect, Xbox rumble mode, strength 5: source 800 * motor/256,
    // LF x0.84 (Joy-Con) / x2.0 (Pro), HF x1.0 with the HF mask 0.101 / 0.2.
    ns2_vib_t jc = { .lf_freq = JC_LF_DEFAULT, .lf_amp = clamp_amp(s * 800.0f * 0.84f),
                     .hf_freq = JC_HF_XBOX,    .hf_amp = clamp_amp(w * 800.0f * 0.101f) };
    ns2_vib_t pro = { .lf_freq = JC_LF_DEFAULT, .lf_amp = clamp_amp(s * 800.0f * 2.0f),
                      .hf_freq = JC_HF_XBOX,    .hf_amp = clamp_amp(w * 800.0f * 0.2f) };
    cap_total(&jc);
    rumble_frame_t f = { .jc = { jc, jc }, .pro = { pro, pro } };
    publish(&f, true, 0);
}

// Switch 1 HD rumble amplitude code (0-100) -> 0..1.
static float hd_amp(unsigned enc)
{
    if (enc == 0) return 0.0f;
    if (enc >= 32) return powf(2.0f, enc / 32.0f) / 8.7f;
    if (enc >= 16) return powf(2.0f, enc / 16.0f) / 17.0f;
    return (enc / 16.0f) * (2.0f / 17.0f);
}

static uint16_t hz_to_code(float hz)
{
    float c = JC_LF_DEFAULT + 256.0f * log2f(hz / 160.0f);
    if (c < 1.0f) c = 1.0f;
    if (c > 511.0f) c = 511.0f;
    return (uint16_t)lrintf(c);
}

// Decodes one 4-byte Switch HD rumble sample.
static void hd_decode(const uint8_t b[4], float *hf_hz, float *hf_a, float *lf_hz, float *lf_a)
{
    unsigned hf9 = b[0] | ((b[1] & 0x01u) << 8);
    *hf_hz = 10.0f * powf(2.0f, (hf9 / 4.0f + 0x60) / 32.0f);
    *hf_a = hd_amp(b[1] >> 1);
    unsigned lf7 = b[2] & 0x7Fu;
    *lf_hz = 10.0f * powf(2.0f, (lf7 + 0x40) / 32.0f);
    int lf_enc = (((int)b[3] - 0x40) << 1) | (b[2] >> 7);
    *lf_a = hd_amp(lf_enc < 0 ? 0 : (unsigned)lf_enc);
}

static void hd_to_frames(const uint8_t b[4], ns2_vib_t *jc, ns2_vib_t *pro, bool *any)
{
    float hf_hz, hf_a, lf_hz, lf_a;
    hd_decode(b, &hf_hz, &hf_a, &lf_hz, &lf_a);
    float k = strength();
    hf_a *= k;
    lf_a *= k;
    uint16_t lf_code = hz_to_code(lf_hz), hf_code = hz_to_code(hf_hz);
    *jc = (ns2_vib_t){ .lf_freq = lf_code, .lf_amp = clamp_amp(lf_a * 1023.0f * 0.6f),
                       .hf_freq = hf_code, .hf_amp = clamp_amp(hf_a * 1023.0f * 0.4f) };
    *pro = (ns2_vib_t){ .lf_freq = lf_code, .lf_amp = clamp_amp(lf_a * 1023.0f),
                        .hf_freq = hf_code, .hf_amp = clamp_amp(hf_a * 1023.0f * 0.6f) };
    cap_total(jc);
    if (jc->lf_amp || jc->hf_amp || pro->lf_amp || pro->hf_amp) {
        *any = true;
    }
}

void rumble_set_hd(const uint8_t left[4], const uint8_t right[4])
{
    rumble_frame_t f;
    bool any = false;
    hd_to_frames(left, &f.jc[0], &f.pro[0], &any);
    hd_to_frames(right, &f.jc[1], &f.pro[1], &any);
    if (!any) {
        rumble_stop();
        return;
    }
    publish(&f, true, esp_timer_get_time() + HD_HOLD_US);
}

bool rumble_get(rumble_frame_t *out)
{
    bool active;
    portENTER_CRITICAL(&s_mux);
    if (s_active && s_expires != 0 && esp_timer_get_time() > s_expires) {
        s_active = false;
    }
    *out = s_frame;
    active = s_active;
    portEXIT_CRITICAL(&s_mux);
    return active;
}
