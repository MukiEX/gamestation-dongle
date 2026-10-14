// Generic HID report-map parser and gamepad decoder (see hid_gamepad.h).
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include "hid_gamepad.h"

#include <stdio.h>
#include <string.h>

#define PAGE_DESKTOP 0x01
#define PAGE_SIM     0x02
#define PAGE_BUTTON  0x09
#define PAGE_CONSUMER 0x0C

#define MAX_USAGES 32

bool hid_parse(const uint8_t *map, size_t len, hid_layout_t *out)
{
    memset(out, 0, sizeof(*out));
    // Global state (with a small push/pop stack).
    struct { uint16_t page; int32_t lmin, lmax; uint8_t size, count, id; } g = { 0 }, stack[4];
    int sp = 0;
    // Local state.
    uint32_t usages[MAX_USAGES];
    int n_usages = 0;
    uint32_t umin = 0, umax = 0;
    bool have_range = false;
    // Bit offset per report ID.
    uint16_t offs[256];
    memset(offs, 0, sizeof(offs));

    size_t i = 0;
    while (i < len) {
        uint8_t p = map[i++];
        if (p == 0xFE) {  // long item: skip
            if (i + 1 >= len) break;
            i += 2 + map[i];
            continue;
        }
        int sz = p & 3;
        if (sz == 3) sz = 4;
        if (i + sz > len) break;
        uint32_t u = 0;
        for (int k = 0; k < sz; k++) u |= (uint32_t)map[i + k] << (8 * k);
        int32_t s = (int32_t)u;
        if (sz == 1) s = (int8_t)u;
        else if (sz == 2) s = (int16_t)u;
        i += sz;
        int type = (p >> 2) & 3, tag = p >> 4;

        if (type == 1) {  // global
            switch (tag) {
            case 0: g.page = (uint16_t)u; break;
            case 1: g.lmin = s; break;
            case 2: g.lmax = s; break;
            case 7: g.size = (uint8_t)u; break;
            case 8: g.id = (uint8_t)u; break;
            case 9: g.count = (uint8_t)u; break;
            case 10: if (sp < 4) stack[sp++] = g; break;
            case 11: if (sp > 0) g = stack[--sp]; break;
            default: break;
            }
        } else if (type == 2) {  // local
            switch (tag) {
            case 0: if (n_usages < MAX_USAGES) usages[n_usages++] = sz == 4 ? u : ((uint32_t)g.page << 16) | u; break;
            case 1: umin = sz == 4 ? u : ((uint32_t)g.page << 16) | u; have_range = true; break;
            case 2: umax = sz == 4 ? u : ((uint32_t)g.page << 16) | u; have_range = true; break;
            default: break;
            }
        } else if (type == 0) {  // main
            if (tag == 8) {  // Input
                bool constant = u & 0x01, variable = u & 0x02;
                uint16_t *off = &offs[g.id];
                if (constant) {
                    *off += g.size * g.count;
                } else if (!variable) {
                    // Array: each slot holds the index of a pressed usage.
                    uint32_t first = have_range ? umin : (n_usages ? usages[0] : 0);
                    uint32_t last = have_range ? umax : (n_usages ? usages[n_usages - 1] : 0);
                    for (int k = 0; k < g.count; k++) {
                        if (out->n >= HID_MAX_FIELDS) { out->truncated = true; break; }
                        hid_field_t *f = &out->f[out->n++];
                        *f = (hid_field_t){ g.id, *off, g.size, (uint16_t)(first >> 16), (uint16_t)first,
                                            (uint16_t)last, g.lmin, g.lmax, true };
                        *off += g.size;
                    }
                } else {
                    for (int k = 0; k < g.count; k++) {
                        uint32_t us;
                        if (have_range) us = umin + k <= umax ? umin + k : umax;
                        else if (n_usages) us = usages[k < n_usages ? k : n_usages - 1];
                        else us = 0;
                        if (out->n >= HID_MAX_FIELDS) { out->truncated = true; *off += g.size; continue; }
                        hid_field_t *f = &out->f[out->n++];
                        *f = (hid_field_t){ g.id, *off, g.size, (uint16_t)(us >> 16), (uint16_t)us, 0,
                                            g.lmin, g.lmax, false };
                        *off += g.size;
                    }
                }
            }
            // Every main item (Input / Output / Feature / Collection) clears locals.
            n_usages = 0;
            have_range = false;
            umin = umax = 0;
        }
    }
    return out->n > 0;
}

static uint32_t get_bits(const uint8_t *d, size_t len, uint16_t off, uint8_t size)
{
    uint32_t v = 0;
    for (int b = 0; b < size && b < 32; b++) {
        size_t bit = off + b;
        if (bit / 8 >= len) break;
        if (d[bit / 8] & (1u << (bit % 8))) v |= 1u << b;
    }
    return v;
}

static float norm(int32_t v, int32_t lmin, int32_t lmax)
{
    if (lmax <= lmin) return 0.0f;
    float t = (float)(v - lmin) / (float)(lmax - lmin);
    return t * 2.0f - 1.0f;
}

void hid_state_init(const hid_layout_t *lay, hid_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->hat = -1;
    for (int i = 0; i < lay->n; i++) {
        const hid_field_t *f = &lay->f[i];
        if (f->page == PAGE_DESKTOP) {
            switch (f->usage) {
            case 0x30: st->has[HID_AX_X] = true; break;
            case 0x31: st->has[HID_AX_Y] = true; break;
            case 0x32: st->has[HID_AX_Z] = true; break;
            case 0x35: st->has[HID_AX_RZ] = true; break;
            case 0x33: st->has[HID_AX_RX] = true; break;
            case 0x34: st->has[HID_AX_RY] = true; break;
            default: break;
            }
        } else if (f->page == PAGE_SIM) {
            if (f->usage == 0xC5) st->has[HID_AX_BRAKE] = true;
            if (f->usage == 0xC4) st->has[HID_AX_ACCEL] = true;
        }
    }
}

bool hid_decode(const hid_layout_t *lay, uint8_t report_id, const uint8_t *d, size_t len, hid_state_t *st)
{
    bool any = false;
    bool cleared_arrays = false;
    for (int i = 0; i < lay->n; i++) {
        const hid_field_t *f = &lay->f[i];
        if (f->report_id != report_id) continue;
        any = true;
        uint32_t raw = get_bits(d, len, f->bit_off, f->size);
        int32_t v = (int32_t)raw;
        if (f->lmin < 0 && f->size < 32 && (raw & (1u << (f->size - 1)))) {
            v = (int32_t)(raw | (~0u << f->size));  // sign-extend
        }
        if (f->array) {
            if (!cleared_arrays) {  // array slots list what is pressed now
                cleared_arrays = true;
                if (f->page == PAGE_BUTTON) st->buttons = 0;
                if (f->page == PAGE_CONSUMER) st->home = st->back = st->menu = false;
            }
            if (v < f->lmin || v > f->lmax || raw == 0) continue;
            uint32_t us = f->usage + (uint32_t)(v - f->lmin);
            if (f->page == PAGE_BUTTON && us >= 1 && us <= 32) st->buttons |= 1u << (us - 1);
            if (f->page == PAGE_CONSUMER) {
                if (us == 0x223) st->home = true;
                if (us == 0x224) st->back = true;
                if (us == 0x40) st->menu = true;
            }
            continue;
        }
        switch (f->page) {
        case PAGE_BUTTON:
            if (f->usage >= 1 && f->usage <= 32) {
                if (raw) st->buttons |= 1u << (f->usage - 1);
                else st->buttons &= ~(1u << (f->usage - 1));
            }
            break;
        case PAGE_DESKTOP:
            switch (f->usage) {
            case 0x30: st->x = norm(v, f->lmin, f->lmax); break;
            case 0x31: st->y = norm(v, f->lmin, f->lmax); break;
            case 0x32: st->z = norm(v, f->lmin, f->lmax); break;
            case 0x35: st->rz = norm(v, f->lmin, f->lmax); break;
            case 0x33: st->rx = norm(v, f->lmin, f->lmax); break;
            case 0x34: st->ry = norm(v, f->lmin, f->lmax); break;
            case 0x39:  // hat: lmin..lmax = up, up-right, ... ; outside = centred
                if (v < f->lmin || v > f->lmax) st->hat = -1;
                else st->hat = (int8_t)((v - f->lmin) * 8 / (f->lmax - f->lmin + 1));
                break;
            case 0x90: case 0x91: case 0x92: case 0x93: break;  // d-pad as usages: rare
            default: break;
            }
            break;
        case PAGE_SIM:
            if (f->usage == 0xC5) st->brake = (norm(v, f->lmin, f->lmax) + 1.0f) / 2.0f;
            if (f->usage == 0xC4) st->accel = (norm(v, f->lmin, f->lmax) + 1.0f) / 2.0f;
            break;
        case PAGE_CONSUMER:
            if (f->usage == 0x223) st->home = raw != 0;
            if (f->usage == 0x224) st->back = raw != 0;
            if (f->usage == 0x40) st->menu = raw != 0;
            break;
        default:
            break;
        }
    }
    return any;
}

const char *hid_usage_name(uint16_t page, uint16_t usage, char *buf, size_t n)
{
    static const char *const desk[] = { "X", "Y", "Z", "Rx", "Ry", "Rz", "Slider", "Dial", "Wheel", "Hat" };
    if (page == PAGE_BUTTON) snprintf(buf, n, "Button %u", usage);
    else if (page == PAGE_DESKTOP && usage >= 0x30 && usage <= 0x39) snprintf(buf, n, "%s", desk[usage - 0x30]);
    else if (page == PAGE_SIM && usage == 0xC4) snprintf(buf, n, "Accelerator");
    else if (page == PAGE_SIM && usage == 0xC5) snprintf(buf, n, "Brake");
    else if (page == PAGE_CONSUMER) snprintf(buf, n, "Consumer 0x%03x", usage);
    else snprintf(buf, n, "page 0x%02x usage 0x%02x", page, usage);
    return buf;
}
