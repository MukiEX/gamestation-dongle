// XInput (Xbox 360 wired controller) output. XInput is not HID: it is a
// vendor-specific interface (class 0xFF, subclass 0x5D, protocol 0x01) that
// Windows' built-in xusb22 driver binds to, so it needs its own small TinyUSB
// class driver. Descriptors and report layout follow GP2040-CE (MIT).
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include <math.h>
#include <string.h>

#include "device/usbd_pvt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ns2_proto.h"
#include "out.h"
#include "rumble.h"

static const char *TAG = "xinput";

#define XINPUT_OUT_SIZE 32

// Buttons byte 1
#define XB_UP    0x01
#define XB_DOWN  0x02
#define XB_LEFT  0x04
#define XB_RIGHT 0x08
#define XB_START 0x10
#define XB_BACK  0x20
#define XB_LS    0x40
#define XB_RS    0x80
// Buttons byte 2
#define XB_LB    0x01
#define XB_RB    0x02
#define XB_GUIDE 0x04
#define XB_A     0x10
#define XB_B     0x20
#define XB_X     0x40
#define XB_Y     0x80

typedef struct __attribute__((packed)) {
    uint8_t report_id;    // 0
    uint8_t report_size;  // 20
    uint8_t buttons1;
    uint8_t buttons2;
    uint8_t lt, rt;
    int16_t lx, ly, rx, ry;  // +Y = up
    uint8_t reserved[6];
} xinput_report_t;

_Static_assert(sizeof(xinput_report_t) == 20, "XInput report is 20 bytes");

static const uint8_t itf_desc[] = {
    // Interface 0: control (vendor 0xFF/0x5D/0x01)
    0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x5D, 0x01, 0x00,
    // Gamepad descriptor (type 0x21), as on a wired Xbox 360 controller
    0x11, 0x21, 0x00, 0x01, 0x01, 0x25, 0x81, 0x14, 0x00, 0x00, 0x00, 0x00, 0x13, 0x01, 0x08, 0x00, 0x00,
    // IN endpoint 0x81, interrupt, 32 bytes, every 1 ms
    0x07, 0x05, 0x81, 0x03, 0x20, 0x00, 0x01,
    // OUT endpoint 0x01, interrupt, 32 bytes, every 8 ms
    0x07, 0x05, 0x01, 0x03, 0x20, 0x00, 0x08,
};

static uint8_t s_ep_in, s_ep_out;
static uint8_t s_out_buf[XINPUT_OUT_SIZE];
static xinput_report_t s_report;   // must stay valid while a transfer runs
static xinput_report_t s_last;
static bool s_have_last;
static int64_t s_last_sent;

// ---- TinyUSB class driver ----------------------------------------------------

static void drv_init(void) {}
static bool drv_deinit(void) { return true; }

static void drv_reset(uint8_t rhport)
{
    (void)rhport;
    s_ep_in = s_ep_out = 0;
    s_have_last = false;
}

static uint16_t drv_open(uint8_t rhport, tusb_desc_interface_t const *itf, uint16_t max_len)
{
    if (itf->bInterfaceClass != TUSB_CLASS_VENDOR_SPECIFIC || itf->bInterfaceSubClass != 0x5D ||
        itf->bInterfaceProtocol != 0x01) {
        return 0;
    }
    uint16_t len = sizeof(tusb_desc_interface_t);
    uint8_t const *p = tu_desc_next(itf);
    TU_VERIFY(tu_desc_type(p) == 0x21, 0);
    len += tu_desc_len(p);
    p = tu_desc_next(p);
    len += itf->bNumEndpoints * sizeof(tusb_desc_endpoint_t);
    TU_VERIFY(max_len >= len, 0);
    TU_ASSERT(usbd_open_edpt_pair(rhport, p, itf->bNumEndpoints, TUSB_XFER_INTERRUPT, &s_ep_out, &s_ep_in), 0);
    if (s_ep_out) {
        usbd_edpt_xfer(rhport, s_ep_out, s_out_buf, sizeof(s_out_buf));
    }
    return len;
}

static bool drv_control(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req)
{
    (void)rhport;
    (void)stage;
    (void)req;
    return false;  // no class requests needed on a PC
}

static void handle_out(const uint8_t *d, uint32_t n)
{
    if (n >= 5 && d[0] == 0x00 && d[1] == 0x08) {
        rumble_set_motors(d[3], d[4]);  // [3] = left (strong), [4] = right (weak)
    } else if (n >= 3 && d[0] == 0x01 && d[1] == 0x03) {
        ESP_LOGD(TAG, "LED pattern %u", d[2]);
    }
}

static bool drv_xfer(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred)
{
    if (ep_addr == s_ep_out) {
        if (result == XFER_RESULT_SUCCESS) {
            handle_out(s_out_buf, xferred);
        }
        usbd_edpt_xfer(rhport, s_ep_out, s_out_buf, sizeof(s_out_buf));
    } else if (ep_addr == s_ep_in) {
        usb_gamepad_poke();  // a queued state change can go out now
    }
    return true;
}

static const usbd_class_driver_t s_driver = {
    .name = "XINPUT",
    .init = drv_init,
    .deinit = drv_deinit,
    .reset = drv_reset,
    .open = drv_open,
    .control_xfer_cb = drv_control,
    .xfer_cb = drv_xfer,
    .xfer_isr = NULL,
    .sof = NULL,
};

// TinyUSB asks the application for extra class drivers. The XInput driver only
// claims XInput interfaces, so it is harmless in the other modes.
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *count)
{
    *count = 1;
    return &s_driver;
}

// ---- Reports ------------------------------------------------------------------

static int16_t axis16(float v)
{
    return (int16_t)lrintf(out_clampf(v, -1.0f, 1.0f) * 32767.0f);
}

static bool send(const pad_out_t *o)
{
    if (s_ep_in == 0 || !tud_mounted()) {
        return true;
    }
    uint32_t b = o->buttons;
    xinput_report_t r = { .report_id = 0, .report_size = sizeof(xinput_report_t) };
    // Positional: Xbox A = south (Switch B), B = east (A), X = west (Y), Y = north (X).
    if (b & NS2_BTN_UP)     r.buttons1 |= XB_UP;
    if (b & NS2_BTN_DOWN)   r.buttons1 |= XB_DOWN;
    if (b & NS2_BTN_LEFT)   r.buttons1 |= XB_LEFT;
    if (b & NS2_BTN_RIGHT)  r.buttons1 |= XB_RIGHT;
    if (b & NS2_BTN_PLUS)   r.buttons1 |= XB_START;
    if (b & NS2_BTN_MINUS)  r.buttons1 |= XB_BACK;
    if (b & NS2_BTN_LSTICK) r.buttons1 |= XB_LS;
    if (b & NS2_BTN_RSTICK) r.buttons1 |= XB_RS;
    if (b & NS2_BTN_L)      r.buttons2 |= XB_LB;
    if (b & NS2_BTN_R)      r.buttons2 |= XB_RB;
    if (b & NS2_BTN_HOME)   r.buttons2 |= XB_GUIDE;
    if (b & NS2_BTN_B)      r.buttons2 |= XB_A;
    if (b & NS2_BTN_A)      r.buttons2 |= XB_B;
    if (b & NS2_BTN_Y)      r.buttons2 |= XB_X;
    if (b & NS2_BTN_X)      r.buttons2 |= XB_Y;
    r.lt = (b & NS2_BTN_ZL) ? 255 : 0;
    r.rt = (b & NS2_BTN_ZR) ? 255 : 0;
    r.lx = axis16(o->lx);
    r.ly = axis16(o->ly);
    r.rx = axis16(o->rx);
    r.ry = axis16(o->ry);

    int64_t now = esp_timer_get_time();
    if (s_have_last && memcmp(&r, &s_last, sizeof(r)) == 0 && now - s_last_sent < 100000) {
        return true;
    }
    if (!usbd_edpt_claim(0, s_ep_in)) {
        return false;
    }
    s_report = r;
    bool ok = usbd_edpt_xfer(0, s_ep_in, (uint8_t *)&s_report, sizeof(s_report));
    usbd_edpt_release(0, s_ep_in);
    if (!ok) {
        return false;
    }
    s_last = r;
    s_have_last = true;
    s_last_sent = now;
    return true;
}

static void reset(void)
{
    s_have_last = false;
    s_ep_in = s_ep_out = 0;
}

const out_driver_t out_xinput = {
    .name = "XInput (Xbox 360)",
    .vid = 0x045E, .pid = 0x028E, .bcd_device = 0x0114,
    .manufacturer = "Microsoft Corporation",
    .product = "Controller",
    .itf_desc = itf_desc, .itf_desc_len = sizeof(itf_desc), .itf_count = 1,
    .hid_report_desc = NULL,
    .dev_class = 0xFF, .dev_subclass = 0xFF, .dev_protocol = 0xFF,
    .send = send,
    .reset = reset,
    .period_ms = 100,
};
