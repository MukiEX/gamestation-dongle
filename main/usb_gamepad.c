// USB device: the gamepad interface of the selected output mode (see out.h)
// plus a CDC serial port that mirrors the firmware log.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include "usb_gamepad.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "tusb.h"

#include "ns2_proto.h"
#include "out.h"
#include "pad.h"
#include "rumble.h"
#include "settings.h"
#include "config_server.h"

static const char *TAG = "usb";

static const out_driver_t *s_drv;
// TinyUSB may only be touched while it is installed; the log task and the
// report task check this, and a mode switch takes s_usb_mtx while it
// reinstalls the stack.
static volatile bool s_usb_up;
static SemaphoreHandle_t s_usb_mtx;
static volatile int s_switch_to = -1;  // pending switch: an out_mode_t, or TARGET_CONFIG
// Set when the host configures the device, cleared on unplug and whenever the
// stack is reinstalled. TinyUSB's own tud_mounted() can report the previous
// stack's state until the new stack's task has initialised, so it isn't used.
static volatile bool s_mounted;

static void usb_event(tinyusb_event_t *event, void *arg)
{
    (void)arg;
    if (event->id == TINYUSB_EVENT_ATTACHED) {
        s_mounted = true;
        if (s_drv && s_drv->is_config) {
            ESP_LOGI(TAG, "PC accepted the configurator's USB network device");
        }
        usb_gamepad_poke();
    } else if (event->id == TINYUSB_EVENT_DETACHED) {
        s_mounted = false;
    }
}

// ---------------------------------------------------------------------------
// Descriptors (built at start-up from the selected mode)
// ---------------------------------------------------------------------------

#define EP_CDC_NOTIF 0x82
#define EP_CDC_OUT   0x03
#define EP_CDC_IN    0x83

static tusb_desc_device_t s_device_desc;
static uint8_t s_config_desc[320];
static char s_serial[13];
#define MOUSE_STRING_INDEX (CONFIG_MAC_STRING_INDEX + 1)
static const char *s_strings[MOUSE_STRING_INDEX + 1];

// Optional USB mouse next to the controller (Mouse setting): an extra HID
// interface. TinyUSB numbers HID instances in interface order, so the mouse is
// instance 1 after a HID controller, or 0 after XInput (not HID).
#define EP_MOUSE_IN 0x84
static bool s_with_mouse;
static uint8_t s_mouse_inst;
static const uint8_t s_mouse_report_desc[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (1)
    0x29, 0x03,        //     Usage Maximum (3)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x03,        //     Report Count (3)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data, Var, Abs)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x05,        //     Report Size (5)
    0x81, 0x03,        //     Input (Const)
    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x16, 0x01, 0x80,  //     Logical Minimum (-32767)
    0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
    0x75, 0x10,        //     Report Size (16)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x06,        //     Input (Data, Var, Rel)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data, Var, Rel)
    0xC0,              //   End Collection
    0xC0,              // End Collection
};
static bool s_with_cdc;  // the log port is part of the current USB device

static const out_driver_t *driver_for(out_mode_t mode);

// Each output mode gets its own identity (USB serial, controller MAC): Steam
// remembers a controller's type by its serial, so one shared identity made it
// show every mode as whichever it met first. XInput keeps the plain MAC.
static uint8_t drv_tag(void)
{
    for (int m = 0; m < OUT_MODE_COUNT; m++) {
        if (driver_for((out_mode_t)m) == s_drv) return (uint8_t)m;
    }
    return 0x0F;  // configurator
}

static void build_descriptors(void)
{
    uint8_t mac[6] = { 0 };
    esp_read_mac(mac, ESP_MAC_BASE);
    mac[5] ^= (uint8_t)(drv_tag() << 4);
    snprintf(s_serial, sizeof(s_serial), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // The log port is always there in the configurator, and in the controller
    // modes unless it was switched off (some consoles reject extra interfaces).
    s_with_cdc = s_drv->is_config || settings_get().log_port;
    bool iad = s_with_cdc || s_drv->is_config;
    s_device_desc = (tusb_desc_device_t){
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,
        // Interface Association Descriptors (CDC, NCM) need the "misc" class.
        .bDeviceClass = iad ? TUSB_CLASS_MISC : s_drv->dev_class,
        .bDeviceSubClass = iad ? MISC_SUBCLASS_COMMON : s_drv->dev_subclass,
        .bDeviceProtocol = iad ? MISC_PROTOCOL_IAD : s_drv->dev_protocol,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
        .idVendor = s_drv->vid,
        .idProduct = s_drv->pid,
        .bcdDevice = s_drv->bcd_device,
        .iManufacturer = 1,
        .iProduct = 2,
        .iSerialNumber = 3,
        .bNumConfigurations = 1,
    };

    uint8_t itf_cdc = s_drv->itf_count;
    const uint8_t cdc[] = { TUD_CDC_DESCRIPTOR(itf_cdc, 5, EP_CDC_NOTIF, 8, EP_CDC_OUT, EP_CDC_IN, 64) };
    uint16_t cdc_len = s_with_cdc ? sizeof(cdc) : 0;
    s_with_mouse = !s_drv->is_config && settings_get().pad_mouse != PAD_MOUSE_OFF;
    s_mouse_inst = s_drv->hid_report_desc ? 1 : 0;
    uint8_t itf_mouse = itf_cdc + (s_with_cdc ? 2 : 0);
    const uint8_t mouse[] = { TUD_HID_DESCRIPTOR(itf_mouse, MOUSE_STRING_INDEX, HID_ITF_PROTOCOL_NONE,
                                                 sizeof(s_mouse_report_desc), EP_MOUSE_IN, 8, 1) };
    uint16_t mouse_len = s_with_mouse ? sizeof(mouse) : 0;
    uint16_t total = TUD_CONFIG_DESC_LEN + s_drv->itf_desc_len + cdc_len + mouse_len;
    configASSERT(total <= sizeof(s_config_desc));
    const uint8_t hdr[] = { TUD_CONFIG_DESCRIPTOR(1, itf_mouse + (s_with_mouse ? 1 : 0), 0, total,
                                                  TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500) };
    size_t n = 0;
    memcpy(s_config_desc + n, hdr, sizeof(hdr));
    n += sizeof(hdr);
    memcpy(s_config_desc + n, s_drv->itf_desc, s_drv->itf_desc_len);
    n += s_drv->itf_desc_len;
    if (s_with_cdc) {
        memcpy(s_config_desc + n, cdc, sizeof(cdc));
        n += sizeof(cdc);
    }
    if (s_with_mouse) {
        memcpy(s_config_desc + n, mouse, sizeof(mouse));
    }

    s_strings[0] = (const char[]){ 0x09, 0x04 };  // English (0x0409)
    s_strings[1] = s_drv->manufacturer;
    s_strings[2] = s_drv->product;
    s_strings[3] = s_serial;
    s_strings[4] = s_drv->product;   // gamepad interface (Windows shows this name)
    s_strings[5] = "Joy-Con 2 Dongle log";
    s_strings[CONFIG_MAC_STRING_INDEX] = config_server_host_mac_string();
    s_strings[MOUSE_STRING_INDEX] = "Joy-Con 2 Dongle mouse";
}

static bool is_mouse(uint8_t instance)
{
    return s_with_mouse && instance == s_mouse_inst;
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    return is_mouse(instance) ? s_mouse_report_desc : s_drv->hid_report_desc;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type,
                               uint8_t *buffer, uint16_t reqlen)
{
    if (is_mouse(instance)) {
        return 0;
    }
    return s_drv->get_report ? s_drv->get_report(report_id, type, buffer, reqlen) : 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type,
                           uint8_t const *buffer, uint16_t bufsize)
{
    if (is_mouse(instance)) {
        return;
    }
    if (s_drv->set_report) {
        s_drv->set_report(report_id, type, buffer, bufsize);
    }
}

void out_get_mac(uint8_t mac[6])
{
    esp_read_mac(mac, ESP_MAC_BT);
    mac[5] ^= (uint8_t)(drv_tag() << 4);
}

uint8_t out_hat(uint32_t b)
{
    bool up = b & NS2_BTN_UP, down = b & NS2_BTN_DOWN;
    bool left = b & NS2_BTN_LEFT, right = b & NS2_BTN_RIGHT;
    if (up && down) up = down = false;
    if (left && right) left = right = false;
    if (up && right) return 1;
    if (right && down) return 3;
    if (down && left) return 5;
    if (left && up) return 7;
    if (up) return 0;
    if (right) return 2;
    if (down) return 4;
    if (left) return 6;
    return 8;
}

// ---------------------------------------------------------------------------
// Log mirror to the CDC port
// ---------------------------------------------------------------------------

// Every log line is formatted ONCE, into a static buffer, by the task that
// logs it; the text is written to the UART console and copied into a ring
// buffer. A separate low-priority task drains the ring to the CDC port, so a
// logging task never calls into TinyUSB and never needs a big stack. The ring
// keeps the most recent 16 KB, so lines logged before the host opened the
// port (boot messages, crash reports) are still delivered.

#define LOG_RING_SIZE 16384

static vprintf_like_t s_uart_vprintf;
static SemaphoreHandle_t s_fmt_mtx;
static volatile bool s_log_ready;
static char s_fmt_buf[320];

static char s_ring[LOG_RING_SIZE];
static size_t s_ring_head;   // next write position
static size_t s_ring_len;    // bytes waiting
static portMUX_TYPE s_ring_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_log_task;

static void ring_push(const char *data, size_t len)
{
    if (len > LOG_RING_SIZE) {
        data += len - LOG_RING_SIZE;
        len = LOG_RING_SIZE;
    }
    portENTER_CRITICAL(&s_ring_mux);
    for (size_t i = 0; i < len; i++) {
        s_ring[s_ring_head] = data[i];
        s_ring_head = (s_ring_head + 1) % LOG_RING_SIZE;
    }
    s_ring_len += len;
    if (s_ring_len > LOG_RING_SIZE) {
        s_ring_len = LOG_RING_SIZE;  // oldest bytes overwritten
    }
    portEXIT_CRITICAL(&s_ring_mux);
}

// Copies up to `max` waiting bytes out of the ring (without removing them).
static size_t ring_peek(char *out, size_t max)
{
    portENTER_CRITICAL(&s_ring_mux);
    size_t n = s_ring_len < max ? s_ring_len : max;
    size_t tail = (s_ring_head + LOG_RING_SIZE - s_ring_len) % LOG_RING_SIZE;
    for (size_t i = 0; i < n; i++) {
        out[i] = s_ring[(tail + i) % LOG_RING_SIZE];
    }
    portEXIT_CRITICAL(&s_ring_mux);
    return n;
}

static void ring_drop(size_t n)
{
    portENTER_CRITICAL(&s_ring_mux);
    s_ring_len = n > s_ring_len ? 0 : s_ring_len - n;
    portEXIT_CRITICAL(&s_ring_mux);
}

static int log_vprintf(const char *fmt, va_list args)
{
    if (!s_log_ready || xPortInIsrContext() || xSemaphoreTake(s_fmt_mtx, 0) != pdTRUE) {
        return s_uart_vprintf(fmt, args);
    }
    int n = vsnprintf(s_fmt_buf, sizeof(s_fmt_buf), fmt, args);
    if (n > 0) {
        size_t len = (size_t)n < sizeof(s_fmt_buf) ? (size_t)n : sizeof(s_fmt_buf) - 1;
        fputs(s_fmt_buf, stdout);
        ring_push(s_fmt_buf, len);
    }
    xSemaphoreGive(s_fmt_mtx);
    if (s_log_task) {
        xTaskNotifyGive(s_log_task);
    }
    return n;
}

static void log_task(void *arg)
{
    (void)arg;
    char chunk[64];
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        // Only send while the host has the port open (DTR raised); until then
        // the ring keeps the most recent lines.
        if (!s_usb_up || xSemaphoreTake(s_usb_mtx, 0) != pdTRUE) {
            continue;
        }
        if (!s_usb_up) {
            xSemaphoreGive(s_usb_mtx);
            continue;
        }
        while (s_mounted && tud_cdc_n_connected(0)) {
            size_t space = tud_cdc_n_write_available(0);
            if (space == 0) {
                tud_cdc_n_write_flush(0);
                vTaskDelay(pdMS_TO_TICKS(2));
                break;
            }
            size_t n = ring_peek(chunk, space < sizeof(chunk) ? space : sizeof(chunk));
            if (n == 0) {
                break;
            }
            size_t written = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (const uint8_t *)chunk, n);
            ring_drop(written);
            if (written < n) {
                break;
            }
        }
        if (s_mounted && tud_cdc_n_connected(0)) {
            tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
        }
        xSemaphoreGive(s_usb_mtx);
    }
}

// ---------------------------------------------------------------------------
// Report task
// ---------------------------------------------------------------------------

#define COMBO_US 3000000  // hold a hotkey combination this long

#define TARGET_NONE   (-1)
#define TARGET_CONFIG 100

static TaskHandle_t s_task;
static void switch_to(int target);
static const out_driver_t *driver_for(out_mode_t mode);
static const char *s_fw_version = "?";

void usb_gamepad_set_firmware_version(const char *v)
{
    s_fw_version = v;
}

const char *usb_gamepad_firmware_version(void)
{
    return s_fw_version;
}

void usb_gamepad_poke(void)
{
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

typedef struct {
    uint32_t combo;
    int64_t since;
    bool armed;
} hotkey_t;

// True once when `combo` (and nothing else) has been held for COMBO_US.
static bool hotkey(hotkey_t *h, uint32_t buttons)
{
    if ((buttons & NS2_BTN_MASK) == h->combo) {
        int64_t now = esp_timer_get_time();
        if (!h->armed) {
            return false;
        }
        if (h->since == 0) {
            h->since = now;
        } else if (now - h->since >= COMBO_US) {
            h->since = 0;
            h->armed = false;
            return true;
        }
    } else {
        h->since = 0;
        h->armed = true;
    }
    return false;
}

static void check_hotkeys(const pad_out_t *o)
{
    static hotkey_t mode = { .combo = NS2_BTN_MINUS | NS2_BTN_PLUS, .armed = true };
    static hotkey_t config = { .combo = NS2_BTN_CAPT | NS2_BTN_HOME, .armed = true };
    if (!settings_get().mode_hotkeys) {
        return;
    }
    if (hotkey(&mode, o->buttons) && !s_drv->is_config) {
        usb_gamepad_next_mode();
    }
    if (hotkey(&config, o->buttons)) {
        if (s_drv->is_config) {
            usb_gamepad_exit_config();
        } else {
            usb_gamepad_enter_config();
        }
    }
}

// Which motion axes the configurator flips for this mode (Steam testing).
static void log_motion_flips(void)
{
    char code[20];
    settings_t st = settings_get();
    if (s_drv == &out_ds4 || s_drv == &out_ds5) {
        ESP_LOGI(TAG, "%s motion: %s", s_drv == &out_ds4 ? "DS4" : "DualSense", settings_flip_code(st.flip_ds4, code));
    } else if (s_drv == &out_switchpro) {
        ESP_LOGI(TAG, "Switch Pro motion: %s", settings_flip_code(st.flip_switch, code));
    }
}

// Pro Controller 2 back buttons: GL / GR press whatever the settings say.
static uint32_t grip_target(grip_map_t m, bool left)
{
    static const uint32_t bits[GRIP_COUNT] = {
        [GRIP_A] = NS2_BTN_A, [GRIP_B] = NS2_BTN_B, [GRIP_X] = NS2_BTN_X, [GRIP_Y] = NS2_BTN_Y,
        [GRIP_L] = NS2_BTN_L, [GRIP_R] = NS2_BTN_R, [GRIP_ZL] = NS2_BTN_ZL, [GRIP_ZR] = NS2_BTN_ZR,
        [GRIP_LSTICK] = NS2_BTN_LSTICK, [GRIP_RSTICK] = NS2_BTN_RSTICK,
        [GRIP_MINUS] = NS2_BTN_MINUS, [GRIP_PLUS] = NS2_BTN_PLUS,
        [GRIP_HOME] = NS2_BTN_HOME, [GRIP_CAPTURE] = NS2_BTN_CAPT,
        [GRIP_UP] = NS2_BTN_UP, [GRIP_DOWN] = NS2_BTN_DOWN, [GRIP_LEFT] = NS2_BTN_LEFT, [GRIP_RIGHT] = NS2_BTN_RIGHT,
        [GRIP_TP_LEFT] = NS2_BTN_TP_LEFT, [GRIP_TP_RIGHT] = NS2_BTN_TP_RIGHT,
    };
    if (m == GRIP_AUTO) {
        if (s_drv == &out_ds4 || s_drv == &out_ds5) return left ? NS2_BTN_TP_LEFT : NS2_BTN_TP_RIGHT;
        if (s_drv == &out_hid || s_drv == &out_steam) {
            return left ? NS2_BTN_GL : NS2_BTN_GR;  // HID buttons 20 / 21; Steam L4 / R4
        }
        return 0;
    }
    return m < GRIP_COUNT ? bits[m] : 0;
}

static void apply_grips(pad_out_t *o)
{
    settings_t st = settings_get();
    uint32_t b = o->buttons & ~(NS2_BTN_GL | NS2_BTN_GR);
    if (o->buttons & NS2_BTN_GL) b |= grip_target(st.grip_gl, true);
    if (o->buttons & NS2_BTN_GR) b |= grip_target(st.grip_gr, false);
    o->buttons = b;
}

// "Label" layout: Nintendo's A/B/X/Y keep their names (Switch A -> Xbox A),
// instead of their positions. The Switch Pro mode is labelled the same as the
// controllers already, so it never swaps.
static void apply_layout(pad_out_t *o)
{
    if (settings_get().layout != LAYOUT_LABELS || s_drv == &out_switchpro) {
        return;
    }
    uint32_t b = o->buttons & ~(NS2_BTN_A | NS2_BTN_B | NS2_BTN_X | NS2_BTN_Y);
    if (o->buttons & NS2_BTN_A) b |= NS2_BTN_B;
    if (o->buttons & NS2_BTN_B) b |= NS2_BTN_A;
    if (o->buttons & NS2_BTN_X) b |= NS2_BTN_Y;
    if (o->buttons & NS2_BTN_Y) b |= NS2_BTN_X;
    o->buttons = b;
}

// The USB mouse: movement built up from the sensors, the borrowed buttons,
// and the wheel from a stick (about 15 notches a second at full tilt).
static void send_mouse(const pad_out_t *o)
{
    static uint8_t last_buttons;
    static float wheel_acc;
    static int64_t last_t;
    int64_t now = esp_timer_get_time();
    float dt = last_t ? (now - last_t) / 1e6f : 0.0f;
    last_t = now;
    if (!tud_hid_n_ready(s_mouse_inst)) {
        return;  // movement keeps building up meanwhile
    }
    wheel_acc += o->mouse_wheel * 15.0f * (dt > 0.1f ? 0.1f : dt);
    int wheel = (int)wheel_acc;
    wheel_acc -= wheel;
    if (o->mouse_wheel == 0.0f) wheel_acc = 0.0f;
    int32_t dx, dy;
    pad_mouse_take(&dx, &dy);
    if (dx == 0 && dy == 0 && wheel == 0 && o->mouse_buttons == last_buttons) {
        return;
    }
    if (dx > 32767) dx = 32767;
    if (dx < -32767) dx = -32767;
    if (dy > 32767) dy = 32767;
    if (dy < -32767) dy = -32767;
    uint8_t r[6] = { o->mouse_buttons, (uint8_t)dx, (uint8_t)((uint16_t)dx >> 8),
                     (uint8_t)dy, (uint8_t)((uint16_t)dy >> 8), (uint8_t)(int8_t)wheel };
    if (tud_hid_n_report(s_mouse_inst, 0, r, sizeof(r))) {
        last_buttons = o->mouse_buttons;
    }
}

static void report_task(void *arg)
{
    (void)arg;
    TickType_t wait = pdMS_TO_TICKS(s_drv->period_ms);
    bool was_mounted = false;
    for (;;) {
        // Woken by every controller report, by the USB side, and at least every period.
        ulTaskNotifyTake(pdTRUE, wait);
        int target = s_switch_to;
        if (target != TARGET_NONE) {
            s_switch_to = TARGET_NONE;
            switch_to(target);
            was_mounted = false;
            wait = pdMS_TO_TICKS(s_drv->period_ms);
            continue;
        }
        if (!s_usb_up) {
            wait = pdMS_TO_TICKS(100);
            continue;
        }
        pad_out_t out;
        pad_get_output(&out);
        check_hotkeys(&out);
        bool mounted = s_mounted;
        if (!mounted) {
            if (was_mounted) {
                rumble_stop();  // the PC went away: never leave a motor running
            }
            was_mounted = false;
            wait = pdMS_TO_TICKS(100);
            continue;
        }
        was_mounted = true;
        apply_grips(&out);   // before the layout, so "A" swaps like the real A
        apply_layout(&out);
        bool done = s_drv->send(&out);
        if (s_with_mouse) {
            send_mouse(&out);
        }
        wait = done ? pdMS_TO_TICKS(s_drv->period_ms) : pdMS_TO_TICKS(1);
    }
}

// ---------------------------------------------------------------------------
// Start-up
// ---------------------------------------------------------------------------

static const out_driver_t *driver_for(out_mode_t mode)
{
    switch (mode) {
    case OUT_MODE_DS4:        return &out_ds4;
    case OUT_MODE_DS5:        return &out_ds5;
    case OUT_MODE_STEAM:      return &out_steam;
    case OUT_MODE_SWITCH_PRO: return &out_switchpro;
    case OUT_MODE_HID:        return &out_hid;
    case OUT_MODE_XINPUT:
    default:                  return &out_xinput;
    }
}

static esp_err_t usb_install(void)
{
    build_descriptors();
    if (s_drv->reset) {
        s_drv->reset();
    }
    // Static: esp_tinyusb keeps a pointer to the descriptor part and its task
    // reads it after tinyusb_driver_install() returns.
    static tinyusb_config_t cfg;
    cfg = (tinyusb_config_t)TINYUSB_DEFAULT_CONFIG();
    cfg.descriptor.device = &s_device_desc;
    cfg.descriptor.string = s_strings;
    cfg.descriptor.string_count = sizeof(s_strings) / sizeof(s_strings[0]);
    cfg.descriptor.full_speed_config = s_config_desc;
    cfg.task.xCoreID = 1;
    cfg.event_cb = usb_event;
    s_mounted = false;
    esp_err_t err = tinyusb_driver_install(&cfg);
    if (err != ESP_OK) {
        return err;
    }
    tinyusb_config_cdcacm_t cdc = {
        .cdc_port = TINYUSB_CDC_ACM_0,
    };
    err = tinyusb_cdcacm_init(&cdc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB serial (log) port failed to start: %s", esp_err_to_name(err));
    }
    s_usb_up = true;
    if (s_drv->started) {
        s_drv->started();
    }
    return ESP_OK;
}

// Re-enumerates as a different device without restarting the dongle, so the
// Bluetooth connections stay up: leave the bus, tear the USB stack down, and
// bring it back with the new descriptors.
static void switch_to(int target)
{
    const out_driver_t *next = target == TARGET_CONFIG ? &out_config : driver_for((out_mode_t)target);
    ESP_LOGW(TAG, "switching USB to %s", next->name);
    vTaskDelay(pdMS_TO_TICKS(250));  // let that line reach the PC, and a rumble cue play
    rumble_stop();
    xSemaphoreTake(s_usb_mtx, portMAX_DELAY);
    s_usb_up = false;
    s_mounted = false;
    tud_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    if (s_drv->stopping) {
        s_drv->stopping();
    }
    tinyusb_cdcacm_deinit(TINYUSB_CDC_ACM_0);
    esp_err_t err = tinyusb_driver_uninstall();
    if (err != ESP_OK) {
        xSemaphoreGive(s_usb_mtx);
        ESP_LOGE(TAG, "couldn't stop USB (%s); restarting instead", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    }
    vTaskDelay(pdMS_TO_TICKS(500));  // long enough for the PC to see an unplug
    s_drv = next;
    err = usb_install();
    xSemaphoreGive(s_usb_mtx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB restart failed (%s); restarting the dongle", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    }
    ESP_LOGI(TAG, "USB started as %s (%04x:%04x)%s; controllers stay connected", s_drv->name, s_drv->vid,
             s_drv->pid, s_with_cdc ? "" : ", log port off");
    log_motion_flips();
}

static void request(int target)
{
    if (s_task) {
        s_switch_to = target;
        usb_gamepad_poke();
    } else {
        ESP_LOGW(TAG, "USB is off in this build");
    }
}

void usb_gamepad_next_mode(void)
{
    if (s_drv && s_drv->is_config) {
        usb_gamepad_exit_config();  // BOOT in the configurator = leave it
        return;
    }
    request(settings_next_out_mode());  // saved for the next power-up too
}

void usb_gamepad_enter_config(void)
{
    if (s_drv && !s_drv->is_config) {
        rumble_cue(RUMBLE_CUE_DOUBLE);  // felt before USB goes away
        request(TARGET_CONFIG);
    }
}

void usb_gamepad_exit_config(void)
{
    if (s_drv && s_drv->is_config) {
        rumble_cue(RUMBLE_CUE_SINGLE);
        request(settings_out_mode());
    }
}

bool usb_gamepad_in_config(void)
{
    return s_drv && s_drv->is_config;
}

void usb_gamepad_reapply(void)
{
    if (s_drv && !s_drv->is_config) {
        request(settings_out_mode());
    }
}

void usb_gamepad_start(void)
{
    s_drv = driver_for(settings_out_mode());
    s_usb_mtx = xSemaphoreCreateMutex();

    // Serial number = the dongle's factory MAC (per mode, see drv_tag), so
    // multiple dongles are distinct; set in build_descriptors.

    esp_err_t err = usb_install();
    if (err != ESP_OK) {
        // Keep running: the UART log still works, and Bluetooth can still be
        // debugged even if USB never comes up.
        ESP_LOGE(TAG, "USB stack failed to start (%s); continuing without USB", esp_err_to_name(err));
        return;
    }
    s_fmt_mtx = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(log_task, "usb_log", 3072, NULL, 2, &s_log_task, 1);
    s_uart_vprintf = esp_log_set_vprintf(log_vprintf);
    s_log_ready = true;

    xTaskCreatePinnedToCore(report_task, "usb_report", 4096, NULL, 7, &s_task, 1);
    ESP_LOGI(TAG, "USB started as %s (%04x:%04x)%s. Change mode: press BOOT or hold - and + for 3 s. "
                  "Configurator: hold BOOT 2 s or Capture + Home for 3 s",
             s_drv->name, s_drv->vid, s_drv->pid, s_with_cdc ? "" : ", log port off");
    log_motion_flips();
}
