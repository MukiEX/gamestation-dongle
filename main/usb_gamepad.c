// SPDX-License-Identifier: GPL-3.0-or-later
#include "usb_gamepad.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "tusb.h"

#include "ns2_proto.h"
#include "pad.h"

static const char *TAG = "usb";

// Test VID/PID (Espressif's VID, a PID from the range used for development).
// Each real output mode will use its own identity later.
#define USB_VID 0x303A
#define USB_PID 0x8243

// ---------------------------------------------------------------------------
// HID report
// ---------------------------------------------------------------------------

#define NUM_BUTTONS 21

typedef struct __attribute__((packed)) {
    uint8_t buttons[3];  // 21 buttons + 3 padding bits
    uint8_t hat;         // low nibble: 0-7 = N, NE, E, ... ; 8 = centred
    int16_t lx, ly;      // X, Y
    int16_t rx, ry;      // Z, Rz
    uint8_t lt, rt;      // Rx, Ry (digital on Joy-Cons: 0 or 255)
} gamepad_report_t;

static const uint8_t hid_report_desc[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    // Buttons
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (1)
    0x29, NUM_BUTTONS, //   Usage Maximum (21)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, NUM_BUTTONS, //   Report Count (21)
    0x81, 0x02,        //   Input (Data, Var, Abs)
    0x75, 0x01,        //   Report Size (1)
    0x95, 24 - NUM_BUTTONS, //   Report Count (padding)
    0x81, 0x03,        //   Input (Const)
    // Hat switch
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
    // Sticks: X, Y, Z, Rz as signed 16-bit
    0x09, 0x30,        //   Usage (X)
    0x09, 0x31,        //   Usage (Y)
    0x09, 0x32,        //   Usage (Z)
    0x09, 0x35,        //   Usage (Rz)
    0x16, 0x01, 0x80,  //   Logical Minimum (-32767)
    0x26, 0xFF, 0x7F,  //   Logical Maximum (32767)
    0x75, 0x10,        //   Report Size (16)
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x02,        //   Input (Data, Var, Abs)
    // Triggers: Rx, Ry as unsigned 8-bit
    0x09, 0x33,        //   Usage (Rx)
    0x09, 0x34,        //   Usage (Ry)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x02,        //   Input (Data, Var, Abs)
    0xC0,              // End Collection
};

_Static_assert(sizeof(gamepad_report_t) == 14, "report layout");

// ---------------------------------------------------------------------------
// Descriptors
// ---------------------------------------------------------------------------

enum { ITF_HID = 0, ITF_CDC, ITF_CDC_DATA, ITF_TOTAL };

#define EP_HID_IN     0x81
#define EP_CDC_NOTIF  0x82
#define EP_CDC_OUT    0x03
#define EP_CDC_IN     0x83

#define CONFIG_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + TUD_CDC_DESC_LEN)

static const tusb_desc_device_t device_desc = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    // Interface Association Descriptor (needed for the CDC function)
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0001,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const uint8_t config_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_TOTAL, 0, CONFIG_LEN, 0, 100),
    TUD_HID_DESCRIPTOR(ITF_HID, 4, HID_ITF_PROTOCOL_NONE, sizeof(hid_report_desc), EP_HID_IN,
                       16, 1),
    TUD_CDC_DESCRIPTOR(ITF_CDC, 5, EP_CDC_NOTIF, 8, EP_CDC_OUT, EP_CDC_IN, 64),
};

static char s_serial[13];

static const char *string_desc[] = {
    (const char[]){ 0x09, 0x04 },  // 0: English (0x0409)
    "Joy-Con 2 Dongle",            // 1: manufacturer
    "Joy-Con 2 Dongle (dev)",      // 2: product
    s_serial,                      // 3: serial (BLE MAC)
    "Gamepad",                     // 4: HID interface
    "Debug log",                   // 5: CDC interface
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return hid_report_desc;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type,
                               uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type,
                           uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance;
    (void)report_id;
    (void)type;
    (void)buffer;
    (void)bufsize;
}

// ---------------------------------------------------------------------------
// Log mirror to the CDC port
// ---------------------------------------------------------------------------

static vprintf_like_t s_uart_vprintf;
static SemaphoreHandle_t s_cdc_mtx;
static volatile bool s_cdc_ready;

static int log_vprintf(const char *fmt, va_list args)
{
    char buf[256];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(buf, sizeof(buf), fmt, copy);
    va_end(copy);

    int ret = s_uart_vprintf(fmt, args);

    if (n > 0 && s_cdc_ready && !xPortInIsrContext() && tud_cdc_n_connected(0) &&
        xSemaphoreTake(s_cdc_mtx, 0) == pdTRUE) {
        size_t len = (size_t)n < sizeof(buf) ? (size_t)n : sizeof(buf) - 1;
        // Never block: if the host isn't reading, log lines are dropped.
        if (tud_cdc_n_write_available(0) >= len) {
            tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (const uint8_t *)buf, len);
            tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
        }
        xSemaphoreGive(s_cdc_mtx);
    }
    return ret;
}

// ---------------------------------------------------------------------------
// Report task
// ---------------------------------------------------------------------------

static TaskHandle_t s_task;

void usb_gamepad_poke(void)
{
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

static int16_t axis16(float v)
{
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return (int16_t)lrintf(v * 32767.0f);
}

static uint8_t hat_from(uint32_t b)
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

static void build_report(const pad_out_t *o, gamepad_report_t *r)
{
    // Positional layout: 1 = south, 2 = east, 3 = west, 4 = north.
    static const uint32_t order[NUM_BUTTONS] = {
        NS2_BTN_B, NS2_BTN_A, NS2_BTN_Y, NS2_BTN_X,
        NS2_BTN_L, NS2_BTN_R, NS2_BTN_ZL, NS2_BTN_ZR,
        NS2_BTN_MINUS, NS2_BTN_PLUS, NS2_BTN_LSTICK, NS2_BTN_RSTICK,
        NS2_BTN_HOME, NS2_BTN_CAPT, NS2_BTN_C,
        NS2_BTN_SL_L, NS2_BTN_SR_L, NS2_BTN_SL_R, NS2_BTN_SR_R,
        NS2_BTN_GL, NS2_BTN_GR,
    };
    memset(r, 0, sizeof(*r));
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (o->buttons & order[i]) {
            r->buttons[i / 8] |= (uint8_t)(1u << (i % 8));
        }
    }
    r->hat = hat_from(o->buttons);
    // HID convention: +Y is down.
    r->lx = axis16(o->lx);
    r->ly = axis16(-o->ly);
    r->rx = axis16(o->rx);
    r->ry = axis16(-o->ry);
    r->lt = (o->buttons & NS2_BTN_ZL) ? 255 : 0;
    r->rt = (o->buttons & NS2_BTN_ZR) ? 255 : 0;
}

static void report_task(void *arg)
{
    (void)arg;
    gamepad_report_t last = { 0 };
    last.hat = 8;
    bool have_last = false;
    int64_t last_sent = 0;
    for (;;) {
        // Woken by every controller report; also re-sends at least every 100 ms.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        if (!tud_mounted() || !tud_hid_ready()) {
            continue;
        }
        pad_out_t out;
        pad_get_output(&out);
        gamepad_report_t r;
        build_report(&out, &r);
        int64_t now = esp_timer_get_time();
        if (have_last && memcmp(&r, &last, sizeof(r)) == 0 && now - last_sent < 100000) {
            continue;
        }
        if (tud_hid_report(0, &r, sizeof(r))) {
            last = r;
            have_last = true;
            last_sent = now;
        }
    }
}

// ---------------------------------------------------------------------------
// Start-up
// ---------------------------------------------------------------------------

void usb_gamepad_start(void)
{
    // Serial number = the dongle's factory MAC, so multiple dongles are distinct.
    uint8_t mac[6] = { 0 };
    esp_read_mac(mac, ESP_MAC_BASE);
    snprintf(s_serial, sizeof(s_serial), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    tinyusb_config_t cfg = TINYUSB_DEFAULT_CONFIG();
    cfg.descriptor.device = &device_desc;
    cfg.descriptor.string = string_desc;
    cfg.descriptor.string_count = sizeof(string_desc) / sizeof(string_desc[0]);
    cfg.descriptor.full_speed_config = config_desc;
    cfg.task.xCoreID = 1;
    ESP_ERROR_CHECK(tinyusb_driver_install(&cfg));

    tinyusb_config_cdcacm_t cdc = {
        .cdc_port = TINYUSB_CDC_ACM_0,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&cdc));
    s_cdc_mtx = xSemaphoreCreateMutex();
    s_uart_vprintf = esp_log_set_vprintf(log_vprintf);
    s_cdc_ready = true;

    xTaskCreatePinnedToCore(report_task, "usb_report", 4096, NULL, 7, &s_task, 1);
    ESP_LOGI(TAG, "USB gamepad started (%04x:%04x)", USB_VID, USB_PID);
}
