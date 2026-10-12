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

static const char *TAG = "usb";

static const out_driver_t *s_drv;

// ---------------------------------------------------------------------------
// Descriptors (built at start-up from the selected mode)
// ---------------------------------------------------------------------------

#define EP_CDC_NOTIF 0x82
#define EP_CDC_OUT   0x03
#define EP_CDC_IN    0x83

static tusb_desc_device_t s_device_desc;
static uint8_t s_config_desc[320];
static char s_serial[13];
static const char *s_strings[6];

static void build_descriptors(void)
{
    s_device_desc = (tusb_desc_device_t){
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,
        // Interface Association Descriptor: needed for the CDC log port.
        .bDeviceClass = TUSB_CLASS_MISC,
        .bDeviceSubClass = MISC_SUBCLASS_COMMON,
        .bDeviceProtocol = MISC_PROTOCOL_IAD,
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
    uint16_t total = TUD_CONFIG_DESC_LEN + s_drv->itf_desc_len + sizeof(cdc);
    configASSERT(total <= sizeof(s_config_desc));
    const uint8_t hdr[] = { TUD_CONFIG_DESCRIPTOR(1, itf_cdc + 2, 0, total, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500) };
    size_t n = 0;
    memcpy(s_config_desc + n, hdr, sizeof(hdr));
    n += sizeof(hdr);
    memcpy(s_config_desc + n, s_drv->itf_desc, s_drv->itf_desc_len);
    n += s_drv->itf_desc_len;
    memcpy(s_config_desc + n, cdc, sizeof(cdc));

    s_strings[0] = (const char[]){ 0x09, 0x04 };  // English (0x0409)
    s_strings[1] = s_drv->manufacturer;
    s_strings[2] = s_drv->product;
    s_strings[3] = s_serial;
    s_strings[4] = s_drv->product;   // gamepad interface (Windows shows this name)
    s_strings[5] = "Joy-Con 2 Dongle log";
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return s_drv->hid_report_desc;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type,
                               uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    return s_drv->get_report ? s_drv->get_report(report_id, type, buffer, reqlen) : 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type,
                           uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance;
    if (s_drv->set_report) {
        s_drv->set_report(report_id, type, buffer, bufsize);
    }
}

void out_get_mac(uint8_t mac[6])
{
    esp_read_mac(mac, ESP_MAC_BT);
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
        while (tud_mounted() && tud_cdc_n_connected(0)) {
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
        if (tud_mounted() && tud_cdc_n_connected(0)) {
            tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
        }
    }
}

// ---------------------------------------------------------------------------
// Report task
// ---------------------------------------------------------------------------

#define MODE_COMBO_US 3000000  // hold - and + this long to change the output mode

static TaskHandle_t s_task;

void usb_gamepad_poke(void)
{
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

static void check_mode_combo(const pad_out_t *o)
{
    static int64_t since;
    const uint32_t combo = NS2_BTN_MINUS | NS2_BTN_PLUS;
    if ((o->buttons & combo) == combo) {
        int64_t now = esp_timer_get_time();
        if (since == 0) {
            since = now;
        } else if (now - since >= MODE_COMBO_US) {
            settings_cycle_out_mode();
        }
    } else {
        since = 0;
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
        pad_out_t out;
        pad_get_output(&out);
        check_mode_combo(&out);
        bool mounted = tud_mounted();
        if (!mounted) {
            if (was_mounted) {
                rumble_stop();  // the PC went away: never leave a motor running
            }
            was_mounted = false;
            wait = pdMS_TO_TICKS(100);
            continue;
        }
        was_mounted = true;
        bool done = s_drv->send(&out);
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
    case OUT_MODE_SWITCH_PRO: return &out_switchpro;
    case OUT_MODE_HID:        return &out_hid;
    case OUT_MODE_XINPUT:
    default:                  return &out_xinput;
    }
}

void usb_gamepad_start(void)
{
    s_drv = driver_for(settings_out_mode());

    // Serial number = the dongle's factory MAC, so multiple dongles are distinct.
    uint8_t mac[6] = { 0 };
    esp_read_mac(mac, ESP_MAC_BASE);
    snprintf(s_serial, sizeof(s_serial), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    build_descriptors();

    tinyusb_config_t cfg = TINYUSB_DEFAULT_CONFIG();
    cfg.descriptor.device = &s_device_desc;
    cfg.descriptor.string = s_strings;
    cfg.descriptor.string_count = sizeof(s_strings) / sizeof(s_strings[0]);
    cfg.descriptor.full_speed_config = s_config_desc;
    cfg.task.xCoreID = 1;
    esp_err_t err = tinyusb_driver_install(&cfg);
    if (err != ESP_OK) {
        // Keep running: the UART log still works, and Bluetooth can still be
        // debugged even if USB never comes up.
        ESP_LOGE(TAG, "USB stack failed to start (%s); continuing without USB", esp_err_to_name(err));
        return;
    }

    tinyusb_config_cdcacm_t cdc = {
        .cdc_port = TINYUSB_CDC_ACM_0,
    };
    err = tinyusb_cdcacm_init(&cdc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB serial (log) port failed to start: %s", esp_err_to_name(err));
    }
    s_fmt_mtx = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(log_task, "usb_log", 3072, NULL, 2, &s_log_task, 1);
    s_uart_vprintf = esp_log_set_vprintf(log_vprintf);
    s_log_ready = true;

    xTaskCreatePinnedToCore(report_task, "usb_report", 4096, NULL, 7, &s_task, 1);
    ESP_LOGI(TAG, "USB started as %s (%04x:%04x); to change: press BOOT, or hold - and + for 3 s",
             s_drv->name, s_drv->vid, s_drv->pid);
}
