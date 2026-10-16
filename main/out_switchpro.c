// Nintendo Switch Pro Controller output (057E:2009), USB protocol as
// implemented by GP2040-CE (MIT): the 0x80 handshake, 0x01 subcommands with
// 0x21 replies, SPI flash reads for calibration and colours, and 0x30 full
// input reports. HD rumble from 0x01 / 0x10 output reports drives the
// Switch 2 controllers' actuators. Gyro and accelerometer are passed through.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "ns2_proto.h"
#include "out.h"
#include "usb_gamepad.h"
#include "settings.h"
#include "rumble.h"

static const char *TAG = "switchpro";

#define EP_IN  0x81
#define EP_OUT 0x01
#define REPORT_PERIOD_US 8000  // a wired Pro Controller reports every 8 ms

static const uint8_t report_desc[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x15, 0x00,        // Logical Minimum (0)
    0x09, 0x04,        // Usage (Joystick)
    0xA1, 0x01,        // Collection (Application)

    0x85, 0x30,        //   Report ID (48)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (0x01)
    0x29, 0x0A,        //   Usage Maximum (0x0A)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x0A,        //   Report Count (10)
    0x55, 0x00,        //   Unit Exponent (0)
    0x65, 0x00,        //   Unit (None)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x0B,        //   Usage Minimum (0x0B)
    0x29, 0x0E,        //   Usage Maximum (0x0E)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x0B, 0x01, 0x00, 0x01, 0x00,  //   Usage (0x010001)
    0xA1, 0x00,        //   Collection (Physical)
    0x0B, 0x30, 0x00, 0x01, 0x00,  //     Usage (0x010030)
    0x0B, 0x31, 0x00, 0x01, 0x00,  //     Usage (0x010031)
    0x0B, 0x32, 0x00, 0x01, 0x00,  //     Usage (0x010032)
    0x0B, 0x35, 0x00, 0x01, 0x00,  //     Usage (0x010035)
    0x15, 0x00,        //     Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00,  //     Logical Maximum (65534)
    0x75, 0x10,        //     Report Size (16)
    0x95, 0x04,        //     Report Count (4)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection
    0x0B, 0x39, 0x00, 0x01, 0x00,  //   Usage (0x010039)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x07,        //   Logical Maximum (7)
    0x35, 0x00,        //   Physical Minimum (0)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x65, 0x14,        //   Unit (System: English Rotation, Length: Centimeter)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x0F,        //   Usage Minimum (0x0F)
    0x29, 0x12,        //   Usage Maximum (0x12)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x34,        //   Report Count (52)
    0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x06, 0x00, 0xFF,  //   Usage Page (Vendor Defined 0xFF00)

    0x85, 0x21,        //   Report ID (33)
    0x09, 0x01,        //   Usage (0x01)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x3F,        //   Report Count (63)
    0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x85, 0x81,        //   Report ID (-127)
    0x09, 0x02,        //   Usage (0x02)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x3F,        //   Report Count (63)
    0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x85, 0x01,        //   Report ID (1)
    0x09, 0x03,        //   Usage (0x03)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x3F,        //   Report Count (63)
    0x91, 0x83,        //   Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Volatile)

    0x85, 0x10,        //   Report ID (16)
    0x09, 0x04,        //   Usage (0x04)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x3F,        //   Report Count (63)
    0x91, 0x83,        //   Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Volatile)

    0x85, 0x80,        //   Report ID (-128)
    0x09, 0x05,        //   Usage (0x05)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x3F,        //   Report Count (63)
    0x91, 0x83,        //   Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Volatile)

    0x85, 0x82,        //   Report ID (-126)
    0x09, 0x06,        //   Usage (0x06)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x3F,        //   Report Count (63)
    0x91, 0x83,        //   Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Volatile)

    0xC0,              // End Collection
};

// Exactly a wired Pro Controller's interface (as in GP2040-CE): no interface
// string, IN endpoint before OUT. TinyUSB's TUD_HID_INOUT_DESCRIPTOR lists
// OUT first; a Switch 2 never finished setting the dongle up with that.
static const uint8_t itf_desc[] = {
    9, TUSB_DESC_INTERFACE, 0, 0, 2, TUSB_CLASS_HID, 0, HID_ITF_PROTOCOL_NONE, 0,
    9, HID_DESC_TYPE_HID, U16_TO_U8S_LE(0x0111), 0, 1, HID_DESC_TYPE_REPORT, U16_TO_U8S_LE(sizeof(report_desc)),
    7, TUSB_DESC_ENDPOINT, EP_IN, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(64), 8,
    7, TUSB_DESC_ENDPOINT, EP_OUT, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(64), 8,
};

// SPI flash contents the host reads (0x6000: factory data, 0x8000: user calibration).
static const uint8_t spi_6000[0x100] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0xA0, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0x02, 0xFF, 0xFF, 0xFF, 0xFF,
    // 0x6020: IMU factory calibration. Zero offsets, accel 16384 = 4 g
    // (4096 LSB/g), gyro 13371 = 936 dps (14.285 LSB per deg/s).
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3B, 0x34, 0x3B, 0x34, 0x3B, 0x34, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xa4, 0x46, 0x6a, 0x00, 0x08, 0x80, 0xa4, 0x46, 0x6a, 0x00, 0x08,
    0x80, 0xa4, 0x46, 0x6a, 0xa4, 0x46, 0x6a, 0xFF, 0x1B, 0x1B, 0x1D, 0xFF,
    0xFF, 0xFF, 0xEC, 0x00, 0x8C, 0xEC, 0x00, 0x8C, 0x01, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x50, 0xFD, 0x00, 0x00,
    0xC6, 0x0F, 0x0F, 0x30, 0x61, 0xAE, 0x90, 0xD9, 0xD4, 0x14, 0x54, 0x41,
    0x15, 0x54, 0xC7, 0x79, 0x9C, 0x33, 0x36, 0x63, 0x0F, 0x30, 0x61, 0xAE,
    0x90, 0xD9, 0xD4, 0x14, 0x54, 0x41, 0x15, 0x54, 0xC7, 0x79, 0x9C, 0x33,
    0x36, 0x63, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};
static const uint8_t spi_8000[0x40] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xB2, 0xA1, 0xa4, 0x46, 0x6a, 0x00, 0x08, 0x80,
    0xa4, 0x46, 0x6a, 0xB2, 0xA1, 0x00, 0x08, 0x80, 0xa4, 0x46, 0x6a, 0xa4,
    0x46, 0x6a, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

// Stick range from the calibration above: centre 0x800, +/- 0x6A4.
#define STICK_CENTER 0x800
#define STICK_RANGE  0x6A4

// ---- Reply queue (filled from the USB task, sent by the report task) --------

#define QLEN 6
static uint8_t s_queue[QLEN][64];
static int s_q_head, s_q_count;
static portMUX_TYPE s_q_mux = portMUX_INITIALIZER_UNLOCKED;

static void queue_reply(const uint8_t *r)
{
    portENTER_CRITICAL(&s_q_mux);
    if (s_q_count < QLEN) {
        memcpy(s_queue[(s_q_head + s_q_count) % QLEN], r, 64);
        s_q_count++;
    }
    portEXIT_CRITICAL(&s_q_mux);
    usb_gamepad_poke();
}

static bool queue_peek(uint8_t *out)
{
    bool have;
    portENTER_CRITICAL(&s_q_mux);
    have = s_q_count > 0;
    if (have) memcpy(out, s_queue[s_q_head], 64);
    portEXIT_CRITICAL(&s_q_mux);
    return have;
}

static void queue_pop(void)
{
    portENTER_CRITICAL(&s_q_mux);
    if (s_q_count > 0) {
        s_q_head = (s_q_head + 1) % QLEN;
        s_q_count--;
    }
    portEXIT_CRITICAL(&s_q_mux);
}

// ---- State --------------------------------------------------------------------

static volatile bool s_ready;       // host finished the USB handshake
static volatile uint8_t s_inputs[10];  // battery/connection, 3 button bytes, 2 sticks
static volatile uint8_t s_imu[36];     // 3 samples of accel X/Y/Z + gyro X/Y/Z
static uint8_t s_timer;
static uint8_t s_report[64];
static int64_t s_last_sent;
static bool s_sent_hello;

static void put_stick(volatile uint8_t *d, float x, float y)
{
    uint16_t sx = (uint16_t)lrintf(STICK_CENTER + out_clampf(x, -1.0f, 1.0f) * STICK_RANGE);
    uint16_t sy = (uint16_t)lrintf(STICK_CENTER + out_clampf(y, -1.0f, 1.0f) * STICK_RANGE);
    d[0] = sx & 0xFF;
    d[1] = (uint8_t)(((sx >> 8) & 0x0F) | ((sy & 0x0F) << 4));
    d[2] = (uint8_t)(sy >> 4);
}

static int16_t s16(float v)
{
    return (int16_t)lrintf(out_clampf(v, -32768.0f, 32767.0f));
}

static void update_inputs(const pad_out_t *o)
{
    uint32_t b = o->buttons;
    uint8_t in[10] = { 0x91 };  // battery full, charging, USB powered
    // Switch layout is the controllers' own: no remapping needed.
    if (b & NS2_BTN_Y)      in[1] |= 0x01;
    if (b & NS2_BTN_X)      in[1] |= 0x02;
    if (b & NS2_BTN_B)      in[1] |= 0x04;
    if (b & NS2_BTN_A)      in[1] |= 0x08;
    if (b & NS2_BTN_R)      in[1] |= 0x40;
    if (b & NS2_BTN_ZR)     in[1] |= 0x80;
    if (b & NS2_BTN_MINUS)  in[2] |= 0x01;
    if (b & NS2_BTN_PLUS)   in[2] |= 0x02;
    if (b & NS2_BTN_RSTICK) in[2] |= 0x04;
    if (b & NS2_BTN_LSTICK) in[2] |= 0x08;
    if (b & NS2_BTN_HOME)   in[2] |= 0x10;
    if (b & NS2_BTN_CAPT)   in[2] |= 0x20;
    in[2] |= 0x80;  // charging grip
    if (b & NS2_BTN_DOWN)   in[3] |= 0x01;
    if (b & NS2_BTN_UP)     in[3] |= 0x02;
    if (b & NS2_BTN_RIGHT)  in[3] |= 0x04;
    if (b & NS2_BTN_LEFT)   in[3] |= 0x08;
    if (b & NS2_BTN_L)      in[3] |= 0x40;
    if (b & NS2_BTN_ZL)     in[3] |= 0x80;
    put_stick(&in[4], o->lx, o->ly);
    put_stick(&in[7], o->rx, o->ry);

    // Switch axes from ours (see pad.h), as SDL maps them back:
    // X = -Z (toward the top), Y = -X (left), Z = Y (out of the face).
    int16_t v[6] = { 0, 0, 4096, 0, 0, 0 };  // no motion: lying flat
    if (o->motion.valid) {
        float g[3], a[3];
        out_motion_flipped(&o->motion, settings_get().flip_switch, g, a);
        const float A = 4096.0f, G = 13371.0f / 936.0f;  // the SPI calibration's scales
        v[0] = s16(-a[2] * A); v[1] = s16(-a[0] * A); v[2] = s16(a[1] * A);
        v[3] = s16(-g[2] * G); v[4] = s16(-g[0] * G); v[5] = s16(g[1] * G);
    }
    uint8_t imu[36];
    for (int k = 0; k < 3; k++) {  // one fresh sample per report: repeat it
        for (int i = 0; i < 6; i++) {
            imu[k * 12 + i * 2] = (uint8_t)v[i];
            imu[k * 12 + i * 2 + 1] = (uint8_t)((uint16_t)v[i] >> 8);
        }
    }
    portENTER_CRITICAL(&s_q_mux);
    memcpy((void *)s_inputs, in, sizeof(in));
    memcpy((void *)s_imu, imu, sizeof(imu));
    portEXIT_CRITICAL(&s_q_mux);
}

static void copy_inputs(uint8_t *dst, uint8_t *imu)
{
    portENTER_CRITICAL(&s_q_mux);
    memcpy(dst, (const void *)s_inputs, sizeof(s_inputs));
    memcpy(imu, (const void *)s_imu, sizeof(s_imu));
    portEXIT_CRITICAL(&s_q_mux);
}

static void identify_reply(uint8_t *r)
{
    uint8_t mac[6];
    out_get_mac(mac);
    memset(r, 0, 64);
    r[0] = 0x81;
    r[1] = 0x01;
    r[2] = 0x00;
    r[3] = 0x03;  // Pro Controller
    for (int i = 0; i < 6; i++) {
        r[4 + i] = mac[5 - i];
    }
}

static bool send(const pad_out_t *o)
{
    update_inputs(o);
    if (!tud_hid_ready()) {
        return false;
    }
    uint8_t r[64];
    if (queue_peek(r)) {
        if (!tud_hid_report(0, r, 64)) {
            return false;
        }
        queue_pop();
        return true;
    }
    if (!s_sent_hello) {
        // Announce ourselves once, as GP2040-CE does; hosts then start the handshake.
        identify_reply(r);
        if (!tud_hid_report(0, r, 64)) {
            return false;
        }
        s_sent_hello = true;
        return true;
    }
    if (!s_ready) {
        return true;
    }
    int64_t now = esp_timer_get_time();
    if (now - s_last_sent < REPORT_PERIOD_US) {
        return true;
    }
    memset(s_report, 0, sizeof(s_report));
    s_report[0] = 0x30;
    s_report[1] = s_timer;
    copy_inputs(&s_report[2], &s_report[13]);
    s_report[12] = 0x09;  // vibrator input report
    if (!tud_hid_report(0, s_report, sizeof(s_report))) {
        return false;
    }
    s_timer++;
    s_last_sent = now;
    return true;
}

static void spi_read(uint8_t *dst, uint32_t addr, uint8_t size)
{
    uint32_t bank = addr & 0xFFFFFF00u, off = addr & 0xFFu;
    if (bank == 0x6000 && off + size <= sizeof(spi_6000)) {
        memcpy(dst, spi_6000 + off, size);
    } else if (bank == 0x8000 && off + size <= sizeof(spi_8000)) {
        memcpy(dst, spi_8000 + off, size);
    } else {
        memset(dst, 0xFF, size);
    }
}

static void handle_subcommand(const uint8_t *d, uint16_t n)
{
    if (n < 11) {
        return;
    }
    uint8_t cmd = d[10];
    uint8_t r[64];
    memset(r, 0, sizeof(r));
    r[0] = 0x21;
    r[1] = s_timer++;
    uint8_t imu_unused[36];
    copy_inputs(&r[2], imu_unused);
    r[13] = 0x80;  // ACK
    r[14] = cmd;
    switch (cmd) {
    case 0x02: {  // device info
        uint8_t mac[6];
        out_get_mac(mac);
        r[13] = 0x82;
        r[15] = 0x04; r[16] = 0x91;  // firmware 4.91
        r[17] = 0x03; r[18] = 0x02;  // Pro Controller
        for (int i = 0; i < 6; i++) r[19 + i] = mac[i];
        r[25] = 0x01;
        r[26] = 0x02;  // colours stored in SPI
        break;
    }
    case 0x03: r[15] = n > 11 ? d[11] : 0x30; break;  // input mode
    case 0x04: r[13] = 0x83; break;                   // trigger button elapsed time
    case 0x10: {  // SPI flash read
        if (n < 16) return;
        uint32_t addr = d[11] | (d[12] << 8) | (d[13] << 16) | ((uint32_t)d[14] << 24);
        uint8_t size = d[15];
        if (size > 64 - 20) size = 64 - 20;
        r[13] = 0x90;
        memcpy(&r[15], &d[11], 5);
        spi_read(&r[20], addr, size);
        break;
    }
    case 0x30: break;                      // player lights
    case 0x31: r[13] = 0xB0; r[15] = 0x01; break;
    case 0x40: break;                      // IMU on/off
    case 0x41: break;                      // IMU sensitivity
    case 0x43: r[13] = 0xC0; r[15] = d[11]; r[16] = n > 12 ? d[12] : 0; break;
    case 0x48: break;                      // vibration on/off
    case 0x50: r[13] = 0xD0; r[15] = 0x83; r[16] = 0x06; break;  // battery voltage
    default:   r[15] = 0x03; break;
    }
    queue_reply(r);
}

// HD rumble from the PC; logged (at most every 2 s) when it isn't silence, so a
// report shows whether the host sent any.
static void host_rumble(const uint8_t *buf)
{
    static const uint8_t neutral[4] = { 0x00, 0x01, 0x40, 0x40 };
    static int64_t last_log;
    bool silent = memcmp(&buf[2], neutral, 4) == 0 && memcmp(&buf[6], neutral, 4) == 0;
    int64_t now = esp_timer_get_time();
    if (!silent && now - last_log > 2000000) {
        last_log = now;
        ESP_LOGI(TAG, "rumble from the PC: %02x %02x %02x %02x / %02x %02x %02x %02x", buf[2], buf[3], buf[4],
                 buf[5], buf[6], buf[7], buf[8], buf[9]);
    }
    rumble_set_hd(&buf[2], &buf[6]);
}

static void set_report(uint8_t id, hid_report_type_t type, const uint8_t *buf, uint16_t len)
{
    (void)type;
    if (id != 0) {
        // Control-pipe SET_REPORT: TinyUSB strips the report ID; put it back.
        // (Interrupt OUT data arrives with id 0 and the ID as the first byte.)
        static uint8_t tmp[64];
        if (len > 63) len = 63;
        tmp[0] = id;
        memcpy(tmp + 1, buf, len);
        buf = tmp;
        len++;
    }
    if (len < 2) {
        return;
    }
    uint8_t r[64];
    switch (buf[0]) {
    case 0x80:
        switch (buf[1]) {
        case 0x01: usb_gamepad_note_stage(USB_STAGE_HELLO); identify_reply(r); queue_reply(r); break;
        case 0x02:
        case 0x03:
            memset(r, 0, sizeof(r));
            r[0] = 0x81;
            r[1] = buf[1];
            queue_reply(r);
            break;
        case 0x04:  // stop USB timeout: start streaming full reports
            if (!s_ready) ESP_LOGI(TAG, "host handshake done, streaming input");
            s_ready = true;
            usb_gamepad_note_stage(USB_STAGE_STREAMING);
            break;
        case 0x05:
            s_ready = false;
            break;
        default:
            break;
        }
        break;
    case 0x01:  // rumble + subcommand
        if (len >= 10) host_rumble(buf);
        handle_subcommand(buf, len);
        break;
    case 0x10:  // rumble only
        if (len >= 10) host_rumble(buf);
        break;
    default:
        break;
    }
}

static void reset(void)
{
    portENTER_CRITICAL(&s_q_mux);
    s_q_head = s_q_count = 0;
    portEXIT_CRITICAL(&s_q_mux);
    s_ready = false;
    s_sent_hello = false;
    s_timer = 0;
}

const out_driver_t out_switchpro = {
    .name = "Switch Pro Controller",
    .vid = 0x057E, .pid = 0x2009, .bcd_device = 0x0210,
    .manufacturer = "Nintendo Co., Ltd.",
    .product = "Pro Controller",
    .serial = "000000000001",
    .itf_desc = itf_desc, .itf_desc_len = sizeof(itf_desc), .itf_count = 1,
    .hid_report_desc = report_desc,
    .send = send,
    .reset = reset,
    .period_ms = 8,
    .set_report = set_report,
};
