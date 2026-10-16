// On-device display for boards that have one:
// * LILYGO T-Dongle-S3 (DONGLE_BOARD_TDONGLE): 0.96" ST7735 IPS, 80x160, plus
//   one APA102 RGB LED. Pins: LCD MOSI 3, SCLK 5, CS 4, DC 2, RST 1,
//   backlight 38 (active low); LED data 40, clock 39.
// * Waveshare ESP32-S3-LCD-1.47 (DONGLE_BOARD_LCD147): 1.47" ST7789 IPS,
//   172x320, plus one WS2812 RGB LED. Pins: LCD MOSI 45, SCLK 40, CS 42,
//   DC 41, RST 39, backlight 48 (active high); LED 38.
// Shows the output mode, the connected controllers with their battery, and a
// status line. The layout is drawn for the 80x160 screen and scaled by 2 on
// the 172x320 one (with fonts made at twice the size). The picture is drawn
// and sent in strips, so a full 172x320 frame never has to fit in RAM.
// Elsewhere display_start() does nothing.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include "display.h"

#if DISPLAY_PRESENT

#include <stdio.h>
#include <string.h>

#include "ble_central.h"
#include "display_font.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "settings.h"
#include "usb_gamepad.h"
#ifdef DONGLE_BOARD_LCD147
#include "driver/rmt_tx.h"
#endif

static const char *TAG = "display";

#ifdef DONGLE_BOARD_TDONGLE
#define BOARD_NAME "T-Dongle-S3 (160x80 ST7735)"
#define PIN_MOSI 3
#define PIN_SCLK 5
#define PIN_CS   4
#define PIN_DC   2
#define PIN_RST  1
#define PIN_BL   38
#define BL_INVERT 1
#define PIN_LED_DI 40
#define PIN_LED_CI 39
// The 80x160 glass sits inside the controller's 132x162 memory: 26 columns
// and 1 row in. Landscape (rotations 0/1) swaps the axes.
#define LONG  160
#define SHORT 80
#define LAND_X_OFF 1
#define LAND_Y_OFF 26
#define PORT_X_OFF 26
#define PORT_Y_OFF 1
#define SCALE 1
// MADCTL per rotation (landscape, landscape turned, upright, upright turned): 0x08 = BGR.
static const uint8_t MADCTL[4] = { 0xA8, 0x68, 0x08, 0xC8 };
#define FONT_BIG   font_big
#define FONT_SMALL font_small
#else  // DONGLE_BOARD_LCD147
#define BOARD_NAME "Waveshare ESP32-S3-LCD-1.47 (320x172 ST7789)"
#define PIN_MOSI 45
#define PIN_SCLK 40
#define PIN_CS   42
#define PIN_DC   41
#define PIN_RST  39
#define PIN_BL   48
#define BL_INVERT 0
#define PIN_LED  38
// The 172x320 glass sits in the controller's 240x320 memory, 34 columns in.
#define LONG  320
#define SHORT 172
#define LAND_X_OFF 0
#define LAND_Y_OFF 34
#define PORT_X_OFF 34
#define PORT_Y_OFF 0
#define SCALE 2
// MV|MX, MV|MY, native, MX|MY (RGB order).
static const uint8_t MADCTL[4] = { 0x60, 0xA0, 0x00, 0xC0 };
#define FONT_BIG   font_big2
#define FONT_SMALL font_small2
#endif

#define Z(v) ((v) * SCALE)       // layout units (80x160 screen) -> pixels
#define STRIP_BYTES (160 * 80 * 2)  // one strip: the whole T-Dongle screen, 40 rows of the 1.47"

static int W = LONG, H = SHORT;
static int X_OFF = LAND_X_OFF, Y_OFF = LAND_Y_OFF;
static int s_y0, s_rows;         // the strip being drawn: rows s_y0 .. s_y0 + s_rows - 1
struct view;

static esp_lcd_panel_io_handle_t s_io;
static uint16_t *s_fb;              // one strip, RGB565, bytes swapped for the panel
static SemaphoreHandle_t s_done;    // last frame finished sending

static inline uint16_t rgb(uint32_t c)
{
    uint16_t v = ((c >> 8) & 0xF800) | ((c >> 5) & 0x07E0) | ((c & 0xFF) >> 3);
    return (uint16_t)((v >> 8) | (v << 8));
}

static inline uint32_t unrgb(uint16_t p)
{
    uint16_t v = (uint16_t)((p >> 8) | (p << 8));
    uint32_t r = (v >> 11) & 0x1F, g = (v >> 5) & 0x3F, b = v & 0x1F;
    return ((r * 255 / 31) << 16) | ((g * 255 / 63) << 8) | (b * 255 / 31);
}

static void fill(int x, int y, int w, int h, uint32_t c)
{
    uint16_t p = rgb(c);
    int j0 = y < s_y0 ? s_y0 : y, j1 = y + h < s_y0 + s_rows ? y + h : s_y0 + s_rows;
    int i0 = x < 0 ? 0 : x, i1 = x + w < W ? x + w : W;
    for (int j = j0; j < j1; j++) {
        uint16_t *row = s_fb + (j - s_y0) * W;
        for (int i = i0; i < i1; i++) row[i] = p;
    }
}

static void blend(int x, int y, uint32_t c, int a /* 0..15 */)
{
    if (x < 0 || x >= W || y < s_y0 || y >= s_y0 + s_rows || a == 0) return;
    uint16_t *px = &s_fb[(y - s_y0) * W + x];
    uint32_t b = unrgb(*px);
    uint32_t out = 0;
    for (int sh = 0; sh <= 16; sh += 8) {
        int fc = (c >> sh) & 0xFF, bc = (b >> sh) & 0xFF;
        out |= (uint32_t)((fc * a + bc * (15 - a)) / 15) << sh;
    }
    *px = rgb(out);
}

// Draws text with its top at y; returns the x after it.
static int text(const font_t *f, int x, int y, const char *s, uint32_t c)
{
    for (; *s; s++) {
        int ch = (unsigned char)*s;
        if (ch < 32 || ch > 126) ch = '?';
        const glyph_t *g = &f->g[ch - 32];
        const uint8_t *bits = f->bits + g->offset;
        if (y + f->height <= s_y0 || y >= s_y0 + s_rows) { x += g->adv; continue; }
        for (int j = 0; j < f->height; j++) {
            for (int i = 0; i < g->w; i++) {
                int n = j * g->w + i;
                int a = (n & 1) ? (bits[n >> 1] & 0x0F) : (bits[n >> 1] >> 4);
                blend(x + g->x0 + i, y + j, c, a);
            }
        }
        x += g->adv;
    }
    return x;
}

static int text_width(const font_t *f, const char *s)
{
    int w = 0;
    for (; *s; s++) {
        int ch = (unsigned char)*s;
        if (ch < 32 || ch > 126) ch = '?';
        w += f->g[ch - 32].adv;
    }
    return w;
}

static void cmd(uint8_t c, const uint8_t *d, size_t n)
{
    esp_lcd_panel_io_tx_param(s_io, c, d, n);
}

static bool IRAM_ATTR on_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *e, void *ctx)
{
    BaseType_t woke = pdFALSE;
    xSemaphoreGiveFromISR(s_done, &woke);
    return woke == pdTRUE;
}

// Rotation 0/1: landscape, either way up. 2/3: portrait (stick upright),
// either way up.
static void set_orientation(uint8_t rot)
{
    rot &= 3;
    cmd(0x36, &MADCTL[rot], 1);
    bool land = rot < 2;
    W = land ? LONG : SHORT;
    H = land ? SHORT : LONG;
    X_OFF = land ? LAND_X_OFF : PORT_X_OFF;
    Y_OFF = land ? LAND_Y_OFF : PORT_Y_OFF;
}

static void panel_reset(void)
{
    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    cmd(0x01, NULL, 0);                                 // SWRESET
    vTaskDelay(pdMS_TO_TICKS(150));
    cmd(0x11, NULL, 0);                                 // SLPOUT
    vTaskDelay(pdMS_TO_TICKS(150));
}

#ifdef DONGLE_BOARD_TDONGLE
// ST7735S init (as Adafruit's "mini 160x80" / LILYGO's examples).
static void panel_init(void)
{
    panel_reset();
    cmd(0xB1, (const uint8_t[]){ 0x01, 0x2C, 0x2D }, 3);  // frame rate
    cmd(0xB2, (const uint8_t[]){ 0x01, 0x2C, 0x2D }, 3);
    cmd(0xB3, (const uint8_t[]){ 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D }, 6);
    cmd(0xB4, (const uint8_t[]){ 0x07 }, 1);              // column inversion
    cmd(0xC0, (const uint8_t[]){ 0xA2, 0x02, 0x84 }, 3);  // power
    cmd(0xC1, (const uint8_t[]){ 0xC5 }, 1);
    cmd(0xC2, (const uint8_t[]){ 0x0A, 0x00 }, 2);
    cmd(0xC3, (const uint8_t[]){ 0x8A, 0x2A }, 2);
    cmd(0xC4, (const uint8_t[]){ 0x8A, 0xEE }, 2);
    cmd(0xC5, (const uint8_t[]){ 0x0E }, 1);
    cmd(0x21, NULL, 0);                                 // INVON (IPS glass)
    set_orientation(settings_get().display_rot);
    cmd(0x3A, (const uint8_t[]){ 0x05 }, 1);              // 16-bit colour
    cmd(0xE0, (const uint8_t[]){ 0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d,
                                 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10 }, 16);
    cmd(0xE1, (const uint8_t[]){ 0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
                                 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10 }, 16);
    cmd(0x13, NULL, 0);                                 // NORON
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd(0x29, NULL, 0);                                 // DISPON
}

#else
// ST7789 init for the 1.47" 172x320 IPS (as in Waveshare's examples).
static void panel_init(void)
{
    panel_reset();
    set_orientation(settings_get().display_rot);
    cmd(0x3A, (const uint8_t[]){ 0x05 }, 1);                    // 16-bit colour
    cmd(0xB2, (const uint8_t[]){ 0x0C, 0x0C, 0x00, 0x33, 0x33 }, 5);  // porch
    cmd(0xB7, (const uint8_t[]){ 0x35 }, 1);                    // gate control
    cmd(0xBB, (const uint8_t[]){ 0x35 }, 1);                    // VCOM
    cmd(0xC0, (const uint8_t[]){ 0x2C }, 1);
    cmd(0xC2, (const uint8_t[]){ 0x01 }, 1);
    cmd(0xC3, (const uint8_t[]){ 0x13 }, 1);
    cmd(0xC4, (const uint8_t[]){ 0x20 }, 1);
    cmd(0xC6, (const uint8_t[]){ 0x0F }, 1);                    // 60 Hz
    cmd(0xD0, (const uint8_t[]){ 0xA4, 0xA1 }, 2);              // power
    cmd(0xE0, (const uint8_t[]){ 0xF0, 0xF0, 0x00, 0x04, 0x04, 0x04, 0x05, 0x29,
                                 0x33, 0x3E, 0x38, 0x12, 0x12, 0x28, 0x30 }, 14);
    cmd(0xE1, (const uint8_t[]){ 0xF0, 0x07, 0x0A, 0x0D, 0x0B, 0x07, 0x28, 0x33,
                                 0x3E, 0x36, 0x14, 0x14, 0x29, 0x32 }, 14);
    cmd(0x21, NULL, 0);                                 // INVON (IPS glass)
    cmd(0x13, NULL, 0);                                 // NORON
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd(0x29, NULL, 0);                                 // DISPON
}
#endif

static void render(const struct view *v);

// Draws the frame strip by strip (each strip drawn in full, clipped) and
// sends each strip to its rows on the panel.
static void flush(const struct view *v)
{
    int rows = STRIP_BYTES / 2 / W;
    for (int y0 = 0; y0 < H; y0 += rows) {
        s_y0 = y0;
        s_rows = y0 + rows <= H ? rows : H - y0;
        render(v);
        uint16_t x1 = X_OFF, x2 = X_OFF + W - 1, y1 = Y_OFF + y0, y2 = Y_OFF + y0 + s_rows - 1;
        uint8_t ca[4] = { x1 >> 8, x1 & 0xFF, x2 >> 8, x2 & 0xFF };
        uint8_t ra[4] = { y1 >> 8, y1 & 0xFF, y2 >> 8, y2 & 0xFF };
        cmd(0x2A, ca, 4);
        cmd(0x2B, ra, 4);
        esp_lcd_panel_io_tx_color(s_io, 0x2C, s_fb, W * s_rows * 2);
        xSemaphoreTake(s_done, pdMS_TO_TICKS(200));  // the strip buffer is ours again
    }
}

static void backlight(uint8_t level /* 0..10 */)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, level * 255 / 10);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

#ifdef DONGLE_BOARD_TDONGLE
static void led_init(void)
{
    gpio_reset_pin(PIN_LED_DI);
    gpio_reset_pin(PIN_LED_CI);
    gpio_set_direction(PIN_LED_DI, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_CI, GPIO_MODE_OUTPUT);
}

// APA102: start frame, one LED frame (brightness, B, G, R), end frame.
static void led_set(uint32_t c, uint8_t bright /* 0..31 */)
{
    uint8_t frame[12] = { 0, 0, 0, 0, (uint8_t)(0xE0 | bright), c & 0xFF, (c >> 8) & 0xFF, (c >> 16) & 0xFF,
                          0xFF, 0xFF, 0xFF, 0xFF };
    for (int i = 0; i < 12; i++) {
        for (int b = 7; b >= 0; b--) {
            gpio_set_level(PIN_LED_DI, (frame[i] >> b) & 1);
            gpio_set_level(PIN_LED_CI, 1);
            gpio_set_level(PIN_LED_CI, 0);
        }
    }
}
#else
static rmt_channel_handle_t s_rmt;
static rmt_encoder_handle_t s_enc;

static void led_init(void)
{
    rmt_tx_channel_config_t c = { .gpio_num = PIN_LED, .clk_src = RMT_CLK_SRC_DEFAULT, .resolution_hz = 10000000,
                                  .mem_block_symbols = 48, .trans_queue_depth = 2 };
    if (rmt_new_tx_channel(&c, &s_rmt) != ESP_OK) { s_rmt = NULL; return; }
    rmt_bytes_encoder_config_t e = {  // WS2812 timing at 0.1 us per tick
        .bit0 = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 },
        .bit1 = { .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3 },
        .flags.msb_first = 1,
    };
    if (rmt_new_bytes_encoder(&e, &s_enc) != ESP_OK) { s_rmt = NULL; return; }
    rmt_enable(s_rmt);
}

// WS2812: green, red, blue; brightness 0..31 scales the colour.
static void led_set(uint32_t c, uint8_t bright /* 0..31 */)
{
    if (s_rmt == NULL) return;
    uint8_t grb[3] = { ((c >> 8) & 0xFF) * bright / 31, ((c >> 16) & 0xFF) * bright / 31, (c & 0xFF) * bright / 31 };
    rmt_transmit_config_t t = { 0 };
    rmt_transmit(s_rmt, s_enc, grb, sizeof(grb), &t);
    rmt_tx_wait_all_done(s_rmt, 100);
}
#endif

typedef struct view {
    int mode;           // out_mode_t, or -1 for the configurator
    bool mounted;
    uint8_t usb_stage;   // usb_stage_t
    bool l, r, pro;     // ready controllers
    bool pad;           // the full controller is a BLE gamepad
    bool pairing;       // a controller is connecting
    uint8_t bat_l, bat_r, bat_pro;  // 0..4 bars, 255 = unknown
    bool dock;
    uint8_t mouse;
    uint8_t bright;
    uint8_t rot;
} view_t;


static uint8_t bars(uint16_t mv)
{
    if (mv < 2500 || mv > 4600) return 255;  // not a plausible reading
    // Joy-Con 2 reports a lower value than its cell voltage. Calibrated
    // against the Switch 2's own gauge: 3262 = full, 3176 = a bit over half.
    if (mv >= 3240) return 4;
    if (mv >= 3170) return 3;
    if (mv >= 3100) return 2;
    if (mv >= 3000) return 1;
    return 0;
}

static void collect(view_t *v)
{
    memset(v, 0, sizeof(*v));
    settings_t st = settings_get();
    v->mode = usb_gamepad_in_config() ? -1 : (int)st.out_mode;
    v->mounted = usb_gamepad_mounted();
    v->usb_stage = (uint8_t)usb_gamepad_stage();
    v->mouse = st.pad_mouse;
    v->bright = st.display_bright;
    v->rot = st.display_rot & 3;
    v->bat_l = v->bat_r = v->bat_pro = 255;
    ble_link_info_t links[4];
    int n = ble_central_links(links, 4);
    for (int i = 0; i < n; i++) {
        if (!links[i].ready) {
            v->pairing = true;
            continue;
        }
        if (links[i].in_dock) v->dock = true;
        uint8_t b = bars(links[i].battery_mv);
        switch (links[i].role) {
        case PAD_ROLE_LEFT:  v->l = true; v->bat_l = b; break;
        case PAD_ROLE_RIGHT: v->r = true; v->bat_r = b; break;
        case PAD_ROLE_FULL:  v->pro = true; v->bat_pro = b; v->pad = links[i].hid; break;
        default: break;
        }
    }
}

static const char *mode_title(int m)
{
    switch (m) {
    case -1:                  return "Configurator";
    case OUT_MODE_XINPUT:     return "XInput";
    case OUT_MODE_DS4:        return "DualShock 4";
    case OUT_MODE_DS5:        return "DualSense";
    case OUT_MODE_SWITCH_PRO: return "Switch Pro";
    case OUT_MODE_STEAM:      return "Steam Ctrl";
    case OUT_MODE_HID:        return "HID gamepad";
    default:                  return "?";
    }
}

static uint32_t mode_color(int m)
{
    switch (m) {
    case -1:                  return 0xF0A84A;
    case OUT_MODE_XINPUT:     return 0x3CC24A;
    case OUT_MODE_DS4:        return 0x2E6BD6;
    case OUT_MODE_DS5:        return 0x8FB8FF;
    case OUT_MODE_SWITCH_PRO: return 0xE60012;
    case OUT_MODE_STEAM:      return 0x66C0F4;
    default:                  return 0xA0A8B4;
    }
}

#define BG    0x101318
#define CHIP  0x262C36
#define INK   0xE8EBF0
#define MUTED 0x8A93A0
#define DIM   0x3A424E

// One controller chip: label and a 4-bar battery. Layout units (see Z).
static void chip(int x, int y, int w, const char *label, bool on, uint8_t bat)
{
    x = Z(x); y = Z(y); w = Z(w);
    int h = Z(20), t = Z(1);
    fill(x, y, w, h, on ? CHIP : BG);
    if (!on) {  // outline only
        fill(x, y, w, t, DIM); fill(x, y + h - t, w, t, DIM);
        fill(x, y, t, h, DIM); fill(x + w - t, y, t, h, DIM);
    }
    text(&FONT_SMALL, x + Z(5), y + Z(3), label, on ? INK : MUTED);
    if (!on) return;
    int bx = x + w - Z(19), by = y + Z(6);
    fill(bx, by, Z(14), Z(8), MUTED);           // battery outline
    fill(bx + t, by + t, Z(12), Z(6), CHIP);
    fill(bx + Z(14), by + Z(2), Z(2), Z(4), MUTED);   // tip
    if (bat == 255) {
        text(&FONT_SMALL, bx + Z(4), y + Z(3), "?", MUTED);
        return;
    }
    uint32_t c = bat <= 1 ? 0xFF6B62 : 0x4CC38A;
    for (int i = 0; i < bat; i++) fill(bx + Z(2 + i * 3), by + Z(2), Z(2), Z(4), c);
}

static uint32_t status_line(const view_t *v, char *line, size_t n)
{
    uint32_t lc = MUTED;
    if (v->mode == -1) {
        snprintf(line, n, "Open 192.168.7.1");
        lc = mode_color(v->mode);
    } else if (!v->mounted) {
        // How far the host got (useful on consoles, where there's no log).
        static const char *const st[] = { "Waiting for the PC...", "USB: device read", "USB: config read",
                                          "USB: HID read", "USB: ready" };
        snprintf(line, n, "%s", st[v->usb_stage < 5 ? v->usb_stage : 0]);
    } else if (v->mode == OUT_MODE_SWITCH_PRO && v->usb_stage < USB_STAGE_STREAMING) {
        snprintf(line, n, v->usb_stage < USB_STAGE_HELLO ? "Switch: no hello yet" : "Switch: handshake...");
    } else if (v->pairing) {
        snprintf(line, n, "Connecting...");
    } else if (v->dock) {
        snprintf(line, n, "In the charging grip");
    } else if (v->mouse == PAD_MOUSE_FULL) {
        snprintf(line, n, "USB mouse on");
    } else if (v->mouse == PAD_MOUSE_POINTER) {
        snprintf(line, n, "USB mouse pointer on");
    } else {
        snprintf(line, n, "Ready");
    }
    return lc;
}

// Word-wraps s into lines of at most `width` pixels; returns the y after it.
static int text_wrap(const font_t *f, int x, int y, int width, const char *s, uint32_t c)
{
    char buf[48];
    while (*s) {
        int n = 0, fit = 0;
        while (s[n]) {
            int end = n;
            while (s[end] && s[end] != ' ') end++;
            snprintf(buf, sizeof(buf), "%.*s", end, s);
            if (text_width(f, buf) > width && fit > 0) break;
            fit = end;
            n = s[end] ? end + 1 : end;
        }
        if (fit == 0) fit = n;
        snprintf(buf, sizeof(buf), "%.*s", fit, s);
        text(f, x, y, buf, c);
        y += f->height;
        s += fit;
        while (*s == ' ') s++;
    }
    return y;
}

static void version_text(char *vs, size_t n)
{
    const char *ver = usb_gamepad_firmware_version();
    snprintf(vs, n, "v%.*s", (int)strcspn(ver, "-"), ver);
}

// Portrait: 80 wide, 160 tall (layout units; Z scales them to pixels).
static void render_portrait(const view_t *v)
{
    int w = W / SCALE;  // width in layout units
    fill(0, 0, W, H, BG);
    uint32_t mc = mode_color(v->mode);
    fill(0, 0, W, Z(4), mc);
    const char *t = mode_title(v->mode);
    switch (v->mode) {  // short names fit the narrow screen in the big font
    case -1:                  t = "Setup"; break;
    case OUT_MODE_DS4:        t = "DS4"; break;
    case OUT_MODE_DS5:        t = "DS5"; break;
    case OUT_MODE_SWITCH_PRO: t = "Switch"; break;
    case OUT_MODE_STEAM:      t = "Steam"; break;
    case OUT_MODE_HID:        t = "HID"; break;
    default: break;
    }
    const font_t *tf = text_width(&FONT_BIG, t) <= W - Z(8) ? &FONT_BIG : &FONT_SMALL;
    text(tf, Z(4), Z(9), t, INK);
    if (v->pro) {
        chip(4, 36, w - 8, v->pad ? "Pad" : "Pro", true, v->bat_pro);
    } else if (v->l || v->r || v->pairing) {
        chip(4, 36, w - 8, "Left", v->l, v->bat_l);
        chip(4, 60, w - 8, "Right", v->r, v->bat_r);
    }
    char line[40];
    uint32_t lc = status_line(v, line, sizeof(line));
    if (!v->pro && !v->l && !v->r && !v->pairing) {
        int y = text_wrap(&FONT_SMALL, Z(4), Z(36), W - Z(8), "No controller: hold SYNC to pair", MUTED);
        text_wrap(&FONT_SMALL, Z(4), y + Z(8), W - Z(8), line, lc);
    } else {
        text_wrap(&FONT_SMALL, Z(4), Z(92), W - Z(8), line, lc);
    }
    char vs[16];
    version_text(vs, sizeof(vs));
    text(&FONT_SMALL, Z(4), H - Z(16), vs, DIM);
}

// Landscape: 160 wide, 80 tall (layout units).
static void render(const view_t *v)
{
    if (W < H) {
        render_portrait(v);
        return;
    }
    fill(0, 0, W, H, BG);
    uint32_t mc = mode_color(v->mode);
    fill(0, 0, Z(4), H, mc);
    text(&FONT_BIG, Z(10), Z(1), mode_title(v->mode), INK);

    if (v->pro) {
        chip(10, 30, 70, v->pad ? "Pad" : "Pro", true, v->bat_pro);
    } else if (v->l || v->r || v->pairing) {
        chip(10, 30, 66, "Left", v->l, v->bat_l);
        chip(84, 30, 66, "Right", v->r, v->bat_r);
    } else {
        text(&FONT_SMALL, Z(10), Z(33), "No controller:", MUTED);
        text(&FONT_SMALL, Z(10), Z(46), "hold SYNC to pair", MUTED);
    }

    char line[40];
    uint32_t lc = status_line(v, line, sizeof(line));
    // Status line at the bottom (the 1.47" screen is a little taller than 2x).
    int ly = H - Z(17);
    text(&FONT_SMALL, Z(10), ly, line, lc);
    char vs[16];
    version_text(vs, sizeof(vs));
    text(&FONT_SMALL, W - Z(4) - text_width(&FONT_SMALL, vs), ly, vs, DIM);
}

static void display_task(void *arg)
{
    (void)arg;
    view_t last, now;
    memset(&last, 0xFF, sizeof(last));
    for (;;) {
        collect(&now);
        if (memcmp(&now, &last, sizeof(now)) != 0) {
            if (now.rot != last.rot) set_orientation(now.rot);
            if (now.bright != last.bright) backlight(now.bright);
            if (now.mode != last.mode) led_set(mode_color(now.mode), 2);
            flush(&now);
            last = now;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void display_start(void)
{
    led_init();
    led_set(0, 0);

    // Backlight: PWM (T-Dongle: active low, inverted output), off until the first frame.
    ledc_timer_config_t t = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_8_BIT,
                              .timer_num = LEDC_TIMER_0, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    ledc_channel_config_t ch = { .gpio_num = PIN_BL, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
                                 .timer_sel = LEDC_TIMER_0, .duty = 0, .flags.output_invert = BL_INVERT };
    ledc_channel_config(&ch);

    spi_bus_config_t bus = { .mosi_io_num = PIN_MOSI, .miso_io_num = -1, .sclk_io_num = PIN_SCLK,
                             .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = STRIP_BYTES + 16 };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus failed: %s", esp_err_to_name(err));
        return;
    }
    s_done = xSemaphoreCreateBinary();
    esp_lcd_panel_io_spi_config_t io = {
        .dc_gpio_num = PIN_DC, .cs_gpio_num = PIN_CS, .pclk_hz = (SCALE > 1 ? 40 : 20) * 1000 * 1000,
        .lcd_cmd_bits = 8, .lcd_param_bits = 8, .spi_mode = 0, .trans_queue_depth = 4,
        .on_color_trans_done = on_done,
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io, &s_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LCD I/O failed: %s", esp_err_to_name(err));
        return;
    }
    s_fb = heap_caps_malloc(STRIP_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (s_fb == NULL) {
        ESP_LOGE(TAG, "no memory for the frame buffer");
        return;
    }
    panel_init();
    ESP_LOGI(TAG, "%s display ready", BOARD_NAME);
    xTaskCreatePinnedToCore(display_task, "display", 4096, NULL, 2, NULL, 0);
}

#else

void display_start(void) {}

#endif
