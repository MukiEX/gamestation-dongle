// On-device display for boards that have one. Currently the LILYGO
// T-Dongle-S3: 0.96" ST7735 IPS, 80x160 (used as 160x80 landscape) on SPI,
// plus one APA102 RGB LED. Shows the output mode, the connected controllers
// with their battery, and a status line. Built only into the T-Dongle-S3
// image (DONGLE_BOARD_TDONGLE); elsewhere display_start() does nothing.
//
// T-Dongle-S3 pins (LILYGO): LCD MOSI 3, SCLK 5, CS 4, DC 2, RST 1,
// backlight 38 (active low); LED data 40, clock 39.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include "display.h"

#ifdef DONGLE_BOARD_TDONGLE

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

static const char *TAG = "display";

#define PIN_MOSI 3
#define PIN_SCLK 5
#define PIN_CS   4
#define PIN_DC   2
#define PIN_RST  1
#define PIN_BL   38
#define PIN_LED_DI 40
#define PIN_LED_CI 39

// The 80x160 glass sits inside the controller's 132x162 memory: 26 columns
// and 1 row in. Landscape (rotations 0/1) swaps the axes.
#define LONG  160
#define SHORT 80
static int W = LONG, H = SHORT;
static int X_OFF = 1, Y_OFF = 26;

static esp_lcd_panel_io_handle_t s_io;
static uint16_t *s_fb;              // RGB565, bytes swapped for the panel
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
    for (int j = y; j < y + h; j++) {
        if (j < 0 || j >= H) continue;
        for (int i = x; i < x + w; i++) {
            if (i >= 0 && i < W) s_fb[j * W + i] = p;
        }
    }
}

static void blend(int x, int y, uint32_t c, int a /* 0..15 */)
{
    if (x < 0 || x >= W || y < 0 || y >= H || a == 0) return;
    uint32_t b = unrgb(s_fb[y * W + x]);
    uint32_t out = 0;
    for (int sh = 0; sh <= 16; sh += 8) {
        int fc = (c >> sh) & 0xFF, bc = (b >> sh) & 0xFF;
        out |= (uint32_t)((fc * a + bc * (15 - a)) / 15) << sh;
    }
    s_fb[y * W + x] = rgb(out);
}

// Draws text with its top at y; returns the x after it.
static int text(const font_t *f, int x, int y, const char *s, uint32_t c)
{
    for (; *s; s++) {
        int ch = (unsigned char)*s;
        if (ch < 32 || ch > 126) ch = '?';
        const glyph_t *g = &f->g[ch - 32];
        const uint8_t *bits = f->bits + g->offset;
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
// either way up. MADCTL: MV swaps axes, MX/MY mirror; 0x08 = BGR.
static void set_orientation(uint8_t rot)
{
    static const uint8_t madctl[4] = { 0xA8, 0x68, 0x08, 0xC8 };
    rot &= 3;
    cmd(0x36, &madctl[rot], 1);
    bool land = rot < 2;
    W = land ? LONG : SHORT;
    H = land ? SHORT : LONG;
    X_OFF = land ? 1 : 26;
    Y_OFF = land ? 26 : 1;
}

// ST7735S init (as Adafruit's "mini 160x80" / LILYGO's examples).
static void panel_init(void)
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

static void flush(void)
{
    uint8_t ca[4] = { 0, X_OFF, 0, X_OFF + W - 1 };
    uint8_t ra[4] = { 0, Y_OFF, 0, Y_OFF + H - 1 };
    cmd(0x2A, ca, 4);
    cmd(0x2B, ra, 4);
    esp_lcd_panel_io_tx_color(s_io, 0x2C, s_fb, W * H * 2);
    xSemaphoreTake(s_done, pdMS_TO_TICKS(200));  // the buffer is ours again
}

static void backlight(uint8_t level /* 0..10 */)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, level * 255 / 10);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
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

typedef struct {
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

// One controller chip: label and a 4-bar battery.
static void chip(int x, int y, int w, const char *label, bool on, uint8_t bat)
{
    fill(x, y, w, 20, on ? CHIP : BG);
    if (!on) {  // outline only
        fill(x, y, w, 1, DIM); fill(x, y + 19, w, 1, DIM);
        fill(x, y, 1, 20, DIM); fill(x + w - 1, y, 1, 20, DIM);
    }
    text(&font_small, x + 5, y + 3, label, on ? INK : MUTED);
    if (!on) return;
    int bx = x + w - 19, by = y + 6;
    fill(bx, by, 14, 8, MUTED);           // battery outline
    fill(bx + 1, by + 1, 12, 6, CHIP);
    fill(bx + 14, by + 2, 2, 4, MUTED);   // tip
    if (bat == 255) {
        text(&font_small, bx + 4, y + 3, "?", MUTED);
        return;
    }
    uint32_t c = bat <= 1 ? 0xFF6B62 : 0x4CC38A;
    for (int i = 0; i < bat; i++) fill(bx + 2 + i * 3, by + 2, 2, 4, c);
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

// Portrait: 80 wide, 160 tall.
static void render_portrait(const view_t *v)
{
    fill(0, 0, W, H, BG);
    uint32_t mc = mode_color(v->mode);
    fill(0, 0, W, 4, mc);
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
    const font_t *tf = text_width(&font_big, t) <= W - 8 ? &font_big : &font_small;
    text(tf, 4, 9, t, INK);
    if (v->pro) {
        chip(4, 36, W - 8, v->pad ? "Pad" : "Pro", true, v->bat_pro);
    } else if (v->l || v->r || v->pairing) {
        chip(4, 36, W - 8, "Left", v->l, v->bat_l);
        chip(4, 60, W - 8, "Right", v->r, v->bat_r);
    }
    char line[40];
    uint32_t lc = status_line(v, line, sizeof(line));
    if (!v->pro && !v->l && !v->r && !v->pairing) {
        int y = text_wrap(&font_small, 4, 36, W - 8, "No controller: hold SYNC to pair", MUTED);
        text_wrap(&font_small, 4, y + 8, W - 8, line, lc);
    } else {
        text_wrap(&font_small, 4, 92, W - 8, line, lc);
    }
    char vs[16];
    version_text(vs, sizeof(vs));
    text(&font_small, 4, H - 16, vs, DIM);
}

static void render(const view_t *v)
{
    if (W < H) {
        render_portrait(v);
        return;
    }
    fill(0, 0, W, H, BG);
    uint32_t mc = mode_color(v->mode);
    fill(0, 0, 4, H, mc);
    text(&font_big, 10, 1, mode_title(v->mode), INK);

    if (v->pro) {
        chip(10, 30, 70, v->pad ? "Pad" : "Pro", true, v->bat_pro);
    } else if (v->l || v->r || v->pairing) {
        chip(10, 30, 66, "Left", v->l, v->bat_l);
        chip(84, 30, 66, "Right", v->r, v->bat_r);
    } else {
        text(&font_small, 10, 33, "No controller:", MUTED);
        text(&font_small, 10, 46, "hold SYNC to pair", MUTED);
    }

    char line[40];
    uint32_t lc = status_line(v, line, sizeof(line));
    text(&font_small, 10, 63, line, lc);
    char vs[16];
    version_text(vs, sizeof(vs));
    text(&font_small, W - 4 - text_width(&font_small, vs), 63, vs, DIM);
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
            render(&now);
            flush();
            last = now;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void display_start(void)
{
    gpio_reset_pin(PIN_LED_DI);
    gpio_reset_pin(PIN_LED_CI);
    gpio_set_direction(PIN_LED_DI, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_CI, GPIO_MODE_OUTPUT);
    led_set(0, 0);

    // Backlight: PWM, active low (inverted output), off until the first frame.
    ledc_timer_config_t t = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_8_BIT,
                              .timer_num = LEDC_TIMER_0, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    ledc_channel_config_t ch = { .gpio_num = PIN_BL, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
                                 .timer_sel = LEDC_TIMER_0, .duty = 0, .flags.output_invert = 1 };
    ledc_channel_config(&ch);

    spi_bus_config_t bus = { .mosi_io_num = PIN_MOSI, .miso_io_num = -1, .sclk_io_num = PIN_SCLK,
                             .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = LONG * SHORT * 2 + 16 };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus failed: %s", esp_err_to_name(err));
        return;
    }
    s_done = xSemaphoreCreateBinary();
    esp_lcd_panel_io_spi_config_t io = {
        .dc_gpio_num = PIN_DC, .cs_gpio_num = PIN_CS, .pclk_hz = 20 * 1000 * 1000,
        .lcd_cmd_bits = 8, .lcd_param_bits = 8, .spi_mode = 0, .trans_queue_depth = 4,
        .on_color_trans_done = on_done,
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io, &s_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LCD I/O failed: %s", esp_err_to_name(err));
        return;
    }
    s_fb = heap_caps_malloc(LONG * SHORT * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (s_fb == NULL) {
        ESP_LOGE(TAG, "no memory for the frame buffer");
        return;
    }
    panel_init();
    ESP_LOGI(TAG, "T-Dongle-S3 display ready (160x80 ST7735)");
    xTaskCreatePinnedToCore(display_task, "display", 4096, NULL, 2, NULL, 0);
}

#else

void display_start(void) {}

#endif
