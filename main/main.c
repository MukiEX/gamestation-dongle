// Joy-Con 2 Dongle firmware.
//
// Connects Switch 2 Joy-Cons / Pro Controller 2 over BLE and exposes them to
// the host as ONE USB controller (a left + right Joy-Con merge into a single
// controller) in the selected output mode: XInput, DualShock 4, Switch Pro or
// generic HID, with rumble. A second USB interface is a serial port carrying
// the log.
//
// BOOT button (GPIO0): short press = next output mode (USB re-enumerates,
// controllers stay connected); hold 2-5 s and release = web configurator at
// http://192.168.7.1 (short press again to leave);
// hold for 5 seconds = forget every paired controller.
//
// Based on Switch2Connect by TommyWabg (GPL-3.0).
// SPDX-License-Identifier: GPL-3.0-or-later

#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "ble_central.h"
#include "bonds.h"
#include "diag.h"
#include "pad.h"
#include "rumble.h"
#include "settings.h"
#include "display.h"
#include "usb_gamepad.h"

static const char *TAG = "main";

#define FW_BASE          "1.2.0"
#if defined(DONGLE_DIAG)
#define FW_VERSION       FW_BASE "-diag"
#elif defined(DONGLE_BOARD_TDONGLE)
#define FW_VERSION       FW_BASE "-tdongle"
#else
#define FW_VERSION       FW_BASE
#endif
#define BOOT_GPIO        GPIO_NUM_0
#define FORGET_HOLD_US   5000000
#define SHORT_PRESS_US   1000000
#define CONFIG_HOLD_US   2000000

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void button_task(void *arg)
{
    (void)arg;
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BOOT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);

    int64_t pressed_since = 0;
    bool fired = false;
    for (;;) {
        bool pressed = gpio_get_level(BOOT_GPIO) == 0;
        int64_t now = esp_timer_get_time();
        if (pressed) {
            if (pressed_since == 0) {
                pressed_since = now;
            } else if (!fired && now - pressed_since >= FORGET_HOLD_US) {
                fired = true;
                ESP_LOGW(TAG, "BOOT held 5 s: forgetting all paired controllers");
                ble_central_forget_all();
                diag_clear_crash();
            }
        } else {
            int64_t held = pressed_since ? now - pressed_since : 0;
            if (!fired && held >= 50000 && held < SHORT_PRESS_US) {
                usb_gamepad_next_mode();     // in the configurator: leave it
            } else if (!fired && held >= CONFIG_HOLD_US) {
                usb_gamepad_enter_config();  // released between 2 and 5 s
            }
            pressed_since = 0;
            fired = false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

#ifdef DONGLE_DIAG
// Diagnostic build: proves the firmware is alive and shows memory over time.
static void heartbeat_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "alive: uptime %lld s, free heap %u bytes (min %u)",
                 esp_timer_get_time() / 1000000, (unsigned)esp_get_free_heap_size(),
                 (unsigned)esp_get_minimum_free_heap_size());
    }
}
#endif

void app_main(void)
{
    bool safe_mode = diag_boot();
    ESP_LOGI(TAG, "Joy-Con 2 Dongle %s starting", FW_VERSION);
    ESP_LOGI(TAG, "stage 1/5: storage");
    init_nvs();
    settings_init();
    rumble_init();
    pad_init(usb_gamepad_poke);
    usb_gamepad_set_firmware_version(FW_VERSION);
#ifdef DONGLE_DIAG
    ESP_LOGW(TAG, "stage 2/5: DIAGNOSTIC BUILD - USB gamepad disabled, log on the built-in USB port");
#else
    ESP_LOGI(TAG, "stage 2/5: USB");
    usb_gamepad_start();   // the log mirror catches everything after this
#endif

    ESP_LOGI(TAG, "Joy-Con 2 Dongle %s", FW_VERSION);
    ESP_LOGI(TAG, "stage 3/5: diagnostics");
    diag_start();
    bonds_init();
    if (safe_mode) {
        ESP_LOGW(TAG, "SAFE MODE: crashed several times in a row, Bluetooth not started. "
                      "Save the log and send it; replug to try normal mode again.");
    } else {
        ESP_LOGI(TAG, "stage 4/5: Bluetooth");
        ble_central_start();
    }

    display_start();       // boards with a display only
    ESP_LOGI(TAG, "stage 5/5: button");
    xTaskCreatePinnedToCore(button_task, "button", 3072, NULL, 3, NULL, 1);
#ifdef DONGLE_DIAG
    xTaskCreatePinnedToCore(heartbeat_task, "heartbeat", 3072, NULL, 1, NULL, 1);
#endif
    ESP_LOGI(TAG, "ready: press a button on a paired controller, or hold SYNC to pair a new one");
}
