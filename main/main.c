// Joy-Con 2 Dongle — step 1 firmware.
//
// Connects Switch 2 Joy-Cons / Pro Controller 2 over BLE and exposes them to
// the host as ONE USB gamepad (a left + right Joy-Con merge into a single
// controller). A second USB interface is a serial port carrying the log.
//
// BOOT button (GPIO0): hold for 5 seconds while running to forget every paired
// controller.
//
// Based on Switch2Connect by TommyWabg (GPL-3.0).
// SPDX-License-Identifier: GPL-3.0-or-later

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "ble_central.h"
#include "bonds.h"
#include "diag.h"
#include "pad.h"
#include "usb_gamepad.h"

static const char *TAG = "main";

#define FW_VERSION       "0.1.1-step1"
#define BOOT_GPIO        GPIO_NUM_0
#define FORGET_HOLD_US   5000000

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
            pressed_since = 0;
            fired = false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    bool safe_mode = diag_boot();
    init_nvs();
    pad_init(usb_gamepad_poke);
    usb_gamepad_start();   // first, so the log mirror catches everything after it

    ESP_LOGI(TAG, "Joy-Con 2 Dongle %s", FW_VERSION);
    diag_start();
    bonds_init();
    if (safe_mode) {
        ESP_LOGW(TAG, "SAFE MODE: crashed several times in a row, Bluetooth not started. "
                      "Save the log and send it; replug to try normal mode again.");
    } else {
        ble_central_start();
    }

    xTaskCreatePinnedToCore(button_task, "button", 3072, NULL, 3, NULL, 1);
    ESP_LOGI(TAG, "ready: press a button on a paired controller, or hold SYNC to pair a new one");
}
