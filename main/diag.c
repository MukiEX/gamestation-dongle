// SPDX-License-Identifier: GPL-3.0-or-later
#include "diag.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_core_dump.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "diag";

#define CRASH_MAGIC       0x4A433244u  // "JC2D"
#define SAFE_MODE_AFTER   3            // consecutive crash resets
#define STABLE_AFTER_US   60000000     // uptime that counts as "not crash-looping"

RTC_NOINIT_ATTR static uint32_t s_magic;
RTC_NOINIT_ATTR static uint32_t s_crash_count;

static esp_reset_reason_t s_reason;
static bool s_safe_mode;
static bool s_have_dump;
static char s_report[640];

static const char *reason_name(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "external reset pin";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "CRASH (panic)";
    case ESP_RST_INT_WDT:   return "CRASH (interrupt watchdog)";
    case ESP_RST_TASK_WDT:  return "CRASH (task watchdog)";
    case ESP_RST_WDT:       return "CRASH (other watchdog)";
    case ESP_RST_DEEPSLEEP: return "wake from deep sleep";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (supply voltage dropped)";
    case ESP_RST_SDIO:      return "SDIO";
    case ESP_RST_USB:       return "USB peripheral reset";
    case ESP_RST_JTAG:      return "JTAG / USB-serial reset";
    default:                return "unknown";
    }
}

static bool is_crash(esp_reset_reason_t r)
{
    return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT ||
           r == ESP_RST_WDT || r == ESP_RST_BROWNOUT;
}

static void build_report(void)
{
    s_report[0] = 0;
    esp_err_t chk = esp_core_dump_image_check();
    if (chk != ESP_OK) {
        snprintf(s_report, sizeof(s_report), "(no stored crash report: %s)", esp_err_to_name(chk));
        return;
    }
    esp_core_dump_summary_t *sum = calloc(1, sizeof(*sum));
    if (sum == NULL) {
        return;
    }
    esp_err_t gs = esp_core_dump_get_summary(sum);
    if (gs != ESP_OK) {
        snprintf(s_report, sizeof(s_report), "(crash report present but unreadable: %s)", esp_err_to_name(gs));
    } else {
        int n = snprintf(s_report, sizeof(s_report),
                         "task=%s pc=0x%08lx cause=%lu vaddr=0x%08lx bt%s:",
                         sum->exc_task, (unsigned long)sum->exc_pc,
                         (unsigned long)sum->ex_info.exc_cause, (unsigned long)sum->ex_info.exc_vaddr,
                         sum->exc_bt_info.corrupted ? "(corrupt)" : "");
        for (uint32_t i = 0; i < sum->exc_bt_info.depth && i < 16 && n > 0 && n < (int)sizeof(s_report) - 12; i++) {
            n += snprintf(s_report + n, sizeof(s_report) - n, " 0x%08lx", (unsigned long)sum->exc_bt_info.bt[i]);
        }
        if (n > 0 && n < (int)sizeof(s_report) - 24) {
            snprintf(s_report + n, sizeof(s_report) - n, " elf=%.8s", (const char *)sum->app_elf_sha256);
        }
        s_have_dump = true;
    }
    free(sum);
}

bool diag_boot(void)
{
    s_reason = esp_reset_reason();
    if (s_magic != CRASH_MAGIC) {
        s_magic = CRASH_MAGIC;
        s_crash_count = 0;
    }
    if (is_crash(s_reason)) {
        s_crash_count++;
    } else {
        s_crash_count = 0;
    }
    s_safe_mode = s_crash_count >= SAFE_MODE_AFTER;
    build_report();
    return s_safe_mode;
}


static void print_status(void)
{
    ESP_LOGI(TAG, "last reset: %s; crashes in a row: %lu%s", reason_name(s_reason),
             (unsigned long)s_crash_count, s_safe_mode ? " -> SAFE MODE (Bluetooth off)" : "");
    if (s_have_dump && !is_crash(s_reason)) {
        // A dump from an earlier crash, already seen on that boot: say so once
        // and clear it, so it doesn't look like a new crash in later logs.
        ESP_LOGI(TAG, "old crash report (from before this power-on): %s", s_report);
        diag_clear_crash();
    } else if (s_have_dump) {
        ESP_LOGW(TAG, "stored crash report: %s", s_report);
    } else if (s_report[0]) {
        ESP_LOGI(TAG, "%s", s_report);
    }
}

static void diag_task(void *arg)
{
    (void)arg;
    // Repeat the status for the first two minutes so a late log connection
    // still sees it, then stay quiet.
    for (int i = 0; i < 8; i++) {
        print_status();
        vTaskDelay(pdMS_TO_TICKS(15000));
        if (esp_timer_get_time() >= STABLE_AFTER_US && s_crash_count != 0) {
            s_crash_count = 0;
            ESP_LOGI(TAG, "running stably; crash-loop counter cleared");
        }
    }
    if (s_crash_count != 0) {
        s_crash_count = 0;
    }
    vTaskDelete(NULL);
}

void diag_start(void)
{
    xTaskCreatePinnedToCore(diag_task, "diag", 3072, NULL, 2, NULL, 1);
}

void diag_clear_crash(void)
{
    if (esp_core_dump_image_erase() == ESP_OK) {
        s_have_dump = false;
        ESP_LOGI(TAG, "stored crash report erased");
    }
}
