// SPDX-License-Identifier: GPL-3.0-or-later
#include "bonds.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"

static const char *TAG = "bonds";

#define BONDS_MAX 8
#define NVS_NS    "dongle"
#define NVS_KEY   "bonds"

typedef struct __attribute__((packed)) {
    uint8_t mac[6];
    uint16_t pid;
    uint32_t seq;  // higher = more recently paired; used for eviction
} bond_t;

static bond_t s_bonds[BONDS_MAX];
static int s_count;
static uint32_t s_seq;
static SemaphoreHandle_t s_mtx;

static void save_locked(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed");
        return;
    }
    esp_err_t err = nvs_set_blob(h, NVS_KEY, s_bonds, sizeof(bond_t) * s_count);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "saving bonds failed: %s", esp_err_to_name(err));
    }
    nvs_close(h);
}

void bonds_init(void)
{
    s_mtx = xSemaphoreCreateMutex();
    nvs_handle_t h;
    s_count = 0;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        size_t len = sizeof(s_bonds);
        if (nvs_get_blob(h, NVS_KEY, s_bonds, &len) == ESP_OK) {
            s_count = (int)(len / sizeof(bond_t));
        }
        nvs_close(h);
    }
    for (int i = 0; i < s_count; i++) {
        if (s_bonds[i].seq > s_seq) {
            s_seq = s_bonds[i].seq;
        }
    }
    ESP_LOGI(TAG, "%d paired controller(s) remembered", s_count);
}

uint16_t bonds_lookup(const uint8_t mac[6])
{
    uint16_t pid = 0;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    for (int i = 0; i < s_count; i++) {
        if (memcmp(s_bonds[i].mac, mac, 6) == 0) {
            pid = s_bonds[i].pid;
            break;
        }
    }
    xSemaphoreGive(s_mtx);
    return pid;
}

void bonds_remember(const uint8_t mac[6], uint16_t pid)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    int slot = -1;
    for (int i = 0; i < s_count; i++) {
        if (memcmp(s_bonds[i].mac, mac, 6) == 0) {
            slot = i;
            break;
        }
    }
    if (slot < 0 && s_count < BONDS_MAX) {
        slot = s_count++;
    }
    if (slot < 0) {
        // Full: evict the oldest pairing.
        slot = 0;
        for (int i = 1; i < s_count; i++) {
            if (s_bonds[i].seq < s_bonds[slot].seq) {
                slot = i;
            }
        }
    }
    bool changed = memcmp(s_bonds[slot].mac, mac, 6) != 0 || s_bonds[slot].pid != pid;
    memcpy(s_bonds[slot].mac, mac, 6);
    s_bonds[slot].pid = pid;
    s_bonds[slot].seq = ++s_seq;
    if (changed) {
        save_locked();
    }
    xSemaphoreGive(s_mtx);
}

void bonds_clear(void)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_count = 0;
    memset(s_bonds, 0, sizeof(s_bonds));
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY);
        nvs_commit(h);
        nvs_close(h);
    }
    xSemaphoreGive(s_mtx);
    ESP_LOGW(TAG, "all remembered controllers forgotten");
}

int bonds_count(void)
{
    return s_count;
}

int bonds_list(bond_info_t *out, int max)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    bond_t sorted[BONDS_MAX];
    int n = s_count;
    memcpy(sorted, s_bonds, sizeof(bond_t) * n);
    xSemaphoreGive(s_mtx);
    for (int i = 1; i < n; i++) {  // newest first
        bond_t b = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j].seq < b.seq) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = b;
    }
    if (n > max) n = max;
    for (int i = 0; i < n; i++) {
        memcpy(out[i].mac, sorted[i].mac, 6);
        out[i].pid = sorted[i].pid;
    }
    return n;
}

bool bonds_forget(const uint8_t mac[6])
{
    bool found = false;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    for (int i = 0; i < s_count; i++) {
        if (memcmp(s_bonds[i].mac, mac, 6) == 0) {
            s_bonds[i] = s_bonds[s_count - 1];
            s_count--;
            found = true;
            break;
        }
    }
    if (found) {
        if (s_count > 0) {
            save_locked();
        } else {
            nvs_handle_t h;
            if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
                nvs_erase_key(h, NVS_KEY);
                nvs_commit(h);
                nvs_close(h);
            }
        }
    }
    xSemaphoreGive(s_mtx);
    return found;
}
