// SPDX-License-Identifier: GPL-3.0-or-later
//
// Connection flow (one controller at a time):
//
//   scan --(advert from a Switch 2 controller whose role is free)--> connect
//   connect --> 2M PHY request, MTU exchange, characteristic discovery,
//               CCCD discovery, subscribe ACK + input notifications
//   GATT ready --> [ctrl task] Switch 2 init commands, read controller info,
//               set input mode 0x30 (Joy-Con), read stick calibration,
//               pair (if the controller was in SYNC mode), player LED,
//               connection buzz --> READY: input flows into pad.c
//
// NimBLE callbacks run in the NimBLE host task and never block or issue GATT
// writes from notification context. Everything that waits on a command reply
// runs in ctrl_task.

#include "ble_central.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "bonds.h"
#include "ns2_proto.h"
#include "pad.h"
#include "rumble.h"
#include "settings.h"

static const char *TAG = "ble";

#define MAX_LINKS          2      // at most L + R (or one Pro) at a time
#define MAX_CHRS           48
#define ACK_MAX            96
#define CMD_TIMEOUT_MS     2000   // Switch2Connect uses 2 s
#define ADV_STALE_US       400000 // ignore adverts queued longer than this
#define CONNECT_TIMEOUT_MS 6000
#define SETTLE_US          1000000
#define JOYCON_STICK_GAIN  1.05f  // as Switch2Connect, for factory calibration

// 7.5 ms connection interval (BLE minimum); short connection events because
// Switch 2 packets are tiny. Same parameters as Switch2Connect's bridge.
#define CONN_ITVL      6
#define CONN_SUPERV_TO 400
#define CONN_MAX_CE    3

typedef enum { CHR_OTHER, CHR_INPUT, CHR_ACK, CHR_CMD, CHR_RUMBLE } chr_kind_t;

typedef struct {
    uint16_t def_handle;
    uint16_t val_handle;
    uint8_t props;
    uint8_t kind;
} chr_t;

typedef enum {
    LS_FREE = 0,
    LS_CONNECTING,
    LS_DISCOVERY,
    LS_INIT,
    LS_READY,
} link_state_t;

typedef struct {
    uint8_t len;
    uint8_t data[ACK_MAX];
} ack_t;

typedef struct {
    volatile link_state_t state;
    uint16_t conn;
    ble_addr_t addr;
    uint16_t adv_pid;   // product ID from the advertisement / bond store (0 = unknown)
    uint16_t pid;       // product ID read from the controller
    pad_role_t role;
    bool pairing;       // connected in SYNC mode: run the pairing handshake

    chr_t chrs[MAX_CHRS];
    int n_chrs;
    uint16_t h_input, h_input_cccd;
    uint16_t h_ack, h_ack_cccd;
    uint16_t h_cmd, h_rumble;
    bool cmd_no_rsp, rumble_no_rsp;
    int dsc_step;       // 0: ACK CCCD, 1: input CCCD
    int sub_step;       // 0: ACK, 1: input

    QueueHandle_t ackq;
    ns2_stick_cal_t cal_a;  // left stick (or a lone Joy-Con R's stick)
    ns2_stick_cal_t cal_b;  // right stick (Pro only)
    bool settled;
    bool logged_press;  // first button press after connecting has been logged
    int64_t settle_deadline;
    uint8_t rumble_id;
    uint8_t rumble_tail;  // silent packets still to send after rumble stops
    volatile uint32_t input_count;
    bool cmd_answered;  // last cmd_exchange got a reply (even a rejection)

    // Gyro zero-rate offset, learned whenever the controller lies still.
    float gyro_bias[3];
    bool bias_known;
    float still_sum[3];
    float still_min[3], still_max[3];
    float still_accel[3];
    int still_n;
} link_t;

typedef enum {
    EV_ADV,
    EV_GATT_READY,
    EV_LINK_GONE,
    EV_FORGET,
    EV_FORGET_ONE,
} ev_type_t;

typedef struct {
    ev_type_t type;
    int link;
    ble_addr_t addr;
    uint16_t pid;
    bool pairing;
    int64_t t;
} ev_t;

static link_t s_links[MAX_LINKS];
static volatile float s_deadzone = 0.03f;   // from settings, refreshed by ctrl_task
static volatile uint16_t s_rate[MAX_LINKS]; // input reports/s over the last second
static portMUX_TYPE s_links_mux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t s_evq;
static uint8_t s_own_addr_type;
static uint8_t s_own_mac[6];   // little-endian (ble_addr_t.val order)
static volatile bool s_synced;
static volatile int s_connecting = -1;

static int gap_event(struct ble_gap_event *event, void *arg);

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void fmt_mac(const uint8_t v[6], char out[18])
{
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", v[5], v[4], v[3], v[2], v[1], v[0]);
}

static link_t *link_by_conn(uint16_t conn)
{
    for (int i = 0; i < MAX_LINKS; i++) {
        if (s_links[i].state != LS_FREE && s_links[i].state != LS_CONNECTING &&
            s_links[i].conn == conn) {
            return &s_links[i];
        }
    }
    return NULL;
}

static int link_index(const link_t *l)
{
    return (int)(l - s_links);
}

static int link_alloc(void)
{
    int idx = -1;
    portENTER_CRITICAL(&s_links_mux);
    for (int i = 0; i < MAX_LINKS; i++) {
        if (s_links[i].state == LS_FREE) {
            QueueHandle_t q = s_links[i].ackq;
            memset(&s_links[i], 0, sizeof(s_links[i]));
            s_links[i].ackq = q;
            s_links[i].conn = BLE_HS_CONN_HANDLE_NONE;
            s_links[i].state = LS_CONNECTING;
            idx = i;
            break;
        }
    }
    portEXIT_CRITICAL(&s_links_mux);
    if (idx >= 0) {
        xQueueReset(s_links[idx].ackq);
    }
    return idx;
}

static void link_free(link_t *l)
{
    portENTER_CRITICAL(&s_links_mux);
    l->state = LS_FREE;
    l->conn = BLE_HS_CONN_HANDLE_NONE;
    l->role = PAD_ROLE_NONE;
    portEXIT_CRITICAL(&s_links_mux);
}

static bool link_with_addr(const ble_addr_t *addr)
{
    for (int i = 0; i < MAX_LINKS; i++) {
        if (s_links[i].state != LS_FREE && memcmp(s_links[i].addr.val, addr->val, 6) == 0) {
            return true;
        }
    }
    return false;
}

static int links_in_use(void)
{
    int n = 0;
    for (int i = 0; i < MAX_LINKS; i++) {
        if (s_links[i].state != LS_FREE) {
            n++;
        }
    }
    return n;
}

// Whether a controller with `role` may connect, given the links already up.
// `ignore` is a link to leave out of the check (the one being evaluated).
static bool role_available(pad_role_t role, int ignore)
{
    bool have_left = false, have_right = false, have_full = false;
    int others = 0;
    for (int i = 0; i < MAX_LINKS; i++) {
        if (i == ignore || s_links[i].state == LS_FREE) {
            continue;
        }
        others++;
        switch (s_links[i].role) {
        case PAD_ROLE_LEFT:  have_left = true; break;
        case PAD_ROLE_RIGHT: have_right = true; break;
        case PAD_ROLE_FULL:  have_full = true; break;
        default: break;
        }
    }
    switch (role) {
    case PAD_ROLE_FULL:  return others == 0;
    case PAD_ROLE_LEFT:  return !have_full && !have_left;
    case PAD_ROLE_RIGHT: return !have_full && !have_right;
    default:             return !have_full && others < MAX_LINKS;  // unknown: decide after connecting
    }
}

static bool want_more_controllers(void)
{
    return role_available(PAD_ROLE_LEFT, -1) || role_available(PAD_ROLE_RIGHT, -1);
}

static void post(const ev_t *ev)
{
    if (s_evq) {
        xQueueSend(s_evq, ev, 0);
    }
}

// ---------------------------------------------------------------------------
// Scanning
// ---------------------------------------------------------------------------

static void scan_update(void)
{
    if (!s_synced) {
        return;
    }
    bool want = s_connecting < 0 && want_more_controllers();
    bool active = ble_gap_disc_active();
    if (want && !active) {
        // Full-duty scanning while nothing is connected; a light duty cycle
        // otherwise so scanning doesn't steal radio time from live links.
        bool idle = links_in_use() == 0;
        struct ble_gap_disc_params p = {
            .itvl = idle ? 0x30 : 0x60,
            .window = idle ? 0x30 : 0x18,
            .filter_policy = 0,
            .limited = 0,
            .passive = 0,
            .filter_duplicates = 0,
        };
        int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &p, gap_event, NULL);
        if (rc != 0 && rc != BLE_HS_EALREADY) {
            ESP_LOGW(TAG, "scan start failed rc=%d", rc);
        }
    } else if (!want && active) {
        ble_gap_disc_cancel();
    }
}

// Parses Nintendo manufacturer data. Returns false if this isn't a Switch 2 controller.
static bool parse_adv(const struct ble_gap_disc_desc *d, uint16_t *pid, bool *pairing, bool *ours)
{
    const uint8_t *p = d->data;
    uint8_t len = d->length_data;
    uint8_t pos = 0;
    while (pos < len) {
        uint8_t ad_len = p[pos];
        if (ad_len == 0 || pos + 1 + ad_len > len) {
            break;
        }
        uint8_t type = p[pos + 1];
        if (type == 0xFF && ad_len >= 3) {
            uint16_t company = (uint16_t)(p[pos + 2] | (p[pos + 3] << 8));
            if (company == NS2_COMPANY_ID) {
                const uint8_t *m = &p[pos + 4];  // payload after the company ID
                int mlen = ad_len - 3;
                if (mlen < 7) {
                    return false;
                }
                uint16_t vid = (uint16_t)(m[3] | (m[4] << 8));
                *pid = (uint16_t)(m[5] | (m[6] << 8));
                if (vid != NS2_VID) {
                    return false;
                }
                // Bytes 10..15: MAC of the host the controller is bonded to
                // (all zero while it is in SYNC/pairing mode).
                static const uint8_t zero[6] = {0};
                if (mlen >= 16) {
                    *pairing = memcmp(&m[10], zero, 6) == 0;
                    *ours = memcmp(&m[10], s_own_mac, 6) == 0;
                } else {
                    *pairing = false;
                    *ours = false;
                }
                return true;
            }
        }
        pos += 1 + ad_len;
    }
    return false;
}

static void on_disc(const struct ble_gap_disc_desc *d)
{
    ev_t ev = { .type = EV_ADV, .addr = d->addr, .t = esp_timer_get_time() };
    if (d->event_type == BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {
        // Directed reconnect advertisement: aimed at us, but carries no payload.
        ev.pid = bonds_lookup(d->addr.val);
        ev.pairing = false;
        post(&ev);
        return;
    }
    uint16_t pid = 0;
    bool pairing = false, ours = false;
    if (!parse_adv(d, &pid, &pairing, &ours)) {
        return;
    }
    if (pad_role_for_pid(pid) == PAD_ROLE_NONE) {
        return;  // e.g. NSO GameCube controller: not supported yet
    }
    if (!pairing && !ours) {
        // Bonded to another host (a Switch 2, a PC...). Connecting would just be
        // dropped by the controller. The user has to hold SYNC to pair it here.
        static int64_t last_hint;
        int64_t now = esp_timer_get_time();
        if (now - last_hint > 5000000) {
            last_hint = now;
            char mac[18];
            fmt_mac(d->addr.val, mac);
            ESP_LOGI(TAG, "%s (%s) is paired to another device; hold its SYNC button to pair it to this dongle",
                     pad_role_name(pad_role_for_pid(pid)), mac);
        }
        return;
    }
    if (!pairing && bonds_lookup(d->addr.val) == 0) {
        // Still bonded to this dongle, but the dongle forgot it (BOOT held 5 s).
        static int64_t last_forgot;
        int64_t now = esp_timer_get_time();
        if (now - last_forgot > 5000000) {
            last_forgot = now;
            char mac[18];
            fmt_mac(d->addr.val, mac);
            ESP_LOGI(TAG, "%s (%s) was forgotten; hold its SYNC button to pair it again",
                     pad_role_name(pad_role_for_pid(pid)), mac);
        }
        return;
    }
    ev.pid = pid;
    ev.pairing = pairing;
    post(&ev);
}

// ---------------------------------------------------------------------------
// GATT setup (runs in the NimBLE host task via callbacks)
// ---------------------------------------------------------------------------

static void fail_link(link_t *l, const char *why)
{
    ESP_LOGW(TAG, "link %d: %s, disconnecting", link_index(l), why);
    ble_gap_terminate(l->conn, BLE_ERR_REM_USER_CONN_TERM);
}

static int on_subscribed(uint16_t conn, const struct ble_gatt_error *err, struct ble_gatt_attr *attr, void *arg);

static void subscribe_next(link_t *l)
{
    static const uint8_t enable_notify[2] = {0x01, 0x00};
    while (l->sub_step < 2) {
        uint16_t cccd = l->sub_step == 0 ? l->h_ack_cccd : l->h_input_cccd;
        int rc = ble_gattc_write_flat(l->conn, cccd, enable_notify, sizeof(enable_notify), on_subscribed, NULL);
        if (rc == 0) {
            return;
        }
        ESP_LOGW(TAG, "link %d: subscribe write failed rc=%d", link_index(l), rc);
        l->sub_step++;
    }
    l->state = LS_INIT;
    ev_t ev = { .type = EV_GATT_READY, .link = link_index(l) };
    post(&ev);
}

static int on_subscribed(uint16_t conn, const struct ble_gatt_error *err, struct ble_gatt_attr *attr, void *arg)
{
    (void)attr;
    (void)arg;
    link_t *l = link_by_conn(conn);
    if (l == NULL) {
        return 0;
    }
    if (err->status != 0) {
        ESP_LOGW(TAG, "link %d: subscribe step %d status=%d", link_index(l), l->sub_step, err->status);
    } else {
        ESP_LOGI(TAG, "link %d: subscribed to %s (CCCD 0x%04x)", link_index(l),
                 l->sub_step == 0 ? "command replies" : "input", l->sub_step == 0 ? l->h_ack_cccd : l->h_input_cccd);
    }
    l->sub_step++;
    subscribe_next(l);
    return 0;
}

static uint16_t chr_end_handle(const link_t *l, uint16_t val_handle)
{
    uint16_t end = 0xFFFF;
    for (int i = 0; i < l->n_chrs; i++) {
        if (l->chrs[i].def_handle > val_handle && l->chrs[i].def_handle - 1 < end) {
            end = l->chrs[i].def_handle - 1;
        }
    }
    return end;
}

static void discover_next_cccd(link_t *l);

static int on_dsc(uint16_t conn, const struct ble_gatt_error *err, uint16_t chr_val_handle,
                  const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)chr_val_handle;
    (void)arg;
    link_t *l = link_by_conn(conn);
    if (l == NULL) {
        return 0;
    }
    if (err->status == 0) {
        if (dsc->uuid.u.type == BLE_UUID_TYPE_16 &&
            ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16) {
            if (l->dsc_step == 0) {
                l->h_ack_cccd = dsc->handle;
            } else {
                l->h_input_cccd = dsc->handle;
            }
        }
        return 0;
    }
    // BLE_HS_EDONE or an error: move on either way.
    l->dsc_step++;
    discover_next_cccd(l);
    return 0;
}

static void discover_next_cccd(link_t *l)
{
    while (l->dsc_step < 2) {
        uint16_t val = l->dsc_step == 0 ? l->h_ack : l->h_input;
        uint16_t end = chr_end_handle(l, val);
        if (end > val) {
            int rc = ble_gattc_disc_all_dscs(l->conn, val, end, on_dsc, NULL);
            if (rc == 0) {
                return;
            }
            ESP_LOGW(TAG, "link %d: descriptor discovery failed rc=%d", link_index(l), rc);
        }
        l->dsc_step++;
    }
    // Fall back to the usual layout (CCCD directly after the value) if needed.
    if (l->h_ack_cccd == 0) {
        l->h_ack_cccd = l->h_ack + 1;
    }
    if (l->h_input_cccd == 0) {
        l->h_input_cccd = l->h_input + 1;
    }
    l->sub_step = 0;
    subscribe_next(l);
}

static chr_kind_t classify(const char *uuid)
{
    if (strcmp(uuid, NS2_UUID_INPUT) == 0) return CHR_INPUT;
    if (strcmp(uuid, NS2_UUID_ACK) == 0) return CHR_ACK;
    if (strcmp(uuid, NS2_UUID_CMD) == 0) return CHR_CMD;
    if (strcmp(uuid, NS2_UUID_RUMBLE_PRO) == 0 ||
        strcmp(uuid, NS2_UUID_RUMBLE_JC_L) == 0 ||
        strcmp(uuid, NS2_UUID_RUMBLE_JC_R) == 0) return CHR_RUMBLE;
    return CHR_OTHER;
}

static int on_chr(uint16_t conn, const struct ble_gatt_error *err, const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;
    link_t *l = link_by_conn(conn);
    if (l == NULL) {
        return 0;
    }
    if (err->status == 0) {
        if (l->n_chrs < MAX_CHRS) {
            char uuid[BLE_UUID_STR_LEN];
            ble_uuid_to_str(&chr->uuid.u, uuid);
            for (char *p = uuid; *p; p++) {
                *p = (char)tolower((unsigned char)*p);
            }
            chr_t *c = &l->chrs[l->n_chrs++];
            c->def_handle = chr->def_handle;
            c->val_handle = chr->val_handle;
            c->props = chr->properties;
            c->kind = classify(uuid);
            switch (c->kind) {
            case CHR_INPUT:
                l->h_input = c->val_handle;
                break;
            case CHR_ACK:
                l->h_ack = c->val_handle;
                break;
            case CHR_CMD:
                l->h_cmd = c->val_handle;
                l->cmd_no_rsp = (c->props & BLE_GATT_CHR_F_WRITE_NO_RSP) != 0;
                break;
            case CHR_RUMBLE:
                l->h_rumble = c->val_handle;
                l->rumble_no_rsp = (c->props & BLE_GATT_CHR_F_WRITE_NO_RSP) != 0;
                break;
            default:
                break;
            }
        }
        return 0;
    }
    if (err->status != BLE_HS_EDONE) {
        fail_link(l, "characteristic discovery failed");
        return 0;
    }
    if (l->h_input == 0 || l->h_ack == 0 || l->h_cmd == 0) {
        fail_link(l, "not a Switch 2 controller (missing characteristics)");
        return 0;
    }
    ESP_LOGI(TAG, "link %d: %d characteristics; input 0x%04x, reply 0x%04x, command 0x%04x%s, rumble 0x%04x",
             link_index(l), l->n_chrs, l->h_input, l->h_ack, l->h_cmd, l->cmd_no_rsp ? " (no-rsp)" : "", l->h_rumble);
    l->dsc_step = 0;
    discover_next_cccd(l);
    return 0;
}

static int on_mtu(uint16_t conn, const struct ble_gatt_error *err, uint16_t mtu, void *arg)
{
    (void)arg;
    link_t *l = link_by_conn(conn);
    if (l == NULL) {
        return 0;
    }
    if (err->status == 0) {
        ESP_LOGI(TAG, "link %d: MTU %u", link_index(l), mtu);
    } else {
        ESP_LOGW(TAG, "link %d: MTU exchange failed (%d)", link_index(l), err->status);
    }
    int rc = ble_gattc_disc_all_chrs(conn, 0x0001, 0xFFFF, on_chr, NULL);
    if (rc != 0) {
        fail_link(l, "could not start characteristic discovery");
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------

// Sensor scales measured by Switch2Connect (360-degree turns, resting
// gravity): accelerometer 4096 LSB/g on both; gyro 14.29 LSB/dps on the
// Pro Controller 2 (ST +-2000 dps), 16.384 on Joy-Cons.
#define ACCEL_LSB_PER_G       4096.0f
#define GYRO_LSB_PER_DPS_PRO  14.285714f
#define GYRO_LSB_PER_DPS_JC   16.384f
// Auto-calibration: ~1 s of samples where the gyro barely moves and gravity
// stays put becomes the new zero-rate offset.
#define STILL_SAMPLES   120
#define STILL_MAX_DPS   8.0f   // |rate - current offset|, per axis
#define STILL_SPREAD    1.5f   // max - min over the window, per axis (dps)
#define STILL_ACCEL_G   0.04f  // gravity drift allowed over the window

static void still_reset(link_t *l, const float accel[3])
{
    l->still_n = 0;
    for (int i = 0; i < 3; i++) {
        l->still_sum[i] = 0.0f;
        l->still_min[i] = 1e9f;
        l->still_max[i] = -1e9f;
        l->still_accel[i] = accel[i];
    }
}

// Raw sensor axes -> the controller's upright frame (pad.h convention).
// Same mapping for Pro and both Joy-Cons, as Switch2Connect does.
static void to_body(const float raw[3], float out[3])
{
    out[0] = raw[0];
    out[1] = raw[2];
    out[2] = -raw[1];
}

static void motion_update(link_t *l, const ns2_input_t *in, pad_motion_t *m)
{
    float lsb_dps = l->role == PAD_ROLE_FULL ? GYRO_LSB_PER_DPS_PRO : GYRO_LSB_PER_DPS_JC;
    float g[3], a[3];
    for (int i = 0; i < 3; i++) {
        g[i] = in->gyro[i] / lsb_dps;
        a[i] = in->accel[i] / ACCEL_LSB_PER_G;
    }
    if (a[0] == 0.0f && a[1] == 0.0f && a[2] == 0.0f) {
        return;  // IMU not streaming (yet)
    }

    bool still = true;
    for (int i = 0; i < 3 && still; i++) {
        if (fabsf(g[i] - l->gyro_bias[i]) > STILL_MAX_DPS ||
            fabsf(a[i] - l->still_accel[i]) > STILL_ACCEL_G) {
            still = false;
        }
    }
    float amag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (!still || l->still_n == 0 || fabsf(amag - 1.0f) > 0.1f) {
        still_reset(l, a);
    }
    for (int i = 0; i < 3; i++) {
        l->still_sum[i] += g[i];
        if (g[i] < l->still_min[i]) l->still_min[i] = g[i];
        if (g[i] > l->still_max[i]) l->still_max[i] = g[i];
    }
    if (++l->still_n >= STILL_SAMPLES) {
        bool quiet = true;
        for (int i = 0; i < 3; i++) {
            if (l->still_max[i] - l->still_min[i] > STILL_SPREAD) quiet = false;
        }
        if (quiet) {
            for (int i = 0; i < 3; i++) {
                l->gyro_bias[i] = l->still_sum[i] / l->still_n;
            }
            if (!l->bias_known) {
                ESP_LOGI(TAG, "link %d: gyro calibrated (offset %.2f %.2f %.2f dps)", link_index(l),
                         l->gyro_bias[0], l->gyro_bias[1], l->gyro_bias[2]);
            }
            l->bias_known = true;
        }
        still_reset(l, a);
    }

    float gc[3];
    for (int i = 0; i < 3; i++) {
        gc[i] = g[i] - l->gyro_bias[i];
    }
    to_body(gc, m->gyro);
    to_body(a, m->accel);
    m->valid = true;
}

static void handle_input(link_t *l, const uint8_t *data, size_t len)
{
    ns2_input_t in;
    if (!ns2_parse_input(data, len, &in)) {
        return;
    }
    l->input_count++;

    // Settle gate: ignore input until the first all-released frame (or a
    // timeout), so connect-time garbage or the held wake button can't fire.
    if (!l->settled) {
        if (in.buttons == 0 || esp_timer_get_time() >= l->settle_deadline) {
            l->settled = true;
        } else {
            return;
        }
    }

    if (!l->logged_press && in.buttons != 0) {
        l->logged_press = true;
        ESP_LOGI(TAG, "link %d (%s): first button press 0x%08lx", link_index(l), pad_role_name(l->role),
                 (unsigned long)in.buttons);
    }
    pad_side_t s = { .buttons = in.buttons };
    const float STICK_DEADZONE = s_deadzone;
    switch (l->role) {
    case PAD_ROLE_LEFT:
        ns2_cal_apply(&l->cal_a, in.ls_x, in.ls_y, JOYCON_STICK_GAIN, STICK_DEADZONE, &s.lx, &s.ly);
        break;
    case PAD_ROLE_RIGHT:
        ns2_cal_apply(&l->cal_a, in.rs_x, in.rs_y, JOYCON_STICK_GAIN, STICK_DEADZONE, &s.rx, &s.ry);
        break;
    case PAD_ROLE_FULL:
        ns2_cal_apply(&l->cal_a, in.ls_x, in.ls_y, 1.0f, STICK_DEADZONE, &s.lx, &s.ly);
        ns2_cal_apply(&l->cal_b, in.rs_x, in.rs_y, 1.0f, STICK_DEADZONE, &s.rx, &s.ry);
        break;
    default:
        return;
    }
    motion_update(l, &in, &s.motion);
    pad_update(l->role, &s);
}

static void handle_notify(const struct ble_gap_event *event)
{
    link_t *l = link_by_conn(event->notify_rx.conn_handle);
    if (l == NULL) {
        return;
    }
    uint8_t buf[ACK_MAX];
    uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
    if (len > sizeof(buf)) {
        len = sizeof(buf);
    }
    if (os_mbuf_copydata(event->notify_rx.om, 0, len, buf) != 0) {
        return;
    }
    uint16_t attr = event->notify_rx.attr_handle;
    if (attr == l->h_input) {
        if (l->state == LS_READY) {
            handle_input(l, buf, len);
        }
    } else if (attr == l->h_ack) {
        ack_t a = { .len = (uint8_t)len };
        memcpy(a.data, buf, len);
        xQueueSend(l->ackq, &a, 0);
    } else if (l->state != LS_READY) {
        ESP_LOGI(TAG, "link %d: notify on handle 0x%04x, %u bytes: %02x %02x %02x %02x",
                 link_index(l), attr, len, len > 0 ? buf[0] : 0, len > 1 ? buf[1] : 0,
                 len > 2 ? buf[2] : 0, len > 3 ? buf[3] : 0);
    }
}

// ---------------------------------------------------------------------------
// GAP events
// ---------------------------------------------------------------------------

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        on_disc(&event->disc);
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        return 0;

    case BLE_GAP_EVENT_CONNECT: {
        int idx = s_connecting;
        s_connecting = -1;
        if (idx < 0) {
            if (event->connect.status == 0) {
                ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            }
            return 0;
        }
        link_t *l = &s_links[idx];
        if (event->connect.status != 0) {
            char mac[18];
            fmt_mac(l->addr.val, mac);
            ESP_LOGW(TAG, "connect to %s failed (status=%d)", mac, event->connect.status);
            link_free(l);
            ev_t ev = { .type = EV_LINK_GONE, .link = idx };
            post(&ev);
            return 0;
        }
        l->conn = event->connect.conn_handle;
        l->state = LS_DISCOVERY;
        char mac[18];
        fmt_mac(l->addr.val, mac);
        ESP_LOGI(TAG, "link %d: connected to %s%s", idx, mac, l->pairing ? " (pairing mode)" : "");
        ble_gap_set_prefered_le_phy(l->conn, BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, 0);
        // Switch 2 controllers use a plain link: no SMP / bonding here.
        int rc = ble_gattc_exchange_mtu(l->conn, on_mtu, NULL);
        if (rc != 0) {
            on_mtu(l->conn, &(struct ble_gatt_error){ .status = rc }, 0, NULL);
        }
        return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
        link_t *l = link_by_conn(event->disconnect.conn.conn_handle);
        if (l == NULL) {
            return 0;
        }
        int idx = link_index(l);
        bool was_ready = l->state == LS_READY;
        pad_role_t role = l->role;
        ESP_LOGI(TAG, "link %d: disconnected (reason=0x%03x)", idx, event->disconnect.reason);
        link_free(l);
        if (was_ready) {
            pad_link_down(role);
        }
        ev_t ev = { .type = EV_LINK_GONE, .link = idx };
        post(&ev);
        return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_RX:
        handle_notify(event);
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE_REQ:
        // Keep every link pinned at 7.5 ms instead of the controller's slower request.
        if (event->conn_update_req.self_params != NULL) {
            struct ble_gap_upd_params *p = event->conn_update_req.self_params;
            p->itvl_min = CONN_ITVL;
            p->itvl_max = CONN_ITVL;
            p->latency = 0;
            p->supervision_timeout = CONN_SUPERV_TO;
            p->min_ce_len = 0;
            p->max_ce_len = CONN_MAX_CE;
        }
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE: {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0) {
            ESP_LOGI(TAG, "conn %u: interval %.2f ms", event->conn_update.conn_handle,
                     desc.conn_itvl * 1.25f);
        }
        return 0;
    }

    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
        ESP_LOGI(TAG, "conn %u: PHY tx=%d rx=%d", event->phy_updated.conn_handle,
                 event->phy_updated.tx_phy, event->phy_updated.rx_phy);
        return 0;

    default:
        return 0;
    }
}

// ---------------------------------------------------------------------------
// Commands (ctrl task only)
// ---------------------------------------------------------------------------

static bool link_alive(const link_t *l)
{
    return l->state == LS_INIT || l->state == LS_READY;
}

static int gatt_write(link_t *l, uint16_t handle, bool no_rsp, const uint8_t *data, size_t len)
{
    if (!link_alive(l) || handle == 0) {
        return BLE_HS_ENOTCONN;
    }
    // A write on a link that is going down makes NimBLE assert, so check the
    // connection still exists right before writing.
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(l->conn, &desc) != 0) {
        return BLE_HS_ENOTCONN;
    }
    return no_rsp ? ble_gattc_write_no_rsp_flat(l->conn, handle, data, len)
                  : ble_gattc_write_flat(l->conn, handle, data, len, NULL, NULL);
}

// Sends one command and waits for its reply. On success copies the reply
// payload (after the 8-byte header) into `out`.
static bool cmd_exchange(link_t *l, uint8_t cmd, uint8_t sub, const uint8_t *data, uint8_t len,
                         uint8_t *out, size_t *out_len)
{
    uint8_t buf[8 + 64];
    if (len > 64) {
        return false;
    }
    buf[0] = cmd;
    buf[1] = 0x91;
    buf[2] = 0x01;
    buf[3] = sub;
    buf[4] = 0x00;
    buf[5] = len;
    buf[6] = 0x00;
    buf[7] = 0x00;
    if (len) {
        memcpy(&buf[8], data, len);
    }

    l->cmd_answered = false;
    for (int attempt = 0; attempt < 2; attempt++) {
        xQueueReset(l->ackq);
        int rc = gatt_write(l, l->h_cmd, l->cmd_no_rsp, buf, 8 + len);
        if (rc != 0) {
            ESP_LOGW(TAG, "link %d: cmd %02x:%02x write failed rc=%d", link_index(l), cmd, sub, rc);
            if (!link_alive(l)) {
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        int64_t deadline = esp_timer_get_time() + CMD_TIMEOUT_MS * 1000LL;
        while (link_alive(l)) {
            int64_t left_us = deadline - esp_timer_get_time();
            if (left_us <= 0) {
                break;
            }
            ack_t a;
            if (xQueueReceive(l->ackq, &a, pdMS_TO_TICKS(left_us / 1000 + 1)) != pdTRUE) {
                break;
            }
            if (a.len < 1 || a.data[0] != cmd) {
                continue;  // stale reply to something else
            }
            if (a.len < 8 || a.data[1] != 0x01) {
                // Some init commands are not supported by every controller
                // (Switch2Connect sends them too); the link is fine.
                ESP_LOGI(TAG, "link %d: cmd %02x:%02x not supported (status=%02x), continuing", link_index(l),
                         cmd, sub, a.len > 1 ? a.data[1] : 0xFF);
                l->cmd_answered = true;
                return false;
            }
            if (out && out_len) {
                size_t n = a.len - 8;
                if (n > *out_len) {
                    n = *out_len;
                }
                memcpy(out, &a.data[8], n);
                *out_len = n;
            }
            return true;
        }
        if (!link_alive(l)) {
            return false;
        }
        struct ble_gap_conn_desc d;
        uint8_t txp = 0, rxp = 0;
        bool have = ble_gap_conn_find(l->conn, &d) == 0;
        ble_gap_read_le_phy(l->conn, &txp, &rxp);
        ESP_LOGW(TAG, "link %d: cmd %02x:%02x timed out%s (conn %s, itvl %.2f ms, phy %u/%u, hci %s)",
                 link_index(l), cmd, sub, attempt == 0 ? ", retrying" : "",
                 have ? "up" : "gone", have ? d.conn_itvl * 1.25f : 0.0f, txp, rxp,
                 esp_vhci_host_check_send_available() ? "ready" : "BLOCKED");
    }
    return false;
}

static bool read_memory(link_t *l, uint32_t addr, uint8_t len, uint8_t *out)
{
    uint8_t req[8] = { len, 0x7E, 0x00, 0x00,
                       (uint8_t)addr, (uint8_t)(addr >> 8), (uint8_t)(addr >> 16), (uint8_t)(addr >> 24) };
    uint8_t resp[ACK_MAX];
    size_t n = sizeof(resp);
    if (!cmd_exchange(l, NS2_CMD_MEMORY, NS2_SUB_MEMORY_READ, req, sizeof(req), resp, &n)) {
        return false;
    }
    if (n < (size_t)(8 + len) || resp[0] != len || memcmp(&resp[4], &req[4], 4) != 0) {
        ESP_LOGW(TAG, "link %d: unexpected memory read reply for 0x%06lx", link_index(l), (unsigned long)addr);
        return false;
    }
    memcpy(out, &resp[8], len);
    return true;
}

static void read_stick_cal(link_t *l, uint32_t user_addr, uint32_t factory_addr, ns2_stick_cal_t *cal)
{
    uint8_t d[11];
    bool ok = read_memory(l, user_addr, sizeof(d), d);
    if (ok && d[0] == 0xFF && d[1] == 0xFF && d[2] == 0xFF) {
        ok = read_memory(l, factory_addr, sizeof(d), d);  // no user calibration stored
    }
    ns2_cal_decode(ok ? d : NULL, ok ? sizeof(d) : 0, cal);
    if (!cal->valid) {
        ESP_LOGW(TAG, "link %d: stick calibration unavailable, using defaults", link_index(l));
    }
}

static bool do_pairing(link_t *l)
{
    // Nintendo's application-level pairing: tell the controller our MAC so it
    // reconnects to this dongle on a button press. LTK values are fixed.
    uint8_t set_mac[14] = { 0x00, 0x02 };
    memcpy(&set_mac[2], s_own_mac, 6);
    memcpy(&set_mac[8], s_own_mac, 6);
    static const uint8_t ltk1[17] = { 0x00, 0xea, 0xbd, 0x47, 0x13, 0x89, 0x35, 0x42, 0xc6,
                                      0x79, 0xee, 0x07, 0xf2, 0x53, 0x2c, 0x6c, 0x31 };
    static const uint8_t ltk2[17] = { 0x00, 0x40, 0xb0, 0x8a, 0x5f, 0xcd, 0x1f, 0x9b, 0x41,
                                      0x12, 0x5c, 0xac, 0xc6, 0x3f, 0x38, 0xa0, 0x73 };
    static const uint8_t finish[1] = { 0x00 };
    return cmd_exchange(l, NS2_CMD_PAIR, NS2_SUB_PAIR_SET_MAC, set_mac, sizeof(set_mac), NULL, NULL) &&
           cmd_exchange(l, NS2_CMD_PAIR, NS2_SUB_PAIR_LTK1, ltk1, sizeof(ltk1), NULL, NULL) &&
           cmd_exchange(l, NS2_CMD_PAIR, NS2_SUB_PAIR_LTK2, ltk2, sizeof(ltk2), NULL, NULL) &&
           cmd_exchange(l, NS2_CMD_PAIR, NS2_SUB_PAIR_FINISH, finish, sizeof(finish), NULL, NULL);
}

static void send_rumble(link_t *l, const ns2_vib_t frames[3])
{
    if (l->h_rumble == 0) {
        return;
    }
    uint8_t pkt[40];
    size_t n = ns2_build_rumble(pkt, ++l->rumble_id, l->role == PAD_ROLE_FULL, frames, NULL);
    gatt_write(l, l->h_rumble, l->rumble_no_rsp, pkt, n);
}

static void connection_buzz(link_t *l)
{
    // A short, crisp click so you can feel which controller just connected.
    ns2_vib_t click = { .lf_freq = 0x0E1, .lf_amp = 0x030, .hf_freq = 0x1E2, .hf_amp = 0x300 };
    ns2_vib_t on[3] = { click, click, click };
    ns2_vib_t off[3] = { NS2_VIB_SILENT, NS2_VIB_SILENT, NS2_VIB_SILENT };
    for (int i = 0; i < 4; i++) {
        send_rumble(l, on);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    send_rumble(l, off);
}

// The Switch 2 controller init sequence, as used by Switch2Connect.
typedef struct {
    uint8_t cmd, sub, len;
    uint8_t data[20];
} init_cmd_t;

static const init_cmd_t INIT_SEQUENCE[] = {
    { 0x03, 0x0d, 8,  { 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff } },
    { 0x07, 0x01, 0,  { 0 } },
    { 0x16, 0x01, 0,  { 0 } },
    { 0x15, 0x03, 1,  { 0x00 } },
    // Feature select: motion (0x04) + mouse (0x10) + magnetometer (0x80) only.
    // Enabling every feature makes Joy-Cons stream phantom ZL/ZR bits.
    { 0x0c, 0x02, 4,  { 0x94, 0x00, 0x00, 0x00 } },
    { 0x11, 0x03, 0,  { 0 } },
    { 0x0a, 0x08, 20, { 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x35,
                        0x00, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
    { 0x0c, 0x04, 4,  { 0x94, 0x00, 0x00, 0x00 } },
    { 0x03, 0x0a, 4,  { 0x09, 0x00, 0x00, 0x00 } },
    { 0x10, 0x01, 0,  { 0 } },
    { 0x01, 0x0c, 0,  { 0 } },
    { 0x01, 0x01, 4,  { 0x00, 0x00, 0x00, 0x00 } },
    { 0x09, 0x07, 8,  { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
};

// Joy-Con 2 Charging Grip: its GL / GR buttons reach the console through the
// Joy-Cons, after command 08:02 turns them on (ndeadly's
// switch2_controller_research; SDL sends the same over USB). They then show up
// as the GL / GR bits in each Joy-Con's own report. Harmless without a grip.
static void grip_setup(link_t *l)
{
    int idx = link_index(l);
    static const uint8_t info_req[4] = { 0x20, 0x00, 0x00, 0x00 };
    uint8_t info[0x40];
    size_t n = sizeof(info);
    if (cmd_exchange(l, 0x08, 0x01, info_req, sizeof(info_req), info, &n) && n >= 20) {
        // 4 bytes, then grip data laid out like factory data: serial at +2.
        char serial[15] = { 0 };
        for (int i = 0; i < 14; i++) {
            char c = (char)info[6 + i];
            serial[i] = isprint((unsigned char)c) ? c : 0;
        }
        if (serial[0]) {
            ESP_LOGI(TAG, "link %d: in a charging grip (serial %s)", idx, serial);
        } else {
            ESP_LOGI(TAG, "link %d: no charging grip", idx);
        }
    }
    static const uint8_t enable[4] = { 0x01, 0x00, 0x00, 0x00 };
    if (cmd_exchange(l, 0x08, 0x02, enable, sizeof(enable), NULL, NULL)) {
        ESP_LOGI(TAG, "link %d: charging grip GL/GR enabled", idx);
    }
}

static bool link_init(link_t *l)
{
    int idx = link_index(l);
    int consecutive_fail = 0;
    for (size_t i = 0; i < sizeof(INIT_SEQUENCE) / sizeof(INIT_SEQUENCE[0]); i++) {
        const init_cmd_t *c = &INIT_SEQUENCE[i];
        if (cmd_exchange(l, c->cmd, c->sub, c->data, c->len, NULL, NULL) || l->cmd_answered) {
            consecutive_fail = 0;
        } else if (!link_alive(l) || ++consecutive_fail >= 3) {
            ESP_LOGW(TAG, "link %d: init sequence aborted at %02x:%02x", idx, c->cmd, c->sub);
            return false;
        }
    }

    uint8_t info[0x40];
    if (read_memory(l, NS2_ADDR_CONTROLLER_INFO, sizeof(info), info)) {
        l->pid = (uint16_t)(info[20] | (info[21] << 8));
        char serial[15] = { 0 };
        memcpy(serial, &info[2], 14);
        for (int i = 0; i < 14; i++) {
            if (!isprint((unsigned char)serial[i])) {
                serial[i] = 0;
            }
        }
        ESP_LOGI(TAG, "link %d: product %04x, serial %s", idx, l->pid, serial);
    } else {
        l->pid = l->adv_pid;
    }

    pad_role_t role = pad_role_for_pid(l->pid);
    if (role == PAD_ROLE_NONE) {
        ESP_LOGW(TAG, "link %d: unsupported product %04x", idx, l->pid);
        return false;
    }
    if (!role_available(role, idx)) {
        ESP_LOGI(TAG, "link %d: a %s is already in use, ignoring this one", idx, pad_role_name(role));
        return false;
    }
    l->role = role;

    if (role != PAD_ROLE_FULL) {
        // Input report format 3 (0x30). In the default format the Joy-Con's
        // status byte leaks into the button field as phantom ZL/ZR. This is a raw
        // write without the normal command header and gets no reply we rely on.
        static const uint8_t set_mode[11] = { 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0x03, 0x30 };
        gatt_write(l, l->h_cmd, l->cmd_no_rsp, set_mode, sizeof(set_mode));
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    read_stick_cal(l, NS2_ADDR_CAL_USER_STICK_1, NS2_ADDR_CAL_STICK_1, &l->cal_a);
    if (role == PAD_ROLE_FULL) {
        read_stick_cal(l, NS2_ADDR_CAL_USER_STICK_2, NS2_ADDR_CAL_STICK_2, &l->cal_b);
    } else {
        grip_setup(l);
    }

    if (l->pairing) {
        if (do_pairing(l)) {
            ESP_LOGI(TAG, "link %d: paired to this dongle", idx);
        } else {
            ESP_LOGW(TAG, "link %d: pairing handshake failed; it will work now but may not reconnect", idx);
        }
    }
    bonds_remember(l->addr.val, l->pid);

    // (The player-1 LED was already set by the last init command, 09:07.)

    // Re-assert 7.5 ms in case the controller negotiated something slower.
    struct ble_gap_upd_params upd = {
        .itvl_min = CONN_ITVL, .itvl_max = CONN_ITVL, .latency = 0,
        .supervision_timeout = CONN_SUPERV_TO, .min_ce_len = 0, .max_ce_len = CONN_MAX_CE,
    };
    struct ble_gap_conn_desc cd;
    if (ble_gap_conn_find(l->conn, &cd) == 0 && cd.conn_itvl != CONN_ITVL) {
        ble_gap_update_params(l->conn, &upd);
    }

    connection_buzz(l);
    return link_alive(l);
}

// ---------------------------------------------------------------------------
// Control task
// ---------------------------------------------------------------------------

static void start_connect(const ev_t *ev)
{
    int idx = link_alloc();
    if (idx < 0) {
        return;
    }
    link_t *l = &s_links[idx];
    l->addr = ev->addr;
    l->adv_pid = ev->pid;
    l->role = pad_role_for_pid(ev->pid);  // provisional, confirmed after reading info
    l->pairing = ev->pairing;
    s_connecting = idx;

    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
    }
    struct ble_gap_conn_params p = {
        .scan_itvl = 0x0010,
        .scan_window = 0x0010,
        .itvl_min = CONN_ITVL,
        .itvl_max = CONN_ITVL,
        .latency = 0,
        .supervision_timeout = CONN_SUPERV_TO,
        .min_ce_len = 0,
        .max_ce_len = CONN_MAX_CE,
    };
    char mac[18];
    fmt_mac(ev->addr.val, mac);
    ESP_LOGI(TAG, "connecting to %s %s%s", ev->pid ? pad_role_name(pad_role_for_pid(ev->pid)) : "controller",
             mac, ev->pairing ? " (pairing)" : "");
    int rc = ble_gap_connect(s_own_addr_type, &ev->addr, CONNECT_TIMEOUT_MS, &p, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_connect failed rc=%d", rc);
        s_connecting = -1;
        link_free(l);
    }
}

static void handle_adv(const ev_t *ev)
{
    if (s_connecting >= 0 || esp_timer_get_time() - ev->t > ADV_STALE_US) {
        return;
    }
    if (link_with_addr(&ev->addr)) {
        return;
    }
    pad_role_t role = pad_role_for_pid(ev->pid);
    if (!role_available(role, -1)) {
        return;  // e.g. a second left Joy-Con: ignored by design
    }
    start_connect(ev);
}

static void handle_gatt_ready(int idx)
{
    link_t *l = &s_links[idx];
    if (l->state != LS_INIT) {
        return;
    }
    // Switch2Connect waits 300 ms between connecting and the first command.
    vTaskDelay(pdMS_TO_TICKS(300));
    if (!link_init(l)) {
        if (link_alive(l)) {
            ble_gap_terminate(l->conn, BLE_ERR_REM_USER_CONN_TERM);
        }
        return;
    }
    l->settled = false;
    l->logged_press = false;
    l->settle_deadline = esp_timer_get_time() + SETTLE_US;
    // Only promote if the link didn't drop in the meantime (the disconnect
    // handler frees it from the NimBLE host task).
    bool promoted = false;
    portENTER_CRITICAL(&s_links_mux);
    if (l->state == LS_INIT) {
        l->state = LS_READY;
        promoted = true;
    }
    portEXIT_CRITICAL(&s_links_mux);
    if (promoted) {
        pad_link_up(l->role);
    }
}

static void forget_all(void)
{
    bonds_clear();
    for (int i = 0; i < MAX_LINKS; i++) {
        if (s_links[i].state >= LS_DISCOVERY) {
            ble_gap_terminate(s_links[i].conn, BLE_ERR_REM_USER_CONN_TERM);
        }
    }
}

static void forget_one(const uint8_t mac[6])
{
    char m[18];
    fmt_mac(mac, m);
    bool known = bonds_forget(mac);
    ESP_LOGW(TAG, "%s %s", m, known ? "forgotten" : "was not remembered");
    for (int i = 0; i < MAX_LINKS; i++) {
        if (s_links[i].state >= LS_DISCOVERY && memcmp(s_links[i].addr.val, mac, 6) == 0) {
            ble_gap_terminate(s_links[i].conn, BLE_ERR_REM_USER_CONN_TERM);
        }
    }
}

static void ctrl_task(void *arg)
{
    (void)arg;
    int64_t last_stats = esp_timer_get_time();
    uint32_t last_counts[MAX_LINKS] = { 0 };
    for (;;) {
        ev_t ev;
        if (xQueueReceive(s_evq, &ev, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (ev.type) {
            case EV_ADV:        handle_adv(&ev); break;
            case EV_GATT_READY: handle_gatt_ready(ev.link); break;
            case EV_FORGET:     forget_all(); break;
            case EV_FORGET_ONE: forget_one(ev.addr.val); break;
            case EV_LINK_GONE:  break;
            }
        }
        scan_update();

        int64_t now = esp_timer_get_time();
        s_deadzone = settings_get().deadzone_pct / 100.0f;
        static int64_t last_rate_t;
        static uint32_t rate_counts[MAX_LINKS];
        if (now - last_rate_t >= 1000000) {
            float secs = (now - last_rate_t) / 1e6f;
            last_rate_t = now;
            for (int i = 0; i < MAX_LINKS; i++) {
                uint32_t c = s_links[i].input_count;
                s_rate[i] = s_links[i].state == LS_READY
                                ? (uint16_t)((c >= rate_counts[i] ? c - rate_counts[i] : c) / secs) : 0;
                rate_counts[i] = c;
            }
        }
        if (now - last_stats >= 10000000) {
            float secs = (now - last_stats) / 1e6f;
            last_stats = now;
            for (int i = 0; i < MAX_LINKS; i++) {
                uint32_t c = s_links[i].input_count;
                if (s_links[i].state == LS_READY) {
                    ESP_LOGI(TAG, "link %d (%s): %.0f input reports/s", i, pad_role_name(s_links[i].role),
                             (c >= last_counts[i] ? c - last_counts[i] : c) / secs);
                }
                last_counts[i] = c;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Start-up
// ---------------------------------------------------------------------------

static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "address infer failed rc=%d", rc);
        return;
    }
    ble_hs_id_copy_addr(s_own_addr_type, s_own_mac, NULL);
    char mac[18];
    fmt_mac(s_own_mac, mac);
    ESP_LOGI(TAG, "dongle BLE address %s", mac);
    s_synced = true;
}

static void on_reset(int reason)
{
    s_synced = false;
    ESP_LOGW(TAG, "BLE host reset, reason=%d", reason);
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ---------------------------------------------------------------------------
// Rumble from the PC
// ---------------------------------------------------------------------------

#define RUMBLE_PERIOD_MS 15  // each packet carries three samples (~5 ms each)

static ns2_vib_t vib_max(ns2_vib_t a, ns2_vib_t b)
{
    // A lone sideways Joy-Con plays both sides: keep the stronger of each band.
    ns2_vib_t r = a;
    if (b.lf_amp > r.lf_amp) { r.lf_amp = b.lf_amp; r.lf_freq = b.lf_freq; }
    if (b.hf_amp > r.hf_amp) { r.hf_amp = b.hf_amp; r.hf_freq = b.hf_freq; }
    return r;
}

static void rumble_task(void *arg)
{
    (void)arg;
    const ns2_vib_t silent[3] = { NS2_VIB_SILENT, NS2_VIB_SILENT, NS2_VIB_SILENT };
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(RUMBLE_PERIOD_MS));
        rumble_frame_t f;
        bool active = rumble_get(&f);
        pad_out_t po;
        pad_get_output(&po);
        for (int i = 0; i < MAX_LINKS; i++) {
            link_t *l = &s_links[i];
            if (l->state != LS_READY || l->h_rumble == 0) {
                continue;
            }
            if (!active) {
                if (l->rumble_tail > 0) {
                    l->rumble_tail--;
                    send_rumble(l, silent);
                }
                continue;
            }
            l->rumble_tail = 3;
            uint8_t pkt[40];
            size_t n;
            if (l->role == PAD_ROLE_FULL) {
                ns2_vib_t lf[3] = { f.pro[0], f.pro[0], f.pro[0] };
                ns2_vib_t rf[3] = { f.pro[1], f.pro[1], f.pro[1] };
                n = ns2_build_rumble(pkt, ++l->rumble_id, true, lf, rf);
            } else {
                ns2_vib_t v;
                if (po.mode == PAD_MODE_PAIR) {
                    v = l->role == PAD_ROLE_LEFT ? f.jc[0] : f.jc[1];
                } else {
                    v = vib_max(f.jc[0], f.jc[1]);
                }
                ns2_vib_t fr[3] = { v, v, v };
                n = ns2_build_rumble(pkt, ++l->rumble_id, false, fr, NULL);
            }
            gatt_write(l, l->h_rumble, l->rumble_no_rsp, pkt, n);
        }
    }
}

void ble_central_start(void)
{
    s_evq = xQueueCreate(16, sizeof(ev_t));
    for (int i = 0; i < MAX_LINKS; i++) {
        s_links[i].ackq = xQueueCreate(4, sizeof(ack_t));
        s_links[i].state = LS_FREE;
        s_links[i].conn = BLE_HS_CONN_HANDLE_NONE;
    }

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth failed to start: %s", esp_err_to_name(err));
        return;
    }
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("Joy-Con 2 Dongle");
    nimble_port_freertos_init(host_task);

    xTaskCreatePinnedToCore(ctrl_task, "ble_ctrl", 6144, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(rumble_task, "ble_rumble", 4096, NULL, 5, NULL, 1);
}

void ble_central_forget_all(void)
{
    ev_t ev = { .type = EV_FORGET };
    post(&ev);
}

void ble_central_forget(const uint8_t mac[6])
{
    ev_t ev = { .type = EV_FORGET_ONE };
    memcpy(ev.addr.val, mac, 6);
    post(&ev);
}

int ble_central_links(ble_link_info_t *out, int max)
{
    int n = 0;
    for (int i = 0; i < MAX_LINKS && n < max; i++) {
        link_t *l = &s_links[i];
        link_state_t st = l->state;
        if (st == LS_FREE) {
            continue;
        }
        memcpy(out[n].mac, l->addr.val, 6);
        out[n].role = l->role != PAD_ROLE_NONE ? l->role : pad_role_for_pid(l->adv_pid);
        out[n].ready = st == LS_READY;
        out[n].rate = s_rate[i];
        n++;
    }
    return n;
}
