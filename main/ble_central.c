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
// Generic Bluetooth LE HID gamepads (HID over GATT, e.g. ModRetro M64 Pro)
// take a second path: connect --> SMP pairing / encryption (bonded, stored
// in NVS) --> [ctrl task] HID service discovery, report map, subscribe input
// reports --> READY as a full controller. Their reports are decoded with
// hid_gamepad.c and mapped like a standard (Android-style) gamepad.
//
// NimBLE callbacks run in the NimBLE host task and never block or issue GATT
// writes from notification context. Everything that waits on a command reply
// runs in ctrl_task.

#include "ble_central.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "bonds.h"
#include "hid_gamepad.h"
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
    volatile uint16_t battery_mv;
    uint8_t bat_raw[8];      // input bytes 29..36, logged to pin down the battery field
    bool cmd_answered;  // last cmd_exchange got a reply (even a rejection)

    // Gyro zero-rate offset, learned whenever the controller lies still.
    float gyro_bias[3];
    bool bias_known;
    float still_sum[3];
    float still_min[3], still_max[3];
    float still_accel[3];
    int still_n;

    // Optical sensor -> trackpad finger.
    bool tp_on;          // on a surface last report
    bool tp_touch;       // finger down
    uint16_t tp_lx, tp_ly;
    int32_t tp_travel;   // sensor counts moved since landing, before the finger goes down
    float tp_x, tp_y;
    bool tp_mouse_pose;  // lying on its rail side (mouse use)
    bool in_dock;        // in the charging grip: GL / GR came from this pair
    bool status_seen;
    uint8_t status_logs;
    uint8_t last_status; // top byte of the button field, logged on change

    // Generic BLE HID gamepad (hid == true).
    bool hid;
    uint16_t hid_start, hid_end;   // HID service handle range
    uint16_t h_map, h_proto;
    struct {
        uint16_t val, cccd, ref;
        uint8_t id, type;          // from the Report Reference descriptor (type 1 = input)
        uint8_t last[32];
        uint8_t last_len;
    } rep[8];
    int n_rep;
    hid_layout_t *layout;          // allocated once per link slot
    hid_state_t hst;
    uint16_t hid_logs;             // input reports logged this connection
    int64_t hid_log_window;
    uint8_t hid_log_burst;
    SemaphoreHandle_t gsem;        // synchronous GATT helpers (ctrl task)
    int gstatus;
    uint8_t *gbuf;
    size_t glen, gcap;
} link_t;

typedef enum {
    EV_ADV,
    EV_GATT_READY,
    EV_LINK_GONE,
    EV_FORGET,
    EV_FORGET_ONE,
    EV_HID_SECURED,
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
static int64_t s_connect_started;

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
            SemaphoreHandle_t gs = s_links[i].gsem;
            hid_layout_t *lay = s_links[i].layout;
            memset(&s_links[i], 0, sizeof(s_links[i]));
            s_links[i].ackq = q;
            s_links[i].gsem = gs;
            s_links[i].layout = lay;
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

// Bluetooth LE HID devices. Adverts and scan responses are merged per
// address (the name and service list are often in the scan response).
typedef struct {
    uint8_t mac[6];
    bool used, hid_uuid, discoverable, logged;
    uint16_t appearance;
    char name[24];
} adv_seen_t;
static adv_seen_t s_seen[16];
static int s_seen_next;
static uint8_t s_reject[4][6];   // non-gamepads found after connecting: ignored until reboot
static int s_reject_next;

static bool rejected(const uint8_t mac[6])
{
    for (int i = 0; i < 4; i++) {
        if (memcmp(s_reject[i], mac, 6) == 0) return true;
    }
    return false;
}

static adv_seen_t *seen_entry(const uint8_t mac[6])
{
    for (int i = 0; i < 16; i++) {
        if (s_seen[i].used && memcmp(s_seen[i].mac, mac, 6) == 0) return &s_seen[i];
    }
    adv_seen_t *e = &s_seen[s_seen_next];
    s_seen_next = (s_seen_next + 1) % 16;
    memset(e, 0, sizeof(*e));
    e->used = true;
    memcpy(e->mac, mac, 6);
    return e;
}

static void merge_adv(const struct ble_gap_disc_desc *d, adv_seen_t *e)
{
    const uint8_t *p = d->data;
    uint8_t len = d->length_data, pos = 0;
    while (pos < len) {
        uint8_t ad_len = p[pos];
        if (ad_len == 0 || pos + 1 + ad_len > len) break;
        uint8_t type = p[pos + 1];
        const uint8_t *v = &p[pos + 2];
        int vlen = ad_len - 1;
        if (type == 0x01 && vlen >= 1) e->discoverable = (v[0] & 0x03) != 0;  // limited / general discoverable
        if ((type == 0x02 || type == 0x03) && vlen >= 2) {
            for (int i = 0; i + 1 < vlen; i += 2) {
                if ((v[i] | (v[i + 1] << 8)) == 0x1812) e->hid_uuid = true;
            }
        }
        if (type == 0x19 && vlen >= 2) e->appearance = (uint16_t)(v[0] | (v[1] << 8));
        if ((type == 0x08 || type == 0x09) && vlen > 0) {
            int n = vlen < (int)sizeof(e->name) - 1 ? vlen : (int)sizeof(e->name) - 1;
            memcpy(e->name, v, n);
            e->name[n] = 0;
        }
        pos += 1 + ad_len;
    }
}

// Appearance 0x03C0..0x03CF is the HID category: 0x03C0 generic, 0x03C3
// joystick, 0x03C4 gamepad. Others (keyboard, mouse, remote 0x0180...) are
// not gamepads. No appearance at all is allowed if the HID service is listed;
// the report map is checked after connecting.
static bool hid_candidate(const adv_seen_t *e)
{
    uint16_t a = e->appearance;
    if (a == 0x03C3 || a == 0x03C4) return true;
    if (!e->hid_uuid) return false;
    return a == 0 || a == 0x03C0;
}

#define HID_PAIR_MIN_RSSI (-75)   // pair only with a gamepad close to the dongle

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
        if (!settings_get().ble_hid || rejected(d->addr.val)) {
            return;
        }
        adv_seen_t *e = seen_entry(d->addr.val);
        merge_adv(d, e);
        bool cand = hid_candidate(e);
        bool bonded = bonds_lookup(d->addr.val) == PAD_PID_BLE_HID;
        // Survey: log each HID-ish or named device once, to identify gamepads.
        if (!e->logged && (e->hid_uuid || e->appearance || e->name[0])) {
            e->logged = true;
            char mac[18];
            fmt_mac(d->addr.val, mac);
            ESP_LOGI(TAG, "seen %s (type %u) \"%s\": appearance 0x%04x%s%s, rssi %d%s", mac, d->addr.type, e->name,
                     e->appearance, e->hid_uuid ? ", HID service" : "", e->discoverable ? ", pairing mode" : "", d->rssi,
                     cand ? " -> gamepad candidate" : "");
        }
        if (bonded) {
            ev.pid = PAD_PID_BLE_HID;
            ev.pairing = false;      // bonded: reconnect
            post(&ev);
        } else if (cand && e->discoverable) {
            if (d->rssi < HID_PAIR_MIN_RSSI) {
                static int64_t last_far;
                int64_t now = esp_timer_get_time();
                if (now - last_far > 5000000) {
                    last_far = now;
                    ESP_LOGI(TAG, "gamepad \"%s\" in pairing mode is too far away (rssi %d); bring it closer", e->name,
                             d->rssi);
                }
                return;
            }
            ev.pid = PAD_PID_BLE_HID;
            ev.pairing = true;       // in pairing mode: pair with it
            post(&ev);
        }
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

// Joy-Con 2 optical sensor -> a finger on a virtual trackpad. "On a surface"
// uses Switch2Connect's thresholds (lift-off distance non-zero and under a
// limit, surface roughness under a limit; three levels). The sensor reports
// a wrapping position, so only its movement is used: the finger lands at the
// pad's centre and moves with the Joy-Con. At the edge it lifts for one report
// and lands at the centre again; trackpads read as relative motion in Steam,
// so movement simply continues.
#define TP_LAND_COUNTS  6       // movement needed before the finger goes down
#define TP_EDGE         28000

static float speed_gain(uint8_t step)
{
    return step >= 10 ? 8.0f * (step - 9) : 0.8f * step;
}

static void trackpad_update(link_t *l, const ns2_input_t *in, const pad_motion_t *m, pad_trackpad_t *tp)
{
    static const uint16_t limit_dist[3] = { 1000, 1500, 3000 };
    static const uint16_t limit_rough[3] = { 4000, 5000, 10000 };
    settings_t st = settings_get();
    int lvl = st.pad_lift <= 2 ? st.pad_lift : 0;
    // Pad units per sensor count: steps 10..19 are speeds 1..10 (8 units per
    // step), steps 1..9 are 0.1..0.9 of speed 1. The finger position is kept
    // as a float, so fractions of a unit add up instead of being lost.
    float gain = speed_gain(st.pad_speed);

    tp->valid = true;
    tp->dist = in->mouse_distance;
    tp->rough = in->mouse_roughness;
    // Mouse use = lying on the rail side: gravity along the Joy-Con's own
    // left-right axis (X). Hysteresis so it doesn't flicker at the boundary.
    if (m->valid) {
        float ax = fabsf(m->accel[0]);
        bool was = l->tp_mouse_pose;
        if (ax > 0.75f) l->tp_mouse_pose = true;
        else if (ax < 0.55f) l->tp_mouse_pose = false;
        if (l->tp_mouse_pose != was) {
            ESP_LOGI(TAG, "link %d: %s", link_index(l),
                     l->tp_mouse_pose ? "lying on its rail side (mouse use)" : "upright again");
        }
    }
    tp->mouse_pose = l->tp_mouse_pose;
    // Lying on its rail side means it is out of the charging grip.
    if (l->in_dock && l->tp_mouse_pose) {
        l->in_dock = false;
        ESP_LOGI(TAG, "link %d: out of the charging grip (lying on its rail side)", link_index(l));
    }
    if (l->in_dock) {
        // In the charging grip the sensor only sees the grip itself: ignore
        // it (no trackpad finger, no grip sensing; Steam sees the grips held).
        tp->in_dock = true;
        tp->grip_mode = false;
        tp->touch = false;
        l->tp_on = l->tp_touch = false;
        l->tp_lx = in->mouse_x;
        l->tp_ly = in->mouse_y;
        return;
    }
    if (st.pad_grip && !l->tp_mouse_pose) {
        // Held normally: the sensor faces the palm, so "something close" = a
        // hand on the grip (the loosest thresholds; a palm is no table).
        tp->grip_mode = true;
        tp->grip = in->mouse_distance != 0 && in->mouse_distance < limit_dist[2] &&
                   in->mouse_roughness < limit_rough[2];
        tp->touch = false;
        l->tp_on = l->tp_touch = false;
        l->tp_lx = in->mouse_x;
        l->tp_ly = in->mouse_y;
        return;
    }
    tp->grip_mode = false;
    // Lying on its rail side (mouse use), mouse grips such as Nintendo's lift
    // the sensor a little off the surface. Measured on a cloth mat: bare
    // Joy-Con distance 142 / roughness 2840, in a grip 652 / 4424 (over
    // Strict's 4000 roughness limit). Allow 1.5x both limits there.
    uint32_t dist_limit = limit_dist[lvl], rough_limit = limit_rough[lvl];
    if (l->tp_mouse_pose) {
        dist_limit = dist_limit * 3 / 2;
        rough_limit = rough_limit * 3 / 2;
    }
    bool on = in->mouse_distance != 0 && in->mouse_distance < dist_limit &&
              in->mouse_roughness < rough_limit;
    if (st.pad_mouse != PAD_MOUSE_OFF) {
        // Mouse setting: the sensor is a USB mouse while the Joy-Con lies on
        // its rail side on a surface; no trackpad / touchpad finger at all.
        // Mouse speed 5 = one pixel per sensor count.
        if (on && l->tp_on && l->tp_mouse_pose) {
            int16_t dx = (int16_t)(in->mouse_x - l->tp_lx);
            int16_t dy = (int16_t)(in->mouse_y - l->tp_ly);
            float mg = speed_gain(st.mouse_speed) / 40.0f;
            pad_mouse_add(dx * mg, dy * mg);
        }
        l->tp_on = on;
        l->tp_touch = false;
        l->tp_lx = in->mouse_x;
        l->tp_ly = in->mouse_y;
        tp->touch = false;
        return;
    }
    if (!on) {
        l->tp_on = false;
        l->tp_touch = false;
    } else if (!l->tp_on) {
        l->tp_on = true;
        l->tp_touch = false;
        l->tp_travel = 0;
    } else {
        int16_t dx = (int16_t)(in->mouse_x - l->tp_lx);
        int16_t dy = (int16_t)(in->mouse_y - l->tp_ly);
        if (!l->tp_touch) {
            l->tp_travel += abs(dx) + abs(dy);
            if (l->tp_travel >= TP_LAND_COUNTS) {
                l->tp_touch = true;
                l->tp_x = l->tp_y = 0.0f;
            }
        } else {
            l->tp_x += dx * gain;
            l->tp_y -= dy * gain;  // sensor +y is toward the player; pad +y is up
        }
    }
    l->tp_lx = in->mouse_x;
    l->tp_ly = in->mouse_y;

    tp->touch = l->tp_touch;
    if (l->tp_touch && (fabsf(l->tp_x) > TP_EDGE || fabsf(l->tp_y) > TP_EDGE)) {
        tp->touch = false;  // one report lifted, then down again at the centre
        l->tp_x = l->tp_y = 0.0f;
    }
    tp->x = (int16_t)l->tp_x;
    tp->y = (int16_t)l->tp_y;
}

static void handle_input(link_t *l, const uint8_t *data, size_t len)
{
    ns2_input_t in;
    if (!ns2_parse_input(data, len, &in)) {
        return;
    }
    l->input_count++;
    l->battery_mv = in.battery_mv;
    memcpy(l->bat_raw, &data[29], sizeof(l->bat_raw));

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
    // Status byte: log changes so its bits (charging, grip attached...) can
    // be worked out from test logs.
    if (!l->status_seen || in.status != l->last_status) {
        if (l->status_logs < 40) {
            l->status_logs++;
            ESP_LOGI(TAG, "link %d: status byte %02x", link_index(l), in.status);
        }
        l->status_seen = true;
        l->last_status = in.status;
    }
    // GL / GR only exist on the charging grip: seeing one means this pair is
    // in it. Applies to every connected Joy-Con until it disconnects or is
    // laid on its rail side.
    if (in.buttons & (NS2_BTN_GL | NS2_BTN_GR)) {
        for (int i = 0; i < MAX_LINKS; i++) {
            link_t *o = &s_links[i];
            if (o->state != LS_FREE && (o->role == PAD_ROLE_LEFT || o->role == PAD_ROLE_RIGHT) && !o->in_dock) {
                o->in_dock = true;
                ESP_LOGI(TAG, "link %d: in the charging grip (GL / GR pressed): its sensor is ignored", i);
            }
        }
    }
    motion_update(l, &in, &s.motion);
    if (l->role != PAD_ROLE_FULL) {
        trackpad_update(l, &in, &s.motion, &s.tp);
    }
    pad_update(l->role, &s);
}

static void hid_notify(link_t *l, uint16_t attr, const uint8_t *buf, uint16_t len);

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
    if (l->hid) {
        hid_notify(l, attr, buf, len);
        return;
    }
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
        if (!l->hid) {
            // 2M PHY for Switch 2 controllers. BLE gamepads stay on 1M: the
            // M64 Pro dropped the link right after switching (supervision
            // timeout during pairing).
            ble_gap_set_prefered_le_phy(l->conn, BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, 0);
        }
        if (l->hid) {
            // HID over GATT needs an encrypted (bonded) link first.
            int src = ble_gap_security_initiate(l->conn);
            if (src != 0) {
                ESP_LOGW(TAG, "link %d: could not start pairing (rc=%d)", idx, src);
                fail_link(l, "pairing failed to start");
            }
            return 0;
        }
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

    case BLE_GAP_EVENT_ENC_CHANGE: {
        link_t *l = link_by_conn(event->enc_change.conn_handle);
        if (l == NULL || !l->hid) {
            return 0;
        }
        if (event->enc_change.status != 0) {
            ESP_LOGW(TAG, "link %d: pairing / encryption failed (status=%d)", link_index(l),
                     event->enc_change.status);
            for (uint8_t t = 0; t <= 3; t++) {  // pair from scratch next time
                ble_addr_t a = { .type = t };
                memcpy(a.val, l->addr.val, 6);
                ble_gap_unpair(&a);
            }
            fail_link(l, "encryption failed");
            return 0;
        }
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(l->conn, &desc) == 0) {
            ESP_LOGI(TAG, "link %d: encrypted (%s, key size %u)", link_index(l),
                     desc.sec_state.bonded ? "bonded" : "not bonded", desc.sec_state.key_size);
        }
        ev_t ev = { .type = EV_HID_SECURED, .link = link_index(l) };
        post(&ev);
        return 0;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        // The controller lost our bond (or was paired elsewhere): forget the
        // old keys and pair again.
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        ESP_LOGI(TAG, "conn %u: pairing again (old keys dropped)", event->repeat_pairing.conn_handle);
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        // "Just works" is the norm for gamepads; accept a numeric comparison.
        struct ble_sm_io io = { .action = event->passkey.params.action };
        if (io.action == BLE_SM_IOACT_NUMCMP) {
            io.numcmp_accept = 1;
            ble_sm_inject_io(event->passkey.conn_handle, &io);
        } else {
            ESP_LOGW(TAG, "conn %u: controller wants passkey action %d; not supported",
                     event->passkey.conn_handle, io.action);
        }
        return 0;
    }

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

    // Player-1 LED only now that the controller is accepted, so a lit
    // controller is one that is in use (an ignored one stays dark).
    static const uint8_t led[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    cmd_exchange(l, 0x09, 0x07, led, sizeof(led), NULL, NULL);

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
// Generic Bluetooth LE HID gamepads (ctrl task, synchronous GATT)
// ---------------------------------------------------------------------------

#define GATT_WAIT_MS 3000
#define HID_MAP_MAX  1024

static bool gatt_wait(link_t *l)
{
    if (xSemaphoreTake(l->gsem, pdMS_TO_TICKS(GATT_WAIT_MS)) != pdTRUE) {
        l->gstatus = BLE_HS_ETIMEOUT;
        return false;
    }
    return l->gstatus == 0;
}

static int g_mtu_cb(uint16_t conn, const struct ble_gatt_error *err, uint16_t mtu, void *arg)
{
    link_t *l = arg;
    l->gstatus = err->status;
    if (err->status == 0) ESP_LOGI(TAG, "link %d: MTU %u", link_index(l), mtu);
    xSemaphoreGive(l->gsem);
    return 0;
}

static int g_svc_cb(uint16_t conn, const struct ble_gatt_error *err, const struct ble_gatt_svc *svc, void *arg)
{
    link_t *l = arg;
    if (err->status == 0) {
        l->hid_start = svc->start_handle;
        l->hid_end = svc->end_handle;
        return 0;
    }
    l->gstatus = err->status == BLE_HS_EDONE ? 0 : err->status;
    xSemaphoreGive(l->gsem);
    return 0;
}

static int g_chr_cb(uint16_t conn, const struct ble_gatt_error *err, const struct ble_gatt_chr *chr, void *arg)
{
    link_t *l = arg;
    if (err->status == 0) {
        if (l->n_chrs < MAX_CHRS) {
            chr_t *c = &l->chrs[l->n_chrs++];
            c->def_handle = chr->def_handle;
            c->val_handle = chr->val_handle;
            c->props = chr->properties;
            c->kind = CHR_OTHER;
            uint16_t u = chr->uuid.u.type == BLE_UUID_TYPE_16 ? ble_uuid_u16(&chr->uuid.u) : 0;
            if (u == 0x2A4B) l->h_map = chr->val_handle;
            if (u == 0x2A4E) l->h_proto = chr->val_handle;
            if (u == 0x2A4D && l->n_rep < 8) {
                l->rep[l->n_rep].val = chr->val_handle;
                l->rep[l->n_rep].type = (chr->properties & BLE_GATT_CHR_F_NOTIFY) ? 1 : 0;  // until the reference says
                l->n_rep++;
            }
        }
        return 0;
    }
    l->gstatus = err->status == BLE_HS_EDONE ? 0 : err->status;
    xSemaphoreGive(l->gsem);
    return 0;
}

static int g_dsc_cb(uint16_t conn, const struct ble_gatt_error *err, uint16_t chr_val, const struct ble_gatt_dsc *dsc,
                    void *arg)
{
    link_t *l = arg;
    if (err->status == 0) {
        uint16_t u = dsc->uuid.u.type == BLE_UUID_TYPE_16 ? ble_uuid_u16(&dsc->uuid.u) : 0;
        for (int i = 0; i < l->n_rep; i++) {
            if (l->rep[i].val != chr_val) continue;
            if (u == 0x2902) l->rep[i].cccd = dsc->handle;
            if (u == 0x2908) l->rep[i].ref = dsc->handle;
        }
        return 0;
    }
    l->gstatus = err->status == BLE_HS_EDONE ? 0 : err->status;
    xSemaphoreGive(l->gsem);
    return 0;
}

static int g_read_cb(uint16_t conn, const struct ble_gatt_error *err, struct ble_gatt_attr *attr, void *arg)
{
    link_t *l = arg;
    if (err->status == 0 && attr != NULL) {
        uint16_t n = OS_MBUF_PKTLEN(attr->om);
        if (l->glen + n > l->gcap) n = (uint16_t)(l->gcap - l->glen);
        os_mbuf_copydata(attr->om, 0, n, l->gbuf + l->glen);
        l->glen += n;
        return 0;  // read_long: more may follow
    }
    l->gstatus = err->status == BLE_HS_EDONE ? 0 : err->status;
    xSemaphoreGive(l->gsem);
    return 0;
}

static int g_write_cb(uint16_t conn, const struct ble_gatt_error *err, struct ble_gatt_attr *attr, void *arg)
{
    link_t *l = arg;
    l->gstatus = err->status;
    xSemaphoreGive(l->gsem);
    return 0;
}

static void log_hex(int idx, const char *what, const uint8_t *d, size_t n)
{
    for (size_t i = 0; i < n; i += 32) {
        char line[3 * 32 + 1];
        int k = 0;
        for (size_t j = i; j < n && j < i + 32; j++) k += snprintf(line + k, sizeof(line) - k, "%02x ", d[j]);
        ESP_LOGI(TAG, "link %d: %s +%03u: %s", idx, what, (unsigned)i, line);
    }
}

static bool hid_setup(link_t *l)
{
    int idx = link_index(l);
    xSemaphoreTake(l->gsem, 0);

    if (ble_gattc_exchange_mtu(l->conn, g_mtu_cb, l) == 0) gatt_wait(l);

    static const ble_uuid16_t hid_uuid = BLE_UUID16_INIT(0x1812);
    l->hid_start = l->hid_end = 0;
    if (ble_gattc_disc_svc_by_uuid(l->conn, &hid_uuid.u, g_svc_cb, l) != 0 || !gatt_wait(l) || l->hid_start == 0) {
        ESP_LOGW(TAG, "link %d: no HID service (status %d)", idx, l->gstatus);
        return false;
    }
    l->n_chrs = 0;
    l->n_rep = 0;
    l->h_map = l->h_proto = 0;
    if (ble_gattc_disc_all_chrs(l->conn, l->hid_start, l->hid_end, g_chr_cb, l) != 0 || !gatt_wait(l)) {
        ESP_LOGW(TAG, "link %d: HID characteristic discovery failed (%d)", idx, l->gstatus);
        return false;
    }
    ESP_LOGI(TAG, "link %d: HID service 0x%04x-0x%04x, %d characteristics, %d reports, map 0x%04x",
             idx, l->hid_start, l->hid_end, l->n_chrs, l->n_rep, l->h_map);

    // Descriptors (CCCD, Report Reference) of each report.
    for (int i = 0; i < l->n_rep; i++) {
        uint16_t end = chr_end_handle(l, l->rep[i].val);
        if (end > l->hid_end) end = l->hid_end;
        if (end > l->rep[i].val && ble_gattc_disc_all_dscs(l->conn, l->rep[i].val, end, g_dsc_cb, l) == 0) {
            gatt_wait(l);
        }
        if (l->rep[i].ref) {
            uint8_t ref[2] = { 0 };
            l->gbuf = ref; l->glen = 0; l->gcap = 2;
            if (ble_gattc_read(l->conn, l->rep[i].ref, g_read_cb, l) == 0 && gatt_wait(l) && l->glen == 2) {
                l->rep[i].id = ref[0];
                l->rep[i].type = ref[1];
            }
        }
        ESP_LOGI(TAG, "link %d: report 0x%04x: id %u, type %u (%s), CCCD 0x%04x", idx, l->rep[i].val, l->rep[i].id,
                 l->rep[i].type, l->rep[i].type == 1 ? "input" : l->rep[i].type == 2 ? "output" : "feature/other",
                 l->rep[i].cccd);
    }

    // Report map.
    if (l->h_map == 0) {
        ESP_LOGW(TAG, "link %d: no report map", idx);
        return false;
    }
    uint8_t *map = malloc(HID_MAP_MAX);
    if (map == NULL) return false;
    l->gbuf = map; l->glen = 0; l->gcap = HID_MAP_MAX;
    if (ble_gattc_read_long(l->conn, l->h_map, 0, g_read_cb, l) != 0 || !gatt_wait(l)) {
        ESP_LOGW(TAG, "link %d: report map read failed (%d)", idx, l->gstatus);
        free(map);
        return false;
    }
    size_t map_len = l->glen;
    ESP_LOGI(TAG, "link %d: report map, %u bytes", idx, (unsigned)map_len);
    log_hex(idx, "map", map, map_len);
    if (l->layout == NULL) l->layout = malloc(sizeof(hid_layout_t));
    if (l->layout == NULL) { free(map); return false; }
    hid_parse(map, map_len, l->layout);
    free(map);
    char nm[32];
    for (int i = 0; i < l->layout->n; i++) {
        const hid_field_t *f = &l->layout->f[i];
        if (f->page == 0x09 && i > 0 && l->layout->f[i - 1].page == 0x09 && !f->array) continue;  // one line per button run
        ESP_LOGI(TAG, "link %d: field id %u bit %u size %u: %s%s [%ld..%ld]", idx, f->report_id, f->bit_off, f->size,
                 hid_usage_name(f->page, f->usage, nm, sizeof(nm)), f->array ? " (array)" : f->page == 0x09 ? "..." : "",
                 (long)f->lmin, (long)f->lmax);
    }
    if (l->layout->truncated) ESP_LOGW(TAG, "link %d: report map has more fields than handled", idx);
    bool controls = false;  // buttons, sticks or a hat, not just media keys
    for (int i = 0; i < l->layout->n && !controls; i++) {
        const hid_field_t *f = &l->layout->f[i];
        if (f->page == 0x09) controls = true;
        if (f->page == 0x01 && (f->usage == 0x30 || f->usage == 0x31 || f->usage == 0x39)) controls = true;
    }
    if (!l->layout->gamepad || !controls) {
        ESP_LOGW(TAG, "link %d: not a gamepad (top-level collection page 0x%02x usage 0x%02x); forgetting it", idx,
                 l->layout->n_app ? l->layout->app_page[0] : 0, l->layout->n_app ? l->layout->app_usage[0] : 0);
        memcpy(s_reject[s_reject_next], l->addr.val, 6);
        s_reject_next = (s_reject_next + 1) % 4;
        bonds_forget(l->addr.val);
        ble_gap_unpair(&l->addr);
        return false;
    }
    hid_state_init(l->layout, &l->hst);

    // Report protocol (not boot), then subscribe every input report.
    if (l->h_proto) {
        static const uint8_t report_mode = 0x01;
        ble_gattc_write_no_rsp_flat(l->conn, l->h_proto, &report_mode, 1);
    }
    int subs = 0;
    for (int i = 0; i < l->n_rep; i++) {
        if (l->rep[i].type != 1 || l->rep[i].cccd == 0) continue;
        static const uint8_t notify_on[2] = { 0x01, 0x00 };
        if (ble_gattc_write_flat(l->conn, l->rep[i].cccd, notify_on, 2, g_write_cb, l) == 0 && gatt_wait(l)) {
            subs++;
        } else {
            ESP_LOGW(TAG, "link %d: subscribing report %u failed (%d)", idx, l->rep[i].id, l->gstatus);
        }
    }
    if (subs == 0) {
        ESP_LOGW(TAG, "link %d: no input reports to subscribe to", idx);
        return false;
    }

    if (!role_available(PAD_ROLE_FULL, idx)) {
        ESP_LOGI(TAG, "link %d: another controller is in use, ignoring this gamepad", idx);
        return false;
    }
    l->role = PAD_ROLE_FULL;
    l->pid = PAD_PID_BLE_HID;
    // Remember the identity address: gamepads like the M64 Pro advertise with
    // changing (private) addresses, which resolve to this once bonded.
    struct ble_gap_conn_desc idd;
    if (ble_gap_conn_find(l->conn, &idd) == 0 && memcmp(idd.peer_id_addr.val, l->addr.val, 6) != 0) {
        char a1[18], a2[18];
        fmt_mac(l->addr.val, a1);
        fmt_mac(idd.peer_id_addr.val, a2);
        ESP_LOGI(TAG, "link %d: %s is %s (identity address, type %u)", idx, a1, a2, idd.peer_id_addr.type);
        l->addr = idd.peer_id_addr;
    }
    bonds_remember(l->addr.val, PAD_PID_BLE_HID);
    ESP_LOGI(TAG, "link %d: BLE gamepad ready (%d input reports)", idx, subs);

    struct ble_gap_upd_params upd = {
        .itvl_min = CONN_ITVL, .itvl_max = CONN_ITVL, .latency = 0,
        .supervision_timeout = CONN_SUPERV_TO, .min_ce_len = 0, .max_ce_len = CONN_MAX_CE,
    };
    struct ble_gap_conn_desc cd;
    if (ble_gap_conn_find(l->conn, &cd) == 0) {
        ESP_LOGI(TAG, "link %d: connection interval %.2f ms", idx, cd.conn_itvl * 1.25f);
        if (cd.conn_itvl != CONN_ITVL) ble_gap_update_params(l->conn, &upd);
    }
    return true;
}

static void handle_hid_secured(int idx)
{
    link_t *l = &s_links[idx];
    if (l->state != LS_DISCOVERY || !l->hid) {
        return;
    }
    l->state = LS_INIT;
    if (!hid_setup(l)) {
        if (link_alive(l)) {
            ble_gap_terminate(l->conn, BLE_ERR_REM_USER_CONN_TERM);
        }
        return;
    }
    l->settled = false;
    l->logged_press = false;
    l->settle_deadline = esp_timer_get_time() + SETTLE_US;
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

// Standard (Android / Xbox BLE) gamepad numbering -> positional Switch
// naming: button 1 = south (B), 2 = east (A), 4 = west (Y), 5 = north (X),
// 7 / 8 = L / R, 9 / 10 = ZL / ZR, 11 = select (-), 12 = start (+),
// 13 = home, 14 / 15 = stick presses.
static uint32_t hid_map_buttons(const hid_state_t *h)
{
    static const uint32_t map[16] = {
        NS2_BTN_B, NS2_BTN_A, NS2_BTN_C, NS2_BTN_Y, NS2_BTN_X, 0, NS2_BTN_L, NS2_BTN_R,
        NS2_BTN_ZL, NS2_BTN_ZR, NS2_BTN_MINUS, NS2_BTN_PLUS, NS2_BTN_HOME, NS2_BTN_LSTICK, NS2_BTN_RSTICK, NS2_BTN_CAPT,
    };
    uint32_t b = 0;
    for (int i = 0; i < 16; i++) {
        if (h->buttons & (1u << i)) b |= map[i];
    }
    static const uint32_t hat[8] = {
        NS2_BTN_UP, NS2_BTN_UP | NS2_BTN_RIGHT, NS2_BTN_RIGHT, NS2_BTN_RIGHT | NS2_BTN_DOWN,
        NS2_BTN_DOWN, NS2_BTN_DOWN | NS2_BTN_LEFT, NS2_BTN_LEFT, NS2_BTN_LEFT | NS2_BTN_UP,
    };
    if (h->hat >= 0 && h->hat < 8) b |= hat[h->hat];
    if (h->home) b |= NS2_BTN_HOME;
    if (h->back) b |= NS2_BTN_MINUS;
    if (h->menu) b |= NS2_BTN_PLUS;
    if (h->has[HID_AX_BRAKE] && h->brake > 0.5f) b |= NS2_BTN_ZL;
    if (h->has[HID_AX_ACCEL] && h->accel > 0.5f) b |= NS2_BTN_ZR;
    return b;
}

static void stick(float x, float y, float dz, float *ox, float *oy)
{
    float m = sqrtf(x * x + y * y);
    if (m <= dz) { *ox = *oy = 0.0f; return; }
    float k = (m > 1.0f ? 1.0f : m);
    k = (k - dz) / (1.0f - dz) / m;
    *ox = x * k;
    *oy = y * k;
}

static void hid_notify(link_t *l, uint16_t attr, const uint8_t *buf, uint16_t len)
{
    int r = -1;
    for (int i = 0; i < l->n_rep; i++) {
        if (l->rep[i].val == attr) { r = i; break; }
    }
    if (r < 0 || l->layout == NULL) {
        return;
    }
    l->input_count++;
    // Log every change while testing (up to 20 per second, 3000 per connection).
    uint8_t n = len < sizeof(l->rep[r].last) ? (uint8_t)len : (uint8_t)sizeof(l->rep[r].last);
    if (n != l->rep[r].last_len || memcmp(l->rep[r].last, buf, n) != 0) {
        memcpy(l->rep[r].last, buf, n);
        l->rep[r].last_len = n;
        int64_t now = esp_timer_get_time();
        if (now - l->hid_log_window > 1000000) { l->hid_log_window = now; l->hid_log_burst = 0; }
        if (l->hid_logs < 3000 && l->hid_log_burst < 20) {
            l->hid_logs++;
            l->hid_log_burst++;
            char hex[3 * 32 + 1];
            int k = 0;
            for (int i = 0; i < n; i++) k += snprintf(hex + k, sizeof(hex) - k, "%02x ", buf[i]);
            ESP_LOGI(TAG, "link %d: report %u: %s", link_index(l), l->rep[r].id, hex);
        }
    }
    if (l->state != LS_READY) {
        return;
    }
    hid_decode(l->layout, l->rep[r].id, buf, len, &l->hst);
    const hid_state_t *h = &l->hst;
    pad_side_t s = { .buttons = hid_map_buttons(h) };
    if (!l->settled) {
        if (s.buttons == 0 || esp_timer_get_time() >= l->settle_deadline) l->settled = true;
        else return;
    }
    if (!l->logged_press && s.buttons != 0) {
        l->logged_press = true;
        ESP_LOGI(TAG, "link %d (BLE gamepad): first button press 0x%08lx (HID buttons 0x%08lx)", link_index(l),
                 (unsigned long)s.buttons, (unsigned long)h->buttons);
    }
    float dz = s_deadzone;
    stick(h->x, -h->y, dz, &s.lx, &s.ly);  // HID +y is down; pad +y is up
    if (h->has[HID_AX_Z] || h->has[HID_AX_RZ]) {
        stick(h->z, -h->rz, dz, &s.rx, &s.ry);
    } else {
        stick(h->rx, -h->ry, dz, &s.rx, &s.ry);
    }
    pad_update(PAD_ROLE_FULL, &s);
}

// ---------------------------------------------------------------------------
// Control task
// ---------------------------------------------------------------------------

static void start_connect(const ev_t *ev)
{
    if (ev->pid == PAD_PID_BLE_HID && ev->pairing && ev->addr.type >= BLE_ADDR_PUBLIC_ID) {
        // A gamepad in pairing mode that the controller still resolves from
        // old keys: drop them and connect to its next (unresolved) advert.
        char m[18];
        fmt_mac(ev->addr.val, m);
        ESP_LOGI(TAG, "%s: dropping old keys before pairing again", m);
        ble_gap_unpair(&ev->addr);
        bonds_forget(ev->addr.val);
        return;
    }
    int idx = link_alloc();
    if (idx < 0) {
        return;
    }
    link_t *l = &s_links[idx];
    l->addr = ev->addr;
    l->adv_pid = ev->pid;
    l->role = pad_role_for_pid(ev->pid);  // provisional, confirmed after reading info
    l->pairing = ev->pairing;
    l->hid = ev->pid == PAD_PID_BLE_HID;
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
    if (l->hid) {
        // Gentler start for generic gamepads (7.5-15 ms, 6 s timeout, no
        // event-length cap); 7.5 ms is requested again once set up.
        p.itvl_max = 12;
        p.supervision_timeout = 600;
        p.max_ce_len = 0;
        if (ev->pairing) {
            // Pairing mode: start clean, so stale keys from an earlier attempt
            // (on either side) can't break encryption.
            for (uint8_t t = 0; t <= 3; t++) {
                ble_addr_t a = { .type = t };
                memcpy(a.val, ev->addr.val, 6);
                ble_gap_unpair(&a);
            }
        }
    }
    s_connect_started = esp_timer_get_time();
    char mac[18];
    fmt_mac(ev->addr.val, mac);
    ESP_LOGI(TAG, "connecting to %s %s%s",
             ev->pid == PAD_PID_BLE_HID ? "BLE gamepad" : ev->pid ? pad_role_name(pad_role_for_pid(ev->pid)) : "controller",
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
    ble_store_clear();  // BLE gamepad keys
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
    for (uint8_t t = 0; t <= 1; t++) {  // BLE gamepad keys (public or random address)
        ble_addr_t a = { .type = t };
        memcpy(a.val, mac, 6);
        ble_gap_unpair(&a);
    }
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
            case EV_HID_SECURED: handle_hid_secured(ev.link); break;
            }
        }
        scan_update();

        int64_t now = esp_timer_get_time();
        // A connection attempt that never reports back (seen with a BLE
        // gamepad): cancel it so scanning resumes.
        int cidx = s_connecting;
        if (cidx >= 0 && now - s_connect_started > (CONNECT_TIMEOUT_MS + 2000) * 1000LL) {
            ESP_LOGW(TAG, "connection attempt %d stuck; cancelling", cidx);
            ble_gap_conn_cancel();
            s_connecting = -1;
            if (s_links[cidx].state == LS_CONNECTING) {
                link_free(&s_links[cidx]);
            }
        }
        s_deadzone = settings_get().deadzone_pct / 100.0f;
        static int64_t last_rate_t;
        static uint32_t rate_counts[MAX_LINKS];
        if (now - last_rate_t >= 1000000) {
            float secs = (now - last_rate_t) / 1e6f;
            last_rate_t = now;
            static uint8_t silent[MAX_LINKS];
            for (int i = 0; i < MAX_LINKS; i++) {
                uint32_t c = s_links[i].input_count;
                s_rate[i] = s_links[i].state == LS_READY
                                ? (uint16_t)((c >= rate_counts[i] ? c - rate_counts[i] : c) / secs) : 0;
                rate_counts[i] = c;
                // Controllers stream input constantly (~125/s), even untouched.
                // A ready link with none for 3 s is stuck: drop it so it can
                // reconnect cleanly instead of sitting there lit but dead.
                if (s_links[i].state == LS_READY && s_rate[i] == 0 && !s_links[i].hid) {
                    if (++silent[i] >= 3) {
                        silent[i] = 0;
                        ESP_LOGW(TAG, "link %d (%s): connected but no input for 3 s (%lu reports in total); "
                                      "dropping it so it reconnects", i, pad_role_name(s_links[i].role),
                                 (unsigned long)c);
                        fail_link(&s_links[i], "no input");
                    }
                } else {
                    silent[i] = 0;
                }
            }
        }
        if (now - last_stats >= 10000000) {
            float secs = (now - last_stats) / 1e6f;
            last_stats = now;
            for (int i = 0; i < MAX_LINKS; i++) {
                uint32_t c = s_links[i].input_count;
                if (s_links[i].state == LS_READY && s_links[i].hid) {
                    ESP_LOGI(TAG, "link %d (BLE gamepad): %.0f input reports/s", i,
                             (c >= last_counts[i] ? c - last_counts[i] : c) / secs);
                } else if (s_links[i].state == LS_READY) {
                    const uint8_t *b = s_links[i].bat_raw;
                    ESP_LOGI(TAG, "link %d (%s): %.0f input reports/s; battery %u (bytes 29-36: "
                                  "%02x %02x %02x %02x %02x %02x %02x %02x)", i, pad_role_name(s_links[i].role),
                             (c >= last_counts[i] ? c - last_counts[i] : c) / secs, s_links[i].battery_mv,
                             b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
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

#define GRIP_REPEAT_MS 2010  // a multiple of RUMBLE_PERIOD_MS

static void rumble_task(void *arg)
{
    (void)arg;
    const ns2_vib_t silent[3] = { NS2_VIB_SILENT, NS2_VIB_SILENT, NS2_VIB_SILENT };
    TickType_t wake = xTaskGetTickCount();
    unsigned tick = 0;
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(RUMBLE_PERIOD_MS));
        // Joy-Cons forget the charging grip's buttons when they leave the grip,
        // and nothing tells us they went back in: repeat the enable every ~2 s.
        if (++tick % (GRIP_REPEAT_MS / RUMBLE_PERIOD_MS) == 0) {
            static const uint8_t grip_on[12] = { 0x08, 0x91, 0x01, 0x02, 0x00, 0x04, 0x00, 0x00,
                                                 0x01, 0x00, 0x00, 0x00 };
            for (int i = 0; i < MAX_LINKS; i++) {
                link_t *l = &s_links[i];
                if (l->state == LS_READY && l->role != PAD_ROLE_FULL && l->h_cmd != 0) {
                    gatt_write(l, l->h_cmd, l->cmd_no_rsp, grip_on, sizeof(grip_on));
                }
            }
        }
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

void ble_store_config_init(void);

void ble_central_start(void)
{
    s_evq = xQueueCreate(16, sizeof(ev_t));
    for (int i = 0; i < MAX_LINKS; i++) {
        s_links[i].ackq = xQueueCreate(4, sizeof(ack_t));
        s_links[i].gsem = xSemaphoreCreateBinary();
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
    // Security, for BLE HID gamepads only (Switch 2 controllers never ask):
    // "just works" bonding, keys kept in NVS.
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();
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
        out[n].battery_mv = st == LS_READY ? l->battery_mv : 0;
        out[n].in_dock = l->in_dock;
        out[n].hid = l->hid;
        n++;
    }
    return n;
}
