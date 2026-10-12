// Web configurator: in config mode the dongle is a USB network adapter (CDC
// NCM) at 192.168.7.1 with a DHCP server, serving a single-page app and a
// small JSON API. The Bluetooth controllers stay connected, so the page can
// show live input.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include "config_server.h"

#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "dhcpserver/dhcpserver.h"
#include "dhcpserver/dhcpserver_options.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "lwip/esp_netif_net_stack.h"
#include "tinyusb_net.h"

#include "ble_central.h"
#include "bonds.h"
#include "ns2_proto.h"
#include "out.h"
#include "pad.h"
#include "settings.h"
#include "usb_gamepad.h"

static const char *TAG = "config";

extern const char web_index_html_start[] asm("_binary_index_html_start");
extern const char web_index_html_end[] asm("_binary_index_html_end");

static esp_netif_t *s_netif;
static volatile uint32_t s_rx_pkts, s_tx_pkts, s_tx_fail;
static volatile bool s_active;  // in config mode
static httpd_handle_t s_httpd;
static uint8_t s_host_mac[6];  // the PC side of the USB network link
static char s_host_mac_str[13];

// ---------------------------------------------------------------------------
// USB network <-> lwIP glue (as in ESP-IDF's sta2eth example)
// ---------------------------------------------------------------------------

static void l2_free(void *h, void *buffer)
{
    (void)h;
    free(buffer);
}

static esp_err_t netif_transmit(void *h, void *buffer, size_t len)
{
    (void)h;
    if (tinyusb_net_send_sync(buffer, (uint16_t)len, NULL, pdMS_TO_TICKS(100)) != ESP_OK) {
        s_tx_fail++;
    } else {
        s_tx_pkts++;
    }
    return ESP_OK;
}

static esp_err_t usb_net_rx(void *buffer, uint16_t len, void *ctx)
{
    (void)ctx;
    s_rx_pkts++;
    if (s_netif == NULL) {
        return ESP_OK;
    }
    void *copy = malloc(len);
    if (copy == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(copy, buffer, len);
    return esp_netif_receive(s_netif, copy, len, NULL);
}

static void on_lease(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    const ip_event_ap_staipassigned_t *e = data;
    ESP_LOGI(TAG, "PC got address " IPSTR " from the dongle", IP2STR(&e->ip));
}

// While the configurator is open, report what the USB network link is doing
// every 5 s, so a page that won't load can be narrowed down from the log.
static void stats_task(void *arg)
{
    (void)arg;
    uint32_t last_rx = 0, last_tx = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (!s_active) {
            continue;
        }
        if (s_rx_pkts != last_rx || s_tx_pkts != last_tx || s_tx_fail) {
            ESP_LOGI(TAG, "network: %lu packets from the PC, %lu to it, %lu failed sends",
                     (unsigned long)s_rx_pkts, (unsigned long)s_tx_pkts, (unsigned long)s_tx_fail);
        } else {
            ESP_LOGW(TAG, "network: nothing from the PC yet (check Device Manager for the "
                          "'Joy-Con 2 Dongle Configurator' network adapter)");
        }
        last_rx = s_rx_pkts;
        last_tx = s_tx_pkts;
    }
}

static esp_err_t netif_create(void)
{
    static bool once;
    if (!once) {
        ESP_ERROR_CHECK(esp_netif_init());
        esp_err_t err = esp_event_loop_create_default();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            return err;
        }
        esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, on_lease, NULL);
        xTaskCreatePinnedToCore(stats_task, "cfg_stats", 3072, NULL, 2, NULL, 1);
        once = true;
    }

    static esp_netif_ip_info_t ip = {
        .ip = { .addr = ESP_IP4TOADDR(192, 168, 7, 1) },
        .gw = { .addr = ESP_IP4TOADDR(192, 168, 7, 1) },
        .netmask = { .addr = ESP_IP4TOADDR(255, 255, 255, 0) },
    };
    esp_netif_inherent_config_t base = {
        .flags = ESP_NETIF_DHCP_SERVER | ESP_NETIF_FLAG_AUTOUP,
        .ip_info = &ip,
        .if_key = "usbcfg",
        .if_desc = "dongle configurator",
        .route_prio = 10,
    };
    esp_netif_driver_ifconfig_t drv = {
        .handle = (void *)1,  // USB NCM is a singleton; must be non-NULL
        .transmit = netif_transmit,
        .driver_free_rx_buffer = l2_free,
    };
    struct esp_netif_netstack_config stack = {
        .lwip = {
            .init_fn = ethernetif_init,
            .input_fn = ethernetif_input,
        },
    };
    esp_netif_config_t cfg = { .base = &base, .driver = &drv, .stack = &stack };
    s_netif = esp_netif_new(&cfg);
    if (s_netif == NULL) {
        return ESP_FAIL;
    }
    // lwIP's side of the link: a different locally administered MAC.
    uint8_t mac[6];
    memcpy(mac, s_host_mac, 6);
    mac[5] ^= 0x01;
    esp_netif_set_mac(s_netif, mac);

    // Hand out an address only: no default route and no DNS, so the PC keeps
    // using its normal network for the internet while the configurator is open.
    dhcps_offer_t off = 0;
    esp_netif_dhcps_option(s_netif, ESP_NETIF_OP_SET, ESP_NETIF_ROUTER_SOLICITATION_ADDRESS, &off, sizeof(off));
    esp_netif_dhcps_option(s_netif, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &off, sizeof(off));
    uint32_t lease_min = 60;
    esp_netif_dhcps_option(s_netif, ESP_NETIF_OP_SET, IP_ADDRESS_LEASE_TIME, &lease_min, sizeof(lease_min));
    esp_netif_action_start(s_netif, 0, 0, 0);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// HTTP handlers
// ---------------------------------------------------------------------------

static void mac_str(const uint8_t le[6], char out[18])
{
    // Stored little-endian (BLE order); shown most significant byte first.
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", le[5], le[4], le[3], le[2], le[1], le[0]);
}

static bool parse_mac(const char *s, uint8_t le[6])
{
    unsigned b[6];
    if (!s || sscanf(s, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        le[5 - i] = (uint8_t)b[i];
    }
    return true;
}

static const char *pid_name(uint16_t pid)
{
    switch (pid) {
    case NS2_PID_JOYCON_L: return "Joy-Con L";
    case NS2_PID_JOYCON_R: return "Joy-Con R";
    case NS2_PID_PRO2:     return "Pro Controller 2";
    default:               return "controller";
    }
}

static esp_err_t send_json(httpd_req_t *req, cJSON *root)
{
    char *txt = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (txt == NULL) {
        return httpd_resp_send_500(req);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_sendstr(req, txt);
    free(txt);
    return err;
}

static cJSON *read_json(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > 1024) {
        return NULL;
    }
    char buf[1025];
    int got = 0;
    while (got < req->content_len) {
        int n = httpd_req_recv(req, buf + got, req->content_len - got);
        if (n <= 0) {
            return NULL;
        }
        got += n;
    }
    buf[got] = 0;
    return cJSON_Parse(buf);
}

static esp_err_t h_index(httpd_req_t *req)
{
    ESP_LOGI(TAG, "page requested");
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, web_index_html_start, web_index_html_end - web_index_html_start - 1);  // minus the NUL
}

static cJSON *settings_json(const settings_t *s)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "mode", settings_out_mode_key(s->out_mode));
    cJSON_AddBoolToObject(o, "logPort", s->log_port);
    cJSON_AddNumberToObject(o, "rumble", s->rumble_strength);
    cJSON_AddStringToObject(o, "layout", s->layout == LAYOUT_LABELS ? "labels" : "positional");
    cJSON_AddNumberToObject(o, "deadzone", s->deadzone_pct);
    cJSON_AddBoolToObject(o, "hotkeys", s->mode_hotkeys);
    return o;
}

static esp_err_t h_state(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "firmware", usb_gamepad_firmware_version());
    settings_t s = settings_get();
    cJSON_AddItemToObject(root, "settings", settings_json(&s));

    cJSON *modes = cJSON_AddArrayToObject(root, "modes");
    for (int m = 0; m < OUT_MODE_COUNT; m++) {
        cJSON *e = cJSON_CreateObject();
        cJSON_AddStringToObject(e, "key", settings_out_mode_key((out_mode_t)m));
        cJSON_AddStringToObject(e, "name", settings_out_mode_name((out_mode_t)m));
        cJSON_AddItemToArray(modes, e);
    }

    ble_link_info_t links[4];
    int nl = ble_central_links(links, 4);
    cJSON *conn = cJSON_AddArrayToObject(root, "connected");
    for (int i = 0; i < nl; i++) {
        char m[18];
        mac_str(links[i].mac, m);
        cJSON *e = cJSON_CreateObject();
        cJSON_AddStringToObject(e, "mac", m);
        cJSON_AddStringToObject(e, "role", pad_role_name(links[i].role));
        cJSON_AddBoolToObject(e, "ready", links[i].ready);
        cJSON_AddNumberToObject(e, "rate", links[i].rate);
        cJSON_AddItemToArray(conn, e);
    }

    bond_info_t bonds[8];
    int nb = bonds_list(bonds, 8);
    cJSON *paired = cJSON_AddArrayToObject(root, "paired");
    for (int i = 0; i < nb; i++) {
        char m[18];
        mac_str(bonds[i].mac, m);
        cJSON *e = cJSON_CreateObject();
        cJSON_AddStringToObject(e, "mac", m);
        cJSON_AddStringToObject(e, "type", pid_name(bonds[i].pid));
        cJSON_AddItemToArray(paired, e);
    }

    pad_out_t o;
    pad_get_output(&o);
    cJSON *in = cJSON_AddObjectToObject(root, "input");
    static const char *mode_names[] = { "none", "pro", "pair", "left", "right" };
    cJSON_AddStringToObject(in, "layout", mode_names[o.mode < 5 ? o.mode : 0]);
    cJSON_AddNumberToObject(in, "buttons", o.buttons);
    cJSON_AddNumberToObject(in, "lx", o.lx);
    cJSON_AddNumberToObject(in, "ly", o.ly);
    cJSON_AddNumberToObject(in, "rx", o.rx);
    cJSON_AddNumberToObject(in, "ry", o.ry);
    return send_json(req, root);
}

static esp_err_t h_settings(httpd_req_t *req)
{
    cJSON *j = read_json(req);
    if (j == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad JSON");
    }
    settings_t s = settings_get();
    cJSON *v;
    if ((v = cJSON_GetObjectItem(j, "mode")) && cJSON_IsString(v)) {
        for (int m = 0; m < OUT_MODE_COUNT; m++) {
            if (strcmp(v->valuestring, settings_out_mode_key((out_mode_t)m)) == 0) {
                s.out_mode = (out_mode_t)m;
            }
        }
    }
    if ((v = cJSON_GetObjectItem(j, "logPort")) && cJSON_IsBool(v)) s.log_port = cJSON_IsTrue(v);
    if ((v = cJSON_GetObjectItem(j, "rumble")) && cJSON_IsNumber(v)) s.rumble_strength = (uint8_t)v->valueint;
    if ((v = cJSON_GetObjectItem(j, "layout")) && cJSON_IsString(v)) {
        s.layout = strcmp(v->valuestring, "labels") == 0 ? LAYOUT_LABELS : LAYOUT_POSITIONAL;
    }
    if ((v = cJSON_GetObjectItem(j, "deadzone")) && cJSON_IsNumber(v)) s.deadzone_pct = (uint8_t)v->valueint;
    if ((v = cJSON_GetObjectItem(j, "hotkeys")) && cJSON_IsBool(v)) s.mode_hotkeys = cJSON_IsTrue(v);
    cJSON_Delete(j);
    settings_set(&s);
    s = settings_get();
    return send_json(req, settings_json(&s));
}

static esp_err_t h_forget(httpd_req_t *req)
{
    cJSON *j = read_json(req);
    if (j == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad JSON");
    }
    cJSON *all = cJSON_GetObjectItem(j, "all");
    cJSON *mac = cJSON_GetObjectItem(j, "mac");
    uint8_t le[6];
    if (all && cJSON_IsTrue(all)) {
        ble_central_forget_all();
    } else if (mac && cJSON_IsString(mac) && parse_mac(mac->valuestring, le)) {
        ble_central_forget(le);
    } else {
        cJSON_Delete(j);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need mac or all");
    }
    cJSON_Delete(j);
    return send_json(req, cJSON_CreateTrue());
}

static esp_err_t h_exit(httpd_req_t *req)
{
    esp_err_t err = send_json(req, cJSON_CreateTrue());
    usb_gamepad_exit_config();  // switches USB after a short delay
    return err;
}

static void httpd_start_once(void)
{
    if (s_httpd) {
        return;
    }
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.core_id = 0;
    cfg.stack_size = 6144;
    cfg.lru_purge_enable = true;
    cfg.max_open_sockets = 5;
    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "web server failed to start");
        s_httpd = NULL;
        return;
    }
    const httpd_uri_t uris[] = {
        { .uri = "/",             .method = HTTP_GET,  .handler = h_index },
        { .uri = "/index.html",   .method = HTTP_GET,  .handler = h_index },
        { .uri = "/api/state",    .method = HTTP_GET,  .handler = h_state },
        { .uri = "/api/settings", .method = HTTP_POST, .handler = h_settings },
        { .uri = "/api/forget",   .method = HTTP_POST, .handler = h_forget },
        { .uri = "/api/exit",     .method = HTTP_POST, .handler = h_exit },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(s_httpd, &uris[i]);
    }
}

// ---------------------------------------------------------------------------
// The config-mode USB "driver"
// ---------------------------------------------------------------------------

#define EP_NCM_NOTIF 0x81
#define EP_NCM_OUT   0x01
#define EP_NCM_IN    0x84

static const uint8_t itf_desc[] = {
    TUD_CDC_NCM_DESCRIPTOR(0, 4, CONFIG_MAC_STRING_INDEX, EP_NCM_NOTIF, 64, EP_NCM_OUT, EP_NCM_IN, 64,
                           CFG_TUD_NET_MTU),
};

const char *config_server_host_mac_string(void)
{
    if (s_host_mac_str[0] == 0) {
        // Locally administered MAC derived from the chip's, so two dongles differ.
        uint8_t base[6];
        esp_read_mac(base, ESP_MAC_BASE);
        s_host_mac[0] = 0x02;
        s_host_mac[1] = 0x4A;  // 'J'
        s_host_mac[2] = 0x32;  // '2'
        s_host_mac[3] = base[3];
        s_host_mac[4] = base[4];
        s_host_mac[5] = (uint8_t)(base[5] & 0xFE);
        snprintf(s_host_mac_str, sizeof(s_host_mac_str), "%02X%02X%02X%02X%02X%02X", s_host_mac[0],
                 s_host_mac[1], s_host_mac[2], s_host_mac[3], s_host_mac[4], s_host_mac[5]);
    }
    return s_host_mac_str;
}

static void cfg_started(void)
{
    config_server_host_mac_string();
    tinyusb_net_config_t net = {
        .on_recv_callback = usb_net_rx,
    };
    memcpy(net.mac_addr, s_host_mac, 6);
    if (tinyusb_net_init(&net) != ESP_OK) {
        ESP_LOGE(TAG, "USB network init failed");
    }
    if (s_netif == NULL && netif_create() != ESP_OK) {
        ESP_LOGE(TAG, "network interface failed to start");
        return;
    }
    httpd_start_once();
    s_active = true;
    ESP_LOGI(TAG, "configurator ready: open http://192.168.7.1 (press BOOT to leave)");
}

static void cfg_stopping(void)
{
    s_active = false;
    tinyusb_net_deinit();
}

static bool cfg_send(const pad_out_t *o)
{
    (void)o;
    return true;
}

const out_driver_t out_config = {
    .name = "web configurator",
    .vid = 0x303A, .pid = 0x8244, .bcd_device = 0x0001,
    .manufacturer = "Joy-Con 2 Dongle",
    .product = "Joy-Con 2 Dongle Configurator",
    .itf_desc = itf_desc, .itf_desc_len = sizeof(itf_desc), .itf_count = 2,
    .hid_report_desc = NULL,
    .send = cfg_send,
    .period_ms = 100,
    .started = cfg_started,
    .stopping = cfg_stopping,
    .is_config = true,
};
