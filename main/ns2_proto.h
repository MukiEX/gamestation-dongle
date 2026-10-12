// Switch 2 controller BLE protocol: constants, report parsing, stick calibration.
//
// Protocol details are ported from Switch2Connect (GPL-3.0, TommyWabg), which in
// turn builds on Nadeflore/switch2-controllers, ndeadly/switch2_controller_research,
// german77/JoyconDriver and darthcloud/BlueRetro research.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// ---- Identification ------------------------------------------------------
#define NS2_COMPANY_ID       0x0553  // Nintendo's BLE manufacturer-data company ID
#define NS2_VID              0x057E
#define NS2_PID_JOYCON_R     0x2066
#define NS2_PID_JOYCON_L     0x2067
#define NS2_PID_PRO2         0x2069
#define NS2_PID_GCN          0x2073  // NSO GameCube controller (not handled yet)

// ---- GATT characteristic UUIDs (lower-case, as printed by ble_uuid_to_str) --
#define NS2_UUID_INPUT        "ab7de9be-89fe-49ad-828f-118f09df7fd2"
#define NS2_UUID_ACK          "c765a961-d9d8-4d36-a20a-5315b111836a"
#define NS2_UUID_CMD          "649d4ac9-8eb7-4e6c-af44-1ea54fe5f005"
#define NS2_UUID_RUMBLE_PRO   "cc483f51-9258-427d-a939-630c31f72b05"
#define NS2_UUID_RUMBLE_JC_R  "fa19b0fb-cd1f-46a7-84a1-bbb09e00c149"
#define NS2_UUID_RUMBLE_JC_L  "289326cb-a471-485d-a8f4-240c14f18241"

// ---- Commands -----------------------------------------------------------
#define NS2_CMD_MEMORY        0x02
#define NS2_SUB_MEMORY_READ   0x04
#define NS2_CMD_LEDS          0x09
#define NS2_SUB_LEDS_PLAYER   0x07
#define NS2_CMD_PAIR          0x15
#define NS2_SUB_PAIR_SET_MAC  0x01
#define NS2_SUB_PAIR_LTK2     0x02
#define NS2_SUB_PAIR_FINISH   0x03
#define NS2_SUB_PAIR_LTK1     0x04

// ---- Memory map --------------------------------------------------------
#define NS2_ADDR_CONTROLLER_INFO   0x00013000
#define NS2_ADDR_CAL_STICK_1       0x000130A8
#define NS2_ADDR_CAL_STICK_2       0x000130E8
#define NS2_ADDR_CAL_USER_STICK_1  0x001FC042
#define NS2_ADDR_CAL_USER_STICK_2  0x001FC062

// ---- Button bits (32-bit little-endian field at offset 4 of the input report)
#define NS2_BTN_Y      0x00000001u
#define NS2_BTN_X      0x00000002u
#define NS2_BTN_B      0x00000004u
#define NS2_BTN_A      0x00000008u
#define NS2_BTN_SR_R   0x00000010u
#define NS2_BTN_SL_R   0x00000020u
#define NS2_BTN_R      0x00000040u
#define NS2_BTN_ZR     0x00000080u
#define NS2_BTN_MINUS  0x00000100u
#define NS2_BTN_PLUS   0x00000200u
#define NS2_BTN_RSTICK 0x00000400u
#define NS2_BTN_LSTICK 0x00000800u
#define NS2_BTN_HOME   0x00001000u
#define NS2_BTN_CAPT   0x00002000u
#define NS2_BTN_C      0x00004000u
#define NS2_BTN_DOWN   0x00010000u
#define NS2_BTN_UP     0x00020000u
#define NS2_BTN_RIGHT  0x00040000u
#define NS2_BTN_LEFT   0x00080000u
#define NS2_BTN_SR_L   0x00100000u
#define NS2_BTN_SL_L   0x00200000u
#define NS2_BTN_L      0x00400000u
#define NS2_BTN_ZL     0x00800000u
#define NS2_BTN_GR     0x01000000u
#define NS2_BTN_GL     0x02000000u
// The top byte of the field is a status byte on Joy-Con/Pro, not buttons.
#define NS2_BTN_MASK   0x03FFFFFFu

typedef struct {
    uint32_t timestamp;
    uint32_t buttons;           // already masked with NS2_BTN_MASK
    uint16_t ls_x, ls_y;        // raw 12-bit
    uint16_t rs_x, rs_y;        // raw 12-bit
    uint16_t mouse_x, mouse_y;  // optical sensor position (wraps)
    uint16_t mouse_roughness;
    uint16_t mouse_distance;
    int16_t  mag[3];
    uint16_t battery_mv;
    int16_t  accel[3];
    int16_t  gyro[3];
} ns2_input_t;

// Parses a Joy-Con 2 / Pro Controller 2 input notification (input mode 0x30).
// Returns false if the report is too short.
bool ns2_parse_input(const uint8_t *data, size_t len, ns2_input_t *out);

typedef struct {
    uint16_t center_x, center_y;
    uint16_t max_x, max_y;   // positive range, as an offset from center
    uint16_t min_x, min_y;   // negative range, as an offset from center
    bool valid;
} ns2_stick_cal_t;

// Decodes 9+ bytes of stick calibration; falls back to safe centred defaults.
void ns2_cal_decode(const uint8_t *data, size_t len, ns2_stick_cal_t *cal);

// Raw 12-bit -> calibrated [-1, 1] with gain and a radial deadzone.
void ns2_cal_apply(const ns2_stick_cal_t *cal, uint16_t raw_x, uint16_t raw_y,
                   float gain, float deadzone, float *out_x, float *out_y);

// One HD Rumble frame (5 bytes on the wire).
typedef struct {
    uint16_t lf_freq;  // 9 bits
    bool     lf_tone;
    uint16_t lf_amp;   // 10 bits
    uint16_t hf_freq;  // 9 bits
    bool     hf_tone;
    uint16_t hf_amp;   // 10 bits
} ns2_vib_t;

#define NS2_VIB_SILENT ((ns2_vib_t){ .lf_freq = 0x0E1, .hf_freq = 0x1E1 })

void ns2_vib_encode(const ns2_vib_t *v, uint8_t out[5]);

// Builds a rumble packet: 17 bytes for a Joy-Con, 33 for a Pro Controller.
// Each frame array holds three consecutive time-samples (~15 ms of playback).
// `right` is only used for the Pro Controller's right motor (NULL = same as left).
size_t ns2_build_rumble(uint8_t *out, uint8_t packet_id, bool is_pro,
                        const ns2_vib_t left[3], const ns2_vib_t right[3]);
