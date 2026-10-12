// Rumble requested by the PC, converted to Switch 2 HD rumble frames.
//
// The USB side (XInput, DS4, Switch Pro...) calls one of the rumble_set_*
// functions; the Bluetooth side polls rumble_get() and streams the frames to
// the connected controllers.
//
// Amplitude scaling follows Switch2Connect's defaults (vibration strength 5,
// frequency 10, Xbox rumble mode).
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ns2_proto.h"

typedef struct {
    ns2_vib_t jc[2];   // for Joy-Cons: [0] = left side, [1] = right side
    ns2_vib_t pro[2];  // for the Pro Controller 2: [0] = left motor, [1] = right motor
} rumble_frame_t;

void rumble_init(void);

// Classic two-motor rumble (XInput, DS4): strong = left/low-frequency motor,
// weak = right/high-frequency motor, 0-255. Holds until changed.
void rumble_set_motors(uint8_t strong, uint8_t weak);

// Switch HD rumble: the 4-byte left and right samples from a Switch Pro
// output report. Fades out if no new sample arrives within 150 ms.
void rumble_set_hd(const uint8_t left[4], const uint8_t right[4]);

void rumble_stop(void);

// A plain tone on one or both sides for a fixed time (Steam Controller
// haptic clicks, pulses and tones). sides: bit 0 = left, bit 1 = right.
// amp 0..1, before the rumble-strength setting.
void rumble_tone(uint8_t sides, float hz, float amp, uint32_t ms);

// Short feedback buzz played by the dongle itself (not the PC), e.g. when the
// configurator opens. Overrides PC rumble while it plays.
typedef enum {
    RUMBLE_CUE_DOUBLE,  // two short pulses: entering the configurator
    RUMBLE_CUE_SINGLE,  // one short pulse: leaving it
} rumble_cue_t;
void rumble_cue(rumble_cue_t cue);

// Current frame; false when nothing should be playing.
bool rumble_get(rumble_frame_t *out);
