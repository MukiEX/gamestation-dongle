# Gamestation Dongle

ESP32-S3 firmware that turns the **My Arcade Atari Gamestation Gamepad** (the wireless pad for the
Gamestation Go / Mega) into a USB controller for PCs, MiSTer and consoles. It connects to the pad over
Bluetooth LE and appears on USB as an Xbox 360 (XInput), DualShock 4, DualSense, Switch Pro, Steam
Controller or generic HID gamepad, optionally with a USB mouse next to it.

Built on the Joy-Con 2 Dongle 1.2.15 (itself based on Switch2Connect): same output modes, web
configurator, BOOT-button controls, boards and flasher. Its Switch 2 controller support is still in the
code and still works.

## Boards

| Build | Board | Flash |
|---|---|---|
| normal | any ESP32-S3 with 16 MB flash (e.g. N16R8 dev boards) | hold BOOT, plug in the USB (OTG) port |
| `tdongle` | LILYGO T-Dongle-S3 (160×80 screen, RGB LED) | hold its button while plugging in |
| `lcd147` | Waveshare ESP32-S3-LCD-1.47 (172×320 screen, RGB LED) | hold BOOT while plugging in |
| diag | any of them: no gamepad, everything on the built-in USB-Serial-JTAG port | |

The easiest way to flash is `gamestation-dongle-tool.html` (Chrome / Edge / Opera, desktop): pick the
board, connect, flash. It carries the images and the exact source they were built from.

## Pairing and use

Put the pad in its Bluetooth pairing mode close to the dongle. It pairs once (bonded, kept in flash) and
reconnects by itself afterwards. Its battery level is read every 30 s and shown on the dongle's screen and
in the configurator.

* **Select + Start, 3 s**: next controller type (also a tap of BOOT).
* **Credit + Home, 3 s**: the web configurator at http://192.168.7.1 (the dongle becomes a USB network
  adapter; also BOOT held 2–5 s). Needs Credit on Capture, the default.
* BOOT held 5 s: forget the paired pad.

Default mapping (positional, so the bottom face button is Xbox A whatever its label):

| Pad | Output |
|---|---|
| A (bottom), B (right), X (left), Y (top) | south, east, west, north |
| L1 / R1 | L / R shoulders |
| L2 / R2 | ZL / ZR triggers (digital) |
| Select / Start / Home | − / + / Home (Back / Start / Guide) |
| D-pad | d-pad |
| Credit | Capture (Share) |
| Keypad 0–9, *, #, Settings | nothing (assign any button in the configurator) |
| Trackball | USB mouse |
| Paddle | right stick X, following the knob's position |

## Paddle and trackball

Set in the configurator, saved in flash.

**Paddle** (81 positions per turn, sent as an absolute angle; the dongle unwraps it into steps):
USB mouse X · mouse wheel · left / right stick X by knob position (Atari paddle style, with a range) ·
left / right stick X by turning speed (springs back) · d-pad left / right taps · **JP3 spinner**.

**Trackball** (relative counts, ±127 per report): USB mouse · left / right stick by rolling speed ·
JP3 spinner from its X or Y axis.

Both have a sensitivity (0.25×–4×) and a direction flip.

### JP3 spinner

Taito F3 boards read an optical spinner as a quadrature encoder on the JAMMA Left / Right lines (jumper
JP3). The dongle can play that encoder on two d-pad lines: phase A on Left, phase B on **Right** (the
real wiring) or **Up**. Each step moves one Gray-code state (00 → 10 → 11 → 01) and holds it for the step
time (default 4 ms, 250 steps/s), so the host never sees both phases change between two polls. A
backlog of up to 128 steps is replayed; beyond that steps are dropped.

Use **Left / Up** unless you know Left + Right gets through: Linux's xpad and hid-nintendo drivers cancel
opposite directions, and DS4, DualSense and generic HID report the d-pad as a hat, which can't hold
both. On the F3 MiSTer core, map the pad's Up to Right.

**D-pad takes over** (on by default): pressing the pad's d-pad hands the two lines back to it, so menus
work normally. The encoder first steps back to its resting state (00), one legal step at a time, so a
JP3 decoder never sees a jump. The next paddle or trackball movement takes the lines over again. With it
off, the encoder always owns the lines and the d-pad doesn't press them.

## The pad's protocol

BLE only, HID over GATT, Just Works pairing with bonding. PnP ID VID 0x000E, PID 0x3412. Report 1
(12 bytes) carries the buttons, the paddle as GD Z (0–80) and the d-pad, keypad, Settings and Home as
keyboard-page bits; report 3 (4 bytes) is the trackball as a relative mouse. Service FF12 is the pad's
firmware update (OTA) channel: don't write to it. See `gs.h` for the byte layout.

## Building

ESP-IDF 5.5 with esp_tinyusb 2.x (TinyUSB 0.19 API). Normal builds fetch esp_tinyusb from the component
registry; offline, put local copies of `esp_tinyusb` and `tinyusb` in a folder and set
`DONGLE_EXTRA_COMPONENTS=<folder> IDF_COMPONENT_MANAGER=0`.

```
idf.py -B build build                                  # plain ESP32-S3
DONGLE_BOARD=tdongle idf.py -B build-tdongle build     # LILYGO T-Dongle-S3
DONGLE_BOARD=lcd147 idf.py -B build-lcd147 build       # Waveshare ESP32-S3-LCD-1.47
DONGLE_DIAG=1 idf.py -B build-diag -D SDKCONFIG=sdkconfig.diag \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.diag" build
tools/make_images.sh 0.1.0                             # merged images in dist/
cd tools/flasher && python3 build.py 0.1.0             # dist/gamestation-dongle-tool.html
```

## License

GPL-3.0-or-later, as the Joy-Con 2 Dongle and Switch2Connect it is based on.
