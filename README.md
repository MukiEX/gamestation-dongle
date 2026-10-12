# Joy-Con 2 Dongle

Firmware for an **ESP32-S3 N16R8** that connects Nintendo Switch 2 controllers
over Bluetooth LE and presents them to a PC/console as **one USB controller**.
A left and a right Joy-Con 2 automatically merge into a single controller.

Based on [Switch2Connect](https://github.com/TommyWabg/Switch2Connect) by
TommyWabg, so this project is **GPL-3.0**.

## Status: step 5 of 5

| Step | What | State |
|---|---|---|
| 1 | Joy-Con 2 / Pro Controller 2 pairing + L+R merge → generic USB HID gamepad, serial debug log | done |
| 2 | XInput, DS4, Switch Pro output modes + rumble | **done (0.2.4)** |
| 3 | Web configurator at 192.168.7.1, settings in flash | **done (0.3.4)** |
| 4 | Gyro, DualSense output with audio haptics, Switch 2 Pro output | gyro **done (0.4.3)**, back buttons, DualSense (0.5.0); audio haptics and Switch 2 Pro still to do |
| 5 | Steam Controller (2026) mode with mouse-sensor trackpads | **this build (0.6.0)** |

### Controllers

* Supported: **Joy-Con 2 (L)**, **Joy-Con 2 (R)**, **Pro Controller 2**.
  (NSO GameCube controller: not yet.)
* **Pairing:** hold the small SYNC button on the controller; it connects and
  pairs to the dongle. After that, pressing any button reconnects it.
* **Merging:** one left + one right Joy-Con become one controller. A pair stays a
  pair until both halves are gone; if one half drops, its inputs read neutral
  and it rejoins when it reconnects.
* **One controller per role:** a second Joy-Con of the same side, or any Joy-Con
  while a Pro Controller is connected, is ignored.
* **Lone Joy-Con:** used sideways as a mini-controller.
* Each controller gives a short buzz and shows the player-1 LED when ready.
* **Forget all pairings:** hold the board's **BOOT** button for 5 seconds.

### Web configurator

Hold the board's **BOOT** button for 2–3 seconds and let go (or hold **Capture**
and **Home** on the controller for 3 seconds). The dongle turns into a small
USB network adapter; open **http://192.168.7.1** in any browser. The PC keeps
its normal internet connection: the dongle hands out an address but no route.

* **Output:** controller type, button layout (positional or Nintendo labels),
  log port on/off, hotkeys on/off.
* **Feel:** rumble strength (0–10, 5 = Switch2Connect's default), stick
  deadzone (applies live).
* **Controllers:** what is connected right now (with reports/s), the paired
  list with Forget buttons, and a live view of every button and both sticks.

**Save & exit** (or a short BOOT press, or Capture + Home again) turns the
dongle back into a controller. A configurator tab left open from an earlier
session reloads itself instead of changing anything. The Bluetooth controllers stay connected the
whole time. Settings are stored on the dongle.

### Output modes

| Mode | Shows up as | Rumble | Gyro |
|---|---|---|---|
| **XInput** (default) | Xbox 360 controller (045E:028E) | yes | – |
| **DualShock 4** | DS4 (054C:09CC) | yes | yes |
| **DualSense** | DualSense (054C:0CE6) | yes (rumble emulation; audio haptics next) | yes |
| **Switch Pro** | Switch Pro Controller (057E:2009) | yes (HD rumble, converted) | yes |
| **Steam Controller** | Steam Controller 2026 (28DE:1302), Joy-Con mouse sensors as its trackpads | yes (rumble, haptic clicks and tones) | yes |
| **Generic HID** | "Joy-Con 2 Dongle" HID gamepad (303A:8243) | no | – |

**Changing mode:** pick it in the configurator, press **BOOT** briefly, or hold
**−** and **+** together for 3 seconds. The dongle drops off USB for about a second and comes back as the
next controller; the Bluetooth controllers stay connected. The mode is saved
for the next power-up. The order is XInput → DS4 → DualSense → Switch Pro → Steam Controller → generic HID.

Switch Pro mode only reads correctly in software that speaks the Pro
Controller's USB protocol (Steam, SDL games, Chrome's gamepad support, a
Switch). Windows' generic game-controller view shows nonsense for it, as it
does for a real wired Pro Controller.

Every mode also has the serial log port unless you turn it off in the
configurator, which consoles may need. Turning it off only removes the
dongle's own USB COM port; the board's second (UART) USB port always carries
the console log. Console use (a real Switch or PS4) is untested.

Buttons are **positional**: the bottom face button is Xbox A / PS Cross / Switch
B, whatever the label on the Nintendo controller says.

| Switch 2 | XInput | DS4 / DualSense | Switch Pro |
|---|---|---|---|
| B (bottom) / A (right) / Y (left) / X (top) | A / B / X / Y | Cross / Circle / Square / Triangle | B / A / Y / X |
| L / R | LB / RB | L1 / R1 | L / R |
| ZL / ZR | LT / RT (full) | L2 / R2 (full) | ZL / ZR |
| − / + | Back / Start | Share / Options (DualSense: Create / Options) | − / + |
| Home | Guide | PS | Home |
| Capture | – | Touchpad click | Capture |
| GL / GR (Pro Controller 2, Joy-Con 2 charging grip) | your choice | touchpad left / right half | your choice |
| C (GameChat) | – | DualSense: mute button | – |
| SL, SR | – | – | – |

**Back buttons (GL / GR)** on the Pro Controller 2 and on the Joy-Con 2
Charging Grip. The grip's buttons reach the dongle through the Joy-Cons: the
dongle sends each Joy-Con the "enable charging grip buttons" command (08:02)
when it connects and every 2 seconds after that (a Joy-Con forgets it when it
leaves the grip). They can be set to any button in the configurator. On
*Auto* they press the left and right half of the touchpad in DS4 mode (the same
trick Switch2Connect uses, so Steam Input and DS4Windows can bind them
separately) and buttons 20 / 21 in generic HID mode. XInput and Switch Pro have
no spare buttons, so there Auto does nothing until you pick one.

**Rumble** follows Switch2Connect's defaults (strength 5, frequency 10): the
large motor drives the low band and the small motor the high band. Switch HD
rumble keeps its frequencies. With a Joy-Con pair each half plays its own
side; a lone Joy-Con plays both.

**Gyro and accelerometer** pass through in DS4 and Switch Pro modes, scaled
to what those controllers report (sensor scales measured by Switch2Connect).
A Joy-Con pair uses the right Joy-Con's motion (the left one's while the right
is away); a lone sideways Joy-Con's axes are turned to match how it is held.
The gyro recalibrates itself whenever the controller rests still for about a
second. The configurator shows live motion under Live input. Verified on hardware
in both modes: decoded the way SDL does (the tool page's Motion tab), and with
Steam's gyro-as-mouse. For a game that reads an axis backwards, the
configurator's *Advanced* section can flip each gyro and accelerometer axis
per mode; the log prints the active code (e.g. `gyro +-+ accel +++`) when
those modes start.

### Steam Controller mode

The dongle presents itself as a wired Steam Controller (2026), which Steam
supports natively, so Steam Input can bind everything, including the
trackpads.

* **Trackpads from the Joy-Con mouse sensors.** Slide a Joy-Con on a surface
  (sensor side down) and a finger moves on that side's trackpad: left Joy-Con →
  left pad, right Joy-Con → right pad. Lifting it lifts the finger. The finger
  starts at the pad's centre and, at the edge, lifts for one report and lands
  at the centre again; Steam reads trackpads as relative motion, so the
  movement continues. *Trackpad speed* and *Trackpad lift-off* are in the
  configurator, which also shows both trackpads live.
* **Buttons:** Home = Steam button, Capture = quick access (…), − / + = the
  left / right centre buttons (Back / Start in SDL), GL / GR = L4 / R4, a Joy-Con's SL / SR = L5 / R5. ZL / ZR are full
  trigger pulls. Joy-Cons have no touch sensing, so a moved stick counts as
  "touched".
* **Haptics:** Steam's rumble drives the Joy-Con motors; its haptic clicks,
  pulses and tones (Identify Controller, trackpad clicks) play as short Joy-Con
  HD-rumble tones at the requested pitch.
* **If Steam offers a firmware update** for this controller, decline it: the
  dongle only imitates the controller.

Sources: the USB descriptors and the answers Steam checks come from
[HIDMaestro](https://github.com/hifihedgehog/HIDMaestro)'s steam-controller-2
profile (MIT); the input layout from SDL's `SDL_hidapi_steam_triton.c`; the
haptic reports from [sc2-research](https://github.com/CouchTurtle/sc2-research)
(MIT).

The generic HID mode is the step-1 layout:

| HID button | Controller | | HID button | Controller |
|---|---|---|---|---|
| 1 | B (south) | | 12 | R stick click |
| 2 | A (east) | | 13 | Home |
| 3 | Y (west) | | 14 | Capture |
| 4 | X (north) | | 15 | C (GameChat) |
| 5 / 6 | L / R | | 16 / 17 | SL / SR (left Joy-Con) |
| 7 / 8 | ZL / ZR | | 18 / 19 | SL / SR (right Joy-Con) |
| 9 / 10 | − / + | | 20 / 21 | GL / GR (Pro 2 back buttons) |
| 11 | L stick click | | | |

## Flashing

Prebuilt images are in `dist/`. You need [esptool](https://github.com/espressif/esptool)
(`pip install esptool`) or Espressif's web flasher.

1. Hold **BOOT**, plug the board's **USB/OTG** port into the PC, release BOOT.
2. Flash the single merged image:

   ```
   esptool.py --chip esp32s3 -p <PORT> -b 921600 write_flash 0x0 dist/joycon2-dongle-0.1.5-merged.bin
   ```

   (Or the separate files: `0x0 bootloader.bin 0x8000 partition-table.bin 0x10000 joycon2_dongle.bin`.)
3. Unplug and replug the OTG port (no BOOT this time).

## Testing

`dist/joycon2-dongle-tool.html` is a standalone page (Chrome/Edge) that flashes
the firmware and, on its Logs tab, captures the dongle's log while you work
through the step's test list, then saves a report. The log port is also
readable with any serial terminal; boot messages go to the board's UART/COM
port at 115200 baud as well.

A gamepad tester such as <https://hardwaretester.com/gamepad> shows buttons,
sticks and has a rumble test for each mode. Every 10 s the log prints the input
reports/s per controller (~133/s is the target).

## Building

Requires ESP-IDF **v5.5.4** (the same version Switch2Connect ships with; its newer Bluetooth controller matters).

```
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

`esp_tinyusb` is fetched by the component manager. For an offline build, put
local copies of `esp_tinyusb` and `tinyusb` in one folder, then set
`DONGLE_EXTRA_COMPONENTS=<folder>` and `IDF_COMPONENT_MANAGER=0`.

## Source layout

| File | Purpose |
|---|---|
| `main/ble_central.c` | Scan, connect, GATT setup, Switch 2 init sequence, pairing, input |
| `main/ns2_proto.c/.h` | Protocol constants, input report parsing, stick calibration, rumble encoding |
| `main/pad.c/.h` | Roles, L+R merge, sideways single Joy-Con mapping |
| `main/usb_gamepad.c` | USB start-up, descriptors for the selected mode, report task, log mirror to CDC |
| `main/out_*.c` | Output modes: XInput, DS4, Switch Pro, generic HID |
| `main/rumble.c` | Converts PC rumble (motors or Switch HD rumble) to Switch 2 rumble frames |
| `main/settings.c` | Settings in NVS (mode, log port, rumble, layout, deadzone, hotkeys) |
| `main/config_server.c`, `main/web/index.html` | Web configurator: USB network (NCM), DHCP, web server, JSON API, page |
| `main/bonds.c` | Remembers paired controllers (MAC → product ID) in NVS |
| `main/main.c` | Start-up, BOOT-button "forget all" |

## Credits

Protocol knowledge comes from Switch2Connect (TommyWabg), Nadeflore/switch2-controllers,
ndeadly/switch2_controller_research, german77/JoyconDriver, TheFrano/joycon2py
and darthcloud/BlueRetro. The XInput, DS4 and Switch Pro descriptors and protocol
handling follow [GP2040-CE](https://github.com/OpenStickCommunity/GP2040-CE) (MIT);
the web configurator approach follows GP2040-CE too.
