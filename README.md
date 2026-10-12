# Joy-Con 2 Dongle

Firmware for an **ESP32-S3 N16R8** that connects Nintendo Switch 2 controllers
over Bluetooth LE and presents them to a PC/console as **one USB controller**.
A left and a right Joy-Con 2 automatically merge into a single controller.

Based on [Switch2Connect](https://github.com/TommyWabg/Switch2Connect) by
TommyWabg, so this project is **GPL-3.0**.

## Status: step 1 of 5

| Step | What | State |
|---|---|---|
| 1 | Joy-Con 2 / Pro Controller 2 pairing + L+R merge → generic USB HID gamepad, serial debug log | **this build** |
| 2 | XInput, DS4, Switch Pro output modes + rumble | next |
| 3 | Web configurator at 192.168.7.1 (hold a button at boot), settings in flash | |
| 4 | Gyro, DualSense output with audio haptics, Switch 2 Pro output | |
| 5 | Steam Controller (2026) mode with mouse-sensor trackpads | |

### What step 1 does

* Scans for Switch 2 controllers. Supported: **Joy-Con 2 (L)**, **Joy-Con 2 (R)**,
  **Pro Controller 2**. (NSO GameCube controller: not yet.)
* **Pairing:** a controller in SYNC mode (hold the small sync button) is
  connected and paired to the dongle. After that, pressing any button on it
  reconnects to the dongle.
* **Merging:** one left + one right Joy-Con become one controller. A pair stays a
  pair until both halves are gone; if one half drops, its inputs read neutral
  and it rejoins when it reconnects.
* **One controller per role:** a second Joy-Con of the same side, or any Joy-Con
  while a Pro Controller is connected, is ignored.
* **Lone Joy-Con:** used sideways as a mini-controller.
* Each controller gives a short buzz and shows player-1 LED when ready.
* **Forget all pairings:** hold the board's **BOOT** button for 5 seconds while
  it is running.

### USB output (step 1)

A generic HID gamepad (VID:PID `303A:8243`, "Joy-Con 2 Dongle") plus a
serial port that mirrors the log.

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

D-pad = hat switch. Left stick = X/Y, right stick = Z/Rz. ZL/ZR also appear as
Rx/Ry axes (0 or 255; Joy-Con triggers are digital).

## Flashing

Prebuilt images are in `dist/`. You need [esptool](https://github.com/espressif/esptool)
(`pip install esptool`) or Espressif's web flasher.

1. Hold **BOOT**, plug the board's **USB/OTG** port into the PC, release BOOT.
2. Flash the single merged image:

   ```
   esptool.py --chip esp32s3 -p <PORT> -b 921600 write_flash 0x0 dist/joycon2-dongle-0.1.4-merged.bin
   ```

   (Or the separate files: `0x0 bootloader.bin 0x8000 partition-table.bin 0x10000 joycon2_dongle.bin`.)
3. Unplug and replug the OTG port (no BOOT this time).

## Testing step 1

1. Open the dongle's **serial port** (the new COM port / `/dev/ttyACM*`) in any
   terminal at any baud rate. You should see
   `Joy-Con 2 Dongle 0.1.4-step1` and `dongle BLE address ...`.
   Logs also come out of the board's **UART/COM** USB-C port at 115200 baud,
   which is useful if the dongle crashes before USB starts.
2. Hold **SYNC** on a Joy-Con 2. Expect, in order:
   `connecting to Joy-Con L ... (pairing)`, `connected`, `MTU`, `product 2067`,
   `paired to this dongle`, a buzz, then `Joy-Con L connected`.
3. Repeat with the other Joy-Con → `Joy-Con pair merged into one controller`.
4. Check the inputs in a gamepad tester: Windows *Set up USB game controllers*
   (`joy.cpl`), `jstest-gtk`/`evtest` on Linux, or <https://hardwaretester.com/gamepad>.
5. Every 10 s the log prints `input reports/s` per controller. ~133/s is the
   target; please report what you see.
6. Turn the Joy-Cons off, then press a button: they should reconnect without SYNC.

**Please send back the log** (from power-on through connecting) whenever
something doesn't work. Things worth checking specifically:

* stick directions (especially a single sideways Joy-Con — the rotation and
  face-button mapping are derived, not yet verified on hardware);
* whether reconnect-by-button-press works after unplugging the dongle;
* the reports/s figure with one and with two Joy-Cons connected.

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
| `main/usb_gamepad.c` | USB descriptors, HID report, log mirror to CDC |
| `main/bonds.c` | Remembers paired controllers (MAC → product ID) in NVS |
| `main/main.c` | Start-up, BOOT-button "forget all" |

## Credits

Protocol knowledge comes from Switch2Connect (TommyWabg), Nadeflore/switch2-controllers,
ndeadly/switch2_controller_research, german77/JoyconDriver, TheFrano/joycon2py
and darthcloud/BlueRetro. The web configurator approach follows GP2040-CE.
