# GSX Explorer

The tools used to work out the Gamestation Gamepad's Bluetooth protocol. Not needed to use the dongle.

* `GSX_Explorer/GSX_Explorer.ino`: Arduino sketch for a plain ESP32 (or ESP32-S3) that connects to the pad
  as a BLE central, bonds, dumps the whole GATT table and report map, subscribes to everything that
  notifies and streams it all over USB serial (115200) as one line per event. Needs the Arduino-ESP32 core
  3.x and NimBLE-Arduino **2.x** (checked against 2.5.2; 3.0 renames a call). Writes report the ATT status.
* `gsx_explorer.html`: the Web Serial page for it (Chrome / Edge, open from disk). Tabs:
  * **Explorer**: devices, GATT tree with read / write / subscribe, parsed report map, live byte diffs,
    a working diagram of the pad, save log, and flashing a built `.bin` over the same port (esptool-js).
  * **Protocol**: the write-up of everything found (reports, HID usages, GATT table, quirks, the
    SmartGlow light numbering and the Go's light report).
  * **Lights probe**: test patterns for the pad's vendor service FFF0 (never the OTA service FF12),
    presets from the Go's light numbering, and an auto-reply to the pad's power-on hello.

## Build and flash the sketch

```
arduino-cli lib install "NimBLE-Arduino@2.5.2"
arduino-cli compile -b esp32:esp32:esp32 --export-binaries GSX_Explorer
```

then flash `GSX_Explorer.ino.merged.bin` at 0x0 from the page's Flash panel (later updates:
`GSX_Explorer.ino.bin` at 0x10000, which keeps the bond).

## Serial protocol

ESP32 to host, one line each: `H` hello/status, `D` scan result, `S`/`C`/`d` services, characteristics,
descriptors, `M` report map, `V` value read, `N` subscribe result, `R <ms> <handle> <report id> <hex>`
notification, `w <handle> <ok> <status>` write result, `P` connection info, `A` pairing, `X` disconnect,
`I`/`W`/`E` info, warning, scan end. Host to ESP32: `hello`, `scan [s]`, `stop`, `connect <addr> [type]`,
`disconnect`, `dump`, `info`, `read <h>`, `write <h> <hex> [nr]`, `sub <h>`, `unsub <h>`,
`conn <min> <max>`, `auto <0|1>`, `forget`. Details at the top of the sketch.

## SmartGlow status

The pad accepts any write to FFF1 and to `0783b03e-8535-b5a0-7140-a304d2495cb8`, and none of the encodings
tried changed its lights, including the 8-byte report `5a 01 01 HI LO 00 00 00` that the Gamestation Go's
Stella writes to `/dev/hidraw1`. After a power-on the pad sends `66`×10 on `0783b03e…` three times, which
looks like a handshake nothing here answers yet. Capturing a Go talking to the pad would settle it.
