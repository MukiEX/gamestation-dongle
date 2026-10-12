#!/usr/bin/env bash
# Builds the merged flash images in dist/. Uses --flash_mode keep so each
# image keeps the header ESP-IDF built (patching the bootloader header to QIO
# makes boards whose flash has QIO disabled reset in a loop before the app runs).
set -euo pipefail
cd "$(dirname "$0")/.."
VER="${1:?usage: make_images.sh <version>}"
merge() {  # $1 = build dir, $2 = output name
  python -m esptool --chip esp32s3 merge_bin -o "dist/$2" \
    --flash_mode keep --flash_freq keep --flash_size 16MB \
    0x0 "$1/bootloader/bootloader.bin" 0x8000 "$1/partition_table/partition-table.bin" \
    0x10000 "$1/joycon2_dongle.bin" >/dev/null
  python3 - "dist/$2" <<'PY'
import sys
d = open(sys.argv[1], 'rb').read(4)
assert d[0] == 0xE9, "bad image magic"
assert d[2] == 2, f"bootloader header flash mode is {d[2]}, expected DIO (2)"
print(sys.argv[1], "ok: DIO header")
PY
}
rm -f dist/*.bin
merge build "joycon2-dongle-$VER-merged.bin"
[ -d build-diag ] && merge build-diag "joycon2-dongle-$VER-diag-merged.bin"
