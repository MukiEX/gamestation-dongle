#!/usr/bin/env bash
# Builds the merged flash images in dist/. Uses --flash_mode keep so each
# image keeps the header ESP-IDF built (patching the bootloader header to QIO
# makes boards whose flash has QIO disabled reset in a loop before the app runs).
set -euo pipefail
cd "$(dirname "$0")/.."
VER="${1:?usage: make_images.sh <version>}"
# Each image must carry the right version string: "<ver>" (pre-1.0 builds:
# "<ver>-stepN") for the normal build, "...-diag" for the diagnostic one (catches a diagnostic build
# dir that lost its DONGLE_DIAG switch).
check_version() {  # $1 = app binary, $2 = "diag" or "normal"
  python3 - "$1" "$2" "$VER" <<'PY'
import re, sys
data = open(sys.argv[1], 'rb').read()
want_diag = sys.argv[2] == 'diag'
found = set(m.decode() for m in re.findall(rb'\b' + re.escape(sys.argv[3].encode()) + rb'(?:-step[0-9]+)?(?:-diag|-tdongle|-lcd147)?(?![\w.])', data))
assert found, f"{sys.argv[1]}: no {sys.argv[3]} version string (stale build?)"
want = {'diag': '-diag', 'tdongle': '-tdongle', 'lcd147': '-lcd147'}.get(sys.argv[2], '')
ok = [v for v in found if v.endswith(('-diag', '-tdongle', '-lcd147')) == bool(want) and v.endswith(want)]
assert ok, f"{sys.argv[1]}: version {sorted(found)} is not a {sys.argv[2]} build"
print(sys.argv[1], 'is', ok[0])
PY
}
merge() {  # $1 = build dir, $2 = output name
  python -m esptool --chip esp32s3 merge_bin -o "dist/$2" \
    --flash_mode keep --flash_freq keep --flash_size 16MB \
    0x0 "$1/bootloader/bootloader.bin" 0x8000 "$1/partition_table/partition-table.bin" \
    0x10000 "$1/gamestation_dongle.bin" >/dev/null
  python3 - "dist/$2" <<'PY'
import sys
d = open(sys.argv[1], 'rb').read(4)
assert d[0] == 0xE9, "bad image magic"
assert d[2] == 2, f"bootloader header flash mode is {d[2]}, expected DIO (2)"
print(sys.argv[1], "ok: DIO header")
PY
}
# Always rebuild first: an image linked before the last source edit (the
# configurator page in particular) once shipped as 0.6.7.
ninja -C build >/dev/null || { echo "normal build failed"; exit 1; }
if [ -d build-diag ]; then ninja -C build-diag >/dev/null || { echo "diagnostic build failed"; exit 1; }; fi
if [ -d build-tdongle ]; then ninja -C build-tdongle >/dev/null || { echo "T-Dongle-S3 build failed"; exit 1; }; fi
if [ -d build-lcd147 ]; then ninja -C build-lcd147 >/dev/null || { echo "ESP32-S3-LCD-1.47 build failed"; exit 1; }; fi
python3 - build/gamestation_dongle.bin main/web/index.html <<'PY'
import sys
app, page = (open(p, 'rb').read() for p in sys.argv[1:])
assert page in app, "build/gamestation_dongle.bin does not contain the current main/web/index.html"
print("configurator page in the image is current")
PY
rm -f dist/*.bin
check_version build/gamestation_dongle.bin normal
merge build "gamestation-dongle-$VER-merged.bin"
if [ -d build-diag ]; then
  check_version build-diag/gamestation_dongle.bin diag
  merge build-diag "gamestation-dongle-$VER-diag-merged.bin"
fi
if [ -d build-tdongle ]; then
  check_version build-tdongle/gamestation_dongle.bin tdongle
  merge build-tdongle "gamestation-dongle-$VER-tdongle-merged.bin"
fi
if [ -d build-lcd147 ]; then
  check_version build-lcd147/gamestation_dongle.bin lcd147
  merge build-lcd147 "gamestation-dongle-$VER-lcd147-merged.bin"
fi
