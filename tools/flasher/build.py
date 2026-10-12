"""Builds the single-file tool page with the firmware images embedded.

usage: python3 build.py <version> [output.html]
Embeds dist/joycon2-dongle-<version>-merged.bin (normal) and, if present,
dist/joycon2-dongle-<version>-diag-merged.bin (diagnostic).
"""
import base64, hashlib, json, os, re, sys

ver = sys.argv[1] if len(sys.argv) > 1 else '0.1.2'
dist = '/home/claude/joycon2-dongle/dist'
out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(dist, 'joycon2-dongle-tool.html')

images = []
for suffix, label in (('', ''), ('-diag', ' (diagnostic)')):
    path = os.path.join(dist, f'joycon2-dongle-{ver}{suffix}-merged.bin')
    if not os.path.exists(path):
        continue
    fw = open(path, 'rb').read()
    assert fw[0] == 0xE9 and fw[2] == 2, f'{path}: bootloader header must be DIO'
    images.append({
        'name': os.path.basename(path),
        'b64': base64.b64encode(fw).decode(),
        'md5': hashlib.md5(fw).hexdigest(),
        'size': len(fw),
    })
assert images, 'no images found'

first = images[0]
t = open('template.html').read()
t = (t.replace('__MD5_JS__', open('md5.js').read())
      .replace('__IMAGES_JSON__', json.dumps(images))
      .replace('__FW_NAME__', first['name'])
      .replace('__FW_MD5__', first['md5'])
      .replace('__FW_SIZE__', f"{first['size']:,} bytes ({first['size'] / 1024:.0f} KB)"))
left = re.findall(r'__[A-Z0-9_]+__', t)
assert not left, left
open(out, 'w').write(t)

scripts = re.findall(r'<script(?: type="module")?>(.*?)</script>', t, re.S)
for i, s in enumerate(scripts):
    open(f'check{i}.' + ('mjs' if i == len(scripts) - 1 else 'js'), 'w').write(s)
print(out, len(t), 'bytes;', ', '.join(im['name'] for im in images))
