"""Builds the single-file tool page with the firmware images embedded.

usage: python3 build.py <version> [output.html]
Embeds dist/joycon2-dongle-<version>-merged.bin (normal) and, if present,
dist/joycon2-dongle-<version>-diag-merged.bin (diagnostic), plus a zip of the
source they were built from (the repository at HEAD, which must be committed).
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

# The exact source the images were built from: the repository at HEAD (minus
# the built files in dist/), which must have no uncommitted changes.
import subprocess
repo = '/home/claude/joycon2-dongle'
dirty = subprocess.run(['git', 'status', '--porcelain', '--', '.', ':(exclude)dist'], cwd=repo,
                       capture_output=True, text=True, check=True).stdout.strip()
assert not dirty, 'commit the source first; uncommitted changes:\n' + dirty
commit = subprocess.run(['git', 'rev-parse', '--short=10', 'HEAD'], cwd=repo,
                        capture_output=True, text=True, check=True).stdout.strip()
src = subprocess.run(['git', 'archive', '--format=zip', '-9', f'--prefix=joycon2-dongle-{ver}/', 'HEAD',
                      '--', '.', ':(exclude)dist'], cwd=repo, capture_output=True, check=True).stdout
src_name = f'joycon2-dongle-{ver}-src.zip'

first = images[0]
t = open('template.html').read()
t = (t.replace('__MD5_JS__', open('md5.js').read())
      .replace('__IMAGES_JSON__', json.dumps(images))
      .replace('__FW_NAME__', first['name'])
      .replace('__FW_MD5__', first['md5'])
      .replace('__SRC_B64__', base64.b64encode(src).decode())
      .replace('__SRC_NAME__', src_name)
      .replace('__SRC_COMMIT__', commit)
      .replace('__SRC_SIZE__', f"{len(src) / 1024:.0f} KB")
      .replace('__FW_SIZE__', f"{first['size']:,} bytes ({first['size'] / 1024:.0f} KB)"))
# Usage diagrams (docs/img), inlined so the page stays one file.
def svg(name):
    s = open(os.path.join(repo, 'docs', 'img', name)).read()
    return re.sub(r'\s(width|height)="\d+"', '', s, count=2)
t = (t.replace('__SVG_SHORTCUTS__', svg('shortcuts.svg'))
      .replace('__SVG_BOOT__', svg('boot-button.svg'))
      .replace('__SVG_MODES__', svg('modes.svg')))
left = re.findall(r'__[A-Z0-9_]+__', t)
assert not left, left
open(out, 'w').write(t)

scripts = re.findall(r'<script(?: type="module")?>(.*?)</script>', t, re.S)
# Each script on its own, for a syntax check (node --check); kept out of the repository.
import tempfile
chk = os.path.join(tempfile.gettempdir(), 'joycon2-tool-check')
os.makedirs(chk, exist_ok=True)
for i, s in enumerate(scripts):
    open(os.path.join(chk, f'check{i}.' + ('mjs' if i == len(scripts) - 1 else 'js')), 'w').write(s)
print(out, len(t), 'bytes;', ', '.join(im['name'] for im in images), '+', src_name, 'at', commit)
