import base64, hashlib, re, sys
fw_path = sys.argv[1] if len(sys.argv) > 1 else '/home/claude/joycon2-dongle/dist/joycon2-dongle-0.1.1-merged.bin'
out = sys.argv[2] if len(sys.argv) > 2 else '/home/claude/joycon2-dongle/dist/joycon2-dongle-tool.html'
fw = open(fw_path, 'rb').read()
t = open('template.html').read()
t = (t.replace('__MD5_JS__', open('md5.js').read())
      .replace('__FW_B64__', base64.b64encode(fw).decode())
      .replace('__FW_MD5__', hashlib.md5(fw).hexdigest())
      .replace('__FW_NAME__', 'joycon2-dongle-0.1.1-merged.bin (0.1.1-step1)')
      .replace('__FW_SIZE__', f"{len(fw):,} bytes ({len(fw)/1024:.0f} KB)"))
left = re.findall(r'__[A-Z0-9_]+__', t)
assert not left, left
open(out, 'w').write(t)
scripts = re.findall(r'<script(?: type="module")?>(.*?)</script>', t, re.S)
for i, s in enumerate(scripts):
    open(f'check{i}.' + ('mjs' if i == len(scripts) - 1 else 'js'), 'w').write(s)
print(out, len(t), 'bytes,', len(scripts), 'scripts')
