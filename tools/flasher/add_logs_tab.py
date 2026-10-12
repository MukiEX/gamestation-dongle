t = open('template.html').read()
assert 'tab-logs' not in t

css = r'''
/* Tabs */
.tabs { display: flex; gap: 4px; border-bottom: 1px solid var(--line); }
.tab {
  border: 0; border-bottom: 2px solid transparent; border-radius: 0; background: none;
  padding: 10px 14px; margin-bottom: -1px; color: var(--muted); font-weight: 600;
}
.tab[aria-selected="true"] { color: var(--ink); border-bottom-color: var(--accent); }
.tabpanel { display: grid; gap: 20px; }

/* Log monitor */
.stats { display: flex; flex-wrap: wrap; gap: 8px; }
.stat { border: 1px solid var(--line); border-radius: 8px; padding: 6px 10px; min-width: 0; }
.stat b { display: block; font: 500 11px var(--font-body); letter-spacing: 0.06em; text-transform: uppercase; color: var(--muted); }
.stat span { font: 500 14px var(--font-mono); font-variant-numeric: tabular-nums; }
.tests { list-style: none; margin: 12px 0 0; padding: 0; display: grid; gap: 0; }
.test { display: grid; grid-template-columns: 2.2em 1fr auto auto; gap: 6px 10px; align-items: center; padding: 8px 0; border-top: 1px solid var(--line); }
.test:first-child { border-top: 0; }
.test .tn { font: 500 12.5px var(--font-mono); color: var(--muted); }
.test .tl { min-width: 0; }
.test.current .tn { color: var(--accent); }
.test.current .tl { font-weight: 600; }
.test select { font-size: 13px; padding: 4px 6px; }
.test button { padding: 5px 10px; font-size: 13px; }
.note { grid-column: 2 / -1; display: none; }
.test.current .note, .test.has-note .note { display: block; }
.note input { width: 100%; font: 13px var(--font-body); padding: 6px 8px; border-radius: 6px; border: 1px solid var(--line); background: var(--bg); color: var(--ink); }
.mon { height: 380px; }
.mon .w { color: #f0c36a; }
.mon .e { color: #ff8a80; }
.mon .mark { color: #8fc2ff; font-weight: 500; }
.mon .ts { color: #6b7686; }
@media (max-width: 560px) {
  .test { grid-template-columns: 2em 1fr; }
  .test select, .test button { grid-column: 2; justify-self: start; }
}
'''
t = t.replace('@media (prefers-reduced-motion', css + '@media (prefers-reduced-motion', 1)

t = t.replace('<title>Joy-Con 2 Dongle Flasher</title>', '<title>Joy-Con 2 Dongle Tool</title>')
t = t.replace('<h1>Joy-Con 2 Dongle Flasher</h1>', '<h1>Joy-Con 2 Dongle Tool</h1>')
t = t.replace('<p class="sub">Writes the dongle firmware to an ESP32-S3 N16R8 over USB, from this page. Works in Chrome, Edge or Opera on a desktop computer.</p>',
              '<p class="sub">Flash the dongle firmware to an ESP32-S3 N16R8, then capture its log while you test. Works in Chrome, Edge or Opera on a desktop computer.</p>')

nav = '''
  <nav class="tabs" role="tablist" aria-label="Tool">
    <button class="tab" role="tab" id="tab-btn-flash" aria-controls="tab-flash" aria-selected="true">Flash</button>
    <button class="tab" role="tab" id="tab-btn-logs" aria-controls="tab-logs" aria-selected="false">Logs and tests</button>
  </nav>

  <div class="tabpanel" id="tab-flash" role="tabpanel" aria-labelledby="tab-btn-flash">
  <section class="panel" aria-labelledby="fw-title">'''
assert '\n  <section class="panel" aria-labelledby="fw-title">' in t
t = t.replace('\n  <section class="panel" aria-labelledby="fw-title">', nav, 1)

logs = '''  </div>

  <div class="tabpanel" id="tab-logs" role="tabpanel" aria-labelledby="tab-btn-logs" hidden>
    <section class="panel" aria-labelledby="mon-title">
      <h2 id="mon-title">Connect to the dongle's log</h2>
      <p class="sub" style="margin-top:6px">Plug the dongle in normally, without BOOT. Pick <b>Joy-Con 2 Dongle (dev)</b>, its log port. The board's <b>UART/COM</b> port carries the same log and also catches crashes before USB starts.</p>
      <div class="row" style="margin-top:12px">
        <button id="mon-connect" class="primary">Connect</button>
        <button id="mon-disconnect" disabled>Disconnect</button>
        <button id="mon-reset" disabled title="UART/COM port only">Restart board</button>
        <span class="chip" id="mon-state">Not connected</span>
      </div>
      <div class="stats" style="margin-top:14px" aria-label="Live readings from the log">
        <div class="stat"><b>Firmware</b><span id="st-fw">–</span></div>
        <div class="stat"><b>Controllers</b><span id="st-ctl">none</span></div>
        <div class="stat"><b>Joy-Con L</b><span id="st-l">– /s</span></div>
        <div class="stat"><b>Joy-Con R</b><span id="st-r">– /s</span></div>
        <div class="stat"><b>Pro</b><span id="st-p">– /s</span></div>
        <div class="stat"><b>Interval</b><span id="st-itvl">–</span></div>
      </div>
    </section>

    <section class="panel" aria-labelledby="tests-title">
      <h2 id="tests-title">Step 1 tests</h2>
      <p class="sub" style="margin-top:6px">Press <b>Start</b> on a test before doing it, so the log is labelled. Pick the result, add a note if anything was off, then save the report.</p>
      <ol class="tests" id="tests"></ol>
    </section>

    <section aria-label="Dongle log" style="display:grid;gap:10px">
      <div class="row">
        <button id="mon-save" class="primary">Save report (.txt)</button>
        <button id="mon-copy">Copy</button>
        <button id="mon-clear">Clear log</button>
        <label class="row" style="gap:6px" for="mon-follow"><input type="checkbox" id="mon-follow" checked> Follow new lines</label>
        <span class="chip" id="mon-count">0 lines</span>
      </div>
      <pre class="log mon" id="mon" aria-live="off"></pre>
    </section>
  </div>

  <footer>'''
assert '\n  <footer>' in t
t = t.replace('\n  <footer>', '\n' + logs, 1)

t = t.replace('<script type="module">', '<script>\n' + open('monitor.js').read() + '\n</script>\n<script type="module">', 1)
open('template.html', 'w').write(t)
print("template updated")
