// ---- Tabs ------------------------------------------------------------------
(function () {
  const tabs = [
    { btn: document.getElementById("tab-btn-flash"), panel: document.getElementById("tab-flash") },
    { btn: document.getElementById("tab-btn-logs"), panel: document.getElementById("tab-logs") },
  ];
  function show(i) {
    tabs.forEach((t, j) => {
      t.btn.setAttribute("aria-selected", String(i === j));
      t.panel.hidden = i !== j;
    });
  }
  tabs.forEach((t, i) => t.btn.addEventListener("click", () => show(i)));
  if (location.hash === "#logs") show(1);
})();

// ---- Log monitor and test report -----------------------------------------
(function () {
  const $ = (id) => document.getElementById(id);
  const TESTS = [
    "Dongle shows up as a gamepad + serial port; log shows version and BLE address",
    "Joy-Con L pairs from SYNC: buzz, player-1 LED, 'Joy-Con L connected'",
    "Joy-Con R pairs; log says the pair merged",
    "Every button and both sticks read correctly in a gamepad tester",
    "Input rate with one Joy-Con connected (wait for a reports/s line)",
    "Input rate with both Joy-Cons connected (wait for a reports/s line)",
    "Power a Joy-Con off, press a button: reconnects without SYNC and its buttons work in the tester",
    "Unplug and replug the dongle, press a button: reconnects",
    "Lone Joy-Con held sideways (turn the other one off first): stick and face buttons point the right way",
    "Second Joy-Con of the same side is ignored",
    "Pro Controller 2 pairs and blocks Joy-Cons",
    "Hold BOOT 5 s, then press buttons on both Joy-Cons: they do NOT reconnect (log says forgotten)",
  ];
  const RESULTS = ["–", "Pass", "Fail", "Partly", "Skipped"];

  const lines = [];          // {ts, text, cls}
  const MAX_DOM = 4000;
  let current = -1;
  const results = TESTS.map(() => ({ result: "–", note: "" }));

  // Build the test list.
  const list = $("tests");
  TESTS.forEach((label, i) => {
    const li = document.createElement("li");
    li.className = "test";
    li.innerHTML =
      '<span class="tn">' + String(i + 1).padStart(2, "0") + "</span>" +
      '<span class="tl"></span>' +
      '<select aria-label="Result"></select>' +
      '<button type="button">Start</button>' +
      '<div class="note"><input type="text" placeholder="Note (what you saw, numbers, anything odd)"></div>';
    li.querySelector(".tl").textContent = label;
    const sel = li.querySelector("select");
    sel.id = "test-result-" + i;
    RESULTS.forEach((r) => { const o = document.createElement("option"); o.textContent = r; sel.appendChild(o); });
    sel.addEventListener("change", () => {
      results[i].result = sel.value;
      mark("RESULT " + (i + 1) + ": " + sel.value);
    });
    const note = li.querySelector("input");
    note.id = "test-note-" + i;
    note.addEventListener("change", () => {
      results[i].note = note.value.trim();
      li.classList.toggle("has-note", !!results[i].note);
    });
    li.querySelector("button").addEventListener("click", () => startTest(i));
    list.appendChild(li);
  });

  function startTest(i) {
    current = i;
    [...list.children].forEach((li, j) => li.classList.toggle("current", j === i));
    mark("TEST " + (i + 1) + ": " + TESTS[i]);
    const input = list.children[i].querySelector("input");
    input.focus({ preventScroll: true });
  }

  // ---- Log view ----
  const mon = $("mon");
  const ANSI = /\x1b\[[0-9;]*m/g;
  function stamp(d) {
    return d.toTimeString().slice(0, 8) + "." + String(d.getMilliseconds()).padStart(3, "0");
  }
  function addLine(text, cls) {
    const ts = stamp(new Date());
    lines.push({ ts, text, cls });
    const row = document.createElement("span");
    if (cls) row.className = cls;
    const t = document.createElement("span");
    t.className = "ts";
    t.textContent = ts + "  ";
    row.appendChild(t);
    row.appendChild(document.createTextNode(text + "\n"));
    mon.appendChild(row);
    while (mon.childNodes.length > MAX_DOM) mon.removeChild(mon.firstChild);
    if ($("mon-follow").checked) mon.scrollTop = mon.scrollHeight;
    $("mon-count").textContent = lines.length.toLocaleString("en-US") + " lines";
  }
  function mark(text) { addLine("===== " + text + " =====", "mark"); }

  // ---- Live readings parsed from the log ----
  const present = new Set();
  function parse(text) {
    let m;
    if ((m = text.match(/Joy-Con 2 Dongle (\S+)/))) $("st-fw").textContent = m[1];
    if ((m = text.match(/\((Joy-Con L|Joy-Con R|Pro Controller)\): (\d+) input reports\/s/))) {
      const id = m[1] === "Joy-Con L" ? "st-l" : m[1] === "Joy-Con R" ? "st-r" : "st-p";
      $(id).textContent = m[2] + " /s";
    }
    if ((m = text.match(/interval ([\d.]+) ms/))) $("st-itvl").textContent = m[1] + " ms";
    if ((m = text.match(/(Joy-Con L|Joy-Con R|Pro Controller) connected/))) present.add(m[1]);
    if ((m = text.match(/(Joy-Con L|Joy-Con R|Pro Controller) disconnected/))) present.delete(m[1]);
    if (/pair merged/.test(text)) present.add("merged");
    if (/pair dissolved/.test(text)) present.delete("merged");
    const names = [...present].filter((n) => n !== "merged");
    $("st-ctl").textContent = names.length
      ? names.join(" + ") + (present.has("merged") ? " (merged)" : "")
      : "none";
  }

  // ---- Serial connection ----
  let port = null, reader = null, pipeDone = null, isUart = false;

  function setState(text, cls) {
    const s = $("mon-state");
    s.textContent = text;
    s.className = "chip" + (cls ? " " + cls : "");
  }

  async function readLoop() {
    const decoder = new TextDecoderStream();
    pipeDone = port.readable.pipeTo(decoder.writable).catch(() => {});
    reader = decoder.readable.getReader();
    let buf = "";
    try {
      for (;;) {
        const { value, done } = await reader.read();
        if (done) break;
        buf += value;
        let i;
        while ((i = buf.indexOf("\n")) >= 0) {
          const raw = buf.slice(0, i).replace(/\r$/, "").replace(ANSI, "");
          buf = buf.slice(i + 1);
          if (!raw) continue;
          const cls = /^E \(/.test(raw) ? "e" : /^W \(/.test(raw) ? "w" : "";
          addLine(raw, cls);
          parse(raw);
        }
      }
    } catch (err) {
      addLine("[serial read stopped: " + err.message + "]", "e");
    }
  }

  async function connect() {
    if (!("serial" in navigator)) {
      setState("USB serial not available in this browser", "warn");
      return;
    }
    let p;
    try {
      p = await navigator.serial.requestPort({});
    } catch (_) {
      return;  // chooser cancelled
    }
    const info = p.getInfo ? p.getInfo() : {};
    // Espressif's USB VID is the dongle's own log port (or the ROM in download
    // mode, PID 0x1001); anything else is the board's UART bridge chip.
    isUart = info.usbVendorId !== 0x303a;
    const isRom = info.usbVendorId === 0x303a && info.usbProductId === 0x1001;
    try {
      await p.open({ baudRate: 115200, bufferSize: 65536 });
    } catch (err) {
      setState("Couldn't open port", "warn");
      addLine("[couldn't open the port: " + err.message + ". If the Flash tab is connected to it, disconnect there first.]", "e");
      return;
    }
    port = p;
    try {
      // The dongle only sends its log once the host raises DTR. On a UART
      // bridge both lines low = run normally (no reset, no download mode).
      await port.setSignals(isUart ? { dataTerminalReady: false, requestToSend: false }
                                   : { dataTerminalReady: true, requestToSend: false });
    } catch (_) {}
    setState(isUart ? "Connected · UART/COM" : isRom ? "Connected · board is in download mode" : "Connected · dongle log port", isRom ? "warn" : "ok");
    if (isRom) addLine("[this is the chip's download mode, not the running firmware: replug the board without BOOT and connect again]", "w");
    mark("connected " + (isUart ? "to UART/COM" : "to dongle log port") + (info.usbProductId ? " (" + info.usbVendorId.toString(16) + ":" + info.usbProductId.toString(16) + ")" : ""));
    $("mon-connect").disabled = true;
    $("mon-disconnect").disabled = false;
    $("mon-reset").disabled = !isUart;
    readLoop().then(() => { if (port) disconnect(true); });
  }

  async function disconnect(lost) {
    const p = port;
    port = null;
    try { if (reader) await reader.cancel(); } catch (_) {}
    try { if (pipeDone) await pipeDone; } catch (_) {}
    try { if (p) await p.close(); } catch (_) {}
    reader = null; pipeDone = null;
    $("mon-connect").disabled = false;
    $("mon-disconnect").disabled = true;
    $("mon-reset").disabled = true;
    setState(lost ? "Port closed (dongle unplugged or restarted)" : "Not connected", lost ? "warn" : "");
    mark(lost ? "port lost" : "disconnected");
  }

  async function restartBoard() {
    if (!port || !isUart) return;
    mark("restarting board");
    try {
      await port.setSignals({ dataTerminalReady: false, requestToSend: true });   // EN low
      await new Promise((r) => setTimeout(r, 120));
      await port.setSignals({ dataTerminalReady: false, requestToSend: false });  // run
    } catch (err) {
      addLine("[restart failed: " + err.message + "]", "e");
    }
  }

  // ---- Report ----
  function report() {
    const out = [];
    out.push("Joy-Con 2 Dongle — step 1 test report");
    out.push("Saved: " + new Date().toString());
    out.push("Firmware: " + $("st-fw").textContent);
    out.push("Browser: " + navigator.userAgent);
    out.push("");
    out.push("Readings: Joy-Con L " + $("st-l").textContent + ", Joy-Con R " + $("st-r").textContent +
             ", Pro " + $("st-p").textContent + ", interval " + $("st-itvl").textContent);
    out.push("");
    out.push("Tests:");
    TESTS.forEach((t, i) => {
      out.push(String(i + 1).padStart(2, " ") + ". [" + results[i].result.padEnd(7) + "] " + t);
      if (results[i].note) out.push("      note: " + results[i].note);
    });
    out.push("");
    out.push("Log:");
    lines.forEach((l) => out.push(l.ts + "  " + l.text));
    return out.join("\n") + "\n";
  }

  $("mon-connect").addEventListener("click", connect);
  $("mon-disconnect").addEventListener("click", () => disconnect(false));
  $("mon-reset").addEventListener("click", restartBoard);
  $("mon-clear").addEventListener("click", () => {
    lines.length = 0;
    mon.textContent = "";
    $("mon-count").textContent = "0 lines";
  });
  $("mon-save").addEventListener("click", () => {
    const blob = new Blob([report()], { type: "text/plain" });
    const a = document.createElement("a");
    const d = new Date();
    a.href = URL.createObjectURL(blob);
    a.download = "dongle-step1-report-" + d.toISOString().slice(0, 16).replace(/[:T]/g, "-") + ".txt";
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 5000);
  });
  $("mon-copy").addEventListener("click", async () => {
    const btn = $("mon-copy");
    try {
      await navigator.clipboard.writeText(report());
      btn.textContent = "Copied";
    } catch (_) {
      const r = document.createRange();
      r.selectNodeContents(mon);
      const sel = getSelection();
      sel.removeAllRanges();
      sel.addRange(r);
      btn.textContent = "Selected — press Ctrl+C";
    }
    setTimeout(() => (btn.textContent = "Copy"), 2500);
  });
  if (navigator.serial) {
    navigator.serial.addEventListener("disconnect", (e) => {
      if (port && e.target === port) disconnect(true);
    });
  }
})();
