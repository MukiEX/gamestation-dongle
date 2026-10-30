/*
  GSX Explorer — BLE explorer firmware for the My Arcade Atari Gamestation Gamepad
  ------------------------------------------------------------------------------
  Target : classic ESP32 (Elegoo ESP32 dev board -> Arduino board "ESP32 Dev Module")
  Needs  : Arduino-ESP32 core 3.x
           NimBLE-Arduino 2.x (Library Manager: "NimBLE-Arduino" by h2zero;
           checked against 2.5.2 — the unreleased 3.0 renames setScanCallbacks)
  Serial : 115200 8N1, newline-terminated text lines (protocol below)

  The ESP32 acts as a BLE central: scans, connects, bonds ("just works"),
  dumps the whole GATT table, reads the HID Report Map and every readable
  characteristic, subscribes to every notify/indicate characteristic and
  streams each notification as one hex line. Bonded address is saved in NVS
  and auto-reconnected when the pad advertises again.

  ESP32 -> host lines
    H <fw> saved=<addr|-> auto=<0|1> link=<0|1>    hello / status
    I <text>          info            W <text>  warning      E <text> scan end
    D <addr> <type> <rssi> <hid 0|1> <appearance hex> <name...>   scan result
    S <svc uuid>                                     service
    C <svc uuid> <chr uuid> <handle> <props RWwNI>   characteristic
    d <chr handle> <desc uuid> [id=<n> type=<n>]     descriptor (+Report Reference)
    M <handle> <hex>                                 HID Report Map
    V <handle> <hex>                                 characteristic value (read)
    N <handle> <ok 0|1>                              subscribe result
    R <ms> <handle> <report id|-1> <hex>             notification
    w <handle> <ok 0|1> <status>                     write result (status: 0, or NimBLE / ATT error;
                                                     0x100 + n = ATT error n, e.g. 0x10d invalid length)
    P itvl=.. lat=.. to=.. mtu=.. enc=.. bonded=..   connection info
    A enc=.. bonded=.. auth=.. key=..                pairing result
    X <reason>                                       disconnected

  host -> ESP32 commands
    hello | scan [seconds, 0=forever] | stop | connect <addr> [type]
    disconnect | dump | info | read <h> | write <h> <hex> [nr]
    sub <h> | unsub <h> | conn <min> <max> (1.25 ms units) | auto <0|1> | forget
*/

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <map>
#include <string>
#include <vector>

#define FW_VERSION "gsx-0.2"
#define BAUD 115200

struct ReportMsg {
  uint32_t ms;
  uint16_t handle;
  uint8_t len;
  uint8_t data[64];
};

static QueueHandle_t reportQ;
static SemaphoreHandle_t seenMtx;
static std::map<std::string, NimBLEAddress> seen;
static std::map<uint16_t, int> reportIdByHandle;
static std::map<uint16_t, NimBLERemoteCharacteristic*> chrByHandle;

static NimBLEClient* client = nullptr;
static Preferences prefs;
static std::string savedAddr;
static uint8_t savedType = 0;
static bool autoConnect = true;

static volatile bool flagStopScan = false;
static volatile bool flagConnect = false;
static volatile bool flagLinkDown = false;
static volatile uint32_t dropped = 0;
static NimBLEAddress pendingAddr;
static String lineBuf;
static SemaphoreHandle_t writeSem;
static volatile int writeStatus;

// ---------------------------------------------------------------- helpers
static void printHex(const uint8_t* d, size_t n) {
  static const char* h = "0123456789abcdef";
  for (size_t i = 0; i < n; i++) {
    Serial.write(h[d[i] >> 4]);
    Serial.write(h[d[i] & 15]);
  }
}

static std::string lower(std::string s) {
  for (auto& c : s) c = (char)tolower((unsigned char)c);
  return s;
}

static bool linkUp() { return client && client->isConnected(); }

static NimBLERemoteCharacteristic* findChr(int h) {
  auto it = chrByHandle.find((uint16_t)h);
  if (it == chrByHandle.end()) {
    Serial.printf("W no_chr %d\n", h);
    return nullptr;
  }
  return it->second;
}

// ---------------------------------------------------------------- callbacks
// Raw GATT write so the ATT error code reaches the host (NimBLE-Arduino's
// writeValue() only logs it).
static int onWriteDone(uint16_t conn, const struct ble_gatt_error* err, struct ble_gatt_attr* attr, void* arg) {
  writeStatus = err->status;
  xSemaphoreGive(writeSem);
  return 0;
}
static void onNotify(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool isNotify) {
  ReportMsg m;
  m.ms = millis();
  m.handle = chr->getHandle();
  m.len = len > sizeof(m.data) ? sizeof(m.data) : (uint8_t)len;
  memcpy(m.data, data, m.len);
  if (xQueueSend(reportQ, &m, 0) != pdTRUE) dropped++;
}

class ScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* d) override {
    const NimBLEAddress& addr = d->getAddress();
    std::string a = addr.toString();
    bool isNew;
    xSemaphoreTake(seenMtx, portMAX_DELAY);
    isNew = seen.find(a) == seen.end();
    seen[a] = addr;
    if (autoConnect && !savedAddr.empty() && a == savedAddr && !flagConnect) {
      pendingAddr = addr;
      flagConnect = true;
      flagStopScan = true;
    }
    xSemaphoreGive(seenMtx);
    if (isNew) {
      bool hid = d->isAdvertisingService(NimBLEUUID((uint16_t)0x1812));
      uint16_t app = d->haveAppearance() ? d->getAppearance() : 0;
      Serial.printf("D %s %u %d %d %04x %s\n", a.c_str(), addr.getType(), d->getRSSI(),
                    hid ? 1 : 0, app, d->getName().c_str());
    }
  }
  void onScanEnd(const NimBLEScanResults& results, int reason) override {
    Serial.printf("E scan_end %d\n", reason);
  }
} scanCB;

class ClientCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* c) override {
    Serial.printf("I link_up %s\n", c->getPeerAddress().toString().c_str());
  }
  void onDisconnect(NimBLEClient* c, int reason) override {
    Serial.printf("X %d\n", reason);
    flagLinkDown = true;
  }
  void onAuthenticationComplete(NimBLEConnInfo& info) override {
    Serial.printf("A enc=%d bonded=%d auth=%d key=%d\n", info.isEncrypted(), info.isBonded(),
                  info.isAuthenticated(), info.getSecKeySize());
  }
  void onConfirmPasskey(NimBLEConnInfo& info, uint32_t pin) override {
    NimBLEDevice::injectConfirmPasskey(info, true);
  }
  void onPassKeyEntry(NimBLEConnInfo& info) override { NimBLEDevice::injectPassKey(info, 0); }
  bool onConnParamsUpdateRequest(NimBLEClient* c, const ble_gap_upd_params* p) override {
    Serial.printf("I peer_param_req min=%u max=%u lat=%u to=%u\n", p->itvl_min, p->itvl_max,
                  p->latency, p->supervision_timeout);
    return true;
  }
} clientCB;

// ---------------------------------------------------------------- actions
static void startScan(uint32_t ms) {
  NimBLEScan* s = NimBLEDevice::getScan();
  if (s->isScanning()) s->stop();
  xSemaphoreTake(seenMtx, portMAX_DELAY);
  seen.clear();
  xSemaphoreGive(seenMtx);
  bool ok = s->start(ms, false, true);
  Serial.printf("I scan_start %lu %d\n", (unsigned long)ms, ok ? 1 : 0);
}

static void printConnInfo() {
  if (!linkUp()) {
    Serial.println("P down");
    return;
  }
  NimBLEConnInfo ci = client->getConnInfo();
  Serial.printf("P itvl=%u lat=%u to=%u mtu=%u enc=%d bonded=%d\n", ci.getConnInterval(),
                ci.getConnLatency(), ci.getConnTimeout(), ci.getMTU(), ci.isEncrypted(),
                ci.isBonded());
}

static void dumpAndSubscribe() {
  reportIdByHandle.clear();
  chrByHandle.clear();

  const std::vector<NimBLERemoteService*>& svcs = client->getServices(true);
  Serial.printf("I services %u\n", (unsigned)svcs.size());
  for (auto* s : svcs) {
    std::string su = s->getUUID().toString();
    Serial.printf("S %s\n", su.c_str());
    const std::vector<NimBLERemoteCharacteristic*>& chrs = s->getCharacteristics(true);
    for (auto* c : chrs) {
      uint16_t h = c->getHandle();
      chrByHandle[h] = c;
      char props[8];
      int p = 0;
      if (c->canRead()) props[p++] = 'R';
      if (c->canWrite()) props[p++] = 'W';
      if (c->canWriteNoResponse()) props[p++] = 'w';
      if (c->canNotify()) props[p++] = 'N';
      if (c->canIndicate()) props[p++] = 'I';
      props[p] = 0;
      Serial.printf("C %s %s %u %s\n", su.c_str(), c->getUUID().toString().c_str(), h,
                    p ? props : "-");
      const std::vector<NimBLERemoteDescriptor*>& ds = c->getDescriptors(true);
      for (auto* d : ds) {
        if (d->getUUID() == NimBLEUUID((uint16_t)0x2908)) {
          NimBLEAttValue v = d->readValue();
          if (v.size() >= 2) {
            reportIdByHandle[h] = v.data()[0];
            Serial.printf("d %u %s id=%u type=%u\n", h, d->getUUID().toString().c_str(),
                          v.data()[0], v.data()[1]);
            continue;
          }
        }
        Serial.printf("d %u %s\n", h, d->getUUID().toString().c_str());
      }
    }
  }

  // Read everything readable (Report Map gets its own tag).
  for (auto& kv : chrByHandle) {
    NimBLERemoteCharacteristic* c = kv.second;
    if (!c->canRead()) continue;
    NimBLEAttValue v = c->readValue();
    bool isMap = c->getUUID() == NimBLEUUID((uint16_t)0x2A4B);
    Serial.printf("%s %u ", isMap ? "M" : "V", kv.first);
    printHex(v.data(), v.size());
    Serial.println();
  }

  // Subscribe to every notify / indicate characteristic, vendor ones included.
  for (auto& kv : chrByHandle) {
    NimBLERemoteCharacteristic* c = kv.second;
    if (!(c->canNotify() || c->canIndicate())) continue;
    bool ok = c->subscribe(c->canNotify(), onNotify, true);
    Serial.printf("N %u %d\n", kv.first, ok ? 1 : 0);
  }
  Serial.println("I dump_done");
}

static void doConnect(NimBLEAddress addr) {
  NimBLEScan* s = NimBLEDevice::getScan();
  if (s->isScanning()) s->stop();
  if (!client) {
    client = NimBLEDevice::createClient();
    client->setClientCallbacks(&clientCB, false);
    client->setConnectTimeout(10000);
  }
  if (client->isConnected()) {
    Serial.println("W already_connected");
    return;
  }
  // Ask for 7.5-15 ms. The pad may counter-propose (logged as peer_param_req).
  client->setConnectionParams(6, 12, 0, 300);
  Serial.printf("I connecting %s %u\n", addr.toString().c_str(), addr.getType());
  if (!client->connect(addr)) {
    Serial.printf("W connect_failed %d\n", client->getLastError());
    if (autoConnect && !savedAddr.empty()) startScan(0);
    return;
  }
  if (!client->secureConnection()) Serial.println("W secure_failed");

  xSemaphoreTake(seenMtx, portMAX_DELAY);
  savedAddr = addr.toString();
  savedType = addr.getType();
  xSemaphoreGive(seenMtx);
  prefs.putString("addr", savedAddr.c_str());
  prefs.putUChar("type", savedType);
  Serial.printf("I saved %s\n", savedAddr.c_str());

  printConnInfo();
  dumpAndSubscribe();
}

static void sayHello() {
  Serial.printf("H %s saved=%s auto=%d link=%d\n", FW_VERSION,
                savedAddr.empty() ? "-" : savedAddr.c_str(), autoConnect ? 1 : 0,
                linkUp() ? 1 : 0);
}

// ---------------------------------------------------------------- commands
static void handleCmd(String line) {
  line.trim();
  if (!line.length()) return;
  std::vector<String> t;
  int i = 0, n = line.length();
  while (i < n) {
    while (i < n && line[i] == ' ') i++;
    int j = i;
    while (j < n && line[j] != ' ') j++;
    if (j > i) t.push_back(line.substring(i, j));
    i = j;
  }
  if (t.empty()) return;
  String cmd = t[0];
  cmd.toLowerCase();

  if (cmd == "hello") {
    sayHello();
  } else if (cmd == "info") {
    sayHello();
    printConnInfo();
  } else if (cmd == "scan") {
    uint32_t sec = t.size() > 1 ? (uint32_t)t[1].toInt() : 10;
    startScan(sec * 1000);
  } else if (cmd == "stop") {
    NimBLEDevice::getScan()->stop();
  } else if (cmd == "connect" && t.size() > 1) {
    std::string a = lower(std::string(t[1].c_str()));
    NimBLEAddress addr;
    bool found = false;
    xSemaphoreTake(seenMtx, portMAX_DELAY);
    auto it = seen.find(a);
    if (it != seen.end()) {
      addr = it->second;
      found = true;
    }
    xSemaphoreGive(seenMtx);
    if (!found) addr = NimBLEAddress(a, t.size() > 2 ? (uint8_t)t[2].toInt() : 0);
    doConnect(addr);
  } else if (cmd == "disconnect") {
    if (linkUp()) client->disconnect();
    else Serial.println("W not_connected");
  } else if (cmd == "dump") {
    if (linkUp()) dumpAndSubscribe();
    else Serial.println("W not_connected");
  } else if (cmd == "read" && t.size() > 1) {
    NimBLERemoteCharacteristic* c = findChr(t[1].toInt());
    if (c) {
      NimBLEAttValue v = c->readValue();
      Serial.printf("V %u ", c->getHandle());
      printHex(v.data(), v.size());
      Serial.println();
    }
  } else if (cmd == "write" && t.size() > 2) {
    NimBLERemoteCharacteristic* c = findChr(t[1].toInt());
    if (c) {
      std::vector<uint8_t> buf;
      String hx = t[2];
      for (int k = 0; k + 1 < (int)hx.length(); k += 2)
        buf.push_back((uint8_t)strtoul(hx.substring(k, k + 2).c_str(), nullptr, 16));
      bool resp = !(t.size() > 3 && t[3] == "nr");
      int status;
      if (!linkUp()) {
        status = BLE_HS_ENOTCONN;
      } else if (resp) {
        xSemaphoreTake(writeSem, 0);
        status = ble_gattc_write_flat(client->getConnHandle(), c->getHandle(), buf.data(), buf.size(),
                                      onWriteDone, nullptr);
        if (status == 0) {
          status = xSemaphoreTake(writeSem, pdMS_TO_TICKS(3000)) == pdTRUE ? writeStatus : BLE_HS_ETIMEOUT;
        }
      } else {
        status = ble_gattc_write_no_rsp_flat(client->getConnHandle(), c->getHandle(), buf.data(), buf.size());
      }
      Serial.printf("w %u %d %d\n", c->getHandle(), status == 0 ? 1 : 0, status);
    }
  } else if (cmd == "sub" && t.size() > 1) {
    NimBLERemoteCharacteristic* c = findChr(t[1].toInt());
    if (c) Serial.printf("N %u %d\n", c->getHandle(), c->subscribe(c->canNotify(), onNotify, true) ? 1 : 0);
  } else if (cmd == "unsub" && t.size() > 1) {
    NimBLERemoteCharacteristic* c = findChr(t[1].toInt());
    if (c) Serial.printf("N %u %d\n", c->getHandle(), c->unsubscribe(true) ? 0 : 1);
  } else if (cmd == "conn" && t.size() > 2) {
    if (linkUp()) {
      client->updateConnParams((uint16_t)t[1].toInt(), (uint16_t)t[2].toInt(), 0, 300);
      Serial.println("I conn_param_requested (send 'info' to check)");
    } else {
      Serial.println("W not_connected");
    }
  } else if (cmd == "auto" && t.size() > 1) {
    autoConnect = t[1].toInt() != 0;
    prefs.putBool("auto", autoConnect);
    sayHello();
  } else if (cmd == "forget") {
    if (linkUp()) client->disconnect();
    NimBLEDevice::deleteAllBonds();
    prefs.remove("addr");
    prefs.remove("type");
    xSemaphoreTake(seenMtx, portMAX_DELAY);
    savedAddr.clear();
    xSemaphoreGive(seenMtx);
    Serial.println("I bonds_cleared");
    sayHello();
  } else {
    Serial.printf("W unknown_cmd %s\n", cmd.c_str());
  }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(BAUD);
  delay(200);

  reportQ = xQueueCreate(96, sizeof(ReportMsg));
  writeSem = xSemaphoreCreateBinary();
  seenMtx = xSemaphoreCreateMutex();

  prefs.begin("gsx", false);
  savedAddr = prefs.getString("addr", "").c_str();
  savedType = prefs.getUChar("type", 0);
  autoConnect = prefs.getBool("auto", true);

  NimBLEDevice::init("GSX");
  NimBLEDevice::setMTU(247);
  NimBLEDevice::setSecurityAuth(true, false, true);  // bond, no MITM, secure connections
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  NimBLEScan* s = NimBLEDevice::getScan();
  s->setScanCallbacks(&scanCB, false);
  s->setActiveScan(true);
  s->setInterval(100);
  s->setWindow(99);

  sayHello();
  if (autoConnect && !savedAddr.empty()) startScan(0);
}

void loop() {
  ReportMsg m;
  int n = 0;
  while (n < 48 && xQueueReceive(reportQ, &m, 0) == pdTRUE) {
    auto it = reportIdByHandle.find(m.handle);
    int id = it == reportIdByHandle.end() ? -1 : it->second;
    Serial.printf("R %lu %u %d ", (unsigned long)m.ms, m.handle, id);
    printHex(m.data, m.len);
    Serial.println();
    n++;
  }

  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (lineBuf.length()) handleCmd(lineBuf);
      lineBuf = "";
    } else if (lineBuf.length() < 600) {
      lineBuf += ch;
    }
  }

  if (flagStopScan) {
    flagStopScan = false;
    NimBLEDevice::getScan()->stop();
  }
  if (flagConnect) {
    NimBLEAddress a;
    xSemaphoreTake(seenMtx, portMAX_DELAY);
    a = pendingAddr;
    xSemaphoreGive(seenMtx);
    doConnect(a);
    flagConnect = false;
  }
  if (flagLinkDown) {
    flagLinkDown = false;
    reportIdByHandle.clear();
    chrByHandle.clear();
    if (autoConnect && !savedAddr.empty()) startScan(0);
  }

  static uint32_t lastDropped = 0;
  if (dropped != lastDropped) {
    lastDropped = dropped;
    Serial.printf("W dropped %lu\n", (unsigned long)lastDropped);
  }
  delay(1);
}
