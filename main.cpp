#include <Arduino.h>
// Meenavan boat device - Wokwi hardware simulation
// ESP32 + OLED + SOS button + TX/RX/Bluetooth LEDs + buzzer.
// Radio stand-in: the Serial monitor.
//   Lines the device prints as "TX <hex>" are frames it would send over VHF.
//   Type "RX <hex>" to feed it a frame as if heard over VHF.
//   Type "SOS" to toggle SOS, "STATUS" for counters.
// Demo mode: a built-in shore model sends a ship list every 5 s, so it runs hands-free.
// Positions, ships and the GPS track are simulated sample data.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define PIN_SOS     4
#define PIN_TX_LED  26
#define PIN_RX_LED  27
#define PIN_BLE_LED 14
#define PIN_BUZZER  25
#define PIN_BATT    34   // 12 V boat battery through a divider (potentiometer in Wokwi)

const float TIME_SCALE = 10.0;          // demo runs 10x faster than real time
const uint8_t T_SHIPS = 1, T_REPORT = 2, T_SOS = 3;
const uint8_t MAX_HOPS = 3;
const uint32_t MY_ID = 419900001UL;
const uint32_t SHORE_ID = 1;
const uint16_t MAX_FRAME = 8 + 12 * 16 + 2;   // 202 bytes

Adafruit_SSD1306 oled(128, 64, &Wire, -1);

struct Ship { bool used; uint32_t mmsi; double lat, lon; float sog, cog; uint8_t kind; uint32_t rxMs; };
Ship ships[16];

struct Seen { uint32_t src; uint8_t seq; };
Seen seen[32];
uint8_t seenIdx = 0;

struct Pending { bool used; uint32_t at; uint8_t buf[MAX_FRAME]; uint16_t len; };
Pending pend[4];

// Simulated GPS: a small boat heading east at 6 km/h
double myLat = 13.0500, myLon = 80.4000;
float mySog = 6.0, myCog = 90.0;
uint8_t mySeq = 0;
bool sosActive = false;

uint32_t lastGps = 0, lastReport = 0, lastSosTx = 0, lastDemo = 0, lastDraw = 0, lastBeep = 0;
uint32_t txOff = 0, rxOff = 0, bleOff = 0, btnLast = 0;
uint32_t framesHeard = 0, framesRelayed = 0, crcFails = 0, dupes = 0;
int alertLevel = 0;
float battV = 12.6;
String line;

// ---------- frame helpers ----------
uint16_t crc16(const uint8_t *d, uint16_t n) {
  uint16_t c = 0xFFFF;
  for (uint16_t i = 0; i < n; i++) {
    c ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : (c << 1);
  }
  return c;
}
void put32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
uint32_t get32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
void putCrc(uint8_t *buf, uint16_t len) { uint16_t c = crc16(buf, len - 2); buf[len - 2] = c >> 8; buf[len - 1] = c & 0xFF; }

void printHex(const char *prefix, const uint8_t *b, uint16_t n) {
  Serial.print(prefix);
  for (uint16_t i = 0; i < n; i++) { if (b[i] < 16) Serial.print('0'); Serial.print(b[i], HEX); }
  Serial.println();
}

uint16_t writeRecord(uint8_t *p, uint32_t id, double lat, double lon, float sog, float cog, uint8_t last) {
  put32(p, id);
  put32(p + 4, (uint32_t)(int32_t)lround(lat * 1e5));
  put32(p + 8, (uint32_t)(int32_t)lround(lon * 1e5));
  p[12] = (uint8_t)constrain((int)lround(sog), 0, 255);
  uint16_t c = (uint16_t)lround(fmod(cog + 360.0, 360.0) * 10);
  p[13] = c >> 8; p[14] = c & 0xFF;
  p[15] = last;
  return 16;
}

void transmit(const uint8_t *buf, uint16_t len) {
  printHex("TX ", buf, len);
  digitalWrite(PIN_TX_LED, HIGH);
  txOff = millis() + 200;
}

// ---------- geometry ----------
void toKm(double lat, double lon, float &x, float &y) {
  y = (lat - myLat) * 111.0;
  x = (lon - myLon) * 111.0 * cos(myLat * M_PI / 180.0);
}
// distance now (km), closest approach (km), minutes until it (sim time); returns level 0/1/2
int assess(const Ship &s, float &d, float &cpa, float &tmin) {
  float ageH = (millis() - s.rxMs) / 3600000.0 * TIME_SCALE;
  float svx = s.sog * sin(s.cog * M_PI / 180), svy = s.sog * cos(s.cog * M_PI / 180);
  float x, y; toKm(s.lat, s.lon, x, y);
  x += svx * ageH; y += svy * ageH;
  float mvx = mySog * sin(myCog * M_PI / 180), mvy = mySog * cos(myCog * M_PI / 180);
  float rvx = svx - mvx, rvy = svy - mvy, v2 = rvx * rvx + rvy * rvy;
  d = sqrt(x * x + y * y); cpa = d; tmin = 9999;
  if (v2 > 0.01) {
    float t = -(x * rvx + y * rvy) / v2;
    if (t > 0) { tmin = t * 60; cpa = sqrt(sq(x + rvx * t) + sq(y + rvy * t)); }
  }
  if (cpa < 1.5 && tmin < 15) return 2;
  if (cpa < 3.0 && tmin < 40) return 1;
  return 0;
}

// ---------- receive path ----------
bool alreadySeen(uint32_t src, uint8_t seq) {
  for (auto &s : seen) if (s.src == src && s.seq == seq && src != 0) return true;
  seen[seenIdx] = {src, seq}; seenIdx = (seenIdx + 1) % 32;
  return false;
}

void scheduleRelay(const uint8_t *buf, uint16_t len, bool urgent) {
  for (auto &p : pend) if (!p.used) {
    memcpy(p.buf, buf, len); p.len = len; p.buf[6]++;   // hops + 1
    putCrc(p.buf, len);
    p.at = millis() + (urgent ? random(20, 100) : random(100, 600));
    p.used = true; return;
  }
}

void upsertShip(const uint8_t *r) {
  uint32_t id = get32(r);
  Ship *slot = nullptr;
  for (auto &s : ships) if (s.used && s.mmsi == id) slot = &s;
  if (!slot) for (auto &s : ships) if (!s.used) { slot = &s; break; }
  if (!slot) return;
  slot->used = true; slot->mmsi = id;
  slot->lat = (int32_t)get32(r + 4) / 1e5; slot->lon = (int32_t)get32(r + 8) / 1e5;
  slot->sog = r[12]; slot->cog = ((r[13] << 8) | r[14]) / 10.0; slot->kind = r[15];
  slot->rxMs = millis();
}

void handleFrame(const uint8_t *buf, uint16_t len) {
  if (len < 10 || len > MAX_FRAME) return;
  uint16_t c = (buf[len - 2] << 8) | buf[len - 1];
  if (crc16(buf, len - 2) != c) { crcFails++; Serial.println("Dropped: CRC check failed"); return; }
  uint8_t type = buf[0], seq = buf[5], hops = buf[6], count = buf[7];
  uint32_t src = get32(buf + 1);
  if (src == MY_ID) return;
  if (alreadySeen(src, seq)) { dupes++; return; }
  framesHeard++;
  digitalWrite(PIN_RX_LED, HIGH); rxOff = millis() + 150;

  if (type == T_SHIPS) {
    if (len != 8 + 16 * count + 2) { Serial.println("Dropped: bad length"); return; }
    for (uint8_t i = 0; i < count; i++) upsertShip(buf + 8 + 16 * i);
    Serial.printf("RX ship list: %u ships, seq %u, hop %u -> sent to phone over Bluetooth\n", count, seq, hops);
    digitalWrite(PIN_BLE_LED, HIGH); bleOff = millis() + 250;
  } else if (type == T_REPORT || type == T_SOS) {
    const uint8_t *r = buf + 8;
    Serial.printf("RX %s from boat %lu at %.4f, %.4f (hop %u)\n", type == T_SOS ? "SOS" : "boat report",
                  (unsigned long)get32(r), (int32_t)get32(r + 4) / 1e5, (int32_t)get32(r + 8) / 1e5, hops);
    if (type == T_SOS) { tone(PIN_BUZZER, 2000, 400); }
  }
  if (hops < MAX_HOPS) scheduleRelay(buf, len, type == T_SOS);
}

// ---------- own transmissions ----------
void sendReport(bool sos) {
  uint8_t buf[8 + 16 + 2];
  buf[0] = sos ? T_SOS : T_REPORT; put32(buf + 1, MY_ID); buf[5] = ++mySeq; buf[6] = 0; buf[7] = 1;
  writeRecord(buf + 8, MY_ID, myLat, myLon, mySog, myCog, sos ? 1 : 0);
  putCrc(buf, sizeof(buf));
  Serial.println(sos ? "Sending SOS" : "Sending boat report");
  transmit(buf, sizeof(buf));
}

// ---------- demo shore (stands in for the shore station + live AIS feed) ----------
struct DemoShip { uint32_t mmsi; double lat, lon; float sog, cog; uint8_t kind; };
DemoShip demo[3];
uint8_t shoreSeq = 0;

void initDemo() {
  // A: crossing ahead, will come close.  B: passing well clear.  C: moving away.
  demo[0] = {419000101, myLat + 0.010, myLon + 0.110, 30, 268, 1};
  demo[1] = {419000102, myLat - 0.120, myLon + 0.060, 25, 300, 2};
  demo[2] = {419000103, myLat + 0.090, myLon - 0.050, 28, 340, 3};
}
void moveDemo(float hours) {
  for (auto &s : demo) {
    s.lat += s.sog * cos(s.cog * M_PI / 180) * hours / 111.0;
    s.lon += s.sog * sin(s.cog * M_PI / 180) * hours / (111.0 * cos(s.lat * M_PI / 180));
  }
}
void shoreFrame() {
  uint8_t buf[8 + 3 * 16 + 2];
  buf[0] = T_SHIPS; put32(buf + 1, SHORE_ID); buf[5] = ++shoreSeq; buf[6] = 1; buf[7] = 3;
  for (int i = 0; i < 3; i++) writeRecord(buf + 8 + 16 * i, demo[i].mmsi, demo[i].lat, demo[i].lon, demo[i].sog, demo[i].cog, demo[i].kind);
  putCrc(buf, sizeof(buf));
  printHex("RX(demo shore) ", buf, sizeof(buf));
  handleFrame(buf, sizeof(buf));
}

// ---------- serial "radio" input ----------
int hexVal(char ch) { if (ch >= '0' && ch <= '9') return ch - '0'; ch |= 0x20; if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10; return -1; }
void handleLine(String s) {
  s.trim();
  if (s.equalsIgnoreCase("SOS")) { sosActive = !sosActive; Serial.println(sosActive ? "SOS ON" : "SOS OFF"); if (sosActive) { sendReport(true); lastSosTx = millis(); } return; }
  if (s.equalsIgnoreCase("STATUS")) { Serial.printf("heard %lu, relayed %lu, duplicates %lu, CRC fails %lu, battery %.1f V\n", framesHeard, framesRelayed, dupes, crcFails, battV); return; }
  if (s.startsWith("RX ")) {
    uint8_t buf[MAX_FRAME]; uint16_t n = 0; String h = s.substring(3); h.replace(" ", "");
    for (uint16_t i = 0; i + 1 < h.length() && n < MAX_FRAME; i += 2) {
      int a = hexVal(h[i]), b = hexVal(h[i + 1]); if (a < 0 || b < 0) { Serial.println("Bad hex"); return; }
      buf[n++] = (a << 4) | b;
    }
    handleFrame(buf, n);
  }
}

// ---------- display and alerts ----------
const char *kindName(uint8_t k) { return k == 1 ? "cargo" : k == 2 ? "tanker" : k == 3 ? "contnr" : "ship"; }

void draw() {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0); oled.print("MEENAVAN"); oled.setCursor(56, 0); oled.printf("%.1fV", battV);
  if (battV < 11.5) { oled.setCursor(86, 0); oled.print("LOW"); }
  if (sosActive) { oled.setCursor(108, 0); oled.print("SOS"); }
  int idx[16], n = 0; float dist[16], cpa[16], tm[16]; int lv[16];
  for (int i = 0; i < 16; i++) if (ships[i].used) { lv[i] = assess(ships[i], dist[i], cpa[i], tm[i]); idx[n++] = i; }
  for (int a = 0; a < n; a++) for (int b = a + 1; b < n; b++)
    if (lv[idx[b]] > lv[idx[a]] || (lv[idx[b]] == lv[idx[a]] && dist[idx[b]] < dist[idx[a]])) { int t = idx[a]; idx[a] = idx[b]; idx[b] = t; }
  alertLevel = n ? lv[idx[0]] : 0;
  oled.setCursor(0, 12);
  if (!n) oled.print("Waiting for ships");
  else if (alertLevel == 2) oled.printf("CLOSE %lu %.1fkm %dm", (unsigned long)(ships[idx[0]].mmsi % 1000), cpa[idx[0]], (int)tm[idx[0]]);
  else if (alertLevel == 1) oled.printf("WATCH %lu %.1fkm %dm", (unsigned long)(ships[idx[0]].mmsi % 1000), cpa[idx[0]], (int)tm[idx[0]]);
  else oled.print("No close approach");
  oled.drawLine(0, 22, 127, 22, SSD1306_WHITE);
  for (int k = 0; k < n && k < 3; k++) {
    int i = idx[k]; oled.setCursor(0, 26 + 10 * k);
    oled.printf("%03lu %-6s %4.1fkm %s", (unsigned long)(ships[i].mmsi % 1000), kindName(ships[i].kind), dist[i], lv[i] == 2 ? "!!" : lv[i] == 1 ? "!" : "");
  }
  oled.setCursor(0, 56); oled.printf("RX%lu RLY%lu", framesHeard, framesRelayed);
  oled.display();
}

void beep(uint32_t now) {
  if (alertLevel == 2 && now - lastBeep > 600) { tone(PIN_BUZZER, 1200, 300); lastBeep = now; }
  else if (alertLevel == 1 && now - lastBeep > 2500) { tone(PIN_BUZZER, 800, 120); lastBeep = now; }
}

// ---------- main ----------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_SOS, INPUT_PULLUP);
  pinMode(PIN_TX_LED, OUTPUT); pinMode(PIN_RX_LED, OUTPUT); pinMode(PIN_BLE_LED, OUTPUT); pinMode(PIN_BUZZER, OUTPUT);
  randomSeed(esp_random());
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("OLED not found");
  initDemo();
  Serial.println("Meenavan boat device (simulation). Type SOS, STATUS, or RX <hex>.");
  draw();
}

void loop() {
  uint32_t now = millis();
  while (Serial.available()) { char ch = Serial.read(); if (ch == '\n' || ch == '\r') { if (line.length()) handleLine(line); line = ""; } else line += ch; }

  if (digitalRead(PIN_SOS) == LOW && now - btnLast > 400) {
    btnLast = now; sosActive = !sosActive; Serial.println(sosActive ? "SOS button: ON" : "SOS button: OFF");
    if (sosActive) { sendReport(true); lastSosTx = now; }
  }
  if (now - lastGps >= 1000) {                       // simulated GPS fix once a second
    float h = (now - lastGps) / 3600000.0 * TIME_SCALE; lastGps = now;
    myLat += mySog * cos(myCog * M_PI / 180) * h / 111.0;
    myLon += mySog * sin(myCog * M_PI / 180) * h / (111.0 * cos(myLat * M_PI / 180));
    moveDemo(h);
    battV = 10.5 + analogRead(PIN_BATT) / 4095.0 * 3.0;
  }
  if (now - lastDemo >= 5000) { lastDemo = now; shoreFrame(); }
  if (sosActive && now - lastSosTx >= 5000) { lastSosTx = now; sendReport(true); }
  if (!sosActive && now - lastReport >= 12000) { lastReport = now; sendReport(false); }

  for (auto &p : pend) if (p.used && now >= p.at) { p.used = false; framesRelayed++; Serial.printf("Relaying frame, hop %u\n", p.buf[6]); transmit(p.buf, p.len); }

  if (txOff && now > txOff) { digitalWrite(PIN_TX_LED, LOW); txOff = 0; }
  if (rxOff && now > rxOff) { digitalWrite(PIN_RX_LED, LOW); rxOff = 0; }
  if (bleOff && now > bleOff) { digitalWrite(PIN_BLE_LED, LOW); bleOff = 0; }
  if (now - lastDraw >= 500) { lastDraw = now; draw(); }
  beep(now);
  delay(5);
}
