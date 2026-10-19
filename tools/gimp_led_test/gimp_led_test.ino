/*
  Gimp head LED test + behaviour demos
  ------------------------------------
  Board : any ESP32-S3 dev board (Arduino IDE, "ESP32S3 Dev Module")
  Lib   : Adafruit NeoPixel (Library Manager)
  Wiring: DATA_PIN -> 330-470 ohm resistor -> DIN of the 7-LED jewel
          5 V and GND to every ring (inject 5 V at both ends of the chain if you can),
          ESP32 GND to the LED GND.
  Chain : jewel 7 (DIN first) -> ring 12 -> ring 16 -> ring 24 (last)

  Open Serial Monitor at 115200, line ending "Newline", and type commands (see help()).

  Directions used everywhere: angle 0 = FRONT of the robot, angles grow CLOCKWISE seen
  from above (90 = robot's right, 180 = back, 270 = left).  Calibrate once with
  'r', 'c', 'f' and 'd', then 'p' prints the numbers to paste into RINGS[] below.
*/
#include <Adafruit_NeoPixel.h>

#define DATA_PIN   4
#define NUM_LEDS   59          // 7 + 12 + 16 + 24
#define FRAME_MS   20

Adafruit_NeoPixel px(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

// ---- ring layout ------------------------------------------------------------
// start : first chain index of the ring
// count : LEDs on the circle (the jewel's centre LED is extra, at 'start')
// front : ring-local index of the LED that faces the robot's front   <- calibrate
// dir   : +1 if indexes increase clockwise seen from above, -1 if not <- calibrate
// tier  : 0 = lowest (24 ring) ... 3 = jewel circle, 4 = jewel centre
struct Ring { const char *name; int start; int count; int front; int dir; int tier; bool hasCentre; };
Ring RINGS[] = {
  {"jewel7", 0,  6,  1, 1, 3, true },   // chain 0 = centre, 1..6 = circle; front = 2nd circle LED (green marker)
  {"ring12", 7,  12, 0, 1, 2, false},   // LED 0 (red marker) glued facing front
  {"ring16", 19, 16, 0, -1, 1, false},  // LED 0 front; this ring counts counter-clockwise
  {"ring24", 35, 24, 0, 1, 0, false},   // LED 0 front
};
const int NRINGS = 4;
const int R_JEWEL = 0, R12 = 1, R16 = 2, R24 = 3;

uint8_t brightness = 60;       // global cap (0-255). 59 LEDs at full white is ~3.5 A!
float   doaDeg = 0;            // simulated "voice came from" angle for the listen demo
bool    doaSweep = true;       // listen demo sweeps the angle unless you set one with 'a'
float   buf[NUM_LEDS][3];      // float frame buffer for fading effects

// Declared before any function: the Arduino IDE inserts its own function
// prototypes above the first function, and they need this type.
enum Mode { OFF, WALK, MARKERS, COMPASS, SINGLE,
            ALIVE, WAKE, LISTEN, THINK, SPEAK, DRIVE, REVERSE, TURN_L, TURN_R,
            ERROR_, MUTED, HAPPY, SLEEP };

// ---- helpers ----------------------------------------------------------------
int chainIndex(int r, int k) {             // ring-local circle index -> chain index
  const Ring &R = RINGS[r];
  k = ((k % R.count) + R.count) % R.count;
  return R.start + (R.hasCentre ? 1 : 0) + k;
}
float ledAngle(int r, int k) {             // physical angle of ring-local LED k
  const Ring &R = RINGS[r];
  float a = R.dir * (k - R.front) * 360.0f / R.count;
  a = fmodf(a, 360.0f); if (a < 0) a += 360.0f;
  return a;
}
int ledAt(int r, float deg) {              // ring-local index of the LED nearest an angle
  const Ring &R = RINGS[r];
  int steps = lroundf(deg / (360.0f / R.count));
  return ((R.front + R.dir * steps) % R.count + R.count) % R.count;
}
float angDiff(float a, float b) { float d = fmodf(fabsf(a - b), 360.0f); return d > 180 ? 360 - d : d; }

void clearBuf() { memset(buf, 0, sizeof(buf)); }
void addPix(int i, float r, float g, float b) {
  if (i < 0 || i >= NUM_LEDS) return;
  buf[i][0] = min(255.0f, buf[i][0] + r); buf[i][1] = min(255.0f, buf[i][1] + g); buf[i][2] = min(255.0f, buf[i][2] + b);
}
void setPix(int i, float r, float g, float b) { if (i >= 0 && i < NUM_LEDS) { buf[i][0] = r; buf[i][1] = g; buf[i][2] = b; } }
void hsv(float h, float s, float v, float &r, float &g, float &b) {   // h 0-360, s/v 0-1 -> 0-255
  uint32_t c = Adafruit_NeoPixel::ColorHSV((uint16_t)(fmodf(h + 360.0f, 360.0f) * 182.04f), s * 255, v * 255);
  r = (c >> 16) & 255; g = (c >> 8) & 255; b = c & 255;
}
void fadeAll(float f) { for (int i = 0; i < NUM_LEDS; i++) for (int c = 0; c < 3; c++) buf[i][c] *= f; }
void flush() {
  for (int i = 0; i < NUM_LEDS; i++)
    px.setPixelColor(i, px.gamma32(px.Color((uint8_t)buf[i][0], (uint8_t)buf[i][1], (uint8_t)buf[i][2])));
  px.show();
}
// call fn(chainIndex, ring, angleDeg, tier) for every LED on the head
template <typename F> void forEachLed(F fn) {
  for (int r = 0; r < NRINGS; r++) {
    for (int k = 0; k < RINGS[r].count; k++) fn(chainIndex(r, k), r, ledAngle(r, k), RINGS[r].tier);
    if (RINGS[r].hasCentre) fn(RINGS[r].start, r, -1.0f, 4);
  }
}

// ---- modes ------------------------------------------------------------------
Mode mode = ALIVE;
uint32_t modeStart = 0;
int singleIdx = 0;
const char *modeName(Mode m) {
  static const char *n[] = {"off","walk","markers","compass","single","alive","wake","listen","think",
                            "speak","drive","reverse","turn-left","turn-right","error","muted","happy","sleep"};
  return n[m];
}
void setMode(Mode m) { mode = m; modeStart = millis(); clearBuf(); Serial.printf("> mode: %s\n", modeName(m)); }

// -- calibration views --
void fxWalk(uint32_t t) {                  // one LED at a time along the chain
  static int last = -1;
  int i = (t / 500) % NUM_LEDS;
  clearBuf(); setPix(i, 255, 255, 255);
  if (i != last) {
    last = i;
    for (int r = 0; r < NRINGS; r++) {
      const Ring &R = RINGS[r];
      int end = R.start + R.count + (R.hasCentre ? 1 : 0);
      if (i >= R.start && i < end) {
        if (R.hasCentre && i == R.start) Serial.printf("chain %2d = %s centre\n", i, R.name);
        else Serial.printf("chain %2d = %s LED %d\n", i, R.name, i - R.start - (R.hasCentre ? 1 : 0));
      }
    }
  }
}
void fxMarkers() {                         // per ring: LED0 red, LED1 green, LED2 blue
  clearBuf();
  for (int r = 0; r < NRINGS; r++) {
    for (int k = 0; k < RINGS[r].count; k++) setPix(chainIndex(r, k), 8, 8, 8);
    setPix(chainIndex(r, 0), 255, 0, 0); setPix(chainIndex(r, 1), 0, 255, 0); setPix(chainIndex(r, 2), 0, 0, 255);
    if (RINGS[r].hasCentre) setPix(RINGS[r].start, 40, 40, 40);
  }
}
void fxCompass() {                         // after calibration: front white, right green, back red, left blue
  clearBuf();
  for (int r = 0; r < NRINGS; r++) {
    setPix(chainIndex(r, ledAt(r, 0)),   255, 255, 255);
    setPix(chainIndex(r, ledAt(r, 90)),  0, 255, 0);
    setPix(chainIndex(r, ledAt(r, 180)), 255, 0, 0);
    setPix(chainIndex(r, ledAt(r, 270)), 0, 0, 255);
  }
}

// -- behaviours --
void fxAlive(uint32_t t) {                 // idle: slow breathing that rolls up the head + rare twinkles
  float s = t / 1000.0f;
  forEachLed([&](int i, int r, float a, int tier) {
    float wave = 0.5f + 0.5f * sinf(s * 0.9f - tier * 0.7f);            // ripple rises through the tiers
    float drift = 0.5f + 0.5f * sinf(s * 0.25f + (a < 0 ? 0 : a) * 0.0175f); // slow colour drift round the head
    float R, G, B; hsv(175 + 25 * drift, 0.9f, 0.05f + 0.20f * wave, R, G, B);
    setPix(i, R, G, B);
  });
  static float tw[NUM_LEDS];
  if (random(1000) < 15) tw[random(NUM_LEDS)] = 1.0f;                  // occasional "thought" twinkle
  for (int i = 0; i < NUM_LEDS; i++) { if (tw[i] > 0.01f) { addPix(i, 120 * tw[i], 120 * tw[i], 160 * tw[i]); tw[i] *= 0.93f; } }
}
void fxListen(uint32_t t) {                // 24 ring points at the speaker, upper rings attentive blue
  if (doaSweep) doaDeg = fmodf(t / 20.0f, 360.0f);
  float s = t / 1000.0f;
  forEachLed([&](int i, int r, float a, int tier) {
    if (r == R24) {
      float d = angDiff(a, doaDeg);
      float w = d < 70 ? powf(cosf(d * 0.0175f * 1.28f), 3) : 0;      // ~140 degree arc
      setPix(i, 40 + 215 * w, 120 + 135 * w, 255 * (0.15f + 0.85f * w));
      if (w < 0.05f) setPix(i, 0, 10, 30);
    } else {
      float v = 0.25f + 0.15f * sinf(s * 6 - tier);
      setPix(i, 0, 60 * v, 255 * v);
    }
  });
}
void fxWake(uint32_t t) {                  // bright blue flash, then hand over to listen
  if (t < 120 || (t > 220 && t < 340)) { forEachLed([&](int i, int, float, int) { setPix(i, 40, 120, 255); }); }
  else if (t < 600) { fadeAll(0.85f); }
  else { mode = LISTEN; }
}
void fxThink(uint32_t t) {                 // busy brain: firing neurons + counter-rotating comets
  fadeAll(0.86f);
  for (int n = 0; n < 3; n++) { float R, G, B; hsv(random(360), 1, 1, R, G, B); addPix(random(NUM_LEDS), R, G, B); }
  float s = t / 1000.0f;
  for (int r = 0; r < NRINGS; r++) {
    float speed = (r % 2 ? -1 : 1) * (90 + 60 * r);                   // deg/s, alternate directions
    float a = fmodf(s * speed + 3600, 360.0f);
    float R, G, B; hsv(fmodf(s * 60 + r * 80, 360), 1, 1, R, G, B);
    addPix(chainIndex(r, ledAt(r, a)), R, G, B);
  }
  if (RINGS[R_JEWEL].hasCentre) { float v = 0.5f + 0.5f * sinf(s * 10); setPix(RINGS[R_JEWEL].start, 255 * v, 255 * v, 255 * v); }
}
void fxSpeak(uint32_t t) {                 // talking: level rises from the bottom ring up with the "voice"
  static float env = 0, target = 0; static uint32_t next = 0;
  if (t >= next) { target = random(100) < 20 ? 0.1f : random(30, 100) / 100.0f; next = t + random(60, 160); }
  env += (target - env) * 0.35f;
  forEachLed([&](int i, int r, float a, int tier) {
    float lit = env * 5.0f - tier;                                     // 0..5 levels up the head
    float v = lit >= 1 ? 1 : (lit > 0 ? lit : 0.05f);
    setPix(i, 30 * v, 255 * v, 90 * v);
  });
}
void fxDrive(uint32_t t, int dirn) {       // bands flow front-to-back (or back-to-front when reversing)
  float s = t / 1000.0f;
  forEachLed([&](int i, int r, float a, int tier) {
    float x = a < 0 ? 0 : cosf(a * 0.0175f);                           // +1 front ... -1 back
    float v = 0.5f + 0.5f * sinf(x * 4.0f + dirn * s * 8.0f);
    v = v * v * v;
    setPix(i, 255 * (0.05f + 0.95f * v), 110 * (0.05f + 0.95f * v), 0);
  });
}
void fxTurn(uint32_t t, int dirn) {        // spiral sweeping the way it is turning
  fadeAll(0.80f);
  float a = fmodf(dirn * t * 0.36f + 3600000.0f, 360.0f);
  for (int r = 0; r < NRINGS; r++) {
    float lag = r * 25.0f * dirn;                                      // upper rings trail -> spiral
    addPix(chainIndex(r, ledAt(r, fmodf(a - lag + 720, 360))), 255, 140, 0);
  }
}
void fxError(uint32_t t) {
  uint32_t p = t % 1500; bool on = p < 120 || (p > 250 && p < 370);
  forEachLed([&](int i, int, float, int) { setPix(i, on ? 255 : 10, 0, 0); });
}
void fxMuted(uint32_t t) {
  clearBuf();
  for (int k = 0; k < RINGS[R24].count; k++) setPix(chainIndex(R24, k), 25, 0, 0);
  setPix(chainIndex(R24, ledAt(R24, 0)), 120, 0, 0);
}
void fxHappy(uint32_t t) {
  float s = t / 1000.0f;
  forEachLed([&](int i, int r, float a, int tier) {
    float R, G, B; hsv((a < 0 ? 0 : a) + tier * 40 + s * 120, 1, 0.8f, R, G, B); setPix(i, R, G, B);
  });
}
void fxSleep(uint32_t t) {
  clearBuf();
  float v = 0.02f + 0.06f * (0.5f + 0.5f * sinf(t / 1000.0f * 0.8f));
  setPix(RINGS[R_JEWEL].start, 120 * v * 4, 0, 255 * v * 4);
}

// ---- serial commands --------------------------------------------------------
void help() {
  Serial.println(F(
    "\n=== Gimp head LED test ===\n"
    "Calibration:\n"
    "  w            walk: one LED at a time along the chain (prints which ring/LED)\n"
    "  r            markers: on every ring LED0 red, LED1 green, LED2 blue\n"
    "  c            compass: front white, right green, back red, left blue (check after calibrating)\n"
    "  n <i>        light chain LED i only\n"
    "  f <ring> <k> set ring's FRONT LED to ring-local index k (ring = 7, 12, 16 or 24)\n"
    "  d <ring>     flip a ring's direction (if green is not clockwise of red seen from above)\n"
    "  p            print calibration to paste into RINGS[]\n"
    "Behaviours:\n"
    "  1 alive   2 wake(->listen)   3 listen   4 think   5 speak\n"
    "  6 drive   7 reverse   8 turn-left   9 turn-right\n"
    "  e error   m muted   h happy   s sleep   0 off\n"
    "  a <deg>    point listen at an angle (0 front, 90 right); 'a' alone = sweep\n"
    "  b <0-255>  brightness cap (now set low on purpose)\n"));
}
int ringFromSize(int sz) { return sz == 7 ? R_JEWEL : sz == 12 ? R12 : sz == 16 ? R16 : sz == 24 ? R24 : -1; }
void printCal() {
  Serial.println("Ring RINGS[] = {");
  for (int r = 0; r < NRINGS; r++) {
    const Ring &R = RINGS[r];
    Serial.printf("  {\"%s\", %d, %d, %d, %d, %d, %s},\n", R.name, R.start, R.count, R.front, R.dir, R.tier, R.hasCentre ? "true " : "false");
  }
  Serial.println("};");
}
void handleLine(String s) {
  s.trim(); if (!s.length()) return;
  char c = s[0]; String arg = s.substring(1); arg.trim();
  int a1 = arg.toInt(); int sp = arg.indexOf(' '); int a2 = sp > 0 ? arg.substring(sp + 1).toInt() : 0;
  switch (c) {
    case 'w': setMode(WALK); break;
    case 'r': setMode(MARKERS); break;
    case 'c': setMode(COMPASS); break;
    case 'n': singleIdx = constrain(a1, 0, NUM_LEDS - 1); setMode(SINGLE); Serial.printf("chain LED %d\n", singleIdx); break;
    case 'f': { int r = ringFromSize(a1); if (r < 0) { Serial.println("ring must be 7/12/16/24"); break; }
                RINGS[r].front = ((a2 % RINGS[r].count) + RINGS[r].count) % RINGS[r].count;
                Serial.printf("%s front = LED %d\n", RINGS[r].name, RINGS[r].front); setMode(COMPASS); break; }
    case 'd': { int r = ringFromSize(a1); if (r < 0) { Serial.println("ring must be 7/12/16/24"); break; }
                RINGS[r].dir = -RINGS[r].dir; Serial.printf("%s dir = %d\n", RINGS[r].name, RINGS[r].dir); setMode(COMPASS); break; }
    case 'p': printCal(); break;
    case '1': setMode(ALIVE); break;   case '2': setMode(WAKE); break;   case '3': setMode(LISTEN); break;
    case '4': setMode(THINK); break;   case '5': setMode(SPEAK); break;  case '6': setMode(DRIVE); break;
    case '7': setMode(REVERSE); break; case '8': setMode(TURN_L); break; case '9': setMode(TURN_R); break;
    case 'e': setMode(ERROR_); break;  case 'm': setMode(MUTED); break;  case 'h': setMode(HAPPY); break;
    case 's': setMode(SLEEP); break;   case '0': setMode(OFF); break;
    case 'a': if (arg.length()) { doaSweep = false; doaDeg = fmodf(a1 + 360, 360); Serial.printf("listen angle %d\n", (int)doaDeg); }
              else { doaSweep = true; Serial.println("listen angle sweeping"); }
              setMode(LISTEN); break;
    case 'b': brightness = constrain(a1, 0, 255); px.setBrightness(brightness); Serial.printf("brightness %d\n", brightness); break;
    default: help();
  }
}

void setup() {
  Serial.begin(115200);
  px.begin(); px.setBrightness(brightness); px.clear(); px.show();
  delay(500); help(); setMode(ALIVE);
}

void loop() {
  // Commands end at Enter, or after a short pause - so any "line ending" setting works
  static String line; static uint32_t lastChar = 0;
  while (Serial.available()) {
    char ch = Serial.read(); lastChar = millis();
    if (ch == '\n' || ch == '\r') { handleLine(line); line = ""; } else line += ch;
  }
  if (line.length() && millis() - lastChar > 300) { handleLine(line); line = ""; }
  static uint32_t lastFrame = 0;
  uint32_t now = millis(); if (now - lastFrame < FRAME_MS) return; lastFrame = now;
  uint32_t t = now - modeStart;
  switch (mode) {
    case OFF: clearBuf(); break;
    case WALK: fxWalk(t); break;          case MARKERS: fxMarkers(); break;   case COMPASS: fxCompass(); break;
    case SINGLE: clearBuf(); setPix(singleIdx, 255, 255, 255); break;
    case ALIVE: fxAlive(t); break;        case WAKE: fxWake(t); break;        case LISTEN: fxListen(t); break;
    case THINK: fxThink(t); break;        case SPEAK: fxSpeak(t); break;
    case DRIVE: fxDrive(t, 1); break;     case REVERSE: fxDrive(t, -1); break;
    case TURN_L: fxTurn(t, -1); break;    case TURN_R: fxTurn(t, 1); break;
    case ERROR_: fxError(t); break;       case MUTED: fxMuted(t); break;
    case HAPPY: fxHappy(t); break;        case SLEEP: fxSleep(t); break;
  }
  flush();
}
