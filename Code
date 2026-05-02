/*
  Kamera-Slider Komplett-Sketch (Arduino Mega + RAMPS 1.6)

  Hardware:
  - Slider Stepper: TMC2208 (Standalone) auf X-Achse (X_STEP/DIR/EN)
  - Kamera Stepper: TMC2208 (Standalone) auf Y-Achse (Y_STEP/DIR/EN)
  - OLED GM009605v4 (SSD1306) I2C 0x3C
 
  Mechanik:
  - Slider: 86cm = 4300 steps (gegeben)
  - Kamera: 180° = 1600 steps (gegeben)
    Mapping: 0° = schaut rechts entlang Slider, 90° = rechtwinklig weg, 180° = links entlang Slider

  Ablauf:
  - Boot: "Press Start" (OK)
  - Kalibrierung:
      Slider: 200 steps away, dann mit 400 steps/s home bis X- auslöst (ohne accel)
      Kamera: 200 steps away, dann mit 200 steps/s home bis X+ auslöst (ohne accel)
  - Menü: Kalibrieren / Parameter / Orbit Mode / Start
  - Parameter-Menü: Delay, Accel, Speed, StartAngle, EndAngle, Orbit, Distance
    +/- ändern, OK next, Back zurück
  - Start:
      Orbit ON: Orbit-Winkel precompute -> "Ready" -> OK -> Delay -> Run
      Orbit OFF: "Ready" -> OK -> Delay -> Run
  - Run:
      Slider fährt mit AccelStepper (Accel, Speed) bis Ende
      Kamera:
        Normal: linear Start->End (in Grad) synchron zum Slider
        Orbit: Keyframe-Lookup (keine trig während Run)
      Display wird während Run/GoingHome NICHT aktualisiert (nur einmal zu Beginn)
  - Going Home:
      Slider fährt zurück bis X-; wenn nicht in 10s -> ERROR, Treiber idle, zurück ins Menü nach OK
  - Parameter werden in EEPROM gespeichert via EEPROM.update()

  Hinweis / Annahmen:
  - Endstops sind INPUT_PULLUP, ausgelöst = LOW.
  - Homing Richtungen (DIR) sind angenommen. Wenn er in die falsche Richtung fährt:
    -> In calibrateAll() bei den DIR Levels tauschen (siehe Kommentare).
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <AccelStepper.h>
#include <math.h>
#include <limits.h>


// ======================= RAMPS 1.6 Pins (Mega) =======================
// X Axis (Slider)
#define X_STEP_PIN    54
#define X_DIR_PIN     55
#define X_ENABLE_PIN  38

// Y Axis (Camera rotation stepper)
#define Y_STEP_PIN    60
#define Y_DIR_PIN     61
#define Y_ENABLE_PIN  56

// Endstops
#define SLIDER_HOME_PIN   3   // X-
#define CAM_HOME_PIN      14   // Y-

// Buttons (INPUT_PULLUP: pressed = LOW)
#define BTN_PLUS_PIN   17
#define BTN_MINUS_PIN  16
#define BTN_OK_PIN     25
#define BTN_BACK_PIN   23

// ======================= OLED =======================
#define OLED_ADDR 0x3C
#define SCREEN_W 128
#define SCREEN_H 64
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// ======================= Buttons (Debounce ohne Struct) =======================
// INPUT_PULLUP: released = HIGH, pressed = LOW
static const uint8_t BTN_PINS[4] = { BTN_PLUS_PIN, BTN_MINUS_PIN, BTN_OK_PIN, BTN_BACK_PIN };
enum { BI_PLUS=0, BI_MINUS=1, BI_OK=2, BI_BACK=3 };

bool readButtonEdge(uint8_t idx) {
  static bool lastRead[4]   = {true, true, true, true};
  static bool lastStable[4] = {true, true, true, true};
  static uint32_t lastChMs[4] = {0,0,0,0};

  uint8_t pin = BTN_PINS[idx];
  bool r = digitalRead(pin);

  if (r != lastRead[idx]) {
    lastRead[idx] = r;
    lastChMs[idx] = millis();
  }

  if (millis() - lastChMs[idx] > 25) {
    if (r != lastStable[idx]) {
      lastStable[idx] = r;
      if (lastStable[idx] == LOW) {
        return true; // pressed edge
      }
    }
  }
  return false;
}


// ======================= Mechanics / Constants =======================
static const long SLIDER_TOTAL_STEPS = 21250;    // 85 cm 42500 ohne Jumper
static const long CAM_TOTAL_STEPS_180 = 4140;   // 180° = 2070 steps

// Homing behavior
static const int SLIDER_UNBLOCK_STEPS = 200;
static const int SLIDER_HOME_SPEED_SPS = 1000;
static const uint16_t SLIDER_HOME_RAPID_SPS = 3000;   // schnell
static const uint16_t SLIDER_HOME_SLOW_SPS  = 1000;    // langsam (dein SLIDER_HOME_SPEED_SPS geht auch)
static const uint16_t SLIDER_HOME_SLOW_ZONE_STEPS = 200; // ab hier langsam
static const uint16_t SLIDER_HOME_BACKOFF_STEPS = 20;    // optional
static const int CAM_UNBLOCK_STEPS = 200;
static const int CAM_HOME_SPEED_SPS = 400;

// Pause
static long runEndTargetSteps = 0;
static bool runWasAborted = false;

static uint8_t loopCur = 1;
static bool autoStartAfterPrep = false;

// Loop
static long loopStartPos = 0;
static long loopEndPos   = 0;
static bool passToEnd    = true;   // true: zum loopEndPos, false: zum loopStartPos


// cam target reset (statt static inside function)
static long camLastTarget = LONG_MIN;


// Going home timeout
static const uint32_t HOME_TIMEOUT_MS = 15000UL;

// Orbit geometry (Object centered, perpendicular to slider)
static const float SLIDER_LENGTH_M = 0.85f;   // 85 cm

// --- Slider speed conversion (steps/s <-> mm/s) ---
static const float SLIDER_TRAVEL_MM = SLIDER_LENGTH_M * 1000.0f; // 0.85m -> 850mm
static const float SLIDER_STEPS_PER_MM = (float)SLIDER_TOTAL_STEPS / SLIDER_TRAVEL_MM;

static inline float spsToMmps(uint16_t sps) {
  return (float)sps / SLIDER_STEPS_PER_MM;   // steps/s -> mm/s
}
static inline uint16_t mmpsToSps(float mmps) {
  if (mmps < 0) mmps = 0;
  return (uint16_t)lroundf(mmps * SLIDER_STEPS_PER_MM); // mm/s -> steps/s
}

//EPROM Speichern
static bool paramsDirty = false;

// Orbit precompute sampling
static const uint8_t ORBIT_SAMPLE_STEP = 2;   // every 2 slider steps

// Orbit Keyframes: store only changes
static const uint16_t ORBIT_MAX_KEYS = 900;

static long camReturnPosSteps = 0;

static const float CAM_POS_MAXSPEED = 1200.0f;  // langsamer fürs Positionieren
static const float CAM_POS_ACCEL    = 1000.0f;

static const float CAM_RUN_MAXSPEED = 1800.0f;
static const float CAM_RUN_ACCEL    = 2500.0f;

// ===== Eilgang / Positionier-Geschwindigkeiten =====
static const float SLIDER_POS_MAXSPEED = 3000.0f;   // steps/s fürs Positionieren (Eilgang)
static const float SLIDER_POS_ACCEL    = 1000.0f;  // steps/s^2 fürs Positionieren


// --- Speed in 0.1 mm/s ---
static const uint16_t SPEED10_MIN = 1;     // 0.1 mm/s (0 würde "steht" bedeuten)
static const uint16_t SPEED10_MAX = 1500;  // 150.0 mm/s (passend zu deinem bisherigen Limit)

static inline float speed10ToMmps(uint16_t s10) {
  return (float)s10 * 0.1f;
}

static inline uint16_t clampSpeed10(int v) {
  if (v < SPEED10_MIN) v = SPEED10_MIN;
  if (v > SPEED10_MAX) v = SPEED10_MAX;
  return (uint16_t)v;
}

// Schrittweite abhängig vom aktuellen Bereich (Grenzen "sauber"):
// - bei 5.0 aufwärts: 0.5
// - bei 10.0 aufwärts: 1.0
// - bei 25.0 aufwärts: 5.0
static inline uint16_t speedStep10(uint16_t cur10, int dir) {
  int probe = (dir > 0) ? (int)cur10 : (int)cur10 - 1; // bei DOWN am Rand in den kleineren Bereich fallen
  if (probe < 0) probe = 0;

  if (probe < 50)  return 1;   // <5.0  => 0.1
  if (probe < 100) return 5;   // <10.0 => 0.5
  if (probe < 250) return 10;  // <25.0 => 1.0
  return 50;                   // >=25.0 => 5.0
}

static inline void changeSpeed10(uint16_t &speed10, int dir) {
  uint16_t step = speedStep10(speed10, dir);
  int v = (int)speed10 + dir * (int)step;
  speed10 = clampSpeed10(v);
}




// ======================= Stepper objects =======================
AccelStepper slider(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper cam(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

// ======================= Parameters + EEPROM =======================
struct Params {
  uint16_t delaySec;     // default 1
  uint16_t accel;        // default 2000
  uint16_t speed10;        // default 2000 steps/s (slider)
  uint8_t startAngle;    // default 30°
  uint8_t endAngle;      // default 150°
  bool reverseDir;      // false = L->R, true = R->L
  bool orbitOn;          // default ON
  uint8_t loopCount;    //  (1 = kein Loop, >1 = Wiederholungen)
  float distanceM;       // default 0.2m
};

static const uint32_t EEPROM_MAGIC = 0x4A4F4E55; // "JONU"
static const uint16_t EEPROM_VER   = 5;

struct EBlob {
  uint32_t magic;
  uint16_t ver;
  Params p;
};

Params P;

template<typename T>
void eepromReadStruct(int addr, T &out) {
  uint8_t *ptr = (uint8_t*)&out;
  for (size_t i = 0; i < sizeof(T); i++) ptr[i] = EEPROM.read(addr + i);
}
template<typename T>
void eepromUpdateStruct(int addr, const T &in) {
  const uint8_t *ptr = (const uint8_t*)&in;
  for (size_t i = 0; i < sizeof(T); i++) EEPROM.update(addr + i, ptr[i]);
}

void loadParams() {
  EBlob b;
  eepromReadStruct(0, b);

  if (b.magic != EEPROM_MAGIC) {
    P.delaySec = 1;
    P.accel = 2000;
    P.speed10 = 500;      // 50.0 mm/s
    P.startAngle = 30;
    P.endAngle = 150;
    P.orbitOn = true;
    P.distanceM = 0.2f;
    P.reverseDir = false;
    P.loopCount = 1;

    return;
  }

  // erst mal direkt übernehmen (Layout bleibt gleich groß, nur Bedeutung von speed-Feld ändert sich)
  P = b.p;

  if (b.ver == 2) {
    // ALT: speed-Feld enthielt steps/s
    float mmps = spsToMmps((uint16_t)P.speed10);
    P.speed10 = clampSpeed10((int)lroundf(mmps * 10.0f));
    P.loopCount = 1; 
    saveParams();
    return;
  }

  if (b.ver == 3) {
    // ALT: speed-Feld enthielt mm/s (integer)
    P.speed10 = clampSpeed10((int)P.speed10 * 10);
    P.loopCount = 1; 
    saveParams();
    return;
  }

  if (b.ver == 4) {
  // ver4 war schon speed10-Format, aber ohne loopCount
  P.loopCount = 1;          // <-- NEU
  saveParams();
  return;
  }

  if (b.ver != EEPROM_VER) {
    // Defaults
    P.delaySec = 1;
    P.accel = 2000;
    P.speed10 = 500;
    P.startAngle = 30;
    P.endAngle = 150;
    P.orbitOn = true;
    P.distanceM = 0.2f;
    P.reverseDir = false;
    P.loopCount = 1;
    return;
  }

  
}

static void fmtHMS(uint32_t sec, char *out, size_t n) {
  uint32_t h = sec / 3600UL;
  uint32_t m = (sec % 3600UL) / 60UL;
  uint32_t s = sec % 60UL;
  if (h > 99) h = 99; // OLED-Layout-Schutz
  snprintf(out, n, "%02lu:%02lu:%02lu",
           (unsigned long)h, (unsigned long)m, (unsigned long)s);
}

// Zeit für einen kompletten Pass (0..SLIDER_TOTAL_STEPS) inkl. Accel/Decel
static uint32_t estimateRunSecOnePass(uint16_t speed10, uint16_t accelSteps) {
  const float d = (float)SLIDER_TOTAL_STEPS; // Weg in steps

  // vmax in steps/s aus Speed (mm/s) * steps/mm
  float vmax = speed10ToMmps(speed10) * SLIDER_STEPS_PER_MM; // steps/s
  float a    = (float)accelSteps;                             // steps/s^2

  if (vmax < 1.0f) vmax = 1.0f;
  if (a    < 1.0f) a    = 1.0f;

  // Gesamt-Rampenweg (Accel+Decel): vmax^2 / a
  const float d_ramp_total = (vmax * vmax) / a;

  float t;
  if (d >= d_ramp_total) {
    // Trapez: hoch, konstant, runter
    t = (2.0f * vmax / a) + ((d - d_ramp_total) / vmax);
  } else {
    // Dreieck: erreicht vmax nie
    t = 2.0f * sqrtf(d / a);
  }

  if (t < 0) t = 0;
  return (uint32_t)lroundf(t);
}


void saveParams() {
  EBlob b;
  b.magic = EEPROM_MAGIC;
  b.ver = EEPROM_VER;
  b.p = P;
  eepromUpdateStruct(0, b);
}

// ======================= Orbit Keyframes (compressed) =======================
uint16_t orbitStepKey[ORBIT_MAX_KEYS];
uint16_t orbitCamKey[ORBIT_MAX_KEYS];
uint16_t orbitKeyCount = 0;
uint16_t orbitKeyIndex = 0;

static inline float clampf(float v, float a, float b) {
  if (v < a) return a;
  if (v > b) return b;
  return v;
}

static inline long degToCamSteps(float deg) {
  // 0..180deg => 0..1600 steps
  deg = clampf(deg, 0.0f, 180.0f);
  float s = deg * (float)CAM_TOTAL_STEPS_180 / 180.0f;
  return lroundf(s);
}
void returnCamToStartAfterHome() {
  enableY(true);

  cam.setEnablePin(Y_ENABLE_PIN);
  cam.setPinsInverted(false, false, true);
  cam.setMaxSpeed(6000.0f);
  cam.setAcceleration(20000.0f);

  long target;
  if (P.orbitOn) {
    // Orbit-Startwinkel bei Slider=0
    float a0 = orbitAngleDegForStep(0, P.distanceM);
    target = degToCamSteps(a0);
  } else {
    // Normal: StartAngle
    target = degToCamSteps((float)P.startAngle);
  }

  cam.moveTo(target);
  while (cam.distanceToGo() != 0) {
    cam.run();
  }

  //enableY(false);
}


// Orbit angle for slider step (0..SLIDER_TOTAL_STEPS)
static float orbitAngleDegForStep(long sliderStep, float distanceM) {
  float d = (distanceM < 0.01f) ? 0.01f : distanceM;

  // sliderStep -> x in meters [-L/2 .. +L/2]
  float x = ((float)sliderStep / (float)SLIDER_TOTAL_STEPS) * SLIDER_LENGTH_M; // 0..L
  x -= SLIDER_LENGTH_M * 0.5f; // -L/2..+L/2

  // Object at (0, d), camera at (x,0), vector to object = (-x, d)
  // Angle from +x axis: atan2(d, -x) -> 0..180
  float ang = atan2f(d, -x) * 180.0f / 3.14159265f;
  return clampf(ang, 0.0f, 180.0f);
}

bool precomputeOrbitKeys() {
  orbitKeyCount = 0;
  orbitKeyIndex = 0;

  long lastCam = -999999;

  // Schrittweite so wählen, dass wir garantiert unter ORBIT_MAX_KEYS bleiben
  const uint16_t minStepForMemory =
      (uint16_t)(SLIDER_TOTAL_STEPS / (long)(ORBIT_MAX_KEYS - 2) + 1);

  // passend zur 100Hz Kamera-Update-Rate
  uint16_t stepFromSpeed = (uint16_t)(mmpsToSps(speed10ToMmps(P.speed10)) / 50);

  if (stepFromSpeed < 1) stepFromSpeed = 1;

  // HIER: kein max<uint16_t> verwenden!
  uint16_t stepInc = (minStepForMemory > stepFromSpeed) ? minStepForMemory : stepFromSpeed;
  if (stepInc < 2) stepInc = 2;

  for (long s = 0; s <= SLIDER_TOTAL_STEPS; s += stepInc) {
    float ang = orbitAngleDegForStep(s, P.distanceM);
    long camSteps = degToCamSteps(ang);

    if (camSteps != lastCam) {
      if (orbitKeyCount >= ORBIT_MAX_KEYS) return false;
      orbitStepKey[orbitKeyCount] = (uint16_t)s;
      orbitCamKey[orbitKeyCount]  = (uint16_t)camSteps;
      orbitKeyCount++;
      lastCam = camSteps;
    }
  }

  // End-Key sicherstellen
  if (orbitKeyCount == 0 || orbitStepKey[orbitKeyCount - 1] != (uint16_t)SLIDER_TOTAL_STEPS) {
    float ang = orbitAngleDegForStep(SLIDER_TOTAL_STEPS, P.distanceM);
    long camSteps = degToCamSteps(ang);
    if (orbitKeyCount >= ORBIT_MAX_KEYS) return false;
    orbitStepKey[orbitKeyCount] = (uint16_t)SLIDER_TOTAL_STEPS;
    orbitCamKey[orbitKeyCount]  = (uint16_t)camSteps;
    orbitKeyCount++;
  }

  return true;
}


uint16_t camTargetFromKeys(long sliderStep) {
  if (orbitKeyCount == 0) return 0;
  if (orbitKeyCount == 1) return orbitCamKey[0];

  if (sliderStep <= orbitStepKey[0]) return orbitCamKey[0];
  if (sliderStep >= orbitStepKey[orbitKeyCount - 1]) return orbitCamKey[orbitKeyCount - 1];

  // finde i so, dass step[i] <= sliderStep < step[i+1]
  int lo = 0;
  int hi = orbitKeyCount - 2; // -2 weil wir i+1 brauchen
  while (lo < hi) {
    int mid = (lo + hi + 1) / 2;
    if (orbitStepKey[mid] <= sliderStep) lo = mid;
    else hi = mid - 1;
  }

  uint16_t s0 = orbitStepKey[lo];
  uint16_t s1 = orbitStepKey[lo + 1];
  uint16_t c0 = orbitCamKey[lo];
  uint16_t c1 = orbitCamKey[lo + 1];

  if (s1 == s0) return c1;

  float t = (float)(sliderStep - s0) / (float)(s1 - s0);
  float cf = (float)c0 + t * ((float)c1 - (float)c0);
  long out = lroundf(cf);

  if (out < 0) out = 0;
  if (out > 65535) out = 65535;
  return (uint16_t)out;
}



// ======================= OLED Helpers =======================
void oledInit() {
  Wire.begin();
  Wire.setClock(400000L);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void oledText(const __FlashStringHelper *l1,
              const __FlashStringHelper *l2 = nullptr,
              const __FlashStringHelper *l3 = nullptr) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(l1);
  if (l2) display.println(l2);
  if (l3) display.println(l3);
  display.display();
}

void oledMainMenu(uint8_t idx) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("Menu"));
  display.println(F("-----"));
  const char* items[] = {"Kalibrieren","Parameter","Orbit Setup","Start"};
  for (uint8_t i=0;i<4;i++){
    display.print(i==idx?F("> "):F("  "));
    display.println(items[i]);
  }

  display.println();

  // Statuszeile: Orbit + Richtung
  display.print(F("Orbit: "));
  display.print(P.orbitOn ? F("ON") : F("OFF"));
  display.print(F("   Dir: "));
  display.println(P.reverseDir ? F("-->") : F("<--"));

  display.display();
}

void oledOrbitMenu(uint8_t idx) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("Orbit Setup"));
  display.println(F("----------"));

  display.print(idx==0 ? F("> ") : F("  "));
  display.print(F("Orbit: "));
  display.println(P.orbitOn ? F("ON") : F("OFF"));

  display.print(idx==1 ? F("> ") : F("  "));
  display.print(F("Dist(m): "));
  display.println(P.distanceM, 2);

  display.println();
  display.println(F("+/- change"));
  display.println(F("OK next, BACK menu"));
  display.display();
}

void oledParams(uint8_t pidx) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("Parameter"));
  display.println(F("---------"));

  switch (pidx) {
    case 0:  display.print(F("> Dir: "));   display.println(P.reverseDir ? F("-->") : F("<--")); break;
    case 1: {
      display.print(F("> Speed: "));
      display.print(P.speed10 / 10);
      display.print('.');
      display.print(P.speed10 % 10);
      display.println(F(" mm/s"));

      uint32_t passSec  = estimateRunSecOnePass(P.speed10, P.accel);

      // Gesamtzeit: LoopCount * Pass + Delay nur 1x am Anfang (so wie dein Ablauf aktuell ist)
      uint32_t totalSec = passSec * (uint32_t)P.loopCount + (uint32_t)P.delaySec;

      char tPass[9], tTot[9];
      fmtHMS(passSec, tPass, sizeof(tPass));
      fmtHMS(totalSec, tTot, sizeof(tTot));

      display.print(F("Pass: "));
      display.println(tPass);

      display.print(F("Total: "));
      display.println(tTot);
    } break;
    case 2: display.print(F("> Accel: ")); display.println(P.accel); break;
    case 3:  display.print(F("> StartAng: ")); display.println(P.startAngle); break;
    case 4:  display.print(F("> EndAng: "));   display.println(P.endAngle); break;
    case 5:  display.print(F("> Delay(s): ")); display.println(P.delaySec); break;
    case 6:  display.print(F("> Loop: "));     display.print(P.loopCount); display.println(F("x")); break; // <-- NEU
  }

  display.println();
  display.println(F("+/- change, OK next"));
  display.println(F("BACK = menu"));
  display.display();
}


void oledRunScreen() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.println(F("Run"));
  display.setTextSize(1);

  display.print(F("OK=Pause  BACK=Abort"));
  display.println();

  if (P.orbitOn) display.println(F("Orbit: ON"));
  else          display.println(F("Orbit: OFF"));

  display.display();
}
void oledPausedScreen() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.println(F("Pause"));
  display.setTextSize(1);
  display.println();
  display.println(F("OK=Resume"));
  display.println(F("BACK=Abort"));
  display.display();
}


void oledGoingHomeScreen() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.println(F("Going home"));
  display.println(F("..."));
  display.display();
}

void oledErrorScreen() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("ERROR"));
  display.println(F("Homing timeout"));
  display.println(F("or orbit memory"));
  display.println();
  display.println(F("OK = back to menu"));
  display.display();
}

// ======================= Enable / Endstop helpers =======================
void enableX(bool on) { digitalWrite(X_ENABLE_PIN, on ? LOW : HIGH); }
void enableY(bool on) { digitalWrite(Y_ENABLE_PIN, on ? LOW : HIGH); }

bool sliderHomeHit() { return digitalRead(SLIDER_HOME_PIN) == LOW; } // INPUT_PULLUP
bool camHomeHit()    { return digitalRead(CAM_HOME_PIN) == LOW; }    // INPUT_PULLUP

// Fixed-speed stepping pulse for homing (no accel)
void pulseStep(uint8_t stepPin, uint16_t stepsPerSec) {
  uint32_t halfUs = 500000UL / stepsPerSec;
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(halfUs);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(halfUs);
}

// ======================= State Machine =======================
enum State {
  ST_PRESS_START,
  ST_CALIBRATING,
  ST_MENU,
  ST_PARAMS,
  ST_ORBIT_MENU, 
  ST_ORBIT_PREP,
  ST_READY,
  ST_DELAY,
  ST_RUN_OUT,
  ST_RUN_PAUSING,     
  ST_RUN_PAUSED,
  ST_RUN_STOPPING,
  ST_GO_HOME,
  ST_ERROR
};

State state = ST_PRESS_START;
bool needsRedraw = true;

uint8_t menuIndex = 0;
uint8_t paramIndex = 0;

uint32_t delayStartMs = 0;
uint32_t homeStartMs = 0;

uint32_t lastCamUpdateMs = 0;

uint8_t orbitMenuIndex = 0;   // 0=Orbit ON/OFF, 1=Distance


// ======================= Calibration =======================
bool calibrateAll() {
  // Show once (no updates during motion)
  oledText(F("Calibrating..."), F("Please wait"));

  // -------- Slider homing --------
  enableX(true);

  // Unblock: move away from X- endstop
  // Assumption: away = DIR HIGH, home = DIR LOW.
  // If wrong: swap HIGH/LOW here and below.
  digitalWrite(X_DIR_PIN, HIGH);
  for (int i=0;i<SLIDER_UNBLOCK_STEPS;i++) pulseStep(X_STEP_PIN, SLIDER_HOME_SPEED_SPS);

  // Home: move towards endstop
  digitalWrite(X_DIR_PIN, LOW);
  long safety = SLIDER_TOTAL_STEPS + 2000;
  while (!sliderHomeHit() && safety--) {
    pulseStep(X_STEP_PIN, SLIDER_HOME_SPEED_SPS);
  }
  if (!sliderHomeHit()) {  return false; }

  slider.setCurrentPosition(0);
  

  // -------- Camera homing --------
  enableY(true);

  // Unblock: move away from camera endstop
  digitalWrite(Y_DIR_PIN, HIGH);
  for (int i=0;i<CAM_UNBLOCK_STEPS;i++) pulseStep(Y_STEP_PIN, CAM_HOME_SPEED_SPS);

  // Home: move towards camera endstop
  digitalWrite(Y_DIR_PIN, LOW);
  long camSafety = CAM_TOTAL_STEPS_180 + 2000;
  while (!camHomeHit() && camSafety--) {
    pulseStep(Y_STEP_PIN, CAM_HOME_SPEED_SPS);
  }
  if (!camHomeHit()) { enableY(false); return false; }

  cam.setCurrentPosition(0);
  //enableY(false);

  return true;
}

static inline long runStartS() { return P.reverseDir ? SLIDER_TOTAL_STEPS : 0; }
static inline long runEndS()   { return P.reverseDir ? 0 : SLIDER_TOTAL_STEPS; }

static long camTargetForSliderPos(long s) {
  if (P.orbitOn) {
    // Orbit-Keyframes sind positionsbasiert (0..TOTAL)
    return (long)camTargetFromKeys(s);
  } else {
    // Normal: StartAngle bei s=0, EndAngle bei s=TOTAL
    float t = (float)s / (float)SLIDER_TOTAL_STEPS;
    float ang = (float)P.startAngle + t * ((float)P.endAngle - (float)P.startAngle);
    return degToCamSteps(ang);
  }
}

static void moveCamBlocking(long targetSteps) {
  cam.moveTo(targetSteps);
  while (cam.distanceToGo() != 0) {
    cam.run();
  }
}

static void moveSliderBlocking(long targetSteps) {
  slider.moveTo(targetSteps);
  while (slider.distanceToGo() != 0) {
    slider.run();
  }
}


// ======================= Run Setup =======================
void setupForRunOut() {


  // Slider Setup
  slider.setEnablePin(X_ENABLE_PIN);
  slider.setPinsInverted(false, false, true);

  // Camera Setup
  cam.setEnablePin(Y_ENABLE_PIN);
  cam.setPinsInverted(false, false, true);
  cam.setMaxSpeed(6000.0f);
  cam.setAcceleration(20000.0f);

  enableX(true);
  enableY(true);

  // Orbit-Keys müssen vorhanden sein (ST_ORBIT_PREP macht das normalerweise)
  if (P.orbitOn && orbitKeyCount == 0) {
    precomputeOrbitKeys();
  }

  const long startPos = loopStartPos;
  const long targetPos = passToEnd ? loopEndPos : loopStartPos;
  // 1) Kamera zuerst in die Start-Ausrichtung bringen (Orbit oder Normal!)
  long camStart = camTargetForSliderPos(startPos);

  cam.setMaxSpeed(CAM_POS_MAXSPEED);
  cam.setAcceleration(CAM_POS_ACCEL);
  


  moveCamBlocking(camStart);

  // 2) Dann Slider zur Startposition fahren (nur nötig bei R->L)
  if (slider.currentPosition() != startPos) {
    // einmalige Anzeige ist ok (kein Update während Bewegung)
    oledText(F("Move to start"), P.reverseDir ? F("-->") : F("<--"));
    moveSliderBlocking(startPos);
  }

  slider.setMaxSpeed((float)mmpsToSps(speed10ToMmps(P.speed10)));


  slider.setAcceleration((float)P.accel);

  // merken, wohin wir nach dem Homing zurück wollen (wenn du das weiterhin nutzt)
  camReturnPosSteps = cam.currentPosition();

  // 3) Run Ziel setzen
slider.moveTo(targetPos);

runEndTargetSteps = targetPos;
  runWasAborted = false;
  camLastTarget = LONG_MIN;   // damit Kamera-Target nach Start sauber gesetzt wird

  lastCamUpdateMs = millis();

  cam.setMaxSpeed(CAM_RUN_MAXSPEED);
  cam.setAcceleration(CAM_RUN_ACCEL);
}

void returnCamToStored() {
  
  cam.setMaxSpeed(CAM_POS_MAXSPEED);
  cam.setAcceleration(CAM_POS_ACCEL);

  cam.setEnablePin(Y_ENABLE_PIN);
  cam.setPinsInverted(false, false, true);
  cam.setMaxSpeed(2000.0f);        // bewusst moderater
  cam.setAcceleration(6000.0f);

  enableY(true);

  cam.moveTo(camReturnPosSteps);
  while (cam.distanceToGo() != 0) {
    cam.run();
  }

  
}

// ======================= Camera update during run (NO DISPLAY) =======================
void updateCamDuringRun() {
  uint32_t now = millis();
  if (now - lastCamUpdateMs < 10) return;
  lastCamUpdateMs = now;

  long s = slider.currentPosition();
  if (s < 0) s = 0;
  if (s > SLIDER_TOTAL_STEPS) s = SLIDER_TOTAL_STEPS;

  long targetCamSteps = camTargetForSliderPos(s);

  if (targetCamSteps != camLastTarget) {
    cam.moveTo(targetCamSteps);
    camLastTarget = targetCamSteps;
  }
}



// ======================= Go Home =======================
bool goHomeWithTimeout() {
  oledGoingHomeScreen();
  enableX(true);

  // --- 1) Smooth zurück auf Position 0 (AccelStepper mit Accel) ---
  slider.setEnablePin(X_ENABLE_PIN);
  slider.setPinsInverted(false, false, true);

  slider.setMaxSpeed((float)SLIDER_HOME_RAPID_SPS);   // z.B. 3000
  slider.setAcceleration((float)P.accel);             // HIER stellst du die Beschleunigung ein!

  homeStartMs = millis();
  slider.moveTo(0);

  // Kamera für Rückdrehen langsam konfigurieren
enableY(true);
cam.setEnablePin(Y_ENABLE_PIN);
cam.setPinsInverted(false, false, true);
cam.setMaxSpeed(CAM_POS_MAXSPEED);     // oder kleiner machen, wenn es noch langsamer sein soll
cam.setAcceleration(CAM_POS_ACCEL);
camLastTarget = LONG_MIN;
lastCamUpdateMs = 0;

slider.moveTo(0);

while (slider.distanceToGo() != 0) {
  if (millis() - homeStartMs > HOME_TIMEOUT_MS) {
    
    return false;
  }

  slider.run();

  // Kamera-Target passend zur aktuellen Slider-Position (Orbit oder normal)
  updateCamDuringRun();
  cam.run();
}

// optional: Kamera sauber “auslaufen” lassen
while (cam.distanceToGo() != 0) cam.run();


  // --- 2) Endstop langsam "antasten" + Backoff + zweite Anfahrt (wie vorher) ---
  // Richtung zum X- Endstop (so wie du es bisher machst)
  digitalWrite(X_DIR_PIN, LOW);

  long safety = 800; // ein paar Schritte reichen, weil wir schon bei pos=0 sind
  while (!sliderHomeHit() && safety--) {
    if (millis() - homeStartMs > HOME_TIMEOUT_MS) {
      
      return false;
    }
    pulseStep(X_STEP_PIN, SLIDER_HOME_SLOW_SPS);
  }
  if (!sliderHomeHit()) {
    
    return false;
  }

  // Backoff
  digitalWrite(X_DIR_PIN, HIGH);
  for (uint16_t i = 0; i < SLIDER_HOME_BACKOFF_STEPS; i++) {
    pulseStep(X_STEP_PIN, SLIDER_HOME_SLOW_SPS);
  }

  // zweite, langsame Anfahrt
  digitalWrite(X_DIR_PIN, LOW);
  safety = SLIDER_HOME_BACKOFF_STEPS + 200;
  while (!sliderHomeHit() && safety--) {
    pulseStep(X_STEP_PIN, SLIDER_HOME_SLOW_SPS);
  }

  slider.setCurrentPosition(0);
  

  returnCamToStored();
  return true;
}


// ======================= Arduino Setup/Loop =======================
void setup() {
  // Step/Dir/Enable pins
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);

  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);

  enableX(true);
  enableY(true);

  // Endstops
  pinMode(SLIDER_HOME_PIN, INPUT_PULLUP);
  pinMode(CAM_HOME_PIN, INPUT_PULLUP);

  // Buttons
  pinMode(BTN_PLUS_PIN, INPUT_PULLUP);
  pinMode(BTN_MINUS_PIN, INPUT_PULLUP);
  pinMode(BTN_OK_PIN, INPUT_PULLUP);
  pinMode(BTN_BACK_PIN, INPUT_PULLUP);

  oledInit();
  loadParams();

  state = ST_PRESS_START;
  needsRedraw = true;
}

void loop() {
bool pPlus  = readButtonEdge(BI_PLUS);
bool pMinus = readButtonEdge(BI_MINUS);
bool pOk    = readButtonEdge(BI_OK);
bool pBack  = readButtonEdge(BI_BACK);


switch (state) {

  case ST_PRESS_START: {
    if (needsRedraw) {
      oledText(F("Press Start"), F("OK = calibrate"));
      needsRedraw = false;
    }
    if (pOk) {
      state = ST_CALIBRATING;
      needsRedraw = true;
    }
  } break;

  case ST_CALIBRATING: {
    bool ok = calibrateAll();
    state = ok ? ST_MENU : ST_ERROR;
    needsRedraw = true;
  } break;

  case ST_MENU: {
    if (needsRedraw) {
      oledMainMenu(menuIndex);
      needsRedraw = false;
    }

    if (pPlus)  { if (menuIndex > 0) menuIndex--; needsRedraw = true; }
    if (pMinus) { if (menuIndex < 3) menuIndex++; needsRedraw = true; }

    if (pOk) {
      if (menuIndex == 0) {
        state = ST_CALIBRATING;
      }
      else if (menuIndex == 1) {
        state = ST_PARAMS;
      }
      else if (menuIndex == 2) {
        orbitMenuIndex = 0;
        state = ST_ORBIT_MENU;
      }
      else if (menuIndex == 3) {
        // Start: Loop-/Auto-Flags sauber zurücksetzen
        loopCur = 1;
        autoStartAfterPrep = false;
        runWasAborted = false;

        state = ST_ORBIT_PREP;
      }
      needsRedraw = true;
    }
  } break;

  case ST_ORBIT_MENU: {
    if (needsRedraw) {
      oledOrbitMenu(orbitMenuIndex);
      needsRedraw = false;
    }

    if (pBack) {
      if (paramsDirty) { saveParams(); paramsDirty = false; }
      state = ST_MENU;
      needsRedraw = true;
      break;
    }

    if (pPlus || pMinus) {
      int dir = pPlus ? +1 : -1;

      if (orbitMenuIndex == 0) {
        // Orbit ON/OFF toggeln
        P.orbitOn = !P.orbitOn;

        orbitKeyCount = 0;   // <-- HIER: Keys ungültig machen

        paramsDirty = true;
        needsRedraw = true;
      }
      else if (orbitMenuIndex == 1) {
        // Distance
        float v = P.distanceM + dir * 0.01f;
        v = clampf(v, 0.05f, 5.0f);
        P.distanceM = v;

        orbitKeyCount = 0;   // <-- HIER: Keys ungültig machen

        paramsDirty = true;
        needsRedraw = true;
      }
    }

    if (pOk) {
      orbitMenuIndex++;
      if (orbitMenuIndex > 1) orbitMenuIndex = 0;
      needsRedraw = true;
    }
  } break;

  case ST_PARAMS: {
    if (needsRedraw) {
      oledParams(paramIndex);
      needsRedraw = false;
    }

    if (pBack) {
      if (paramsDirty) { saveParams(); paramsDirty = false; }
      state = ST_MENU;
      needsRedraw = true;
      break;
    }


    if (pPlus || pMinus) {
      int dir = pPlus ? +1 : -1;

      switch (paramIndex) {
        case 0: { // Direction toggle
          P.reverseDir = !P.reverseDir;
        } break;

        case 1: { // Speed (0.1mm/s steps)
          changeSpeed10(P.speed10, dir);

          orbitKeyCount = 0; // optional sinnvoll: Orbit sampling hängt an Speed
        } break;

        case 2: { // Accel
          int v = (int)P.accel + dir * 200;
          if (v < 0) v = 0;
          if (v > 20000) v = 20000;
          P.accel = (uint16_t)v;
        } break;

        case 3: { // Start angle
          int v = (int)P.startAngle + dir;
          if (v < 0) v = 0;
          if (v > 180) v = 180;
          P.startAngle = (uint8_t)v;
        } break;

        case 4: { // End angle
          int v = (int)P.endAngle + dir;
          if (v < 0) v = 0;
          if (v > 180) v = 180;
          P.endAngle = (uint8_t)v;
        } break;

        case 5: { // Delay sec
          int v = (int)P.delaySec + dir;
          if (v < 0) v = 0;
          if (v > 60) v = 60;
          P.delaySec = (uint16_t)v;
        } break;

        case 6: { // Loop count
          int v = (int)P.loopCount + dir;
          if (v < 1) v = 1;
          if (v > 99) v = 99;
          P.loopCount = (uint8_t)v;
        } break;
      }

      paramsDirty = true;
      needsRedraw = true;

    }

    if (pOk) {
      paramIndex++;
      if (paramIndex > 6) paramIndex = 0;
      needsRedraw = true;
    }
  } break;

  case ST_ORBIT_PREP: {
    // Orbit Keys nur wenn Orbit aktiv
    if (P.orbitOn) {
      oledText(F("Orbit calc..."), F("please wait"));
      bool ok = precomputeOrbitKeys();
      if (!ok) { state = ST_ERROR; needsRedraw = true; break; }
    }

    // Direkt auf Startposition fahren (Eilgang) und Kamera in Startausrichtung bringen
    oledText(F("Move to start"), P.reverseDir ? F("-->") : F("<--"));

    // Steppers aktivieren & konfigurieren
    slider.setEnablePin(X_ENABLE_PIN);
    slider.setPinsInverted(false, false, true);
    cam.setEnablePin(Y_ENABLE_PIN);
    cam.setPinsInverted(false, false, true);

    enableX(true);
    enableY(true);

    const long startPos = runStartS();
    long camStart = camTargetForSliderPos(startPos);

    // Kamera positionieren
    cam.setMaxSpeed(CAM_POS_MAXSPEED);
    cam.setAcceleration(CAM_POS_ACCEL);
    moveCamBlocking(camStart);

    // Slider positionieren (Eilgang)
    slider.setMaxSpeed(SLIDER_POS_MAXSPEED);
    slider.setAcceleration(SLIDER_POS_ACCEL);
    moveSliderBlocking(startPos);

    // Merken für Rückstellen nach Homing
    camReturnPosSteps = cam.currentPosition();

    // Wenn Loop-Weiterlauf: direkt in Delay, sonst Ready anzeigen
    if (autoStartAfterPrep) {
      state = ST_DELAY;
      delayStartMs = millis();
      oledText(F("Delay..."), F("get ready"));
      needsRedraw = false;
    } else {
      state = ST_READY;
      needsRedraw = true;
    }
  } break;

  case ST_READY: {
    if (needsRedraw) {
      display.clearDisplay();
      display.setCursor(0,0);
      display.println(F("Ready"));
      display.println(F("OK = start run"));
      display.println(F("BACK = menu"));
      display.println();
      display.print(F("Orbit: "));
      display.println(P.orbitOn ? F("ON") : F("OFF"));
      display.print(F("Dir: "));
      display.println(P.reverseDir ? F("-->") : F("<--"));
      display.display();
      needsRedraw = false;
    }

    if (pBack) { state = ST_MENU; needsRedraw = true; }

    if (pOk) {
      loopCur = 1;
      autoStartAfterPrep = false;
      runWasAborted = false;

      loopStartPos = runStartS();
      loopEndPos   = runEndS();
      passToEnd    = true;

      state = ST_DELAY;
      delayStartMs = millis();
      oledText(F("Delay..."), F("get ready"));
      needsRedraw = false;
    }

  } break;

  case ST_DELAY: {
    if (millis() - delayStartMs >= (uint32_t)P.delaySec * 1000UL) {
      oledRunScreen();
      setupForRunOut();          // setzt runEndTargetSteps, camLastTarget usw.
      state = ST_RUN_OUT;
      needsRedraw = false;
    }
  } break;

  case ST_RUN_OUT: {
    // OK = Pause
    if (pOk) {
      slider.stop();
      //cam.stop();
      state = ST_RUN_PAUSING;
      needsRedraw = false;
      break;
    }

    // BACK = Abort
    if (pBack) {
      runWasAborted = true;
      slider.stop();
      cam.stop();
      state = ST_RUN_STOPPING;
      needsRedraw = true;
      break;
    }

    // normal laufen
    slider.run();
    updateCamDuringRun();
    cam.run();

    if (slider.distanceToGo() == 0) {
      state = ST_GO_HOME;
      needsRedraw = true;
    }
  } break;

  case ST_RUN_PAUSING: {

    slider.run();
    updateCamDuringRun();
    cam.run();


    if (slider.distanceToGo() == 0 && cam.distanceToGo() == 0 &&
        slider.speed() == 0.0f && cam.speed() == 0.0f) {
      state = ST_RUN_PAUSED;
      needsRedraw = true;
    }
  } break;

  case ST_RUN_PAUSED: {
    if (needsRedraw) {
      oledPausedScreen();
      needsRedraw = false;
    }

    // OK = Resume
    if (pOk) {
      slider.moveTo(runEndTargetSteps);
      camLastTarget = LONG_MIN;
      lastCamUpdateMs = millis();

      oledRunScreen();
      state = ST_RUN_OUT;
      needsRedraw = false;
      break;
    }

    // BACK = Abort
    if (pBack) {
      runWasAborted = true;
      slider.stop();
      cam.stop();
      state = ST_RUN_STOPPING;
      needsRedraw = true;
      break;
    }
  } break;

  case ST_RUN_STOPPING: {
    // KEIN updateCamDuringRun() hier!
    slider.run();
    cam.run();

    if (slider.distanceToGo() == 0 && cam.distanceToGo() == 0 &&
        slider.speed() == 0.0f && cam.speed() == 0.0f) {
      state = ST_GO_HOME;
      needsRedraw = true;
    }
  } break;

  case ST_GO_HOME: {

  // Wenn abgebrochen: immer echten Home/Endstop machen
  if (runWasAborted) {
    bool ok = goHomeWithTimeout();
    state = ok ? ST_MENU : ST_ERROR;
    needsRedraw = true;
    break;
  }

  // Loop / Ping-Pong: nächsten Pass direkt starten
  if (P.loopCount > 1 && loopCur < P.loopCount) {
    loopCur++;
    passToEnd = !passToEnd;

    long nextTarget = passToEnd ? loopEndPos : loopStartPos;

    // Run-Parameter sicher setzen
    enableX(true);
    enableY(true);

    slider.setMaxSpeed((float)mmpsToSps(speed10ToMmps(P.speed10)));
    slider.setAcceleration((float)P.accel);
    cam.setMaxSpeed(CAM_RUN_MAXSPEED);
    cam.setAcceleration(CAM_RUN_ACCEL);

    slider.moveTo(nextTarget);
    runEndTargetSteps = nextTarget;
    camLastTarget = LONG_MIN;
    lastCamUpdateMs = millis();

    // Einmalige Anzeige ist ok
    oledRunScreen();

    state = ST_RUN_OUT;
    needsRedraw = false;
    break;
  }

  // Letzter Pass fertig -> jetzt wirklich homen
  bool ok = goHomeWithTimeout();
  state = ok ? ST_MENU : ST_ERROR;
  needsRedraw = true;

  } break;


  case ST_ERROR: {
    if (needsRedraw) {
      oledErrorScreen();
      needsRedraw = false;
    }

    enableX(false);
    enableY(false);

    if (pOk) {
      state = ST_MENU;
      needsRedraw = true;
    }
  } break;
}

  
}
