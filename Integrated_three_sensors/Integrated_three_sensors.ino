/*
 * Smart Blind Stick — Combined Sketch
 * CO3302 Computer Engineering Project — Group 08
 * ================================================================
 * NOTE ON THIS FILE: your handoff notes referenced an existing combined
 * sketch (sensors + motor + buzzer + SOS placeholder) but its code text
 * wasn't available to rebuild from directly, so this file is written
 * fresh from your documented pin map, calibration values, and behavior
 * rules below. Two places where the notes were ambiguous, I made an
 * explicit choice — flagged with "DESIGN CHOICE" comments — please
 * sanity-check those against what you actually had before:
 *
 *   1. Vibration patterns per hazard type weren't specified beyond
 *      "three distinct patterns implemented" — I designed: obstacle =
 *      pulse rate that speeds up as distance decreases, water = a
 *      double-pulse pattern, dark = a slow single long pulse.
 *   2. SOS confirmation: your notes say the buzzer is "reserved
 *      exclusively for Find-My-Stick" but also that the SOS placeholder
 *      "beeps to confirm press." Since Find-My-Stick's buzzer state
 *      machine now owns D7 exclusively (and mixing the two would cause
 *      exactly the kind of RF/buzzer conflict you just spent a while
 *      debugging), SOS confirmation here uses a distinct vibration
 *      pattern instead of the buzzer. Swap it back if you actually want
 *      the buzzer for SOS too.
 *
 * GPS / SMS: still not implemented, per your instruction. sosHandler()
 * below only prints a placeholder to Serial and gives the vibration
 * confirmation — the AT+CIPGSMLOC / AT+CMGS logic is a separate,
 * later piece of work and is NOT in this file.
 *
 * Priority logic: Obstacle > Water > Dark, only the highest-priority
 * hazard drives the vibration motor at any moment (per your notes).
 * Find-My-Stick (buzzer) and SOS (vibration override) are independent
 * of hazard sensing and can interrupt/coexist as described inline.
 *
 * Everything here is non-blocking (no delay() in loop()), consistent
 * with the two bugs already found and fixed in this project (blocking
 * pulseIn() on the ultrasonic sensor, and a blocking delay(500) in the
 * old SOS handler — both corrupted RF timing).
 * ================================================================
 */

#include <RCSwitch.h>

// ---------------------------------------------------------------------
// PIN MAP (matches your documented table)
// ---------------------------------------------------------------------
const int PIN_RF_RX        = 2;   // RF RX signal, hardware interrupt (rc-switch)
// D3 / D4 reserved for SIM800L (TXD/RXD) — not used until GPS/SMS work begins
const int PIN_VIBRATION    = 5;   // Vibration motor, via BC547 transistor
const int PIN_SOS_BUTTON   = 6;   // SOS button, INPUT_PULLUP, debounced
const int PIN_BUZZER       = 7;   // Buzzer — Find-My-Stick ONLY
const int PIN_US_ECHO      = 8;   // HC-SR04 ECHO
const int PIN_US_TRIG      = 9;   // HC-SR04 TRIG
const int PIN_MOISTURE_AO  = A0;  // Moisture sensor analog out
const int PIN_LDR_AO       = A1;  // LDR module analog out

// ---------------------------------------------------------------------
// CALIBRATED THRESHOLDS (from your testing)
// ---------------------------------------------------------------------
const int WET_THRESHOLD   = 850;  // below this = wet (dry ~1023, wet ~714)
const int DARK_THRESHOLD  = 400;  // above this = dark (bright ~20, normal ~100, covered ~800)
const int OBSTACLE_CM     = 50;   // obstacle alert distance

// =======================================================================
// SECTION 1 — ULTRASONIC (non-blocking)
// =======================================================================
enum UsState { US_IDLE, US_TRIGGERED, US_ECHO_STARTED };
UsState us_state = US_IDLE;
unsigned long us_lastCycleMs = 0;
unsigned long us_triggerStartUs = 0;
unsigned long us_echoStartUs = 0;
const unsigned long US_CYCLE_MS   = 60;    // ~min gap between pings
const unsigned long US_TIMEOUT_US = 30000; // ~5m round-trip timeout
long us_distanceCm = -1; // -1 = no echo / out of range

void ultrasonic_loop() {
  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  switch (us_state) {
    case US_IDLE:
      if (nowMs - us_lastCycleMs >= US_CYCLE_MS) {
        digitalWrite(PIN_US_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_US_TRIG, HIGH);
        delayMicroseconds(10);   // negligible, not the blocking culprit (pulseIn() was)
        digitalWrite(PIN_US_TRIG, LOW);
        us_triggerStartUs = micros();
        us_state = US_TRIGGERED;
      }
      break;

    case US_TRIGGERED:
      if (digitalRead(PIN_US_ECHO) == HIGH) {
        us_echoStartUs = nowUs;
        us_state = US_ECHO_STARTED;
      } else if (nowUs - us_triggerStartUs > US_TIMEOUT_US) {
        us_distanceCm = -1;
        us_lastCycleMs = nowMs;
        us_state = US_IDLE;
      }
      break;

    case US_ECHO_STARTED:
      if (digitalRead(PIN_US_ECHO) == LOW) {
        unsigned long duration = nowUs - us_echoStartUs;
        us_distanceCm = duration / 58; // standard us->cm conversion
        us_lastCycleMs = nowMs;
        us_state = US_IDLE;
      } else if (nowUs - us_echoStartUs > US_TIMEOUT_US) {
        us_distanceCm = -1;
        us_lastCycleMs = nowMs;
        us_state = US_IDLE;
      }
      break;
  }
}

// =======================================================================
// SECTION 2 — MOISTURE (300ms debounce)
// =======================================================================
bool moisture_isWet = false;
bool moisture_pendingWet = false;
unsigned long moisture_pendingSinceMs = 0;
const unsigned long MOISTURE_DEBOUNCE_MS = 300;

void moisture_loop() {
  int reading = analogRead(PIN_MOISTURE_AO);
  bool rawWet = (reading < WET_THRESHOLD);

  if (rawWet != moisture_pendingWet) {
    moisture_pendingWet = rawWet;
    moisture_pendingSinceMs = millis();
  } else if (millis() - moisture_pendingSinceMs >= MOISTURE_DEBOUNCE_MS) {
    moisture_isWet = moisture_pendingWet;
  }
}

// =======================================================================
// SECTION 3 — LDR (dark sensing)
// =======================================================================
bool ldr_isDark = false;

void ldr_loop() {
  int reading = analogRead(PIN_LDR_AO);
  ldr_isDark = (reading > DARK_THRESHOLD);
}

// =======================================================================
// SECTION 4 — HAZARD PRIORITY (Obstacle > Water > Dark)
// =======================================================================
enum Hazard { HAZ_NONE, HAZ_OBSTACLE, HAZ_WATER, HAZ_DARK };

// Returns int (not Hazard) on purpose -- the Arduino IDE auto-generates
// function prototypes and inserts them near the top of the file, before
// custom enum definitions exist yet. A function returning a custom enum
// type breaks that auto-generated prototype ("does not name a type").
// Returning int sidesteps it; enum constants convert to int implicitly.
int currentHazard() {
  if (us_distanceCm >= 0 && us_distanceCm <= OBSTACLE_CM) return HAZ_OBSTACLE;
  if (moisture_isWet) return HAZ_WATER;
  if (ldr_isDark) return HAZ_DARK;
  return HAZ_NONE;
}

// =======================================================================
// SECTION 5 — VIBRATION MOTOR (non-blocking, per-hazard patterns)
// DESIGN CHOICE: see note at top of file.
// =======================================================================
int vib_activeHazard = HAZ_NONE;
bool   vib_pinOn = false;
unsigned long vib_stateAt = 0;
bool   vib_sosOverride = false; // SOS pattern temporarily takes the pin

// Obstacle: on/off duty cycle whose period shrinks as distance shrinks
unsigned long obstaclePeriodMs() {
  long d = us_distanceCm;
  if (d < 0) d = OBSTACLE_CM;
  if (d > OBSTACLE_CM) d = OBSTACLE_CM;
  // 0cm -> ~80ms period (fast/urgent), 50cm -> ~400ms period (relaxed)
  return map(d, 0, OBSTACLE_CM, 80, 400);
}

// Water: double-pulse pattern, repeated (on,off,on,off-long-gap)
const unsigned long WATER_PHASE_MS[] = {120, 100, 120, 500};
const bool          WATER_PHASE_ON[] = {true, false, true, false};
const int WATER_PATTERN_LEN = 4;
int vib_waterStep = 0;

// Dark: slow single long pulse
const unsigned long DARK_ON_MS = 600;
const unsigned long DARK_OFF_MS = 600;

void vibration_setPin(bool on) {
  vib_pinOn = on;
  digitalWrite(PIN_VIBRATION, on ? HIGH : LOW);
  vib_stateAt = millis();
}

void vibration_loop() {
  if (vib_sosOverride) return; // SOS pattern owns the pin right now

  int h = currentHazard();
  if (h != vib_activeHazard) {
    // Hazard changed -> reset pattern state cleanly
    vib_activeHazard = h;
    vib_waterStep = 0;
    vibration_setPin(h == HAZ_WATER ? WATER_PHASE_ON[0] : false);
  }

  unsigned long elapsed = millis() - vib_stateAt;

  switch (vib_activeHazard) {
    case HAZ_NONE:
      if (vib_pinOn) vibration_setPin(false);
      break;

    case HAZ_OBSTACLE: {
      unsigned long period = obstaclePeriodMs();
      unsigned long half = period / 2;
      if (vib_pinOn && elapsed >= half) vibration_setPin(false);
      else if (!vib_pinOn && elapsed >= half) vibration_setPin(true);
      break;
    }

    case HAZ_WATER: {
      unsigned long stepDur = WATER_PHASE_MS[vib_waterStep];
      if (elapsed >= stepDur) {
        vib_waterStep = (vib_waterStep + 1) % WATER_PATTERN_LEN;
        vibration_setPin(WATER_PHASE_ON[vib_waterStep]);
      }
      break;
    }

    case HAZ_DARK: {
      unsigned long onOff = vib_pinOn ? DARK_ON_MS : DARK_OFF_MS;
      if (elapsed >= onOff) vibration_setPin(!vib_pinOn);
      break;
    }
  }
}

// Brief SOS confirmation pattern: 3 long pulses, then release the pin
// back to whatever hazard pattern is current.
bool   sos_vibActive = false;
int    sos_vibPulsesDone = 0;
const unsigned long SOS_VIB_ON_MS = 400;
const unsigned long SOS_VIB_OFF_MS = 200;
const int SOS_VIB_PULSES = 3;
unsigned long sos_vibStateAt = 0;
bool sos_vibPinOn = false;

void sosVibration_start() {
  vib_sosOverride = true;
  sos_vibActive = true;
  sos_vibPulsesDone = 0;
  sos_vibPinOn = true;
  sos_vibStateAt = millis();
  digitalWrite(PIN_VIBRATION, HIGH);
}

void sosVibration_loop() {
  if (!sos_vibActive) return;
  unsigned long elapsed = millis() - sos_vibStateAt;
  if (sos_vibPinOn && elapsed >= SOS_VIB_ON_MS) {
    digitalWrite(PIN_VIBRATION, LOW);
    sos_vibPinOn = false;
    sos_vibStateAt = millis();
    sos_vibPulsesDone++;
  } else if (!sos_vibPinOn && elapsed >= SOS_VIB_OFF_MS) {
    if (sos_vibPulsesDone >= SOS_VIB_PULSES) {
      sos_vibActive = false;
      vib_sosOverride = false; // hand the pin back to hazard logic
      vib_activeHazard = HAZ_NONE; // force a clean re-evaluation next loop
    } else {
      digitalWrite(PIN_VIBRATION, HIGH);
      sos_vibPinOn = true;
      sos_vibStateAt = millis();
    }
  }
}

// =======================================================================
// SECTION 6 — SOS BUTTON (debounced, edge-triggered, non-blocking)
// GPS/SMS is a TODO — see sosHandler() below.
// =======================================================================
bool sos_lastReading = HIGH;
bool sos_debouncedState = HIGH;
unsigned long sos_lastChangeMs = 0;
const unsigned long SOS_DEBOUNCE_MS = 50;

void sosHandler() {
  // TODO (next project step, GPS not ready yet):
  //   1. AT+CIPGSMLOC=1,1 to get real coordinates (currently returns
  //      0.000000,0.000000 indoors — retest outdoors first).
  //   2. Parse the returned lat/long.
  //   3. AT+CMGF=1 / AT+CMGS to send the SOS SMS with coordinates.
  // None of that is wired in yet. For now this just confirms the press.
  Serial.println(F("SOS BUTTON PRESSED -- GPS/SMS not yet implemented (placeholder)"));
  sosVibration_start();
}

void sosButton_loop() {
  bool reading = digitalRead(PIN_SOS_BUTTON);

  if (reading != sos_lastReading) {
    sos_lastChangeMs = millis();
  }

  if (millis() - sos_lastChangeMs >= SOS_DEBOUNCE_MS) {
    if (reading != sos_debouncedState) {
      sos_debouncedState = reading;
      if (sos_debouncedState == LOW) { // pressed (INPUT_PULLUP -> LOW on press)
        sosHandler();
      }
    }
  }

  sos_lastReading = reading;
}

// =======================================================================
// SECTION 7 — FIND-MY-STICK (RF + buzzer, confirmed working)
// =======================================================================
RCSwitch mySwitch = RCSwitch();

const unsigned long FMS_EXPECTED_VALUE    = 5592332UL;
const unsigned int  FMS_EXPECTED_BITS     = 24;
const unsigned int  FMS_EXPECTED_PROTOCOL = 1;

const unsigned int  FMS_REPEAT_WINDOW_MS = 400;
const unsigned int  FMS_REQUIRED_REPEATS = 2;
const unsigned long FMS_BUZZ_ON_MS       = 200;
const unsigned long FMS_BUZZ_OFF_MS      = 150;
const unsigned int  FMS_BUZZ_PULSES      = 5;
const unsigned long FMS_RELEASE_GAP_MS   = 300;

enum FmsState { FMS_IDLE, FMS_BUZZING, FMS_WAIT_RELEASE };
FmsState fms_state = FMS_IDLE;

unsigned long fms_lastReceiveTime = 0;
unsigned int  fms_consecutiveCount = 0;
unsigned long fms_lastMatchTime = 0;

unsigned long fms_buzzStateAt = 0;
unsigned int  fms_buzzPulsesDone = 0;
bool          fms_buzzPinOn = false;

void findMyStick_setup() {
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  mySwitch.enableReceive(digitalPinToInterrupt(PIN_RF_RX));
}

void findMyStick_loop() {
  if (mySwitch.available()) {
    unsigned long value    = mySwitch.getReceivedValue();
    unsigned int  bits     = mySwitch.getReceivedBitlength();
    unsigned int  protocol = mySwitch.getReceivedProtocol();
    mySwitch.resetAvailable();

    bool isMatch = (value == FMS_EXPECTED_VALUE &&
                    bits  == FMS_EXPECTED_BITS &&
                    protocol == FMS_EXPECTED_PROTOCOL);

    if (isMatch) {
      fms_lastMatchTime = millis();

      if (fms_state == FMS_IDLE) {
        unsigned long now = millis();
        if (now - fms_lastReceiveTime <= FMS_REPEAT_WINDOW_MS) {
          fms_consecutiveCount++;
        } else {
          fms_consecutiveCount = 1;
        }
        fms_lastReceiveTime = now;

        if (fms_consecutiveCount >= FMS_REQUIRED_REPEATS) {
          fms_state = FMS_BUZZING;
          fms_buzzPulsesDone = 0;
          fms_buzzPinOn = true;
          fms_buzzStateAt = now;
          digitalWrite(PIN_BUZZER, HIGH);
        }
      }
    }
  }

  if (fms_state == FMS_BUZZING) {
    unsigned long elapsed = millis() - fms_buzzStateAt;
    if (fms_buzzPinOn && elapsed >= FMS_BUZZ_ON_MS) {
      digitalWrite(PIN_BUZZER, LOW);
      fms_buzzPinOn = false;
      fms_buzzStateAt = millis();
      fms_buzzPulsesDone++;
    } else if (!fms_buzzPinOn && elapsed >= FMS_BUZZ_OFF_MS) {
      if (fms_buzzPulsesDone >= FMS_BUZZ_PULSES) {
        digitalWrite(PIN_BUZZER, LOW);
        fms_state = FMS_WAIT_RELEASE;
        fms_consecutiveCount = 0;
      } else {
        digitalWrite(PIN_BUZZER, HIGH);
        fms_buzzPinOn = true;
        fms_buzzStateAt = millis();
      }
    }
  }

  if (fms_state == FMS_WAIT_RELEASE) {
    if (millis() - fms_lastMatchTime >= FMS_RELEASE_GAP_MS) {
      fms_state = FMS_IDLE;
    }
  }
}

// =======================================================================
// SETUP / LOOP
// =======================================================================
void setup() {
  Serial.begin(9600);
  Serial.println(F("=== Smart Blind Stick -- Combined Sketch ==="));

  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  pinMode(PIN_VIBRATION, OUTPUT);
  digitalWrite(PIN_VIBRATION, LOW);
  pinMode(PIN_SOS_BUTTON, INPUT_PULLUP);

  findMyStick_setup(); // also sets up PIN_BUZZER

  Serial.println(F("GPS/SMS not yet wired in -- SOS button only prints + vibrates for now."));
}

void loop() {
  // All non-blocking -- safe to call every pass with no delay() anywhere.
  ultrasonic_loop();
  moisture_loop();
  ldr_loop();
  vibration_loop();
  sosButton_loop();
  sosVibration_loop();
  findMyStick_loop();
}