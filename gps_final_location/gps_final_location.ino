/*
 * SmartStick_UnitB_Final.ino
 * ------------------------------------------------------------
 * Smart Blind Stick -- UNIT B (Arduino UNO)
 * GPS + GSM + SOS button. Sends the user's live location by SMS.
 *
 * Unit A (Arduino Nano) handles obstacle, water, dark, vibration and
 * Find-My-Stick independently. The two units share only the power
 * rail -- no data passes between them.
 *
 * HOW IT WORKS
 *   The GPS stays powered continuously so it holds a satellite fix,
 *   but is only READ in short bursts every 10 seconds. The position
 *   is cached. Pressing SOS sends the CACHED coordinates instantly --
 *   no waiting for a fresh fix, which matters when someone needs help.
 *
 *   Burst reading exists because only ONE SoftwareSerial instance can
 *   listen at a time. The port is handed back and forth: GPS by
 *   default, SIM800L during a send.
 *
 * WIRING (Arduino UNO)
 *   SIM800L EVB 5V/4V -> 5V rail, with 1000uF cap AT the EVB's pins
 *   SIM800L EVB GND   -> common ground
 *   SIM800L EVB SIM_TXD -> D3
 *   SIM800L EVB SIM_RXD -> D4       (no divider -- EVB shifts onboard)
 *
 *   NEO-7M VCC -> 5V        NEO-7M TX -> D10
 *   NEO-7M GND -> GND       NEO-7M RX -> unconnected
 *
 *   SOS button -> D6 and GND   (INPUT_PULLUP, no resistor needed)
 *
 * UPLOAD: disconnect the EVB's 5V first, close Serial Monitor,
 *         upload, then reconnect 5V.
 *
 * SERIAL COMMANDS (9600 baud, "Both NL & CR")
 *   s   simulate an SOS press (bench testing without the button)
 *   g   show current GPS status
 *   c   check the SMSC digit count
 *   anything else -> sent to the module as an AT command
 * ------------------------------------------------------------
 */

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

SoftwareSerial gpsSerial(10, 11); // D10 = RX from GPS TX
SoftwareSerial simSerial(3, 4);   // D3 = RX from SIM_TXD, D4 = TX

TinyGPSPlus gps;

// ---- Settings -------------------------------------------------------
const char PHONE_NUMBER[] = "+94762608628";   // emergency contact
const char SMSC_NUMBER[]  = "+9471000003";    // verified working
const int  SOS_PIN        = 6;
// ---------------------------------------------------------------------

// GPS burst timing
const unsigned long GPS_INTERVAL_MS = 10000; // read every 10s
const unsigned long GPS_BURST_MS    = 1500;  // listen for 1.5s
unsigned long lastGpsMs = 0;

double cachedLat = 0.0, cachedLng = 0.0;
bool   haveFix = false;
unsigned long fixAgeMs = 0;

// SOS button debounce
bool sosLast = HIGH, sosState = HIGH;
unsigned long sosChangeMs = 0;
const unsigned long SOS_DEBOUNCE_MS = 50;

bool ready = false;
uint16_t resetSeen = 0;

char inLine[72]; uint8_t inIdx = 0;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  simSerial.begin(9600);

  pinMode(SOS_PIN, INPUT_PULLUP);

  Serial.println(F("=== Smart Blind Stick -- Unit B ==="));
  Serial.print(F("Emergency contact: "));
  Serial.println(PHONE_NUMBER);
  Serial.println(F("Initialising GSM..."));

  simSerial.listen();
  initGSM();

  gpsSerial.listen();
  Serial.println(F("Ready. GPS acquiring. Press SOS or type 's'."));
  Serial.println();
}

void loop() {
  readMonitor();
  handleGpsBurst();
  handleSosButton();
}

// ---------------------------------------------------------------------
void initGSM() {
  waitFor("AT", "OK", 2000, 15);     // retry until the module answers

  // The module answers AT before it has finished booting. It emits RDY,
  // then +CPIN: READY, then Call Ready and SMS Ready over roughly ten
  // seconds -- and config commands sent during that window get ignored
  // without any error. Wait for the SIM to report READY first.
  Serial.println(F("   waiting for SIM to come ready..."));
  for (uint8_t i = 0; i < 10; i++) {
    if (atCmd("AT+CPIN?", "READY", 3000)) break;
  }

  atCmd("ATE0", "OK", 2000);
  atCmd("AT+CMEE=2", "OK", 2000);
  atCmd("AT+CMGF=1", "OK", 2000);
  atCmd("AT+CSCS=\"GSM\"", "OK", 2000);

  // CSMP sets the message's first octet, VALIDITY PERIOD, protocol id
  // and data coding scheme. This matters more than it looks: if the
  // module's stored validity period is 0, the network accepts the
  // message (returning +CMGS quite happily) and then expires it
  // immediately, so it never reaches the handset. 167 = 24 hours.
  // The last 0 is the data coding scheme -- plain GSM 7-bit.
  atCmd("AT+CSMP=49,167,0,0", "OK", 2000);

  // Route delivery reports back to us so we can tell "accepted" apart
  // from "actually delivered".
  atCmd("AT+CNMI=2,1,2,1,0", "OK", 2000);

  // Write the corrected service centre -- without this, messages are
  // accepted with an OK and then silently never delivered. Retry and
  // VERIFY, because a write that silently failed looks identical to
  // one that worked until the first message goes missing.
  char cmd[48];
  snprintf(cmd, sizeof(cmd), "AT+CSCA=\"%s\",145", SMSC_NUMBER);

  bool smscWritten = false;
  for (uint8_t i = 0; i < 3 && !smscWritten; i++) {
    smscWritten = atCmd(cmd, "OK", 4000);
    if (!smscWritten) Serial.println(F("   SMSC write failed, retrying..."));
  }
  if (!smscWritten) {
    Serial.println(F("!! COULD NOT WRITE SMSC -- messages will not deliver"));
  }

  checkSMSC();

  // Wait for network registration before declaring ready.
  for (uint8_t i = 0; i < 20; i++) {
    if (isRegistered()) { ready = true; break; }
    Serial.println(F("   waiting for network..."));
  }

  if (ready) {
    Serial.println(F("** GSM registered **"));
    atCmd("AT+CSQ", "OK", 2000);
  } else {
    Serial.println(F("!! Not registered. SOS will still try to send."));
  }
}

// ---------------------------------------------------------------------
// Take the port, read GPS for a short burst, cache the fix, release.
// Short bursts keep SoftwareSerial's interrupt load low.
// ---------------------------------------------------------------------
void handleGpsBurst() {
  if (millis() - lastGpsMs < GPS_INTERVAL_MS) return;
  lastGpsMs = millis();

  gpsSerial.listen();

  unsigned long start = millis();
  while (millis() - start < GPS_BURST_MS) {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    cachedLat = gps.location.lat();
    cachedLng = gps.location.lng();
    haveFix = true;
    fixAgeMs = millis();

    Serial.print(F("GPS "));
    Serial.print(cachedLat, 6);
    Serial.print(F(", "));
    Serial.print(cachedLng, 6);
    Serial.print(F("  sats="));
    Serial.print(gps.satellites.value());
    Serial.print(F("  hdop="));
    Serial.println(gps.hdop.hdop(), 2);
  } else {
    Serial.print(F("GPS no fix. sats="));
    Serial.print(gps.satellites.value());
    Serial.print(F("  chars="));
    Serial.println(gps.charsProcessed());
  }
}

// ---------------------------------------------------------------------
void handleSosButton() {
  bool reading = digitalRead(SOS_PIN);

  if (reading != sosLast) sosChangeMs = millis();

  if (millis() - sosChangeMs >= SOS_DEBOUNCE_MS) {
    if (reading != sosState) {
      sosState = reading;
      if (sosState == LOW) sendSOS();   // pressed
    }
  }
  sosLast = reading;
}

// ---------------------------------------------------------------------
// Send the SOS. Retries up to SEND_ATTEMPTS times, because a module
// reset mid-send is a POWER event, not a permanent failure -- the next
// attempt a few seconds later usually succeeds.
//
// The message is kept deliberately SHORT. Every character is one more
// character transmitted at 2A, and a shorter burst is a smaller demand
// on a rail that is evidently marginal. "https://" is dropped because
// phones linkify maps.google.com without it.
// ---------------------------------------------------------------------
const uint8_t SEND_ATTEMPTS = 3;

void sendSOS() {
  char latStr[12], lngStr[12], msg[80];

  Serial.println();
  Serial.println(F("=============== SOS ==============="));

  if (haveFix) {
    // snprintf on AVR cannot format %f -- dtostrf is the workaround.
    dtostrf(cachedLat, 0, 5, latStr);   // 5dp is ~1m, plenty, and shorter
    dtostrf(cachedLng, 0, 5, lngStr);
    snprintf(msg, sizeof(msg),
             "SOS! Need help. maps.google.com/?q=%s,%s", latStr, lngStr);

    Serial.print(F("Fix from "));
    Serial.print((millis() - fixAgeMs) / 1000);
    Serial.println(F("s ago"));
  } else {
    snprintf(msg, sizeof(msg), "SOS! Need help. No GPS fix.");
    Serial.println(F("!! No GPS fix"));
  }

  Serial.print(F("Msg ("));
  Serial.print(strlen(msg));
  Serial.print(F(" chars): "));
  Serial.println(msg);

  simSerial.listen();

  bool sent = false;
  for (uint8_t attempt = 1; attempt <= SEND_ATTEMPTS && !sent; attempt++) {
    Serial.print(F("-- attempt "));
    Serial.print(attempt);
    Serial.print(F(" of "));
    Serial.println(SEND_ATTEMPTS);

    sent = trySend(msg);

    if (!sent && attempt < SEND_ATTEMPTS) {
      Serial.println(F("   failed -- letting the supply recover..."));
      // Give the rail and the module time to settle before retrying.
      unsigned long w = millis();
      while (millis() - w < 5000) { }
    }
  }

  Serial.println();
  Serial.println(sent ? F(">> SMS SENT -- check the phone")
                      : F(">> ALL ATTEMPTS FAILED"));

  if (!sent) {
    Serial.println(F("   Repeated failure with corrupted output means the"));
    Serial.println(F("   5V rail is sagging under the transmit burst."));
    Serial.println(F("   Measure it at the EVB pins while sending, and"));
    Serial.println(F("   add more capacitance there."));
  }
  Serial.println(F("==================================="));
  Serial.println();

  gpsSerial.listen();
  lastGpsMs = millis();
}

// ---------------------------------------------------------------------
// One send attempt. Returns true only on a genuine +CMGS acceptance.
// ---------------------------------------------------------------------
bool trySend(const char* msg) {
  // Re-assert text mode -- a reset silently reverts to PDU mode, and
  // AT+CMGS with a quoted number then returns a bare ERROR.
  if (!atCmd("AT+CMGF=1", "OK", 3000)) {
    Serial.println(F("   text mode failed"));
    return false;
  }

  simSerial.print(F("AT+CMGS=\""));
  simSerial.print(PHONE_NUMBER);
  simSerial.println(F("\""));

  bool prompt = false, error = false;
  char buf[48]; uint8_t idx = 0; buf[0] = '\0';

  unsigned long start = millis();
  while (millis() - start < 8000 && !prompt && !error) {
    while (simSerial.available()) {
      char c = simSerial.read();
      Serial.write(c);
      if (c == '>') prompt = true;
      if (idx < sizeof(buf) - 1) { buf[idx++] = c; buf[idx] = '\0'; }
      if (strstr(buf, "ERROR")) error = true;
    }
  }

  if (!prompt) {
    Serial.println();
    Serial.println(error ? F("   ERROR -- module not in text mode (reset?)")
                         : F("   no '>' prompt"));
    return false;
  }

  // Small settle after the prompt before pushing the body.
  unsigned long w = millis();
  while (millis() - w < 200) { }

  simSerial.print(msg);
  simSerial.write(26); // Ctrl+Z

  bool accepted = false, delivered = false;
  idx = 0; buf[0] = '\0';

  // Keep listening for the FULL window even after +CMGS. The delivery
  // report (+CDS) arrives seconds to tens of seconds later, and it is
  // the only thing that distinguishes "the network took it" from "the
  // message actually reached the handset".
  start = millis();
  while (millis() - start < 40000 && !delivered) {
    while (simSerial.available()) {
      char c = simSerial.read();
      Serial.write(c);
      if (idx < sizeof(buf) - 1) { buf[idx++] = c; buf[idx] = '\0'; }
      else { memmove(buf, buf + 16, idx - 16 + 1); idx -= 16; }

      if (strstr(buf, "+CMGS:")) {
        accepted = true;
        Serial.println();
        Serial.println(F("   ACCEPTED by network (+CMGS)"));
        Serial.println(F("   ...waiting for delivery report (+CDS)"));
      }
      if (strstr(buf, "+CDS:")) {
        delivered = true;
        Serial.println();
        Serial.println(F("   *** DELIVERED -- network confirmed arrival ***"));
      }
      if (strstr(buf, "+CMS ERROR")) {
        Serial.println();
        Serial.println(F("   network rejected it -- see the error code"));
        return false;
      }
      if (strstr(buf, "RDY") || strstr(buf, "Call Ready")) {
        resetSeen++;
        Serial.println();
        Serial.println(F("   !! MODULE RESET MID-SEND -- power sagged"));
        return false;
      }
    }
  }

  Serial.println();
  return accepted;
}

// ---------------------------------------------------------------------
void checkSMSC() {
  char buf[96]; uint8_t idx = 0; buf[0] = '\0';

  simSerial.println(F("AT+CSCA?"));
  unsigned long start = millis();
  while (millis() - start < 3000) {
    while (simSerial.available()) {
      char c = simSerial.read();
      Serial.write(c);
      if (idx < sizeof(buf) - 1) { buf[idx++] = c; buf[idx] = '\0'; }
    }
  }
  Serial.println();

  char* q1 = strchr(buf, '"');
  char* q2 = q1 ? strchr(q1 + 1, '"') : NULL;
  if (!q1 || !q2) return;

  uint8_t digits = 0;
  for (char* p = q1 + 1; p < q2; p++) if (*p >= '0' && *p <= '9') digits++;

  Serial.print(F("SMSC digits: "));
  Serial.print(digits);
  Serial.println(digits == 10 ? F("  OK") : F("  WRONG -- expected 10"));
}

// ---------------------------------------------------------------------
bool isRegistered() {
  char buf[80]; uint8_t idx = 0; buf[0] = '\0';

  simSerial.println(F("AT+CREG?"));
  unsigned long start = millis();
  while (millis() - start < 3000) {
    while (simSerial.available()) {
      char c = simSerial.read();
      if (idx < sizeof(buf) - 1) { buf[idx++] = c; buf[idx] = '\0'; }
    }
  }
  return (strstr(buf, "+CREG: 0,1") || strstr(buf, "+CREG: 0,5"));
}

// ---------------------------------------------------------------------
bool atCmd(const char* cmd, const char* expect, unsigned long timeoutMs) {
  char buf[80]; uint8_t idx = 0; buf[0] = '\0';

  Serial.print(F(">> "));
  Serial.println(cmd);
  simSerial.println(cmd);

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (simSerial.available()) {
      char c = simSerial.read();
      Serial.write(c);
      if (idx < sizeof(buf) - 1) { buf[idx++] = c; buf[idx] = '\0'; }
    }
    if (strstr(buf, expect)) { Serial.println(); return true; }
  }
  Serial.println();
  return false;
}

bool waitFor(const char* cmd, const char* expect, unsigned long t, uint8_t tries) {
  for (uint8_t i = 0; i < tries; i++) {
    if (atCmd(cmd, expect, t)) return true;
    Serial.println(F("   retrying..."));
  }
  return false;
}

// ---------------------------------------------------------------------
void readMonitor() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (inIdx > 0) { inLine[inIdx] = '\0'; handleLine(inLine); inIdx = 0; }
    } else if (inIdx < sizeof(inLine) - 1) {
      inLine[inIdx++] = c;
    }
  }
}

void handleLine(const char* line) {
  if (strcmp(line, "s") == 0) { sendSOS(); return; }

  if (strcmp(line, "g") == 0) {
    Serial.print(F("GPS fix: "));
    if (haveFix) {
      Serial.print(cachedLat, 6); Serial.print(F(", ")); Serial.print(cachedLng, 6);
      Serial.print(F("  age "));
      Serial.print((millis() - fixAgeMs) / 1000);
      Serial.println(F("s"));
    } else {
      Serial.println(F("none yet"));
    }
    return;
  }

  if (strcmp(line, "c") == 0) {
    simSerial.listen();
    checkSMSC();
    gpsSerial.listen();
    return;
  }

  simSerial.listen();
  Serial.print(F(">> "));
  Serial.println(line);
  simSerial.println(line);
  unsigned long start = millis();
  while (millis() - start < 3000) {
    while (simSerial.available()) Serial.write(simSerial.read());
  }
  Serial.println();
  gpsSerial.listen();
}