// servo controller: esp32c6 supermini -> pca9685 (i2c 0x40) on sda=gpio20, scl=gpio19.
// tactile data: arduino nano reading a FlexiTac 16x32 matrix, sent one-way over UART into
// GPIO17 (~19 Hz; frame format in include/tactile_frame.h, design in docs/tactile-uart-link.md).
// current: two INA219s (0x40, 0x45) on a second i2c bus (Wire1 = C6 LP I2C, fixed pins sda=gpio6, scl=gpio7).
//
// Serial: 115200, newline-terminated lines, '\r' ignored
//   Host -> MCU
//     S <idx> <angle> [<idx> <angle> ...]   set one or more servos (idx 0-6, angle 0-180)
//                                           e.g. "S 0 90 3 45 6 120"
//     C                                     report calibration for all servos
//     C <idx>                               report calibration for one servo
//     C <idx> <min_us> <max_us>             set pulse width at 0 / 180 deg (400-2600, min < max);
//                                           servo is re-driven at its current angle and the
//                                           values are saved to flash (survive reboot)
//     R                                     report reverse flag for all servos
//     R <idx>                               report reverse flag for one servo
//     R <idx> <0|1>                         set reverse flag (1 = angle a drives the pulse for 180-a),
//                                           saved to flash. If it changes, that servo goes limp
//                                           (target -1) until its next S, so it never jumps.
//   MCU -> Host
//     D <ok> <a0>..<a6> <i0> <i1> <hex>     streamed at STREAM_HZ:
//                                           ok  = 1 if a valid tactile frame arrived in the last
//                                                 TACTILE_STALE_MS, 0 if the link is stale/dead
//                                                 (hex = last good frame, all zeros before any)
//                                           a   = servo target angles, -1 = limp (not driven)
//                                           i0  = current (mA, 1 decimal) from INA219 @0x40, i1 from @0x45;
//                                                 "nan" if that sensor wasn't found at boot
//                                           hex = 512 tactile bytes (0-255) as 1024 hex chars, row-major
//     CAL <idx> <min_us> <max_us>           calibration report (reply to C)
//     REV <idx> <0|1>                       reverse flag report (reply to R)
//     ERR <reason>                          bad command; nothing in that command is applied
//
// servos whose requested angle equals their current target are skipped (no I2C write).
// S has no reply; the next D line shows the new targets.
//
// SAFETY: every servo is LIMP at boot (all PCA9685 outputs off, targets -1). Nothing moves until
// the host sends S for that servo. Booting / flashing never drives a servo. (2026-11-08: a servo
// without end stops burned out stalling at the old fixed boot pose.)

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <Preferences.h>
#include "tactile_frame.h"

#define I2C_SDA 20
#define I2C_SCL 19
#define PCA_ADDR 0x40

// One-way UART from the Nano (RX only). GPIO17 = "RX" on the Supermini silkscreen, UART0.
#define TACTILE_RX_PIN 17
HardwareSerial &tactileUart = Serial0;
const uint32_t TACTILE_STALE_MS = 200;  // ok = 0 if no valid frame for this long

// INA219 current sensors (Adafruit library defaults: 0.1 ohm shunt, 32V / 2A range, 0.1 mA steps)
// On Wire1: the C6 LP I2C controller, whose pins are fixed in hardware (SDA=GPIO6, SCL=GPIO7).
#define INA_SDA 6
#define INA_SCL 7
const uint8_t INA_ADDR[2] = {0x40, 0x45};  // no bridge = 0x40 (separate bus from the PCA9685), A0+A1 bridged = 0x45

// INA219 config. 0x399F = library default (1 shunt sample, 532 us). 64-sample averaging (0x39F7)
// was tried 2026-11-08 and gave 1.4-2.9 A readings during moves, but that test ran with the
// servos not on their proper supply, so it's inconclusive. See docs/firmware-notes.md before
// changing this.
const uint16_t INA_CONFIG = 0x399F;
const float INA_SHUNT_OHMS = 0.1f;   // standard INA219 breakout shunt
const uint32_t INA_CHECK_MS = 1000;  // re-read config this often; rewrite if the chip reset

// Servo PWM frame rate. Standard 50 Hz. (58.85 Hz would make a 64-sample INA219 window cover
// exactly 2 frames; only worth it if averaging is re-enabled. Changing this changes holding
// current readings, so recalibrate.)
const float SERVO_PWM_HZ = 50.0f;

const uint32_t STREAM_HZ = 25;          // tactile arrives at ~19 Hz; D lines repeat the latest frame
const uint32_t STREAM_PERIOD_MS = 1000 / STREAM_HZ;

constexpr uint8_t NUM_SERVOS = 7;

// pca9685 channel each servo index is wired to (see docs/hardware-wiring.md):
//   0 = ch0 bottom squeeze (i0)   1 = ch4 bottom finger R   2 = ch8 halves up/down
//   3 = ch12 bottom finger L      4 = ch3 top finger R      5 = ch7 top squeeze (i1)
//   6 = ch15 top finger L
const uint8_t SERVO_CHANNEL[NUM_SERVOS] = {0, 4, 8, 12, 3, 7, 15};

// default pulse width (us) at 0 and 180 degrees; overridden by values saved with C
const uint16_t DEFAULT_MIN_US = 500;
const uint16_t DEFAULT_MAX_US = 2400;
const uint16_t CAL_LIMIT_LO = 400;   // sanity bounds for C, protects servos from typos
const uint16_t CAL_LIMIT_HI = 2600;

const int16_t LIMP = -1;  // targetAngle value for "not driven"

Adafruit_PWMServoDriver pwm(PCA_ADDR, Wire);
Preferences prefs;
int16_t targetAngle[NUM_SERVOS];  // 0-180, or LIMP
uint16_t minUs[NUM_SERVOS];
uint16_t maxUs[NUM_SERVOS];
uint8_t reversed[NUM_SERVOS];     // 1 = mounted the other way: angle a -> pulse for 180 - a

void loadCalibration() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    minUs[i] = DEFAULT_MIN_US;
    maxUs[i] = DEFAULT_MAX_US;
  }
  prefs.begin("servo", true);
  // only use saved data if it matches the current array size
  if (prefs.getBytesLength("min") == sizeof(minUs) && prefs.getBytesLength("max") == sizeof(maxUs)) {
    prefs.getBytes("min", minUs, sizeof(minUs));
    prefs.getBytes("max", maxUs, sizeof(maxUs));
  }
  memset(reversed, 0, sizeof(reversed));
  if (prefs.getBytesLength("rev") == sizeof(reversed)) prefs.getBytes("rev", reversed, sizeof(reversed));
  prefs.end();
}

void saveCalibration() {
  prefs.begin("servo", false);
  prefs.putBytes("min", minUs, sizeof(minUs));
  prefs.putBytes("max", maxUs, sizeof(maxUs));
  prefs.putBytes("rev", reversed, sizeof(reversed));
  prefs.end();
}

void writeServo(uint8_t idx, uint8_t angle) {
  uint8_t a = reversed[idx] ? 180 - angle : angle;
  uint16_t us = map(a, 0, 180, minUs[idx], maxUs[idx]);
  pwm.writeMicroseconds(SERVO_CHANNEL[idx], us);
}

// Output fully off: no pulses, so the servo is limp and draws ~nothing.
void pcaChannelOff(uint8_t ch) {
  pwm.setPWM(ch, 0, 4096);  // 4096 = "full off" bit
}

void servoOff(uint8_t idx) {
  pcaChannelOff(SERVO_CHANNEL[idx]);
  targetAngle[idx] = LIMP;
}

Adafruit_INA219 ina[2] = {Adafruit_INA219(INA_ADDR[0]), Adafruit_INA219(INA_ADDR[1])};
bool inaOk[2];

// Raw config register access. The library has no setter for averaging, and begin() ->
// setCalibration_32V_2A() writes 0x399F, so this must run after begin().
bool inaWriteConfig(uint8_t addr, uint16_t cfg) {
  Wire1.beginTransmission(addr);
  Wire1.write(0x00);
  Wire1.write(cfg >> 8);
  Wire1.write(cfg & 0xFF);
  return Wire1.endTransmission() == 0;
}

// Returns -1 on I2C error.
int32_t inaReadConfig(uint8_t addr) {
  Wire1.beginTransmission(addr);
  Wire1.write(0x00);
  if (Wire1.endTransmission() != 0) return -1;
  if (Wire1.requestFrom(addr, (uint8_t)2) != 2) return -1;
  uint16_t hi = Wire1.read();
  return (hi << 8) | Wire1.read();
}

// Current from the shunt voltage register: I = V / R. Same 0.1 mA resolution as the library's
// getCurrent_mA() (10 uV LSB / 0.1 ohm), but it doesn't touch the calibration register.
// (getCurrent_mA() rewrites the calibration register before every read; this skips that.)
float inaCurrent_mA(uint8_t i) {
  return ina[i].getShuntVoltage_mV() / INA_SHUNT_OHMS;
}

// Called once a second: if a sensor's config isn't INA_CONFIG (e.g. a power glitch reset it),
// write it back. Costs one 2-byte read per sensor when nothing is wrong.
void inaMaintain() {
  for (uint8_t i = 0; i < 2; i++) {
    if (!inaOk[i]) continue;
    int32_t cfg = inaReadConfig(INA_ADDR[i]);
    if (cfg >= 0 && cfg != INA_CONFIG) inaWriteConfig(INA_ADDR[i], INA_CONFIG);
  }
}

uint8_t tactile[TACTILE_COUNT];  // last good frame from the Nano
uint32_t lastTactileMs = 0;      // millis() of last good frame, 0 = never

// Call every loop() iteration. Never blocks. Finds "A5 5A", collects data + checksum,
// and copies into tactile[] only if the 16-bit sum matches.
void pollTactile() {
  static uint8_t buf[TACTILE_COUNT + 2];  // data + checksum
  static uint16_t pos = 0;
  static uint8_t state = 0;               // 0 = want A5, 1 = want 5A, 2 = reading body

  while (tactileUart.available()) {
    uint8_t b = tactileUart.read();
    if (state == 0) {
      if (b == TACTILE_HDR0) state = 1;
    } else if (state == 1) {
      if (b == TACTILE_HDR1) {
        state = 2;
        pos = 0;
      } else {
        state = (b == TACTILE_HDR0) ? 1 : 0;
      }
    } else {
      buf[pos++] = b;
      if (pos == sizeof(buf)) {
        uint16_t sum = 0;
        for (uint16_t i = 0; i < TACTILE_COUNT; i++) sum += buf[i];
        uint16_t got = buf[TACTILE_COUNT] | (buf[TACTILE_COUNT + 1] << 8);
        if (sum == got) {
          memcpy(tactile, buf, TACTILE_COUNT);
          lastTactileMs = millis();
          if (lastTactileMs == 0) lastTactileMs = 1;  // 0 is reserved for "never"
        }
        state = 0;  // good or bad, go back to looking for a header
      }
    }
  }
}

bool tactileFresh() {
  return lastTactileMs != 0 && millis() - lastTactileMs < TACTILE_STALE_MS;
}

// Sends one D line. Drops the whole line (never half of one) if the USB buffer is full
// or no host is connected, so the stream can't stall the servo loop.
void sendData(bool ok) {
  static const char HEX_DIGITS[] = "0123456789abcdef";
  static char buf[64 + 2 * TACTILE_COUNT + 2];
  int n = snprintf(buf, sizeof(buf), "D %d", ok ? 1 : 0);
  for (uint8_t i = 0; i < NUM_SERVOS; i++) n += snprintf(buf + n, sizeof(buf) - n, " %d", targetAngle[i]);
  for (uint8_t i = 0; i < 2; i++) {
    if (inaOk[i]) n += snprintf(buf + n, sizeof(buf) - n, " %.1f", inaCurrent_mA(i));
    else n += snprintf(buf + n, sizeof(buf) - n, " nan");
  }
  buf[n++] = ' ';
  for (uint16_t i = 0; i < TACTILE_COUNT; i++) {
    buf[n++] = HEX_DIGITS[tactile[i] >> 4];
    buf[n++] = HEX_DIGITS[tactile[i] & 0x0F];
  }
  buf[n++] = '\n';
  if (Serial.isConnected() && Serial.availableForWrite() >= n) Serial.write(buf, n);
}

// parses integer token; return false if missing, NaN, or out of range
bool parseInt(char *tok, long lo, long hi, long &out) {
  if (!tok) return false;
  char *end;
  out = strtol(tok, &end, 10);
  return *end == '\0' && end != tok && out >= lo && out <= hi;
}

void handleSet(char *args) {
  // validate the whole command first so a bad pair doesn't leave half-applied pose
  int16_t pending[NUM_SERVOS];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) pending[i] = -1;

  int pairs = 0;
  char *tok = strtok(args, " ");
  while (tok) {
    long idx, angle;
    if (!parseInt(tok, 0, NUM_SERVOS - 1, idx)) {
      Serial.printf("ERR bad index '%s'\n", tok);
      return;
    }
    char *angTok = strtok(nullptr, " ");
    if (!parseInt(angTok, 0, 180, angle)) {
      Serial.printf("ERR bad angle for servo %ld\n", idx);
      return;
    }
    pending[idx] = angle;  // repeated index in one command: last one wins
    pairs++;
    tok = strtok(nullptr, " ");
  }
  if (pairs == 0) {
    Serial.println("ERR S needs <idx> <angle> pairs");
    return;
  }

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (pending[i] < 0 || pending[i] == targetAngle[i]) continue;
    targetAngle[i] = pending[i];
    writeServo(i, targetAngle[i]);
  }
}

void sendCal(uint8_t idx) {
  Serial.printf("CAL %u %u %u\n", idx, minUs[idx], maxUs[idx]);
}

void handleCal(char *args) {
  char *idxTok = strtok(args, " ");
  if (!idxTok) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) sendCal(i);
    return;
  }
  long idx, lo, hi;
  if (!parseInt(idxTok, 0, NUM_SERVOS - 1, idx)) {
    Serial.printf("ERR bad index '%s'\n", idxTok);
    return;
  }
  char *loTok = strtok(nullptr, " ");
  if (!loTok) {
    sendCal(idx);
    return;
  }
  char *hiTok = strtok(nullptr, " ");
  if (!parseInt(loTok, CAL_LIMIT_LO, CAL_LIMIT_HI, lo) ||
      !parseInt(hiTok, CAL_LIMIT_LO, CAL_LIMIT_HI, hi) || lo >= hi ||
      strtok(nullptr, " ")) {
    Serial.printf("ERR C needs <idx> <min_us> <max_us>, %u-%u, min < max\n", CAL_LIMIT_LO, CAL_LIMIT_HI);
    return;
  }
  if (lo != minUs[idx] || hi != maxUs[idx]) {  // skip flash write + servo update if unchanged
    minUs[idx] = lo;
    maxUs[idx] = hi;
    saveCalibration();
    if (targetAngle[idx] != LIMP) writeServo(idx, targetAngle[idx]);
  }
  sendCal(idx);
}

void sendRev(uint8_t idx) {
  Serial.printf("REV %u %u\n", idx, reversed[idx]);
}

void handleRev(char *args) {
  char *idxTok = strtok(args, " ");
  if (!idxTok) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) sendRev(i);
    return;
  }
  long idx, val;
  if (!parseInt(idxTok, 0, NUM_SERVOS - 1, idx)) {
    Serial.printf("ERR bad index '%s'\n", idxTok);
    return;
  }
  char *valTok = strtok(nullptr, " ");
  if (!valTok) {
    sendRev(idx);
    return;
  }
  if (!parseInt(valTok, 0, 1, val) || strtok(nullptr, " ")) {
    Serial.println("ERR R needs <idx> <0|1>");
    return;
  }
  if (val != reversed[idx]) {
    reversed[idx] = val;
    saveCalibration();
    // The same target angle now means the mirrored position. Don't jump there: go limp and
    // wait for the host to send a fresh S for this servo.
    servoOff(idx);
  }
  sendRev(idx);
}

void handleLine(char *line) {
  while (*line == ' ') line++;
  if (*line == '\0') return;

  char cmd = toupper(line[0]);
  char *args = line + 1;
  if (cmd == 'S' && (*args == ' ' || *args == '\0')) {
    handleSet(args);
  } else if (cmd == 'C' && (*args == ' ' || *args == '\0')) {
    handleCal(args);
  } else if (cmd == 'R' && (*args == ' ' || *args == '\0')) {
    handleRev(args);
  } else {
    Serial.printf("ERR unknown command '%s'\n", line);
  }
}

void pollSerial() {
  static char line[128];
  static size_t len = 0;
  static bool overflow = false;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (overflow) {
        Serial.println("ERR line too long");
      } else {
        line[len] = '\0';
        handleLine(line);
      }
      len = 0;
      overflow = false;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    } else {
      overflow = true;
    }
  }
}

void setup() {
  Serial.setTxBufferSize(4096);  // a D line is ~1.1 KB
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // never block on USB writes
  loadCalibration();

  // ~11.5 bytes/ms arrive; a 40 ms tick plus INA/I2C time can collect ~500 bytes, so the
  // default 256-byte buffer would overflow. Must be set before begin().
  tactileUart.setRxBufferSize(2048);
  tactileUart.begin(TACTILE_BAUD, SERIAL_8N1, TACTILE_RX_PIN, -1);  // -1 = no TX

  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_PWM_HZ);
  // The PCA9685 keeps its outputs across an ESP32 reset (pwm.begin() doesn't clear them), so
  // explicitly switch every channel off, including the ones no servo is mapped to.
  for (uint8_t ch = 0; ch < 16; ch++) pcaChannelOff(ch);
  for (uint8_t i = 0; i < NUM_SERVOS; i++) targetAngle[i] = LIMP;
  delay(10);

  Wire1.begin(INA_SDA, INA_SCL, 100000);
  for (uint8_t i = 0; i < 2; i++) {
    inaOk[i] = ina[i].begin(&Wire1);
    if (inaOk[i]) inaWriteConfig(INA_ADDR[i], INA_CONFIG);
  }
}

void loop() {
  pollSerial();
  pollTactile();

  static uint32_t lastTick = 0;
  uint32_t now = millis();
  if (now - lastTick >= STREAM_PERIOD_MS) {
    lastTick += STREAM_PERIOD_MS;
    if (now - lastTick >= STREAM_PERIOD_MS) lastTick = now;  // fell behind: don't burst to catch up
    sendData(tactileFresh());
  }

  static uint32_t lastInaCheck = 0;
  if (now - lastInaCheck >= INA_CHECK_MS) {
    lastInaCheck = now;
    inaMaintain();
  }
}
