// servo controller: esp32c6 supermini -> pca9685 (i2c 0x40) on sda=gpio20, scl=gpio19.
//
// Serial: 115200, newline-terminated lines, '\r' ignored
//   Host -> MCU
//     S <idx> <angle> [<idx> <angle> ...]   set one or more servos (idx 0-6, angle 0-180)
//                                           e.g. "S 0 90 3 45 6 120"
//     G                                     request current targets
//     C                                     report calibration for all servos
//     C <idx>                               report calibration for one servo
//     C <idx> <min_us> <max_us>             set pulse width at 0 / 180 deg (400-2600, min < max);
//                                           servo is re-driven at its current angle and the
//                                           values are saved to flash (survive reboot)
//   MCU -> Host
//     T <a0> <a1> <a2> <a3> <a4> <a5> <a6>  current targets; sent at boot, after any S that
//                                           actually changed something, and in reply to G
//     CAL <idx> <min_us> <max_us>           calibration report (reply to C)
//     ERR <reason>                          bad command; nothing in that command is applied
//
// servos whose requested angle equals their current target are skipped (no I2C write),
// S command that changes nothing produces no output at all

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

#define I2C_SDA 20
#define I2C_SCL 19
#define PCA_ADDR 0x40

constexpr uint8_t NUM_SERVOS = 7;

// pca9685 channel each servo index is wired to
const uint8_t SERVO_CHANNEL[NUM_SERVOS] = {0, 1, 2, 3, 4, 5, 6};

// default pulse width (us) at 0 and 180 degrees; overridden by values saved with C
const uint16_t DEFAULT_MIN_US = 500;
const uint16_t DEFAULT_MAX_US = 2400;
const uint16_t CAL_LIMIT_LO = 400;   // sanity bounds for C, protects servos from typos
const uint16_t CAL_LIMIT_HI = 2600;

// pose written at boot
const uint8_t INITIAL_ANGLE[NUM_SERVOS] = {0, 0, 0, 0, 0, 0, 0};

Adafruit_PWMServoDriver pwm(PCA_ADDR, Wire);
Preferences prefs;
uint8_t targetAngle[NUM_SERVOS];
uint16_t minUs[NUM_SERVOS];
uint16_t maxUs[NUM_SERVOS];

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
  prefs.end();
}

void saveCalibration() {
  prefs.begin("servo", false);
  prefs.putBytes("min", minUs, sizeof(minUs));
  prefs.putBytes("max", maxUs, sizeof(maxUs));
  prefs.end();
}

void writeServo(uint8_t idx, uint8_t angle) {
  uint16_t us = map(angle, 0, 180, minUs[idx], maxUs[idx]);
  pwm.writeMicroseconds(SERVO_CHANNEL[idx], us);
}

void sendTargets() {
  char buf[48];
  int n = snprintf(buf, sizeof(buf), "T");
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    n += snprintf(buf + n, sizeof(buf) - n, " %u", targetAngle[i]);
  }
  Serial.println(buf);
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

  bool changed = false;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (pending[i] < 0 || pending[i] == targetAngle[i]) continue;
    targetAngle[i] = pending[i];
    writeServo(i, targetAngle[i]);
    changed = true;
  }
  if (changed) sendTargets();
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
    writeServo(idx, targetAngle[idx]);
  }
  sendCal(idx);
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
  } else if (cmd == 'G' && *args == '\0') {
    sendTargets();
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
  Serial.begin(115200);
  loadCalibration();

  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    targetAngle[i] = INITIAL_ANGLE[i];
    writeServo(i, targetAngle[i]);
  }

  delay(1500);  // let USB CDC enumerate so the boot report isn't lost
  sendTargets();
}

void loop() {
  pollSerial();
}
