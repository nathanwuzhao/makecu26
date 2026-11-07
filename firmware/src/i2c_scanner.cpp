// i2c scanner for ESP32-C6 Supermini.
// scan 0x08-0x77 every 3 s and prints any devices that ACK.
// flash: pio run -e i2c_scanner -t upload -t monitor

#include <Arduino.h>
#include <Wire.h>

// Set these to the GPIOs your SDA/SCL wires are on.
#define I2C_SDA 20
#define I2C_SCL 19

void setup() {
  Serial.begin(115200);
  delay(1500);  // give USB CDC time to enumerate
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  Serial.printf("I2C scanner: SDA=GPIO%d SCL=GPIO%d\n", I2C_SDA, I2C_SCL);
}

void loop() {
  int found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("  found 0x%02X", addr);
      if (addr == 0x40) Serial.print("  (PCA9685 default)");
      if (addr == 0x70) Serial.print("  (PCA9685 all-call)");
      Serial.println();
      found++;
    } else if (err == 4) {
      Serial.printf("  unknown error at 0x%02X\n", addr);
    }
  }
  Serial.printf("%d device(s) found\n\n", found);
  delay(3000);
}
