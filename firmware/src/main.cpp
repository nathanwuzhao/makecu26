#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  Serial0.begin(115200);
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial0.println("test");
  delay(100);
}
