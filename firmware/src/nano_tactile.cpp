// Arduino Nano: scans the FlexiTac 16 x 32 tactile matrix and sends each frame to the ESP32
// over a one-way SoftwareSerial UART. See docs/tactile-uart-link.md.
// Scan logic follows FlexiTac 32_16_fast.ino (https://github.com/FlexiTac/FlexiTac, CC BY-NC 4.0):
// same pins, same mux/shift-register sequence, same ADC prescaler, same 8-bit (>> 2) readings.
// Frame format: include/tactile_frame.h
// Flash: pio run -e nano -t upload
//
// Pins:
//   FlexiTac (unchanged): A0 = ADC in, D2 = shift-register data, D3 = shift-register clock,
//                         D4-D7 = mux channel select, D8 = mux inhibit (low = enabled), D9 = unused mux inhibit
//   Link to ESP32:        D11 / PB3 / ICSP 4 = TX -> BSS138 level shifter -> ESP32-C6 GPIO17 (RX)
//                         D12 / PB4 / ICSP 1 = SoftwareSerial RX placeholder, unused (listening is off)
//                         GND shared (ICSP 6)
//
// Note: SoftwareSerial defines every pin-change ISR itself, so this file must not define any.

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "tactile_frame.h"

#define PIN_ADC_INPUT A0
#define PIN_SHIFT_REGISTER_DATA 2
#define PIN_SHIFT_REGISTER_CLOCK 3
#define PIN_MUX_CHANNEL_0 4  // D4-D7 = PD4-PD7
#define PIN_MUX_INHIBIT_0 8
#define PIN_MUX_INHIBIT_1 9

#define PIN_LINK_RX 12  // unused, SoftwareSerial just needs a pin
#define PIN_LINK_TX 11

#define SET_SR_DATA_HIGH() PORTD |= B00000100
#define SET_SR_DATA_LOW() PORTD &= ~B00000100
#define SET_SR_CLK_HIGH() PORTD |= B00001000
#define SET_SR_CLK_LOW() PORTD &= ~B00001000

SoftwareSerial link(PIN_LINK_RX, PIN_LINK_TX);

uint8_t frame[TACTILE_COUNT];
uint16_t frameSum;

static void setRow(uint8_t row) {
  PORTD = (PORTD & 0x0F) | ((row & 0x0F) << 4);  // D4-D7, leaves D0-D3 alone
}

static void shiftColumn(bool isFirst) {
  if (isFirst) SET_SR_DATA_HIGH();
  SET_SR_CLK_HIGH();
  SET_SR_CLK_LOW();
  if (isFirst) SET_SR_DATA_LOW();
}

// Fills frame[] and frameSum (16-bit sum of the data bytes only).
static void scanFrame() {
  uint16_t sum = 0;
  uint16_t k = 0;
  for (uint8_t row = 0; row < TACTILE_ROWS; row++) {
    setRow(row);
    shiftColumn(true);
    shiftColumn(false);
    for (uint8_t col = 0; col < TACTILE_COLS; col++) {
      uint8_t v = analogRead(PIN_ADC_INPUT) >> 2;
      frame[k++] = v;
      sum += v;
      shiftColumn(false);
    }
  }
  frameSum = sum;
}

void setup() {
  pinMode(PIN_ADC_INPUT, INPUT);
  pinMode(PIN_SHIFT_REGISTER_DATA, OUTPUT);
  pinMode(PIN_SHIFT_REGISTER_CLOCK, OUTPUT);
  for (uint8_t i = 0; i < 4; i++) pinMode(PIN_MUX_CHANNEL_0 + i, OUTPUT);
  pinMode(PIN_MUX_INHIBIT_0, OUTPUT);
  pinMode(PIN_MUX_INHIBIT_1, OUTPUT);
  digitalWrite(PIN_MUX_INHIBIT_0, LOW);  // single mux, always enabled (as in FlexiTac with MUX_COUNT 1)
  digitalWrite(PIN_MUX_INHIBIT_1, LOW);

  // ADC prescaler CLK/16, same as FlexiTac (fast; fine for 8-bit readings)
  ADCSRA = (ADCSRA & ~(_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0))) | _BV(ADPS2);

  link.begin(TACTILE_BAUD);
  link.stopListening();  // TX only: turn off the RX pin-change interrupt
}

void loop() {
  scanFrame();
  link.write(TACTILE_HDR0);
  link.write(TACTILE_HDR1);
  link.write(frame, TACTILE_COUNT);
  link.write(frameSum & 0xFF);
  link.write(frameSum >> 8);
}
