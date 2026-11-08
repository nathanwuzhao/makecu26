// Shared UART frame format: Nano (TX only) -> ESP32-C6 (RX only). See docs/tactile-uart-link.md.
// Sensor: FlexiTac 16 x 32 tactile matrix, 8-bit readings (ADC >> 2, same as FlexiTac 32_16_fast.ino).
//
// offset  size  content
// 0       1     TACTILE_HDR0 (0xA5)
// 1       1     TACTILE_HDR1 (0x5A)
// 2       512   readings, row-major (row 0 cols 0-31, row 1 cols 0-31, ...), 0-255
// 514     2     checksum: 16-bit sum of the 512 data bytes, little-endian
// total   516 bytes, 115200 8N1
#pragma once
#include <stdint.h>

#define TACTILE_ROWS 16
#define TACTILE_COLS 32
#define TACTILE_COUNT (TACTILE_ROWS * TACTILE_COLS)
#define TACTILE_HDR0 0xA5
#define TACTILE_HDR1 0x5A
#define TACTILE_FRAME_LEN (2 + TACTILE_COUNT + 2)
#define TACTILE_BAUD 115200
