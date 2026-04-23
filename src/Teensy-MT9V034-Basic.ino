/**
 * Realtime Embedded Vision Pipeline
 * Teensy 4.x + MT9V034 — Basic CPU-Polled Capture
 * Version: 1.0
 *
 * Baseline CPU-polled 8-bit parallel capture — no FlexIO,
 * no DMA. Pixel acquisition is handled entirely in software
 * by polling PCLK, HREF, and VSYNC via digitalRead().
 *
 * Performance: ~18 FPS at 640x480 (CPU-bound).
 * Compare against FlexIO+DMA variant to quantify
 * the hardware offload advantage (~3.3x throughput gain).
 *
 * This file exists to document the baseline and justify
 * the FlexIO+DMA architecture choice.
 *
 * CONFIGURATION:
 *   Set dataPins[8], VSYNC, HREF, PCLK to match your wiring.
 *   Set FRAME_WIDTH and FRAME_HEIGHT to your target resolution.
 *
 * Author:   Humayun Khan
 * Role:     Embedded Vision Consultant
 * Contact:  humayunnaveedkhan@gmail.com
 * LinkedIn: linkedin.com/in/humayunnaveedkhan
 *
 * © 2025 Humayun Khan. All rights reserved.
 */

#include <Wire.h>

// ============================================================
// PIN MAP — adjust to your board wiring
// ============================================================
const uint8_t dataPins[8] = {17, 16, 15, 14, 19, 18, 21, 20}; // D0–D7
const uint8_t VSYNC = 2;
const uint8_t HREF  = 3;
const uint8_t PCLK  = 4;

// ============================================================
// FRAME GEOMETRY
// ============================================================
#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480

// Frame buffer — stored in SRAM
uint8_t frame[FRAME_HEIGHT][FRAME_WIDTH];

// ============================================================
// READ ONE PIXEL via digital polling (CPU-bound)
// ============================================================
uint8_t readPixel() {
  uint8_t val = 0;
  for (int i = 0; i < 8; i++) {
    val |= (digitalRead(dataPins[i]) << i);
  }
  return val;
}

// ============================================================
// WAIT FOR VSYNC (frame boundary sync)
// ============================================================
void waitForFrameStart() {
  while (digitalRead(VSYNC));   // wait for LOW (end of prev frame)
  while (!digitalRead(VSYNC));  // wait for HIGH (start of new frame)
}

// ============================================================
// CAPTURE ONE FULL FRAME
// All pixel reads are CPU-polled — no DMA, no FlexIO.
// This is the bottleneck at ~18 FPS @ 640x480.
// ============================================================
void captureFrame() {
  waitForFrameStart();

  for (int row = 0; row < FRAME_HEIGHT; row++) {
    // Wait for HREF HIGH (start of valid line)
    while (!digitalRead(HREF));

    int col = 0;
    while (digitalRead(HREF) && col < FRAME_WIDTH) {
      while (!digitalRead(PCLK));      // wait for rising edge
      frame[row][col++] = readPixel();
      while (digitalRead(PCLK));       // wait for falling edge
    }

    while (digitalRead(HREF));         // wait for end of line

    // Pad if line was shorter than expected
    while (col < FRAME_WIDTH) {
      frame[row][col++] = 0;
    }
  }
}

// ============================================================
// SEND FRAME OVER USB SERIAL
// Simple header + raw payload (no width/height encoding)
// ============================================================
void sendFrame() {
  // Frame marker
  Serial.write(0xFF);
  Serial.write(0xAA);

  for (int row = 0; row < FRAME_HEIGHT; row++) {
    Serial.write(frame[row], FRAME_WIDTH);
  }
}

// ============================================================
// ARDUINO SETUP & LOOP
// ============================================================
void setup() {
  for (int i = 0; i < 8; i++) pinMode(dataPins[i], INPUT);
  pinMode(VSYNC, INPUT);
  pinMode(HREF,  INPUT);
  pinMode(PCLK,  INPUT);

  Serial.begin(6000000);
  delay(200);
  Serial.println("MT9V034 Basic Capture Ready.");
  Serial.println("Note: CPU-polled mode. ~18 FPS at 640x480.");
  Serial.println("See FlexIO+DMA variant for ~60 FPS.");
}

void loop() {
  captureFrame();
  sendFrame();
}
