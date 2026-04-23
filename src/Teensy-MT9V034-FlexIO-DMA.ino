/**
 * Realtime Embedded Vision Pipeline
 * Teensy 4.x + MT9V034 — FlexIO2 + DMA
 * Version: 1.0
 *
 * Deterministic 8-bit parallel grayscale capture from
 * MT9V034 global-shutter sensor using FlexIO2 SHIFTIN
 * + DMA ping-pong line buffers. CPU load during capture
 * is near zero — pixel acquisition is fully hardware-driven.
 *
 * Resolutions validated: 100x80 through 720x520 @ ~60 FPS
 * USB throughput is the primary bottleneck at higher res.
 *
 * Frame protocol:
 *   [FF AA FF AA][uint16_le W][uint16_le H][grayscale payload]
 *
 * Build environment:
 *   Teensyduino 1.59 + KurtE FlexIO_t4 library
 *
 * CONFIGURATION:
 *   Set PIN_D[8], PIN_PCLK, PIN_HREF, PIN_VSYNC to match
 *   your board wiring. All 8 data pins must map to consecutive
 *   FlexIO2 indices — verify with pFlex->mapIOPinToFlexPin().
 *   Set CAM_W and CAM_H to your target resolution.
 *
 * Author:   Humayun Khan
 * Role:     Embedded Vision Consultant
 * Contact:  humayunnaveedkhan@gmail.com
 * LinkedIn: linkedin.com/in/humayunnaveedkhan
 *
 * © 2025 Humayun Khan. All rights reserved.
 */

#include <Arduino.h>
#include <Wire.h>
#include <DMAChannel.h>
#include <FlexIO_t4.h>

#include "mt9v034_registermap.h"

// ============================================================
// PIN MAP — adjust to your board wiring
// All 8 data pins must be consecutive in FlexIO2 pin space.
// ============================================================
static constexpr uint8_t PIN_D[8]  = {10, 12, 11, 13, 8, 7, 36, 37}; // D0..D7
static constexpr uint8_t PIN_PCLK  = 32;  // Pixel clock (external)
static constexpr uint8_t PIN_HREF  = 34;  // Line gate  (HIGH = active)
static constexpr uint8_t PIN_VSYNC = 33;  // Frame sync (GPIO interrupt)

// ============================================================
// CAMERA GEOMETRY — change for other resolutions
// Validated: 100x80, 240x120, 320x240, 640x480, 720x480, 720x520
// ============================================================
static const uint8_t  CAM_I2C_ADDR = 0x48;
static const uint16_t CAM_W        = 720;
static const uint16_t CAM_H        = 480;

// Frame stream header
static const uint8_t HDR[4] = {0xFF, 0xAA, 0xFF, 0xAA};

// ============================================================
// BUFFERS
// ============================================================
DMAMEM static uint8_t frame[(uint32_t)CAM_W * CAM_H]; // ~345 KB @ 720x480
DMAMEM static uint8_t lineBuf0[1024];
DMAMEM static uint8_t lineBuf1[1024];

static volatile uint32_t cur_line    = 0;
static volatile bool     frame_active = false;
static volatile bool     dma_busy    = false;
static uint8_t *volatile active_linebuf = lineBuf0;

// FlexIO / DMA handles
static const uint8_t FLEX_INDEX = 1;  // FlexIO2
FlexIOHandler      *pFlex = nullptr;
IMXRT_FLEXIO_t     *p     = nullptr;
DMAChannel          dma;

static const uint8_t SHIFTER = 0;
static const uint8_t TIMER   = 0;

static uint8_t flex_d0   = 0;
static uint8_t flex_pclk = 0;
static uint8_t flex_href = 0;

// ============================================================
// I2C HELPER
// ============================================================
static inline void writeReg16(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(CAM_I2C_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  Wire.endTransmission();
}

// ============================================================
// MT9V034 SENSOR INIT
// ============================================================
static void sensor_init_config() {
  // Master clock, DOUT enable, sequential context A
  writeReg16(MTV_CHIP_CONTROL_REG,
             MTV_CHIP_CONTROL_MASTER_MODE |
             MTV_CHIP_CONTROL_DOUT_ENABLE |
             MTV_CHIP_CONTROL_SEQUENTIAL);

  // 8-bit companded output (reduces bandwidth vs 10-bit raw)
  writeReg16(MTV_ADC_RES_CTRL_REG, MTV_ADC_RES_COMP_A);

  // Disable Auto Exposure Control and Auto Gain Control
  writeReg16(MTV_AEC_AGC_ENABLE_REG,
             MTV_AEC_DISABLE_A | MTV_AGC_DISABLE_A);

  // Analog gain — 1x
  writeReg16(MTV_ANALOG_GAIN_CTRL_REG_A,
             (uint16_t)(16 | MTV_ANALOG_GAIN_FORCE_0_75));

  // Digital gain tiles — neutral
  for (uint16_t i = 0; i <= 24; i++)
    writeReg16(MTV_TILED_DIGITAL_GAIN_REG + i,
               (uint16_t)(0 | (0xF << 4)));

  // ROI — centred at target resolution
  const uint16_t col_start =
    MINIMUM_COLUMN_START + ((MAX_IMAGE_WIDTH - CAM_W) / 2);
  writeReg16(MTV_COLUMN_START_REG_A,  col_start);
  writeReg16(MTV_ROW_START_REG_A,     MINIMUM_ROW_START);
  writeReg16(MTV_WINDOW_WIDTH_REG_A,  CAM_W);
  writeReg16(MTV_WINDOW_HEIGHT_REG_A, CAM_H);

  // Blanking (nominal)
  writeReg16(MTV_HOR_BLANKING_REG_A, 94);
  writeReg16(MTV_VER_BLANKING_REG_A, 45);

  // Row noise correction — enabled
  writeReg16(MTV_ROW_NOISE_CORR_CTRL_REG, 0x0001);

  // PCLK polarity — normal (set 0x0010 to invert if frames appear shifted)
  writeReg16(0x72, 0x0000);

  // Coarse exposure — adjust for scene brightness
  writeReg16(MTV_COARSE_SW_TOTAL_REG_A, 350);

  // No row/column binning
  writeReg16(MTV_READ_MODE_REG_A, 0x0000);
}

// ============================================================
// USB FRAME STREAM
// ============================================================
static void send_frame() {
  Serial.write(HDR, 4);
  uint16_t w = CAM_W, h = CAM_H;
  Serial.write((uint8_t*)&w, 2);
  Serial.write((uint8_t*)&h, 2);
  Serial.write(frame, (uint32_t)w * h);
  Serial.send_now();
}

// ============================================================
// DMA ISR — called when one line transfer completes
// ============================================================
static void dma_isr() {
  dma.clearInterrupt();
  dma.disable();
  if (frame_active && cur_line < CAM_H) {
    memcpy(frame + (uint32_t)cur_line * CAM_W,
           active_linebuf, CAM_W);
    cur_line++;
  }
  dma_busy = false;
}

// ============================================================
// ARM DMA FOR NEXT LINE
// ============================================================
static void arm_dma_for_line(uint8_t *linebuf, uint32_t count) {
  volatile uint8_t *src8;

#if defined(__IMXRT1062__)
  #if defined(FLEXIO_SHIFTBUFBYS)
    src8 = (volatile uint8_t*)&p->SHIFTBUFBYS[SHIFTER];
  #elif defined(FLEXIO_SHIFTBUFBBS)
    src8 = (volatile uint8_t*)&p->SHIFTBUFBBS[SHIFTER];
  #else
    src8 = (volatile uint8_t*)&p->SHIFTBUF[SHIFTER];
  #endif
#else
    src8 = (volatile uint8_t*)&p->SHIFTBUF[SHIFTER];
#endif

  dma.source(*src8);
  dma.destinationBuffer(linebuf, count);
  dma.transferSize(1);
  dma.disableOnCompletion();
  dma.triggerAtHardwareEvent(
#if defined(DMAMUX_SOURCE_FLEXIO2_SHIFTER0)
    DMAMUX_SOURCE_FLEXIO2_SHIFTER0
#else
    DMAMUX_SOURCE_FLEXIO2_REQUEST0
#endif
  );
  dma.enable();
  dma_busy = true;
}

// ============================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================
static void isr_vsync() {
  // Falling edge = frame start; rising edge = frame end
  if (digitalReadFast(PIN_VSYNC) == LOW) {
    frame_active = true;
    cur_line = 0;
  } else {
    frame_active = false;
  }
}

static void isr_href_rise() {
  if (!frame_active)      return;
  if (cur_line >= CAM_H)  return;
  if (dma_busy)           return;
  // Swap ping-pong buffer
  active_linebuf = (active_linebuf == lineBuf0) ? lineBuf1 : lineBuf0;
  arm_dma_for_line(active_linebuf, CAM_W);
}

// ============================================================
// FLEXIO2 PARALLEL-8 INIT
// ============================================================
static void flexio_init_parallel8() {
  pFlex = FlexIOHandler::flexIOHandler_list[FLEX_INDEX]; // FlexIO2
  p = &pFlex->port();

  // Route all camera signals to FlexIO
  for (int i = 0; i < 8; i++) pFlex->setIOPinToFlexMode(PIN_D[i]);
  pFlex->setIOPinToFlexMode(PIN_PCLK);
  pFlex->setIOPinToFlexMode(PIN_HREF);

  flex_d0   = pFlex->mapIOPinToFlexPin(PIN_D[0]);
  flex_pclk = pFlex->mapIOPinToFlexPin(PIN_PCLK);
  flex_href = pFlex->mapIOPinToFlexPin(PIN_HREF);

  // Reset then enable FlexIO module
  p->CTRL = FLEXIO_CTRL_FLEXEN | FLEXIO_CTRL_SWRST;
  p->CTRL = FLEXIO_CTRL_FLEXEN;

  // SHIFTER: receive, 8-bit parallel bus
  p->SHIFTCFG[SHIFTER] =
      FLEXIO_SHIFTCFG_PWIDTH(7);         // 8 parallel pins

  p->SHIFTCTL[SHIFTER] =
      FLEXIO_SHIFTCTL_TIMSEL(TIMER)    |
      FLEXIO_SHIFTCTL_PINCFG(1)        |  // input
      FLEXIO_SHIFTCTL_PINSEL(flex_d0)  |
      FLEXIO_SHIFTCTL_SMOD(1);            // receive mode

  // TIMER: decrement on PCLK; enable/disable on HREF
  p->TIMCFG[TIMER] =
      FLEXIO_TIMCFG_TIMOUT(0)  |
      FLEXIO_TIMCFG_TIMDEC(2)  |   // decrement on pin (PCLK)
      FLEXIO_TIMCFG_TIMRST(2)  |   // reset on trigger rising
      FLEXIO_TIMCFG_TIMDIS(2)  |   // disable on trigger falling
      FLEXIO_TIMCFG_TIMENA(2);     // enable on trigger rising

  p->TIMCTL[TIMER] =
      FLEXIO_TIMCTL_TRGSEL(flex_href) |
      FLEXIO_TIMCTL_TRGPOL           |   // active high
      FLEXIO_TIMCTL_TRGSRC           |   // internal trigger
      FLEXIO_TIMCTL_PINSEL(flex_pclk) |
      FLEXIO_TIMCTL_TIMOD(1);            // single 16-bit counter

  p->TIMCMP[TIMER] = 0xFFFF;

  // Enable DMA request on shifter flag
  p->SHIFTSIEN = (1u << SHIFTER);

  // Clear status flags
  p->SHIFTSTAT = (1u << SHIFTER);
  p->TIMSTAT   = (1u << TIMER);
}

// ============================================================
// HOST COMMAND HANDLER (optional serial control)
// ============================================================
static void handleHost() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("EXP ")) {
    // Set coarse exposure: e.g. "EXP 500"
    writeReg16(MTV_COARSE_SW_TOTAL_REG_A,
               (uint16_t)cmd.substring(4).toInt());
  } else if (cmd.startsWith("POL ")) {
    // Toggle PCLK polarity: "POL 1" inverts, "POL 0" normal
    uint16_t inv = (uint16_t)(cmd.substring(4).toInt()
                              ? 0x0010 : 0x0000);
    writeReg16(0x72, inv);
  }
}

// ============================================================
// ARDUINO SETUP & LOOP
// ============================================================
void setup() {
  Serial.begin(2000000);
  while (!Serial && millis() < 1200) {}

  for (int i = 0; i < 8; i++) pinMode(PIN_D[i], INPUT);
  pinMode(PIN_PCLK,  INPUT);
  pinMode(PIN_HREF,  INPUT);
  pinMode(PIN_VSYNC, INPUT);

  Wire.begin();
  Wire.setClock(400000);

  sensor_init_config();
  delay(30);

  flexio_init_parallel8();

  dma.disable();
  dma.attachInterrupt(dma_isr);
  dma.interruptAtCompletion();

  attachInterrupt(digitalPinToInterrupt(PIN_VSYNC),
                  isr_vsync, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_HREF),
                  isr_href_rise, RISING);
}

void loop() {
  handleHost();

  // Send completed frame over USB
  static bool prev_active = false;
  bool now_active = frame_active;
  if (prev_active && !now_active) {
    send_frame();
  }
  prev_active = now_active;
}
