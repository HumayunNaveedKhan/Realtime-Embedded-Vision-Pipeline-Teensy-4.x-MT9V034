# Realtime Embedded Vision Pipeline — Teensy 4.x + MT9V034

> Deterministic real-time grayscale video capture pipeline 
> using Teensy 4.x, MT9V034 global-shutter camera, FlexIO2 
> parallel-8 + DMA — achieving up to 720×480 @ ~60 FPS 
> with minimal CPU overhead.

---

## Overview

This project documents a feasibility study for achieving 
maximum resolution and frame rate from an MT9V034 
global-shutter camera sensor paired with a Teensy 4.x 
microcontroller — targeting real-time edge vision for 
robotics and industrial inspection applications.

The pipeline was designed and tested on a robotic arm end-effector for integrated 
field testing.

**Status:** Validated — multiple resolutions confirmed  
**Client:** Industrial R&D (Confidential)  
**Goal:** Establish Teensy + global-shutter feasibility 
as a stepping stone toward ASIC design  

---

## Problem It Solves

Most embedded vision pipelines on microcontrollers either:
- Use CPU-polled capture (too slow for >30 FPS)
- Require dedicated FPGAs or DSPs (expensive, complex)

This project establishes what the Teensy 4.x iMXRT1062 
(FlexIO + DMA hardware) can achieve with an MT9V034 
global-shutter sensor using an 8-bit parallel bus — 
identifying resolution/FPS bottlenecks to inform ASIC 
architecture decisions.

---

## System Architecture
[MT9V034 Global Shutter Sensor]
8-bit parallel data bus (D0–D7)

PCLK (pixel clock)
HREF (line gate)
VSYNC (frame sync)
↓
[Teensy 4.x iMXRT1062]
FlexIO2 SHIFTIN (8-bit parallel)
clocked by external PCLK
gated by HREF
↓
DMA ping-pong line buffers
→ frame buffer (DMAMEM ~345 KB)
↓
USB Serial stream
[FF AA FF AA][W uint16][H uint16][payload]
↓
[Python Viewer — OpenCV + NumPy]
Real-time display + FPS overlay
optional frame save to disk


---

## Resolutions & Performance Tested

| Resolution | FPS Achieved | Method |
|---|---|---|
| 100 × 80 | ~60 FPS | FlexIO2 + DMA |
| 240 × 120 | ~60 FPS | FlexIO2 + DMA |
| 320 × 240 | ~60 FPS | FlexIO2 + DMA |
| 640 × 480 | ~60 FPS | FlexIO2 + DMA |
| **720 × 480** | **~60 FPS** | **FlexIO2 + DMA** |
| 720 × 520 | ~60 FPS | FlexIO2 + DMA |
| 640 × 480 | ~18 FPS | Basic (no FlexIO) |

**Key finding:** CPU-polled capture (no FlexIO) is bottlenecked 
at ~18 FPS at 640×480. FlexIO2 + DMA offloads pixel capture 
entirely from the CPU, achieving deterministic ~60 FPS across 
all tested resolutions — limited by USB throughput, not MCU.

---

## Hardware Stack

| Component | Details |
|---|---|
| Microcontroller | Teensy 4.x (iMXRT1062, 600 MHz) |
| Camera | MT9V034 global-shutter sensor (grayscale) |
| Interface | 8-bit parallel (PCLK, HREF, VSYNC, D0–D7) |
| Camera config | 8-bit companded mode, ROI-configurable |
| Capture engine | FlexIO2 SHIFTIN + DMA ping-pong |
| Frame buffer | DMAMEM ~345 KB (720×480 × 1 byte) |
| Host interface | USB Serial (2 Mbaud) |
| I²C config | MT9V034 register map via Wire @ 400 kHz |
| Platform | Teensyduino 1.59 + KurtE FlexIO_t4 |

---

## Software Stack

| Layer | Technology |
|---|---|
| Firmware | C++ (Teensyduino / Arduino) |
| Parallel capture | FlexIO2 SHIFTIN, 8-bit (PWIDTH=7) |
| DMA | DMAChannel ping-pong line buffers |
| Frame sync | VSYNC ISR (CHANGE) + HREF ISR (RISING) |
| Camera config | I²C register writes (MT9V034 map) |
| Host stream | USB Serial — framed protocol |
| Visualisation | Python 3 — serial + NumPy + OpenCV |

---

## Source Code

| File | Description |
|---|---|
| [`src/Teensy-MT9V034-FlexIO-DMA.ino`](src/Teensy-MT9V034-FlexIO-DMA.ino) | Main firmware — FlexIO2 parallel-8 + DMA, ISRs, I²C camera init, USB stream |
| [`src/Teensy-MT9V034-Basic.ino`](src/Teensy-MT9V034-Basic.ino) | Baseline firmware — CPU-polled capture (no FlexIO), for comparison |
| [`src/viewer.py`](src/viewer.py) | Python host viewer — USB serial reader, NumPy frame decode, OpenCV display, FPS overlay |

> ⚠️ Set `PIN_D[8]`, `PIN_PCLK`, `PIN_HREF`, `PIN_VSYNC` 
> to match your board wiring before flashing.  
> Set `SERIAL_PORT` in `viewer.py` to your Teensy COM port.

---

## Deployment

**Integration:** Mounted on a **Customized 5 DOF robotic arm 
end-effector** for field validation — verified real-time 
frame streaming while the camera module tracks target 
positions during arm motion.

**Environment:** HHRCM Lab, NCRA-NEDUET, Karachi, Pakistan

> 📌 This was a feasibility and benchmarking study. 
> Results are from DevOps testing. Client application 
> details are confidential.

---

## Key Findings

- **FlexIO2 + DMA is the correct architecture** for 
  parallel camera interfaces on iMXRT1062 — CPU load 
  during capture is near zero
- **USB throughput is the real bottleneck** at higher 
  resolutions, not the MCU or FlexIO engine
- **MT9V034 8-bit companded mode** significantly reduces 
  bandwidth versus 10-bit raw while preserving 
  dynamic range for grayscale applications
- **ASIC recommendation:** A dedicated parallel-8 
  capture block with on-chip SRAM buffering and USB3 
  output would remove all remaining bottlenecks

---

## Pin Map (Default)

| Signal | Teensy Pin | Notes |
|---|---|---|
| D0 | 10 | LSB — must be consecutive in FlexIO2 space |
| D1 | 12 | |
| D2 | 11 | |
| D3 | 13 | |
| D4 | 8 | |
| D5 | 7 | |
| D6 | 36 | |
| D7 | 37 | MSB |
| PCLK | 32 | External sample clock |
| HREF | 34 | Line gate (HIGH = active pixels) |
| VSYNC | 33 | Frame sync (GPIO interrupt) |
| SDA | 18 | I²C — MT9V034 config |
| SCL | 19 | I²C — MT9V034 config |

> All 8 data pins must map to **consecutive FlexIO2 
> indices** — verify with `pFlex->mapIOPinToFlexPin()`.

---

## Related Work

- [NED-EIoT](https://github.com/HumayunNaveedKhan/Embedded-Industrial-Energy-Monitoring-Device-NED_EIoT) 
  — Industrial Energy & Equipment Health Monitor
- [NED-SILL](https://github.com/HumayunNaveedKhan/Smart-Industrial-Liquid-Level-Monitoring-Device-NED_SILL) 
  — Smart Industrial Liquid Level Monitor

---

## Author

**Humayun Khan**  
Embedded Systems Consultant,
Co-Founder, Haptronica (incubated in BIC NEDUET) & RobAutoStem (incubated in BIC NEDUET, NIC Karachi-Cohort 12)

📧 humayunnaveedkhan@gmail.com  
🔗 [LinkedIn](https://linkedin.com/in/humayunnaveedkhan)  
🌐 [Portfolio](https://humayunnaveedkhan.github.io/portfolio)

---

## License

© 2025 Humayun Khan, Embedded Systems Consultant.  
All rights reserved.  
No part of this repository may be reproduced or used 
commercially without written permission.
