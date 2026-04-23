# Realtime Embedded Vision Pipeline — Teensy 4.x + MT9V034

> Deterministic real-time grayscale video capture pipeline
> using Teensy 4.x, MT9V034 global-shutter camera,
> FlexIO2 parallel-8 + DMA — achieving up to 720×480
> @ ~60 FPS with near-zero CPU overhead.

---

## Overview

This project documents a feasibility study for achieving
maximum resolution and frame rate from an MT9V034
global-shutter camera sensor paired with a Teensy 4.x
microcontroller — targeting real-time edge vision for
robotics and industrial inspection applications.

Designed and validated as an independent embedded vision
consultancy engagement. The pipeline was tested on a
**Denso VS-6577 robotic arm end-effector** at HHRCM Lab,
NCRA-NEDUET to validate real-world integration.

**Author:** Humayun Khan — Embedded Vision Consultant  
**Status:** Validated — multiple resolutions confirmed  
**Client:** Industrial R&D (Confidential)  
**Goal:** Establish Teensy + global-shutter feasibility
as a stepping stone toward custom ASIC design

---

## Problem It Solves

Most embedded vision pipelines on microcontrollers either:
- Use CPU-polled capture — too slow for >30 FPS
- Require dedicated FPGAs or DSPs — expensive, complex

This project establishes what the Teensy 4.x iMXRT1062
(FlexIO2 + DMA hardware blocks) can achieve with an
MT9V034 global-shutter sensor on an 8-bit parallel bus —
identifying resolution/FPS bottlenecks to inform
ASIC architecture decisions.

---

## System Architecture
<img width="1536" height="1024" alt="Architecture-Teensy-Embedded vision" src="https://github.com/user-attachments/assets/092478d1-7d03-4961-8650-84732ea192e7" />

```


[MT9V034 Global Shutter Sensor]
 8-bit parallel bus (D0–D7)
 + PCLK  (pixel clock)
 + HREF  (line gate — HIGH = active pixels)
 + VSYNC (frame sync)
 + I²C   (register configuration)
         ↓
[Teensy 4.x — iMXRT1062 600 MHz]
 FlexIO2 SHIFTIN (8-bit parallel, PWIDTH=7)
 clocked by external PCLK, gated by HREF
         ↓
 DMA ping-pong line buffers (lineBuf0 / lineBuf1)
 → DMAMEM frame buffer (~345 KB @ 720×480)
         ↓
 USB Serial stream — framed protocol:
 [FF AA FF AA][uint16 W][uint16 H][payload]
         ↓
[Python Host — viewer.py]
 pyserial reader thread → NumPy decode
 → OpenCV real-time display + FPS overlay
```

---

## Resolutions & Performance

| Resolution | FPS | Method |
|---|---|---|
| 100 × 80 | ~60 FPS | FlexIO2 + DMA |
| 240 × 120 | ~60 FPS | FlexIO2 + DMA |
| 320 × 240 | ~60 FPS | FlexIO2 + DMA |
| 640 × 480 | ~60 FPS | FlexIO2 + DMA |
| **720 × 480** | **~60 FPS** | **FlexIO2 + DMA** |
| 720 × 520 | ~60 FPS | FlexIO2 + DMA |
| 640 × 480 | ~18 FPS | Basic CPU-polled |

**Key finding:** CPU-polled capture is bottlenecked at
~18 FPS at 640×480. FlexIO2 + DMA offloads pixel capture
entirely from the CPU — achieving deterministic ~60 FPS
across all tested resolutions. The primary remaining
bottleneck is USB throughput, not the MCU or FlexIO engine.

---

## Hardware Stack

| Component | Details |
|---|---|
| Microcontroller | Teensy 4.x (iMXRT1062, 600 MHz) |
| Camera | MT9V034 global-shutter sensor (8-bit grayscale) |
| Bus | 8-bit parallel (PCLK · HREF · VSYNC · D0–D7) |
| Capture engine | FlexIO2 SHIFTIN + DMA ping-pong |
| Frame buffer | DMAMEM ~345 KB (720×480 × 1 byte) |
| Camera config | 8-bit companded via I²C @ 400 kHz |
| Host interface | USB Serial (2 Mbaud) |
| Build platform | Teensyduino 1.59 + KurtE FlexIO_t4 |

<img width="798" height="436" alt="Teensy" src="https://github.com/user-attachments/assets/2231b554-4d3f-4468-adad-3a6747bee843" />

<img width="800" height="1000" alt="Teensy-cam-mtv0634" src="https://github.com/user-attachments/assets/6c7456df-1ae6-4637-9605-1b9f12bf153b" />


---

## Software Stack

| Layer | Technology |
|---|---|
| Firmware | C++ (Teensyduino / Arduino) |
| Parallel capture | FlexIO2 SHIFTIN, 8-bit (PWIDTH=7) |
| DMA | DMAChannel ping-pong line buffers |
| Frame sync | VSYNC ISR (CHANGE) + HREF ISR (RISING) |
| Camera config | I²C register map (MT9V034) |
| Host stream | USB Serial — framed protocol |
| Visualisation | Python 3 — pyserial + NumPy + OpenCV |

<img width="400" height="300" alt="Teensy-cam5" src="https://github.com/user-attachments/assets/a39729c0-00c8-4a18-9389-fa54446418f2" />
<img width="400" height="300" alt="Teensy-cam3" src="https://github.com/user-attachments/assets/f549a5a8-cae7-42d4-9ff3-31f3757ab4e8" />
---

## Source Code

| File | Description |
|---|---|
| [`src/Teensy-MT9V034-FlexIO-DMA.ino`](src/Teensy-MT9V034-FlexIO-DMA.ino) | Main firmware — FlexIO2 parallel-8 + DMA, ISRs, I²C sensor init, USB stream, host commands |
| [`src/Teensy-MT9V034-Basic.ino`](src/Teensy-MT9V034-Basic.ino) | Baseline firmware — CPU-polled capture (no FlexIO), ~18 FPS @ 640×480 |
| [`src/viewer.py`](src/viewer.py) | Python host viewer — serial reader thread, NumPy frame decode, OpenCV display, FPS overlay |

> ⚠️ Set `PIN_D[8]`, `PIN_PCLK`, `PIN_HREF`, `PIN_VSYNC`
> to match your board wiring before flashing.
> All 8 data pins must map to consecutive FlexIO2 indices.
> Set `SERIAL_PORT` in `viewer.py` to your Teensy COM port.

---

## Pin Map (Default)

| Signal | Teensy Pin | Notes |
|---|---|---|
| D0 | 10 | LSB |
| D1 | 12 | |
| D2 | 11 | |
| D3 | 13 | |
| D4 | 8 | |
| D5 | 7 | |
| D6 | 36 | |
| D7 | 37 | MSB |
| PCLK | 32 | External pixel clock |
| HREF | 34 | Line gate |
| VSYNC | 33 | Frame sync (GPIO interrupt) |
| SDA | 18 | I²C — camera config |
| SCL | 19 | I²C — camera config |

> D0–D7 must be **consecutive in FlexIO2 pin space**.
> Verify with `pFlex->mapIOPinToFlexPin()`.
> If non-consecutive, use the two-nibble (4+4) variant.

---

## Deployment

<img width="600" height="800" alt="Teensy-cam4" src="https://github.com/user-attachments/assets/c9e7a011-3b68-43d5-a0ab-f89052c5c23b" />


**Integration test:** Camera module mounted on
**Custom-built robotic arm end-effector** at HHRCM Lab,
NCRA-NEDUET — tested real-time frame streaming at
target FPS during arm motion.


> 📌 This was a feasibility and benchmarking study.
> Results are from DevOps testing.
> Client application details are confidential.

---

## Key Findings & ASIC Recommendation

- **FlexIO2 + DMA is the correct architecture** for
  parallel camera interfaces on iMXRT1062 — CPU load
  near zero during capture
- **USB is the bottleneck** at higher resolutions,
  not the MCU or FlexIO engine
- **8-bit companded mode** significantly reduces
  bandwidth vs 10-bit raw while preserving dynamic range
- **ASIC recommendation:** Dedicated parallel-8 capture
  block + on-chip SRAM + USB 3.0 output removes all
  remaining bottlenecks identified in this study

---

## Related Projects

- [NED-EIoT](https://github.com/HumayunNaveedKhan/Embedded-Industrial-Energy-Monitoring-Device-NED_EIoT)
  — Industrial Energy & Equipment Health Monitor
- [NED-SILL](https://github.com/HumayunNaveedKhan/Smart-Industrial-Liquid-Level-Monitoring-Device-NED_SILL)
  — Smart Industrial Liquid Level Monitor

---

## Author

**Humayun Khan** — Embedded Vision Consultant  
Team Lead, HHRCM Lab, NCRA-NEDUET  
Co-Founder, Haptronica & RobAutoStem (NIC Karachi, Cohort 12)

📧 humayunnaveedkhan@gmail.com  
🔗 [LinkedIn](https://linkedin.com/in/humayunnaveedkhan)  
🌐 [Portfolio](https://humayunnaveedkhan.github.io/portfolio)

---

## License

© 2025 Humayun Khan. All rights reserved.  
No part of this repository may be reproduced or used
commercially without written permission.
