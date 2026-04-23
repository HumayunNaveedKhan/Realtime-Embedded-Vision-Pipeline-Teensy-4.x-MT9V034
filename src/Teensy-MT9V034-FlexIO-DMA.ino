/**
 * Realtime Embedded Vision Pipeline
 * Teensy 4.x + MT9V034 — FlexIO2 + DMA
 * Version: 1.0
 *
 * Deterministic 8-bit parallel grayscale capture from 
 * MT9V034 global-shutter sensor using FlexIO2 SHIFTIN 
 * + DMA ping-pong line buffers.
 *
 * Resolutions validated: 100×80 through 720×520 @ ~60 FPS
 * Protocol: [FF AA FF AA][uint16 W][uint16 H][payload]
 *
 * Build: Teensyduino 1.59 + KurtE FlexIO_t4
 *
 * CONFIGURATION:
 * Set PIN_D[8], PIN_PCLK, PIN_HREF, PIN_VSYNC to match
 * your board wiring. Verify FlexIO2 pin consecutiveness.
 *
 * Author:  Humayun Khan
 * Lab:     HHRCM Lab, NCRA-NEDUET, Karachi, Pakistan
 *
 * © 2025 Humayun Khan, Embedded Systems Consultant.
 */
