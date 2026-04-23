"""
Realtime Embedded Vision Pipeline
MT9V034 Frame Viewer — Python Host
Version: 1.0

USB serial frame reader and real-time OpenCV viewer
for Teensy MT9V034 firmware. Decodes framed protocol:
[FF AA FF AA][uint16 W][uint16 H][grayscale payload]

Dependencies: pyserial, numpy, opencv-python
  pip install pyserial numpy opencv-python

Usage:
  1. Set SERIAL_PORT to your Teensy COM port
  2. Run: python viewer.py
  3. Press 'q' to quit

Author:  Humayun Khan
Role:    Embedded Vision Consultant
Contact: humayunnaveedkhan@gmail.com

© 2025 Humayun Khan. All rights reserved.
"""

import serial
import numpy as np
import cv2
import threading
import queue
import os
import time

SERIAL_PORT = 'COM3'    # <-- set to your Teensy COM port
BAUD_RATE   = 2000000   # Teensy HS USB ignores baud, but set anyway
HEADER      = b'\xFF\xAA\xFF\xAA'
SAVE_DIR    = "captured_frames"
SAVE_EVERY  = 0         # 0 = no auto-save; N = save every N frames

frame_q = queue.Queue(maxsize=120)

def reader(ser):
    """
    Background thread: reads raw bytes from Teensy USB serial,
    finds frame headers, validates dimensions, and pushes
    decoded numpy grayscale frames into frame_q.
    """
    buf = bytearray()
    MAX_W, MAX_H = 752, 480
    MAX_PIX = MAX_W * MAX_H

    while True:
        chunk = ser.read(32768)
        if chunk:
            buf += chunk

        while True:
            idx = buf.find(HEADER)
            if idx < 0:
                if len(buf) > 3:
                    buf = buf[-3:]
                break
            if idx > 0:
                buf = buf[idx:]
                idx = 0
            if len(buf) < 8:
                break

            # Parse width and height (little-endian uint16)
            w = buf[4] | (buf[5] << 8)
            h = buf[6] | (buf[7] << 8)
            need = w * h

            if w < 1 or h < 1 or need < 1 or need > MAX_PIX:
                buf = buf[1:]
                continue

            total = 8 + need
            if len(buf) < total:
                break

            payload = buf[8:total]
            buf = buf[total:]

            try:
                frame = np.frombuffer(payload,
                                      dtype=np.uint8).reshape(h, w)
                try:
                    frame_q.put_nowait(frame)
                except queue.Full:
                    _ = frame_q.get_nowait()
                    frame_q.put_nowait(frame)
            except Exception:
                pass


def main():
    os.makedirs(SAVE_DIR, exist_ok=True)
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.02)
    ser.reset_input_buffer()

    threading.Thread(target=reader, args=(ser,), daemon=True).start()

    frames = 0
    last   = time.time()

    while True:
        try:
            frame = frame_q.get(timeout=0.3)
        except queue.Empty:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        now = time.time()
        fps = 1.0 / max(1e-6, now - last)
        last = now

        # Display at fixed 720x480 regardless of capture resolution
        disp = cv2.resize(frame, (720, 480),
                          interpolation=cv2.INTER_NEAREST)

        # FPS overlay (top-right)
        text = f"{fps:.1f} FPS"
        (tw, th), _ = cv2.getTextSize(text,
                                      cv2.FONT_HERSHEY_SIMPLEX,
                                      0.5, 1)
        cv2.putText(disp, text,
                    (disp.shape[1] - tw - 6, th + 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255,), 1, cv2.LINE_AA)

        cv2.imshow("MT9V034 FlexIO2 — Realtime Vision Pipeline", disp)

        frames += 1
        if SAVE_EVERY and (frames % SAVE_EVERY == 0):
            cv2.imwrite(
                os.path.join(SAVE_DIR, f"f_{frames:06d}.png"),
                frame
            )

        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
