#!/usr/bin/env python3
"""Fake ESP32-CAM: sends a test pattern (or a webcam) over UDP so you can test
the receiver without the hardware.

  ros2 run esp32_cam_udp udp_test_sender --host 127.0.0.1 --port 5005
  ros2 run esp32_cam_udp udp_test_sender --webcam 0
"""
import argparse
import socket
import struct
import time

import cv2
import numpy as np

CHUNK = 1200


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5005)
    ap.add_argument('--webcam', type=int, default=None)
    ap.add_argument('--fps', type=float, default=15)
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.webcam) if a.webcam is not None else None
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fid = 0
    while True:
        if cap is not None:
            ok, img = cap.read()
            if not ok:
                break
            img = cv2.resize(img, (320, 240))
        else:
            img = np.zeros((240, 320, 3), np.uint8)
            cv2.putText(img, f'frame {fid}', (40, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        data = buf.tobytes()
        total = (len(data) + CHUNK - 1) // CHUNK
        for i in range(total):
            hdr = struct.pack('<HHH', fid & 0xFFFF, i, total)
            sock.sendto(hdr + data[i * CHUNK:(i + 1) * CHUNK], (a.host, a.port))
        fid += 1
        time.sleep(1.0 / a.fps)


if __name__ == '__main__':
    main()
