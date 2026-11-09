#!/usr/bin/env python3
"""Receive JPEG frames chunked over UDP from an ESP32-CAM and publish to ROS 2.

Packet format: 6-byte header (frame_id, chunk_idx, chunk_count; uint16 LE) + JPEG bytes.
"""
import socket
import struct
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class UdpCameraNode(Node):
    def __init__(self):
        super().__init__('udp_camera_node')
        self.declare_parameter('port', 5005)
        self.declare_parameter('topic', '/camera/image/compressed')
        self.declare_parameter('frame_id', 'esp32_cam')
        self.declare_parameter('show', False)
        self.declare_parameter('detect_qr', False)
        self.declare_parameter('qr_topic', '/qr/data')
        self.declare_parameter('stale_timeout', 0.5)
        self.declare_parameter('rcvbuf_mb', 4)

        self.port = int(self.get_parameter('port').value)
        self.frame_name = self.get_parameter('frame_id').value
        self.show = bool(self.get_parameter('show').value)
        self.detect_qr = bool(self.get_parameter('detect_qr').value)
        self.stale = float(self.get_parameter('stale_timeout').value)

        self.img_pub = self.create_publisher(
            CompressedImage, self.get_parameter('topic').value, 1)
        self.qr_pub = self.create_publisher(
            String, self.get_parameter('qr_topic').value, 10)
        self.qr = cv2.QRCodeDetector() if self.detect_qr else None
        self.last_qr = ''
        self.last_qr_seen = 0.0
        self.declare_parameter('rotate_180', True)        # rotate the image 180 deg (preview + QR decode)
        self.declare_parameter('rotate_published', False)  # also rotate the image published on the topic
        self.rotate = bool(self.get_parameter('rotate_180').value)
        self.rotate_pub = bool(self.get_parameter('rotate_published').value)
        self.declare_parameter('qr_gap', 2.0)   # s QR must be out of view before the same text re-triggers
        self.qr_gap = float(self.get_parameter('qr_gap').value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF,
                             int(self.get_parameter('rcvbuf_mb').value) * 1024 * 1024)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.settimeout(1.0)

        self.frames = {}          # frame_id -> {total, chunks{idx: bytes}, t}
        self.last_done = -1
        self.count, self.t0 = 0, time.time()
        self.dropped = 0
        self.running = True

        # Reception runs in its own thread; decoding/QR/show runs in a second
        # thread so a slow decode never makes the socket drop packets.
        self.latest = None
        self.latest_lock = threading.Lock()
        self.latest_evt = threading.Event()
        threading.Thread(target=self.recv_loop, daemon=True).start()
        if self.show or self.qr is not None:
            threading.Thread(target=self.vision_loop, daemon=True).start()

        self.get_logger().info(f'Listening for ESP32-CAM UDP video on port {self.port}')

    def recv_loop(self):
        last_purge = time.time()
        while self.running:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) <= 6:
                continue

            fid, idx, total = struct.unpack('<HHH', data[:6])
            if total == 0 or idx >= total:
                continue
            f = self.frames.setdefault(fid, {'total': total, 'chunks': {}, 't': time.time()})
            f['chunks'][idx] = data[6:]

            if len(f['chunks']) == f['total']:
                jpeg = b''.join(f['chunks'][i] for i in range(f['total']))
                del self.frames[fid]
                # drop frames older than the last delivered one (uint16 wrap-safe)
                if self.last_done != -1 and ((fid - self.last_done) & 0xFFFF) > 0x8000:
                    continue
                self.last_done = fid
                self.handle_frame(jpeg)

            now = time.time()
            if now - last_purge > 0.2:
                last_purge = now
                for k in [k for k, v in self.frames.items() if now - v['t'] > self.stale]:
                    del self.frames[k]
                    self.dropped += 1

    def handle_frame(self, jpeg: bytes):
        if not (jpeg[:2] == b'\xff\xd8' and jpeg[-2:] == b'\xff\xd9'):
            self.dropped += 1
            return

        if self.rotate and self.rotate_pub:
            im = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if im is not None:
                ok, buf = cv2.imencode('.jpg', cv2.rotate(im, cv2.ROTATE_180),
                                       [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    jpeg = buf.tobytes()

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_name
        msg.format = 'jpeg'
        msg.data = jpeg
        self.img_pub.publish(msg)

        self.count += 1
        dt = time.time() - self.t0
        if dt >= 5.0:
            self.get_logger().info(
                f'{self.count / dt:.1f} fps, {self.dropped} incomplete frames dropped')
            self.count, self.dropped, self.t0 = 0, 0, time.time()

        if self.show or self.qr is not None:
            with self.latest_lock:
                self.latest = jpeg
            self.latest_evt.set()

    def vision_loop(self):
        while self.running:
            if not self.latest_evt.wait(0.5):
                continue
            self.latest_evt.clear()
            with self.latest_lock:
                jpeg = self.latest
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            if self.rotate and not self.rotate_pub:
                img = cv2.rotate(img, cv2.ROTATE_180)
            if self.qr is not None:
                text = self.decode_qr(img)
                if text:
                    now = time.time()
                    # published on EVERY detection so the gripper node knows the QR is still in view
                    self.qr_pub.publish(String(data=text))
                    if text != self.last_qr or now - self.last_qr_seen > self.qr_gap:
                        print(f'\n*** QR CODE: {text}\n', flush=True)
                        self.get_logger().info(f'QR detected: {text}')
                    self.last_qr, self.last_qr_seen = text, now
            if self.show:
                cv2.imshow('esp32_cam_udp', img)
                cv2.waitKey(1)

    def decode_qr(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        up = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        for g in (gray, up):
            try:
                text, _, _ = self.qr.detectAndDecode(g)
            except cv2.error:
                text = ''
            if text:
                return text
        return ''

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
