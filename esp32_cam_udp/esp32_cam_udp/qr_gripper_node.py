#!/usr/bin/env python3
"""QR in view -> gripper closes; QR gone -> gripper opens. Gripper ESP32 over serial.

Serial protocol (same as the arm firmware):  GRIPPER:<angle>\n   (0-180)

Behaviour:
  * QR detected on /qr/data          -> send GRIPPER:<gripper_angle>   (default 0 = closed)
  * no QR for release_timeout secs   -> send GRIPPER:<release_angle>   (default 90 = open)

Params:
  port, baud_rate, qr_topic, joint
  trigger_text      only this exact QR text counts; '' = any QR
  gripper_angle     angle when QR seen            (default 0)
  release_angle     angle when QR not seen        (default 90)
  release_timeout   seconds without QR before releasing (default 1.5)
Manual: publish std_msgs/Int32 on /gripper/angle.
"""
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Int32, String


class QrGripperNode(Node):
    def __init__(self):
        super().__init__('qr_gripper_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('qr_topic', '/qr/data')
        self.declare_parameter('trigger_text', '')
        self.declare_parameter('gripper_angle', 0)
        self.declare_parameter('release_angle', 90)
        self.declare_parameter('release_timeout', 1.5)
        self.declare_parameter('joint', 'GRIPPER')
        self.declare_parameter('connect_wait', 2.0)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud_rate').value)
        self.trigger = self.get_parameter('trigger_text').value
        self.close_angle = int(self.get_parameter('gripper_angle').value)
        self.open_angle = int(self.get_parameter('release_angle').value)
        self.timeout = float(self.get_parameter('release_timeout').value)
        self.joint = self.get_parameter('joint').value
        self.wait = float(self.get_parameter('connect_wait').value)

        self.ser = None
        self.lock = threading.Lock()
        self.closed = False
        self.last_seen = 0.0
        self.manual_until = 0.0

        if self.connect():
            self.send(self.open_angle)       # start open

        self.create_subscription(String, self.get_parameter('qr_topic').value, self.on_qr, 10)
        self.create_subscription(Int32, '/gripper/angle', self.on_manual, 10)
        self.create_timer(0.1, self.check_release)
        self.get_logger().info(
            f'QR seen -> {self.joint}:{self.close_angle}; '
            f'no QR for {self.timeout}s -> {self.joint}:{self.open_angle}')

    def connect(self):
        try:
            with self.lock:
                self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(self.wait)            # ESP32 resets when the port opens
            threading.Thread(target=self.reader, args=(self.ser,), daemon=True).start()
            self.get_logger().info(f'Gripper ESP32 connected on {self.port}')
            return True
        except Exception as exc:
            self.ser = None
            self.get_logger().error(f'Cannot open {self.port}: {exc}')
            return False

    def reader(self, ser):
        while rclpy.ok() and ser.is_open:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.get_logger().info(f'ESP32: {line}')
            except Exception:
                break

    def send(self, angle):
        angle = max(0, min(180, int(angle)))
        if self.ser is None or not self.ser.is_open:
            if not self.connect():
                return False
        try:
            with self.lock:
                self.ser.write(f'{self.joint}:{angle}\n'.encode())
            self.get_logger().info(f'Sent {self.joint}:{angle}')
            return True
        except Exception as exc:
            self.get_logger().error(f'Serial write failed: {exc}')
            self.ser = None
            return False

    def on_qr(self, msg: String):
        text = msg.data.strip()
        if self.trigger and text != self.trigger:
            return
        self.last_seen = time.time()
        if not self.closed:
            self.get_logger().info(f'QR "{text}" -> closing gripper ({self.close_angle} deg)')
            if self.send(self.close_angle):
                self.closed = True

    def check_release(self):
        if self.closed and time.time() - self.last_seen > self.timeout:
            self.get_logger().info(f'QR lost -> opening gripper ({self.open_angle} deg)')
            if self.send(self.open_angle):
                self.closed = False

    def on_manual(self, msg: Int32):
        self.send(msg.data)

    def destroy_node(self):
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QrGripperNode()
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
