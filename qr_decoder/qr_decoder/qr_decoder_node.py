import json

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String

from qr_decoder.qr_tracker import QRTracker

GREEN, YELLOW, RED = (0, 255, 0), (0, 255, 255), (0, 0, 255)
ORANGE = (0, 140, 255)   # the box the gripper grips


class QRDecoderNode(Node):

    def __init__(self):
        super().__init__('qr_decoder_node')

        self.declare_parameter('camera_index', 2)
        self.declare_parameter('width', 1280)     # more pixels per module
        self.declare_parameter('height', 720)
        self.declare_parameter('autofocus', False)  # AF hunting blurs the QR
        self.declare_parameter('focus', 40)         # manual focus (0-255)
        self.declare_parameter('hold_sec', 1.0)
        self.declare_parameter('show_window', True)

        idx = self.get_parameter('camera_index').value
        self.camera = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not self.camera.isOpened():
            raise RuntimeError(f'Cannot open /dev/video{idx}. Close VLC.')

        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH,
                        self.get_parameter('width').value)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT,
                        self.get_parameter('height').value)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        af = 1 if self.get_parameter('autofocus').value else 0
        self.camera.set(cv2.CAP_PROP_AUTOFOCUS, af)
        if not af:
            self.camera.set(cv2.CAP_PROP_FOCUS,
                            self.get_parameter('focus').value)

        self.tracker = QRTracker(hold_sec=self.get_parameter('hold_sec').value)
        self.show = self.get_parameter('show_window').value

        self.pub_value = self.create_publisher(String, '/qr_code', 10)
        self.pub_info = self.create_publisher(String, '/qr_info', 10)
        self.pub_center = self.create_publisher(PointStamped, '/qr_center', 10)
        # the cardboard box = gripper target
        self.pub_box = self.create_publisher(PointStamped, '/box_center', 10)
        self.pub_box_info = self.create_publisher(String, '/box_info', 10)

        self.last_value = None
        self.last_log = 0.0
        self.timer = self.create_timer(0.03, self.process_frame)

        W = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        H = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Camera /dev/video{idx} @ {W}x{H}')

    def publish(self, r):
        now = self.get_clock().now().to_msg()

        if r.value:
            m = String()
            m.data = r.value
            self.pub_value.publish(m)

        # pixel coords of the QR centre (x right, y down, origin top-left);
        # z carries the QR side length in px (a rough distance cue)
        p = PointStamped()
        p.header.stamp = now
        p.header.frame_id = 'camera'
        p.point.x, p.point.y, p.point.z = (
            float(r.center[0]), float(r.center[1]), float(r.size))
        self.pub_center.publish(p)

        info = String()
        info.data = json.dumps({
            'status': r.status,
            'value': r.value,
            'center_px': [round(float(r.center[0]), 1),
                          round(float(r.center[1]), 1)],
            'corners_px': [[round(float(x), 1), round(float(y), 1)]
                           for x, y in r.corners],   # TL, TR, BR, BL
            'size_px': round(float(r.size), 1),
            'angle_deg': round(float(r.angle), 1),
            'offset_norm': [round(float(v), 3) for v in r.offset_norm],
            'frame': list(r.frame_size),
        })
        self.pub_info.publish(info)

        if r.box_center is not None:
            # x,y = box centre px; z = box orientation (deg) for the gripper
            b = PointStamped()
            b.header.stamp = now
            b.header.frame_id = 'camera'
            b.point.x, b.point.y, b.point.z = (
                float(r.box_center[0]), float(r.box_center[1]),
                float(r.box_angle))
            self.pub_box.publish(b)

            bi = String()
            bi.data = json.dumps({
                'qr_value': r.value,
                'center_px': [round(float(r.box_center[0]), 1),
                              round(float(r.box_center[1]), 1)],
                'corners_px': [[round(float(x), 1), round(float(y), 1)]
                               for x, y in r.box_corners],  # TL,TR,BR,BL
                'size_px': [round(float(r.box_size[0]), 1),
                            round(float(r.box_size[1]), 1)],
                'grip_angle_deg': round(float(r.box_angle), 1),
                'offset_norm': [
                    round((r.box_center[0] - r.frame_size[0] / 2)
                          / (r.frame_size[0] / 2), 3),
                    round((r.box_center[1] - r.frame_size[1] / 2)
                          / (r.frame_size[1] / 2), 3)],
                'frame': list(r.frame_size),
            })
            self.pub_box_info.publish(bi)

    def draw(self, frame, r):
        if r.status == 'SEARCHING':
            cv2.putText(frame, 'Searching for QR...', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, RED, 2)
            return
        col = GREEN if r.status == 'DECODED' else YELLOW
        cv2.polylines(frame, [r.corners.astype(np.int32).reshape(-1, 1, 2)],
                      True, col, 2)
        for name, pt in zip(('TL', 'TR', 'BR', 'BL'), r.corners):
            cv2.circle(frame, (int(pt[0]), int(pt[1])), 4, col, -1)
            cv2.putText(frame, name, (int(pt[0]) + 5, int(pt[1]) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
        cx, cy = int(r.center[0]), int(r.center[1])
        cv2.drawMarker(frame, (cx, cy), col, cv2.MARKER_CROSS, 12, 1)
        if r.box_corners is not None:
            cv2.polylines(frame,
                          [r.box_corners.astype(np.int32).reshape(-1, 1, 2)],
                          True, ORANGE, 2)
            bx, by = int(r.box_center[0]), int(r.box_center[1])
            cv2.drawMarker(frame, (bx, by), ORANGE, cv2.MARKER_CROSS, 28, 2)
            cv2.putText(frame, f'BOX ({bx},{by}) {r.box_size[0]:.0f}x'
                        f'{r.box_size[1]:.0f}px grip={r.box_angle:.0f}deg',
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, ORANGE, 2)
        label = f'{r.status}: {r.value[:40]}' if r.value else \
            'TRACKING (not decoded yet)'
        cv2.putText(frame, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        cv2.putText(frame, f'center=({cx},{cy}) size={r.size:.0f}px '
                    f'angle={r.angle:.0f}deg', (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

    def process_frame(self):
        ok, frame = self.camera.read()
        if not ok:
            return

        r = self.tracker.update(frame)

        if r.status != 'SEARCHING':
            self.publish(r)
            if r.value and r.value != self.last_value:
                self.get_logger().info(f'QR VALUE: {r.value}')
                self.last_value = r.value
            t = self.get_clock().now().nanoseconds * 1e-9
            if t - self.last_log > 1.0:
                self.get_logger().info(
                    f'{r.status} qr=({r.center[0]:.0f},{r.center[1]:.0f}) '
                    + (f'box=({r.box_center[0]:.0f},{r.box_center[1]:.0f})'
                       if r.box_center is not None else 'box=n/a'))
                self.last_log = t
        else:
            self.last_value = None

        if self.show:
            self.draw(frame, r)
            cv2.imshow('Logitech QR Scanner', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.camera.isOpened():
            self.camera.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = QRDecoderNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as error:
        if str(error):
            print(error)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()