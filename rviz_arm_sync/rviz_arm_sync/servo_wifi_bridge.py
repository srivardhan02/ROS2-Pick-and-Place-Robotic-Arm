#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class ServoWifiBridge(Node):
    def __init__(self):
        super().__init__('servo_wifi_bridge')

        self.joint_order = [
            'BASE',
            'SHOULDER',
            'ELBOW',
            'WRIST1',
            'WRIST2',
            'GRIPPER',
        ]

        # Safe software defaults only; no physical servo movement is intended.
        self.angles = {name: 90.0 for name in self.joint_order}

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/servo_angles',
            10
        )

        self.subscription = self.create_subscription(
            JointState,
            '/arm/joint_command',
            self.command_callback,
            10
        )

        self.get_logger().info(
            'Wi-Fi bridge ready: /arm/joint_command -> /servo_angles'
        )
        self.get_logger().info(
            'Servo order: BASE, SHOULDER, ELBOW, WRIST1, WRIST2, GRIPPER'
        )

    def command_callback(self, msg):
        updated = False

        for name, position in zip(msg.name, msg.position):
            joint = name.upper()

            if joint in self.angles:
                self.angles[joint] = max(0.0, min(180.0, float(position)))
                updated = True

        if not updated:
            return

        output = Float32MultiArray()
        output.data = [float(self.angles[name]) for name in self.joint_order]
        self.publisher.publish(output)

        self.get_logger().info(
            'Published servo angles: ' +
            ', '.join(
                f'{name}={self.angles[name]:.0f}'
                for name in self.joint_order
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = ServoWifiBridge()

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
