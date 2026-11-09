#!/usr/bin/env python3
"""Mirror RViz sliders onto the real arm.

Sub  /joint_states        sensor_msgs/JointState  (radians, URDF joint names)
Pub  /arm/joint_command   sensor_msgs/JointState  (degrees, BASE..GRIPPER)

/arm/joint_command is consumed by the existing robot_arm_controller/arm_node,
which sends the serial commands to the ESP32. Only joints whose servo angle
changed are published, so the serial link is not spammed.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class RvizToArm(Node):
    def __init__(self):
        super().__init__('rviz_to_arm')

        self.declare_parameter('urdf_joints', ['waist_joint', 'shoulder_joint', 'elbow_joint',
                                               'wrist_roll_joint', 'wrist_pitch_joint'])
        self.declare_parameter('arm_joints', ['BASE', 'SHOULDER', 'ELBOW', 'WRIST1', 'WRIST2'])
        self.declare_parameter('directions', [1.0, -1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('centers', [90.0] * 5)
        self.declare_parameter('gripper_urdf_joint', 'gripper_joint')
        self.declare_parameter('gripper_arm_joint', 'GRIPPER')
        self.declare_parameter('gripper_urdf_closed', -0.55)
        self.declare_parameter('gripper_urdf_open', 0.80)
        self.declare_parameter('gripper_servo_closed', 40.0)
        self.declare_parameter('gripper_servo_open', 140.0)

        gp = self.get_parameter
        self.urdf = list(gp('urdf_joints').value)
        self.arm = list(gp('arm_joints').value)
        self.dirs = list(gp('directions').value)
        self.centers = list(gp('centers').value)
        self.g_urdf = gp('gripper_urdf_joint').value
        self.g_arm = gp('gripper_arm_joint').value
        self.g_uc = gp('gripper_urdf_closed').value
        self.g_uo = gp('gripper_urdf_open').value
        self.g_sc = gp('gripper_servo_closed').value
        self.g_so = gp('gripper_servo_open').value

        if not (len(self.urdf) == len(self.arm) == len(self.dirs) == len(self.centers)):
            raise ValueError('urdf_joints, arm_joints, directions, centers must have equal length')

        self.rad = {n: 0.0 for n in self.urdf + [self.g_urdf]}
        self.last_sent = {}   # arm joint name -> last int angle sent

        self.pub = self.create_publisher(JointState, '/arm/joint_command', 10)
        self.create_subscription(JointState, '/joint_states', self.cb, 10)
        self.get_logger().info('rviz_to_arm: /joint_states -> /arm/joint_command')

    def cb(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            if n in self.rad:
                self.rad[n] = p

        targets = {}
        for u, a, d, c in zip(self.urdf, self.arm, self.dirs, self.centers):
            targets[a] = int(round(clamp(c + d * math.degrees(self.rad[u]), 0.0, 180.0)))

        t = clamp((self.rad[self.g_urdf] - self.g_uc) / (self.g_uo - self.g_uc), 0.0, 1.0)
        targets[self.g_arm] = int(round(self.g_sc + t * (self.g_so - self.g_sc)))

        changed = {k: v for k, v in targets.items() if self.last_sent.get(k) != v}
        if not changed:
            return

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(changed.keys())
        out.position = [float(v) for v in changed.values()]
        self.pub.publish(out)
        self.last_sent.update(changed)


def main(args=None):
    rclpy.init(args=args)
    node = RvizToArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
