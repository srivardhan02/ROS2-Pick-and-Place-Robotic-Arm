#!/usr/bin/env python3
"""ESP32 6-DOF arm controller node (ROS 2). Angles are DEGREES, 0-180.

Pub  /arm/joint_states     sensor_msgs/JointState  current pose (10 Hz)
Pub  /arm/status           std_msgs/String
Pub  /arm/esp32_feedback   std_msgs/String         raw ESP32 replies
Sub  /arm/joint_command    sensor_msgs/JointState  name[] + position[]
Sub  /arm/save_position    std_msgs/String         HOME|PICK|PLACE1|PLACE2|PLACE3
Sub  /arm/goto_position    std_msgs/String
Srv  /arm/start, /arm/stop std_srvs/Trigger
"""
import json
import os
import threading

import rclpy
import serial
import serial.tools.list_ports
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

JOINTS = ["BASE", "SHOULDER", "ELBOW", "WRIST1", "WRIST2", "GRIPPER"]
POSITION_NAMES = ["HOME", "PICK", "PLACE1", "PLACE2", "PLACE3"]
DEFAULT_ANGLES = {j: 90 for j in JOINTS}


class RobotArmNode(Node):
    def __init__(self):
        super().__init__("robot_arm_controller")

        self.declare_parameter("port", "")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("save_file", "")
        self.declare_parameter("step_delay", 0.04)
        self.declare_parameter("settle_seconds", 0.4)
        self.declare_parameter("degrees_per_step", 2)

        self.baud = self.get_parameter("baud_rate").value
        self.save_file = (self.get_parameter("save_file").value
                          or os.path.expanduser("~/robot_arm_positions.json"))
        self.step_delay = self.get_parameter("step_delay").value
        self.settle = self.get_parameter("settle_seconds").value
        self.deg_step = max(1, int(self.get_parameter("degrees_per_step").value))

        self.ser = None
        self.saved_positions = {}
        self.current_pose = DEFAULT_ANGLES.copy()
        self.running = False
        self.stop_event = threading.Event()
        self.serial_lock = threading.Lock()
        self.pose_lock = threading.Lock()
        self.status_cb = None
        self.pose_cb = None
        self.finished_cb = None

        self.pose_pub = self.create_publisher(JointState, "/arm/joint_states", 10)
        self.status_pub = self.create_publisher(String, "/arm/status", 10)
        self.fb_pub = self.create_publisher(String, "/arm/esp32_feedback", 10)

        self.create_subscription(JointState, "/arm/joint_command", self.on_joint_command, 10)
        self.create_subscription(String, "/arm/save_position", self.on_save_position, 10)
        self.create_subscription(String, "/arm/goto_position", self.on_goto_position, 10)
        self.create_service(Trigger, "/arm/start", self.srv_start)
        self.create_service(Trigger, "/arm/stop", self.srv_stop)
        self.create_timer(0.1, self.publish_pose)

        self.load_positions()
        port = self.get_parameter("port").value
        if port:
            threading.Thread(target=self.connect, args=(port,), daemon=True).start()

    # ---------------- status ----------------
    def set_status(self, text, color="black"):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))
        if self.status_cb:
            self.status_cb(text, color)

    # ---------------- serial ----------------
    @staticmethod
    def find_ports():
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port):
        try:
            with self.serial_lock:
                if self.ser is not None and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(port, self.baud, timeout=1)
            self.stop_event.wait(2.0)  # ESP32 resets when the port opens
            threading.Thread(target=self._reader_loop, daemon=True).start()
            self.set_status(f"Connected: {port}", "green")
            return True
        except Exception as exc:
            self.set_status(f"Serial connection error: {exc}", "red")
            return False

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def _reader_loop(self):
        while rclpy.ok() and self.is_connected():
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self.fb_pub.publish(String(data=line))
            except Exception:
                break

    def send_command(self, joint, angle):
        try:
            with self.serial_lock:
                if not self.is_connected():
                    return False
                self.ser.write(f"{joint}:{int(angle)}\n".encode("utf-8"))
            return True
        except Exception as exc:
            self.get_logger().error(f"Serial error: {exc}")
            return False

    # ---------------- pose ----------------
    def set_joint(self, joint, angle):
        angle = max(0, min(180, int(angle)))
        with self.pose_lock:
            self.current_pose[joint] = angle
        if self.is_connected() and not self.running:
            self.send_command(joint, angle)

    def publish_pose(self):
        with self.pose_lock:
            pose = self.current_pose.copy()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = [float(pose[j]) for j in JOINTS]
        self.pose_pub.publish(msg)

    def reset_pose(self):
        if self.running:
            return
        with self.pose_lock:
            self.current_pose.update(DEFAULT_ANGLES)
        if self.pose_cb:
            self.pose_cb(DEFAULT_ANGLES.copy())
        for j in JOINTS:
            self.send_command(j, DEFAULT_ANGLES[j])

    # ---------------- save / load ----------------
    def save_positions_to_file(self):
        try:
            with open(self.save_file, "w", encoding="utf-8") as f:
                json.dump(self.saved_positions, f, indent=2)
        except Exception as exc:
            self.set_status(f"Save error: {exc}", "red")

    def load_positions(self):
        if not os.path.exists(self.save_file):
            return
        try:
            with open(self.save_file, "r", encoding="utf-8") as f:
                data = json.load(f)
            valid = {}
            for name in POSITION_NAMES:
                pose = data.get(name)
                if isinstance(pose, dict) and all(
                    j in pose and isinstance(pose[j], (int, float)) and 0 <= pose[j] <= 180
                    for j in JOINTS
                ):
                    valid[name] = {j: int(pose[j]) for j in JOINTS}
            self.saved_positions = valid
        except Exception as exc:
            self.get_logger().warn(f"Could not load saved poses: {exc}")

    def save_position(self, name):
        if self.running:
            self.set_status("Stop the sequence before saving", "red")
            return False
        if name not in POSITION_NAMES:
            self.set_status(f"Unknown position '{name}'", "red")
            return False
        with self.pose_lock:
            self.saved_positions[name] = self.current_pose.copy()
        self.save_positions_to_file()
        self.set_status(f"Saved {name}", "green")
        return True

    # ---------------- motion ----------------
    def move_to_pose(self, target, settle=None):
        settle = self.settle if settle is None else settle
        with self.pose_lock:
            start = self.current_pose.copy()

        largest = max(abs(int(target[j]) - int(start[j])) for j in JOINTS)
        steps = max(1, int(largest / self.deg_step) + 1)

        for step in range(1, steps + 1):
            if self.stop_event.is_set():
                return False
            pose = {j: round(start[j] + (int(target[j]) - start[j]) * step / steps)
                    for j in JOINTS}
            for j in JOINTS:
                if self.stop_event.is_set():
                    return False
                if not self.send_command(j, pose[j]):
                    self.set_status("Serial connection lost", "red")
                    return False
            with self.pose_lock:
                self.current_pose = pose.copy()
            if self.pose_cb:
                self.pose_cb(pose.copy())
            if self.stop_event.wait(self.step_delay):
                return False
        return not self.stop_event.wait(settle)

    @staticmethod
    def pose_with_gripper(pose, gripper_angle):
        result = pose.copy()
        result["GRIPPER"] = int(gripper_angle)
        return result

    def autonomous_worker(self):
        try:
            while not self.stop_event.is_set():
                home = self.saved_positions["HOME"].copy()
                pick = self.saved_positions["PICK"].copy()
                places = [(n, self.saved_positions[n].copy())
                          for n in ("PLACE1", "PLACE2", "PLACE3")]
                grip_closed = pick["GRIPPER"]

                for place_name, place_pose in places:
                    if self.stop_event.is_set():
                        break
                    sequence = [
                        ("HOME", home),
                        ("PICK", pick),
                        ("HOME (holding object)", self.pose_with_gripper(home, grip_closed)),
                        (place_name, place_pose),
                        ("HOME", home),
                    ]
                    for step_name, target in sequence:
                        if self.stop_event.is_set():
                            break
                        self.set_status(f"Moving to: {step_name}", "blue")
                        if not self.move_to_pose(target):
                            break
        except Exception as exc:
            self.set_status(f"Sequence error: {exc}", "red")
        finally:
            self.running = False
            if self.stop_event.is_set():
                self.set_status("Sequence stopped", "red")
            else:
                self.set_status("Sequence completed", "green")
            if self.finished_cb:
                self.finished_cb()

    def start_autonomous(self):
        if self.running:
            return False, "Already running"
        if not self.is_connected():
            return False, "ESP32 not connected"
        missing = [n for n in POSITION_NAMES if n not in self.saved_positions]
        if missing:
            return False, "Missing positions: " + ", ".join(missing)
        self.running = True
        self.stop_event.clear()
        threading.Thread(target=self.autonomous_worker, daemon=True).start()
        return True, "Sequence started"

    def stop_autonomous(self):
        self.stop_event.set()
        self.set_status("Stopping sequence...", "red")

    def goto_position(self, name):
        if self.running:
            self.set_status("Busy: sequence running", "red")
            return
        if name not in self.saved_positions:
            self.set_status(f"{name} not saved", "red")
            return
        pose = self.saved_positions[name].copy()

        def _go():
            self.running = True
            self.stop_event.clear()
            self.move_to_pose(pose, settle=0.0)
            self.running = False
            if self.finished_cb:
                self.finished_cb()

        threading.Thread(target=_go, daemon=True).start()

    # ---------------- ros callbacks ----------------
    def on_joint_command(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name.upper() in JOINTS:
                self.set_joint(name.upper(), pos)
        if self.pose_cb:
            with self.pose_lock:
                self.pose_cb(self.current_pose.copy())

    def on_save_position(self, msg: String):
        self.save_position(msg.data.strip().upper())

    def on_goto_position(self, msg: String):
        self.goto_position(msg.data.strip().upper())

    def srv_start(self, request, response):
        response.success, response.message = self.start_autonomous()
        return response

    def srv_stop(self, request, response):
        self.stop_autonomous()
        response.success, response.message = True, "Stop requested"
        return response

    def shutdown(self):
        self.stop_event.set()
        try:
            if self.is_connected():
                self.ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
