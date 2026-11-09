#!/usr/bin/env python3

import json
import os
import threading
import time
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


JOINT_NAMES = [
    "BASE",
    "SHOULDER",
    "ELBOW",
    "WRIST1",
    "WRIST2",
    "GRIPPER",
]

WAYPOINT_NAMES = ["PICK", "GRIP", "PLACE1", "PLACE2"]

POSE_FILE = os.path.expanduser("~/pick_place_waypoints.json")

DEFAULT_ANGLES = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]

# Gripper angles are NOT fixed any more: the gripper value stored in PICK is the
# "open" angle, the one stored in GRIP is the "closed on the box" angle.

# Movement settings
ANGLE_STEP = 2.0
STEP_DELAY = 0.06
SETTLE_TIME = 0.5


class WaypointNode(Node):
    def __init__(self):
        super().__init__("pick_place_waypoint_node")

        self.publisher = self.create_publisher(
            JointState,
            "/arm/joint_command",
            10,
        )

        self.lock = threading.Lock()
        self.current_angles = DEFAULT_ANGLES.copy()

        # Receive the latest joint commands from RViz and this GUI.
        self.subscription = self.create_subscription(
            JointState,
            "/arm/joint_command",
            self.command_callback,
            10,
        )

        # Latest box seen by qr_decoder_node (used to record the grip view).
        self.box = None
        self.create_subscription(String, "/box_info", self.box_callback, 10)

        self.get_logger().info(
            "Waypoint node ready. Listening on /arm/joint_command"
        )

    def box_callback(self, msg):
        try:
            data = json.loads(msg.data)
            cx, cy = data["center_px"]
            a, b = data["size_px"]
            self.box = {
                "t": time.time(),
                "cx": float(cx),
                "cy": float(cy),
                "size": 0.5 * (float(a) + float(b)),
            }
        except Exception:
            pass

    def get_box(self, max_age=1.0):
        box = self.box
        if box is None or time.time() - box["t"] > max_age:
            return None
        return {k: box[k] for k in ("cx", "cy", "size")}

    def command_callback(self, msg):
        if not msg.name or not msg.position:
            return

        incoming = dict(zip(msg.name, msg.position))

        with self.lock:
            for i, name in enumerate(JOINT_NAMES):
                if name in incoming:
                    self.current_angles[i] = max(
                        0.0,
                        min(180.0, float(incoming[name])),
                    )

    def get_angles(self):
        with self.lock:
            return self.current_angles.copy()

    def publish_angles(self, angles):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES.copy()
        msg.position = [
            max(0.0, min(180.0, float(angle)))
            for angle in angles
        ]
        self.publisher.publish(msg)


class WaypointGUI:
    def __init__(self, node):
        self.node = node

        self.root = tk.Tk()
        self.root.title("ROS 2 Pick and Place Waypoints")
        self.root.geometry("680x600")
        self.root.minsize(560, 520)

        self.poses = {
            name: None for name in WAYPOINT_NAMES
        }
        self.grip_view = None

        self.running = False
        self.stop_event = threading.Event()

        self.load_poses()
        self.build_gui()
        self.refresh_status()

        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def build_gui(self):
        title = tk.Label(
            self.root,
            text="PICK AND PLACE WAYPOINT CONTROLLER",
            font=("Arial", 15, "bold"),
        )
        title.pack(pady=(15, 5))

        description = tk.Label(
            self.root,
            text=(
                "PICK = box visible to camera (gripper open)  |  "
                "GRIP = gripper around the box (gripper closed)\n"
                "PLACE1 / PLACE2 = drop spots. Set each pose with the "
                "RViz sliders, then save."
            ),
            font=("Arial", 10),
        )
        description.pack(pady=(0, 12))

        # Current joint angles
        current_frame = tk.LabelFrame(
            self.root,
            text="Current Joint Angles (degrees)",
            padx=10,
            pady=8,
        )
        current_frame.pack(fill="x", padx=15, pady=5)

        self.current_label = tk.Label(
            current_frame,
            text="Waiting for joint commands...",
            font=("Courier", 10),
            justify="left",
        )
        self.current_label.pack(anchor="w")

        # Four independent waypoint save buttons
        save_frame = tk.LabelFrame(
            self.root,
            text="Save Waypoints",
            padx=10,
            pady=10,
        )
        save_frame.pack(fill="x", padx=15, pady=8)

        self.pose_labels = {}

        for name in WAYPOINT_NAMES:
            row = tk.Frame(save_frame)
            row.pack(fill="x", pady=4)

            button = tk.Button(
                row,
                text=f"Save {name}",
                width=16,
                command=lambda waypoint=name: self.save_waypoint(
                    waypoint
                ),
            )
            button.pack(side="left")

            status = tk.Label(
                row,
                text="Not saved",
                anchor="w",
                font=("Arial", 10),
            )
            status.pack(side="left", padx=12)

            self.pose_labels[name] = status

        # Sequence controls
        control_frame = tk.LabelFrame(
            self.root,
            text="Sequence Controls",
            padx=10,
            pady=10,
        )
        control_frame.pack(fill="x", padx=15, pady=8)

        self.start_button = tk.Button(
            control_frame,
            text="START PICK AND PLACE",
            height=2,
            command=self.start_sequence,
        )
        self.start_button.pack(fill="x", pady=4)

        self.stop_button = tk.Button(
            control_frame,
            text="STOP",
            height=2,
            state="disabled",
            command=self.stop_sequence,
        )
        self.stop_button.pack(fill="x", pady=4)

        # Sequence status
        self.sequence_status = tk.Label(
            self.root,
            text="Status: Ready",
            font=("Arial", 10, "bold"),
            wraplength=560,
        )
        self.sequence_status.pack(pady=8)

        note = tk.Label(
            self.root,
            text=(
                "Saved poses are stored in "
                "~/pick_place_waypoints.json"
            ),
            font=("Arial", 9),
        )
        note.pack(pady=(0, 10))

    def refresh_status(self):
        angles = self.node.get_angles()

        self.current_label.config(
            text="\n".join(
                f"{name:10s}: {angle:6.1f}°"
                for name, angle in zip(JOINT_NAMES, angles)
            )
        )

        for name in WAYPOINT_NAMES:
            pose = self.poses.get(name)

            if pose is None:
                self.pose_labels[name].config(
                    text="Not saved"
                )
            else:
                self.pose_labels[name].config(
                    text="Saved: " + ", ".join(
                        f"{value:.0f}" for value in pose
                    ) + "°"
                )

        self.root.after(200, self.refresh_status)

    def save_waypoint(self, name):
        # Saving one pose must not disable any other Save button.
        if self.running:
            messagebox.showinfo(
                "Sequence running",
                "Stop the sequence before changing saved poses.",
                parent=self.root,
            )
            return

        angles = self.node.get_angles()

        self.poses[name] = angles.copy()

        try:
            self.save_poses()
        except Exception as exc:
            messagebox.showerror(
                "Save error",
                f"Could not save {name}:\n{exc}",
                parent=self.root,
            )
            return

        extra = ""
        if name == "GRIP":
            view = self.node.get_box()
            if view is not None:
                self.grip_view = view
                extra = " (box grip view recorded too)"
                try:
                    self.save_poses()
                except Exception:
                    pass
            else:
                extra = " (box not in camera view - grip view not recorded)"

        self.sequence_status.config(
            text=f"Status: {name} saved successfully{extra}"
        )

        self.node.get_logger().info(
            f"{name} saved: {angles}"
        )

    def save_poses(self):
        directory = os.path.dirname(POSE_FILE)
        os.makedirs(directory, exist_ok=True)

        data = {
            name: self.poses[name]
            for name in WAYPOINT_NAMES
            if self.poses[name] is not None
        }

        temporary_file = POSE_FILE + ".tmp"

        if self.grip_view is not None:
            data["GRIP_VIEW"] = self.grip_view

        with open(temporary_file, "w", encoding="utf-8") as file:
            json.dump(data, file, indent=2)

        os.replace(temporary_file, POSE_FILE)

    def load_poses(self):
        if not os.path.exists(POSE_FILE):
            return

        try:
            with open(POSE_FILE, "r", encoding="utf-8") as file:
                data = json.load(file)

            view = data.get("GRIP_VIEW")
            if isinstance(view, dict):
                self.grip_view = view

            for name in WAYPOINT_NAMES:
                pose = data.get(name)

                if (
                    isinstance(pose, list)
                    and len(pose) == len(JOINT_NAMES)
                ):
                    self.poses[name] = [
                        max(0.0, min(180.0, float(value)))
                        for value in pose
                    ]

        except (OSError, ValueError, TypeError) as exc:
            print(f"Could not load saved poses: {exc}")

    def start_sequence(self):
        if self.running:
            return

        missing = [
            name for name in WAYPOINT_NAMES
            if self.poses[name] is None
        ]

        if missing:
            messagebox.showwarning(
                "Missing waypoints",
                "Please save all four poses first:\n\n"
                + "\n".join(missing),
                parent=self.root,
            )
            return

        confirm = messagebox.askyesno(
            "Start sequence",
            "Run PICK -> GRIP -> PLACE1, then PICK -> GRIP -> PLACE2?\n\n"
            "Make sure the workspace is clear and the arm "
            "is safe to move.",
            parent=self.root,
        )

        if not confirm:
            return

        self.running = True
        self.stop_event.clear()

        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")

        self.sequence_status.config(
            text="Status: Starting sequence..."
        )

        threading.Thread(
            target=self.run_sequence,
            daemon=True,
        ).start()

    def stop_sequence(self):
        self.stop_event.set()
        self.sequence_status.config(
            text="Status: Stop requested..."
        )

    def set_status(self, text):
        self.root.after(
            0,
            lambda: self.sequence_status.config(
                text=f"Status: {text}"
            ),
        )

    def move_to(self, target):
        target = [
            max(0.0, min(180.0, float(value)))
            for value in target
        ]

        start = self.node.get_angles()
        largest_difference = max(
            abs(target[i] - start[i])
            for i in range(len(JOINT_NAMES))
        )

        steps = max(
            1,
            int(largest_difference / ANGLE_STEP) + 1,
        )

        for step in range(1, steps + 1):
            if self.stop_event.is_set():
                return False

            fraction = step / steps

            intermediate = [
                start[i] + (target[i] - start[i]) * fraction
                for i in range(len(JOINT_NAMES))
            ]

            self.node.publish_angles(intermediate)

            if self.stop_event.wait(STEP_DELAY):
                return False

        if self.stop_event.wait(SETTLE_TIME):
            return False

        return True

    def pose_with_gripper(self, name, gripper_angle):
        pose = self.poses[name].copy()
        pose[5] = gripper_angle
        return pose

    def run_sequence(self):
        try:
            # Gripper angles come from the saved poses.
            open_angle = self.poses["PICK"][5]
            closed_angle = self.poses["GRIP"][5]

            for place_name in ["PLACE1", "PLACE2"]:
                self.set_status("Moving to PICK (gripper open)")
                if not self.move_to(
                    self.pose_with_gripper("PICK", open_angle)
                ):
                    return

                self.set_status("Moving to GRIP position")
                if not self.move_to(
                    self.pose_with_gripper("GRIP", open_angle)
                ):
                    return

                self.set_status("Closing gripper on the box")
                if not self.move_to(
                    self.pose_with_gripper("GRIP", closed_angle)
                ):
                    return

                self.set_status("Lifting to PICK")
                if not self.move_to(
                    self.pose_with_gripper("PICK", closed_angle)
                ):
                    return

                self.set_status(f"Moving to {place_name}")
                if not self.move_to(
                    self.pose_with_gripper(place_name, closed_angle)
                ):
                    return

                self.set_status(f"Opening gripper at {place_name}")
                if not self.move_to(
                    self.pose_with_gripper(place_name, open_angle)
                ):
                    return

            self.set_status("Sequence completed")

        except Exception as exc:
            self.node.get_logger().error(
                f"Sequence failed: {exc}"
            )
            self.set_status(f"Error: {exc}")

        finally:
            self.root.after(0, self.finish_sequence)

    def finish_sequence(self):
        self.running = False
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")

        if self.stop_event.is_set():
            self.sequence_status.config(
                text="Status: Sequence stopped"
            )

    def close(self):
        self.stop_event.set()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)

    node = WaypointNode()

    executor_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    executor_thread.start()

    gui = WaypointGUI(node)

    try:
        gui.root.mainloop()
    finally:
        gui.stop_event.set()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
