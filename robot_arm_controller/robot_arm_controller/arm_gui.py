#!/usr/bin/env python3
"""Tkinter GUI + ROS 2 node in one process."""
import queue
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_arm_controller.arm_node import JOINTS, POSITION_NAMES, RobotArmNode


class ArmGUI:
    def __init__(self, node: RobotArmNode):
        self.node = node
        self.q = queue.Queue()
        self.sliders, self.value_labels, self.position_status = {}, {}, {}
        self.updating_sliders = False

        self.root = tk.Tk()
        self.root.title("ESP32 Robot Arm - One Pick, Three Places (ROS 2)")
        self.root.geometry("850x850")
        self.root.minsize(760, 760)

        node.status_cb = lambda t, c: self.q.put(lambda: self.status_label.config(text=t, foreground=c))
        node.pose_cb = lambda p: self.q.put(lambda: self.update_sliders(p))
        node.finished_cb = lambda: self.q.put(self.on_finished)

        self.build()
        self.refresh_ports()
        self.refresh_position_status()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(50, self.poll_queue)

    def poll_queue(self):
        try:
            while True:
                self.q.get_nowait()()
        except queue.Empty:
            pass
        self.refresh_position_status()
        if self.node.is_connected():
            self.connect_button.config(state="disabled")
        self.root.after(50, self.poll_queue)

    def build(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill="both", expand=True)
        ttk.Label(main, text="6-DOF ROBOT ARM CONTROLLER (ROS 2)",
                  font=("Arial", 18, "bold")).pack(pady=(0, 10))

        conn = ttk.LabelFrame(main, text="ESP32 Connection", padding=8)
        conn.pack(fill="x", pady=4)
        self.port_combo = ttk.Combobox(conn, width=18, state="readonly")
        self.port_combo.grid(row=0, column=0, padx=5)
        ttk.Button(conn, text="Refresh Ports", command=self.refresh_ports).grid(row=0, column=1, padx=5)
        self.connect_button = ttk.Button(conn, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=2, padx=5)

        self.status_label = ttk.Label(main, text="Not connected", foreground="red")
        self.status_label.pack(pady=4)

        sf = ttk.LabelFrame(main, text="Manual Joint Control", padding=8)
        sf.pack(fill="x", pady=4)
        for row, joint in enumerate(JOINTS):
            ttk.Label(sf, text=joint, width=12).grid(row=row, column=0, sticky="w", padx=4, pady=4)
            s = ttk.Scale(sf, from_=0, to=180, orient="horizontal",
                          command=lambda v, j=joint: self.slider_changed(j, v))
            s.set(90)
            s.grid(row=row, column=1, sticky="ew", padx=5, pady=4)
            self.sliders[joint] = s
            self.value_labels[joint] = ttk.Label(sf, text="90°", width=5)
            self.value_labels[joint].grid(row=row, column=2, padx=4)
        sf.columnconfigure(1, weight=1)

        ttk.Button(main, text="RESET SLIDERS TO 90°", command=self.node.reset_pose).pack(pady=5)

        save = ttk.LabelFrame(main, text="Save Five Positions", padding=8)
        save.pack(fill="x", pady=4)
        ttk.Label(save, text="Choose position:").grid(row=0, column=0, padx=5)
        self.position_combo = ttk.Combobox(save, values=POSITION_NAMES, state="readonly", width=12)
        self.position_combo.current(0)
        self.position_combo.grid(row=0, column=1, padx=5)
        ttk.Button(save, text="SAVE CURRENT SLIDERS", command=self.save_selected).grid(row=0, column=2, padx=5)
        ttk.Button(save, text="LOAD SELECTED TO SLIDERS", command=self.load_selected).grid(row=0, column=3, padx=5)
        for row, name in enumerate(POSITION_NAMES, start=1):
            lbl = ttk.Label(save, text=f"{name}: NOT SAVED", foreground="gray")
            lbl.grid(row=row, column=0, columnspan=4, sticky="w", padx=8, pady=1)
            self.position_status[name] = lbl

        seq = ttk.LabelFrame(main, text="Autonomous Sequence", padding=8)
        seq.pack(fill="x", pady=4)
        ttk.Label(seq, justify="left", text=(
            "HOME → PICK → HOME → PLACE1 → HOME\n"
            "HOME → PICK → HOME → PLACE2 → HOME\n"
            "HOME → PICK → HOME → PLACE3 → HOME\n"
            "Then repeat from PLACE1 until STOP.")).pack(anchor="w", padx=5, pady=5)
        btns = ttk.Frame(seq)
        btns.pack(pady=5)
        self.start_button = ttk.Button(btns, text="START AUTONOMOUS", command=self.start)
        self.start_button.grid(row=0, column=0, padx=8)
        self.stop_button = ttk.Button(btns, text="STOP", command=self.stop, state="disabled")
        self.stop_button.grid(row=0, column=1, padx=8)

    def refresh_ports(self):
        ports = self.node.find_ports()
        self.port_combo["values"] = ports
        if ports:
            if self.port_combo.get() not in ports:
                self.port_combo.current(0)
        else:
            self.port_combo.set("")

    def connect(self):
        port = self.port_combo.get().strip()
        if not port:
            messagebox.showwarning("ESP32", "Select the ESP32 serial port.")
            return
        self.connect_button.config(state="disabled")

        def _c():
            if not self.node.connect(port):
                self.q.put(lambda: self.connect_button.config(state="normal"))

        threading.Thread(target=_c, daemon=True).start()

    def slider_changed(self, joint, value):
        angle = int(float(value))
        self.value_labels[joint].config(text=f"{angle}°")
        if self.updating_sliders:
            return
        self.node.set_joint(joint, angle)

    def update_sliders(self, pose):
        self.updating_sliders = True
        for j in JOINTS:
            a = int(pose[j])
            self.sliders[j].set(a)
            self.value_labels[j].config(text=f"{a}°")
        self.updating_sliders = False

    def refresh_position_status(self):
        for name in POSITION_NAMES:
            saved = name in self.node.saved_positions
            self.position_status[name].config(
                text=f"{name}: {'SAVED' if saved else 'NOT SAVED'}",
                foreground="green" if saved else "gray")

    def save_selected(self):
        if self.node.running:
            messagebox.showwarning("Busy", "Stop the autonomous sequence before saving.")
            return
        self.node.save_position(self.position_combo.get())

    def load_selected(self):
        name = self.position_combo.get()
        if name not in self.node.saved_positions:
            messagebox.showinfo("Position not saved", f"Save {name} first.")
            return
        pose = self.node.saved_positions[name].copy()
        with self.node.pose_lock:
            self.node.current_pose.update(pose)
        self.update_sliders(pose)
        if self.node.is_connected() and not self.node.running:
            for j in JOINTS:
                self.node.send_command(j, pose[j])

    def start(self):
        if not self.node.is_connected():
            messagebox.showwarning("ESP32", "Connect the ESP32 first.")
            return
        missing = [n for n in POSITION_NAMES if n not in self.node.saved_positions]
        if missing:
            messagebox.showwarning("Missing positions",
                                   "Please save all five positions first:\n" + ", ".join(missing))
            return
        if not messagebox.askyesno(
            "Start autonomous sequence",
            "Sequence:\nHOME → PICK → HOME → PLACE1 → HOME\n"
            "HOME → PICK → HOME → PLACE2 → HOME\n"
            "HOME → PICK → HOME → PLACE3 → HOME\n\n"
            "This sequence repeats until STOP is pressed.\n"
            "Ensure the arm is clear of people and obstacles."):
            return
        ok, msg = self.node.start_autonomous()
        if ok:
            self.start_button.config(state="disabled")
            self.stop_button.config(state="normal")
        else:
            messagebox.showwarning("Cannot start", msg)

    def stop(self):
        self.node.stop_autonomous()
        self.stop_button.config(state="disabled")

    def on_finished(self):
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")

    def on_close(self):
        self.node.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = RobotArmNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        ArmGUI(node).run()
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
