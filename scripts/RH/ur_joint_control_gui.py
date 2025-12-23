#!/usr/bin/env python3
"""
Simple GUI for controlling UR robot joints in real-time using sliders.
"""
import tkinter as tk
from tkinter import ttk
import rtde_control
import rtde_receive
import threading
import time
import math


ROBOT_IP = "192.168.2.2"

# Servo control parameters
SERVO_TIME = 0.008  # 8ms = 125Hz
LOOKAHEAD_TIME = 0.2
SERVO_GAIN = 100.0
MAX_JOINT_SPEED = 0.5  # Maximum joint speed in rad/s (default)

# Joint limits in degrees (converted to radians internally)
JOINT_LIMITS_DEG = [
    (-360, 360),  # Joint 0
    (-360, 360),  # Joint 1
    (-360, 360),  # Joint 2
    (-360, 360),  # Joint 3
    (-360, 360),  # Joint 4
    (-360, 360),  # Joint 5
]


class URJointController:
    def __init__(self, master):
        self.master = master
        master.title("UR Robot Joint Controller")
        master.geometry("800x600")

        self.robot_ip = ROBOT_IP
        self.rtde_c = None
        self.rtde_r = None
        self.control_active = False
        self.control_thread = None

        # Target joint positions (in radians)
        self.target_joints = [0.0] * 6
        self.current_joints = [0.0] * 6
        self.smoothed_targets = [0.0] * 6  # Smoothed targets with speed limiting
        self.max_speed = MAX_JOINT_SPEED  # rad/s

        # Create GUI
        self.create_widgets()

    def create_widgets(self):
        # Connection frame
        conn_frame = ttk.LabelFrame(self.master, text="Connection", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text=f"Robot IP: {self.robot_ip}").pack(side="left", padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.connect)
        self.connect_btn.pack(side="left", padx=5)

        self.disconnect_btn = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_btn.pack(side="left", padx=5)

        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.pack(side="left", padx=20)

        # Control frame
        ctrl_frame = ttk.LabelFrame(self.master, text="Control", padding=10)
        ctrl_frame.pack(fill="x", padx=10, pady=5)

        self.start_btn = ttk.Button(ctrl_frame, text="Start Control", command=self.start_control, state="disabled")
        self.start_btn.pack(side="left", padx=5)

        self.stop_btn = ttk.Button(ctrl_frame, text="Stop Control", command=self.stop_control, state="disabled")
        self.stop_btn.pack(side="left", padx=5)

        self.read_current_btn = ttk.Button(ctrl_frame, text="Read Current Position",
                                           command=self.read_current_position, state="disabled")
        self.read_current_btn.pack(side="left", padx=5)

        # Speed control
        ttk.Label(ctrl_frame, text="Max Speed:").pack(side="left", padx=(20, 5))
        self.speed_scale = tk.Scale(ctrl_frame, from_=0.05, to=2.0, orient="horizontal",
                                    resolution=0.05, length=200,
                                    command=self.on_speed_change)
        self.speed_scale.set(self.max_speed)
        self.speed_scale.pack(side="left", padx=5)
        self.speed_label = ttk.Label(ctrl_frame, text=f"{self.max_speed:.2f} rad/s")
        self.speed_label.pack(side="left", padx=5)

        # Sliders frame
        sliders_frame = ttk.LabelFrame(self.master, text="Joint Positions", padding=10)
        sliders_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.sliders = []
        self.current_labels = []
        self.target_labels = []

        for i in range(6):
            # Joint frame
            joint_frame = ttk.Frame(sliders_frame)
            joint_frame.pack(fill="x", pady=5)

            # Joint label
            ttk.Label(joint_frame, text=f"Joint {i}:", width=8).pack(side="left", padx=5)

            # Current position label
            current_label = ttk.Label(joint_frame, text="0.0°", width=10, foreground="blue")
            current_label.pack(side="left", padx=5)
            self.current_labels.append(current_label)

            # Slider
            min_deg, max_deg = JOINT_LIMITS_DEG[i]
            slider = tk.Scale(joint_frame, from_=min_deg, to=max_deg, orient="horizontal",
                            length=400, command=lambda val, idx=i: self.on_slider_change(idx, val))
            slider.set(0)
            slider.pack(side="left", padx=5)
            self.sliders.append(slider)

            # Target position label
            target_label = ttk.Label(joint_frame, text="→ 0.0°", width=12, foreground="green")
            target_label.pack(side="left", padx=5)
            self.target_labels.append(target_label)

        # Info frame
        info_frame = ttk.LabelFrame(self.master, text="Info", padding=10)
        info_frame.pack(fill="x", padx=10, pady=5)

        info_text = """
        Instructions:
        1. Click 'Connect' to connect to the robot
        2. Click 'Read Current Position' to set sliders to current joint positions
        3. Click 'Start Control' to enable real-time control
        4. Move sliders to control joint positions
        5. Click 'Stop Control' when done

        Blue = Current Position | Green = Target Position
        """
        ttk.Label(info_frame, text=info_text, justify="left").pack()

    def connect(self):
        try:
            self.status_label.config(text="Status: Connecting...", foreground="orange")
            self.master.update()

            self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)

            self.status_label.config(text="Status: Connected", foreground="green")
            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")
            self.start_btn.config(state="normal")
            self.read_current_btn.config(state="normal")

            # Read initial position
            self.read_current_position()

        except Exception as e:
            self.status_label.config(text=f"Status: Error - {e}", foreground="red")

    def disconnect(self):
        self.stop_control()

        if self.rtde_c:
            self.rtde_c.disconnect()
        if self.rtde_r:
            self.rtde_r.disconnect()

        self.status_label.config(text="Status: Disconnected", foreground="red")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="disabled")
        self.read_current_btn.config(state="disabled")

    def read_current_position(self):
        if not self.rtde_r:
            return

        try:
            self.current_joints = self.rtde_r.getActualQ()

            # Update sliders and target to current position
            for i, pos_rad in enumerate(self.current_joints):
                pos_deg = math.degrees(pos_rad)
                self.sliders[i].set(pos_deg)
                self.target_joints[i] = pos_rad
                self.smoothed_targets[i] = pos_rad  # Initialize smoothed targets
                self.current_labels[i].config(text=f"{pos_deg:.1f}°")
                self.target_labels[i].config(text=f"→ {pos_deg:.1f}°")

        except Exception as e:
            print(f"Error reading position: {e}")

    def on_slider_change(self, joint_idx, value):
        # Convert degrees to radians
        value_rad = math.radians(float(value))
        self.target_joints[joint_idx] = value_rad
        self.target_labels[joint_idx].config(text=f"→ {float(value):.1f}°")

    def on_speed_change(self, value):
        self.max_speed = float(value)
        self.speed_label.config(text=f"{self.max_speed:.2f} rad/s")

    def control_loop(self):
        """Real-time control loop running in separate thread."""
        while self.control_active:
            try:
                # Read current position
                self.current_joints = self.rtde_r.getActualQ()

                # Update current position labels
                for i, pos_rad in enumerate(self.current_joints):
                    pos_deg = math.degrees(pos_rad)
                    self.current_labels[i].config(text=f"{pos_deg:.1f}°")

                # Apply speed limiting to smooth targets
                for i in range(6):
                    delta = self.target_joints[i] - self.smoothed_targets[i]
                    max_delta = self.max_speed * SERVO_TIME  # Maximum change per time step

                    # Clamp the delta to max speed
                    if abs(delta) > max_delta:
                        delta = math.copysign(max_delta, delta)

                    self.smoothed_targets[i] += delta

                # Send servo command with smoothed targets
                t_start = self.rtde_c.initPeriod()
                self.rtde_c.servoJ(
                    self.smoothed_targets,
                    0,  # velocity (not used)
                    0,  # acceleration (not used)
                    SERVO_TIME,
                    LOOKAHEAD_TIME,
                    SERVO_GAIN
                )
                self.rtde_c.waitPeriod(t_start)

            except Exception as e:
                print(f"Control loop error: {e}")
                self.control_active = False
                break

    def start_control(self):
        if not self.rtde_c or not self.rtde_r:
            return

        self.control_active = True
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.status_label.config(text="Status: Control Active", foreground="green")

    def stop_control(self):
        self.control_active = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)

        if self.rtde_c:
            try:
                self.rtde_c.servoStop()
            except:
                pass

        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.status_label.config(text="Status: Connected", foreground="green")

    def on_closing(self):
        self.disconnect()
        self.master.destroy()


def main():
    root = tk.Tk()
    app = URJointController(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
