"""
Teleoperation Monitoring GUI

Displays real-time data for dual UR5e teleoperation:
- Hand wrist tracking (both hands)
- Headset pose
- Robot joint angles
- Debug logging

Usage:
    Integrated with teleop_dual_ur5e_mujoco.py
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import time
import numpy as np
from datetime import datetime

from xrobotoolkit_teleop.common.xr_client import XrClient


class TeleopMonitorGUI:
    def __init__(self, controller=None, start_monitoring=True, rtde_receiver=None):
        """
        Initialize the monitoring GUI.

        Args:
            controller: Reference to MujocoTeleopController instance for joint data
            start_monitoring: Whether to start monitoring thread automatically (default: True)
            rtde_receiver: RTDE receiver interface for right robot (optional)
        """
        print("[GUI] Initializing monitoring GUI...")
        self.controller = controller
        self.rtde_receiver = rtde_receiver

        print("[GUI] Getting XrClient from controller...")
        # Reuse the controller's XrClient instead of creating a new one
        # (XRoboToolkit SDK doesn't support multiple instances)
        if controller is not None:
            self.xr_client = controller.xr_client
        else:
            print("[GUI] No controller provided, creating new XrClient...")
            self.xr_client = XrClient()

        # Data storage
        self.hand_data = {'left': {}, 'right': {}}
        self.headset_data = {}
        self.joint_data = {'left_arm': np.zeros(6), 'right_arm': np.zeros(6)}
        self.tcp_data = {'left': {}, 'right': {}}

        # Monitoring state
        self.monitoring = False
        self.monitor_thread = None
        self.update_rate_hz = 10
        self.last_update_time = time.time()
        self.actual_update_rate = 0.0
        self.update_counter = 0

        # Hand tracking control state
        self.hand_tracking_enabled = True

        # Create GUI
        print("[GUI] Creating Tk window...")
        self.root = tk.Tk()
        self.root.title("Teleoperation Monitor")
        self.root.geometry("2000x2000")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        print("[GUI] Setting up GUI layout...")
        self.setup_gui()

        # Start monitoring if requested
        if start_monitoring:
            print("[GUI] Starting monitoring thread...")
            self.start_monitoring()

        print("[GUI] Initialization complete!")

    def setup_gui(self):
        """Set up the GUI layout with all frames and widgets."""
        # Control panel at top
        self.create_control_panel()

        # Hand tracking data
        self.create_hand_tracking_panel()

        # Headset tracking
        self.create_headset_panel()

        # Joint angles
        self.create_joint_angles_panel()

        # TCP positions and orientations
        self.create_tcp_position_panel()

        # Debug log at bottom
        self.create_debug_log_panel()

    def create_control_panel(self):
        """Create control panel with clear button and status."""
        frame = ttk.LabelFrame(self.root, text="Control Panel", padding=10)
        frame.pack(fill="x", padx=10, pady=5)

        # Hand tracking toggle button
        self.hand_tracking_btn = ttk.Button(
            frame,
            text="Disable Hand Tracking",
            command=self.toggle_hand_tracking,
            width=20
        )
        self.hand_tracking_btn.pack(side="left", padx=5)

        # Hand tracking status indicator
        self.hand_tracking_status = ttk.Label(
            frame,
            text="● ENABLED",
            foreground="green",
            font=("Arial", 9, "bold")
        )
        self.hand_tracking_status.pack(side="left", padx=5)

        # Get Arm Pose button (only show if RTDE is available)
        if self.rtde_receiver is not None:
            self.get_arm_pose_btn = ttk.Button(
                frame,
                text="Get Arm Pose",
                command=self.get_arm_pose_from_robot,
                width=15
            )
            self.get_arm_pose_btn.pack(side="left", padx=20)

        # Clear logs button
        self.clear_logs_btn = ttk.Button(frame, text="Clear Logs", command=self.clear_logs)
        self.clear_logs_btn.pack(side="left", padx=20)

        # Status label
        self.status_label = ttk.Label(frame, text="Status: ● MONITORING    Update Rate: 0.0 Hz",
                                     font=("Arial", 10))
        self.status_label.pack(side="left", padx=20)

    def create_hand_tracking_panel(self):
        """Create panel for hand tracking data."""
        frame = ttk.LabelFrame(self.root, text="Hand Tracking Data", padding=10)
        frame.pack(fill="both", expand=False, padx=10, pady=5)

        # Create two columns for left and right hands
        left_frame = ttk.Frame(frame)
        left_frame.pack(side="left", fill="both", expand=True, padx=5)

        right_frame = ttk.Frame(frame)
        right_frame.pack(side="left", fill="both", expand=True, padx=5)

        # Left hand
        self.left_hand_widgets = self.create_hand_widgets(left_frame, "Left Hand")

        # Right hand
        self.right_hand_widgets = self.create_hand_widgets(right_frame, "Right Hand")

    def create_hand_widgets(self, parent, title):
        """Create widgets for displaying a single hand's data."""
        widgets = {}

        # Title
        title_label = ttk.Label(parent, text=title, font=("Arial", 11, "bold"))
        title_label.pack(anchor="w")

        # Status
        widgets['status'] = ttk.Label(parent, text="Status: ○ INACTIVE", foreground="red")
        widgets['status'].pack(anchor="w", pady=2)

        # Position
        pos_label = ttk.Label(parent, text="Position:", font=("Arial", 9, "bold"))
        pos_label.pack(anchor="w", pady=(5, 0))

        widgets['pos_x'] = ttk.Label(parent, text="  X:  0.000 m", font=("Courier", 9))
        widgets['pos_x'].pack(anchor="w")

        widgets['pos_y'] = ttk.Label(parent, text="  Y:  0.000 m", font=("Courier", 9))
        widgets['pos_y'].pack(anchor="w")

        widgets['pos_z'] = ttk.Label(parent, text="  Z:  0.000 m", font=("Courier", 9))
        widgets['pos_z'].pack(anchor="w")

        # Orientation
        orient_label = ttk.Label(parent, text="Orientation (°):", font=("Arial", 9, "bold"))
        orient_label.pack(anchor="w", pady=(5, 0))

        widgets['roll'] = ttk.Label(parent, text="  Roll:    0.0", font=("Courier", 9))
        widgets['roll'].pack(anchor="w")

        widgets['pitch'] = ttk.Label(parent, text="  Pitch:   0.0", font=("Courier", 9))
        widgets['pitch'].pack(anchor="w")

        widgets['yaw'] = ttk.Label(parent, text="  Yaw:     0.0", font=("Courier", 9))
        widgets['yaw'].pack(anchor="w")

        return widgets

    def create_headset_panel(self):
        """Create panel for headset tracking."""
        frame = ttk.LabelFrame(self.root, text="Headset Tracking", padding=10)
        frame.pack(fill="x", padx=10, pady=5)

        self.headset_pos_label = ttk.Label(frame,
                                          text="Position: X: 0.000 m  Y: 0.000 m  Z: 0.000 m",
                                          font=("Courier", 9))
        self.headset_pos_label.pack(anchor="w")

        self.headset_orient_label = ttk.Label(frame,
                                             text="Orientation: Yaw: 0.0°  Pitch: 0.0°  Roll: 0.0°",
                                             font=("Courier", 9))
        self.headset_orient_label.pack(anchor="w")

    def create_joint_angles_panel(self):
        """Create panel for robot joint angles."""
        frame = ttk.LabelFrame(self.root, text="Robot Joint Angles (degrees)", padding=10)
        frame.pack(fill="both", expand=False, padx=10, pady=5)

        # Create two columns for left and right arms
        left_frame = ttk.Frame(frame)
        left_frame.pack(side="left", fill="both", expand=True, padx=5)

        right_frame = ttk.Frame(frame)
        right_frame.pack(side="left", fill="both", expand=True, padx=5)

        # Left arm
        left_title = ttk.Label(left_frame, text="Left Arm", font=("Arial", 10, "bold"))
        left_title.pack(anchor="w")

        self.left_joint_labels = []
        for i in range(6):
            label = ttk.Label(left_frame, text=f"J{i}:    0.0°", font=("Courier", 9))
            label.pack(anchor="w")
            self.left_joint_labels.append(label)

        # Right arm
        right_title = ttk.Label(right_frame, text="Right Arm", font=("Arial", 10, "bold"))
        right_title.pack(anchor="w")

        self.right_joint_labels = []
        for i in range(6):
            label = ttk.Label(right_frame, text=f"J{i}:    0.0°", font=("Courier", 9))
            label.pack(anchor="w")
            self.right_joint_labels.append(label)

    def create_tcp_position_panel(self):
        """Create panel for TCP positions and orientations from MuJoCo."""
        frame = ttk.LabelFrame(self.root, text="TCP Positions & Orientations (MuJoCo)", padding=10)
        frame.pack(fill="both", expand=False, padx=10, pady=5)

        # Two columns for left and right TCP
        left_frame = ttk.Frame(frame)
        left_frame.pack(side="left", fill="both", expand=True, padx=5)

        right_frame = ttk.Frame(frame)
        right_frame.pack(side="left", fill="both", expand=True, padx=5)

        # Left TCP
        self.left_tcp_widgets = self.create_tcp_widgets(left_frame, "Left TCP")

        # Right TCP
        self.right_tcp_widgets = self.create_tcp_widgets(right_frame, "Right TCP")

    def create_tcp_widgets(self, parent, title):
        """Create widgets for displaying a single TCP's data."""
        widgets = {}

        # Title
        title_label = ttk.Label(parent, text=title, font=("Arial", 10, "bold"))
        title_label.pack(anchor="w")

        # Position
        pos_label = ttk.Label(parent, text="Position:", font=("Arial", 9, "bold"))
        pos_label.pack(anchor="w", pady=(5, 0))

        widgets['pos_x'] = ttk.Label(parent, text="  X:  0.000 m", font=("Courier", 9))
        widgets['pos_x'].pack(anchor="w")

        widgets['pos_y'] = ttk.Label(parent, text="  Y:  0.000 m", font=("Courier", 9))
        widgets['pos_y'].pack(anchor="w")

        widgets['pos_z'] = ttk.Label(parent, text="  Z:  0.000 m", font=("Courier", 9))
        widgets['pos_z'].pack(anchor="w")

        # Orientation
        orient_label = ttk.Label(parent, text="Orientation (°):", font=("Arial", 9, "bold"))
        orient_label.pack(anchor="w", pady=(5, 0))

        widgets['roll'] = ttk.Label(parent, text="  Roll:    0.0", font=("Courier", 9))
        widgets['roll'].pack(anchor="w")

        widgets['pitch'] = ttk.Label(parent, text="  Pitch:   0.0", font=("Courier", 9))
        widgets['pitch'].pack(anchor="w")

        widgets['yaw'] = ttk.Label(parent, text="  Yaw:     0.0", font=("Courier", 9))
        widgets['yaw'].pack(anchor="w")

        return widgets

    def create_debug_log_panel(self):
        """Create panel for debug logging."""
        frame = ttk.LabelFrame(self.root, text="Debug Log", padding=10)
        frame.pack(fill="both", expand=True, padx=10, pady=5)

        # Scrolled text widget for logs
        self.log_text = scrolledtext.ScrolledText(frame, height=10, font=("Courier", 9))
        self.log_text.pack(fill="both", expand=True)

        # Configure text tags for different log levels
        self.log_text.tag_config("INFO", foreground="black")
        self.log_text.tag_config("WARNING", foreground="orange")
        self.log_text.tag_config("ERROR", foreground="red")

    def start_monitoring(self):
        """Start the monitoring thread."""
        if not self.monitoring:
            self.monitoring = True
            self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
            self.monitor_thread.start()
            self.log_message("Monitoring started", "INFO")

    def stop_monitoring(self):
        """Stop the monitoring thread."""
        if self.monitoring:
            self.monitoring = False
            if self.monitor_thread is not None:
                self.monitor_thread.join(timeout=1.0)
            self.log_message("Monitoring stopped", "INFO")

    def monitor_loop(self):
        """Main monitoring loop (runs in separate thread at ~10Hz)."""
        while self.monitoring:
            try:
                start_time = time.time()

                # Only fetch data if hand tracking is enabled
                if self.hand_tracking_enabled:
                    # Fetch all data
                    self.fetch_hand_data()
                    self.fetch_headset_data()
                    self.fetch_joint_data()
                    self.fetch_tcp_data()

                    # Calculate actual update rate
                    current_time = time.time()
                    dt = current_time - self.last_update_time
                    if dt > 0:
                        self.actual_update_rate = 0.9 * self.actual_update_rate + 0.1 * (1.0 / dt)
                    self.last_update_time = current_time

                    # Schedule GUI update on main thread (thread-safe)
                    self.root.after(0, self.update_display)

                # Sleep to maintain ~10Hz rate
                elapsed = time.time() - start_time
                sleep_time = max(0, (1.0 / self.update_rate_hz) - elapsed)
                time.sleep(sleep_time)

            except Exception as e:
                self.log_message(f"Error in monitor loop: {e}", "ERROR")
                time.sleep(0.1)

    def fetch_hand_data(self):
        """Fetch hand wrist tracking data from XrClient."""
        # Left hand
        left_wrist = self.xr_client.get_pose_by_name("left_hand_wrist")
        self.hand_data['left'] = self._parse_pose(left_wrist)

        # Right hand
        right_wrist = self.xr_client.get_pose_by_name("right_hand_wrist")
        self.hand_data['right'] = self._parse_pose(right_wrist)

    def fetch_headset_data(self):
        """Fetch headset pose from XrClient."""
        headset_pose = self.xr_client.get_pose_by_name("headset")
        self.headset_data = self._parse_pose(headset_pose)

    def fetch_joint_data(self):
        """Fetch robot joint angles from controller."""
        if self.controller is not None:
            try:
                q = self.controller.placo_robot.state.q.copy()

                # Extract arm joints (skip floating base at indices 0-6)
                self.joint_data = {
                    'left_arm': np.degrees(q[7:13]),   # 6 joints
                    'right_arm': np.degrees(q[13:19])  # 6 joints
                }
            except Exception as e:
                self.log_message(f"Error fetching joint data: {e}", "WARNING")

    def fetch_tcp_data(self):
        """Fetch TCP positions and orientations from MuJoCo via controller."""
        if self.controller is not None:
            try:
                # Left TCP
                left_pos, left_quat = self.controller._get_link_pose("left_tool0")
                # MuJoCo returns quaternions in [w, x, y, z] format, convert to [x, y, z, w]
                left_quat_xyzw = np.array([left_quat[1], left_quat[2], left_quat[3], left_quat[0]])
                left_euler_rad = self._quat_to_euler(left_quat_xyzw)
                left_euler_deg = np.degrees(left_euler_rad)

                self.tcp_data['left'] = {
                    'position': left_pos,
                    'quaternion': left_quat,
                    'euler_deg': left_euler_deg,
                    'active': True
                }

                # Right TCP
                right_pos, right_quat = self.controller._get_link_pose("right_tool0")
                # MuJoCo returns quaternions in [w, x, y, z] format, convert to [x, y, z, w]
                right_quat_xyzw = np.array([right_quat[1], right_quat[2], right_quat[3], right_quat[0]])
                right_euler_rad = self._quat_to_euler(right_quat_xyzw)
                right_euler_deg = np.degrees(right_euler_rad)

                self.tcp_data['right'] = {
                    'position': right_pos,
                    'quaternion': right_quat,
                    'euler_deg': right_euler_deg,
                    'active': True
                }
            except Exception as e:
                self.log_message(f"Error fetching TCP data: {e}", "WARNING")
                self.tcp_data = {'left': {'active': False}, 'right': {'active': False}}

    def _parse_pose(self, pose):
        """
        Parse pose array into position and Euler angles.

        Args:
            pose: numpy array [x, y, z, qx, qy, qz, qw] or None

        Returns:
            dict with 'position', 'euler_deg', 'active' or {'active': False} if None
        """
        if pose is None:
            return {'active': False}

        position = pose[0:3]
        quaternion = pose[3:7]  # [qx, qy, qz, qw]

        # Convert quaternion to Euler angles
        euler_rad = self._quat_to_euler(quaternion)
        euler_deg = np.degrees(euler_rad)

        return {
            'position': position,
            'euler_deg': euler_deg,
            'active': True
        }

    def _quat_to_euler(self, q):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).

        Args:
            q: quaternion [qx, qy, qz, qw]

        Returns:
            numpy array [roll, pitch, yaw] in radians
        """
        qx, qy, qz, qw = q[0], q[1], q[2], q[3]

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])

    def update_display(self):
        """Update all GUI elements with current data (called on main thread)."""
        # Update status
        self.status_label.config(text=f"Status: ● MONITORING    Update Rate: {self.actual_update_rate:.1f} Hz")

        # Update left hand
        self.update_hand_display(self.left_hand_widgets, self.hand_data['left'], "Left")

        # Update right hand
        self.update_hand_display(self.right_hand_widgets, self.hand_data['right'], "Right")

        # Update headset
        self.update_headset_display()

        # Update joint angles
        self.update_joint_display()

        # Update TCP positions
        self.update_tcp_display()

    def update_hand_display(self, widgets, data, hand_name):
        """Update display for a single hand."""
        if data.get('active'):
            # Active - show green status
            widgets['status'].config(text="Status: ● ACTIVE", foreground="green")

            # Update position
            pos = data['position']
            widgets['pos_x'].config(text=f"  X: {pos[0]:7.3f} m")
            widgets['pos_y'].config(text=f"  Y: {pos[1]:7.3f} m")
            widgets['pos_z'].config(text=f"  Z: {pos[2]:7.3f} m")

            # Update orientation
            euler = data['euler_deg']
            widgets['roll'].config(text=f"  Roll:  {euler[0]:6.1f}")
            widgets['pitch'].config(text=f"  Pitch: {euler[1]:6.1f}")
            widgets['yaw'].config(text=f"  Yaw:   {euler[2]:6.1f}")
        else:
            # Inactive - show red status
            widgets['status'].config(text="Status: ○ INACTIVE", foreground="red")

    def update_headset_display(self):
        """Update headset display."""
        if self.headset_data.get('active'):
            pos = self.headset_data['position']
            euler = self.headset_data['euler_deg']

            self.headset_pos_label.config(
                text=f"Position: X: {pos[0]:.3f} m  Y: {pos[1]:.3f} m  Z: {pos[2]:.3f} m"
            )
            self.headset_orient_label.config(
                text=f"Orientation: Yaw: {euler[2]:.1f}°  Pitch: {euler[1]:.1f}°  Roll: {euler[0]:.1f}°"
            )
        else:
            self.headset_pos_label.config(text="Position: No tracking data")
            self.headset_orient_label.config(text="Orientation: No tracking data")

    def update_joint_display(self):
        """Update joint angle display."""
        # Left arm
        for i, angle in enumerate(self.joint_data['left_arm']):
            self.left_joint_labels[i].config(text=f"J{i}: {angle:6.1f}°")

        # Right arm
        for i, angle in enumerate(self.joint_data['right_arm']):
            self.right_joint_labels[i].config(text=f"J{i}: {angle:6.1f}°")

    def update_tcp_display(self):
        """Update TCP position and orientation display."""
        # Left TCP
        self.update_tcp_widgets(self.left_tcp_widgets, self.tcp_data['left'])

        # Right TCP
        self.update_tcp_widgets(self.right_tcp_widgets, self.tcp_data['right'])

    def update_tcp_widgets(self, widgets, data):
        """Update display for a single TCP."""
        if data.get('active'):
            # Update position
            pos = data['position']
            widgets['pos_x'].config(text=f"  X: {pos[0]:7.3f} m")
            widgets['pos_y'].config(text=f"  Y: {pos[1]:7.3f} m")
            widgets['pos_z'].config(text=f"  Z: {pos[2]:7.3f} m")

            # Update orientation
            euler = data['euler_deg']
            widgets['roll'].config(text=f"  Roll:  {euler[0]:6.1f}")
            widgets['pitch'].config(text=f"  Pitch: {euler[1]:6.1f}")
            widgets['yaw'].config(text=f"  Yaw:   {euler[2]:6.1f}")
        else:
            # Show inactive status
            widgets['pos_x'].config(text="  X:  N/A")
            widgets['pos_y'].config(text="  Y:  N/A")
            widgets['pos_z'].config(text="  Z:  N/A")
            widgets['roll'].config(text="  Roll:  N/A")
            widgets['pitch'].config(text="  Pitch: N/A")
            widgets['yaw'].config(text="  Yaw:   N/A")

    def log_message(self, message, level="INFO"):
        """
        Add timestamped message to debug log.

        Args:
            message: Message string to log
            level: Log level ("INFO", "WARNING", "ERROR")
        """
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        formatted_msg = f"[{timestamp}] {level}: {message}\n"

        # Insert into text widget on main thread
        def insert_log():
            self.log_text.insert(tk.END, formatted_msg, level)
            self.log_text.see(tk.END)  # Auto-scroll to bottom

        if threading.current_thread() == threading.main_thread():
            insert_log()
        else:
            self.root.after(0, insert_log)

    def clear_logs(self):
        """Clear the debug log."""
        self.log_text.delete(1.0, tk.END)
        self.log_message("Logs cleared", "INFO")

    def toggle_hand_tracking(self):
        """Toggle hand tracking control on/off."""
        self.hand_tracking_enabled = not self.hand_tracking_enabled

        if self.hand_tracking_enabled:
            # Enable hand tracking
            self.hand_tracking_btn.config(text="Disable Hand Tracking")
            self.hand_tracking_status.config(text="● ENABLED", foreground="green")
            self.log_message("Hand tracking control ENABLED - Robot will respond to hand movements", "INFO")
        else:
            # Disable hand tracking
            self.hand_tracking_btn.config(text="Enable Hand Tracking")
            self.hand_tracking_status.config(text="○ DISABLED", foreground="red")
            self.log_message("Hand tracking control DISABLED - Robot frozen at current position", "INFO")

    def get_arm_pose_from_robot(self):
        """Get current joint positions from real robot via RTDE and update simulation."""
        if self.rtde_receiver is None:
            self.log_message("RTDE not available - cannot get arm pose", "ERROR")
            return

        if self.controller is None:
            self.log_message("Controller not available - cannot update pose", "ERROR")
            return

        try:
            # Get current joint positions from right arm
            right_joints = np.array(self.rtde_receiver.getActualQ())

            self.log_message(
                f"Retrieved joint positions - Right: {np.degrees(right_joints).round(1)}",
                "INFO"
            )

            # Update placo robot state
            # The joint order in placo is: [floating_base (7), left_arm (6), right_arm (6)]
            q = self.controller.placo_robot.state.q.copy()
            q[13:19] = right_joints  # Right arm joints (indices 13-18)

            # Update placo state
            self.controller.placo_robot.state.q = q
            self.controller.placo_robot.update_kinematics()

            # Update MuJoCo simulation
            # Get MuJoCo joint indices for the right UR5e arm
            right_joint_names = [
                "right_shoulder_pan_joint",
                "right_shoulder_lift_joint",
                "right_elbow_joint",
                "right_wrist_1_joint",
                "right_wrist_2_joint",
                "right_wrist_3_joint"
            ]

            # Update MuJoCo joint positions for right arm
            for i, joint_name in enumerate(right_joint_names):
                joint_id = self.controller.mj_model.joint(joint_name).id
                qpos_addr = self.controller.mj_model.jnt_qposadr[joint_id]
                self.controller.mj_data.qpos[qpos_addr] = right_joints[i]

            # Forward kinematics to update MuJoCo state
            import mujoco
            mujoco.mj_forward(self.controller.mj_model, self.controller.mj_data)

            self.log_message("Successfully updated simulation with right arm pose", "INFO")

        except Exception as e:
            self.log_message(f"Error getting arm pose from robot: {e}", "ERROR")
            import traceback
            traceback.print_exc()

    def update_once(self):
        """
        Update GUI once (for external calling from control loop).
        This method can be called periodically instead of running the monitoring thread.
        """
        # Update every Nth call to maintain ~10Hz rate
        self.update_counter += 1
        if self.update_counter % 10 == 0:  # Assuming control loop is ~100Hz
            # Only fetch data if hand tracking is enabled
            if self.hand_tracking_enabled:
                # Fetch all data
                self.fetch_hand_data()
                self.fetch_headset_data()
                self.fetch_joint_data()
                self.fetch_tcp_data()

                # Calculate update rate
                current_time = time.time()
                dt = current_time - self.last_update_time
                if dt > 0:
                    self.actual_update_rate = 0.9 * self.actual_update_rate + 0.1 * (1.0 / dt)
                self.last_update_time = current_time

                # Update display
                self.update_display()

        # Process GUI events
        self.root.update()

    def on_closing(self):
        """Handle window close event."""
        self.stop_monitoring()
        self.root.quit()  # Exit mainloop
        self.root.destroy()

    def run(self):
        """Run the GUI main loop."""
        self.root.mainloop()


def main():
    """Standalone testing mode."""
    print("Running monitoring GUI in standalone mode (no controller)...")
    gui = TeleopMonitorGUI(controller=None)
    gui.run()


if __name__ == "__main__":
    main()
