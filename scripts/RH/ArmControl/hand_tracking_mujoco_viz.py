import os
import tkinter as tk
import threading
import time
import math

import tyro
import numpy as np
import mujoco
from mujoco import viewer as mj_viewer
import rtde_control
import rtde_receive

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
    MujocoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


class HardwareMujocoTeleopController(MujocoTeleopController):
    def __init__(self, *args, hardware_ip="192.168.2.2", max_joint_velocity=1.0, **kwargs):
        self.hardware_ip = hardware_ip
        self.rtde_c = None
        self.rtde_r = None
        self.arm_joint_indices = None
        self.control_mode = "stopped"  # "stopped", "hand_tracking", "manual"
        self.q_target = None  # From IK solver
        self.q_manual = None  # From GUI sliders
        self.q_actual = None  # From hardware feedback
        self.q_command = None  # Final command to send
        self.q_command_prev = None  # Previous command for velocity limiting
        self.q_simulated = None  # Joint positions from MuJoCo after physics step
        self.q_simulated_prev = None  # For secondary velocity limiting
        self.send_to_hardware = False  # Control whether to send commands to hardware
        self.is_connected = False

        # Speed limiting
        self.max_joint_velocity = max_joint_velocity  # rad/s
        self.dt = 1.0 / 60.0  # Assume 60Hz control loop
        self.last_update_time = None

        # Logging data
        self.log_data = {
            'hand_tracking': {
                'left_hand_pose': None,
                'right_hand_pose': None,
                'left_hand_delta': None,
                'right_hand_delta': None,
            },
            'ik_result': {
                'q_target': None,
                'q_target_velocity': None,
                'velocity_limited': False,
            },
            'control': {
                'mode': 'stopped',
                'q_command': None,     # IK output (velocity limited)
                'q_simulated': None,   # MuJoCo output (after physics)
                'q_actual': None,      # Hardware feedback
                'error': None,         # q_simulated - q_actual
            }
        }

        # Setup GUI
        self.gui_root = tk.Tk()
        self.gui_root.title(f"Linear Pipeline Control - {self.hardware_ip}")
        self.gui_root.geometry("800x1000")  # Increased for logging panel
        
        self.connect_btn = tk.Button(self.gui_root, text="Connect to Robot", command=self.connect, bg="blue", fg="white", font=("Arial", 12))
        self.connect_btn.pack(pady=10, fill="x", padx=20)

        # Mode Selection Frame
        mode_frame = tk.Frame(self.gui_root)
        mode_frame.pack(fill="x", padx=20, pady=5)

        self.start_btn = tk.Button(mode_frame, text="Start Hand Tracking", command=self.start_control, bg="green", fg="white", font=("Arial", 12), state="disabled", width=20)
        self.start_btn.pack(side="left", padx=5, fill="x", expand=True)

        self.manual_btn = tk.Button(mode_frame, text="Manual Control", command=self.toggle_manual_control, bg="purple", fg="white", font=("Arial", 12), state="disabled", width=20)
        self.manual_btn.pack(side="right", padx=5, fill="x", expand=True)
        
        self.stop_btn = tk.Button(self.gui_root, text="EMERGENCY STOP", command=self.emergency_stop, bg="red", fg="white", font=("Arial", 12, "bold"), state="disabled")
        self.stop_btn.pack(pady=5, fill="x", padx=20)

        # Send to Hardware toggle
        self.send_hardware_var = tk.BooleanVar(value=False)  # Default: OFF (viz-only mode)
        self.send_hardware_checkbox = tk.Checkbutton(
            self.gui_root,
            text="Send to Hardware",
            variable=self.send_hardware_var,
            command=self._toggle_hardware_send,
            font=("Arial", 11),
            bg="#FFD700",  # Gold background for visibility
            activebackground="#FFA500"
        )
        self.send_hardware_checkbox.pack(pady=5, padx=20, fill="x")

        # Add warning label
        self.hardware_warning_label = tk.Label(
            self.gui_root,
            text="⚠️  CAUTION: Enables physical robot control",
            font=("Arial", 9),
            fg="red",
            bg="white"
        )
        self.hardware_warning_label.pack(pady=2)

        self.status_label = tk.Label(self.gui_root, text="Status: Disconnected", fg="red", font=("Arial", 10, "bold"))
        self.status_label.pack(pady=5)

        # Create frame for joint angle displays
        self.angles_frame = tk.LabelFrame(self.gui_root, text="Joint Angles (degrees)", padx=10, pady=10)
        self.angles_frame.pack(fill="both", expand=True, padx=20, pady=5)

        # Header
        tk.Label(self.angles_frame, text="Joint", font=("Courier", 10, "bold"), width=8).grid(row=0, column=0)
        tk.Label(self.angles_frame, text="Target", font=("Courier", 10, "bold"), width=12).grid(row=0, column=1)
        tk.Label(self.angles_frame, text="Actual", font=("Courier", 10, "bold"), width=12).grid(row=0, column=2)
        tk.Label(self.angles_frame, text="Error", font=("Courier", 10, "bold"), width=12).grid(row=0, column=3)

        # Create labels for each joint
        self.joint_display_labels = []
        for i in range(6):
            joint_label = tk.Label(self.angles_frame, text=f"J{i}", font=("Courier", 10), width=8)
            target_label = tk.Label(self.angles_frame, text="---", font=("Courier", 10), width=12)
            actual_label = tk.Label(self.angles_frame, text="---", font=("Courier", 10), width=12)
            error_label = tk.Label(self.angles_frame, text="---", font=("Courier", 10), width=12)

            joint_label.grid(row=i+1, column=0, pady=2)
            target_label.grid(row=i+1, column=1, pady=2)
            actual_label.grid(row=i+1, column=2, pady=2)
            error_label.grid(row=i+1, column=3, pady=2)

            self.joint_display_labels.append({
                'target': target_label,
                'actual': actual_label,
                'error': error_label
            })

        # Logging display frame
        self.log_frame = tk.LabelFrame(self.gui_root, text="Data Logging", padx=10, pady=10)
        self.log_frame.pack(fill="both", expand=True, padx=20, pady=5)

        # Create text widget for logging with scrollbar
        log_scroll = tk.Scrollbar(self.log_frame)
        log_scroll.pack(side="right", fill="y")

        self.log_text = tk.Text(self.log_frame, height=12, font=("Courier", 9),
                                yscrollcommand=log_scroll.set, bg="#f0f0f0")
        self.log_text.pack(side="left", fill="both", expand=True)
        log_scroll.config(command=self.log_text.yview)
        self.log_text.insert("1.0", "Waiting for data...\n")
        self.log_text.config(state="disabled")

        # Sliders Frame (Initially hidden)
        self.sliders_frame = tk.LabelFrame(self.gui_root, text="Manual Joint Control", padx=10, pady=10)
        self.sliders = []
        self.slider_labels = []

        for i in range(6):
            frame = tk.Frame(self.sliders_frame)
            frame.pack(fill="x", pady=2)

            lbl = tk.Label(frame, text=f"J{i}: 0.0°", width=10)
            lbl.pack(side="left")
            self.slider_labels.append(lbl)

            # Using standard UR limits -360 to 360, but can be adjusted
            slider = tk.Scale(frame, from_=-360, to=360, orient="horizontal", length=400, command=lambda val, idx=i: self.on_slider_change(idx, val))
            slider.pack(side="right", fill="x", expand=True)
            self.sliders.append(slider)

        # Initialize parent
        super().__init__(*args, **kwargs)

        # Initialize q_manual to zeros if not connected (for manual mode without hardware)
        if self.q_manual is None:
            self.q_manual = np.zeros(6)

    def connect(self):
        try:
            self.status_label.config(text="Status: Connecting...", fg="orange")
            self.gui_root.update()

            print(f"Connecting to UR hardware at {self.hardware_ip}...")
            self.rtde_c = rtde_control.RTDEControlInterface(self.hardware_ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.hardware_ip)

            self.is_connected = True
            self.control_mode = "stopped"
            self.status_label.config(text=f"Status: Connected (Stopped)", fg="orange")
            self.connect_btn.config(state="disabled", text="Connected")
            self.start_btn.config(state="normal")
            self.manual_btn.config(state="normal")
            self.stop_btn.config(state="normal")

            print("Successfully connected to UR hardware.")

            # Initialize with current robot position
            current_q = self.rtde_r.getActualQ()
            self.q_actual = np.array(current_q)
            self.q_manual = self.q_actual.copy()
            self.q_command = self.q_actual.copy()

        except Exception as e:
            self.status_label.config(text=f"Status: Connection Error", fg="red")
            print(f"Failed to connect to UR hardware: {e}")

    def start_control(self):
        # Can start hand tracking even without robot connection (for visualization only)
        self.control_mode = "hand_tracking"
        self.sliders_frame.pack_forget()  # Hide sliders
        if self.is_connected:
            self.status_label.config(text="Status: Hand Tracking ACTIVE", fg="green")
        else:
            self.status_label.config(text="Status: Hand Tracking (Viz Only)", fg="blue")
        print(f"Hand tracking control STARTED. Connected: {self.is_connected}")

    def toggle_manual_control(self):
        if self.is_connected:
            self.control_mode = "manual"

            # Update sliders to current actual position before showing
            if self.q_actual is not None:
                self.q_manual = self.q_actual.copy()
                for i, val in enumerate(self.q_manual):
                    deg = math.degrees(val)
                    self.sliders[i].set(deg)
                    self.slider_labels[i].config(text=f"J{i}: {deg:.1f}°")

            self.sliders_frame.pack(fill="both", expand=True, padx=10, pady=5)
            self.status_label.config(text="Status: Manual Control ACTIVE", fg="purple")
            print("Manual control STARTED.")

    def on_slider_change(self, idx, value):
        if self.control_mode == "manual" and self.q_manual is not None:
            self.q_manual[idx] = math.radians(float(value))
            self.slider_labels[idx].config(text=f"J{idx}: {float(value):.1f}°")

    def emergency_stop(self):
        self.control_mode = "stopped"
        self.sliders_frame.pack_forget()
        self.status_label.config(text="Status: STOPPED", fg="red")
        print("Hardware control STOPPED (Emergency).")

        if self.rtde_c:
            try:
                self.rtde_c.servoStop()
            except Exception as e:
                print(f"Error stopping hardware: {e}")

    def _toggle_hardware_send(self):
        """Toggle hardware sending on/off."""
        self.send_to_hardware = self.send_hardware_var.get()
        if self.send_to_hardware:
            print("[HARDWARE] Sending to hardware ENABLED")
            self.hardware_warning_label.config(text="⚠️  Hardware control ACTIVE", fg="red")
        else:
            print("[HARDWARE] Sending to hardware DISABLED (visualization only)")
            self.hardware_warning_label.config(text="✓  Visualization-only mode (safe)", fg="green")

    def _placo_setup(self):
        super()._placo_setup()
        
        RIGHT_ROBOT_IP = "192.168.2.3"
        prefix = "left"
        if self.hardware_ip == RIGHT_ROBOT_IP:
            prefix = "right"
        
        print(f"Hardware IP: {self.hardware_ip} -> Selecting {prefix} arm joints.")

        # Identify arm joint indices in Placo
        joint_names = [
            f"{prefix}_shoulder_pan_joint",
            f"{prefix}_shoulder_lift_joint",
            f"{prefix}_elbow_joint",
            f"{prefix}_wrist_1_joint",
            f"{prefix}_wrist_2_joint",
            f"{prefix}_wrist_3_joint",
        ]
        
        self.arm_joint_indices = []
        for name in joint_names:
            try:
                idx = self.placo_robot.get_joint_offset(name)
                self.arm_joint_indices.append(idx)
                print(f"  Found {name} at index {idx}")
            except Exception as e:
                print(f"Warning: Could not find joint {name} in Placo model: {e}")
        
        if not self.arm_joint_indices:
            print(f"ERROR: No {prefix} arm joints found! Hardware control will not work.")

    def _robot_setup(self):
        super()._robot_setup()

    def _log_hand_tracking_data(self):
        """Capture and log hand tracking data before IK processing."""
        if self.control_mode != "hand_tracking":
            return

        # Log hand tracking poses for configured manipulators
        for src_name, config in self.manipulator_config.items():
            pose_source = config.get("pose_source", "")

            # Get raw XR pose
            xr_pose = self.xr_client.get_pose_by_name(pose_source)

            if xr_pose is not None:
                # Determine which hand this is
                if "left" in src_name.lower():
                    hand_key = "left_hand"
                elif "right" in src_name.lower():
                    hand_key = "right_hand"
                else:
                    continue

                # Store pose data
                self.log_data['hand_tracking'][f'{hand_key}_pose'] = xr_pose.copy()

                # Calculate and store delta if reference exists
                if self.ref_controller_xyz.get(src_name) is not None:
                    controller_xyz = np.array([xr_pose[0], xr_pose[1], xr_pose[2]])
                    controller_xyz = self.R_headset_world @ controller_xyz

                    delta_xyz = (controller_xyz - self.ref_controller_xyz[src_name]) * self.scale_factor

                    controller_quat = [xr_pose[6], xr_pose[3], xr_pose[4], xr_pose[5]]
                    from xrobotoolkit_teleop.utils.geometry import quat_diff_as_angle_axis
                    delta_rot = quat_diff_as_angle_axis(self.ref_controller_quat[src_name], controller_quat)

                    delta = np.concatenate([delta_xyz, delta_rot])
                    self.log_data['hand_tracking'][f'{hand_key}_delta'] = delta

    def _apply_velocity_limit(self, q_desired, q_previous):
        """Apply velocity limiting to commanded joint angles.

        Args:
            q_desired: Desired joint angles [6x1]
            q_previous: Previous joint angles [6x1]

        Returns:
            q_limited: Velocity-limited joint angles [6x1]
            was_limited: Boolean indicating if any joint was limited
        """
        if q_desired is None or q_previous is None:
            return q_desired, False

        # Calculate desired velocity
        delta_q = q_desired - q_previous
        velocity = delta_q / self.dt

        # Calculate velocity magnitude for each joint
        velocity_magnitude = np.abs(velocity)

        # Check if any joint exceeds limit
        max_vel = np.max(velocity_magnitude)
        was_limited = max_vel > self.max_joint_velocity

        if was_limited:
            # Scale down all velocities proportionally
            scale = self.max_joint_velocity / max_vel
            velocity_limited = velocity * scale
            q_limited = q_previous + velocity_limited * self.dt

            # Log velocity limiting
            self.log_data['ik_result']['velocity_limited'] = True
            self.log_data['ik_result']['q_target_velocity'] = velocity_magnitude
        else:
            q_limited = q_desired
            self.log_data['ik_result']['velocity_limited'] = False
            self.log_data['ik_result']['q_target_velocity'] = velocity_magnitude

        return q_limited, was_limited

    def _extract_q_target_from_ik(self):
        """Extract target joint angles from IK solver result.

        ALWAYS extract when IK runs, regardless of robot connection status.
        This ensures the GUI always shows the IK target.
        """
        if self.arm_joint_indices and len(self.arm_joint_indices) > 0:
            try:
                self.q_target = np.array([self.placo_robot.state.q[i] for i in self.arm_joint_indices])
                # Log IK result
                self.log_data['ik_result']['q_target'] = self.q_target.copy()

                # Debug: Print q_target occasionally (every 60 frames = 1 second)
                if not hasattr(self, '_extract_counter'):
                    self._extract_counter = 0
                self._extract_counter += 1
                if self._extract_counter % 60 == 0:
                    print(f"[DEBUG] q_target extracted: {np.degrees(self.q_target).round(2)} deg")

            except IndexError as e:
                print(f"IndexError in _extract_q_target_from_ik: {e}")
        else:
            # If arm_joint_indices not set up yet, log a warning once
            if not hasattr(self, '_warned_no_indices'):
                print("Warning: arm_joint_indices not set up. Target extraction disabled.")
                self._warned_no_indices = True

    def _send_command_to_mujoco(self):
        """Send q_command to MuJoCo simulation for visualization."""
        # Update placo robot state with the command we're sending
        if self.arm_joint_indices and self.q_command is not None:
            for i, idx in enumerate(self.arm_joint_indices):
                self.placo_robot.state.q[idx] = self.q_command[i]

        # Update MuJoCo simulation
        super()._send_command()

    def _read_from_mujoco(self) -> np.ndarray:
        """
        Read simulated joint positions from MuJoCo after physics step.

        Returns:
            q_simulated: Simulated joint positions [6x1] for the arm
        """
        from xrobotoolkit_teleop.utils.mujoco_utils import calc_placo_q_from_mujoco_qpos

        # Read full MuJoCo state after physics step
        mj_qpos = self.mj_data.qpos.copy()

        # Convert to Placo format
        placo_q_full = calc_placo_q_from_mujoco_qpos(
            self.mj_model,
            self.placo_robot,
            mj_qpos,
            floating_base=False  # Dual UR5e has fixed base
        )

        # Extract arm joints only (6-DOF)
        if self.arm_joint_indices and len(self.arm_joint_indices) == 6:
            q_simulated = np.array([placo_q_full[i] for i in self.arm_joint_indices])

            # Sanity check for NaN/Inf (simulation instability detection)
            if np.any(np.isnan(q_simulated)) or np.any(np.isinf(q_simulated)):
                print("ERROR: Simulation unstable (NaN/Inf detected)")
                self.emergency_stop()
                return self.q_actual.copy() if self.q_actual is not None else None

            return q_simulated
        else:
            print("ERROR: arm_joint_indices not properly configured")
            return None

    def _send_simulated_to_hardware(self, q_simulated: np.ndarray):
        """
        Send simulated joint positions to physical robot hardware.

        Args:
            q_simulated: Simulated joint positions from MuJoCo [6x1]
        """
        # Check if hardware sending is enabled (safety feature for testing)
        if not self.send_to_hardware:
            # Visualization-only mode: skip hardware command
            return

        if not self.is_connected or q_simulated is None:
            return

        # Apply secondary velocity limiting for safety
        if self.q_simulated_prev is not None:
            q_limited, was_limited = self._apply_velocity_limit(
                q_simulated,
                self.q_simulated_prev
            )
            if was_limited:
                print("[SAFETY] Simulated state velocity limited before hardware")
        else:
            q_limited = q_simulated

        # Store for next cycle
        self.q_simulated_prev = q_limited.copy()

        # Hardware servo constants
        SERVO_TIME = 0.008
        LOOKAHEAD_TIME = 0.2
        SERVO_GAIN = 100.0

        try:
            if self.control_mode in ["hand_tracking", "manual"]:
                t_start = self.rtde_c.initPeriod()
                self.rtde_c.servoJ(
                    q_limited.tolist(),
                    0.0,  # velocity (let servoJ calculate)
                    0.0,  # acceleration
                    SERVO_TIME,
                    LOOKAHEAD_TIME,
                    SERVO_GAIN
                )
                self.rtde_c.waitPeriod(t_start)
        except Exception as e:
            print(f"Hardware command error: {e}")

    def _update_logging_data(self):
        """Update logging data structure with current state."""
        self.log_data['control']['mode'] = self.control_mode
        self.log_data['control']['q_command'] = self.q_command.copy() if self.q_command is not None else None
        self.log_data['control']['q_simulated'] = self.q_simulated.copy() if self.q_simulated is not None else None
        self.log_data['control']['q_actual'] = self.q_actual.copy() if self.q_actual is not None else None

        # Calculate error: simulated vs actual (digital twin tracking error)
        if self.q_simulated is not None and self.q_actual is not None:
            self.log_data['control']['error'] = self.q_simulated - self.q_actual

    def _send_command_to_hardware(self):
        """Send q_command to physical robot hardware."""
        if not self.is_connected or self.q_command is None:
            return

        # Constants for UR servo control
        SERVO_TIME = 0.008
        LOOKAHEAD_TIME = 0.2
        SERVO_GAIN = 100.0

        try:
            if self.control_mode in ["hand_tracking", "manual"]:
                t_start = self.rtde_c.initPeriod()
                self.rtde_c.servoJ(
                    self.q_command.tolist(),
                    0.0,  # velocity
                    0.0,  # acceleration
                    SERVO_TIME,
                    LOOKAHEAD_TIME,
                    SERVO_GAIN
                )
                self.rtde_c.waitPeriod(t_start)
        except Exception as e:
            print(f"Hardware command error: {e}")

    def _update_gui_display(self):
        """Update GUI with current joint angles and errors."""
        # Always update if we have 6 joints to display
        if self.q_target is None and self.q_actual is None:
            return

        for i in range(6):
            # Display SIMULATED angle (from MuJoCo physics)
            if self.q_simulated is not None and i < len(self.q_simulated):
                simulated_deg = math.degrees(self.q_simulated[i])
                self.joint_display_labels[i]['target'].config(
                    text=f"{simulated_deg:>7.2f}°",
                    fg="blue"  # Blue to indicate "simulated" not "target"
                )
            else:
                self.joint_display_labels[i]['target'].config(text="---")

            # Display actual angle (only if connected)
            if self.q_actual is not None and i < len(self.q_actual):
                actual_deg = math.degrees(self.q_actual[i])
                self.joint_display_labels[i]['actual'].config(text=f"{actual_deg:>7.2f}°")
            else:
                self.joint_display_labels[i]['actual'].config(text="---")

            # Display error (simulated - actual)
            if self.q_simulated is not None and self.q_actual is not None:
                error_deg = math.degrees(self.q_simulated[i] - self.q_actual[i])
                # Color code errors (red if > 5 degrees)
                if abs(error_deg) > 5.0:
                    self.joint_display_labels[i]['error'].config(
                        text=f"{error_deg:>+7.2f}°", fg="red"
                    )
                else:
                    self.joint_display_labels[i]['error'].config(
                        text=f"{error_deg:>+7.2f}°", fg="black"
                    )
            else:
                self.joint_display_labels[i]['error'].config(text="---")

    def _update_log_display(self):
        """Update the logging display with current data."""
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")

        log_output = []
        log_output.append("=" * 70)
        log_output.append(f"  CONTROL MODE: {self.log_data['control']['mode'].upper()}")
        log_output.append("=" * 70)

        # IK Results
        if self.log_data['ik_result']['q_target'] is not None:
            log_output.append("\n[IK SOLVER OUTPUT]")
            q_target = self.log_data['ik_result']['q_target']
            log_output.append(f"  Target Joints (rad):  [{', '.join([f'{x:>7.4f}' for x in q_target])}]")
            log_output.append(f"  Target Joints (deg):  [{', '.join([f'{math.degrees(x):>7.2f}' for x in q_target])}]")

            # Velocity info
            if self.log_data['ik_result']['q_target_velocity'] is not None:
                vel = self.log_data['ik_result']['q_target_velocity']
                log_output.append(f"  Joint Velocities:     [{', '.join([f'{x:>7.4f}' for x in vel])}] rad/s")
                max_vel = np.max(vel)
                log_output.append(f"  Max Velocity:         {max_vel:.4f} rad/s (limit: {self.max_joint_velocity:.4f})")
                if self.log_data['ik_result']['velocity_limited']:
                    log_output.append("  ⚠️  VELOCITY LIMITED!")

        # Hand Tracking Data (if available)
        if self.control_mode == "hand_tracking":
            log_output.append("\n[HAND TRACKING INPUT]")

            for hand_name in ['left_hand', 'right_hand']:
                pose_key = f'{hand_name}_pose'
                delta_key = f'{hand_name}_delta'

                if self.log_data['hand_tracking'][pose_key] is not None:
                    pose = self.log_data['hand_tracking'][pose_key]
                    log_output.append(f"  {hand_name.upper().replace('_', ' ')}:")
                    log_output.append(f"    Position (m):  [{pose[0]:>7.4f}, {pose[1]:>7.4f}, {pose[2]:>7.4f}]")
                    log_output.append(f"    Quaternion:    [{pose[3]:>7.4f}, {pose[4]:>7.4f}, {pose[5]:>7.4f}, {pose[6]:>7.4f}]")

                if self.log_data['hand_tracking'][delta_key] is not None:
                    delta = self.log_data['hand_tracking'][delta_key]
                    log_output.append(f"    Delta Pos (m): [{delta[0]:>7.4f}, {delta[1]:>7.4f}, {delta[2]:>7.4f}]")
                    if len(delta) > 3:
                        log_output.append(f"    Delta Rot:     [{delta[3]:>7.4f}, {delta[4]:>7.4f}, {delta[5]:>7.4f}]")

        # MuJoCo Simulation Output
        log_output.append("\n[MUJOCO SIMULATION OUTPUT]")
        if self.log_data['control']['q_simulated'] is not None:
            q_sim = self.log_data['control']['q_simulated']
            log_output.append(f"  Simulated Joints (rad): [{', '.join([f'{x:>7.4f}' for x in q_sim])}]")
            log_output.append(f"  Simulated Joints (deg): [{', '.join([f'{math.degrees(x):>7.2f}' for x in q_sim])}]")

            # Show difference between command and simulated (physics effect)
            if self.log_data['control']['q_command'] is not None:
                q_cmd = self.log_data['control']['q_command']
                physics_delta = q_sim - q_cmd
                log_output.append(f"  Physics Delta (deg):    [{', '.join([f'{math.degrees(x):>+7.2f}' for x in physics_delta])}]")
                max_delta = np.max(np.abs(physics_delta))
                log_output.append(f"  Max Physics Effect:     {math.degrees(max_delta):.2f}°")

        # Control Output
        log_output.append("\n[CONTROL OUTPUT]")
        if self.log_data['control']['q_command'] is not None:
            q_cmd = self.log_data['control']['q_command']
            log_output.append(f"  Command Joints (rad): [{', '.join([f'{x:>7.4f}' for x in q_cmd])}]")
            log_output.append(f"  Command Joints (deg): [{', '.join([f'{math.degrees(x):>7.2f}' for x in q_cmd])}]")

        if self.log_data['control']['q_actual'] is not None:
            q_act = self.log_data['control']['q_actual']
            log_output.append(f"  Actual Joints (rad):  [{', '.join([f'{x:>7.4f}' for x in q_act])}]")
            log_output.append(f"  Actual Joints (deg):  [{', '.join([f'{math.degrees(x):>7.2f}' for x in q_act])}]")

        if self.log_data['control']['error'] is not None:
            err = self.log_data['control']['error']
            log_output.append(f"  Error (rad):          [{', '.join([f'{x:>+7.4f}' for x in err])}]")
            log_output.append(f"  Error (deg):          [{', '.join([f'{math.degrees(x):>+7.2f}' for x in err])}]")
            log_output.append(f"  RMS Error:            {np.sqrt(np.mean(err**2)):.4f} rad ({math.degrees(np.sqrt(np.mean(err**2))):.2f}°)")

        log_output.append("\n" + "=" * 70)

        self.log_text.insert("1.0", "\n".join(log_output))
        self.log_text.config(state="disabled")
        self.log_text.see("1.0")  # Scroll to top

    def _select_command_based_on_mode(self):
        """LINEAR PIPELINE: Select q_command based on control mode with velocity limiting."""
        q_desired = None

        if self.control_mode == "hand_tracking":
            if self.q_target is not None:
                q_desired = self.q_target.copy()
        elif self.control_mode == "manual":
            if self.q_manual is not None:
                q_desired = self.q_manual.copy()
        elif self.control_mode == "stopped":
            # Hold current position
            if self.q_actual is not None:
                q_desired = self.q_actual.copy()

        # Apply velocity limiting
        if q_desired is not None:
            if self.q_command_prev is None:
                # First command, no previous reference
                self.q_command = q_desired.copy()
                self.q_command_prev = self.q_command.copy()
            else:
                # Apply velocity limit
                self.q_command, was_limited = self._apply_velocity_limit(q_desired, self.q_command_prev)
                self.q_command_prev = self.q_command.copy()

                if was_limited:
                    print(f"[VELOCITY LIMIT] Max velocity exceeded in {self.control_mode} mode")

    def run(self):
        """
        LINEAR PIPELINE: Main control loop.

        DIGITAL TWIN ARCHITECTURE:
        - Commands sent to MuJoCo first
        - Physics simulation computes physically-valid state
        - Hardware follows simulated state (digital twin synchronization)
        - Result: Hardware experiences gravity, inertia, collision effects
        """
        with mj_viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
            # Set up viewer camera
            viewer.cam.azimuth = 0
            viewer.cam.elevation = -50
            viewer.cam.distance = 2.0
            viewer.cam.lookat = [0.2, 0, 0]

            while not self._stop_event.is_set():
                try:
                    # ═══════════════════════════════════════════════════════
                    # LINEAR DATAFLOW PIPELINE
                    # ═══════════════════════════════════════════════════════

                    # STEP 1: Read actual joint angles from hardware
                    if self.is_connected and self.rtde_r:
                        self.q_actual = np.array(self.rtde_r.getActualQ())

                    # STEP 2: Update robot state in simulation
                    self._update_robot_state()

                    # STEP 2.5: Log hand tracking data (before IK)
                    self._log_hand_tracking_data()

                    # STEP 3: Process hand tracking and run IK solver
                    self._update_ik()
                    self._update_gripper_target()
                    self._update_mocap_target()

                    # STEP 4: Extract q_target from IK solution
                    self._extract_q_target_from_ik()

                    # STEP 5: SELECT q_command based on control mode
                    self._select_command_based_on_mode()

                    # STEP 6: Send q_command to MuJoCo FIRST
                    self._send_command_to_mujoco()

                    # STEP 7: Physics step (with gravity, dynamics, collisions)
                    mujoco.mj_step(self.mj_model, self.mj_data)

                    # STEP 8: Read simulated joint state from MuJoCo
                    self.q_simulated = self._read_from_mujoco()

                    # STEP 9: Send simulated state to hardware
                    self._send_simulated_to_hardware(self.q_simulated)

                    # STEP 10: Update logging data
                    self._update_logging_data()

                    # STEP 11: Update GUI displays
                    self._update_gui_display()
                    self._update_log_display()
                    self.gui_root.update()

                    # STEP 12: Render viewer
                    viewer.sync()

                except KeyboardInterrupt:
                    print("\nTeleoperation stopped.")
                    self._stop_event.set()
                except tk.TclError:
                    print("\nGUI Window closed. Stopping.")
                    self._stop_event.set()

        # Cleanup
        print("\nCleaning up...")
        self.emergency_stop()
        self.is_connected = False

        if self.rtde_c:
            print("Closing UR hardware connection...")
            try:
                self.rtde_c.servoStop()
                self.rtde_c.disconnect()
            except:
                pass
        if self.rtde_r:
            try:
                self.rtde_r.disconnect()
            except:
                pass

        try:
            self.gui_root.destroy()
        except:
            pass


def main(
    xml_path: str = os.path.join(ASSET_PATH, "universal_robots_ur5e/scene_dual_arm.xml"),
    robot_urdf_path: str = os.path.join(ASSET_PATH, "universal_robots_ur5e/dual_ur5e.urdf"),
    scale_factor: float = 1.5,
    visualize_placo: bool = True,
    hardware_ip: str = "192.168.2.2",
    max_joint_velocity: float = 1.0,
):
    """
    Main function to run dual UR5e hand tracking visualization in MuJoCo with hardware control.

    Uses hand wrist tracking (no VR controllers needed).
    Continuous control - arms follow hands automatically.

    Args:
        xml_path: Path to MuJoCo XML scene file
        robot_urdf_path: Path to robot URDF file
        scale_factor: Scaling factor for hand tracking movements
        visualize_placo: Enable Placo visualization in browser
        hardware_ip: IP address of UR5e robot
        max_joint_velocity: Maximum joint velocity limit in rad/s (default: 1.0)
    """
    config = {
        "right_hand": {
            "link_name": "right_tool0",
            "pose_source": "right_hand_wrist",  # Hand tracking instead of controller
            "vis_target": "right_target",
            # No control_trigger needed - continuous hand tracking control
        },
        "left_hand": {
            "link_name": "left_tool0",
            "pose_source": "left_hand_wrist",  # Hand tracking instead of controller
            "vis_target": "left_target",
            # No control_trigger needed - continuous hand tracking control
        },
    }

    # Create and initialize the teleoperation controller
    controller = HardwareMujocoTeleopController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        hardware_ip=hardware_ip,
        max_joint_velocity=max_joint_velocity,
    )

    # additional constraints hardcoded here for now
    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
