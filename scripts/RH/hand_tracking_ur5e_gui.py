#!/usr/bin/env python3
"""
Hand Tracking UR5e Robot Control GUI

Controls a single UR5e robot arm using hand tracking from VR headset.
The robot continuously follows hand movements with real-time inverse kinematics.
"""
import time
import os
import sys
import tkinter as tk
from tkinter import ttk
import threading
import numpy as np
import webbrowser

# --- Setup Python Path for Imports ---
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, os.pardir, os.pardir))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Robot control
import rtde_control
import rtde_receive

# IK and visualization
import placo
import meshcat.transformations as tf
from placo_utils.visualization import robot_viz, frame_viz

# Project utilities
from xrobotoolkit_teleop.common.xr_client import XrClient
from xrobotoolkit_teleop.utils.geometry import (
    R_HEADSET_TO_WORLD,
    apply_delta_pose,
    quat_diff_as_angle_axis,
)

# --- Constants & Configuration ---
ROBOT_IP = "192.168.2.2"
URDF_PATH = os.path.join(project_root, "assets/universal_robots_ur5e/ur5e.urdf")

# Home position for single UR5e (radians)
HOME_Q = np.deg2rad([165.26, -47.50, 118.93, -38.96, 87.51, 149.56])

# Control parameters
SCALE_FACTOR = 1.2  # Hand movement to robot movement scaling
SERVO_TIME = 0.008  # 125Hz control loop
LOOKAHEAD_TIME = 0.1
SERVO_GAIN = 300.0
DEFAULT_MAX_VELOCITY = 0.5  # rad/s maximum joint speed (default)

# Thread update rates
HAND_TRACKING_RATE = 0.1  # 10Hz
IK_RATE = 0.05  # 20Hz
GUI_UPDATE_RATE = 0.1  # 10Hz


class HandTrackingUR5eController:
    def __init__(self, master):
        self.master = master
        master.title("Hand Tracking UR5e Controller")
        master.geometry("600x800")

        # Robot connection
        self.robot_ip = ROBOT_IP
        self.rtde_c = None
        self.rtde_r = None

        # XR and IK
        self.xr_client = None
        self.placo_robot = None
        self.solver = None
        self.placo_vis = None
        self.effector_task = None
        self.meshcat_url = None

        # Thread control
        self.running = True
        self.control_active = False
        self.emergency_stop = threading.Event()
        self.data_lock = threading.Lock()

        # Threads
        self.hand_tracking_thread = None
        self.ik_thread = None
        self.robot_control_thread = None

        # Hand tracking state
        self.hand_choice = tk.StringVar(value="right")
        self.hand_pose = None  # [x, y, z, qx, qy, qz, qw]
        self.tracking_active = False
        self.wrist_position = np.zeros(3)

        # IK anchor state (for continuous tracking)
        self.ref_hand_xyz = None
        self.ref_hand_quat = None
        self.ref_ee_xyz = None
        self.ref_ee_quat = None

        # Joint state
        self.target_joints = HOME_Q.copy()
        self.current_joints = np.zeros(6)
        self.smoothed_targets = HOME_Q.copy()

        # IK computation rate tracking
        self.ik_rate_hz = 0.0
        self.last_ik_time = time.time()

        # Speed control
        self.max_velocity = DEFAULT_MAX_VELOCITY

        # Create GUI
        self.create_widgets()

        # Initialize XR Client
        self.init_xr_client()

        # Start GUI update loop
        self.update_gui_display()

    def create_widgets(self):
        """Build the GUI layout"""
        # Connection frame
        conn_frame = ttk.LabelFrame(self.master, text="Connection", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text=f"Robot IP: {self.robot_ip}").pack(side="left", padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.connect_robot)
        self.connect_btn.pack(side="left", padx=5)

        self.disconnect_btn = ttk.Button(conn_frame, text="Disconnect",
                                         command=self.disconnect_robot, state="disabled")
        self.disconnect_btn.pack(side="left", padx=5)

        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.pack(side="left", padx=20)

        # Control frame
        ctrl_frame = ttk.LabelFrame(self.master, text="Control", padding=10)
        ctrl_frame.pack(fill="x", padx=10, pady=5)

        # Hand selection
        hand_frame = ttk.Frame(ctrl_frame)
        hand_frame.pack(fill="x", pady=5)
        ttk.Label(hand_frame, text="Hand:").pack(side="left", padx=5)
        ttk.Radiobutton(hand_frame, text="Left", variable=self.hand_choice,
                       value="left").pack(side="left", padx=5)
        ttk.Radiobutton(hand_frame, text="Right", variable=self.hand_choice,
                       value="right").pack(side="left", padx=5)

        # Control buttons
        btn_frame = ttk.Frame(ctrl_frame)
        btn_frame.pack(fill="x", pady=5)

        self.start_btn = ttk.Button(btn_frame, text="Start Control",
                                    command=self.start_control, state="disabled")
        self.start_btn.pack(side="left", padx=5)

        self.stop_btn = ttk.Button(btn_frame, text="Stop Control",
                                   command=self.stop_control, state="disabled")
        self.stop_btn.pack(side="left", padx=5)

        # Emergency stop button (RED, prominent)
        self.emergency_btn = ttk.Button(ctrl_frame, text="EMERGENCY STOP",
                                       command=self.emergency_stop_clicked)
        self.emergency_btn.pack(pady=10, fill="x")
        # Configure style for emergency button
        style = ttk.Style()
        style.configure("Emergency.TButton", foreground="red", font=("Arial", 12, "bold"))
        self.emergency_btn.configure(style="Emergency.TButton")

        # Meshcat button
        self.meshcat_btn = ttk.Button(ctrl_frame, text="Open Meshcat Viewer",
                                      command=self.open_meshcat_viewer, state="disabled")
        self.meshcat_btn.pack(pady=5, fill="x")

        # Speed control
        speed_frame = ttk.Frame(ctrl_frame)
        speed_frame.pack(fill="x", pady=10)

        ttk.Label(speed_frame, text="Max Speed:").pack(side="left", padx=5)

        self.speed_scale = tk.Scale(
            speed_frame,
            from_=0.05,
            to=2.0,
            orient="horizontal",
            resolution=0.05,
            length=200,
            command=self.on_speed_change
        )
        self.speed_scale.set(self.max_velocity)
        self.speed_scale.pack(side="left", padx=5)

        self.speed_label = ttk.Label(speed_frame, text=f"{self.max_velocity:.2f} rad/s")
        self.speed_label.pack(side="left", padx=5)

        # Hand tracking status frame
        tracking_frame = ttk.LabelFrame(self.master, text="Hand Tracking Status", padding=10)
        tracking_frame.pack(fill="x", padx=10, pady=5)

        self.tracking_status_label = ttk.Label(tracking_frame, text="Tracking: ● INACTIVE",
                                              foreground="red")
        self.tracking_status_label.pack(anchor="w", pady=2)

        self.wrist_pos_label = ttk.Label(tracking_frame,
                                        text="Wrist Position: [0.000, 0.000, 0.000]")
        self.wrist_pos_label.pack(anchor="w", pady=2)

        # Joint positions frame
        joints_frame = ttk.LabelFrame(self.master, text="Joint Positions", padding=10)
        joints_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.joint_labels = []
        for i in range(6):
            joint_frame = ttk.Frame(joints_frame)
            joint_frame.pack(fill="x", pady=2)

            label = ttk.Label(joint_frame,
                            text=f"Joint {i}: Current:   0.0°  Target:   0.0°",
                            font=("Courier", 10))
            label.pack(anchor="w")
            self.joint_labels.append(label)

        # IK solver status frame
        ik_frame = ttk.LabelFrame(self.master, text="IK Solver Status", padding=10)
        ik_frame.pack(fill="x", padx=10, pady=5)

        self.ik_rate_label = ttk.Label(ik_frame, text="IK Rate: 0.0 Hz")
        self.ik_rate_label.pack(anchor="w", pady=2)

        self.meshcat_url_label = ttk.Label(ik_frame, text="Meshcat: Not initialized")
        self.meshcat_url_label.pack(anchor="w", pady=2)

    def init_xr_client(self):
        """Initialize XR client (don't start thread yet - wait for user to start control)"""
        try:
            print("Initializing XrClient...")
            self.xr_client = XrClient()
            print("XrClient initialized successfully")
            print("Hand tracking thread will start when you click 'Start Control'")

        except Exception as e:
            print(f"Error initializing XrClient: {e}")
            import traceback
            traceback.print_exc()
            self.xr_client = None

    def connect_robot(self):
        """Connect to UR robot via RTDE"""
        try:
            self.status_label.config(text="Status: Connecting...", foreground="orange")
            self.master.update()

            # Create RTDE connections
            self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)

            # Read initial joint positions
            self.current_joints = np.array(self.rtde_r.getActualQ())

            # Initialize Placo solver
            self.init_placo_solver()

            # Initialize Meshcat visualization
            self.init_meshcat()

            self.status_label.config(text="Status: Connected", foreground="green")
            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")
            self.start_btn.config(state="normal")
            self.meshcat_btn.config(state="normal")

            print(f"Connected to UR robot at {self.robot_ip}")

        except Exception as e:
            self.status_label.config(text=f"Status: Error - {e}", foreground="red")
            print(f"Connection error: {e}")

    def disconnect_robot(self):
        """Disconnect from UR robot"""
        # Stop control if active
        if self.control_active:
            self.stop_control()

        # Close RTDE connections
        if self.rtde_c:
            try:
                self.rtde_c.disconnect()
            except:
                pass
            self.rtde_c = None

        if self.rtde_r:
            try:
                self.rtde_r.disconnect()
            except:
                pass
            self.rtde_r = None

        self.status_label.config(text="Status: Disconnected", foreground="red")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="disabled")
        self.meshcat_btn.config(state="disabled")

        print("Disconnected from robot")

    def init_placo_solver(self):
        """Initialize Placo IK solver"""
        print(f"[Placo] Loading URDF: {URDF_PATH}")

        # Load robot model
        self.placo_robot = placo.RobotWrapper(URDF_PATH)
        print(f"[Placo] Robot DOF: {self.placo_robot.model.nq}")

        # Create kinematics solver
        self.solver = placo.KinematicsSolver(self.placo_robot)
        self.solver.dt = 0.01  # 100Hz internal timestep
        self.solver.mask_fbase(True)  # Fix base (no floating base for single arm)

        # Set to current actual robot position (not home)
        current_q = np.array(self.rtde_r.getActualQ())
        print(f"[Placo] Current robot joints (deg): {np.rad2deg(current_q)}")

        # Update placo state to match actual robot
        self.placo_robot.state.q[:6] = current_q
        self.target_joints = current_q.copy()  # Initialize targets to current position
        self.placo_robot.update_kinematics()

        # Add regularization task (keeps robot near comfortable pose)
        self.solver.add_kinetic_energy_regularization_task(1e-6)

        # Create end-effector frame task
        T_current = self.placo_robot.get_T_world_frame("tool0")
        print(f"[Placo] Initial EE position: {T_current[:3, 3]}")
        self.effector_task = self.solver.add_frame_task("tool0", T_current)
        self.effector_task.configure("eef", "soft", 1.0)

        print("[Placo] IK solver initialized successfully")

        # Start IK computation thread
        self.ik_thread = threading.Thread(
            target=self.ik_computation_worker,
            daemon=True
        )
        self.ik_thread.start()
        print("IK computation thread started")

    def init_meshcat(self):
        """Initialize Meshcat visualization"""
        try:
            self.placo_vis = robot_viz(self.placo_robot)
            self.meshcat_url = self.placo_vis.viewer.url()

            # Initial display
            self.placo_vis.display(self.placo_robot.state.q)

            # Add target frame visualization
            frame_viz("target_frame", np.eye(4))

            self.meshcat_url_label.config(text=f"Meshcat: {self.meshcat_url}")
            print(f"Meshcat viewer initialized at: {self.meshcat_url}")

        except Exception as e:
            print(f"Error initializing Meshcat: {e}")
            self.meshcat_url = None

    def hand_tracking_worker(self):
        """Thread 1: Collect hand tracking data at 10Hz"""
        print("Hand tracking worker started")
        last_tracking_state = None

        # Small delay to ensure XrClient is ready
        time.sleep(0.5)

        while self.running and self.xr_client:
            try:
                # Get hand wrist pose based on user selection
                pose_name = f"{self.hand_choice.get()}_hand_wrist"

                # Safely get pose with exception handling
                try:
                    hand_pose = self.xr_client.get_pose_by_name(pose_name)
                except Exception as e:
                    print(f"[HandTrack] Error getting pose: {e}")
                    hand_pose = None

                with self.data_lock:
                    if hand_pose is not None:
                        # Hand is being tracked
                        self.hand_pose = hand_pose
                        self.wrist_position = np.array(hand_pose[:3])
                        self.tracking_active = True

                        # Debug: Print when tracking starts
                        if last_tracking_state != True:
                            print(f"[HandTrack] ACTIVE - {pose_name} detected")
                            last_tracking_state = True
                    else:
                        # Hand tracking lost - freeze at last known pose
                        if last_tracking_state != False:
                            print(f"[HandTrack] LOST - {pose_name} not detected")
                            last_tracking_state = False
                        self.tracking_active = False
                        # Don't clear hand_pose - keep last known position

                time.sleep(HAND_TRACKING_RATE)

            except Exception as e:
                print(f"Hand tracking error: {e}")
                with self.data_lock:
                    self.tracking_active = False
                time.sleep(HAND_TRACKING_RATE)

        print("Hand tracking worker stopped")

    def ik_computation_worker(self):
        """Thread 2: Compute IK at 20Hz"""
        print("IK computation worker started")

        while self.running and self.placo_robot:
            try:
                ik_start_time = time.time()

                if self.emergency_stop.is_set():
                    time.sleep(IK_RATE)
                    continue

                # Read hand pose from tracking thread
                with self.data_lock:
                    hand_pose = self.hand_pose
                    tracking = self.tracking_active

                if hand_pose is None:
                    print("[IK] Hand pose is None")
                    self.ref_hand_xyz = None
                    self.ref_hand_quat = None
                    time.sleep(IK_RATE)
                    continue

                if not tracking:
                    print("[IK] Hand tracking not active")
                    # Reset anchor when tracking lost
                    self.ref_hand_xyz = None
                    self.ref_hand_quat = None
                    time.sleep(IK_RATE)
                    continue

                # Transform hand pose from headset space to world space
                pos = np.array(hand_pose[:3])
                quat_input = np.array([hand_pose[6], hand_pose[3], hand_pose[4], hand_pose[5]])  # w,x,y,z

                # Apply headset-to-world rotation to position
                pos_world = R_HEADSET_TO_WORLD @ pos

                # Apply headset-to-world rotation to quaternion
                R_transform = np.eye(4)
                R_transform[:3, :3] = R_HEADSET_TO_WORLD
                R_quat = tf.quaternion_from_matrix(R_transform)
                quat_world = tf.quaternion_multiply(
                    tf.quaternion_multiply(R_quat, quat_input),
                    tf.quaternion_conjugate(R_quat)
                )

                # Anchor on first valid frame
                if self.ref_hand_xyz is None:
                    self.ref_hand_xyz = pos_world.copy()
                    self.ref_hand_quat = quat_world.copy()

                    # Get current robot end-effector pose
                    T_ee = self.placo_robot.get_T_world_frame("tool0")
                    self.ref_ee_xyz = T_ee[:3, 3].copy()
                    self.ref_ee_quat = tf.quaternion_from_matrix(T_ee)

                    print(f"[IK] ANCHORED - Hand pos: {pos_world}, EE pos: {self.ref_ee_xyz}")

                # Calculate delta movement from anchor point
                delta_xyz = (pos_world - self.ref_hand_xyz) * SCALE_FACTOR
                delta_rot = quat_diff_as_angle_axis(self.ref_hand_quat, quat_world)

                # Apply delta to initial robot pose
                target_xyz, target_quat = apply_delta_pose(
                    self.ref_ee_xyz, self.ref_ee_quat, delta_xyz, delta_rot
                )

                # Update IK task target
                T_target = tf.quaternion_matrix(target_quat)
                T_target[:3, 3] = target_xyz
                self.effector_task.T_world_frame = T_target

                # Solve IK
                self.solver.solve(True)

                # Extract joint solution
                new_targets = self.placo_robot.state.q[:6].copy()

                # Check for NaN values
                if np.any(np.isnan(new_targets)):
                    print(f"[IK] ERROR: Got NaN values in joint solution!")
                    print(f"[IK] Target EE pos: {target_xyz}")
                    print(f"[IK] Delta: {delta_xyz}")
                    # Don't update targets if we got NaN
                    time.sleep(IK_RATE)
                    continue

                # Debug: Print delta and targets periodically
                if int(time.time() * 2) % 10 == 0:  # Every ~5 seconds
                    print(f"[IK] Delta XYZ: {delta_xyz}, Target joints (deg): {np.rad2deg(new_targets)}")

                with self.data_lock:
                    self.target_joints = new_targets

                # Update meshcat visualization
                if self.placo_vis:
                    self.placo_vis.display(self.placo_robot.state.q)
                    frame_viz("target_frame", T_target)

                # Update IK rate
                ik_duration = time.time() - ik_start_time
                self.ik_rate_hz = 1.0 / max(ik_duration, 0.001)

                time.sleep(IK_RATE)

            except Exception as e:
                print(f"IK computation error: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(IK_RATE)

        print("IK computation worker stopped")

    def robot_control_worker(self):
        """Thread 3: Send commands to robot at 125Hz"""
        print("[RobotCtrl] Robot control worker started")
        iteration = 0

        while self.running and self.control_active:
            iteration += 1
            try:
                if self.emergency_stop.is_set():
                    # Emergency stop - halt immediately
                    print("Emergency stop activated in control loop")
                    self.rtde_c.servoStop()
                    break

                # Read current joint positions from robot
                current = np.array(self.rtde_r.getActualQ())

                # Read target joint positions from IK thread
                with self.data_lock:
                    self.current_joints = current.copy()
                    targets = self.target_joints.copy()

                # Debug: Print targets periodically
                if iteration % 1250 == 0:  # Every ~10 seconds at 125Hz
                    print(f"[RobotCtrl] Current (deg): {np.rad2deg(current)}")
                    print(f"[RobotCtrl] Targets (deg): {np.rad2deg(targets)}")

                # Apply velocity limiting for safety
                for i in range(6):
                    delta = targets[i] - self.smoothed_targets[i]
                    max_delta = self.max_velocity * SERVO_TIME

                    # Clamp delta to max speed
                    if abs(delta) > max_delta:
                        delta = np.sign(delta) * max_delta

                    self.smoothed_targets[i] += delta

                # Send servo command with timing
                t_start = self.rtde_c.initPeriod()
                self.rtde_c.servoJ(
                    self.smoothed_targets.tolist(),
                    0,  # velocity (not used in servoJ)
                    0,  # acceleration (not used in servoJ)
                    SERVO_TIME,
                    LOOKAHEAD_TIME,
                    SERVO_GAIN
                )
                self.rtde_c.waitPeriod(t_start)

            except Exception as e:
                print(f"Robot control error: {e}")
                import traceback
                traceback.print_exc()
                self.control_active = False
                break

        print("Robot control worker stopped")

    def start_control(self):
        """Start robot control"""
        if not self.rtde_c or not self.rtde_r:
            print("Error: Robot not connected")
            return

        if not self.placo_robot or not self.solver:
            print("Error: Placo IK not initialized")
            return

        if not self.xr_client:
            print("Error: XrClient not initialized")
            return

        # Start hand tracking thread if not already running
        if self.hand_tracking_thread is None or not self.hand_tracking_thread.is_alive():
            print("Starting hand tracking thread...")
            self.hand_tracking_thread = threading.Thread(
                target=self.hand_tracking_worker,
                daemon=True
            )
            self.hand_tracking_thread.start()
            # Give it a moment to start
            time.sleep(0.5)

        # Initialize smoothed targets to current position
        current = np.array(self.rtde_r.getActualQ())
        self.smoothed_targets = current.copy()

        # Clear emergency stop flag
        self.emergency_stop.clear()

        # Reset IK anchors (will re-anchor when hand detected)
        self.ref_hand_xyz = None
        self.ref_hand_quat = None

        # Set control active flag
        self.control_active = True

        # Start robot control thread
        self.robot_control_thread = threading.Thread(
            target=self.robot_control_worker,
            daemon=True
        )
        self.robot_control_thread.start()

        # Update GUI
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.status_label.config(text="Status: Control Active", foreground="green")

        print("Robot control started")

    def stop_control(self):
        """Stop robot control"""
        # Set control inactive flag
        self.control_active = False

        # Wait for robot control thread to finish
        if self.robot_control_thread and self.robot_control_thread.is_alive():
            self.robot_control_thread.join(timeout=1.0)

        # Stop servo mode
        if self.rtde_c:
            try:
                self.rtde_c.servoStop()
            except:
                pass

        # Reset IK anchors
        self.ref_hand_xyz = None
        self.ref_hand_quat = None

        # Update GUI
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.status_label.config(text="Status: Connected", foreground="green")

        print("Robot control stopped")

    def emergency_stop_clicked(self):
        """Emergency stop button handler"""
        print("EMERGENCY STOP PRESSED")

        # Set emergency stop flag
        self.emergency_stop.set()

        # Stop control
        self.stop_control()

        # Update status
        self.status_label.config(text="Status: EMERGENCY STOPPED", foreground="red")

    def on_speed_change(self, value):
        """Speed slider callback"""
        self.max_velocity = float(value)
        self.speed_label.config(text=f"{self.max_velocity:.2f} rad/s")

    def open_meshcat_viewer(self):
        """Open meshcat in web browser"""
        if self.meshcat_url:
            webbrowser.open(self.meshcat_url)

    def update_gui_display(self):
        """Main thread: Update GUI displays at 10Hz"""
        if not self.running:
            return

        try:
            # Read shared data with lock
            with self.data_lock:
                tracking = self.tracking_active
                wrist_pos = self.wrist_position.copy()
                current_joints = self.current_joints.copy()
                target_joints = self.target_joints.copy()

            # Update hand tracking status
            if tracking:
                self.tracking_status_label.config(
                    text="Tracking: ● ACTIVE",
                    foreground="green"
                )
                self.wrist_pos_label.config(
                    text=f"Wrist Position: [{wrist_pos[0]:.3f}, {wrist_pos[1]:.3f}, {wrist_pos[2]:.3f}]"
                )
            else:
                self.tracking_status_label.config(
                    text="Tracking: ○ INACTIVE",
                    foreground="red"
                )

            # Update joint positions (convert radians to degrees for display)
            for i in range(6):
                current_deg = np.rad2deg(current_joints[i])
                target_deg = np.rad2deg(target_joints[i])
                self.joint_labels[i].config(
                    text=f"Joint {i}: Current: {current_deg:6.1f}°  Target: {target_deg:6.1f}°"
                )

            # Update IK rate
            self.ik_rate_label.config(text=f"IK Rate: {self.ik_rate_hz:.1f} Hz")

        except Exception as e:
            print(f"GUI update error: {e}")

        # Schedule next update
        self.master.after(int(GUI_UPDATE_RATE * 1000), self.update_gui_display)

    def cleanup(self):
        """Cleanup all resources"""
        print("Cleaning up...")

        # Set running flag to false to stop all threads
        self.running = False

        # Stop control if active
        if self.control_active:
            self.stop_control()

        # Wait for threads to finish
        if self.hand_tracking_thread and self.hand_tracking_thread.is_alive():
            self.hand_tracking_thread.join(timeout=1.0)

        if self.ik_thread and self.ik_thread.is_alive():
            self.ik_thread.join(timeout=1.0)

        if self.robot_control_thread and self.robot_control_thread.is_alive():
            self.robot_control_thread.join(timeout=1.0)

        # Close XR client
        if self.xr_client:
            try:
                self.xr_client.close()
            except:
                pass
            self.xr_client = None

        # Close robot connections
        if self.rtde_c:
            try:
                self.rtde_c.disconnect()
            except:
                pass
            self.rtde_c = None

        if self.rtde_r:
            try:
                self.rtde_r.disconnect()
            except:
                pass
            self.rtde_r = None

        print("Cleanup complete")

    def on_closing(self):
        """Window close handler"""
        self.cleanup()
        self.master.destroy()


def main():
    root = tk.Tk()
    app = HandTrackingUR5eController(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
