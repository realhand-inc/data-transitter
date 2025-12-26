import json
import threading
import time
from typing import Any, Dict, Optional

import numpy as np
import zmq


class ZmqXrClient:
    """ZMQ-backed XR client receiving XR data snapshots over PUB/SUB."""

    def __init__(self, endpoint: str = "tcp://127.0.0.1:5558", poll_timeout_ms: int = 100):
        self.endpoint = endpoint
        self._poll_timeout_ms = poll_timeout_ms
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.connect(endpoint)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")

        self._lock = threading.Lock()
        self._latest: Dict[str, Any] = {}
        self._running = True
        self.last_receive_ts = 0.0
        self.recv_count = 0
        self._log_every = 50
        self._log_all = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def _recv_loop(self) -> None:
        poller = zmq.Poller()
        poller.register(self._socket, zmq.POLLIN)

        while self._running:
            events = dict(poller.poll(self._poll_timeout_ms))
            if self._socket not in events:
                continue
            try:
                msg = self._socket.recv_string()
                payload = json.loads(msg)
            except Exception:
                continue

            parsed = self._parse_payload(payload)
            if not parsed:
                continue

            with self._lock:
                self._latest = parsed
                self.last_receive_ts = time.time()
                self.recv_count += 1
                if self._log_all:
                    self._log_full_snapshot(parsed)
                elif self.recv_count == 1 or self.recv_count % self._log_every == 0:
                    print(
                        f"[ZMQ XR] recv {self.recv_count} from {self.endpoint} "
                        f"(headset={'ok' if parsed.get('headset_pose') is not None else 'none'})"
                    )

    def _log_full_snapshot(self, parsed: Dict[str, Any]) -> None:
        headset = parsed.get("headset_pose")
        left_hand = parsed.get("left_hand", {})
        right_hand = parsed.get("right_hand", {})
        left_joints = left_hand.get("joints")
        right_joints = right_hand.get("joints")
        print(f"[ZMQ XR] recv {self.recv_count} from {self.endpoint}")
        print(f"  headset_pose: {headset}")
        print(f"  left_hand_active: {left_hand.get('is_active')} joints: {left_joints}")
        print(f"  right_hand_active: {right_hand.get('is_active')} joints: {right_joints}")

    def _parse_payload(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if "headset_pose" in payload:
            return self._parse_raw_payload(payload)

        headset = payload.get("headset", {})
        left_ctrl = payload.get("leftController", {})
        right_ctrl = payload.get("rightController", {})
        left_hand = payload.get("leftHand", {})
        right_hand = payload.get("rightHand", {})

        parsed = {
            "headset_pose": self._parse_pose(headset.get("pose_raw") or headset.get("pose"), headset.get("status")),
            "left_controller_pose": self._parse_pose(left_ctrl.get("pose_raw") or left_ctrl.get("pose")),
            "right_controller_pose": self._parse_pose(right_ctrl.get("pose_raw") or right_ctrl.get("pose")),
            "left_controller": left_ctrl,
            "right_controller": right_ctrl,
            "left_hand": self._parse_hand(left_hand),
            "right_hand": self._parse_hand(right_hand),
            "timestamp_ns": headset.get("timeStampNs")
            or payload.get("timeStampNs")
            or payload.get("timestamp_ns"),
        }
        return parsed

    def _parse_raw_payload(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        left_joints = payload.get("left_hand_joints")
        right_joints = payload.get("right_hand_joints")
        parsed = {
            "headset_pose": self._parse_pose(payload.get("headset_pose")),
            "left_controller_pose": self._parse_pose(payload.get("left_controller_pose")),
            "right_controller_pose": self._parse_pose(payload.get("right_controller_pose")),
            "left_controller": {},
            "right_controller": {},
            "left_hand": {
                "is_active": left_joints is not None,
                "joints": self._parse_joint_list(left_joints),
                "timestamp_ns": payload.get("timestamp_ns"),
            },
            "right_hand": {
                "is_active": right_joints is not None,
                "joints": self._parse_joint_list(right_joints),
                "timestamp_ns": payload.get("timestamp_ns"),
            },
            "timestamp_ns": payload.get("timestamp_ns"),
        }
        return parsed

    def _parse_pose(self, value: Any, status: Any = None) -> Optional[np.ndarray]:
        if status == 0:
            return None
        if value is None:
            return None
        if isinstance(value, (list, tuple, np.ndarray)):
            return np.array(value, dtype=float)
        if isinstance(value, str):
            parts = [p.strip() for p in value.split(",") if p.strip()]
            if not parts:
                return None
            return np.array([float(p) for p in parts], dtype=float)
        return None

    def _parse_joint_list(self, joints: Any) -> Optional[np.ndarray]:
        if joints is None:
            return None
        if isinstance(joints, np.ndarray):
            return joints.astype(float)
        if isinstance(joints, list):
            if not joints:
                return np.zeros((0, 7), dtype=float)
            if isinstance(joints[0], (list, tuple, np.ndarray)):
                return np.array(joints, dtype=float)
            if isinstance(joints[0], str):
                parsed = [self._parse_pose(j) for j in joints]
                parsed = [p for p in parsed if p is not None]
                return np.array(parsed, dtype=float) if parsed else None
        return None

    def _parse_hand(self, hand_data: Dict[str, Any]) -> Dict[str, Any]:
        is_active = bool(hand_data.get("isActive"))
        joints_raw = hand_data.get("joints_raw")
        joints = None
        if is_active:
            joints = self._parse_joint_list(joints_raw or hand_data.get("joints"))
        return {
            "is_active": is_active,
            "joints": joints,
            "timestamp_ns": hand_data.get("timeStampNs"),
        }

    def _get_latest(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._latest) if self._latest else {}

    def get_pose_by_name(self, name: str) -> Optional[np.ndarray]:
        data = self._get_latest()
        if name == "left_controller":
            return data.get("left_controller_pose")
        if name == "right_controller":
            return data.get("right_controller_pose")
        if name == "headset":
            return data.get("headset_pose")
        if name == "left_hand_wrist":
            hand = data.get("left_hand", {})
            joints = hand.get("joints")
            if joints is None or len(joints) <= 1:
                raise ValueError("Left hand tracking inactive (no joints).")
            return joints[1]
        if name == "right_hand_wrist":
            hand = data.get("right_hand", {})
            joints = hand.get("joints")
            if joints is None or len(joints) <= 1:
                raise ValueError("Right hand tracking inactive (no joints).")
            return joints[1]
        raise ValueError(
            f"Invalid name: {name}. Valid names are: 'left_controller', "
            "'right_controller', 'headset', 'left_hand_wrist', 'right_hand_wrist'."
        )

    def get_key_value_by_name(self, name: str) -> float:
        data = self._get_latest()
        if name == "left_trigger":
            return float(data.get("left_controller", {}).get("trigger", 0.0))
        if name == "right_trigger":
            return float(data.get("right_controller", {}).get("trigger", 0.0))
        if name == "left_grip":
            return float(data.get("left_controller", {}).get("grip", 0.0))
        if name == "right_grip":
            return float(data.get("right_controller", {}).get("grip", 0.0))
        raise ValueError(
            f"Invalid name: {name}. Valid names are: 'left_trigger', 'right_trigger', 'left_grip', 'right_grip'."
        )

    def get_button_state_by_name(self, name: str) -> bool:
        data = self._get_latest()
        left = data.get("left_controller", {})
        right = data.get("right_controller", {})
        if name == "A":
            return bool(right.get("primaryButton", False))
        if name == "B":
            return bool(right.get("secondaryButton", False))
        if name == "X":
            return bool(left.get("primaryButton", False))
        if name == "Y":
            return bool(left.get("secondaryButton", False))
        if name == "left_menu_button":
            return bool(left.get("menuButton", False))
        if name == "right_menu_button":
            return bool(right.get("menuButton", False))
        if name == "left_axis_click":
            return bool(left.get("axisClick", False))
        if name == "right_axis_click":
            return bool(right.get("axisClick", False))
        raise ValueError(
            f"Invalid name: {name}. Valid names are: 'A', 'B', 'X', 'Y', "
            "'left_menu_button', 'right_menu_button', 'left_axis_click', 'right_axis_click'."
        )

    def get_timestamp_ns(self) -> int:
        data = self._get_latest()
        ts = data.get("timestamp_ns")
        return int(ts) if ts is not None else 0

    def get_hand_tracking_state(self, hand: str) -> Optional[np.ndarray]:
        data = self._get_latest()
        if hand.lower() == "left":
            joints = data.get("left_hand", {}).get("joints")
        elif hand.lower() == "right":
            joints = data.get("right_hand", {}).get("joints")
        else:
            raise ValueError(f"Invalid hand: {hand}. Valid hands are: 'left', 'right'.")
        return joints if joints is not None and len(joints) else None

    def get_full_hand_state(self, hand: str) -> Optional[Dict[str, Any]]:
        joints = self.get_hand_tracking_state(hand)
        if joints is None:
            return None
        return {
            "joints": joints,
            "timestamp_ns": self.get_timestamp_ns(),
            "is_active": True,
        }

    def get_joystick_state(self, controller: str) -> list[float]:
        data = self._get_latest()
        if controller.lower() == "left":
            return [
                float(data.get("left_controller", {}).get("axisX", 0.0)),
                float(data.get("left_controller", {}).get("axisY", 0.0)),
            ]
        if controller.lower() == "right":
            return [
                float(data.get("right_controller", {}).get("axisX", 0.0)),
                float(data.get("right_controller", {}).get("axisY", 0.0)),
            ]
        raise ValueError(f"Invalid controller: {controller}. Valid controllers are: 'left', 'right'.")

    def get_motion_tracker_data(self) -> dict:
        return {}

    def close(self) -> None:
        self._running = False
        try:
            self._socket.close()
        except Exception:
            pass
