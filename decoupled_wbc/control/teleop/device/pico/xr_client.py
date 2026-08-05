import json
import os
import socket
import struct
import threading
import time
from typing import Any

import numpy as np

# xrt interface is defined in
# https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind/blob/main/bindings/py_bindings.cpp

_PACKET_HEAD_FROM_UNITY = 0x3F
_PACKET_END = 0xA5
_PACKET_TO_CONTROLLER_FUNCTION = 0x6D

_DEFAULT_UDP_HOST = "0.0.0.0"
_DEFAULT_UDP_PORT = 63901
_DEFAULT_UDP_RCVBUF = 4 * 1024 * 1024
_DEFAULT_UDP_RECV_BYTES = 1024 * 1024

_IDENTITY_POSE = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)
_ZERO6 = np.zeros(6, dtype=np.float64)


def _int_from_env(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None:
        return default

    try:
        return int(value)
    except ValueError:
        return default


class XrClient:
    """Client for XR data.

    By default this receives low-latency Mocopi/XRoboToolkitCompat UDP frames from Unity.
    Set XROBO_TRANSPORT=sdk to use the original XRoboToolkit PC Service SDK backend.
    """

    def __init__(
        self,
        transport: str | None = None,
        udp_host: str | None = None,
        udp_port: int | None = None,
        udp_rcvbuf: int | None = None,
    ):
        self.transport = (transport or os.environ.get("XROBO_TRANSPORT", "udp")).lower()
        self._xrt = None

        self._udp_socket: socket.socket | None = None
        self._udp_thread: threading.Thread | None = None
        self._udp_running = False
        self._udp_lock = threading.Lock()
        self._latest_tracking: dict[str, Any] | None = None
        self._latest_sender: tuple[str, int] | None = None
        self._latest_monotonic_time = 0.0

        if self.transport == "sdk":
            self._init_sdk()
        elif self.transport == "udp":
            self._init_udp(
                udp_host or os.environ.get("XROBO_UDP_HOST", _DEFAULT_UDP_HOST),
                udp_port if udp_port is not None else _int_from_env("XROBO_UDP_PORT", _DEFAULT_UDP_PORT),
                udp_rcvbuf
                if udp_rcvbuf is not None
                else _int_from_env("XROBO_UDP_RCVBUF", _DEFAULT_UDP_RCVBUF),
            )
        else:
            raise ValueError("Invalid XROBO_TRANSPORT. Expected 'udp' or 'sdk'.")

    @property
    def uses_sdk_service(self) -> bool:
        return self.transport == "sdk"

    def _init_sdk(self):
        import xrobotoolkit_sdk as xrt

        self._xrt = xrt
        self._xrt.init()
        print("XRoboToolkit SDK initialized.")

    def _init_udp(self, host: str, port: int, rcvbuf: int):
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        udp_socket.settimeout(0.1)
        udp_socket.bind((host, port))

        actual_rcvbuf = udp_socket.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        self._udp_socket = udp_socket
        self._udp_running = True
        self._udp_thread = threading.Thread(
            target=self._udp_receive_loop,
            name="XRoboToolkit UDP Receiver",
            daemon=True,
        )
        self._udp_thread.start()
        print(
            "XRoboToolkit UDP receiver listening on "
            f"{host}:{port} (SO_RCVBUF requested={rcvbuf}, actual={actual_rcvbuf})."
        )

    def _udp_receive_loop(self):
        assert self._udp_socket is not None
        while self._udp_running:
            try:
                data, sender = self._udp_socket.recvfrom(_DEFAULT_UDP_RECV_BYTES)
            except socket.timeout:
                continue
            except OSError:
                break

            tracking = self._try_parse_tracking_packet(data)
            if tracking is None:
                continue

            with self._udp_lock:
                self._latest_tracking = tracking
                self._latest_sender = sender
                self._latest_monotonic_time = time.monotonic()

    def _try_parse_tracking_packet(self, data: bytes) -> dict[str, Any] | None:
        if len(data) < 15:
            return None

        if data[0] != _PACKET_HEAD_FROM_UNITY:
            return None

        body_len = struct.unpack_from("<i", data, 2)[0]
        total_len = 15 + body_len
        if body_len < 0 or len(data) < total_len:
            return None

        if data[total_len - 1] != _PACKET_END:
            return None

        cmd = data[1]
        if cmd != _PACKET_TO_CONTROLLER_FUNCTION:
            return None

        timestamp_ms = struct.unpack_from("<q", data, 6 + body_len)[0]
        body = data[6 : 6 + body_len].decode("utf-8", errors="replace")
        tracking = self._try_parse_tracking_body(body)
        if tracking is None:
            return None

        tracking["_packet_timestamp_ms"] = timestamp_ms
        return tracking

    @staticmethod
    def _try_parse_tracking_body(body: str) -> dict[str, Any] | None:
        try:
            outer = json.loads(body)
        except json.JSONDecodeError:
            outer = None

        if isinstance(outer, dict):
            function_name = outer.get("functionName")
            value = outer.get("value")
            if function_name and function_name != "Tracking":
                return None
        else:
            value = body

        if isinstance(value, dict):
            return value

        if not isinstance(value, str):
            return None

        try:
            tracking = json.loads(value)
        except json.JSONDecodeError:
            return None

        return tracking if isinstance(tracking, dict) else None

    def _get_latest_tracking(self) -> dict[str, Any] | None:
        with self._udp_lock:
            return self._latest_tracking

    def get_latest_receive_monotonic_time(self) -> float | None:
        """Return when the latest valid UDP tracking packet was received.

        The SDK transport does not expose a host receive timestamp.  UDP does,
        and callers that enforce input freshness should prefer this value over
        an optional device timestamp embedded in the JSON payload.
        """
        if self.transport == "sdk":
            return None

        with self._udp_lock:
            received_at = self._latest_monotonic_time
        return received_at if received_at > 0.0 else None

    def get_scene_reset_input_snapshot(self) -> dict[str, Any] | None:
        """Atomically return the complete UDP input required by the reset gate.

        Missing buttons or axes invalidate the whole snapshot.  In particular,
        absent joystick fields must not be interpreted as centered sticks.
        """
        if self.transport == "sdk":
            return None

        with self._udp_lock:
            tracking = self._latest_tracking
            received_at = self._latest_monotonic_time
            if tracking is None or received_at <= 0.0:
                return None

            controller = tracking.get("Controller")
            if not isinstance(controller, dict):
                return None
            left = controller.get("left")
            right = controller.get("right")
            if not isinstance(left, dict) or not isinstance(right, dict):
                return None

            required_left = ("primaryButton", "secondaryButton", "axisX", "axisY")
            required_right = ("primaryButton", "secondaryButton", "axisX", "axisY")
            if not all(key in left for key in required_left) or not all(
                key in right for key in required_right
            ):
                return None

            try:
                def parse_button(value: Any) -> bool:
                    if isinstance(value, bool):
                        return value
                    if isinstance(value, (int, float)) and value in (0, 1):
                        return bool(value)
                    raise ValueError("invalid controller button value")

                axes = (
                    float(left["axisX"]),
                    float(left["axisY"]),
                    float(right["axisX"]),
                    float(right["axisY"]),
                )
                a_pressed = parse_button(right["primaryButton"])
                b_pressed = parse_button(right["secondaryButton"])
                x_pressed = parse_button(left["primaryButton"])
                y_pressed = parse_button(left["secondaryButton"])
            except (TypeError, ValueError):
                return None

            return {
                "a_pressed": a_pressed,
                "b_pressed": b_pressed,
                "x_pressed": x_pressed,
                "y_pressed": y_pressed,
                "axes": axes,
                "received_monotonic": received_at,
            }

    def get_pose_by_name(self, name: str) -> np.ndarray:
        """Returns pose [x, y, z, qx, qy, qz, qw] by name."""
        if self.transport == "sdk":
            if name == "left_controller":
                return self._xrt.get_left_controller_pose()
            if name == "right_controller":
                return self._xrt.get_right_controller_pose()
            if name == "headset":
                return self._xrt.get_headset_pose()
            raise ValueError(
                f"Invalid name: {name}. Valid names are: 'left_controller', 'right_controller', 'headset'."
            )

        if name == "left_controller":
            return self._get_controller_or_body_pose("left", 20)
        if name == "right_controller":
            return self._get_controller_or_body_pose("right", 21)
        if name == "headset":
            return self._get_head_pose()

        raise ValueError(
            f"Invalid name: {name}. Valid names are: 'left_controller', 'right_controller', 'headset'."
        )

    def _get_controller_or_body_pose(self, side: str, body_joint_index: int) -> np.ndarray:
        tracking = self._get_latest_tracking()
        if tracking is None:
            return _IDENTITY_POSE.copy()

        controller_pose = self._get_controller_pose(tracking, side)
        if controller_pose is not None and not self._is_identity_pose(controller_pose):
            return controller_pose

        body_pose = self._get_body_joint_pose(tracking, body_joint_index)
        if body_pose is not None:
            return body_pose

        return controller_pose if controller_pose is not None else _IDENTITY_POSE.copy()

    def _get_head_pose(self) -> np.ndarray:
        tracking = self._get_latest_tracking()
        if tracking is None:
            return _IDENTITY_POSE.copy()

        head = tracking.get("Head")
        if isinstance(head, dict):
            head_pose = self._parse_vector(head.get("pose"), 7, _IDENTITY_POSE)
            if head_pose is not None:
                return head_pose

        body_head_pose = self._get_body_joint_pose(tracking, 15)
        return body_head_pose if body_head_pose is not None else _IDENTITY_POSE.copy()

    @staticmethod
    def _get_controller_pose(tracking: dict[str, Any], side: str) -> np.ndarray | None:
        controller = tracking.get("Controller")
        if not isinstance(controller, dict):
            return None

        side_data = controller.get(side)
        if not isinstance(side_data, dict):
            return None

        return XrClient._parse_vector(side_data.get("pose"), 7, _IDENTITY_POSE)

    @staticmethod
    def _get_body_joint_pose(tracking: dict[str, Any], index: int) -> np.ndarray | None:
        joint = XrClient._get_body_joint(tracking, index)
        if joint is None:
            return None

        return XrClient._parse_vector(joint.get("p"), 7, _IDENTITY_POSE)

    @staticmethod
    def _get_body_joint(tracking: dict[str, Any], index: int) -> dict[str, Any] | None:
        body = tracking.get("Body")
        if not isinstance(body, dict):
            return None

        joints = body.get("joints")
        if not isinstance(joints, list) or index < 0 or index >= len(joints):
            return None

        joint = joints[index]
        return joint if isinstance(joint, dict) else None

    @staticmethod
    def _is_identity_pose(pose: np.ndarray) -> bool:
        return bool(np.allclose(pose, _IDENTITY_POSE, atol=1e-8))

    @staticmethod
    def _parse_vector(value: Any, expected_len: int, default: np.ndarray) -> np.ndarray | None:
        if value is None:
            return default.copy()

        if isinstance(value, str):
            parts = [part.strip() for part in value.split(",") if part.strip()]
        elif isinstance(value, (list, tuple, np.ndarray)):
            parts = list(value)
        else:
            return default.copy()

        try:
            parsed = [float(part) for part in parts[:expected_len]]
        except (TypeError, ValueError):
            return default.copy()

        if len(parsed) < expected_len:
            padded = default.astype(np.float64).copy()
            padded[: len(parsed)] = parsed
            return padded

        return np.asarray(parsed, dtype=np.float64)

    def get_key_value_by_name(self, name: str) -> float:
        """Returns the trigger/grip value by name."""
        if self.transport == "sdk":
            if name == "left_trigger":
                return self._xrt.get_left_trigger()
            if name == "right_trigger":
                return self._xrt.get_right_trigger()
            if name == "left_grip":
                return self._xrt.get_left_grip()
            if name == "right_grip":
                return self._xrt.get_right_grip()
            raise ValueError(
                f"Invalid name: {name}. Valid names are: "
                "'left_trigger', 'right_trigger', 'left_grip', 'right_grip'."
            )

        mapping = {
            "left_trigger": ("left", "trigger"),
            "right_trigger": ("right", "trigger"),
            "left_grip": ("left", "grip"),
            "right_grip": ("right", "grip"),
        }
        if name not in mapping:
            raise ValueError(
                f"Invalid name: {name}. Valid names are: "
                "'left_trigger', 'right_trigger', 'left_grip', 'right_grip'."
            )

        side, key = mapping[name]
        return float(self._get_controller_value(side, key, 0.0))

    def get_button_state_by_name(self, name: str) -> bool:
        """Returns the button state by name."""
        if self.transport == "sdk":
            if name == "A":
                return self._xrt.get_A_button()
            if name == "B":
                return self._xrt.get_B_button()
            if name == "X":
                return self._xrt.get_X_button()
            if name == "Y":
                return self._xrt.get_Y_button()
            if name == "left_menu_button":
                return self._xrt.get_left_menu_button()
            if name == "right_menu_button":
                return self._xrt.get_right_menu_button()
            if name == "left_axis_click":
                return self._xrt.get_left_axis_click()
            if name == "right_axis_click":
                return self._xrt.get_right_axis_click()
            raise ValueError(
                f"Invalid name: {name}. Valid names are: 'A', 'B', 'X', 'Y', "
                "'left_menu_button', 'right_menu_button', 'left_axis_click', 'right_axis_click'."
            )

        mapping = {
            "A": ("right", "primaryButton"),
            "B": ("right", "secondaryButton"),
            "X": ("left", "primaryButton"),
            "Y": ("left", "secondaryButton"),
            "left_menu_button": ("left", "menuButton"),
            "right_menu_button": ("right", "menuButton"),
            "left_axis_click": ("left", "axisClick"),
            "right_axis_click": ("right", "axisClick"),
        }
        if name not in mapping:
            raise ValueError(
                f"Invalid name: {name}. Valid names are: 'A', 'B', 'X', 'Y', "
                "'left_menu_button', 'right_menu_button', 'left_axis_click', 'right_axis_click'."
            )

        side, key = mapping[name]
        return bool(self._get_controller_value(side, key, False))

    def _get_controller_value(self, side: str, key: str, default: Any) -> Any:
        tracking = self._get_latest_tracking()
        if tracking is None:
            return default

        controller = tracking.get("Controller")
        if not isinstance(controller, dict):
            return default

        side_data = controller.get(side)
        if not isinstance(side_data, dict):
            return default

        return side_data.get(key, default)

    def get_timestamp_ns(self) -> int:
        """Returns the latest XR timestamp in nanoseconds."""
        if self.transport == "sdk":
            return self._xrt.get_time_stamp_ns()

        with self._udp_lock:
            tracking = self._latest_tracking
            received_at = self._latest_monotonic_time
        if tracking is None:
            return 0

        body = tracking.get("Body")
        if isinstance(body, dict) and "timeStampNs" in body:
            return int(body["timeStampNs"])

        if "timeStampNs" in tracking:
            return int(tracking["timeStampNs"])

        packet_timestamp_ms = tracking.get("_packet_timestamp_ms")
        if packet_timestamp_ms is not None:
            return int(packet_timestamp_ms) * 1_000_000

        # Never synthesize a fresh timestamp on every read: after UDP stops,
        # doing so makes a stale pose/button frame look live forever.  The host
        # receive time changes only when a valid packet actually arrives.
        return int(received_at * 1_000_000_000) if received_at > 0.0 else 0

    def get_hand_tracking_state(self, hand: str) -> np.ndarray | None:
        """Returns the hand tracking state for the specified hand, or None if unavailable."""
        if self.transport == "sdk":
            if hand.lower() == "left":
                if not self._xrt.get_left_hand_is_active():
                    return None
                return self._xrt.get_left_hand_tracking_state()
            if hand.lower() == "right":
                if not self._xrt.get_right_hand_is_active():
                    return None
                return self._xrt.get_right_hand_tracking_state()
            raise ValueError(f"Invalid hand: {hand}. Valid hands are: 'left', 'right'.")

        if hand.lower() not in ("left", "right"):
            raise ValueError(f"Invalid hand: {hand}. Valid hands are: 'left', 'right'.")

        return None

    def get_joystick_state(self, controller: str) -> list[float]:
        """Returns joystick [x, y] for the specified controller."""
        if self.transport == "sdk":
            if controller.lower() == "left":
                return self._xrt.get_left_axis()
            if controller.lower() == "right":
                return self._xrt.get_right_axis()
            raise ValueError(
                f"Invalid controller: {controller}. Valid controllers are: 'left', 'right'."
            )

        side = controller.lower()
        if side not in ("left", "right"):
            raise ValueError(
                f"Invalid controller: {controller}. Valid controllers are: 'left', 'right'."
            )

        return [
            float(self._get_controller_value(side, "axisX", 0.0)),
            float(self._get_controller_value(side, "axisY", 0.0)),
        ]

    def get_motion_tracker_data(self) -> dict:
        """Returns motion tracker data when using the SDK backend."""
        if self.transport != "sdk":
            return {}

        num_motion_data = self._xrt.num_motion_data_available()
        if num_motion_data == 0:
            return {}

        poses = self._xrt.get_motion_tracker_pose()
        velocities = self._xrt.get_motion_tracker_velocity()
        accelerations = self._xrt.get_motion_tracker_acceleration()
        serial_numbers = self._xrt.get_motion_tracker_serial_numbers()

        tracker_data = {}
        for i in range(num_motion_data):
            serial = serial_numbers[i]
            tracker_data[serial] = {
                "pose": poses[i],
                "velocity": velocities[i],
                "acceleration": accelerations[i],
            }

        return tracker_data

    def get_body_tracking_data(self) -> dict | None:
        """Returns complete body tracking data or None if unavailable."""
        if self.transport == "sdk":
            if not self._xrt.is_body_data_available():
                return None

            return {
                "poses": self._xrt.get_body_joints_pose(),
                "velocities": self._xrt.get_body_joints_velocity(),
                "accelerations": self._xrt.get_body_joints_acceleration(),
            }

        tracking = self._get_latest_tracking()
        if tracking is None:
            return None

        body = tracking.get("Body")
        if not isinstance(body, dict):
            return None

        joints = body.get("joints")
        if not isinstance(joints, list) or not joints:
            return None

        joint_count = int(body.get("len", len(joints)))
        joint_count = min(joint_count, len(joints))
        poses = np.zeros((joint_count, 7), dtype=np.float64)
        velocities = np.zeros((joint_count, 6), dtype=np.float64)
        accelerations = np.zeros((joint_count, 6), dtype=np.float64)
        imu_timestamps = np.zeros(joint_count, dtype=np.int64)
        body_timestamp = int(body.get("timeStampNs", tracking.get("timeStampNs", 0)))

        for index in range(joint_count):
            joint = joints[index] if isinstance(joints[index], dict) else {}
            poses[index] = self._parse_vector(joint.get("p"), 7, _IDENTITY_POSE)
            velocities[index] = self._parse_vector(joint.get("va"), 6, _ZERO6)
            accelerations[index] = self._parse_vector(joint.get("wva"), 6, _ZERO6)
            imu_timestamps[index] = int(joint.get("t", body_timestamp))

        return {
            "poses": poses,
            "velocities": velocities,
            "accelerations": accelerations,
            "imu_timestamps": imu_timestamps,
            "body_timestamp": body_timestamp,
        }

    def close(self):
        if self.transport == "sdk":
            if self._xrt is not None:
                self._xrt.close()
            return

        self._udp_running = False
        if self._udp_socket is not None:
            try:
                self._udp_socket.close()
            except OSError:
                pass
            self._udp_socket = None

        if self._udp_thread is not None and self._udp_thread.is_alive():
            self._udp_thread.join(timeout=0.2)
            self._udp_thread = None
