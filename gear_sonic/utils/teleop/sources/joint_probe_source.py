"""Single-joint G1 reference probe for validating POSE joint ordering."""

from __future__ import annotations

import math
import threading
import time
from typing import Any

import numpy as np

from gear_sonic.utils.teleop.sources.base import (
    FullBodyReference,
    G1_DEFAULT_JOINT_POS_ISAACLAB,
    G1_DEFAULT_ROOT_POS_W,
    G1_ISAACLAB_TO_MUJOCO_IDX,
    MocapFrame,
    Pose7D,
)
from gear_sonic.utils.teleop.sources.g1_body_fk import G1BodyFk


G1_ISAACLAB_JOINT_NAMES = (
    "left_hip_pitch",
    "right_hip_pitch",
    "waist_yaw",
    "left_hip_roll",
    "right_hip_roll",
    "waist_roll",
    "left_hip_yaw",
    "right_hip_yaw",
    "waist_pitch",
    "left_knee",
    "right_knee",
    "left_shoulder_pitch",
    "right_shoulder_pitch",
    "left_ankle_pitch",
    "right_ankle_pitch",
    "left_shoulder_roll",
    "right_shoulder_roll",
    "left_ankle_roll",
    "right_ankle_roll",
    "left_shoulder_yaw",
    "right_shoulder_yaw",
    "left_elbow",
    "right_elbow",
    "left_wrist_roll",
    "right_wrist_roll",
    "left_wrist_pitch",
    "right_wrist_pitch",
    "left_wrist_yaw",
    "right_wrist_yaw",
)


def resolve_g1_isaaclab_joint_index(index: int | None, name: str | None) -> int:
    if name:
        normalized = name.strip().removesuffix("_joint").removesuffix("_link")
        try:
            return G1_ISAACLAB_JOINT_NAMES.index(normalized)
        except ValueError as exc:
            raise ValueError(
                f"unknown G1 IsaacLab joint name {name!r}; expected one of "
                f"{', '.join(G1_ISAACLAB_JOINT_NAMES)}"
            ) from exc
    resolved = 9 if index is None else int(index)
    if resolved < 0 or resolved >= len(G1_ISAACLAB_JOINT_NAMES):
        raise ValueError(
            f"joint probe index must be in [0, {len(G1_ISAACLAB_JOINT_NAMES) - 1}], "
            f"got {resolved}"
        )
    return resolved


class JointProbePlaybackSource:
    """Replay a smooth single-joint offset in IsaacLab order."""

    def __init__(
        self,
        joint_index: int,
        amplitude_rad: float = 0.3,
        frequency_hz: float = 0.25,
        target_fps: float = 50.0,
    ):
        self.joint_index = int(joint_index)
        if self.joint_index < 0 or self.joint_index >= len(G1_ISAACLAB_JOINT_NAMES):
            raise ValueError(f"joint_index out of range: {joint_index}")
        self.joint_name = G1_ISAACLAB_JOINT_NAMES[self.joint_index]
        self.mujoco_index = int(G1_ISAACLAB_TO_MUJOCO_IDX[self.joint_index])
        self.amplitude_rad = float(amplitude_rad)
        self.frequency_hz = float(frequency_hz)
        self.target_fps = max(1.0, float(target_fps))

        self._fk = G1BodyFk()
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._lock = threading.Lock()
        self._latest: MocapFrame | None = None
        self._frames_emitted = 0
        self._last_offset_rad = 0.0
        self._last_velocity_radps = 0.0

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._running.set()
        self._thread = threading.Thread(target=self._run, name="JointProbePlaybackSource", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def get_latest(self) -> MocapFrame | None:
        with self._lock:
            return self._latest

    @property
    def diagnostics(self) -> dict[str, Any]:
        with self._lock:
            return {
                "fps": self.target_fps,
                "received_packets": self._frames_emitted,
                "frames_emitted": self._frames_emitted,
                "dropped_packets": 0,
                "last_error": None,
                "has_frame": self._latest is not None,
                "joint_index": self.joint_index,
                "mujoco_index": self.mujoco_index,
                "joint_name": self.joint_name,
                "offset_rad": self._last_offset_rad,
                "velocity_radps": self._last_velocity_radps,
            }

    def _run(self) -> None:
        frame_period_s = 1.0 / self.target_fps
        stream_frame_idx = self._frames_emitted
        start_s = time.time()
        next_tick_s = start_s

        while self._running.is_set():
            elapsed_s = time.time() - start_s
            frame = self._build_frame(stream_frame_idx, elapsed_s)
            with self._lock:
                self._latest = frame
                self._frames_emitted = stream_frame_idx + 1

            stream_frame_idx += 1
            next_tick_s += frame_period_s
            sleep_s = next_tick_s - time.time()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick_s = time.time()

    def _build_frame(self, frame_idx: int, elapsed_s: float) -> MocapFrame:
        omega = 2.0 * math.pi * max(0.0, self.frequency_hz)
        offset = self.amplitude_rad * math.sin(omega * elapsed_s)
        velocity = self.amplitude_rad * omega * math.cos(omega * elapsed_s)
        joint_pos = G1_DEFAULT_JOINT_POS_ISAACLAB.copy()
        joint_vel = np.zeros(29, dtype=np.float32)
        joint_pos[self.joint_index] += np.float32(offset)
        joint_vel[self.joint_index] = np.float32(velocity)

        root_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        root_pos = G1_DEFAULT_ROOT_POS_W.copy()
        body_pos = self._fk.compute_body_pos14_world(
            joint_pos.reshape(1, 29),
            root_pos.reshape(1, 3),
            root_quat.reshape(1, 4),
        )[0]

        full_body = FullBodyReference(
            smpl_joints=np.zeros((24, 3), dtype=np.float32),
            smpl_pose=np.zeros((21, 3), dtype=np.float32),
            body_quat_w=root_quat,
            body_pos_w=root_pos,
            body_pos=body_pos,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            frame_index=int(frame_idx),
        )
        root_pose = Pose7D(position=root_pos, quat_wxyz=root_quat)
        self._last_offset_rad = float(offset)
        self._last_velocity_radps = float(velocity)
        return MocapFrame(
            source="joint_probe",
            host_time_s=time.time(),
            frame_index=int(frame_idx),
            fps=self.target_fps,
            joints={"root": root_pose},
            full_body=full_body,
            metadata={
                "format": "joint_probe",
                "joint_index": self.joint_index,
                "mujoco_index": self.mujoco_index,
                "joint_name": self.joint_name,
                "offset_rad": float(offset),
                "velocity_radps": float(velocity),
            },
        )
