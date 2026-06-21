"""POSE topic publisher for SMPL-like streamed motion references."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np
import zmq

from gear_sonic.utils.teleop.sources import (
    FullBodyReference,
    G1_DEFAULT_JOINT_POS_ISAACLAB,
    G1_LOWER_BODY_JOINT_IDX_ISAACLAB,
)
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import pack_pose_message

SMPL_LOWER_BODY_JOINT_IDX = np.array([1, 2, 4, 5, 7, 8, 10, 11], dtype=np.int64)
SMPL_LOWER_BODY_POSE_IDX = np.array([0, 1, 3, 4, 6, 7, 9, 10], dtype=np.int64)


@dataclass
class PoseStreamPublisher:
    """Maintain a sliding POSE window and publish protocol v2/v3 messages."""

    window_size: int = 5
    protocol_version: int = 3
    _buffers: dict[str, deque] = field(init=False, repr=False)
    _last_frame_index: int | None = field(default=None, init=False)
    _generated_frame_index: int = field(default=0, init=False)
    _diagnostics: dict[str, float] = field(default_factory=dict, init=False, repr=False)
    sent_messages: int = 0

    def __post_init__(self) -> None:
        if self.window_size <= 0:
            raise ValueError("window_size must be positive")
        if self.protocol_version not in (2, 3):
            raise ValueError("PoseStreamPublisher supports protocol v2 or v3")
        self._buffers = {
            "smpl_pose": deque(maxlen=self.window_size),
            "smpl_joints": deque(maxlen=self.window_size),
            "body_quat_w": deque(maxlen=self.window_size),
            "joint_pos": deque(maxlen=self.window_size),
            "joint_vel": deque(maxlen=self.window_size),
            "frame_index": deque(maxlen=self.window_size),
        }

    @property
    def buffered_frames(self) -> int:
        return len(self._buffers["frame_index"])

    @property
    def is_ready(self) -> bool:
        return self.buffered_frames >= self.window_size

    @property
    def diagnostics(self) -> dict[str, float]:
        return dict(self._diagnostics)

    def reset(self) -> None:
        for buffer in self._buffers.values():
            buffer.clear()
        self._last_frame_index = None

    def publish(
        self,
        socket: zmq.Socket,
        reference: FullBodyReference,
        frame_index: int | None = None,
        vr_position: np.ndarray | None = None,
        vr_orientation: np.ndarray | None = None,
        catch_up: bool = True,
    ) -> bool:
        resolved_frame_index = self._resolve_frame_index(reference, frame_index)
        if self._last_frame_index == resolved_frame_index:
            return False
        if self._last_frame_index is not None and resolved_frame_index < self._last_frame_index:
            self.reset()

        self._append_reference(reference, resolved_frame_index)
        self._last_frame_index = resolved_frame_index
        if not self.is_ready:
            return False

        data = {
            "smpl_pose": np.stack(self._buffers["smpl_pose"], axis=0).astype(np.float32),
            "smpl_joints": np.stack(self._buffers["smpl_joints"], axis=0).astype(np.float32),
            "body_quat_w": np.stack(self._buffers["body_quat_w"], axis=0).astype(np.float32),
            "frame_index": np.asarray(self._buffers["frame_index"], dtype=np.int64),
            "catch_up": np.array([catch_up], dtype=bool),
        }
        if self.protocol_version == 3:
            data["joint_pos"] = np.stack(self._buffers["joint_pos"], axis=0).astype(np.float32)
            data["joint_vel"] = np.stack(self._buffers["joint_vel"], axis=0).astype(np.float32)
        if vr_position is not None:
            data["vr_position"] = np.asarray(vr_position, dtype=np.float32).reshape(9)
        if vr_orientation is not None:
            data["vr_orientation"] = np.asarray(vr_orientation, dtype=np.float32).reshape(12)

        socket.send(pack_pose_message(data, topic="pose", version=self.protocol_version))
        self.sent_messages += 1
        return True

    def _resolve_frame_index(
        self, reference: FullBodyReference, frame_index: int | None
    ) -> int:
        if frame_index is not None:
            return int(frame_index)
        if reference.frame_index is not None:
            return int(reference.frame_index)
        frame_idx = self._generated_frame_index
        self._generated_frame_index += 1
        return frame_idx

    def _append_reference(self, reference: FullBodyReference, frame_index: int) -> None:
        self._buffers["smpl_pose"].append(reference.smpl_pose)
        self._buffers["smpl_joints"].append(reference.smpl_joints)
        self._buffers["body_quat_w"].append(reference.body_quat_w)
        self._buffers["joint_pos"].append(reference.joint_pos)
        self._buffers["joint_vel"].append(reference.joint_vel)
        self._buffers["frame_index"].append(int(frame_index))
        self._diagnostics = _compute_pose_diagnostics(reference)


def _compute_pose_diagnostics(reference: FullBodyReference) -> dict[str, float]:
    joint_pos = np.asarray(reference.joint_pos, dtype=np.float32).reshape(-1)
    joint_vel = np.asarray(reference.joint_vel, dtype=np.float32).reshape(-1)
    lower_joint_pos = joint_pos[G1_LOWER_BODY_JOINT_IDX_ISAACLAB]
    lower_default = G1_DEFAULT_JOINT_POS_ISAACLAB[G1_LOWER_BODY_JOINT_IDX_ISAACLAB]
    smpl_lower_joints = np.asarray(reference.smpl_joints, dtype=np.float32)[
        SMPL_LOWER_BODY_JOINT_IDX
    ]
    smpl_lower_pose = np.asarray(reference.smpl_pose, dtype=np.float32)[SMPL_LOWER_BODY_POSE_IDX]
    quat = np.asarray(reference.body_quat_w, dtype=np.float32).reshape(4)
    quat_norm = float(np.linalg.norm(quat))
    if quat_norm > 1e-8 and np.isfinite(quat_norm):
        quat = quat / quat_norm
    else:
        quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    _, qx, qy, _ = quat
    root_z_dot = float(np.clip(1.0 - 2.0 * (qx * qx + qy * qy), -1.0, 1.0))
    root_tilt_rad = float(np.arccos(root_z_dot))

    return {
        "joint_pos_min": float(np.min(joint_pos)),
        "joint_pos_max": float(np.max(joint_pos)),
        "joint_pos_abs_max": float(np.max(np.abs(joint_pos))),
        "joint_vel_abs_max": float(np.max(np.abs(joint_vel))),
        "lower_joint_default_delta_abs_max": float(
            np.max(np.abs(lower_joint_pos - lower_default))
        ),
        "smpl_lower_z_min": float(np.min(smpl_lower_joints[:, 2])),
        "smpl_lower_z_max": float(np.max(smpl_lower_joints[:, 2])),
        "smpl_lower_span_m": float(
            np.linalg.norm(np.max(smpl_lower_joints, axis=0) - np.min(smpl_lower_joints, axis=0))
        ),
        "smpl_lower_pose_abs_max_rad": float(np.max(np.linalg.norm(smpl_lower_pose, axis=1))),
        "root_tilt_rad": root_tilt_rad,
    }
