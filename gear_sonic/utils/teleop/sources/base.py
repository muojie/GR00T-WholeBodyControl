"""Shared data types for teleoperation motion sources."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np
from scipy.spatial.transform import Rotation


G1_JOINT_COUNT = 29
SMPL_L_ELBOW_IDX = 17
SMPL_R_ELBOW_IDX = 18
SMPL_L_WRIST_IDX = 19
SMPL_R_WRIST_IDX = 20
G1_L_WRIST_ROLL_IDX = 23
G1_R_WRIST_ROLL_IDX = 24
G1_L_WRIST_PITCH_IDX = 25
G1_R_WRIST_PITCH_IDX = 26
G1_L_WRIST_YAW_IDX = 27
G1_R_WRIST_YAW_IDX = 28


def _vector(name: str, value: Any, length: int) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float32)
    if arr.shape != (length,):
        raise ValueError(f"{name} must have shape ({length},), got {arr.shape}")
    return arr


def normalize_quat_wxyz(quat_wxyz: Any) -> np.ndarray:
    """Return a normalized wxyz quaternion, falling back to identity for bad input."""
    quat = _vector("quat_wxyz", quat_wxyz, 4)
    norm = float(np.linalg.norm(quat))
    if norm < 1e-8 or not np.isfinite(norm):
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    return (quat / norm).astype(np.float32)


def _quat_multiply_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.split(a, 4, axis=1)
    bw, bx, by, bz = np.split(b, 4, axis=1)
    return np.concatenate(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=1,
    )


def _rotvec_to_quat_wxyz(rotvec: np.ndarray) -> np.ndarray:
    angles = np.linalg.norm(rotvec, axis=1, keepdims=True)
    half_angles = 0.5 * angles
    scale = np.empty_like(angles, dtype=np.float32)
    small = angles < 1e-8
    scale[small] = 0.5
    scale[~small] = np.sin(half_angles[~small]) / angles[~small]
    return np.concatenate([np.cos(half_angles), scale * rotvec], axis=1).astype(np.float32)


def _decompose_rotation_aa(rotation_aa: np.ndarray, axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    quat = _rotvec_to_quat_wxyz(np.asarray(rotation_aa, dtype=np.float32).reshape(-1, 3))
    axis = np.asarray(axis, dtype=np.float32)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-8 or not np.isfinite(axis_norm):
        raise ValueError("twist axis must be non-zero")
    axis = axis / axis_norm

    twist_vector = np.dot(quat[:, 1:4], axis)[:, None] * axis
    quat_twist = np.concatenate([quat[:, 0:1], twist_vector], axis=1)
    twist_norm = np.linalg.norm(quat_twist, axis=1, keepdims=True)
    quat_twist = quat_twist / np.maximum(twist_norm, 1e-8)

    quat_twist_inv = quat_twist * np.array([1.0, -1.0, -1.0, -1.0], dtype=np.float32)
    quat_swing = _quat_multiply_wxyz(quat_twist_inv, quat)
    return quat_twist.astype(np.float32), quat_swing.astype(np.float32)


def smpl_pose_to_g1_wrist_joint_pos(smpl_pose: Any) -> np.ndarray:
    """Project SMPL elbow/wrist rotations to the six G1 wrist joints used by deploy."""
    body_pose = np.asarray(smpl_pose, dtype=np.float32).reshape(-1, 21, 3)
    joint_pos = np.zeros(G1_JOINT_COUNT, dtype=np.float32)

    smpl_l_elbow_aa = body_pose[:, SMPL_L_ELBOW_IDX]
    smpl_l_wrist_aa = body_pose[:, SMPL_L_WRIST_IDX]
    smpl_r_elbow_aa = body_pose[:, SMPL_R_ELBOW_IDX]
    smpl_r_wrist_aa = body_pose[:, SMPL_R_WRIST_IDX]

    _, g1_l_elbow_q_swing = _decompose_rotation_aa(smpl_l_elbow_aa, np.array([0.0, 1.0, 0.0]))
    _, g1_r_elbow_q_swing = _decompose_rotation_aa(smpl_r_elbow_aa, np.array([0.0, 1.0, 0.0]))

    l_elbow_swing_euler = Rotation.from_quat(g1_l_elbow_q_swing[:, [1, 2, 3, 0]]).as_euler(
        "XYZ", degrees=False
    )
    r_elbow_swing_euler = Rotation.from_quat(g1_r_elbow_q_swing[:, [1, 2, 3, 0]]).as_euler(
        "XYZ", degrees=False
    )
    l_wrist_euler = Rotation.from_rotvec(smpl_l_wrist_aa).as_euler("XYZ", degrees=False)
    r_wrist_euler = Rotation.from_rotvec(smpl_r_wrist_aa).as_euler("XYZ", degrees=False)

    g1_l_wrist_roll = l_elbow_swing_euler[:, 0] + l_wrist_euler[:, 0]
    g1_l_wrist_pitch = -l_wrist_euler[:, 1]
    g1_l_wrist_yaw = l_elbow_swing_euler[:, 2] + l_wrist_euler[:, 2]

    g1_r_wrist_roll = -(r_elbow_swing_euler[:, 0] + r_wrist_euler[:, 0])
    g1_r_wrist_pitch = -r_wrist_euler[:, 1]
    g1_r_wrist_yaw = r_elbow_swing_euler[:, 2] + r_wrist_euler[:, 2]

    joint_pos[G1_L_WRIST_ROLL_IDX] = g1_l_wrist_roll[0]
    joint_pos[G1_L_WRIST_PITCH_IDX] = -g1_l_wrist_pitch[0]
    joint_pos[G1_L_WRIST_YAW_IDX] = g1_l_wrist_yaw[0]
    joint_pos[G1_R_WRIST_ROLL_IDX] = g1_r_wrist_roll[0]
    joint_pos[G1_R_WRIST_PITCH_IDX] = g1_r_wrist_pitch[0]
    joint_pos[G1_R_WRIST_YAW_IDX] = g1_r_wrist_yaw[0]
    return joint_pos.astype(np.float32)


@dataclass
class Pose7D:
    """Position plus quaternion in the repository's VR 3-point convention."""

    position: np.ndarray
    quat_wxyz: np.ndarray

    def __post_init__(self) -> None:
        self.position = _vector("position", self.position, 3)
        self.quat_wxyz = normalize_quat_wxyz(self.quat_wxyz)

    def as_pose7(self) -> np.ndarray:
        return np.concatenate((self.position, self.quat_wxyz)).astype(np.float32)


@dataclass
class FullBodyReference:
    """SMPL-like full-body reference data for the deploy pose stream."""

    smpl_joints: np.ndarray
    smpl_pose: np.ndarray
    body_quat_w: np.ndarray
    joint_pos: np.ndarray | None = None
    joint_vel: np.ndarray | None = None
    frame_index: int | None = None

    def __post_init__(self) -> None:
        self.smpl_joints = np.asarray(self.smpl_joints, dtype=np.float32)
        if self.smpl_joints.shape != (24, 3):
            raise ValueError(f"smpl_joints must have shape (24, 3), got {self.smpl_joints.shape}")

        self.smpl_pose = np.asarray(self.smpl_pose, dtype=np.float32)
        if self.smpl_pose.shape != (21, 3):
            raise ValueError(f"smpl_pose must have shape (21, 3), got {self.smpl_pose.shape}")

        self.body_quat_w = normalize_quat_wxyz(self.body_quat_w)

        if self.joint_pos is None:
            self.joint_pos = smpl_pose_to_g1_wrist_joint_pos(self.smpl_pose)
        else:
            self.joint_pos = _vector("joint_pos", self.joint_pos, G1_JOINT_COUNT)

        if self.joint_vel is None:
            self.joint_vel = np.zeros(G1_JOINT_COUNT, dtype=np.float32)
        else:
            self.joint_vel = _vector("joint_vel", self.joint_vel, G1_JOINT_COUNT)


@dataclass
class MocapFrame:
    """Canonical frame passed from motion-capture sources to teleop managers."""

    source: str
    host_time_s: float
    source_time_ns: int | None = None
    frame_index: int | None = None
    fps: float = 0.0
    joints: dict[str, Pose7D] = field(default_factory=dict)
    bones: dict[int, Pose7D] = field(default_factory=dict)
    direct_vr_position: np.ndarray | None = None
    direct_vr_orientation: np.ndarray | None = None
    full_body: FullBodyReference | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.direct_vr_position is not None:
            self.direct_vr_position = _vector("direct_vr_position", self.direct_vr_position, 9)
        if self.direct_vr_orientation is not None:
            self.direct_vr_orientation = _vector(
                "direct_vr_orientation", self.direct_vr_orientation, 12
            )


class MocapSource(Protocol):
    """Minimal interface for threaded or polled mocap sources."""

    def start(self) -> None:
        ...

    def stop(self) -> None:
        ...

    def get_latest(self) -> MocapFrame | None:
        ...
