"""POSE topic publisher for SMPL-like streamed motion references."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation
import zmq

from gear_sonic.utils.teleop.sources import (
    FullBodyReference,
    G1_DEFAULT_JOINT_POS_ISAACLAB,
    G1_LOWER_BODY_JOINT_IDX_ISAACLAB,
    G1_WRIST_JOINT_IDX_ISAACLAB,
    G1_WRIST_JOINT_LOWER_LIMIT_ISAACLAB,
    G1_WRIST_JOINT_UPPER_LIMIT_ISAACLAB,
)
from gear_sonic.utils.teleop.sources.base import normalize_quat_wxyz
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import pack_pose_message

SMPL_LOWER_BODY_JOINT_IDX = np.array([1, 2, 4, 5, 7, 8, 10, 11], dtype=np.int64)
SMPL_LOWER_BODY_POSE_IDX = np.array([0, 1, 3, 4, 6, 7, 9, 10], dtype=np.int64)


@dataclass
class PoseStreamPublisher:
    """Maintain a sliding POSE window and publish protocol v1/v2/v3 messages."""

    window_size: int = 5
    protocol_version: int = 3
    encoder_mode: int = 2
    enable_reference_filter: bool = True
    reference_alpha: float = 0.35
    max_smpl_joint_speed_mps: float = 1.2
    max_smpl_pose_speed_radps: float = 3.0
    max_root_angular_speed_radps: float = 2.5
    max_joint_speed_radps: float = 4.0
    root_tilt_limit_rad: float = 0.45
    root_yaw_only: bool = False
    _buffers: dict[str, deque] = field(init=False, repr=False)
    _last_frame_index: int | None = field(default=None, init=False)
    _generated_frame_index: int = field(default=0, init=False)
    _last_filter_time_s: float | None = field(default=None, init=False, repr=False)
    _last_filtered_reference: FullBodyReference | None = field(
        default=None, init=False, repr=False
    )
    _last_filter_metrics: dict[str, float] = field(default_factory=dict, init=False, repr=False)
    _diagnostics: dict[str, float] = field(default_factory=dict, init=False, repr=False)
    sent_messages: int = 0

    def __post_init__(self) -> None:
        if self.window_size <= 0:
            raise ValueError("window_size must be positive")
        if self.protocol_version not in (1, 2, 3):
            raise ValueError("PoseStreamPublisher supports protocol v1, v2, or v3")
        self.reference_alpha = _clamp01(self.reference_alpha)
        self._buffers = {
            "smpl_pose": deque(maxlen=self.window_size),
            "smpl_joints": deque(maxlen=self.window_size),
            "body_pos": deque(maxlen=self.window_size),
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

    def reset_filter(self) -> None:
        self._last_filter_time_s = None
        self._last_filtered_reference = None
        self._last_filter_metrics = {}

    def publish(
        self,
        socket: zmq.Socket,
        reference: FullBodyReference,
        frame_index: int | None = None,
        vr_position: np.ndarray | None = None,
        vr_orientation: np.ndarray | None = None,
        timestamp_s: float | None = None,
        catch_up: bool = True,
    ) -> bool:
        resolved_frame_index = self._resolve_frame_index(reference, frame_index)
        if self._last_frame_index == resolved_frame_index:
            return False
        if self._last_frame_index is not None and resolved_frame_index < self._last_frame_index:
            self.reset()
            self.reset_filter()

        filtered_reference = self._filter_reference(reference, timestamp_s)
        self._append_reference(filtered_reference, resolved_frame_index)
        self._last_frame_index = resolved_frame_index
        if not self.is_ready:
            return False

        data = {
            "body_pos": np.stack(self._buffers["body_pos"], axis=0).astype(np.float32),
            "body_quat_w": np.stack(self._buffers["body_quat_w"], axis=0).astype(np.float32),
            "frame_index": np.asarray(self._buffers["frame_index"], dtype=np.int64),
            "catch_up": np.array([catch_up], dtype=bool),
            "encoder_mode": np.array([int(self.encoder_mode)], dtype=np.int32),
        }
        if self.protocol_version in (2, 3):
            data["smpl_pose"] = np.stack(self._buffers["smpl_pose"], axis=0).astype(np.float32)
            data["smpl_joints"] = np.stack(self._buffers["smpl_joints"], axis=0).astype(
                np.float32
            )
        if self.protocol_version in (1, 3):
            data["joint_pos"] = np.stack(self._buffers["joint_pos"], axis=0).astype(np.float32)
            data["joint_vel"] = np.stack(self._buffers["joint_vel"], axis=0).astype(np.float32)
        if vr_position is not None:
            data["vr_position"] = np.asarray(vr_position, dtype=np.float32).reshape(9)
        if vr_orientation is not None:
            data["vr_orientation"] = np.asarray(vr_orientation, dtype=np.float32).reshape(12)

        socket.send(pack_pose_message(data, topic="pose", version=self.protocol_version))
        self.sent_messages += 1
        return True

    def publish_bootstrap(
        self,
        socket: zmq.Socket,
        reference: FullBodyReference,
        frame_index: int | None = None,
        vr_position: np.ndarray | None = None,
        vr_orientation: np.ndarray | None = None,
        timestamp_s: float | None = None,
        catch_up: bool = True,
    ) -> bool:
        """Send one full POSE window by holding the first available reference.

        This lets deploy enter POSE control without waiting for a full real-time
        window to accumulate. The normal sliding buffer is intentionally left
        empty so subsequent real frames build a clean, monotonic window.
        """
        resolved_frame_index = self._resolve_frame_index(reference, frame_index)
        if self._last_frame_index == resolved_frame_index:
            return False

        filtered_reference = self._filter_reference(reference, timestamp_s)
        self._last_frame_index = resolved_frame_index
        self._diagnostics = _compute_pose_diagnostics(filtered_reference)
        self._diagnostics.update(self._last_filter_metrics)

        frame_index_window = (
            np.arange(self.window_size, dtype=np.int64) + int(resolved_frame_index)
        )
        data = {
            "body_pos": np.repeat(
                _reference_body_pos(filtered_reference)[None, ...],
                self.window_size,
                axis=0,
            ).astype(np.float32),
            "body_quat_w": np.repeat(
                filtered_reference.body_quat_w.reshape(1, 4),
                self.window_size,
                axis=0,
            ).astype(np.float32),
            "frame_index": frame_index_window,
            "catch_up": np.array([catch_up], dtype=bool),
            "encoder_mode": np.array([int(self.encoder_mode)], dtype=np.int32),
        }
        if self.protocol_version in (2, 3):
            data["smpl_pose"] = np.repeat(
                filtered_reference.smpl_pose[None, ...],
                self.window_size,
                axis=0,
            ).astype(np.float32)
            data["smpl_joints"] = np.repeat(
                filtered_reference.smpl_joints[None, ...],
                self.window_size,
                axis=0,
            ).astype(np.float32)
        if self.protocol_version in (1, 3):
            data["joint_pos"] = np.repeat(
                filtered_reference.joint_pos.reshape(1, -1),
                self.window_size,
                axis=0,
            ).astype(np.float32)
            data["joint_vel"] = np.zeros(
                (self.window_size, filtered_reference.joint_vel.reshape(-1).shape[0]),
                dtype=np.float32,
            )
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
        self._buffers["body_pos"].append(_reference_body_pos(reference))
        self._buffers["body_quat_w"].append(reference.body_quat_w)
        self._buffers["joint_pos"].append(reference.joint_pos)
        self._buffers["joint_vel"].append(reference.joint_vel)
        self._buffers["frame_index"].append(int(frame_index))
        self._diagnostics = _compute_pose_diagnostics(reference)
        self._diagnostics.update(self._last_filter_metrics)

    def _filter_reference(
        self, reference: FullBodyReference, timestamp_s: float | None
    ) -> FullBodyReference:
        target = _copy_reference_with_root_tilt_limit(reference, self.root_tilt_limit_rad)
        if self.root_yaw_only:
            target = _copy_reference_with_body_quat(target, _yaw_only_quat_wxyz(target.body_quat_w))
        if not self.enable_reference_filter:
            self._last_filter_time_s = float(timestamp_s) if timestamp_s is not None else None
            self._last_filtered_reference = target
            self._last_filter_metrics = {"pose_filter_enabled": 0.0}
            return target

        if (
            self._last_filter_time_s is None
            or self._last_filtered_reference is None
            or timestamp_s is None
        ):
            self._last_filter_time_s = float(timestamp_s) if timestamp_s is not None else None
            self._last_filtered_reference = target
            metrics = _compute_filter_metrics(reference, target, dt_s=0.0)
            self._last_filter_metrics = metrics
            return target

        dt_s = max(1e-3, min(0.2, float(timestamp_s) - self._last_filter_time_s))
        prev = self._last_filtered_reference
        alpha = self.reference_alpha

        smpl_joints = _smooth_vectors(
            prev.smpl_joints,
            target.smpl_joints,
            alpha=alpha,
            max_step=self.max_smpl_joint_speed_mps * dt_s
            if self.max_smpl_joint_speed_mps > 0.0
            else None,
        )
        smpl_pose = _smooth_rotvecs(
            prev.smpl_pose,
            target.smpl_pose,
            alpha=alpha,
            max_angle=self.max_smpl_pose_speed_radps * dt_s
            if self.max_smpl_pose_speed_radps > 0.0
            else None,
        )
        body_pos = _smooth_vectors(
            prev.body_pos_w.reshape(1, 3),
            target.body_pos_w.reshape(1, 3),
            alpha=alpha,
            max_step=self.max_smpl_joint_speed_mps * dt_s
            if self.max_smpl_joint_speed_mps > 0.0
            else None,
        ).reshape(3)
        body_pos_all = None
        if target.body_pos is not None:
            if prev.body_pos is not None and prev.body_pos.shape == target.body_pos.shape:
                body_pos_all = _smooth_vectors(
                    prev.body_pos,
                    target.body_pos,
                    alpha=alpha,
                    max_step=self.max_smpl_joint_speed_mps * dt_s
                    if self.max_smpl_joint_speed_mps > 0.0
                    else None,
                )
            else:
                body_pos_all = target.body_pos
        limited_root_quat = (
            _limit_quat_step(
                prev.body_quat_w,
                target.body_quat_w,
                self.max_root_angular_speed_radps * dt_s,
            )
            if self.max_root_angular_speed_radps > 0.0
            else target.body_quat_w
        )
        body_quat_w = _slerp_quat_wxyz(prev.body_quat_w, limited_root_quat, alpha)
        body_quat_w = _clamp_root_tilt(body_quat_w, self.root_tilt_limit_rad)
        if self.root_yaw_only:
            body_quat_w = _yaw_only_quat_wxyz(body_quat_w)

        joint_pos = _smooth_vectors(
            prev.joint_pos.reshape(-1, 1),
            target.joint_pos.reshape(-1, 1),
            alpha=alpha,
            max_step=self.max_joint_speed_radps * dt_s
            if self.max_joint_speed_radps > 0.0
            else None,
        ).reshape(-1)
        joint_vel = ((joint_pos - prev.joint_pos) / dt_s).astype(np.float32)

        filtered = FullBodyReference(
            smpl_joints=smpl_joints,
            smpl_pose=smpl_pose,
            body_quat_w=body_quat_w,
            body_pos_w=body_pos,
            body_pos=body_pos_all,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            frame_index=reference.frame_index,
        )
        self._last_filter_time_s = float(timestamp_s)
        self._last_filtered_reference = filtered
        metrics = _compute_filter_metrics(reference, filtered, dt_s=dt_s)
        self._last_filter_metrics = metrics
        return filtered


def _reference_body_pos(reference: FullBodyReference) -> np.ndarray:
    return reference.body_pos if reference.body_pos is not None else reference.body_pos_w


def _compute_pose_diagnostics(reference: FullBodyReference) -> dict[str, float]:
    joint_pos = np.asarray(reference.joint_pos, dtype=np.float32).reshape(-1)
    joint_vel = np.asarray(reference.joint_vel, dtype=np.float32).reshape(-1)
    lower_joint_pos = joint_pos[G1_LOWER_BODY_JOINT_IDX_ISAACLAB]
    lower_default = G1_DEFAULT_JOINT_POS_ISAACLAB[G1_LOWER_BODY_JOINT_IDX_ISAACLAB]
    wrist_joint_pos = joint_pos[G1_WRIST_JOINT_IDX_ISAACLAB]
    wrist_joint_vel = joint_vel[G1_WRIST_JOINT_IDX_ISAACLAB]
    wrist_limit_margin = np.minimum(
        wrist_joint_pos - G1_WRIST_JOINT_LOWER_LIMIT_ISAACLAB,
        G1_WRIST_JOINT_UPPER_LIMIT_ISAACLAB - wrist_joint_pos,
    )
    smpl_lower_joints = np.asarray(reference.smpl_joints, dtype=np.float32)[
        SMPL_LOWER_BODY_JOINT_IDX
    ]
    smpl_lower_pose = np.asarray(reference.smpl_pose, dtype=np.float32)[SMPL_LOWER_BODY_POSE_IDX]
    body_pos = np.asarray(reference.body_pos_w, dtype=np.float32).reshape(3)
    body_pos_count = 1 if reference.body_pos is None else int(reference.body_pos.shape[0])
    body_pos_root_z = body_pos[2]
    if reference.body_pos is not None and reference.body_pos.shape[0] > 0:
        body_pos_root_z = float(reference.body_pos[0, 2])
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
        "wrist_joint_pos_abs_max": float(np.max(np.abs(wrist_joint_pos))),
        "wrist_joint_vel_abs_max": float(np.max(np.abs(wrist_joint_vel))),
        "wrist_joint_limit_margin_min": float(np.min(wrist_limit_margin)),
        "lower_joint_default_delta_abs_max": float(
            np.max(np.abs(lower_joint_pos - lower_default))
        ),
        "smpl_lower_z_min": float(np.min(smpl_lower_joints[:, 2])),
        "smpl_lower_z_max": float(np.max(smpl_lower_joints[:, 2])),
        "smpl_lower_span_m": float(
            np.linalg.norm(np.max(smpl_lower_joints, axis=0) - np.min(smpl_lower_joints, axis=0))
        ),
        "smpl_lower_pose_abs_max_rad": float(np.max(np.linalg.norm(smpl_lower_pose, axis=1))),
        "root_pos_z_m": float(body_pos[2]),
        "body_pos_root_z_m": float(body_pos_root_z),
        "root_tilt_rad": root_tilt_rad,
        "body_pos_count": float(body_pos_count),
    }


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _copy_reference_with_root_tilt_limit(
    reference: FullBodyReference, root_tilt_limit_rad: float
) -> FullBodyReference:
    return FullBodyReference(
        smpl_joints=reference.smpl_joints,
        smpl_pose=reference.smpl_pose,
        body_quat_w=_clamp_root_tilt(reference.body_quat_w, root_tilt_limit_rad),
        body_pos_w=reference.body_pos_w,
        body_pos=reference.body_pos,
        joint_pos=reference.joint_pos,
        joint_vel=reference.joint_vel,
        frame_index=reference.frame_index,
    )


def _copy_reference_with_body_quat(
    reference: FullBodyReference, body_quat_w: np.ndarray
) -> FullBodyReference:
    return FullBodyReference(
        smpl_joints=reference.smpl_joints,
        smpl_pose=reference.smpl_pose,
        body_quat_w=body_quat_w,
        body_pos_w=reference.body_pos_w,
        body_pos=reference.body_pos,
        joint_pos=reference.joint_pos,
        joint_vel=reference.joint_vel,
        frame_index=reference.frame_index,
    )


def _smooth_vectors(
    previous: np.ndarray,
    target: np.ndarray,
    alpha: float,
    max_step: float | None,
) -> np.ndarray:
    previous = np.asarray(previous, dtype=np.float32)
    target = np.asarray(target, dtype=np.float32)
    limited = _limit_vector_step(previous, target, max_step)
    return (previous + alpha * (limited - previous)).astype(np.float32)


def _limit_vector_step(
    previous: np.ndarray, target: np.ndarray, max_step: float | None
) -> np.ndarray:
    if max_step is None:
        return target.astype(np.float32)
    delta = target - previous
    flat = delta.reshape(-1, delta.shape[-1])
    norms = np.linalg.norm(flat, axis=1, keepdims=True)
    scale = np.ones_like(norms)
    mask = norms[:, 0] > max_step
    scale[mask, 0] = max_step / np.maximum(norms[mask, 0], 1e-8)
    limited = previous.reshape(-1, previous.shape[-1]) + flat * scale
    return limited.reshape(previous.shape).astype(np.float32)


def _smooth_rotvecs(
    previous: np.ndarray,
    target: np.ndarray,
    alpha: float,
    max_angle: float | None,
) -> np.ndarray:
    original_shape = np.asarray(target).shape
    previous = np.asarray(previous, dtype=np.float32).reshape(-1, 3)
    target = np.asarray(target, dtype=np.float32).reshape(-1, 3)
    out = np.zeros_like(previous)
    for idx, (prev_rv, target_rv) in enumerate(zip(previous, target, strict=True)):
        prev_rot = Rotation.from_rotvec(prev_rv)
        target_rot = Rotation.from_rotvec(target_rv)
        delta = (prev_rot.inv() * target_rot).as_rotvec()
        angle = float(np.linalg.norm(delta))
        if max_angle is not None and angle > max_angle:
            delta = delta * (max_angle / max(angle, 1e-8))
        out[idx] = (prev_rot * Rotation.from_rotvec(delta * alpha)).as_rotvec()
    return out.reshape(original_shape).astype(np.float32)


def _limit_quat_step(previous: np.ndarray, target: np.ndarray, max_angle_rad: float) -> np.ndarray:
    previous = normalize_quat_wxyz(previous)
    target = normalize_quat_wxyz(target)
    if float(np.dot(previous, target)) < 0.0:
        target = -target
    dot = float(np.clip(np.dot(previous, target), -1.0, 1.0))
    angle = 2.0 * np.arccos(abs(dot))
    if angle <= max_angle_rad or angle < 1e-6:
        return target.astype(np.float32)
    return _slerp_quat_wxyz(previous, target, max_angle_rad / angle)


def _slerp_quat_wxyz(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    q0 = normalize_quat_wxyz(q0)
    q1 = normalize_quat_wxyz(q1)
    alpha = _clamp01(alpha)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return normalize_quat_wxyz(q0 + alpha * (q1 - q0))

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * alpha
    s0 = np.cos(theta) - dot * np.sin(theta) / sin_theta_0
    s1 = np.sin(theta) / sin_theta_0
    return normalize_quat_wxyz((s0 * q0) + (s1 * q1))


def _root_tilt_rad(quat_wxyz: np.ndarray) -> float:
    quat = normalize_quat_wxyz(quat_wxyz)
    _, qx, qy, _ = quat
    root_z_dot = float(np.clip(1.0 - 2.0 * (qx * qx + qy * qy), -1.0, 1.0))
    return float(np.arccos(root_z_dot))


def _clamp_root_tilt(quat_wxyz: np.ndarray, max_tilt_rad: float) -> np.ndarray:
    if max_tilt_rad <= 0.0:
        return normalize_quat_wxyz(quat_wxyz)
    quat = normalize_quat_wxyz(quat_wxyz)
    tilt = _root_tilt_rad(quat)
    if tilt <= max_tilt_rad:
        return quat

    rot = Rotation.from_quat(quat[[1, 2, 3, 0]])
    forward = rot.apply([1.0, 0.0, 0.0])
    yaw = float(np.arctan2(forward[1], forward[0]))
    yaw_rot = Rotation.from_euler("z", yaw)
    residual = (yaw_rot.inv() * rot).as_rotvec()
    residual_angle = float(np.linalg.norm(residual))
    if residual_angle < 1e-8:
        return quat
    clamped = yaw_rot * Rotation.from_rotvec(
        residual * (max_tilt_rad / max(residual_angle, 1e-8))
    )
    clamped_xyzw = clamped.as_quat()
    return normalize_quat_wxyz(clamped_xyzw[[3, 0, 1, 2]].astype(np.float32))


def _yaw_only_quat_wxyz(quat_wxyz: np.ndarray) -> np.ndarray:
    quat = normalize_quat_wxyz(quat_wxyz)
    rot = Rotation.from_quat(quat[[1, 2, 3, 0]])
    forward = rot.apply([1.0, 0.0, 0.0])
    yaw = float(np.arctan2(forward[1], forward[0]))
    yaw_xyzw = Rotation.from_euler("z", yaw).as_quat()
    return normalize_quat_wxyz(yaw_xyzw[[3, 0, 1, 2]].astype(np.float32))


def _max_rotvec_error_rad(raw: np.ndarray, filtered: np.ndarray) -> float:
    raw = np.asarray(raw, dtype=np.float32).reshape(-1, 3)
    filtered = np.asarray(filtered, dtype=np.float32).reshape(-1, 3)
    max_error = 0.0
    for raw_rv, filtered_rv in zip(raw, filtered, strict=True):
        error = (
            Rotation.from_rotvec(filtered_rv).inv() * Rotation.from_rotvec(raw_rv)
        ).as_rotvec()
        max_error = max(max_error, float(np.linalg.norm(error)))
    return max_error


def _compute_filter_metrics(
    raw: FullBodyReference, filtered: FullBodyReference, dt_s: float
) -> dict[str, float]:
    joint_lag = np.linalg.norm(raw.smpl_joints - filtered.smpl_joints, axis=1)
    joint_pos_lag = np.abs(raw.joint_pos - filtered.joint_pos)
    wrist_joint_pos_lag = joint_pos_lag[G1_WRIST_JOINT_IDX_ISAACLAB]
    return {
        "pose_filter_enabled": 1.0,
        "pose_filter_dt_s": float(dt_s),
        "pose_smpl_joint_lag_max_m": float(np.max(joint_lag)),
        "pose_smpl_pose_lag_max_rad": _max_rotvec_error_rad(raw.smpl_pose, filtered.smpl_pose),
        "pose_joint_pos_lag_max_rad": float(np.max(joint_pos_lag)),
        "pose_wrist_joint_pos_lag_max_rad": float(np.max(wrist_joint_pos_lag)),
        "pose_root_tilt_raw_rad": _root_tilt_rad(raw.body_quat_w),
        "pose_root_tilt_filtered_rad": _root_tilt_rad(filtered.body_quat_w),
    }
