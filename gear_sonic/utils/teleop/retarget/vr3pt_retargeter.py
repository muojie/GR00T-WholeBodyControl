"""Utilities for converting mocap frames into deploy-side VR 3-point targets."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from gear_sonic.utils.teleop.sources.base import MocapFrame, Pose7D, normalize_quat_wxyz


DEFAULT_VR_POSITION = np.array(
    [
        0.0903,
        0.1615,
        -0.2411,
        0.1280,
        -0.1522,
        -0.2461,
        0.0241,
        -0.0081,
        0.4028,
    ],
    dtype=np.float32,
)
DEFAULT_VR_ORIENTATION = np.array(
    [
        0.7295,
        0.3145,
        0.5533,
        -0.2506,
        0.7320,
        -0.2639,
        0.5395,
        0.3217,
        0.9991,
        0.011,
        0.0402,
        -0.0002,
    ],
    dtype=np.float32,
)


@dataclass
class VR3PointTarget:
    position: np.ndarray
    orientation: np.ndarray
    metrics: dict[str, float] | None = None


class VR3PointRetargeter:
    """Extract, calibrate, and smooth left wrist, right wrist, and head targets."""

    TORSO_LINK_OFFSET_Z = 0.05
    NECK_LINK_LENGTH = 0.35

    def __init__(
        self,
        calibrate_on_first_frame: bool = True,
        position_scale: float = 1.0,
        allow_bone_translation_vr: bool = False,
        fk_calibration: bool = True,
        require_fk_calibration: bool = False,
        enable_filter: bool = True,
        position_alpha: float = 0.45,
        orientation_alpha: float = 0.45,
        max_position_speed: float = 3.0,
        max_position_accel: float = 25.0,
        max_angular_speed: float = 8.0,
        robot_model: Any | None = None,
        log_prefix: str = "VR3PointRetargeter",
    ):
        self.calibrate_on_first_frame = bool(calibrate_on_first_frame)
        self.position_scale = float(position_scale)
        self.allow_bone_translation_vr = bool(allow_bone_translation_vr)

        self.fk_calibration = bool(fk_calibration)
        self.require_fk_calibration = bool(require_fk_calibration)
        self.enable_filter = bool(enable_filter)
        self.position_alpha = _clamp01(position_alpha)
        self.orientation_alpha = _clamp01(orientation_alpha)
        self.max_position_speed = float(max_position_speed)
        self.max_position_accel = float(max_position_accel)
        self.max_angular_speed = float(max_angular_speed)
        self.log_prefix = log_prefix

        self._robot_model = robot_model
        self._get_g1_key_frame_poses = None
        self._fk_warning_printed = False

        self._position_offset: np.ndarray | None = None
        self._calibration_neck_quat_inv: np.ndarray | None = None
        self._calibration_lwrist_offset: np.ndarray | None = None
        self._calibration_rwrist_offset: np.ndarray | None = None
        self._calibration_lwrist_rot_offset: Rotation | None = None
        self._calibration_rwrist_rot_offset: Rotation | None = None

        self._last_filter_time_s: float | None = None
        self._last_filtered_position: np.ndarray | None = None
        self._last_filtered_orientation: np.ndarray | None = None
        self._last_velocity: np.ndarray | None = None
        self._last_metrics: dict[str, float] = {}

    def reset(self) -> None:
        self._position_offset = None
        self._calibration_neck_quat_inv = None
        self._calibration_lwrist_offset = None
        self._calibration_rwrist_offset = None
        self._calibration_lwrist_rot_offset = None
        self._calibration_rwrist_rot_offset = None
        self.reset_filter()

    def reset_filter(self) -> None:
        self._last_filter_time_s = None
        self._last_filtered_position = None
        self._last_filtered_orientation = None
        self._last_velocity = None
        self._last_metrics = {}

    @property
    def diagnostics(self) -> dict[str, float]:
        return dict(self._last_metrics)

    def preload(self) -> None:
        """Load optional FK dependencies before live mocap frames start arriving."""
        if not self.fk_calibration:
            return
        try:
            self._ensure_fk_dependencies()
        except Exception as exc:
            if self.require_fk_calibration:
                raise
            if not self._fk_warning_printed:
                print(
                    f"[{self.log_prefix}] FK calibration unavailable, "
                    f"falling back to position-only calibration: {exc}"
                )
                self._fk_warning_printed = True
            self.fk_calibration = False

    def build_target(self, frame: MocapFrame) -> VR3PointTarget | None:
        if frame.direct_vr_position is not None:
            position = frame.direct_vr_position.copy()
            orientation = (
                frame.direct_vr_orientation.copy()
                if frame.direct_vr_orientation is not None
                else DEFAULT_VR_ORIENTATION.copy()
            )
            return self._filter_target(position, orientation, frame.host_time_s)

        source_pose = self._extract_pose_from_joints(frame)
        if source_pose is None:
            return None

        source_pose[:, :3] *= self.position_scale
        if self.calibrate_on_first_frame:
            target_pose = self._calibrate_pose(source_pose)
        else:
            target_pose = source_pose

        position = target_pose[:, :3].reshape(-1).astype(np.float32)
        orientation = target_pose[:, 3:7].reshape(-1).astype(np.float32)
        return self._filter_target(position, orientation, frame.host_time_s)

    def _extract_pose_from_joints(self, frame: MocapFrame) -> np.ndarray | None:
        left = _find_joint(frame, ("left_wrist", "left_hand", "l_hand", "left_controller"))
        right = _find_joint(frame, ("right_wrist", "right_hand", "r_hand", "right_controller"))
        head = _find_joint(frame, ("head", "neck", "neck_2", "head_tracker"))

        if self.allow_bone_translation_vr and (left is None or right is None):
            left = left or frame.bones.get(14)
            right = right or frame.bones.get(18)
            head = head or frame.bones.get(10) or frame.bones.get(9)

        if left is None or right is None:
            return None

        pose = np.concatenate(
            (
                DEFAULT_VR_POSITION.reshape(3, 3),
                DEFAULT_VR_ORIENTATION.reshape(3, 4),
            ),
            axis=1,
        ).astype(np.float32)
        pose[0, :3] = left.position
        pose[0, 3:7] = left.quat_wxyz
        pose[1, :3] = right.position
        pose[1, 3:7] = right.quat_wxyz

        if head is not None:
            pose[2, :3] = head.position
            pose[2, 3:7] = head.quat_wxyz

        return pose

    def _calibrate_pose(self, pose: np.ndarray) -> np.ndarray:
        if self.fk_calibration:
            try:
                return self._apply_fk_calibration(pose)
            except Exception as exc:
                if self.require_fk_calibration:
                    raise
                if not self._fk_warning_printed:
                    print(
                        f"[{self.log_prefix}] FK calibration unavailable, "
                        f"falling back to position-only calibration: {exc}"
                    )
                    self._fk_warning_printed = True
                self.fk_calibration = False

        return self._apply_legacy_position_calibration(pose)

    def _apply_legacy_position_calibration(self, pose: np.ndarray) -> np.ndarray:
        calibrated = pose.copy()
        position = calibrated[:, :3].reshape(-1)
        if self._position_offset is None:
            self._position_offset = DEFAULT_VR_POSITION - position
        calibrated[:, :3] = (position + self._position_offset).reshape(3, 3)
        return calibrated

    def _apply_fk_calibration(self, pose: np.ndarray) -> np.ndarray:
        if self._calibration_neck_quat_inv is None:
            self._capture_fk_calibration(pose)

        assert self._calibration_neck_quat_inv is not None
        calib_inv_rot = Rotation.from_quat(self._calibration_neck_quat_inv, scalar_first=True)
        calibrated = pose.copy()

        head_rot = Rotation.from_quat(pose[2, 3:7], scalar_first=True)
        calibrated[2, 3:7] = (calib_inv_rot * head_rot).as_quat(scalar_first=True)

        if self._calibration_lwrist_offset is not None:
            calibrated[0, :3] = calib_inv_rot.apply(pose[0, :3]) - self._calibration_lwrist_offset
        if self._calibration_rwrist_offset is not None:
            calibrated[1, :3] = calib_inv_rot.apply(pose[1, :3]) - self._calibration_rwrist_offset

        if self._calibration_lwrist_rot_offset is not None:
            left_corrected = calib_inv_rot * Rotation.from_quat(pose[0, 3:7], scalar_first=True)
            calibrated[0, 3:7] = (
                self._calibration_lwrist_rot_offset * left_corrected
            ).as_quat(scalar_first=True)
        if self._calibration_rwrist_rot_offset is not None:
            right_corrected = calib_inv_rot * Rotation.from_quat(pose[1, 3:7], scalar_first=True)
            calibrated[1, 3:7] = (
                self._calibration_rwrist_rot_offset * right_corrected
            ).as_quat(scalar_first=True)

        neck_z = Rotation.from_quat(calibrated[2, 3:7], scalar_first=True).apply([0.0, 0.0, 1.0])
        calibrated[2, :3] = np.array([0.0, 0.0, self.TORSO_LINK_OFFSET_Z], dtype=np.float32) + (
            self.NECK_LINK_LENGTH * neck_z
        )
        return calibrated.astype(np.float32)

    def _capture_fk_calibration(self, pose: np.ndarray) -> None:
        self._ensure_fk_dependencies()

        neck_rot = Rotation.from_quat(pose[2, 3:7], scalar_first=True)
        self._calibration_neck_quat_inv = neck_rot.inv().as_quat(scalar_first=True)
        calib_inv_rot = Rotation.from_quat(self._calibration_neck_quat_inv, scalar_first=True)

        left_pos_corrected = calib_inv_rot.apply(pose[0, :3])
        right_pos_corrected = calib_inv_rot.apply(pose[1, :3])
        left_rot_corrected = calib_inv_rot * Rotation.from_quat(pose[0, 3:7], scalar_first=True)
        right_rot_corrected = calib_inv_rot * Rotation.from_quat(pose[1, 3:7], scalar_first=True)

        g1_poses = self._get_g1_key_frame_poses(self._robot_model)
        g1_left_pos = g1_poses["left_wrist"]["position"]
        g1_right_pos = g1_poses["right_wrist"]["position"]
        g1_left_rot = Rotation.from_quat(
            g1_poses["left_wrist"]["orientation_wxyz"], scalar_first=True
        )
        g1_right_rot = Rotation.from_quat(
            g1_poses["right_wrist"]["orientation_wxyz"], scalar_first=True
        )

        self._calibration_lwrist_offset = left_pos_corrected - g1_left_pos
        self._calibration_rwrist_offset = right_pos_corrected - g1_right_pos
        self._calibration_lwrist_rot_offset = g1_left_rot * left_rot_corrected.inv()
        self._calibration_rwrist_rot_offset = g1_right_rot * right_rot_corrected.inv()
        print(
            f"[{self.log_prefix}] FK calibration captured: "
            f"L offset={self._calibration_lwrist_offset.round(4).tolist()} "
            f"R offset={self._calibration_rwrist_offset.round(4).tolist()}"
        )

    def _ensure_fk_dependencies(self) -> None:
        if self._get_g1_key_frame_poses is None:
            from gear_sonic.utils.teleop.vis.vr3pt_pose_visualizer import (
                get_g1_key_frame_poses,
            )

            self._get_g1_key_frame_poses = get_g1_key_frame_poses

        if self._robot_model is None:
            from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model

            self._robot_model = instantiate_g1_robot_model()

    def _filter_target(
        self, position: np.ndarray, orientation: np.ndarray, timestamp_s: float
    ) -> VR3PointTarget:
        position = np.asarray(position, dtype=np.float32).reshape(3, 3)
        orientation = _normalize_orientation_array(orientation)

        if not self.enable_filter:
            metrics = self._compute_metrics(position, position, np.zeros_like(position))
            self._last_metrics = metrics
            return VR3PointTarget(
                position=position.reshape(-1),
                orientation=orientation.reshape(-1),
                metrics=metrics,
            )

        if (
            self._last_filter_time_s is None
            or self._last_filtered_position is None
            or self._last_filtered_orientation is None
        ):
            self._last_filter_time_s = float(timestamp_s)
            self._last_filtered_position = position.copy()
            self._last_filtered_orientation = orientation.copy()
            self._last_velocity = np.zeros_like(position)
            metrics = self._compute_metrics(position, position, self._last_velocity)
            self._last_metrics = metrics
            return VR3PointTarget(
                position=position.reshape(-1),
                orientation=orientation.reshape(-1),
                metrics=metrics,
            )

        dt = max(1e-3, min(0.2, float(timestamp_s) - self._last_filter_time_s))
        last_position = self._last_filtered_position
        last_orientation = self._last_filtered_orientation
        last_velocity = (
            self._last_velocity if self._last_velocity is not None else np.zeros_like(last_position)
        )

        limited_position = _limit_position_step(
            last_position,
            position,
            max_step=self.max_position_speed * dt if self.max_position_speed > 0.0 else None,
        )
        smoothed_position = last_position + self.position_alpha * (limited_position - last_position)

        if self.max_position_accel > 0.0:
            desired_velocity = (smoothed_position - last_position) / dt
            velocity = _limit_position_step(
                last_velocity,
                desired_velocity,
                max_step=self.max_position_accel * dt,
            )
            smoothed_position = last_position + velocity * dt
        else:
            velocity = (smoothed_position - last_position) / dt

        limited_orientation = np.vstack(
            [
                _limit_quat_step(last_orientation[i], orientation[i], self.max_angular_speed * dt)
                if self.max_angular_speed > 0.0
                else orientation[i]
                for i in range(3)
            ]
        )
        smoothed_orientation = np.vstack(
            [
                _slerp_quat_wxyz(last_orientation[i], limited_orientation[i], self.orientation_alpha)
                for i in range(3)
            ]
        ).astype(np.float32)

        self._last_filter_time_s = float(timestamp_s)
        self._last_filtered_position = smoothed_position.astype(np.float32)
        self._last_filtered_orientation = smoothed_orientation
        self._last_velocity = velocity.astype(np.float32)
        metrics = self._compute_metrics(
            position,
            self._last_filtered_position,
            self._last_velocity,
        )
        self._last_metrics = metrics

        return VR3PointTarget(
            position=self._last_filtered_position.reshape(-1),
            orientation=self._last_filtered_orientation.reshape(-1),
            metrics=metrics,
        )

    def _compute_metrics(
        self, raw_position: np.ndarray, filtered_position: np.ndarray, velocity: np.ndarray
    ) -> dict[str, float]:
        raw_position = np.asarray(raw_position, dtype=np.float32).reshape(3, 3)
        filtered_position = np.asarray(filtered_position, dtype=np.float32).reshape(3, 3)
        velocity = np.asarray(velocity, dtype=np.float32).reshape(3, 3)

        filter_delta = np.linalg.norm(raw_position - filtered_position, axis=1)
        speeds = np.linalg.norm(velocity, axis=1)
        wrist_span = float(np.linalg.norm(filtered_position[0] - filtered_position[1]))
        left_height = float(filtered_position[0, 2])
        right_height = float(filtered_position[1, 2])
        head_height = float(filtered_position[2, 2])

        return {
            "wrist_span_m": wrist_span,
            "left_wrist_height_m": left_height,
            "right_wrist_height_m": right_height,
            "head_height_m": head_height,
            "max_filter_delta_m": float(np.max(filter_delta)),
            "max_speed_mps": float(np.max(speeds)),
            "fk_calibrated": 1.0 if self._calibration_neck_quat_inv is not None else 0.0,
            "filter_enabled": 1.0 if self.enable_filter else 0.0,
        }


def _find_joint(frame: MocapFrame, aliases: tuple[str, ...]) -> Pose7D | None:
    for alias in aliases:
        pose = frame.joints.get(alias)
        if pose is not None:
            return pose
    return None


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _normalize_orientation_array(orientation: np.ndarray) -> np.ndarray:
    quats = np.asarray(orientation, dtype=np.float32).reshape(3, 4)
    return np.vstack([normalize_quat_wxyz(quat) for quat in quats]).astype(np.float32)


def _limit_position_step(
    previous: np.ndarray, target: np.ndarray, max_step: float | None
) -> np.ndarray:
    if max_step is None:
        return target
    delta = target - previous
    norms = np.linalg.norm(delta, axis=1, keepdims=True)
    scale = np.ones_like(norms)
    mask = norms[:, 0] > max_step
    scale[mask, 0] = max_step / np.maximum(norms[mask, 0], 1e-8)
    return previous + delta * scale


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
