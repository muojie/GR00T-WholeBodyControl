"""Optional G1 upper-body IK for mocap-backed teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from gear_sonic.utils.teleop.retarget.vr3pt_retargeter import VR3PointTarget
from gear_sonic.utils.teleop.vis.vr3pt_pose_visualizer import (
    G1_KEY_FRAME_OFFSETS,
    G1_LEFT_WRIST_FRAME,
    G1_RIGHT_WRIST_FRAME,
)


UPPER_BODY_JOINT_NAMES = (
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)


@dataclass
class UpperBodyIKTarget:
    position: np.ndarray
    velocity: np.ndarray
    metrics: dict[str, float]


class UpperBodyIKRetargeter:
    """Solve a damped least-squares G1 upper-body target from VR3PT wrists.

    The output order matches deploy-side `upper_body_position`: waist first,
    then left arm, then right arm in MuJoCo body-joint order.
    """

    _WRIST_SPECS = (
        ("left_wrist", G1_LEFT_WRIST_FRAME, 0),
        ("right_wrist", G1_RIGHT_WRIST_FRAME, 1),
    )

    def __init__(
        self,
        iterations: int = 8,
        damping: float = 0.08,
        position_weight: float = 1.0,
        orientation_weight: float = 0.15,
        posture_weight: float = 0.03,
        step_size: float = 0.7,
        max_joint_step: float = 0.08,
        robot_model: Any | None = None,
        log_prefix: str = "UpperBodyIKRetargeter",
    ):
        self.iterations = max(1, int(iterations))
        self.damping = max(1e-6, float(damping))
        self.position_weight = max(0.0, float(position_weight))
        self.orientation_weight = max(0.0, float(orientation_weight))
        self.posture_weight = max(0.0, float(posture_weight))
        self.step_size = max(0.0, float(step_size))
        self.max_joint_step = max(0.0, float(max_joint_step))
        self.log_prefix = log_prefix

        self._robot_model = robot_model
        self._joint_indices: list[int] | None = None
        self._default_q: np.ndarray | None = None
        self._last_q: np.ndarray | None = None
        self._last_output: np.ndarray | None = None
        self._last_time_s: float | None = None
        self._last_metrics: dict[str, float] = {}

    def reset(self) -> None:
        self._last_q = None
        self._last_output = None
        self._last_time_s = None
        self._last_metrics = {}

    @property
    def diagnostics(self) -> dict[str, float]:
        return dict(self._last_metrics)

    def preload(self) -> None:
        self._ensure_robot_model()

    def build_target(
        self, vr_target: VR3PointTarget, timestamp_s: float
    ) -> UpperBodyIKTarget | None:
        self._ensure_robot_model()
        assert self._robot_model is not None
        assert self._joint_indices is not None
        assert self._default_q is not None

        vr_position = np.asarray(vr_target.position, dtype=np.float64).reshape(3, 3)
        vr_orientation = np.asarray(vr_target.orientation, dtype=np.float64).reshape(3, 4)

        q = self._last_q.copy() if self._last_q is not None else self._default_q.copy()
        default_control = self._default_q[self._joint_indices]

        for _ in range(self.iterations):
            residual_parts = []
            jacobian_parts = []

            self._robot_model.cache_forward_kinematics(q, auto_clip=False)
            for key, frame_name, target_idx in self._WRIST_SPECS:
                placement = self._robot_model.frame_placement(frame_name)
                offset = np.asarray(G1_KEY_FRAME_OFFSETS[key], dtype=np.float64)
                world_offset = placement.rotation @ offset
                point_position = placement.translation + world_offset

                position_error = vr_position[target_idx] - point_position
                current_rot = Rotation.from_matrix(placement.rotation)
                target_rot = Rotation.from_quat(
                    vr_orientation[target_idx],
                    scalar_first=True,
                )
                orientation_error = (target_rot * current_rot.inv()).as_rotvec()

                jacobian = self._robot_model.frame_jacobian(
                    frame_name,
                    q,
                    reference_frame=pin.LOCAL_WORLD_ALIGNED,
                )
                point_jacobian = jacobian[:3] - _skew(world_offset) @ jacobian[3:6]
                control_jacobian = jacobian[:, self._joint_indices]
                point_control_jacobian = point_jacobian[:, self._joint_indices]

                residual_parts.append(self.position_weight * position_error)
                jacobian_parts.append(self.position_weight * point_control_jacobian)
                if self.orientation_weight > 0.0:
                    residual_parts.append(self.orientation_weight * orientation_error)
                    jacobian_parts.append(self.orientation_weight * control_jacobian[3:6])

            if self.posture_weight > 0.0:
                residual_parts.append(self.posture_weight * (default_control - q[self._joint_indices]))
                jacobian_parts.append(self.posture_weight * np.eye(len(self._joint_indices)))

            residual = np.concatenate(residual_parts)
            jacobian = np.vstack(jacobian_parts)
            lhs = jacobian.T @ jacobian
            lhs += (self.damping**2) * np.eye(lhs.shape[0])
            rhs = jacobian.T @ residual
            try:
                delta = np.linalg.solve(lhs, rhs)
            except np.linalg.LinAlgError:
                delta = np.linalg.lstsq(lhs, rhs, rcond=None)[0]

            delta *= self.step_size
            if self.max_joint_step > 0.0:
                delta = np.clip(delta, -self.max_joint_step, self.max_joint_step)
            q[self._joint_indices] += delta
            q = self._robot_model.clip_configuration(q)

        output = q[self._joint_indices].astype(np.float32)
        dt = 0.0 if self._last_time_s is None else max(1e-3, timestamp_s - self._last_time_s)
        if self._last_output is None or dt <= 0.0:
            velocity = np.zeros_like(output)
        else:
            velocity = ((output - self._last_output) / dt).astype(np.float32)

        metrics = self._compute_metrics(q, vr_position)
        self._last_q = q.copy()
        self._last_output = output.copy()
        self._last_time_s = float(timestamp_s)
        self._last_metrics = metrics

        return UpperBodyIKTarget(position=output, velocity=velocity, metrics=metrics)

    def _ensure_robot_model(self) -> None:
        if self._robot_model is None:
            from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model

            self._robot_model = instantiate_g1_robot_model(
                waist_location="lower_and_upper_body",
                high_elbow_pose=False,
            )

        if self._joint_indices is None:
            self._joint_indices = [
                self._robot_model.dof_index(joint_name) for joint_name in UPPER_BODY_JOINT_NAMES
            ]
        if self._default_q is None:
            self._default_q = self._robot_model.default_body_pose.copy()

    def _compute_metrics(self, q: np.ndarray, vr_position: np.ndarray) -> dict[str, float]:
        assert self._robot_model is not None
        self._robot_model.cache_forward_kinematics(q, auto_clip=False)

        errors = []
        for key, frame_name, target_idx in self._WRIST_SPECS:
            placement = self._robot_model.frame_placement(frame_name)
            offset = np.asarray(G1_KEY_FRAME_OFFSETS[key], dtype=np.float64)
            point_position = placement.translation + placement.rotation @ offset
            errors.append(float(np.linalg.norm(vr_position[target_idx] - point_position)))

        controlled = q[self._joint_indices] if self._joint_indices is not None else np.array([])
        default = self._default_q[self._joint_indices] if self._joint_indices is not None else controlled
        limits_lower = self._robot_model.lower_joint_limits[self._joint_indices]
        limits_upper = self._robot_model.upper_joint_limits[self._joint_indices]
        limit_margin = np.minimum(controlled - limits_lower, limits_upper - controlled)

        return {
            "upper_body_ik_enabled": 1.0,
            "upper_body_ik_max_wrist_error_m": float(max(errors)),
            "upper_body_ik_mean_wrist_error_m": float(np.mean(errors)),
            "upper_body_ik_max_abs_joint_rad": float(np.max(np.abs(controlled))),
            "upper_body_ik_max_default_delta_rad": float(np.max(np.abs(controlled - default))),
            "upper_body_ik_min_limit_margin_rad": float(np.min(limit_margin)),
        }


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=np.float64).reshape(3)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=np.float64,
    )
