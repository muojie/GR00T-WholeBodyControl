"""BVH to G1 joint-reference playback source.

This source keeps the runtime path compatible with the robot-filtered PKL route:
BVH frames are retargeted to G1 29-DOF joint references and published through
POSE protocol v1 / encoder mode 0.
"""

from __future__ import annotations

import os.path as osp
import threading
import time
from dataclasses import dataclass
from typing import Any

import joblib
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from gear_sonic.utils.teleop.sources.base import (
    FullBodyReference,
    G1_DEFAULT_JOINT_POS_ISAACLAB,
    G1_DEFAULT_ROOT_POS_W,
    G1_MUJOCO_TO_ISAACLAB_IDX,
    G1_WRIST_JOINT_IDX_ISAACLAB,
    MocapFrame,
    Pose7D,
    normalize_quat_wxyz,
    smpl_pose_to_g1_wrist_joint_pos,
)
from gear_sonic.utils.teleop.sources.bvh_source import (
    _build_full_body_reference,
    _find_first_joint_index,
    _rotation_from_wxyz,
    load_bvh_motion,
)
from gear_sonic.utils.teleop.sources.g1_body_fk import G1BodyFk
from gear_sonic.utils.teleop.sources.robot_pkl_source import RobotPklMotion


G1_JOINT_LOWER_LIMIT_MUJOCO = np.array(
    [
        -2.5,
        -0.8,
        -1.2,
        -0.1,
        -1.2,
        -0.7,
        -2.5,
        -0.8,
        -1.2,
        -0.1,
        -1.2,
        -0.7,
        -1.4,
        -0.8,
        -0.8,
        -2.8,
        -1.8,
        -2.2,
        -0.3,
        -1.97222,
        -1.61443,
        -1.61443,
        -2.8,
        -1.8,
        -2.2,
        -0.3,
        -1.97222,
        -1.61443,
        -1.61443,
    ],
    dtype=np.float32,
)
G1_JOINT_UPPER_LIMIT_MUJOCO = np.array(
    [
        2.5,
        0.8,
        1.2,
        2.4,
        0.8,
        0.7,
        2.5,
        0.8,
        1.2,
        2.4,
        0.8,
        0.7,
        1.4,
        0.8,
        0.8,
        2.8,
        1.8,
        2.2,
        2.2,
        1.97222,
        1.61443,
        1.61443,
        2.8,
        1.8,
        2.2,
        2.2,
        1.97222,
        1.61443,
        1.61443,
    ],
    dtype=np.float32,
)
G1_JOINT_LOWER_LIMIT_ISAACLAB = np.empty_like(G1_JOINT_LOWER_LIMIT_MUJOCO)
G1_JOINT_UPPER_LIMIT_ISAACLAB = np.empty_like(G1_JOINT_UPPER_LIMIT_MUJOCO)
G1_JOINT_LOWER_LIMIT_ISAACLAB[G1_MUJOCO_TO_ISAACLAB_IDX] = G1_JOINT_LOWER_LIMIT_MUJOCO
G1_JOINT_UPPER_LIMIT_ISAACLAB[G1_MUJOCO_TO_ISAACLAB_IDX] = G1_JOINT_UPPER_LIMIT_MUJOCO
G1_JOINT_DEFAULT_DELTA_LIMIT_MUJOCO = np.array(
    [
        1.00,
        0.45,
        0.50,
        0.85,
        0.45,
        0.35,
        1.00,
        0.45,
        0.50,
        0.85,
        0.45,
        0.35,
        0.45,
        0.30,
        0.30,
        1.00,
        1.00,
        1.00,
        1.00,
        0.90,
        0.80,
        0.80,
        1.00,
        1.00,
        1.00,
        1.00,
        0.90,
        0.80,
        0.80,
    ],
    dtype=np.float32,
)
G1_SKELETON_JOINT_SIGN_MUJOCO = np.ones(29, dtype=np.float32)
G1_SKELETON_JOINT_SIGN_MUJOCO[
    [
        1,   # left_hip_roll_joint
        2,   # left_hip_yaw_joint
        5,   # left_ankle_roll_joint
        7,   # right_hip_roll_joint
        8,   # right_hip_yaw_joint
        18,  # left_elbow_joint
        21,  # left_wrist_yaw_joint
        25,  # right_elbow_joint
        26,  # right_wrist_roll_joint
        28,  # right_wrist_yaw_joint
    ]
] = -1.0


@dataclass
class BvhG1RetargetConfig:
    method: str = "skeleton"
    retarget_scale: float = 1.0
    lower_body_scale: float = 0.60
    upper_body_scale: float = 0.85
    wrist_scale: float = 0.55
    waist_scale: float = 0.25
    max_joint_velocity_radps: float = 6.0
    max_joint_step_rad: float = 0.0
    joint_filter_alpha: float = 0.45
    joint_limit_margin_rad: float = 0.05
    joint_delta_limit_scale: float = 0.8
    skeleton_sign_correction: bool = True
    skeleton_segment_direction: str = "all"
    skeleton_axis_mapping: str = "bvh_y_forward"
    ik_mode: str = "numeric"
    root_mode: str = "yaw"
    max_root_angular_velocity_radps: float = 0.0
    root_tilt_limit_rad: float = 0.25
    min_root_height_m: float = 0.74
    enable_body_fk: bool = True


@dataclass
class BvhG1RetargetContext:
    bvh_motion: Any
    frame_indices: np.ndarray
    root_pos: np.ndarray
    root_quat: np.ndarray
    source_indices: dict[str, int]
    fk: G1BodyFk
    default_body_rots: list[Rotation]
    default_segment_dirs: dict[str, np.ndarray]
    enabled_segment_keys: frozenset[str]
    source0_rots: dict[str, Rotation]
    root0_pos: np.ndarray | None
    root0_inv: Rotation | None
    config: BvhG1RetargetConfig

    @property
    def frame_count(self) -> int:
        return int(self.frame_indices.shape[0])

    @property
    def playback_fps(self) -> float:
        return float(self.bvh_motion.playback_fps)

    @property
    def source_fps(self) -> float:
        return float(self.bvh_motion.source_fps)

    @property
    def path(self) -> str:
        return str(self.bvh_motion.path)

    @property
    def motion_name(self) -> str:
        return osp.splitext(osp.basename(self.bvh_motion.path))[0]


BVH_G1_SOURCE_ALIASES = {
    "root": ("root", "Hips", "Hip", "Pelvis", "Root"),
    "spine": ("torso_2", "Spine", "Spine1", "Spine01", "mixamorig:Spine"),
    "chest": ("torso_6", "torso_5", "UpperChest", "Chest", "Spine2", "Spine3"),
    "left_upper_arm": (
        "l_up_arm",
        "LeftArm",
        "LeftUpperArm",
        "L_UpperArm",
        "mixamorig:LeftArm",
    ),
    "left_lower_arm": (
        "l_low_arm",
        "LeftForeArm",
        "LeftLowerArm",
        "L_ForeArm",
        "mixamorig:LeftForeArm",
    ),
    "left_hand": ("l_hand", "LeftHand", "LeftWrist", "L_Hand", "mixamorig:LeftHand"),
    "right_upper_arm": (
        "r_up_arm",
        "RightArm",
        "RightUpperArm",
        "R_UpperArm",
        "mixamorig:RightArm",
    ),
    "right_lower_arm": (
        "r_low_arm",
        "RightForeArm",
        "RightLowerArm",
        "R_ForeArm",
        "mixamorig:RightForeArm",
    ),
    "right_hand": ("r_hand", "RightHand", "RightWrist", "R_Hand", "mixamorig:RightHand"),
    "left_upper_leg": (
        "l_up_leg",
        "LeftUpLeg",
        "LeftUpperLeg",
        "LeftThigh",
        "mixamorig:LeftUpLeg",
    ),
    "left_lower_leg": (
        "l_low_leg",
        "LeftLeg",
        "LeftLowerLeg",
        "LeftShin",
        "mixamorig:LeftLeg",
    ),
    "left_foot": ("l_foot", "LeftFoot", "LeftAnkle", "L_Foot", "mixamorig:LeftFoot"),
    "right_upper_leg": (
        "r_up_leg",
        "RightUpLeg",
        "RightUpperLeg",
        "RightThigh",
        "mixamorig:RightUpLeg",
    ),
    "right_lower_leg": (
        "r_low_leg",
        "RightLeg",
        "RightLowerLeg",
        "RightShin",
        "mixamorig:RightLeg",
    ),
    "right_foot": ("r_foot", "RightFoot", "RightAnkle", "R_Foot", "mixamorig:RightFoot"),
}


G1_BODY_TO_BVH_SOURCE_KEY = {
    "left_hip_pitch_link": "left_upper_leg",
    "left_hip_roll_link": "left_upper_leg",
    "left_hip_yaw_link": "left_upper_leg",
    "left_knee_link": "left_lower_leg",
    "left_ankle_pitch_link": "left_foot",
    "left_ankle_roll_link": "left_foot",
    "right_hip_pitch_link": "right_upper_leg",
    "right_hip_roll_link": "right_upper_leg",
    "right_hip_yaw_link": "right_upper_leg",
    "right_knee_link": "right_lower_leg",
    "right_ankle_pitch_link": "right_foot",
    "right_ankle_roll_link": "right_foot",
    "waist_yaw_link": "chest",
    "waist_roll_link": "chest",
    "torso_link": "chest",
    "left_shoulder_pitch_link": "left_upper_arm",
    "left_shoulder_roll_link": "left_upper_arm",
    "left_shoulder_yaw_link": "left_upper_arm",
    "left_elbow_link": "left_lower_arm",
    "left_wrist_roll_link": "left_hand",
    "left_wrist_pitch_link": "left_hand",
    "left_wrist_yaw_link": "left_hand",
    "right_shoulder_pitch_link": "right_upper_arm",
    "right_shoulder_roll_link": "right_upper_arm",
    "right_shoulder_yaw_link": "right_upper_arm",
    "right_elbow_link": "right_lower_arm",
    "right_wrist_roll_link": "right_hand",
    "right_wrist_pitch_link": "right_hand",
    "right_wrist_yaw_link": "right_hand",
}

BVH_G1_SEGMENT_SOURCE_POINTS = {
    "left_upper_arm": ("left_upper_arm", "left_lower_arm"),
    "left_forearm": ("left_lower_arm", "left_hand"),
    "right_upper_arm": ("right_upper_arm", "right_lower_arm"),
    "right_forearm": ("right_lower_arm", "right_hand"),
    "left_thigh": ("left_upper_leg", "left_lower_leg"),
    "left_shin": ("left_lower_leg", "left_foot"),
    "right_thigh": ("right_upper_leg", "right_lower_leg"),
    "right_shin": ("right_lower_leg", "right_foot"),
}

G1_SEGMENT_BODY_POINTS = {
    "left_upper_arm": ("left_shoulder_roll_link", "left_elbow_link"),
    "left_forearm": ("left_elbow_link", "left_wrist_yaw_link"),
    "right_upper_arm": ("right_shoulder_roll_link", "right_elbow_link"),
    "right_forearm": ("right_elbow_link", "right_wrist_yaw_link"),
    "left_thigh": ("left_hip_roll_link", "left_knee_link"),
    "left_shin": ("left_knee_link", "left_ankle_roll_link"),
    "right_thigh": ("right_hip_roll_link", "right_knee_link"),
    "right_shin": ("right_knee_link", "right_ankle_roll_link"),
}

G1_BODY_TO_BVH_SEGMENT_KEY = {
    "left_shoulder_pitch_link": "left_upper_arm",
    "left_shoulder_roll_link": "left_upper_arm",
    "left_shoulder_yaw_link": "left_upper_arm",
    "right_shoulder_pitch_link": "right_upper_arm",
    "right_shoulder_roll_link": "right_upper_arm",
    "right_shoulder_yaw_link": "right_upper_arm",
    "left_hip_pitch_link": "left_thigh",
    "left_hip_roll_link": "left_thigh",
    "left_hip_yaw_link": "left_thigh",
    "left_knee_link": "left_shin",
    "left_ankle_pitch_link": "left_shin",
    "left_ankle_roll_link": "left_shin",
    "right_hip_pitch_link": "right_thigh",
    "right_hip_roll_link": "right_thigh",
    "right_hip_yaw_link": "right_thigh",
    "right_knee_link": "right_shin",
    "right_ankle_pitch_link": "right_shin",
    "right_ankle_roll_link": "right_shin",
}

UPPER_SEGMENT_KEYS = frozenset(
    ("left_upper_arm", "left_forearm", "right_upper_arm", "right_forearm")
)
ALL_SEGMENT_KEYS = frozenset(BVH_G1_SEGMENT_SOURCE_POINTS)


class BvhG1PlaybackSource:
    """Replay BVH as a G1 joint-reference stream."""

    def __init__(
        self,
        bvh_file: str,
        target_fps: float | None = None,
        loop: bool = False,
        unit_scale: float = 0.01,
        y_up_to_z_up: bool = True,
        local_root: bool = False,
        align_root: bool = True,
        retarget_config: BvhG1RetargetConfig | None = None,
        runtime_mode: str = "precompute",
    ):
        self.bvh_file = bvh_file
        self.target_fps = target_fps
        self.loop = bool(loop)
        self.unit_scale = float(unit_scale)
        self.y_up_to_z_up = bool(y_up_to_z_up)
        self.local_root = bool(local_root)
        self.align_root = bool(align_root)
        self.retarget_config = retarget_config or BvhG1RetargetConfig()
        self.runtime_mode = str(runtime_mode or "precompute").strip().lower()
        if self.runtime_mode not in {"precompute", "online"}:
            raise ValueError(
                f"Unsupported bvh_g1 runtime_mode {runtime_mode!r}; expected precompute or online"
            )

        self.motion: RobotPklMotion | None = None
        self.retarget_context: BvhG1RetargetContext | None = None
        if self.runtime_mode == "precompute":
            self.motion = load_bvh_g1_motion(
                bvh_file=bvh_file,
                target_fps=target_fps,
                unit_scale=unit_scale,
                y_up_to_z_up=y_up_to_z_up,
                local_root=local_root,
                align_root=align_root,
                retarget_config=self.retarget_config,
            )
        else:
            self.retarget_context = prepare_bvh_g1_retarget_context(
                bvh_file=bvh_file,
                target_fps=target_fps,
                unit_scale=unit_scale,
                y_up_to_z_up=y_up_to_z_up,
                local_root=local_root,
                align_root=align_root,
                retarget_config=self.retarget_config,
            )
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._lock = threading.Lock()
        self._latest: MocapFrame | None = None
        self._last_error: str | None = None
        self._frames_emitted = 0
        self._stopped_at_end = False
        self._last_joint_pos: np.ndarray | None = None
        self._last_output_joint_pos: np.ndarray | None = None
        self._last_source_frame_idx: int | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._running.set()
        self._thread = threading.Thread(target=self._run, name="BvhG1PlaybackSource", daemon=True)
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
        fps = self._playback_fps
        with self._lock:
            return {
                "fps": fps,
                "received_packets": self._frames_emitted,
                "frames_emitted": self._frames_emitted,
                "dropped_packets": 0,
                "last_error": self._last_error,
                "has_frame": self._latest is not None,
                "stopped_at_end": self._stopped_at_end,
                "runtime_mode": self.runtime_mode,
            }

    @property
    def _frame_count(self) -> int:
        if self.motion is not None:
            return int(self.motion.frame_count)
        if self.retarget_context is not None:
            return int(self.retarget_context.frame_count)
        return 0

    @property
    def _playback_fps(self) -> float:
        if self.motion is not None:
            return float(self.motion.playback_fps)
        if self.retarget_context is not None:
            return float(self.retarget_context.playback_fps)
        return 0.0

    def _run(self) -> None:
        frame_period_s = 1.0 / max(1.0, self._playback_fps)
        frame_idx = 0
        stream_frame_idx = self._frames_emitted
        next_tick_s = time.time()

        while self._running.is_set():
            try:
                frame = self._build_frame(frame_idx, stream_frame_idx)
            except Exception as exc:
                with self._lock:
                    self._last_error = str(exc)
                break

            with self._lock:
                self._latest = frame
                self._frames_emitted = stream_frame_idx + 1
                self._last_error = None

            stream_frame_idx += 1
            frame_idx += 1
            if frame_idx >= self._frame_count:
                if self.loop:
                    frame_idx = frame_idx % self._frame_count
                    self._last_joint_pos = None
                    self._last_output_joint_pos = None
                    self._last_source_frame_idx = None
                else:
                    with self._lock:
                        self._stopped_at_end = True
                    break

            next_tick_s += frame_period_s
            sleep_s = next_tick_s - time.time()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick_s = time.time()

    def _build_frame(self, frame_idx: int, stream_frame_idx: int) -> MocapFrame:
        if self.motion is None:
            return self._build_online_frame(frame_idx, stream_frame_idx)

        full_body = FullBodyReference(
            smpl_joints=np.zeros((24, 3), dtype=np.float32),
            smpl_pose=np.zeros((21, 3), dtype=np.float32),
            body_quat_w=self.motion.root_quat_wxyz[frame_idx],
            body_pos_w=self.motion.root_pos_w[frame_idx],
            body_pos=self.motion.body_pos[frame_idx] if self.motion.body_pos is not None else None,
            joint_pos=self.motion.joint_pos_isaaclab[frame_idx],
            joint_vel=self.motion.joint_vel_isaaclab[frame_idx],
            frame_index=int(stream_frame_idx),
        )
        root_pose = Pose7D(
            position=self.motion.root_pos_w[frame_idx],
            quat_wxyz=self.motion.root_quat_wxyz[frame_idx],
        )
        return MocapFrame(
            source="bvh_g1",
            host_time_s=time.time(),
            frame_index=int(stream_frame_idx),
            fps=float(self.motion.playback_fps),
            joints={"root": root_pose},
            full_body=full_body,
            metadata={
                "format": "bvh_g1",
                "path": self.motion.path,
                "motion_name": self.motion.motion_name,
                "source_frame_index": int(frame_idx),
                "source_fps": self.motion.source_fps,
            },
        )

    def _build_online_frame(self, frame_idx: int, stream_frame_idx: int) -> MocapFrame:
        if self.retarget_context is None:
            raise RuntimeError("bvh_g1 online retarget context is not initialized")

        context = self.retarget_context
        source_frame_idx = int(context.frame_indices[frame_idx])
        raw_joint_pos = retarget_bvh_g1_frame(context, frame_idx)
        joint_pos = _stabilize_joint_position_step(
            raw_joint_pos,
            self._last_output_joint_pos,
            fps=context.playback_fps,
            max_joint_velocity_radps=context.config.max_joint_velocity_radps,
            max_joint_step_rad=context.config.max_joint_step_rad,
            alpha=context.config.joint_filter_alpha,
        )
        joint_vel = np.zeros_like(joint_pos, dtype=np.float32)
        if (
            self._last_joint_pos is not None
            and self._last_source_frame_idx is not None
            and source_frame_idx > self._last_source_frame_idx
        ):
            dt_s = max(
                1e-3,
                float(source_frame_idx - self._last_source_frame_idx)
                / max(1.0, context.bvh_motion.source_fps),
            )
            joint_vel = ((joint_pos - self._last_joint_pos) / dt_s).astype(np.float32)
            max_vel = float(context.config.max_joint_velocity_radps)
            if max_vel > 0.0:
                joint_vel = np.clip(joint_vel, -max_vel, max_vel).astype(np.float32)

        self._last_joint_pos = joint_pos.copy()
        self._last_output_joint_pos = joint_pos.copy()
        self._last_source_frame_idx = source_frame_idx

        body_pos = None
        if context.config.enable_body_fk:
            body_pos = context.fk.compute_body_pos14_world(
                joint_pos.reshape(1, 29),
                context.root_pos[frame_idx].reshape(1, 3),
                context.root_quat[frame_idx].reshape(1, 4),
            )[0]

        full_body = FullBodyReference(
            smpl_joints=np.zeros((24, 3), dtype=np.float32),
            smpl_pose=np.zeros((21, 3), dtype=np.float32),
            body_quat_w=context.root_quat[frame_idx],
            body_pos_w=context.root_pos[frame_idx],
            body_pos=body_pos,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            frame_index=int(stream_frame_idx),
        )
        root_pose = Pose7D(
            position=context.root_pos[frame_idx],
            quat_wxyz=context.root_quat[frame_idx],
        )
        return MocapFrame(
            source="bvh_g1",
            host_time_s=time.time(),
            frame_index=int(stream_frame_idx),
            fps=float(context.playback_fps),
            joints={"root": root_pose},
            full_body=full_body,
            metadata={
                "format": "bvh_g1",
                "runtime_mode": "online",
                "path": context.path,
                "motion_name": context.motion_name,
                "source_frame_index": source_frame_idx,
                "source_fps": context.source_fps,
            },
        )


def load_bvh_g1_motion(
    bvh_file: str,
    target_fps: float | None = None,
    unit_scale: float = 0.01,
    y_up_to_z_up: bool = True,
    local_root: bool = False,
    align_root: bool = True,
    retarget_config: BvhG1RetargetConfig | None = None,
) -> RobotPklMotion:
    context = prepare_bvh_g1_retarget_context(
        bvh_file=bvh_file,
        target_fps=target_fps,
        unit_scale=unit_scale,
        y_up_to_z_up=y_up_to_z_up,
        local_root=local_root,
        align_root=align_root,
        retarget_config=retarget_config,
    )
    cfg = context.config
    bvh_motion = context.bvh_motion
    frame_indices = context.frame_indices
    root_pos = context.root_pos
    root_quat = context.root_quat

    method = str(cfg.method or "skeleton").strip().lower()
    if method == "skeleton":
        joint_pos = _retarget_bvh_world_rotations_to_g1_prepared(context)
    elif method == "heuristic":
        joint_pos = np.zeros((frame_indices.shape[0], 29), dtype=np.float32)
        for out_idx, frame_idx in enumerate(frame_indices):
            full_body = _build_full_body_reference(bvh_motion, int(frame_idx))
            joint_pos[out_idx] = bvh_smpl_pose_to_g1_joint_pos(full_body.smpl_pose, cfg)
    else:
        raise ValueError(
            f"Unsupported bvh_g1 retarget method {cfg.method!r}; expected skeleton or heuristic"
        )
    joint_pos = _stabilize_joint_positions(
        joint_pos,
        fps=bvh_motion.playback_fps,
        max_joint_velocity_radps=cfg.max_joint_velocity_radps,
        max_joint_step_rad=cfg.max_joint_step_rad,
        alpha=cfg.joint_filter_alpha,
    )
    joint_vel = _finite_difference(
        joint_pos,
        bvh_motion.playback_fps,
        max_abs_velocity=cfg.max_joint_velocity_radps,
    )
    body_pos = None
    if cfg.enable_body_fk:
        body_pos = context.fk.compute_body_pos14_world(
            joint_pos,
            root_pos,
            root_quat,
        )
    return RobotPklMotion(
        path=osp.abspath(bvh_file),
        motion_name=osp.splitext(osp.basename(bvh_file))[0],
        joint_pos_isaaclab=joint_pos,
        joint_vel_isaaclab=joint_vel,
        root_pos_w=root_pos,
        root_quat_wxyz=root_quat,
        source_fps=float(bvh_motion.source_fps),
        playback_fps=float(bvh_motion.playback_fps),
        body_pos=body_pos,
    )


def prepare_bvh_g1_retarget_context(
    bvh_file: str,
    target_fps: float | None = None,
    unit_scale: float = 0.01,
    y_up_to_z_up: bool = True,
    local_root: bool = False,
    align_root: bool = True,
    retarget_config: BvhG1RetargetConfig | None = None,
) -> BvhG1RetargetContext:
    cfg = retarget_config or BvhG1RetargetConfig()
    bvh_motion = load_bvh_motion(
        bvh_file,
        target_fps=target_fps,
        unit_scale=unit_scale,
        y_up_to_z_up=y_up_to_z_up,
        body_local=local_root,
        lower_body_retarget_scale=0.0,
    )
    return prepare_bvh_g1_retarget_context_from_motion(
        bvh_motion,
        align_root=align_root,
        retarget_config=cfg,
    )


def prepare_bvh_g1_retarget_context_from_motion(
    bvh_motion,
    align_root: bool = True,
    retarget_config: BvhG1RetargetConfig | None = None,
) -> BvhG1RetargetContext:
    """Prepare reusable BVH->G1 state from an already decoded BVH motion.

    This is used both by file playback and by streaming receivers that mutate a
    single-frame motion object as new UDP packets arrive.
    """
    cfg = retarget_config or BvhG1RetargetConfig()

    frame_indices = np.arange(0, bvh_motion.frame_count, bvh_motion.frame_stride, dtype=np.int64)
    root_pos = np.zeros((frame_indices.shape[0], 3), dtype=np.float32)
    root_quat = np.zeros((frame_indices.shape[0], 4), dtype=np.float32)

    root_source_idx = bvh_motion.smpl_source_indices[0]
    root0_pos = None
    root0_inv = None
    for out_idx, frame_idx in enumerate(frame_indices):
        if root_source_idx is not None:
            raw_pos = bvh_motion.world_positions[frame_idx, root_source_idx]
            raw_quat = bvh_motion.world_quat_wxyz[frame_idx, root_source_idx]
        else:
            raw_pos = G1_DEFAULT_ROOT_POS_W
            raw_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

        if align_root:
            if root0_pos is None:
                root0_pos = np.asarray(raw_pos, dtype=np.float32).copy()
                root0_inv = _rotation_from_wxyz(raw_quat).inv()
            root_pos[out_idx], root_quat[out_idx] = _aligned_bvh_root_pose(
                raw_pos,
                raw_quat,
                root0_pos=root0_pos,
                root0_inv=root0_inv,
            )
        else:
            root_pos[out_idx] = np.asarray(raw_pos, dtype=np.float32)
            root_quat[out_idx] = normalize_quat_wxyz(raw_quat)

    if cfg.min_root_height_m > 0.0:
        root_pos[:, 2] = np.maximum(root_pos[:, 2], float(cfg.min_root_height_m))
    root_quat = _stabilize_root_quats(
        root_quat,
        fps=bvh_motion.playback_fps,
        mode=cfg.root_mode,
        max_angular_velocity_radps=cfg.max_root_angular_velocity_radps,
        max_tilt_rad=cfg.root_tilt_limit_rad,
    )

    fk = G1BodyFk()
    source_indices = _resolve_skeleton_source_indices(bvh_motion.joint_names)
    method = str(cfg.method or "skeleton").strip().lower()
    missing_required = [
        key
        for key in (
            "left_upper_leg",
            "left_lower_leg",
            "right_upper_leg",
            "right_lower_leg",
            "chest",
        )
        if key not in source_indices
    ]
    if method == "skeleton" and missing_required:
        raise ValueError(
            "BVH skeleton retarget is missing required source joints "
            f"{missing_required}; available joints include {bvh_motion.joint_names[:30]}"
        )

    default_mujoco = G1_DEFAULT_JOINT_POS_ISAACLAB[G1_MUJOCO_TO_ISAACLAB_IDX]
    default_body_pos, default_body_rots = fk._compute_world_frame(
        default_mujoco,
        G1_DEFAULT_ROOT_POS_W,
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
    )
    default_segment_dirs = _g1_default_segment_directions(fk, default_body_pos)
    enabled_segment_keys = _enabled_segment_direction_keys(cfg)
    source0_rots = _aligned_source_rotations(
        bvh_motion,
        int(frame_indices[0]),
        source_indices,
        align_root=align_root,
        root0_inv=root0_inv,
    )
    return BvhG1RetargetContext(
        bvh_motion=bvh_motion,
        frame_indices=frame_indices,
        root_pos=root_pos,
        root_quat=root_quat,
        source_indices=source_indices,
        fk=fk,
        default_body_rots=default_body_rots,
        default_segment_dirs=default_segment_dirs,
        enabled_segment_keys=enabled_segment_keys,
        source0_rots=source0_rots,
        root0_pos=root0_pos,
        root0_inv=root0_inv,
        config=cfg,
    )


def update_bvh_g1_retarget_context_frame(
    context: BvhG1RetargetContext,
    world_positions: np.ndarray,
    world_quat_wxyz: np.ndarray,
    *,
    source_fps: float | None = None,
    playback_fps: float | None = None,
) -> None:
    """Update a single-frame retarget context with a newly received BVH frame."""
    positions = np.asarray(world_positions, dtype=np.float32)
    quats = np.asarray(world_quat_wxyz, dtype=np.float32)
    if positions.shape != context.bvh_motion.world_positions[0].shape:
        raise ValueError(
            f"stream world_positions shape {positions.shape} does not match "
            f"{context.bvh_motion.world_positions[0].shape}"
        )
    if quats.shape != context.bvh_motion.world_quat_wxyz[0].shape:
        raise ValueError(
            f"stream world_quat_wxyz shape {quats.shape} does not match "
            f"{context.bvh_motion.world_quat_wxyz[0].shape}"
        )

    context.bvh_motion.world_positions[0] = positions
    context.bvh_motion.world_quat_wxyz[0] = quats
    if source_fps is not None and source_fps > 0.0:
        context.bvh_motion.source_fps = float(source_fps)
        context.bvh_motion.source_frame_time_s = 1.0 / float(source_fps)
    if playback_fps is not None and playback_fps > 0.0:
        context.bvh_motion.playback_fps = float(playback_fps)

    root_source_idx = context.bvh_motion.smpl_source_indices[0]
    if root_source_idx is not None:
        raw_pos = positions[root_source_idx]
        raw_quat = quats[root_source_idx]
    else:
        raw_pos = G1_DEFAULT_ROOT_POS_W
        raw_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

    if context.root0_inv is not None and context.root0_pos is not None:
        context.root_pos[0], context.root_quat[0] = _aligned_bvh_root_pose(
            raw_pos,
            raw_quat,
            root0_pos=context.root0_pos,
            root0_inv=context.root0_inv,
        )
    else:
        context.root_pos[0] = np.asarray(raw_pos, dtype=np.float32)
        context.root_quat[0] = normalize_quat_wxyz(raw_quat)

    if context.config.min_root_height_m > 0.0:
        context.root_pos[0, 2] = max(
            float(context.root_pos[0, 2]),
            float(context.config.min_root_height_m),
        )
    context.root_quat[:] = _stabilize_root_quats(
        context.root_quat,
        fps=context.bvh_motion.playback_fps,
        mode=context.config.root_mode,
        max_angular_velocity_radps=context.config.max_root_angular_velocity_radps,
        max_tilt_rad=context.config.root_tilt_limit_rad,
    )


def _aligned_bvh_root_pose(
    raw_pos: np.ndarray,
    raw_quat_wxyz: np.ndarray,
    *,
    root0_pos: np.ndarray,
    root0_inv: Rotation,
) -> tuple[np.ndarray, np.ndarray]:
    aligned_pos = root0_pos + root0_inv.apply(np.asarray(raw_pos, dtype=np.float32) - root0_pos)
    aligned_rot = root0_inv * _rotation_from_wxyz(raw_quat_wxyz)
    aligned_quat = normalize_quat_wxyz(aligned_rot.as_quat()[[3, 0, 1, 2]].astype(np.float32))
    return aligned_pos.astype(np.float32), aligned_quat


def bvh_smpl_pose_to_g1_joint_pos(
    smpl_pose: np.ndarray,
    config: BvhG1RetargetConfig | None = None,
) -> np.ndarray:
    cfg = config or BvhG1RetargetConfig()
    scale = max(0.0, float(cfg.retarget_scale))
    lower_scale = scale * max(0.0, float(cfg.lower_body_scale))
    upper_scale = scale * max(0.0, float(cfg.upper_body_scale))
    wrist_scale = scale * max(0.0, float(cfg.wrist_scale))
    waist_scale = scale * max(0.0, float(cfg.waist_scale))

    pose = np.asarray(smpl_pose, dtype=np.float32).reshape(21, 3)
    joint_pos = G1_DEFAULT_JOINT_POS_ISAACLAB.copy()
    wrist_projection = smpl_pose_to_g1_wrist_joint_pos(pose)
    joint_pos[G1_WRIST_JOINT_IDX_ISAACLAB] = (
        G1_DEFAULT_JOINT_POS_ISAACLAB[G1_WRIST_JOINT_IDX_ISAACLAB]
        + wrist_scale
        * (
            wrist_projection[G1_WRIST_JOINT_IDX_ISAACLAB]
            - G1_DEFAULT_JOINT_POS_ISAACLAB[G1_WRIST_JOINT_IDX_ISAACLAB]
        )
    )

    _apply_lower_body(joint_pos, pose, lower_scale)
    _apply_waist(joint_pos, pose, waist_scale)
    _apply_upper_body(joint_pos, pose, upper_scale)

    return _clip_joint_limits(joint_pos, margin_rad=cfg.joint_limit_margin_rad).astype(
        np.float32
    )


def _retarget_bvh_world_rotations_to_g1(
    bvh_motion,
    frame_indices: np.ndarray,
    root_quat_wxyz: np.ndarray,
    align_root: bool,
    root0_inv: Rotation | None,
    config: BvhG1RetargetConfig,
) -> np.ndarray:
    """Legacy wrapper kept for focused tests; production uses prepared context."""
    fk = G1BodyFk()
    source_indices = _resolve_skeleton_source_indices(bvh_motion.joint_names)
    missing_required = [
        key
        for key in (
            "left_upper_leg",
            "left_lower_leg",
            "right_upper_leg",
            "right_lower_leg",
            "chest",
        )
        if key not in source_indices
    ]
    if missing_required:
        raise ValueError(
            "BVH skeleton retarget is missing required source joints "
            f"{missing_required}; available joints include {bvh_motion.joint_names[:30]}"
        )

    default_mujoco = G1_DEFAULT_JOINT_POS_ISAACLAB[G1_MUJOCO_TO_ISAACLAB_IDX]
    default_body_pos, default_body_rots = fk._compute_world_frame(
        default_mujoco,
        G1_DEFAULT_ROOT_POS_W,
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
    )
    default_segment_dirs = _g1_default_segment_directions(fk, default_body_pos)
    enabled_segment_keys = _enabled_segment_direction_keys(config)
    source0_rots = _aligned_source_rotations(
        bvh_motion,
        int(frame_indices[0]),
        source_indices,
        align_root=align_root,
        root0_inv=root0_inv,
    )

    joint_pos = np.zeros((frame_indices.shape[0], 29), dtype=np.float32)
    for out_idx, frame_idx in enumerate(frame_indices):
        joint_pos[out_idx] = _retarget_bvh_g1_frame_impl(
            bvh_motion=bvh_motion,
            frame_idx=int(frame_idx),
            root_quat_wxyz=root_quat_wxyz[out_idx],
            source_indices=source_indices,
            fk=fk,
            default_body_rots=default_body_rots,
            default_segment_dirs=default_segment_dirs,
            enabled_segment_keys=enabled_segment_keys,
            source0_rots=source0_rots,
            align_root=align_root,
            root0_inv=root0_inv,
            config=config,
        )
    return joint_pos


def _retarget_bvh_world_rotations_to_g1_prepared(
    context: BvhG1RetargetContext,
) -> np.ndarray:
    joint_pos = np.zeros((context.frame_count, 29), dtype=np.float32)
    for out_idx in range(context.frame_count):
        joint_pos[out_idx] = retarget_bvh_g1_frame(context, out_idx)
    return joint_pos


def retarget_bvh_g1_frame(
    context: BvhG1RetargetContext,
    output_frame_idx: int,
) -> np.ndarray:
    frame_idx = int(context.frame_indices[output_frame_idx])
    return _retarget_bvh_g1_frame_impl(
        bvh_motion=context.bvh_motion,
        frame_idx=frame_idx,
        root_quat_wxyz=context.root_quat[output_frame_idx],
        source_indices=context.source_indices,
        fk=context.fk,
        default_body_rots=context.default_body_rots,
        default_segment_dirs=context.default_segment_dirs,
        enabled_segment_keys=context.enabled_segment_keys,
        source0_rots=context.source0_rots,
        align_root=context.root0_inv is not None,
        root0_inv=context.root0_inv,
        config=context.config,
    )


def _retarget_bvh_g1_frame_impl(
    *,
    bvh_motion,
    frame_idx: int,
    root_quat_wxyz: np.ndarray,
    source_indices: dict[str, int],
    fk: G1BodyFk,
    default_body_rots: list[Rotation],
    default_segment_dirs: dict[str, np.ndarray],
    enabled_segment_keys: frozenset[str],
    source0_rots: dict[str, Rotation],
    align_root: bool,
    root0_inv: Rotation | None,
    config: BvhG1RetargetConfig,
) -> np.ndarray:
    source_rots = _aligned_source_rotations(
        bvh_motion,
        int(frame_idx),
        source_indices,
        align_root=align_root,
        root0_inv=root0_inv,
    )
    source_segment_dirs = _source_segment_directions(
        bvh_motion,
        int(frame_idx),
        source_indices,
        enabled_segment_keys,
        config,
    )
    segment_swings = _source_segment_swings(
        source_segment_dirs,
        default_segment_dirs,
        _rotation_from_wxyz(root_quat_wxyz),
    )
    target_body_rots = list(default_body_rots)
    for body_idx, node in enumerate(fk.nodes):
        segment_key = G1_BODY_TO_BVH_SEGMENT_KEY.get(node.name)
        if segment_key in segment_swings:
            target_body_rots[body_idx] = (
                segment_swings[segment_key] * default_body_rots[body_idx]
            )
            continue
        source_key = G1_BODY_TO_BVH_SOURCE_KEY.get(node.name)
        if source_key is None or source_key not in source_rots:
            continue
        source_delta = source_rots[source_key] * source0_rots[source_key].inv()
        target_body_rots[body_idx] = source_delta * default_body_rots[body_idx]

    frame_joint_pos = _project_g1_body_rotations_to_joint_pos(
        fk,
        target_body_rots,
        root_quat_wxyz,
        config,
    )
    _apply_elbow_bend_from_segments(
        frame_joint_pos,
        source_segment_dirs,
        default_segment_dirs,
        config,
    )

    ik_mode = str(config.ik_mode or "numeric").strip().lower()
    if ik_mode in {"numeric", "least_squares", "lsq"}:
        _refine_lower_body_direction_ik(frame_joint_pos, source_segment_dirs, fk, config)
        _refine_upper_body_direction_ik(frame_joint_pos, source_segment_dirs, fk, config)
    elif ik_mode in {"analytic"}:
        _apply_knee_bend_from_segments(
            frame_joint_pos,
            source_segment_dirs,
            default_segment_dirs,
            config,
        )
    elif ik_mode in {"fast", "hybrid"}:
        _apply_knee_bend_from_segments(
            frame_joint_pos,
            source_segment_dirs,
            default_segment_dirs,
            config,
        )
        _refine_lower_body_direction_ik_fast(frame_joint_pos, source_segment_dirs, fk, config)
        _refine_upper_body_direction_ik_fast(frame_joint_pos, source_segment_dirs, fk, config)
    elif ik_mode in {"off", "none", "0", "false", "no"}:
        pass
    else:
        raise ValueError(
            f"Unsupported bvh_g1 ik_mode {config.ik_mode!r}; expected numeric, fast, analytic, or off"
        )
    return frame_joint_pos


def _resolve_skeleton_source_indices(joint_names: list[str]) -> dict[str, int]:
    source_indices: dict[str, int] = {}
    for key, aliases in BVH_G1_SOURCE_ALIASES.items():
        idx = _find_first_joint_index(joint_names, aliases)
        if idx is not None:
            source_indices[key] = idx
    return source_indices


def _aligned_source_rotations(
    bvh_motion,
    frame_idx: int,
    source_indices: dict[str, int],
    align_root: bool,
    root0_inv: Rotation | None,
) -> dict[str, Rotation]:
    align_rot = root0_inv if align_root and root0_inv is not None else Rotation.identity()
    return {
        key: align_rot * _rotation_from_wxyz(bvh_motion.world_quat_wxyz[frame_idx, source_idx])
        for key, source_idx in source_indices.items()
    }


def _enabled_segment_direction_keys(config: BvhG1RetargetConfig) -> frozenset[str]:
    mode = str(config.skeleton_segment_direction or "off").strip().lower()
    if mode in {"0", "false", "no", "none", "off"}:
        return frozenset()
    if mode == "upper":
        return UPPER_SEGMENT_KEYS
    if mode == "all":
        return ALL_SEGMENT_KEYS
    raise ValueError(
        "Unsupported bvh_g1 skeleton_segment_direction "
        f"{config.skeleton_segment_direction!r}; expected off, upper, or all"
    )


def _g1_default_segment_directions(
    fk: G1BodyFk,
    default_body_pos: np.ndarray,
) -> dict[str, np.ndarray]:
    out: dict[str, np.ndarray] = {}
    for segment_key, (start_body, end_body) in G1_SEGMENT_BODY_POINTS.items():
        start_idx = fk.body_name_to_idx.get(start_body)
        end_idx = fk.body_name_to_idx.get(end_body)
        if start_idx is None or end_idx is None:
            continue
        direction = _unit_vector(default_body_pos[end_idx] - default_body_pos[start_idx])
        if direction is not None:
            out[segment_key] = direction
    return out


def _source_segment_directions(
    bvh_motion,
    frame_idx: int,
    source_indices: dict[str, int],
    enabled_segment_keys: frozenset[str],
    config: BvhG1RetargetConfig,
) -> dict[str, np.ndarray]:
    if not enabled_segment_keys or "root" not in source_indices:
        return {}

    root_idx = source_indices["root"]
    source_root_inv = _rotation_from_wxyz(bvh_motion.world_quat_wxyz[frame_idx, root_idx]).inv()
    out: dict[str, np.ndarray] = {}
    for segment_key in enabled_segment_keys:
        source_points = BVH_G1_SEGMENT_SOURCE_POINTS.get(segment_key)
        if source_points is None:
            continue
        start_key, end_key = source_points
        if start_key not in source_indices or end_key not in source_indices:
            continue
        start_pos = bvh_motion.world_positions[frame_idx, source_indices[start_key]]
        end_pos = bvh_motion.world_positions[frame_idx, source_indices[end_key]]
        source_dir = _unit_vector(source_root_inv.apply(end_pos - start_pos))
        if source_dir is None:
            continue
        out[segment_key] = _map_bvh_direction_to_g1(source_dir, config)
    return out


def _map_bvh_direction_to_g1(
    direction: np.ndarray,
    config: BvhG1RetargetConfig,
) -> np.ndarray:
    mode = str(config.skeleton_axis_mapping or "identity").strip().lower()
    value = np.asarray(direction, dtype=np.float64)
    if mode in {"identity", "none", "off"}:
        mapped = value
    elif mode in {"bvh_y_forward", "neg_y_forward", "raynos"}:
        # RAYNOS-style BVH after Y-up -> Z-up conversion uses -Y as the
        # character forward axis, while G1/MuJoCo uses +X as forward.
        mapped = np.array([-value[1], value[0], value[2]], dtype=np.float64)
    else:
        raise ValueError(
            "Unsupported bvh_g1 skeleton_axis_mapping "
            f"{config.skeleton_axis_mapping!r}; expected identity or bvh_y_forward"
        )
    unit = _unit_vector(mapped)
    return unit if unit is not None else np.array([0.0, 0.0, -1.0], dtype=np.float64)


def _source_segment_swings(
    source_segment_dirs: dict[str, np.ndarray],
    default_segment_dirs: dict[str, np.ndarray],
    root_rot: Rotation,
) -> dict[str, Rotation]:
    out: dict[str, Rotation] = {}
    for segment_key, source_dir in source_segment_dirs.items():
        default_dir = default_segment_dirs.get(segment_key)
        if default_dir is None:
            continue
        out[segment_key] = root_rot * _rotation_between_vectors(default_dir, source_dir)
    return out


def _unit_vector(vector: np.ndarray) -> np.ndarray | None:
    value = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(value))
    if norm < 1e-8 or not np.isfinite(norm):
        return None
    return (value / norm).astype(np.float64)


def _rotation_between_vectors(from_vector: np.ndarray, to_vector: np.ndarray) -> Rotation:
    source = _unit_vector(from_vector)
    target = _unit_vector(to_vector)
    if source is None or target is None:
        return Rotation.identity()

    dot = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if dot > 1.0 - 1e-8:
        return Rotation.identity()
    if dot < -1.0 + 1e-8:
        axis = np.cross(source, np.array([1.0, 0.0, 0.0], dtype=np.float64))
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < 1e-8:
            axis = np.cross(source, np.array([0.0, 1.0, 0.0], dtype=np.float64))
            axis_norm = float(np.linalg.norm(axis))
        return Rotation.from_rotvec(axis / max(axis_norm, 1e-8) * np.pi)

    axis = np.cross(source, target)
    axis_norm = float(np.linalg.norm(axis))
    angle = float(np.arctan2(axis_norm, dot))
    return Rotation.from_rotvec(axis / max(axis_norm, 1e-8) * angle)


def _apply_elbow_bend_from_segments(
    joint_pos_isaaclab: np.ndarray,
    source_segment_dirs: dict[str, np.ndarray],
    default_segment_dirs: dict[str, np.ndarray],
    config: BvhG1RetargetConfig,
) -> None:
    for side, mujoco_idx in (("left", 18), ("right", 25)):
        upper_key = f"{side}_upper_arm"
        forearm_key = f"{side}_forearm"
        source_upper = source_segment_dirs.get(upper_key)
        source_forearm = source_segment_dirs.get(forearm_key)
        default_upper = default_segment_dirs.get(upper_key)
        default_forearm = default_segment_dirs.get(forearm_key)
        if (
            source_upper is None
            or source_forearm is None
            or default_upper is None
            or default_forearm is None
        ):
            continue

        source_bend = _angle_between_unit_vectors(source_upper, source_forearm)
        default_bend = _angle_between_unit_vectors(default_upper, default_forearm)
        default_angle = _default_mj(mujoco_idx)
        scale = max(0.0, float(config.retarget_scale)) * max(
            0.0,
            float(config.upper_body_scale),
        )
        target = default_angle + scale * (default_bend - source_bend)

        delta_scale = max(0.0, float(config.joint_delta_limit_scale))
        if delta_scale > 0.0:
            delta_limit = float(G1_JOINT_DEFAULT_DELTA_LIMIT_MUJOCO[mujoco_idx]) * delta_scale
            target = float(
                np.clip(
                    target,
                    default_angle - delta_limit,
                    default_angle + delta_limit,
                )
            )

        margin = max(0.0, float(config.joint_limit_margin_rad))
        lower = float(G1_JOINT_LOWER_LIMIT_MUJOCO[mujoco_idx] + margin)
        upper = float(G1_JOINT_UPPER_LIMIT_MUJOCO[mujoco_idx] - margin)
        if lower < upper:
            target = float(np.clip(target, lower, upper))
        _set_mj(joint_pos_isaaclab, mujoco_idx, target)


def _apply_knee_bend_from_segments(
    joint_pos_isaaclab: np.ndarray,
    source_segment_dirs: dict[str, np.ndarray],
    default_segment_dirs: dict[str, np.ndarray],
    config: BvhG1RetargetConfig,
) -> None:
    scale = max(0.0, float(config.retarget_scale)) * max(
        0.0,
        float(config.lower_body_scale),
    )
    if scale <= 0.0:
        return

    for side, mujoco_idx in (("left", 3), ("right", 9)):
        thigh_key = f"{side}_thigh"
        shin_key = f"{side}_shin"
        source_thigh = source_segment_dirs.get(thigh_key)
        source_shin = source_segment_dirs.get(shin_key)
        default_thigh = default_segment_dirs.get(thigh_key)
        default_shin = default_segment_dirs.get(shin_key)
        if (
            source_thigh is None
            or source_shin is None
            or default_thigh is None
            or default_shin is None
        ):
            continue

        source_bend = _angle_between_unit_vectors(source_thigh, source_shin)
        default_bend = _angle_between_unit_vectors(default_thigh, default_shin)
        target = _default_mj(mujoco_idx) + scale * (source_bend - default_bend)
        target = _limit_mujoco_joint_target(target, mujoco_idx, config)
        _set_mj(joint_pos_isaaclab, mujoco_idx, target)


def _limit_mujoco_joint_target(
    target: float,
    mujoco_idx: int,
    config: BvhG1RetargetConfig,
) -> float:
    default_angle = _default_mj(mujoco_idx)
    delta_scale = max(0.0, float(config.joint_delta_limit_scale))
    if delta_scale > 0.0:
        delta_limit = float(G1_JOINT_DEFAULT_DELTA_LIMIT_MUJOCO[mujoco_idx]) * delta_scale
        target = float(
            np.clip(
                target,
                default_angle - delta_limit,
                default_angle + delta_limit,
            )
        )

    margin = max(0.0, float(config.joint_limit_margin_rad))
    lower = float(G1_JOINT_LOWER_LIMIT_MUJOCO[mujoco_idx] + margin)
    upper = float(G1_JOINT_UPPER_LIMIT_MUJOCO[mujoco_idx] - margin)
    if lower < upper:
        target = float(np.clip(target, lower, upper))
    return target


def _angle_between_unit_vectors(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.arccos(np.clip(np.dot(a, b), -1.0, 1.0)))


def _refine_lower_body_direction_ik(
    joint_pos_isaaclab: np.ndarray,
    source_segment_dirs: dict[str, np.ndarray],
    fk: G1BodyFk,
    config: BvhG1RetargetConfig,
) -> None:
    if not source_segment_dirs:
        return

    joint_pos_mujoco = joint_pos_isaaclab[G1_MUJOCO_TO_ISAACLAB_IDX].astype(np.float64).copy()
    for side, joint_indexes, hip_body, knee_body, ankle_body in (
        (
            "left",
            np.array([0, 1, 2, 3], dtype=np.int64),
            "left_hip_roll_link",
            "left_knee_link",
            "left_ankle_roll_link",
        ),
        (
            "right",
            np.array([6, 7, 8, 9], dtype=np.int64),
            "right_hip_roll_link",
            "right_knee_link",
            "right_ankle_roll_link",
        ),
    ):
        target_thigh = source_segment_dirs.get(f"{side}_thigh")
        target_shin = source_segment_dirs.get(f"{side}_shin")
        if target_thigh is None or target_shin is None:
            continue

        lower, upper = _bounded_mujoco_joint_limits(joint_indexes, config)
        if np.any(lower >= upper):
            continue
        x0 = np.clip(joint_pos_mujoco[joint_indexes], lower, upper)
        hip_idx = fk.body_name_to_idx[hip_body]
        knee_idx = fk.body_name_to_idx[knee_body]
        ankle_idx = fk.body_name_to_idx[ankle_body]

        def residual(values: np.ndarray) -> np.ndarray:
            candidate = joint_pos_mujoco.copy()
            candidate[joint_indexes] = values
            body_pos, _ = fk._compute_world_frame(
                candidate.astype(np.float32),
                np.zeros(3, dtype=np.float32),
                np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            )
            thigh_dir = _unit_vector(body_pos[knee_idx] - body_pos[hip_idx])
            shin_dir = _unit_vector(body_pos[ankle_idx] - body_pos[knee_idx])
            if thigh_dir is None or shin_dir is None:
                return np.zeros(10, dtype=np.float64)
            regularizer = 0.08 * (values - x0)
            return np.concatenate(
                (
                    1.35 * (thigh_dir - target_thigh),
                    shin_dir - target_shin,
                    regularizer,
                )
            )

        result = least_squares(
            residual,
            x0,
            bounds=(lower, upper),
            max_nfev=12,
            ftol=1e-3,
            xtol=1e-3,
            gtol=1e-3,
        )
        joint_pos_mujoco[joint_indexes] = result.x

    joint_pos_isaaclab[:] = 0.0
    joint_pos_isaaclab[G1_MUJOCO_TO_ISAACLAB_IDX] = joint_pos_mujoco.astype(np.float32)


def _refine_lower_body_direction_ik_fast(
    joint_pos_isaaclab: np.ndarray,
    source_segment_dirs: dict[str, np.ndarray],
    fk: G1BodyFk,
    config: BvhG1RetargetConfig,
) -> None:
    if not source_segment_dirs:
        return

    specs = (
        (
            "left",
            np.array([0, 1, 2, 3], dtype=np.int64),
            "left_hip_roll_link",
            "left_knee_link",
            "left_ankle_roll_link",
            "thigh",
            "shin",
            0.08,
        ),
        (
            "right",
            np.array([6, 7, 8, 9], dtype=np.int64),
            "right_hip_roll_link",
            "right_knee_link",
            "right_ankle_roll_link",
            "thigh",
            "shin",
            0.08,
        ),
    )
    _refine_direction_ik_fast(joint_pos_isaaclab, source_segment_dirs, fk, config, specs)


def _refine_upper_body_direction_ik(
    joint_pos_isaaclab: np.ndarray,
    source_segment_dirs: dict[str, np.ndarray],
    fk: G1BodyFk,
    config: BvhG1RetargetConfig,
) -> None:
    if not source_segment_dirs:
        return

    joint_pos_mujoco = joint_pos_isaaclab[G1_MUJOCO_TO_ISAACLAB_IDX].astype(np.float64).copy()
    for side, joint_indexes, shoulder_body, elbow_body, wrist_body in (
        (
            "left",
            np.array([15, 16, 17, 18], dtype=np.int64),
            "left_shoulder_roll_link",
            "left_elbow_link",
            "left_wrist_yaw_link",
        ),
        (
            "right",
            np.array([22, 23, 24, 25], dtype=np.int64),
            "right_shoulder_roll_link",
            "right_elbow_link",
            "right_wrist_yaw_link",
        ),
    ):
        target_upper = source_segment_dirs.get(f"{side}_upper_arm")
        target_forearm = source_segment_dirs.get(f"{side}_forearm")
        if target_upper is None or target_forearm is None:
            continue

        lower, upper = _bounded_mujoco_joint_limits(joint_indexes, config)
        if np.any(lower >= upper):
            continue
        x0 = np.clip(joint_pos_mujoco[joint_indexes], lower, upper)
        shoulder_idx = fk.body_name_to_idx[shoulder_body]
        elbow_idx = fk.body_name_to_idx[elbow_body]
        wrist_idx = fk.body_name_to_idx[wrist_body]

        def residual(values: np.ndarray) -> np.ndarray:
            candidate = joint_pos_mujoco.copy()
            candidate[joint_indexes] = values
            body_pos, _ = fk._compute_world_frame(
                candidate.astype(np.float32),
                np.zeros(3, dtype=np.float32),
                np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            )
            upper_dir = _unit_vector(body_pos[elbow_idx] - body_pos[shoulder_idx])
            forearm_dir = _unit_vector(body_pos[wrist_idx] - body_pos[elbow_idx])
            if upper_dir is None or forearm_dir is None:
                return np.zeros(10, dtype=np.float64)
            regularizer = 0.06 * (values - x0)
            return np.concatenate(
                (
                    1.35 * (upper_dir - target_upper),
                    forearm_dir - target_forearm,
                    regularizer,
                )
            )

        result = least_squares(
            residual,
            x0,
            bounds=(lower, upper),
            max_nfev=10,
            ftol=1e-3,
            xtol=1e-3,
            gtol=1e-3,
        )
        joint_pos_mujoco[joint_indexes] = result.x

    joint_pos_isaaclab[:] = 0.0
    joint_pos_isaaclab[G1_MUJOCO_TO_ISAACLAB_IDX] = joint_pos_mujoco.astype(np.float32)


def _refine_upper_body_direction_ik_fast(
    joint_pos_isaaclab: np.ndarray,
    source_segment_dirs: dict[str, np.ndarray],
    fk: G1BodyFk,
    config: BvhG1RetargetConfig,
) -> None:
    if not source_segment_dirs:
        return

    specs = (
        (
            "left",
            np.array([15, 16, 17, 18], dtype=np.int64),
            "left_shoulder_roll_link",
            "left_elbow_link",
            "left_wrist_yaw_link",
            "upper_arm",
            "forearm",
            0.06,
        ),
        (
            "right",
            np.array([22, 23, 24, 25], dtype=np.int64),
            "right_shoulder_roll_link",
            "right_elbow_link",
            "right_wrist_yaw_link",
            "upper_arm",
            "forearm",
            0.06,
        ),
    )
    _refine_direction_ik_fast(joint_pos_isaaclab, source_segment_dirs, fk, config, specs)


def _refine_direction_ik_fast(
    joint_pos_isaaclab: np.ndarray,
    source_segment_dirs: dict[str, np.ndarray],
    fk: G1BodyFk,
    config: BvhG1RetargetConfig,
    specs: tuple[tuple[str, np.ndarray, str, str, str, str, str, float], ...],
) -> None:
    joint_pos_mujoco = joint_pos_isaaclab[G1_MUJOCO_TO_ISAACLAB_IDX].astype(np.float64).copy()
    root_pos = np.zeros(3, dtype=np.float32)
    root_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

    for (
        side,
        joint_indexes,
        start_body,
        mid_body,
        end_body,
        first_segment,
        second_segment,
        regularization,
    ) in specs:
        target_first = source_segment_dirs.get(f"{side}_{first_segment}")
        target_second = source_segment_dirs.get(f"{side}_{second_segment}")
        if target_first is None or target_second is None:
            continue

        lower, upper = _bounded_mujoco_joint_limits(joint_indexes, config)
        if np.any(lower >= upper):
            continue
        x0 = np.clip(joint_pos_mujoco[joint_indexes], lower, upper)
        start_idx = fk.body_name_to_idx[start_body]
        mid_idx = fk.body_name_to_idx[mid_body]
        end_idx = fk.body_name_to_idx[end_body]

        def residual(values: np.ndarray) -> np.ndarray:
            candidate = joint_pos_mujoco.copy()
            candidate[joint_indexes] = values
            body_pos, _ = fk._compute_world_frame(
                candidate.astype(np.float32),
                root_pos,
                root_quat,
            )
            first_dir = _unit_vector(body_pos[mid_idx] - body_pos[start_idx])
            second_dir = _unit_vector(body_pos[end_idx] - body_pos[mid_idx])
            if first_dir is None or second_dir is None:
                return np.zeros(10, dtype=np.float64)
            return np.concatenate(
                (
                    1.35 * (first_dir - target_first),
                    second_dir - target_second,
                    regularization * (values - x0),
                )
            )

        values = x0.copy()
        base_residual = residual(values)
        jacobian = np.zeros((base_residual.shape[0], values.shape[0]), dtype=np.float64)
        eps = 1e-3
        for col in range(values.shape[0]):
            perturbed = values.copy()
            perturbed[col] = float(np.clip(perturbed[col] + eps, lower[col], upper[col]))
            delta = perturbed[col] - values[col]
            if abs(delta) < 1e-8:
                continue
            jacobian[:, col] = (residual(perturbed) - base_residual) / delta

        lhs = jacobian.T @ jacobian + 1e-3 * np.eye(values.shape[0], dtype=np.float64)
        rhs = -(jacobian.T @ base_residual)
        try:
            step = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            continue

        step = np.clip(step, -0.08, 0.08)
        best_values = values
        best_cost = float(np.dot(base_residual, base_residual))
        for step_scale in (1.0, 0.5, 0.25, 0.125):
            candidate_values = np.clip(values + step * step_scale, lower, upper)
            candidate_residual = residual(candidate_values)
            candidate_cost = float(np.dot(candidate_residual, candidate_residual))
            if candidate_cost < best_cost:
                best_cost = candidate_cost
                best_values = candidate_values

        joint_pos_mujoco[joint_indexes] = best_values

    joint_pos_isaaclab[:] = 0.0
    joint_pos_isaaclab[G1_MUJOCO_TO_ISAACLAB_IDX] = joint_pos_mujoco.astype(np.float32)


def _bounded_mujoco_joint_limits(
    mujoco_joint_indexes: np.ndarray,
    config: BvhG1RetargetConfig,
) -> tuple[np.ndarray, np.ndarray]:
    indexes = np.asarray(mujoco_joint_indexes, dtype=np.int64)
    margin = max(0.0, float(config.joint_limit_margin_rad))
    lower = G1_JOINT_LOWER_LIMIT_MUJOCO[indexes].astype(np.float64) + margin
    upper = G1_JOINT_UPPER_LIMIT_MUJOCO[indexes].astype(np.float64) - margin
    delta_scale = max(0.0, float(config.joint_delta_limit_scale))
    if delta_scale > 0.0:
        default_mujoco = G1_DEFAULT_JOINT_POS_ISAACLAB[G1_MUJOCO_TO_ISAACLAB_IDX]
        delta = G1_JOINT_DEFAULT_DELTA_LIMIT_MUJOCO[indexes].astype(np.float64) * delta_scale
        lower = np.maximum(lower, default_mujoco[indexes].astype(np.float64) - delta)
        upper = np.minimum(upper, default_mujoco[indexes].astype(np.float64) + delta)
    return lower, upper


def _project_g1_body_rotations_to_joint_pos(
    fk: G1BodyFk,
    target_body_rots: list[Rotation],
    root_quat_wxyz: np.ndarray,
    config: BvhG1RetargetConfig,
) -> np.ndarray:
    default_mujoco = G1_DEFAULT_JOINT_POS_ISAACLAB[G1_MUJOCO_TO_ISAACLAB_IDX]
    joint_pos_mujoco = default_mujoco.copy()
    body_rot_w: list[Rotation] = [Rotation.identity() for _ in fk.nodes]
    body_rot_w[0] = _rotation_from_wxyz(root_quat_wxyz)

    for body_idx, node in enumerate(fk.nodes[1:], start=1):
        parent_rot = body_rot_w[node.parent]
        local_base = Rotation.from_quat(node.local_quat_wxyz[[1, 2, 3, 0]])
        joint_axis = node.joint_axis
        angle = 0.0
        if node.joint_motor_idx is not None and joint_axis is not None:
            local_target = parent_rot.inv() * target_body_rots[body_idx]
            joint_delta = local_base.inv() * local_target
            raw_angle = _signed_twist_angle(joint_delta, joint_axis)
            default_angle = float(default_mujoco[node.joint_motor_idx])
            sign = (
                float(G1_SKELETON_JOINT_SIGN_MUJOCO[node.joint_motor_idx])
                if config.skeleton_sign_correction
                else 1.0
            )
            raw_angle = default_angle + sign * (raw_angle - default_angle)
            scale = _joint_scale(node.joint_motor_idx, config)
            angle = default_angle + scale * (raw_angle - default_angle)
            joint_pos_mujoco[node.joint_motor_idx] = angle
        axis = np.asarray(joint_axis if joint_axis is not None else [0.0, 0.0, 1.0])
        body_rot_w[body_idx] = parent_rot * local_base * Rotation.from_rotvec(axis * float(angle))

    delta_scale = max(0.0, float(config.joint_delta_limit_scale))
    if delta_scale > 0.0:
        delta_limit = G1_JOINT_DEFAULT_DELTA_LIMIT_MUJOCO * delta_scale
        joint_pos_mujoco = np.clip(
            joint_pos_mujoco,
            default_mujoco - delta_limit,
            default_mujoco + delta_limit,
        )
    joint_pos_isaaclab = np.zeros(29, dtype=np.float32)
    joint_pos_isaaclab[G1_MUJOCO_TO_ISAACLAB_IDX] = joint_pos_mujoco
    return _clip_joint_limits(joint_pos_isaaclab, margin_rad=config.joint_limit_margin_rad).astype(
        np.float32
    )


def _signed_twist_angle(rotation: Rotation, axis: np.ndarray) -> float:
    axis = np.asarray(axis, dtype=np.float64)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-8 or not np.isfinite(axis_norm):
        return 0.0
    axis = axis / axis_norm
    quat_xyzw = rotation.as_quat()
    vector = quat_xyzw[:3]
    scalar = float(quat_xyzw[3])
    twist_vector = axis * float(np.dot(vector, axis))
    twist_norm = float(np.linalg.norm(twist_vector))
    if twist_norm < 1e-10:
        return 0.0
    angle = 2.0 * np.arctan2(twist_norm, scalar)
    if np.dot(twist_vector, axis) < 0.0:
        angle = -angle
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def _joint_scale(mujoco_joint_idx: int, config: BvhG1RetargetConfig) -> float:
    scale = max(0.0, float(config.retarget_scale))
    if 0 <= mujoco_joint_idx <= 11:
        return scale * max(0.0, float(config.lower_body_scale))
    if 12 <= mujoco_joint_idx <= 14:
        return scale * max(0.0, float(config.waist_scale))
    if mujoco_joint_idx in {19, 20, 21, 26, 27, 28}:
        return scale * max(0.0, float(config.wrist_scale))
    return scale * max(0.0, float(config.upper_body_scale))


def save_bvh_g1_motion_lib_pkl(motion: RobotPklMotion, output_path: str) -> None:
    dof_mujoco = motion.joint_pos_isaaclab[:, G1_MUJOCO_TO_ISAACLAB_IDX]
    dof_vel_mujoco = motion.joint_vel_isaaclab[:, G1_MUJOCO_TO_ISAACLAB_IDX]
    root_rot_xyzw = motion.root_quat_wxyz[:, [1, 2, 3, 0]]
    entry = {
        "root_trans_offset": motion.root_pos_w.astype(np.float32),
        "pose_aa": np.zeros((motion.frame_count, 30, 3), dtype=np.float32),
        "dof": dof_mujoco.astype(np.float32),
        "dof_vel": dof_vel_mujoco.astype(np.float32),
        "root_rot": root_rot_xyzw.astype(np.float32),
        "smpl_joints": np.zeros((motion.frame_count, 24, 3), dtype=np.float32),
        "fps": float(motion.playback_fps),
    }
    if motion.body_pos is not None:
        entry["body_pos"] = motion.body_pos.astype(np.float32)
    joblib.dump({motion.motion_name: entry}, output_path, compress=True)


def _apply_lower_body(joint_pos: np.ndarray, pose: np.ndarray, scale: float) -> None:
    if scale <= 0.0:
        return
    lhip = pose[0]
    rhip = pose[1]
    lknee = pose[3]
    rknee = pose[4]
    lankle = pose[6]
    rankle = pose[7]

    _set_mj(joint_pos, 0, _default_mj(0) - 0.75 * scale * lhip[0])
    _set_mj(joint_pos, 1, _default_mj(1) + 0.35 * scale * lhip[1])
    _set_mj(joint_pos, 2, _default_mj(2) + 0.35 * scale * lhip[2])
    _set_mj(joint_pos, 3, _default_mj(3) + 0.80 * scale * np.linalg.norm(lknee))
    _set_mj(joint_pos, 4, _default_mj(4) - 0.45 * scale * lankle[0])
    _set_mj(joint_pos, 5, _default_mj(5) + 0.30 * scale * lankle[1])

    _set_mj(joint_pos, 6, _default_mj(6) - 0.75 * scale * rhip[0])
    _set_mj(joint_pos, 7, _default_mj(7) - 0.35 * scale * rhip[1])
    _set_mj(joint_pos, 8, _default_mj(8) - 0.35 * scale * rhip[2])
    _set_mj(joint_pos, 9, _default_mj(9) + 0.80 * scale * np.linalg.norm(rknee))
    _set_mj(joint_pos, 10, _default_mj(10) - 0.45 * scale * rankle[0])
    _set_mj(joint_pos, 11, _default_mj(11) - 0.30 * scale * rankle[1])


def _apply_waist(joint_pos: np.ndarray, pose: np.ndarray, scale: float) -> None:
    if scale <= 0.0:
        return
    spine = pose[2]
    chest = pose[5]
    torso = spine + chest
    _set_mj(joint_pos, 12, _default_mj(12) + 0.45 * scale * torso[2])
    _set_mj(joint_pos, 13, _default_mj(13) + 0.35 * scale * torso[0])
    _set_mj(joint_pos, 14, _default_mj(14) - 0.35 * scale * torso[1])


def _apply_upper_body(joint_pos: np.ndarray, pose: np.ndarray, scale: float) -> None:
    if scale <= 0.0:
        return
    lshoulder = _euler_xyz(pose[12])
    rshoulder = _euler_xyz(pose[13])
    lelbow = pose[17]
    relbow = pose[18]

    _set_mj(joint_pos, 15, _default_mj(15) - 0.85 * scale * lshoulder[1])
    _set_mj(joint_pos, 16, _default_mj(16) + 0.75 * scale * lshoulder[0])
    _set_mj(joint_pos, 17, _default_mj(17) + 0.55 * scale * lshoulder[2])
    _set_mj(joint_pos, 18, _default_mj(18) + 0.35 * scale * np.linalg.norm(lelbow))

    _set_mj(joint_pos, 22, _default_mj(22) + 0.85 * scale * rshoulder[1])
    _set_mj(joint_pos, 23, _default_mj(23) + 0.75 * scale * rshoulder[0])
    _set_mj(joint_pos, 24, _default_mj(24) + 0.55 * scale * rshoulder[2])
    _set_mj(joint_pos, 25, _default_mj(25) + 0.35 * scale * np.linalg.norm(relbow))


def _clip_joint_limits(joint_pos: np.ndarray, margin_rad: float) -> np.ndarray:
    margin = max(0.0, float(margin_rad))
    lower = G1_JOINT_LOWER_LIMIT_ISAACLAB + margin
    upper = G1_JOINT_UPPER_LIMIT_ISAACLAB - margin
    invalid = lower >= upper
    if np.any(invalid):
        lower = lower.copy()
        upper = upper.copy()
        lower[invalid] = G1_JOINT_LOWER_LIMIT_ISAACLAB[invalid]
        upper[invalid] = G1_JOINT_UPPER_LIMIT_ISAACLAB[invalid]
    return np.clip(joint_pos, lower, upper)


def _set_mj(joint_pos_isaaclab: np.ndarray, mujoco_idx: int, value: float) -> None:
    joint_pos_isaaclab[G1_MUJOCO_TO_ISAACLAB_IDX[mujoco_idx]] = float(value)


def _default_mj(mujoco_idx: int) -> float:
    return float(G1_DEFAULT_JOINT_POS_ISAACLAB[G1_MUJOCO_TO_ISAACLAB_IDX[mujoco_idx]])


def _euler_xyz(rotvec: np.ndarray) -> np.ndarray:
    return Rotation.from_rotvec(np.asarray(rotvec, dtype=np.float32)).as_euler(
        "XYZ", degrees=False
    )


def _finite_difference(
    values: np.ndarray,
    fps: float,
    max_abs_velocity: float | None = None,
) -> np.ndarray:
    vel = np.zeros_like(values, dtype=np.float32)
    if values.shape[0] > 1:
        vel[:-1] = (values[1:] - values[:-1]) * float(fps)
        vel[-1] = vel[-2]
    if max_abs_velocity is not None and max_abs_velocity > 0.0:
        vel = np.clip(vel, -float(max_abs_velocity), float(max_abs_velocity))
    return vel


def _stabilize_root_quats(
    root_quat_wxyz: np.ndarray,
    fps: float,
    mode: str,
    max_angular_velocity_radps: float,
    max_tilt_rad: float,
) -> np.ndarray:
    if root_quat_wxyz.shape[0] == 0:
        return root_quat_wxyz.astype(np.float32)

    normalized_mode = str(mode or "yaw").strip().lower()
    if normalized_mode not in {"source", "yaw", "locked"}:
        raise ValueError(
            f"Unsupported bvh_g1 root_mode {mode!r}; expected source, yaw, or locked"
        )

    prepared = np.empty_like(root_quat_wxyz, dtype=np.float32)
    for idx, quat in enumerate(root_quat_wxyz):
        target = normalize_quat_wxyz(quat)
        if normalized_mode == "locked":
            target = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        elif normalized_mode == "yaw":
            target = _yaw_only_quat_wxyz(target)
        else:
            target = _clamp_root_tilt(target, max_tilt_rad)
        prepared[idx] = target

    max_step = float(max_angular_velocity_radps) / max(1.0, float(fps))
    if max_step <= 0.0 or normalized_mode == "locked":
        return prepared

    out = np.empty_like(prepared, dtype=np.float32)
    out[0] = prepared[0]
    for idx in range(1, prepared.shape[0]):
        out[idx] = _limit_quat_step(out[idx - 1], prepared[idx], max_step)
    return out.astype(np.float32)


def _limit_quat_step(
    previous_wxyz: np.ndarray,
    target_wxyz: np.ndarray,
    max_angle_rad: float,
) -> np.ndarray:
    if max_angle_rad <= 0.0:
        return normalize_quat_wxyz(target_wxyz)

    previous = _rotation_from_wxyz(previous_wxyz)
    target = _rotation_from_wxyz(target_wxyz)
    delta = previous.inv() * target
    rotvec = delta.as_rotvec()
    angle = float(np.linalg.norm(rotvec))
    if angle <= max_angle_rad or angle < 1e-8:
        return normalize_quat_wxyz(target_wxyz)

    limited = previous * Rotation.from_rotvec(rotvec * (max_angle_rad / angle))
    quat_xyzw = limited.as_quat()
    return normalize_quat_wxyz(quat_xyzw[[3, 0, 1, 2]].astype(np.float32))


def _clamp_root_tilt(quat_wxyz: np.ndarray, max_tilt_rad: float) -> np.ndarray:
    if max_tilt_rad <= 0.0:
        return normalize_quat_wxyz(quat_wxyz)

    quat = normalize_quat_wxyz(quat_wxyz)
    tilt = _root_tilt_rad(quat)
    if tilt <= max_tilt_rad:
        return quat

    rot = _rotation_from_wxyz(quat)
    yaw_rot = Rotation.from_euler("z", _root_yaw_rad(quat))
    residual = (yaw_rot.inv() * rot).as_rotvec()
    residual_angle = float(np.linalg.norm(residual))
    if residual_angle < 1e-8:
        return quat

    clamped = yaw_rot * Rotation.from_rotvec(
        residual * (float(max_tilt_rad) / residual_angle)
    )
    quat_xyzw = clamped.as_quat()
    return normalize_quat_wxyz(quat_xyzw[[3, 0, 1, 2]].astype(np.float32))


def _yaw_only_quat_wxyz(quat_wxyz: np.ndarray) -> np.ndarray:
    yaw_xyzw = Rotation.from_euler("z", _root_yaw_rad(quat_wxyz)).as_quat()
    return normalize_quat_wxyz(yaw_xyzw[[3, 0, 1, 2]].astype(np.float32))


def _root_yaw_rad(quat_wxyz: np.ndarray) -> float:
    rot = _rotation_from_wxyz(quat_wxyz)
    forward = rot.apply([1.0, 0.0, 0.0])
    return float(np.arctan2(forward[1], forward[0]))


def _root_tilt_rad(quat_wxyz: np.ndarray) -> float:
    quat = normalize_quat_wxyz(quat_wxyz)
    _, qx, qy, _ = quat
    root_z_dot = float(np.clip(1.0 - 2.0 * (qx * qx + qy * qy), -1.0, 1.0))
    return float(np.arccos(root_z_dot))


def _stabilize_joint_positions(
    values: np.ndarray,
    fps: float,
    max_joint_velocity_radps: float,
    max_joint_step_rad: float,
    alpha: float,
) -> np.ndarray:
    if values.shape[0] <= 1:
        return values.astype(np.float32)

    alpha = float(np.clip(alpha, 0.0, 1.0))
    max_step = float(max_joint_step_rad)
    if max_step <= 0.0 and max_joint_velocity_radps > 0.0:
        max_step = float(max_joint_velocity_radps) / max(1.0, float(fps))
    if max_step <= 0.0 and alpha >= 1.0:
        return values.astype(np.float32)

    out = np.empty_like(values, dtype=np.float32)
    out[0] = values[0]
    for idx in range(1, values.shape[0]):
        target = values[idx]
        if alpha < 1.0:
            target = out[idx - 1] + alpha * (target - out[idx - 1])
        delta = target - out[idx - 1]
        if max_step > 0.0:
            delta = np.clip(delta, -max_step, max_step)
        out[idx] = out[idx - 1] + delta
    return out.astype(np.float32)


def _stabilize_joint_position_step(
    target: np.ndarray,
    previous: np.ndarray | None,
    fps: float,
    max_joint_velocity_radps: float,
    max_joint_step_rad: float,
    alpha: float,
) -> np.ndarray:
    target = np.asarray(target, dtype=np.float32).reshape(29)
    if previous is None:
        return target.astype(np.float32)

    previous = np.asarray(previous, dtype=np.float32).reshape(29)
    alpha = float(np.clip(alpha, 0.0, 1.0))
    if alpha < 1.0:
        target = previous + alpha * (target - previous)

    max_step = float(max_joint_step_rad)
    if max_step <= 0.0 and max_joint_velocity_radps > 0.0:
        max_step = float(max_joint_velocity_radps) / max(1.0, float(fps))
    if max_step > 0.0:
        target = previous + np.clip(target - previous, -max_step, max_step)
    return target.astype(np.float32)
