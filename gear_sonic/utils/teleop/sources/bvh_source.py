"""BVH playback source for mocap manager development."""

from __future__ import annotations

import os.path as osp
import re
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from gear_sonic.utils.teleop.sources.base import (
    FullBodyReference,
    G1_DEFAULT_JOINT_POS_ISAACLAB,
    G1_LOWER_BODY_JOINT_IDX_ISAACLAB,
    MocapFrame,
    Pose7D,
    smpl_pose_to_g1_wrist_joint_pos,
)


BVH_DEFAULT_JOINT_ALIASES = {
    "root": ("Hips", "Hip", "Pelvis", "Root", "root"),
    "pelvis": ("Hips", "Hip", "Pelvis", "Root", "root"),
    "spine": (
        "Spine",
        "Spine1",
        "Spine01",
        "torso_2",
        "torso_3",
        "mixamorig:Spine",
    ),
    "chest": (
        "Chest",
        "UpperChest",
        "Spine2",
        "Spine02",
        "Spine3",
        "torso_5",
        "torso_6",
        "torso_7",
        "mixamorig:Spine1",
        "mixamorig:Spine2",
    ),
    "neck": ("Neck", "Neck1", "neck_1", "neck_2", "mixamorig:Neck"),
    "head": ("Head", "HeadTop_End", "HeadEnd", "mixamorig:Head"),
    "left_shoulder": (
        "left_shoulder",
        "left_upper_arm",
        "LeftShoulder",
        "LeftArm",
        "L_Shoulder",
        "L_UpperArm",
        "l_shoulder",
        "l_up_arm",
        "mixamorig:LeftShoulder",
        "mixamorig:LeftArm",
    ),
    "left_elbow": (
        "left_lower_arm",
        "LeftForeArm",
        "LeftLowerArm",
        "LeftElbow",
        "L_ForeArm",
        "L_LowerArm",
        "l_low_arm",
        "mixamorig:LeftForeArm",
    ),
    "left_wrist": ("left_wrist", "LeftHand", "LeftWrist", "L_Hand", "mixamorig:LeftHand"),
    "right_shoulder": (
        "right_shoulder",
        "right_upper_arm",
        "RightShoulder",
        "RightArm",
        "R_Shoulder",
        "R_UpperArm",
        "r_shoulder",
        "r_up_arm",
        "mixamorig:RightShoulder",
        "mixamorig:RightArm",
    ),
    "right_elbow": (
        "right_lower_arm",
        "RightForeArm",
        "RightLowerArm",
        "RightElbow",
        "R_ForeArm",
        "R_LowerArm",
        "r_low_arm",
        "mixamorig:RightForeArm",
    ),
    "right_wrist": ("right_wrist", "RightHand", "RightWrist", "R_Hand", "mixamorig:RightHand"),
    "left_hip": (
        "LeftUpLeg",
        "LeftUpperLeg",
        "LeftThigh",
        "L_UpLeg",
        "L_Hip",
        "l_up_leg",
        "left_upper_leg",
        "mixamorig:LeftUpLeg",
    ),
    "right_hip": (
        "RightUpLeg",
        "RightUpperLeg",
        "RightThigh",
        "R_UpLeg",
        "R_Hip",
        "r_up_leg",
        "right_upper_leg",
        "mixamorig:RightUpLeg",
    ),
    "left_knee": (
        "LeftLeg",
        "LeftLowerLeg",
        "LeftKnee",
        "L_Leg",
        "L_LowerLeg",
        "l_low_leg",
        "left_lower_leg",
        "mixamorig:LeftLeg",
    ),
    "right_knee": (
        "RightLeg",
        "RightLowerLeg",
        "RightKnee",
        "R_Leg",
        "R_LowerLeg",
        "r_low_leg",
        "right_lower_leg",
        "mixamorig:RightLeg",
    ),
    "left_ankle": (
        "LeftFoot",
        "LeftAnkle",
        "L_Foot",
        "l_foot",
        "left_foot",
        "mixamorig:LeftFoot",
    ),
    "right_ankle": (
        "RightFoot",
        "RightAnkle",
        "R_Foot",
        "r_foot",
        "right_foot",
        "mixamorig:RightFoot",
    ),
    "left_foot": (
        "LeftToeBase",
        "LeftToe",
        "LeftToe_End",
        "l_toes",
        "left_toes",
        "mixamorig:LeftToeBase",
    ),
    "right_foot": (
        "RightToeBase",
        "RightToe",
        "RightToe_End",
        "r_toes",
        "right_toes",
        "mixamorig:RightToeBase",
    ),
}

SMPL_PARENT_INDICES = [
    -1,
    0,
    0,
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    9,
    9,
    12,
    13,
    14,
    16,
    17,
    18,
    19,
    20,
    21,
]

SMPL_JOINT_SOURCE_KEYS = [
    "pelvis",
    "left_hip",
    "right_hip",
    "spine",
    "left_knee",
    "right_knee",
    "chest",
    "left_ankle",
    "right_ankle",
    "chest",
    "left_foot",
    "right_foot",
    "neck",
    "left_shoulder",
    "right_shoulder",
    "head",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_wrist",
    "right_wrist",
]


@dataclass
class BvhMotion:
    path: str
    joint_names: list[str]
    world_positions: np.ndarray
    world_quat_wxyz: np.ndarray
    source_fps: float
    source_frame_time_s: float
    frame_stride: int
    playback_fps: float
    selected_indices: dict[str, int]
    smpl_source_indices: list[int | None]
    lower_body_retarget_scale: float = 0.0

    @property
    def frame_count(self) -> int:
        return int(self.world_positions.shape[0])


class BvhPlaybackSource:
    """Threaded source that replays a BVH file as canonical mocap frames."""

    def __init__(
        self,
        bvh_file: str,
        target_fps: float | None = None,
        loop: bool = False,
        unit_scale: float = 0.01,
        y_up_to_z_up: bool = True,
        body_local: bool = True,
        lower_body_retarget_scale: float = 0.0,
    ):
        self.bvh_file = bvh_file
        self.target_fps = target_fps
        self.loop = bool(loop)
        self.unit_scale = float(unit_scale)
        self.y_up_to_z_up = bool(y_up_to_z_up)
        self.body_local = bool(body_local)
        self.lower_body_retarget_scale = max(0.0, float(lower_body_retarget_scale))

        self.motion = load_bvh_motion(
            bvh_file,
            target_fps=target_fps,
            unit_scale=unit_scale,
            y_up_to_z_up=y_up_to_z_up,
            body_local=body_local,
            lower_body_retarget_scale=self.lower_body_retarget_scale,
        )
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._lock = threading.Lock()
        self._latest: MocapFrame | None = None
        self._last_error: str | None = None
        self._frames_emitted = 0
        self._stopped_at_end = False
        self._last_joint_pos: np.ndarray | None = None
        self._last_stream_frame_idx: int | None = None
        self._last_source_frame_idx: int | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._running.set()
        self._thread = threading.Thread(target=self._run, name="BvhPlaybackSource", daemon=True)
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
                "fps": self.motion.playback_fps,
                "received_packets": self._frames_emitted,
                "frames_emitted": self._frames_emitted,
                "dropped_packets": 0,
                "last_error": self._last_error,
                "has_frame": self._latest is not None,
                "stopped_at_end": self._stopped_at_end,
                "lower_body_retarget_scale": self.motion.lower_body_retarget_scale,
            }

    def _run(self) -> None:
        frame_period_s = 1.0 / max(1.0, self.motion.playback_fps)
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
            frame_idx += self.motion.frame_stride
            if frame_idx >= self.motion.frame_count:
                if self.loop:
                    frame_idx = frame_idx % self.motion.frame_count
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
        joints: dict[str, Pose7D] = {}
        for target_name, joint_idx in self.motion.selected_indices.items():
            joints[target_name] = Pose7D(
                position=self.motion.world_positions[frame_idx, joint_idx],
                quat_wxyz=self.motion.world_quat_wxyz[frame_idx, joint_idx],
            )
        full_body = _build_full_body_reference(self.motion, frame_idx)
        if self.motion.lower_body_retarget_scale > 0.0:
            joint_vel = np.zeros_like(full_body.joint_pos, dtype=np.float32)
            if (
                self._last_joint_pos is not None
                and self._last_stream_frame_idx is not None
                and self._last_source_frame_idx is not None
                and frame_idx > self._last_source_frame_idx
            ):
                dt_s = max(
                    1e-3,
                    float(stream_frame_idx - self._last_stream_frame_idx)
                    / max(1.0, self.motion.playback_fps),
                )
                joint_vel = ((full_body.joint_pos - self._last_joint_pos) / dt_s).astype(np.float32)
            full_body.joint_vel = joint_vel
            self._last_joint_pos = full_body.joint_pos.copy()
            self._last_stream_frame_idx = int(stream_frame_idx)
            self._last_source_frame_idx = int(frame_idx)

        return MocapFrame(
            source="bvh",
            host_time_s=time.time(),
            frame_index=int(stream_frame_idx),
            fps=float(self.motion.playback_fps),
            joints=joints,
            full_body=full_body,
            metadata={
                "format": "bvh",
                "path": self.motion.path,
                "source_frame_index": int(frame_idx),
                "source_fps": self.motion.source_fps,
                "frame_stride": self.motion.frame_stride,
                "joint_names": self.motion.joint_names,
            },
        )


def load_bvh_motion(
    bvh_file: str,
    target_fps: float | None = None,
    unit_scale: float = 0.01,
    y_up_to_z_up: bool = True,
    body_local: bool = True,
    lower_body_retarget_scale: float = 0.0,
) -> BvhMotion:
    joints, channel_order, motion_data, _, source_frame_time_s = parse_bvh_file(bvh_file)
    source_fps = 1.0 / source_frame_time_s if source_frame_time_s > 0.0 else 30.0
    playback_fps = float(target_fps) if target_fps and target_fps > 0.0 else source_fps
    frame_stride = max(1, int(round(source_fps / playback_fps))) if source_fps > playback_fps else 1
    playback_fps = source_fps / frame_stride

    joint_names = [joint["name"] for joint in joints]
    world_positions, world_rots = compute_bvh_fk(joints, channel_order, motion_data)
    world_positions = world_positions.astype(np.float32) * float(unit_scale)

    if body_local:
        root_idx = _find_first_joint_index(joint_names, BVH_DEFAULT_JOINT_ALIASES["root"])
        if root_idx is not None:
            world_positions = world_positions - world_positions[:, root_idx : root_idx + 1, :]

    if y_up_to_z_up:
        world_positions = _positions_y_up_to_z_up(world_positions)
        world_rots = _rotations_y_up_to_z_up(world_rots)

    world_quat_xyzw = Rotation.from_matrix(world_rots.reshape(-1, 3, 3)).as_quat()
    world_quat_wxyz = world_quat_xyzw[:, [3, 0, 1, 2]].reshape(
        world_rots.shape[0], world_rots.shape[1], 4
    )

    selected_indices = _resolve_selected_indices(joint_names)
    missing = [name for name in ("left_wrist", "right_wrist") if name not in selected_indices]
    if missing:
        raise ValueError(
            f"BVH file {bvh_file!r} is missing required joints for {missing}; "
            f"available joints include: {joint_names[:20]}"
        )

    return BvhMotion(
        path=osp.abspath(bvh_file),
        joint_names=joint_names,
        world_positions=world_positions.astype(np.float32),
        world_quat_wxyz=world_quat_wxyz.astype(np.float32),
        source_fps=float(source_fps),
        source_frame_time_s=float(source_frame_time_s),
        frame_stride=int(frame_stride),
        playback_fps=float(playback_fps),
        selected_indices=selected_indices,
        smpl_source_indices=[
            selected_indices.get(source_key) for source_key in SMPL_JOINT_SOURCE_KEYS
        ],
        lower_body_retarget_scale=max(0.0, float(lower_body_retarget_scale)),
    )


def build_full_body_reference_from_skeleton_frame(
    joint_names: list[str] | tuple[str, ...],
    world_positions: np.ndarray,
    world_quat_wxyz: np.ndarray,
    *,
    frame_index: int | None = None,
    lower_body_retarget_scale: float = 0.0,
    body_quat_w: np.ndarray | None = None,
    body_pos_w: np.ndarray | None = None,
    body_pos: np.ndarray | None = None,
    joint_pos: np.ndarray | None = None,
    joint_vel: np.ndarray | None = None,
) -> FullBodyReference:
    """Build one SMPL-like reference from a named world-space skeleton frame."""
    names = [str(name) for name in joint_names]
    positions = np.asarray(world_positions, dtype=np.float32)
    quats = np.asarray(world_quat_wxyz, dtype=np.float32)
    if positions.shape != (len(names), 3):
        raise ValueError(
            f"world_positions must have shape ({len(names)}, 3), got {positions.shape}"
        )
    if quats.shape != (len(names), 4):
        raise ValueError(f"world_quat_wxyz must have shape ({len(names)}, 4), got {quats.shape}")

    selected_indices = _resolve_selected_indices(names)
    motion = BvhMotion(
        path="skeleton_frame",
        joint_names=names,
        world_positions=positions.reshape(1, len(names), 3),
        world_quat_wxyz=quats.reshape(1, len(names), 4),
        source_fps=0.0,
        source_frame_time_s=0.0,
        frame_stride=1,
        playback_fps=0.0,
        selected_indices=selected_indices,
        smpl_source_indices=[
            selected_indices.get(source_key) for source_key in SMPL_JOINT_SOURCE_KEYS
        ],
        lower_body_retarget_scale=max(0.0, float(lower_body_retarget_scale)),
    )
    reference = _build_full_body_reference(motion, 0)
    return FullBodyReference(
        smpl_joints=reference.smpl_joints,
        smpl_pose=reference.smpl_pose,
        body_quat_w=reference.body_quat_w if body_quat_w is None else body_quat_w,
        body_pos_w=reference.body_pos_w if body_pos_w is None else body_pos_w,
        body_pos=body_pos,
        joint_pos=reference.joint_pos if joint_pos is None else joint_pos,
        joint_vel=reference.joint_vel if joint_vel is None else joint_vel,
        frame_index=frame_index,
    )


def _build_full_body_reference(motion: BvhMotion, frame_idx: int) -> FullBodyReference:
    smpl_joints = np.zeros((24, 3), dtype=np.float32)
    smpl_pose = np.zeros((21, 3), dtype=np.float32)

    root_idx = motion.smpl_source_indices[0]
    root_position = (
        motion.world_positions[frame_idx, root_idx]
        if root_idx is not None
        else np.zeros(3, dtype=np.float32)
    )
    root_quat_wxyz = (
        motion.world_quat_wxyz[frame_idx, root_idx]
        if root_idx is not None
        else np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    )
    root_rot = _rotation_from_wxyz(root_quat_wxyz)
    root_inv = root_rot.inv()

    for smpl_idx, source_idx in enumerate(motion.smpl_source_indices):
        if source_idx is not None:
            world_position = motion.world_positions[frame_idx, source_idx]
            smpl_joints[smpl_idx] = root_inv.apply(world_position - root_position)
            continue

        parent_idx = SMPL_PARENT_INDICES[smpl_idx]
        if parent_idx >= 0:
            smpl_joints[smpl_idx] = smpl_joints[parent_idx]

    world_quats_xyzw = motion.world_quat_wxyz[frame_idx][:, [1, 2, 3, 0]]
    world_rots = Rotation.from_quat(world_quats_xyzw)
    for smpl_idx in range(1, min(22, len(motion.smpl_source_indices))):
        source_idx = motion.smpl_source_indices[smpl_idx]
        if source_idx is None:
            continue

        parent_smpl_idx = SMPL_PARENT_INDICES[smpl_idx]
        parent_source_idx = (
            motion.smpl_source_indices[parent_smpl_idx] if parent_smpl_idx >= 0 else None
        )
        child_rot = world_rots[source_idx]
        if parent_source_idx is not None:
            local_rot = world_rots[parent_source_idx].inv() * child_rot
        else:
            local_rot = root_inv * child_rot
        smpl_pose[smpl_idx - 1] = local_rot.as_rotvec().astype(np.float32)

    joint_pos = smpl_pose_to_g1_wrist_joint_pos(smpl_pose)
    if motion.lower_body_retarget_scale > 0.0:
        _apply_smpl_lower_body_to_g1_joint_pos(
            joint_pos,
            smpl_pose,
            motion.lower_body_retarget_scale,
        )

    return FullBodyReference(
        smpl_joints=smpl_joints,
        smpl_pose=smpl_pose,
        body_quat_w=root_quat_wxyz,
        joint_pos=joint_pos,
        frame_index=int(frame_idx),
    )


def _apply_smpl_lower_body_to_g1_joint_pos(
    joint_pos: np.ndarray,
    smpl_pose: np.ndarray,
    scale: float,
) -> None:
    scale = max(0.0, float(scale))
    if scale <= 0.0:
        return

    body_pose = np.asarray(smpl_pose, dtype=np.float32).reshape(21, 3)
    default = G1_DEFAULT_JOINT_POS_ISAACLAB
    lower = G1_LOWER_BODY_JOINT_IDX_ISAACLAB

    left_hip = body_pose[0]
    right_hip = body_pose[1]
    left_knee = body_pose[3]
    right_knee = body_pose[4]
    left_ankle = body_pose[6]
    right_ankle = body_pose[7]

    def delta(value: float, gain: float, limit: float) -> float:
        return float(np.clip(value * gain * scale, -limit, limit))

    def knee_delta(rotvec: np.ndarray) -> float:
        flexion = float(np.linalg.norm(rotvec))
        return float(np.clip(flexion * 0.45 * scale, -0.10, 0.50))

    # Match deploy's lower_body_joint_mujoco_order_in_isaaclab_index sampling order.
    # The policy observes these 12 slots as L hip pitch/roll/yaw, L knee,
    # L ankle pitch/roll, then the right side.
    joint_pos[lower[0]] = default[lower[0]] + delta(-left_hip[0], 0.35, 0.35)
    joint_pos[lower[1]] = default[lower[1]] + delta(left_hip[1], 0.25, 0.25)
    joint_pos[lower[2]] = default[lower[2]] + delta(left_hip[2], 0.20, 0.25)
    joint_pos[lower[3]] = default[lower[3]] + knee_delta(left_knee)
    joint_pos[lower[4]] = default[lower[4]] + delta(-left_ankle[0], 0.25, 0.25)
    joint_pos[lower[5]] = default[lower[5]] + delta(left_ankle[1], 0.20, 0.20)

    joint_pos[lower[6]] = default[lower[6]] + delta(-right_hip[0], 0.35, 0.35)
    joint_pos[lower[7]] = default[lower[7]] + delta(-right_hip[1], 0.25, 0.25)
    joint_pos[lower[8]] = default[lower[8]] + delta(-right_hip[2], 0.20, 0.25)
    joint_pos[lower[9]] = default[lower[9]] + knee_delta(right_knee)
    joint_pos[lower[10]] = default[lower[10]] + delta(-right_ankle[0], 0.25, 0.25)
    joint_pos[lower[11]] = default[lower[11]] + delta(-right_ankle[1], 0.20, 0.20)


def parse_bvh_file(filepath: str):
    with open(filepath, encoding="utf-8") as f:
        lines = f.readlines()

    joints: list[dict[str, Any]] = []
    joint_stack: list[int | None] = []
    channel_order: list[tuple[int, str]] = []
    i = 0

    while i < len(lines):
        line = lines[i].strip()
        if line == "MOTION":
            i += 1
            break

        match = re.match(r"(ROOT|JOINT)\s+(\S+)", line)
        if match:
            name = match.group(2)
            parent_idx = next((idx for idx in reversed(joint_stack) if idx is not None), -1)
            joints.append({"name": name, "offset": None, "channels": [], "parent_idx": parent_idx})
            joint_stack.append(len(joints) - 1)
        elif line == "End Site":
            joint_stack.append(None)
        elif line.startswith("OFFSET") and joint_stack and joint_stack[-1] is not None:
            values = [float(x) for x in line.split()[1:]]
            joints[joint_stack[-1]]["offset"] = np.asarray(values, dtype=np.float32)
        elif line.startswith("CHANNELS") and joint_stack and joint_stack[-1] is not None:
            parts = line.split()
            channel_count = int(parts[1])
            channel_names = parts[2 : 2 + channel_count]
            joints[joint_stack[-1]]["channels"] = channel_names
            for channel_name in channel_names:
                channel_order.append((joint_stack[-1], channel_name))
        elif line == "}" and joint_stack:
            joint_stack.pop()
        i += 1

    if i + 1 >= len(lines):
        raise ValueError(f"BVH file {filepath!r} has no MOTION section")

    frames_line = lines[i].strip()
    frame_count = int(frames_line.split(":")[1])
    i += 1
    frame_time_s = float(lines[i].strip().split(":")[1])
    i += 1

    motion_data = np.empty((frame_count, len(channel_order)), dtype=np.float32)
    for frame_idx in range(frame_count):
        values = lines[i].strip().split()
        if len(values) != len(channel_order):
            raise ValueError(
                f"frame {frame_idx} in {filepath!r} has {len(values)} values, "
                f"expected {len(channel_order)}"
            )
        motion_data[frame_idx] = [float(value) for value in values]
        i += 1

    return joints, channel_order, motion_data, frame_count, frame_time_s


def compute_bvh_fk(joints, channel_order, motion_data):
    frame_count = motion_data.shape[0]
    joint_count = len(joints)
    joint_channels = {joint_idx: [] for joint_idx in range(joint_count)}
    for channel_idx, (joint_idx, channel_name) in enumerate(channel_order):
        joint_channels[joint_idx].append((channel_idx, channel_name))

    world_rots = np.zeros((frame_count, joint_count, 3, 3), dtype=np.float32)
    world_pos = np.zeros((frame_count, joint_count, 3), dtype=np.float32)

    for joint_idx, joint in enumerate(joints):
        offset = joint["offset"] if joint["offset"] is not None else np.zeros(3, dtype=np.float32)
        pos_channels: dict[str, int] = {}
        rot_order = ""
        rot_indices = []

        for channel_idx, channel_name in joint_channels[joint_idx]:
            if channel_name.endswith("position"):
                pos_channels[channel_name] = channel_idx
            elif channel_name.endswith("rotation"):
                rot_order += channel_name[0].lower()
                rot_indices.append(channel_idx)

        if pos_channels:
            local_pos = np.zeros((frame_count, 3), dtype=np.float32)
            if "Xposition" in pos_channels:
                local_pos[:, 0] = motion_data[:, pos_channels["Xposition"]]
            if "Yposition" in pos_channels:
                local_pos[:, 1] = motion_data[:, pos_channels["Yposition"]]
            if "Zposition" in pos_channels:
                local_pos[:, 2] = motion_data[:, pos_channels["Zposition"]]
        else:
            local_pos = np.tile(offset, (frame_count, 1)).astype(np.float32)

        if rot_order:
            local_rot = Rotation.from_euler(
                rot_order.upper(), motion_data[:, rot_indices], degrees=True
            ).as_matrix()
        else:
            local_rot = np.tile(np.eye(3, dtype=np.float32), (frame_count, 1, 1))

        parent_idx = int(joint["parent_idx"])
        if parent_idx < 0:
            world_rots[:, joint_idx] = local_rot
            world_pos[:, joint_idx] = local_pos
        else:
            parent_rot = world_rots[:, parent_idx]
            parent_pos = world_pos[:, parent_idx]
            world_pos[:, joint_idx] = parent_pos + np.einsum("fij,fj->fi", parent_rot, local_pos)
            world_rots[:, joint_idx] = np.einsum("fij,fjk->fik", parent_rot, local_rot)

    return world_pos, world_rots


def _resolve_selected_indices(joint_names: list[str]) -> dict[str, int]:
    selected: dict[str, int] = {}
    for target_name, aliases in BVH_DEFAULT_JOINT_ALIASES.items():
        idx = _find_first_joint_index(joint_names, aliases)
        if idx is not None:
            selected[target_name] = idx
    return selected


def _rotation_from_wxyz(quat_wxyz: np.ndarray) -> Rotation:
    quat = np.asarray(quat_wxyz, dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if norm < 1e-8 or not np.isfinite(norm):
        quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    else:
        quat = quat / norm
    return Rotation.from_quat(quat[[1, 2, 3, 0]])


def _find_first_joint_index(joint_names: list[str], aliases: tuple[str, ...]) -> int | None:
    name_to_idx = {name: idx for idx, name in enumerate(joint_names)}
    lower_to_idx = {name.lower(): idx for idx, name in enumerate(joint_names)}
    for alias in aliases:
        if alias in name_to_idx:
            return name_to_idx[alias]
        if alias.lower() in lower_to_idx:
            return lower_to_idx[alias.lower()]
    return None


def _positions_y_up_to_z_up(positions: np.ndarray) -> np.ndarray:
    converted = positions.copy()
    converted[..., 0] = positions[..., 0]
    converted[..., 1] = -positions[..., 2]
    converted[..., 2] = positions[..., 1]
    return converted


def _rotations_y_up_to_z_up(rotations: np.ndarray) -> np.ndarray:
    basis = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float32,
    )
    return np.einsum("ij,ftjk,lk->ftil", basis, rotations, basis)
