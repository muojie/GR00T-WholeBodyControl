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

from gear_sonic.utils.teleop.sources.base import MocapFrame, Pose7D


BVH_DEFAULT_JOINT_ALIASES = {
    "root": ("Hips", "Hip", "Pelvis", "Root", "root"),
    "left_wrist": ("LeftHand", "LeftWrist", "L_Hand", "mixamorig:LeftHand"),
    "right_wrist": ("RightHand", "RightWrist", "R_Hand", "mixamorig:RightHand"),
    "head": ("Head", "HeadTop_End", "HeadEnd", "mixamorig:Head"),
}


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
    ):
        self.bvh_file = bvh_file
        self.target_fps = target_fps
        self.loop = bool(loop)
        self.unit_scale = float(unit_scale)
        self.y_up_to_z_up = bool(y_up_to_z_up)
        self.body_local = bool(body_local)

        self.motion = load_bvh_motion(
            bvh_file,
            target_fps=target_fps,
            unit_scale=unit_scale,
            y_up_to_z_up=y_up_to_z_up,
            body_local=body_local,
        )
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._lock = threading.Lock()
        self._latest: MocapFrame | None = None
        self._last_error: str | None = None
        self._frames_emitted = 0
        self._stopped_at_end = False

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
            }

    def _run(self) -> None:
        frame_period_s = 1.0 / max(1.0, self.motion.playback_fps)
        frame_idx = 0
        next_tick_s = time.time()

        while self._running.is_set():
            try:
                frame = self._build_frame(frame_idx)
            except Exception as exc:
                with self._lock:
                    self._last_error = str(exc)
                break

            with self._lock:
                self._latest = frame
                self._frames_emitted += 1
                self._last_error = None

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

    def _build_frame(self, frame_idx: int) -> MocapFrame:
        joints: dict[str, Pose7D] = {}
        for target_name, joint_idx in self.motion.selected_indices.items():
            joints[target_name] = Pose7D(
                position=self.motion.world_positions[frame_idx, joint_idx],
                quat_wxyz=self.motion.world_quat_wxyz[frame_idx, joint_idx],
            )

        return MocapFrame(
            source="bvh",
            host_time_s=time.time(),
            frame_index=int(frame_idx),
            fps=float(self.motion.playback_fps),
            joints=joints,
            metadata={
                "format": "bvh",
                "path": self.motion.path,
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
    )


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
