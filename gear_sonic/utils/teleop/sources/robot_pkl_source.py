"""Retargeted robot PKL playback source for SONIC/G1 mocap validation."""

from __future__ import annotations

import os.path as osp
import threading
import time
from dataclasses import dataclass
from typing import Any

import joblib
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from gear_sonic.utils.teleop.sources.base import (
    FullBodyReference,
    G1_DEFAULT_ROOT_POS_W,
    G1_ISAACLAB_TO_MUJOCO_IDX,
    MocapFrame,
    Pose7D,
    normalize_quat_wxyz,
)


@dataclass
class RobotPklMotion:
    path: str
    motion_name: str
    joint_pos_isaaclab: np.ndarray
    joint_vel_isaaclab: np.ndarray
    root_pos_w: np.ndarray
    root_quat_wxyz: np.ndarray
    source_fps: float
    playback_fps: float
    body_pos: np.ndarray | None = None

    @property
    def frame_count(self) -> int:
        return int(self.joint_pos_isaaclab.shape[0])


class RobotPklPlaybackSource:
    """Threaded source that replays retargeted G1 PKL mocap as joint-reference frames.

    The expected PKL format matches the robot-filtered samples used by the
    IsaacLab SONIC mocap work: ``{motion_name: {dof, root_rot, root_trans_offset}}``.
    ``dof`` is stored in MuJoCo/URDF order and is converted to SONIC/IsaacLab order.
    """

    def __init__(
        self,
        pkl_file: str,
        target_fps: float | None = None,
        loop: bool = False,
        align_root: bool = True,
    ):
        self.pkl_file = pkl_file
        self.target_fps = target_fps
        self.loop = bool(loop)
        self.align_root = bool(align_root)

        self.motion = load_robot_pkl_motion(
            pkl_file,
            target_fps=target_fps,
            align_root=align_root,
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
        self._thread = threading.Thread(target=self._run, name="RobotPklPlaybackSource", daemon=True)
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
            source="robot_pkl",
            host_time_s=time.time(),
            frame_index=int(stream_frame_idx),
            fps=float(self.motion.playback_fps),
            joints={"root": root_pose},
            full_body=full_body,
            metadata={
                "format": "robot_pkl",
                "path": self.motion.path,
                "motion_name": self.motion.motion_name,
                "source_frame_index": int(frame_idx),
                "source_fps": self.motion.source_fps,
            },
        )


def load_robot_pkl_motion(
    pkl_file: str,
    target_fps: float | None = None,
    align_root: bool = True,
) -> RobotPklMotion:
    raw = joblib.load(pkl_file)
    if not isinstance(raw, dict) or not raw:
        raise ValueError(f"robot PKL {pkl_file!r} must contain a non-empty motion dict")
    motion_name = next(iter(raw.keys()))
    motion = raw[motion_name]

    dof_mujoco = np.asarray(motion["dof"], dtype=np.float32)
    if dof_mujoco.ndim != 2 or dof_mujoco.shape[1] != 29:
        raise ValueError(f"robot PKL dof must have shape (T, 29), got {dof_mujoco.shape}")

    dof_vel_mujoco = None
    if "dof_vel" in motion:
        dof_vel_mujoco = np.asarray(motion["dof_vel"], dtype=np.float32)
    elif "joint_vel" in motion:
        dof_vel_mujoco = np.asarray(motion["joint_vel"], dtype=np.float32)
    if dof_vel_mujoco is not None and dof_vel_mujoco.shape != dof_mujoco.shape:
        raise ValueError(
            f"robot PKL dof_vel must have shape {dof_mujoco.shape}, got {dof_vel_mujoco.shape}"
        )

    root_rot_xyzw = np.asarray(motion["root_rot"], dtype=np.float32)
    if root_rot_xyzw.shape != (dof_mujoco.shape[0], 4):
        raise ValueError(
            f"robot PKL root_rot must have shape ({dof_mujoco.shape[0]}, 4), "
            f"got {root_rot_xyzw.shape}"
        )

    root_trans = np.asarray(
        motion.get("root_trans_offset", np.tile(G1_DEFAULT_ROOT_POS_W, (dof_mujoco.shape[0], 1))),
        dtype=np.float32,
    )
    if root_trans.shape != (dof_mujoco.shape[0], 3):
        raise ValueError(
            f"robot PKL root_trans_offset must have shape ({dof_mujoco.shape[0]}, 3), "
            f"got {root_trans.shape}"
        )

    body_pos = None
    if "body_pos" in motion:
        body_pos = np.asarray(motion["body_pos"], dtype=np.float32)
        if body_pos.ndim != 3 or body_pos.shape[0] != dof_mujoco.shape[0] or body_pos.shape[2] != 3:
            raise ValueError(
                f"robot PKL body_pos must have shape ({dof_mujoco.shape[0]}, B, 3), "
                f"got {body_pos.shape}"
            )

    source_fps = float(motion.get("fps") or 30.0)
    playback_fps = float(target_fps) if target_fps and target_fps > 0.0 else 50.0
    dof_mujoco, root_rot_xyzw, root_trans, dof_vel_mujoco, body_pos = _resample_motion(
        dof_mujoco,
        root_rot_xyzw,
        root_trans,
        source_fps=source_fps,
        target_fps=playback_fps,
        dof_vel=dof_vel_mujoco,
        body_pos=body_pos,
    )

    if align_root and root_rot_xyzw.shape[0] > 0:
        root0 = Rotation.from_quat(root_rot_xyzw[0])
        inv_root0 = root0.inv()
        root_trans0 = root_trans[0].copy()
        root_trans = root_trans0 + inv_root0.apply(root_trans - root_trans0)
        root_rot_xyzw = (inv_root0 * Rotation.from_quat(root_rot_xyzw)).as_quat()

    root_quat_wxyz = root_rot_xyzw[:, [3, 0, 1, 2]].astype(np.float32)
    root_quat_wxyz = np.stack([normalize_quat_wxyz(q) for q in root_quat_wxyz], axis=0)
    joint_pos_isaaclab = dof_mujoco[:, G1_ISAACLAB_TO_MUJOCO_IDX].astype(np.float32)
    if dof_vel_mujoco is not None:
        joint_vel_isaaclab = dof_vel_mujoco[:, G1_ISAACLAB_TO_MUJOCO_IDX].astype(np.float32)
    else:
        joint_vel_isaaclab = _finite_difference(joint_pos_isaaclab, playback_fps)

    return RobotPklMotion(
        path=osp.abspath(pkl_file),
        motion_name=str(motion_name),
        joint_pos_isaaclab=joint_pos_isaaclab,
        joint_vel_isaaclab=joint_vel_isaaclab,
        root_pos_w=root_trans.astype(np.float32),
        root_quat_wxyz=root_quat_wxyz,
        source_fps=float(source_fps),
        playback_fps=float(playback_fps),
        body_pos=body_pos.astype(np.float32) if body_pos is not None else None,
    )


def _resample_motion(
    dof: np.ndarray,
    root_rot_xyzw: np.ndarray,
    root_trans: np.ndarray,
    source_fps: float,
    target_fps: float,
    dof_vel: np.ndarray | None = None,
    body_pos: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray | None, np.ndarray | None]:
    if abs(float(source_fps) - float(target_fps)) < 1e-6:
        return (
            dof.astype(np.float32),
            root_rot_xyzw.astype(np.float32),
            root_trans.astype(np.float32),
            dof_vel.astype(np.float32) if dof_vel is not None else None,
            body_pos.astype(np.float32) if body_pos is not None else None,
        )

    frame_count = dof.shape[0]
    duration_s = max(0.0, float(frame_count - 1) / max(1e-6, float(source_fps)))
    t_src = np.arange(frame_count, dtype=np.float64) / float(source_fps)
    t_out = np.arange(0.0, duration_s, 1.0 / float(target_fps), dtype=np.float64)
    if t_out.size == 0:
        t_out = np.array([0.0], dtype=np.float64)

    dof_out = _interp_array(dof, t_src, t_out)
    dof_vel_out = _interp_array(dof_vel, t_src, t_out) if dof_vel is not None else None
    body_pos_out = _interp_body_pos(body_pos, t_src, t_out) if body_pos is not None else None
    root_trans_out = _interp_array(root_trans, t_src, t_out)
    root_rot_out = Slerp(t_src, Rotation.from_quat(root_rot_xyzw))(t_out).as_quat()
    return dof_out, root_rot_out.astype(np.float32), root_trans_out, dof_vel_out, body_pos_out


def _interp_array(values: np.ndarray, t_src: np.ndarray, t_out: np.ndarray) -> np.ndarray:
    out = np.empty((t_out.shape[0], values.shape[1]), dtype=np.float32)
    for idx in range(values.shape[1]):
        out[:, idx] = np.interp(t_out, t_src, values[:, idx])
    return out


def _interp_body_pos(values: np.ndarray, t_src: np.ndarray, t_out: np.ndarray) -> np.ndarray:
    original_shape = values.shape
    flat = values.reshape(original_shape[0], -1)
    return _interp_array(flat, t_src, t_out).reshape(t_out.shape[0], original_shape[1], 3)


def _finite_difference(values: np.ndarray, fps: float) -> np.ndarray:
    vel = np.zeros_like(values, dtype=np.float32)
    if values.shape[0] > 1:
        vel[:-1] = (values[1:] - values[:-1]) * float(fps)
        vel[-1] = vel[-2]
    return vel
