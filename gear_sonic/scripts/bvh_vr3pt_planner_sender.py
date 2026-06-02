#!/usr/bin/env python3
"""Stream a BVH file as VR 3-point planner targets for SONIC ZMQ manager.

This is a lightweight bridge for offline BVH files that are not recorded by
PICO. It parses the BVH, computes FK, extracts left hand, right hand, and
head/neck targets relative to the root, then publishes them on the same
`planner` topic consumed by `ZMQManager` in planner VR-3PT mode.
"""

from __future__ import annotations

import argparse
import os
import re
import time
from dataclasses import dataclass
from typing import Iterable

import numpy as np
from scipy.spatial.transform import Rotation
import zmq

from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
    build_command_message,
    build_planner_message,
)


DEFAULT_ORIENTATION_WXYZ = [
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
]


@dataclass
class BvhData:
    names: list[str]
    parents: list[int]
    offsets: np.ndarray
    joint_channels: list[list[tuple[int, str]]]
    motion: np.ndarray
    frame_time: float


def _parse_bvh(path: str) -> BvhData:
    with open(path, encoding="utf-8", errors="replace") as f:
        lines = f.readlines()

    names: list[str] = []
    parents: list[int] = []
    offsets: list[np.ndarray] = []
    joint_stack: list[int] = []
    channel_order: list[tuple[int, str]] = []
    channels_by_joint: list[list[tuple[int, str]]] = []

    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line == "MOTION":
            i += 1
            break

        m = re.match(r"(ROOT|JOINT)\s+(\S+)", line)
        if m:
            names.append(m.group(2))
            parents.append(joint_stack[-1] if joint_stack else -1)
            offsets.append(np.zeros(3, dtype=np.float64))
            channels_by_joint.append([])
            joint_stack.append(len(names) - 1)
        elif line.startswith("OFFSET") and joint_stack:
            offsets[joint_stack[-1]] = np.array(
                [float(x) for x in line.split()[1:4]], dtype=np.float64
            )
        elif line.startswith("CHANNELS") and joint_stack:
            parts = line.split()
            count = int(parts[1])
            joint_idx = joint_stack[-1]
            for ch_name in parts[2 : 2 + count]:
                channel_idx = len(channel_order)
                channel_order.append((joint_idx, ch_name))
                channels_by_joint[joint_idx].append((channel_idx, ch_name))
        elif line == "}" and joint_stack:
            joint_stack.pop()
        i += 1

    if i >= len(lines) or not lines[i].strip().startswith("Frames:"):
        raise ValueError("BVH MOTION section is missing a Frames line")
    frames = int(lines[i].split(":")[1])
    i += 1

    if i >= len(lines) or not lines[i].strip().startswith("Frame Time:"):
        raise ValueError("BVH MOTION section is missing a Frame Time line")
    frame_time = float(lines[i].split(":")[1])
    i += 1

    motion = np.empty((frames, len(channel_order)), dtype=np.float64)
    for frame_idx in range(frames):
        values = lines[i + frame_idx].strip().split()
        if len(values) != len(channel_order):
            raise ValueError(
                f"Frame {frame_idx} has {len(values)} values, expected {len(channel_order)}"
            )
        motion[frame_idx] = [float(v) for v in values]

    return BvhData(names, parents, np.vstack(offsets), channels_by_joint, motion, frame_time)


def _compute_fk(data: BvhData) -> tuple[np.ndarray, np.ndarray]:
    frames = data.motion.shape[0]
    joints = len(data.names)
    world_pos = np.zeros((frames, joints, 3), dtype=np.float64)
    world_rot = np.tile(np.eye(3, dtype=np.float64), (frames, joints, 1, 1))

    for joint_idx in range(joints):
        pos_channels: dict[str, int] = {}
        rot_order = ""
        rot_indices: list[int] = []

        for channel_idx, channel_name in data.joint_channels[joint_idx]:
            if channel_name.endswith("position"):
                pos_channels[channel_name] = channel_idx
            elif channel_name.endswith("rotation"):
                rot_order += channel_name[0].lower()
                rot_indices.append(channel_idx)

        if pos_channels:
            local_pos = np.zeros((frames, 3), dtype=np.float64)
            if "Xposition" in pos_channels:
                local_pos[:, 0] = data.motion[:, pos_channels["Xposition"]]
            if "Yposition" in pos_channels:
                local_pos[:, 1] = data.motion[:, pos_channels["Yposition"]]
            if "Zposition" in pos_channels:
                local_pos[:, 2] = data.motion[:, pos_channels["Zposition"]]
        else:
            local_pos = np.repeat(data.offsets[joint_idx][None, :], frames, axis=0)

        if rot_order:
            local_rot = Rotation.from_euler(
                rot_order.upper(), data.motion[:, rot_indices], degrees=True
            ).as_matrix()
        else:
            local_rot = np.tile(np.eye(3, dtype=np.float64), (frames, 1, 1))

        parent = data.parents[joint_idx]
        if parent < 0:
            world_pos[:, joint_idx] = local_pos
            world_rot[:, joint_idx] = local_rot
        else:
            world_pos[:, joint_idx] = world_pos[:, parent] + np.einsum(
                "fij,fj->fi", world_rot[:, parent], local_pos
            )
            world_rot[:, joint_idx] = np.einsum("fij,fjk->fik", world_rot[:, parent], local_rot)

    return world_pos, world_rot


def _first_existing(names: list[str], candidates: Iterable[str]) -> int:
    lower_to_idx = {name.lower(): i for i, name in enumerate(names)}
    for candidate in candidates:
        idx = lower_to_idx.get(candidate.lower())
        if idx is not None:
            return idx
    raise ValueError(f"None of these joints exist in BVH: {', '.join(candidates)}")


def _axis_basis(axis_map: str) -> np.ndarray:
    """Return matrix B such that robot_vec = B @ bvh_vec."""
    if axis_map == "mcp":
        # Common mocap convention in this BVH: X=left, Y=up, Z=forward.
        return np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    if axis_map == "soma":
        # Conversion used by extract_soma_joints_from_bvh.py: (x, y, z) -> (x, -z, y).
        return np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    if axis_map == "unity":
        # Conversion used by PICO Unity poses: (x, y, z) -> (-x, z, y).
        return np.array([[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])
    raise ValueError(f"Unsupported axis map: {axis_map}")


def _to_robot_frame(points_yup: np.ndarray, axis_map: str) -> np.ndarray:
    basis = _axis_basis(axis_map)
    return np.einsum("ij,...j->...i", basis, points_yup)


def _retarget_vr3pt(
    data: BvhData,
    position_scale: float,
    send_orientation: bool,
    root_joint: str | None,
    axis_map: str,
) -> tuple[np.ndarray, np.ndarray | None]:
    world_pos, world_rot = _compute_fk(data)

    root_idx = (
        data.names.index(root_joint)
        if root_joint is not None
        else _first_existing(data.names, ["root", "hips", "pelvis"])
    )
    left_idx = _first_existing(data.names, ["l_hand", "left_hand", "LeftHand"])
    right_idx = _first_existing(data.names, ["r_hand", "right_hand", "RightHand"])
    head_idx = _first_existing(data.names, ["head", "neck_2", "neck_1", "Head", "Neck"])
    selected = [left_idx, right_idx, head_idx]

    root_rot_inv = np.swapaxes(world_rot[:, root_idx], 1, 2)
    rel_yup = np.einsum(
        "fij,fpj->fpi", root_rot_inv, world_pos[:, selected] - world_pos[:, root_idx, None, :]
    )
    positions = _to_robot_frame(rel_yup / 100.0, axis_map) * position_scale

    if not send_orientation:
        return positions.astype(np.float32), None

    basis = _axis_basis(axis_map)
    rel_rot_yup = np.einsum("fij,fpjk->fpik", root_rot_inv, world_rot[:, selected])
    rel_rot_robot = np.einsum("ij,fpjk,kl->fpil", basis, rel_rot_yup, basis.T)
    quat_xyzw = Rotation.from_matrix(rel_rot_robot.reshape(-1, 3, 3)).as_quat()
    quat_wxyz = quat_xyzw[:, [3, 0, 1, 2]].reshape(positions.shape[0], 3, 4)
    return positions.astype(np.float32), quat_wxyz.astype(np.float32)


def _send_initial_messages(socket: zmq.Socket) -> None:
    # Give subscribers time to connect. PUB/SUB drops early messages otherwise.
    time.sleep(0.5)
    socket.send(build_command_message(start=True, stop=False, planner=True))
    socket.send(
        build_planner_message(
            mode=0,
            movement=[0.0, 0.0, 0.0],
            facing=[1.0, 0.0, 0.0],
            speed=-1.0,
            height=-1.0,
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Publish BVH left/right hand/head as SONIC planner VR-3PT targets."
    )
    parser.add_argument(
        "bvh",
        nargs="?",
        default="/home/nolo/MCPM_20260526_190029.BVH",
        help="BVH file to stream",
    )
    parser.add_argument("--port", type=int, default=5556, help="ZMQ PUB port")
    parser.add_argument("--bind-host", default="*", help="ZMQ bind host, default '*'")
    parser.add_argument("--fps", type=float, default=None, help="Playback FPS; default uses BVH FPS")
    parser.add_argument("--scale", type=float, default=1.0, help="Scale relative 3-point positions")
    parser.add_argument(
        "--axis-map",
        choices=["mcp", "soma", "unity"],
        default="mcp",
        help="BVH-to-robot axis map. Default 'mcp' fits MCPM_*.BVH.",
    )
    parser.add_argument("--root-joint", default=None, help="Root joint name for body-local targets")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--end-frame", type=int, default=None)
    parser.add_argument("--loop", action="store_true", help="Loop playback until interrupted")
    parser.add_argument(
        "--send-orientation",
        action="store_true",
        help="Also send BVH-derived relative quaternions instead of SONIC defaults",
    )
    parser.add_argument("--dry-run", action="store_true", help="Parse and print stats without ZMQ")
    args = parser.parse_args()

    if not os.path.exists(args.bvh):
        raise FileNotFoundError(args.bvh)

    data = _parse_bvh(args.bvh)
    positions, orientations = _retarget_vr3pt(
        data, args.scale, args.send_orientation, args.root_joint, args.axis_map
    )

    source_fps = 1.0 / data.frame_time
    fps = float(args.fps if args.fps is not None else source_fps)
    start = max(0, args.start_frame)
    end = min(len(positions), args.end_frame if args.end_frame is not None else len(positions))
    if start >= end:
        raise ValueError(f"Invalid frame range: start={start}, end={end}")

    print(
        f"Loaded {args.bvh}: {len(data.names)} joints, {len(positions)} frames, "
        f"{source_fps:.2f} Hz BVH -> {fps:.2f} Hz playback"
    )
    print(f"Streaming frames [{start}, {end}) as [left_hand, right_hand, head] VR-3PT targets")
    print(f"First target positions (m): {positions[start].reshape(-1).round(4).tolist()}")

    if args.dry_run:
        return

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    endpoint = f"tcp://{args.bind_host}:{args.port}"
    socket.bind(endpoint)
    print(f"ZMQ PUB bound to {endpoint}")

    try:
        _send_initial_messages(socket)
        frame_dt = 1.0 / fps
        while True:
            for frame in range(start, end):
                vr_position = positions[frame].reshape(-1).tolist()
                vr_orientation = (
                    orientations[frame].reshape(-1).tolist()
                    if orientations is not None
                    else DEFAULT_ORIENTATION_WXYZ
                )
                socket.send(
                    build_planner_message(
                        mode=0,
                        movement=[0.0, 0.0, 0.0],
                        facing=[1.0, 0.0, 0.0],
                        speed=-1.0,
                        height=-1.0,
                        vr_3pt_position=vr_position,
                        vr_3pt_orientation=vr_orientation,
                    )
                )
                time.sleep(frame_dt)

            if not args.loop:
                break
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        try:
            socket.send(build_command_message(start=False, stop=True, planner=True))
            time.sleep(0.05)
        finally:
            socket.close()
            context.term()


if __name__ == "__main__":
    main()
