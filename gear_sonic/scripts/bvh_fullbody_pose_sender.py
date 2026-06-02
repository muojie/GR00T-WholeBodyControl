#!/usr/bin/env python3
"""Retarget a BVH file to G1 29-DOF motion and stream it over ZMQ.

This sender uses the existing decoupled_wbc Pink/Pinocchio IK stack to turn a
human BVH into conservative G1 whole-body joint targets. It aligns the first
BVH frame to the robot's default standing pose, then tracks BVH relative motion
for hands, feet, head, and torso.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import zmq

from decoupled_wbc.control.robot_model.instantiation.g1 import instantiate_g1_robot_model
from gear_sonic.scripts.bvh_vr3pt_planner_sender import (
    DEFAULT_ORIENTATION_WXYZ,
    _axis_basis,
    _compute_fk,
    _first_existing,
    _parse_bvh,
    _retarget_vr3pt,
)
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
    build_command_message,
    build_planner_message,
    pack_pose_message,
)


ISAACLAB_TO_MUJOCO = np.array(
    [0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18, 2, 5, 8, 11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28],
    dtype=np.int64,
)
MUJOCO_TO_ISAACLAB = np.empty_like(ISAACLAB_TO_MUJOCO)
MUJOCO_TO_ISAACLAB[ISAACLAB_TO_MUJOCO] = np.arange(ISAACLAB_TO_MUJOCO.size)


BVH_TO_G1_TARGETS = {
    "left_wrist_yaw_link": ["l_hand", "left_hand", "LeftHand"],
    "right_wrist_yaw_link": ["r_hand", "right_hand", "RightHand"],
    "left_ankle_roll_link": ["l_foot", "left_foot", "LeftFoot"],
    "right_ankle_roll_link": ["r_foot", "right_foot", "RightFoot"],
    "head_link": ["head", "Head", "neck_2", "neck_1"],
    "torso_link": ["torso_7", "chest", "Chest", "torso_6"],
}


class SimpleG1IK:
    """Small dependency-light IK wrapper around Pinocchio FK + scipy least_squares."""

    def __init__(
        self,
        robot,
        position_cost: float,
        orientation_cost: float,
        posture_cost: float,
        max_nfev: int,
    ):
        self.robot = robot
        self.body_indices = robot.get_body_actuated_joint_indices()
        self.q_template = robot.default_body_pose.copy()
        self.q_default_body = self.q_template[self.body_indices].copy()
        self.lower = robot.lower_joint_limits[self.body_indices]
        self.upper = robot.upper_joint_limits[self.body_indices]
        self.position_scale = float(np.sqrt(position_cost))
        self.orientation_scale = float(np.sqrt(max(0.0, orientation_cost)))
        self.posture_scale = float(np.sqrt(posture_cost))
        self.max_nfev = max_nfev
        self.x = self.q_default_body.copy()

    def _residual(self, x: np.ndarray, targets: dict[str, np.ndarray]) -> np.ndarray:
        q = self.q_template.copy()
        q[self.body_indices] = x
        self.robot.cache_forward_kinematics(q)

        residuals = []
        for link_name, target in targets.items():
            actual = self.robot.frame_placement(link_name).np
            residuals.append(self.position_scale * (actual[:3, 3] - target[:3, 3]))
            if self.orientation_scale > 0.0:
                rot_err = Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).as_rotvec()
                residuals.append(self.orientation_scale * rot_err)

        residuals.append(self.posture_scale * (x - self.q_default_body))
        return np.concatenate(residuals)

    def __call__(self, targets: dict[str, np.ndarray]) -> np.ndarray:
        result = least_squares(
            self._residual,
            self.x,
            args=(targets,),
            bounds=(self.lower, self.upper),
            max_nfev=self.max_nfev,
            xtol=1e-4,
            ftol=1e-4,
            gtol=1e-4,
            verbose=0,
        )
        self.x = result.x.astype(np.float64)
        q = self.q_template.copy()
        q[self.body_indices] = self.x
        return q


def _make_ik_solver(
    position_cost: float,
    orientation_cost: float,
    max_nfev: int,
    posture_cost: float,
):
    robot = instantiate_g1_robot_model(waist_location="lower_and_upper_body")
    solver = SimpleG1IK(robot, position_cost, orientation_cost, posture_cost, max_nfev)
    return robot, solver


def _retarget_with_ik(
    bvh_path: str,
    axis_map: str,
    scale: float,
    start: int,
    end: int | None,
    stride: int,
    position_cost: float,
    orientation_cost: float,
    max_nfev: int,
    posture_cost: float,
) -> tuple[np.ndarray, float]:
    data = _parse_bvh(bvh_path)
    world_pos, world_rot = _compute_fk(data)
    basis = _axis_basis(axis_map)

    root_idx = _first_existing(data.names, ["root", "hips", "pelvis"])
    root_rot_inv = np.swapaxes(world_rot[:, root_idx], 1, 2)

    frame_end = min(world_pos.shape[0], end if end is not None else world_pos.shape[0])
    frame_indices = list(range(max(0, start), frame_end, max(1, stride)))
    if not frame_indices:
        raise ValueError(f"No frames selected from start={start}, end={frame_end}, stride={stride}")

    robot, solver = _make_ik_solver(position_cost, orientation_cost, max_nfev, posture_cost)
    robot.cache_forward_kinematics(robot.default_body_pose)

    bvh_joint_indices = {
        link_name: _first_existing(data.names, candidates)
        for link_name, candidates in BVH_TO_G1_TARGETS.items()
    }
    default_poses = {
        link_name: robot.frame_placement(link_name).np.copy()
        for link_name in BVH_TO_G1_TARGETS
    }

    first_frame = frame_indices[0]
    first_rel_pos = {}
    first_rel_rot = {}
    for link_name, bvh_idx in bvh_joint_indices.items():
        rel_yup = root_rot_inv[first_frame] @ (world_pos[first_frame, bvh_idx] - world_pos[first_frame, root_idx])
        first_rel_pos[link_name] = basis @ (rel_yup / 100.0)
        rel_rot_yup = root_rot_inv[first_frame] @ world_rot[first_frame, bvh_idx]
        first_rel_rot[link_name] = basis @ rel_rot_yup @ basis.T

    q_mujoco = np.zeros((len(frame_indices), 29), dtype=np.float32)

    for out_idx, frame in enumerate(frame_indices):
        targets = {}
        for link_name, bvh_idx in bvh_joint_indices.items():
            rel_yup = root_rot_inv[frame] @ (world_pos[frame, bvh_idx] - world_pos[frame, root_idx])
            rel_pos_robot = basis @ (rel_yup / 100.0)
            displacement = (rel_pos_robot - first_rel_pos[link_name]) * scale

            target = default_poses[link_name].copy()
            target[:3, 3] = default_poses[link_name][:3, 3] + displacement

            if orientation_cost > 0.0:
                rel_rot_yup = root_rot_inv[frame] @ world_rot[frame, bvh_idx]
                rel_rot_robot = basis @ rel_rot_yup @ basis.T
                delta_rot = first_rel_rot[link_name].T @ rel_rot_robot
                target[:3, :3] = default_poses[link_name][:3, :3] @ delta_rot

            targets[link_name] = target

        q_full = solver(targets)
        q_mujoco[out_idx] = q_full[robot.get_body_actuated_joint_indices()].astype(np.float32)

        if out_idx % 100 == 0:
            print(f"[IK] {out_idx + 1}/{len(frame_indices)} frames", flush=True)

    playback_fps = (1.0 / data.frame_time) / max(1, stride)
    return q_mujoco, playback_fps


def _stream_pose(
    q_mujoco: np.ndarray,
    fps: float,
    port: int,
    bind_host: str,
    chunk_size: int,
    loop: bool,
    start_control: bool,
) -> None:
    q_isaaclab = q_mujoco[:, MUJOCO_TO_ISAACLAB].astype(np.float32)
    joint_vel = np.gradient(q_isaaclab, 1.0 / fps, axis=0).astype(np.float32)
    body_quat = np.zeros((q_isaaclab.shape[0], 1, 4), dtype=np.float32)
    body_quat[:, 0, 0] = 1.0

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    endpoint = f"tcp://{bind_host}:{port}"
    socket.bind(endpoint)
    print(f"ZMQ PUB bound to {endpoint}")

    try:
        time.sleep(0.5)
        for _ in range(10):
            socket.send(build_command_message(start=start_control, stop=False, planner=False))
            time.sleep(0.05)

        frame_dt = 1.0 / fps
        global_frame = 0
        while True:
            socket.send(build_command_message(start=start_control, stop=False, planner=False))
            for start in range(0, q_isaaclab.shape[0], chunk_size):
                end = min(q_isaaclab.shape[0], start + chunk_size)
                indices = np.arange(global_frame, global_frame + (end - start), dtype=np.int64)
                msg = pack_pose_message(
                    {
                        "joint_pos": q_isaaclab[start:end],
                        "joint_vel": joint_vel[start:end],
                        "body_quat_w": body_quat[start:end],
                        "frame_index": indices,
                        "catch_up": np.array([False], dtype=bool),
                    },
                    topic="pose",
                    version=1,
                )
                socket.send(msg)
                global_frame += end - start
                time.sleep(frame_dt * (end - start))

            if not loop:
                break
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        try:
            socket.send(build_command_message(start=False, stop=True, planner=False))
            time.sleep(0.05)
        finally:
            socket.close()
            context.term()


def _estimate_root_speed(
    bvh_path: str,
    axis_map: str,
    start: int,
    end: int | None,
    stride: int,
    speed_scale: float,
    speed_override: float | None,
) -> tuple[float, int, int, float]:
    data = _parse_bvh(bvh_path)
    world_pos, _ = _compute_fk(data)
    root_idx = _first_existing(data.names, ["root", "hips", "pelvis"])
    basis = _axis_basis(axis_map)
    root_robot = np.einsum("ij,fj->fi", basis, world_pos[:, root_idx] / 100.0)

    frame_start = max(0, start)
    frame_end = min(root_robot.shape[0], end if end is not None else root_robot.shape[0])
    selected = root_robot[frame_start:frame_end:max(1, stride)]
    if selected.shape[0] < 2:
        return 0.0 if speed_override is None else speed_override, frame_start, frame_end, 1.0 / data.frame_time

    fps = (1.0 / data.frame_time) / max(1, stride)
    if speed_override is not None:
        return speed_override, frame_start, frame_end, fps

    # Use net forward displacement in robot +X. This keeps the command stable
    # even if the BVH root has small tracking jitter.
    duration = (selected.shape[0] - 1) / fps
    forward_speed = (selected[-1, 0] - selected[0, 0]) / max(duration, 1e-6)
    return float(max(0.0, forward_speed * speed_scale)), frame_start, frame_end, fps


def _stream_planner_locomotion(
    bvh_path: str,
    axis_map: str,
    scale: float,
    start: int,
    end: int | None,
    stride: int,
    port: int,
    bind_host: str,
    loop: bool,
    locomotion_mode: int,
    speed_scale: float,
    speed_override: float | None,
    send_orientation: bool,
) -> None:
    data = _parse_bvh(bvh_path)
    positions, orientations = _retarget_vr3pt(data, scale, send_orientation, None, axis_map)
    speed, frame_start, frame_end, fps = _estimate_root_speed(
        bvh_path, axis_map, start, end, stride, speed_scale, speed_override
    )
    frame_indices = list(range(frame_start, frame_end, max(1, stride)))
    if not frame_indices:
        raise ValueError(f"No frames selected from start={start}, end={frame_end}, stride={stride}")

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    endpoint = f"tcp://{bind_host}:{port}"
    socket.bind(endpoint)
    print(f"ZMQ PUB bound to {endpoint}")
    print(
        f"Planner locomotion: mode={locomotion_mode}, speed={speed:.3f} m/s, "
        f"{len(frame_indices)} frames @ {fps:.2f} Hz"
    )

    try:
        time.sleep(0.5)
        for _ in range(10):
            socket.send(build_command_message(start=True, stop=False, planner=True))
            time.sleep(0.05)

        frame_dt = 1.0 / fps
        while True:
            socket.send(build_command_message(start=True, stop=False, planner=True))
            for frame in frame_indices:
                socket.send(build_command_message(start=True, stop=False, planner=True))
                vr_orientation = (
                    orientations[frame].reshape(-1).tolist()
                    if orientations is not None
                    else DEFAULT_ORIENTATION_WXYZ
                )
                socket.send(
                    build_planner_message(
                        mode=locomotion_mode if speed > 0.03 else 0,
                        movement=[1.0, 0.0, 0.0] if speed > 0.03 else [0.0, 0.0, 0.0],
                        facing=[1.0, 0.0, 0.0],
                        speed=speed if speed > 0.03 else -1.0,
                        height=-1.0,
                        vr_3pt_position=positions[frame].reshape(-1).tolist(),
                        vr_3pt_orientation=vr_orientation,
                    )
                )
                time.sleep(frame_dt)

            if not loop:
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Retarget BVH to G1 29DOF and stream pose topic.")
    parser.add_argument(
        "bvh",
        nargs="?",
        default="/home/nolo/MCPM_20260526_190029.BVH",
        help="BVH file to retarget",
    )
    parser.add_argument("--port", type=int, default=5556)
    parser.add_argument("--bind-host", default="*")
    parser.add_argument("--axis-map", choices=["mcp", "soma", "unity"], default="mcp")
    parser.add_argument(
        "--control-mode",
        choices=["planner", "streamed"],
        default="planner",
        help="planner walks using SONIC locomotion planner + BVH upper-body VR3PT; streamed plays IK 29DOF in place.",
    )
    parser.add_argument("--scale", type=float, default=0.7, help="Scale BVH relative displacements")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--end-frame", type=int, default=None)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--chunk-size", type=int, default=5)
    parser.add_argument("--position-cost", type=float, default=8.0)
    parser.add_argument("--orientation-cost", type=float, default=0.0)
    parser.add_argument("--max-nfev", type=int, default=25, help="Max least-squares evaluations per frame")
    parser.add_argument("--posture-cost", type=float, default=0.02)
    parser.add_argument("--output-npz", default="", help="Optional cache path for retargeted q_mujoco")
    parser.add_argument("--input-npz", default="", help="Load cached retargeted q_mujoco instead of IK")
    parser.add_argument("--dry-run", action="store_true", help="Retarget but do not publish")
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--locomotion-mode", type=int, default=2, help="SONIC LocomotionMode, 2=WALK")
    parser.add_argument("--speed-scale", type=float, default=1.0, help="Scale estimated BVH root speed")
    parser.add_argument("--planner-speed", type=float, default=None, help="Override planner walking speed")
    parser.add_argument("--send-orientation", action="store_true", help="Send BVH wrist/head orientations")
    parser.add_argument(
        "--no-start-control",
        action="store_true",
        help="Do not send the start-control command, only switch to streamed-motion mode.",
    )
    args = parser.parse_args()

    if args.control_mode == "planner":
        speed, frame_start, frame_end, fps = _estimate_root_speed(
            args.bvh,
            args.axis_map,
            args.start_frame,
            args.end_frame,
            args.stride,
            args.speed_scale,
            args.planner_speed,
        )
        print(
            f"Planner mode preview: frames [{frame_start}, {frame_end}), "
            f"{fps:.2f} Hz, speed={speed:.3f} m/s"
        )
        if args.dry_run:
            return
        _stream_planner_locomotion(
            args.bvh,
            args.axis_map,
            args.scale,
            args.start_frame,
            args.end_frame,
            args.stride,
            args.port,
            args.bind_host,
            args.loop,
            args.locomotion_mode,
            args.speed_scale,
            args.planner_speed,
            args.send_orientation,
        )
        return

    estimated_speed, frame_start, frame_end, _ = _estimate_root_speed(
        args.bvh,
        args.axis_map,
        args.start_frame,
        args.end_frame,
        args.stride,
        args.speed_scale,
        args.planner_speed,
    )
    if estimated_speed > 0.03:
        print(
            "WARNING: streamed mode does not command BVH root X/Y locomotion; "
            f"this clip has estimated root speed {estimated_speed:.3f} m/s "
            f"over frames [{frame_start}, {frame_end}). "
            "Walking clips may look like the robot is being dragged. "
            "Use --control-mode planner for locomotion."
        )

    if args.input_npz:
        cached = np.load(args.input_npz)
        q_mujoco = cached["q_mujoco"].astype(np.float32)
        fps = float(cached["fps"])
        print(f"Loaded cached retargeting: {args.input_npz}, {q_mujoco.shape[0]} frames @ {fps:.2f} Hz")
    else:
        if not os.path.exists(args.bvh):
            raise FileNotFoundError(args.bvh)
        q_mujoco, fps = _retarget_with_ik(
            args.bvh,
            args.axis_map,
            args.scale,
            args.start_frame,
            args.end_frame,
            args.stride,
            args.position_cost,
            args.orientation_cost,
            args.max_nfev,
            args.posture_cost,
        )
        print(f"Retargeted {q_mujoco.shape[0]} frames @ {fps:.2f} Hz")
        if args.output_npz:
            np.savez_compressed(args.output_npz, q_mujoco=q_mujoco, fps=np.array([fps], dtype=np.float32))
            print(f"Saved cache: {args.output_npz}")

    print(f"q_mujoco range: [{q_mujoco.min():.3f}, {q_mujoco.max():.3f}]")
    if args.dry_run:
        return

    _stream_pose(
        q_mujoco,
        fps,
        args.port,
        args.bind_host,
        args.chunk_size,
        args.loop,
        start_control=not args.no_start_control,
    )


if __name__ == "__main__":
    main()
