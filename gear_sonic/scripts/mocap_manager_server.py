"""Generic mocap teleop manager for non-PICO input sources.

This entry point publishes the same ZMQ topics consumed by the deploy side:
`command`, `planner`, and `manager_state`. It is intentionally separate from
`pico_manager_thread_server.py` so new motion-capture sources can be validated
without changing the PICO/XR control path.
"""

from __future__ import annotations

import argparse
import sys
import time
from enum import IntEnum

import numpy as np
import zmq

from gear_sonic.utils.teleop.controls import LineControlSource
from gear_sonic.utils.teleop.retarget import UpperBodyIKRetargeter, VR3PointRetargeter
from gear_sonic.utils.teleop.sources import (
    BVH_STREAM_DEFAULT_PORT,
    G1_ISAACLAB_JOINT_NAMES,
    MOCOPI_DEFAULT_PORT,
    BvhG1PlaybackSource,
    BvhG1RetargetConfig,
    BvhPlaybackSource,
    BvhStreamUdpSource,
    JointProbePlaybackSource,
    MocopiUdpSource,
    RobotPklPlaybackSource,
    resolve_g1_isaaclab_joint_index,
)
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
    build_command_message,
    build_planner_message,
    pack_pose_message,
)
from gear_sonic.utils.teleop.zmq.zmq_pose_sender import PoseStreamPublisher


class LocomotionMode(IntEnum):
    IDLE = 0
    SLOW_WALK = 1
    WALK = 2
    RUN = 3
    IDLE_SQUAT = 4
    IDLE_KNEEL_TWO_LEGS = 5
    IDLE_KNEEL = 6
    IDLE_LYING_FACE_DOWN = 7
    CRAWLING = 8
    IDLE_BOXING = 9
    WALK_BOXING = 10
    LEFT_PUNCH = 11
    RIGHT_PUNCH = 12
    RANDOM_PUNCH = 13
    ELBOW_CRAWLING = 14
    LEFT_HOOK = 15
    RIGHT_HOOK = 16
    FORWARD_JUMP = 17
    STEALTH_WALK = 18
    INJURED_WALK = 19


class StreamMode(IntEnum):
    OFF = 0
    POSE = 1
    PLANNER = 2
    PLANNER_FROZEN_UPPER_BODY = 3
    POSE_PAUSE = 4
    PLANNER_VR_3PT = 5


POSE_DEBUG_WINDOW_SIZE = 5
POSE_CONTROL_WINDOW_SIZE = 80
POSE_FILTER_PROFILES = {
    "stable": {
        "enable_reference_filter": True,
        "reference_alpha": 0.35,
        "max_smpl_joint_speed_mps": 1.2,
        "max_smpl_pose_speed_radps": 3.0,
        "max_root_angular_speed_radps": 2.5,
        "max_joint_speed_radps": 4.0,
        "root_tilt_limit_rad": 0.45,
    },
    "responsive": {
        "enable_reference_filter": True,
        "reference_alpha": 0.75,
        "max_smpl_joint_speed_mps": 3.5,
        "max_smpl_pose_speed_radps": 10.0,
        "max_root_angular_speed_radps": 7.0,
        "max_joint_speed_radps": 14.0,
        "root_tilt_limit_rad": 0.65,
    },
    "off": {
        "enable_reference_filter": False,
        "reference_alpha": 1.0,
        "max_smpl_joint_speed_mps": 0.0,
        "max_smpl_pose_speed_radps": 0.0,
        "max_root_angular_speed_radps": 0.0,
        "max_joint_speed_radps": 0.0,
        "root_tilt_limit_rad": 0.0,
    },
}


def _send_manager_state(socket: zmq.Socket, stream_mode: StreamMode, toggle_dc: bool, toggle_da: bool) -> None:
    socket.send(
        pack_pose_message(
            {
                "stream_mode": np.array([stream_mode.value], dtype=np.int32),
                "toggle_data_collection": np.array([toggle_dc], dtype=bool),
                "toggle_data_abort": np.array([toggle_da], dtype=bool),
            },
            topic="manager_state",
        )
    )


def _create_vr3pt_visualizer(args: argparse.Namespace):
    if not args.visualize_vr3pt:
        return None

    try:
        from gear_sonic.utils.teleop.vis.vr3pt_pose_visualizer import VR3PtPoseVisualizer
    except ImportError as exc:
        raise RuntimeError(
            "VR 3-point visualization requires PyVista/VTK dependencies from .venv_teleop"
        ) from exc

    visualizer = VR3PtPoseVisualizer(with_g1_robot=args.visualize_g1)
    visualizer.create_realtime_plotter(
        interactive=True,
        window_size=(args.visualize_width, args.visualize_height),
        with_reference_frames=True,
    )
    print("[MocapManager] VR 3-point visualization window opened")
    return visualizer


def _vr_arrays_to_pose(vr_position: np.ndarray, vr_orientation: np.ndarray) -> np.ndarray:
    return np.concatenate(
        (
            np.asarray(vr_position, dtype=np.float32).reshape(3, 3),
            np.asarray(vr_orientation, dtype=np.float32).reshape(3, 4),
        ),
        axis=1,
    )


def _resolve_pose_encoder_mode(args: argparse.Namespace) -> int:
    encoder_modes = {
        "g1": 0,
        "teleop": 1,
        "smpl": 2,
    }
    return encoder_modes[args.pose_encoder_mode]


def _resolve_bvh_lower_body_retarget_scale(args: argparse.Namespace) -> float:
    if args.bvh_lower_body_retarget_scale is not None:
        return min(1.0, max(0.0, float(args.bvh_lower_body_retarget_scale)))
    return 0.0


def _build_bvh_g1_retarget_config(
    args: argparse.Namespace,
    *,
    ik_mode: str,
) -> BvhG1RetargetConfig:
    return BvhG1RetargetConfig(
        method=args.bvh_g1_method,
        retarget_scale=args.bvh_g1_retarget_scale,
        lower_body_scale=args.bvh_g1_lower_scale,
        upper_body_scale=args.bvh_g1_upper_scale,
        wrist_scale=args.bvh_g1_wrist_scale,
        waist_scale=args.bvh_g1_waist_scale,
        max_joint_velocity_radps=args.bvh_g1_max_joint_velocity,
        max_joint_step_rad=args.bvh_g1_max_joint_step,
        joint_filter_alpha=args.bvh_g1_joint_filter_alpha,
        joint_limit_margin_rad=args.bvh_g1_joint_limit_margin,
        joint_delta_limit_scale=args.bvh_g1_joint_delta_limit_scale,
        skeleton_sign_correction=not args.bvh_g1_no_skeleton_sign_correction,
        skeleton_segment_direction=args.bvh_g1_segment_direction,
        skeleton_axis_mapping=args.bvh_g1_axis_map,
        ik_mode=ik_mode,
        root_mode=args.bvh_g1_root_mode,
        max_root_angular_velocity_radps=args.bvh_g1_root_max_angular_velocity,
        root_tilt_limit_rad=args.bvh_g1_root_tilt_limit,
        min_root_height_m=args.bvh_g1_min_root_height,
        enable_body_fk=not args.bvh_g1_no_body_fk,
        smpl_joints_source=args.bvh_g1_smpl_joints_source,
    )


def _create_source(args: argparse.Namespace):
    if args.source == "joint_probe":
        joint_index = resolve_g1_isaaclab_joint_index(
            args.joint_probe_index,
            args.joint_probe_name,
        )
        source = JointProbePlaybackSource(
            joint_index=joint_index,
            amplitude_rad=args.joint_probe_amplitude,
            frequency_hz=args.joint_probe_frequency,
            target_fps=args.joint_probe_fps,
        )
        description = (
            f"probing G1 IsaacLab joint[{source.joint_index}]={source.joint_name} "
            f"(MuJoCo index {source.mujoco_index}) with sine amplitude="
            f"{source.amplitude_rad:.3f}rad frequency={source.frequency_hz:.3f}Hz "
            f"at {source.target_fps:.1f}Hz"
        )
        return source, description

    if args.source == "mocopi":
        source = MocopiUdpSource(
            bind_host=args.mocopi_host,
            port=args.mocopi_port,
            packet_format=args.mocopi_format,
        )
        description = (
            f"listening for mocopi UDP on {args.mocopi_host}:{args.mocopi_port} "
            f"({args.mocopi_format})"
        )
        return source, description

    if args.source == "bvh":
        if not args.bvh_file:
            raise ValueError("--bvh-file is required when --source bvh")
        lower_body_retarget_scale = _resolve_bvh_lower_body_retarget_scale(args)
        source = BvhPlaybackSource(
            bvh_file=args.bvh_file,
            target_fps=args.bvh_fps,
            loop=args.bvh_loop,
            unit_scale=args.bvh_unit_scale,
            y_up_to_z_up=not args.bvh_no_y_up_to_z_up,
            body_local=not args.bvh_world_frame,
            lower_body_retarget_scale=lower_body_retarget_scale,
        )
        description = (
            f"playing BVH {args.bvh_file} at {source.motion.playback_fps:.1f} Hz "
            f"(source_fps={source.motion.source_fps:.1f}, stride={source.motion.frame_stride}, "
            f"loop={args.bvh_loop}, lower_body_scale={lower_body_retarget_scale:.2f})"
        )
        return source, description

    if args.source == "pkl":
        if not args.pkl_file:
            raise ValueError("--pkl-file is required when --source pkl")
        source = RobotPklPlaybackSource(
            pkl_file=args.pkl_file,
            target_fps=args.pkl_fps,
            loop=args.pkl_loop,
            align_root=not args.pkl_no_align_root,
        )
        description = (
            f"playing robot PKL {args.pkl_file} motion={source.motion.motion_name!r} "
            f"at {source.motion.playback_fps:.1f} Hz "
            f"(source_fps={source.motion.source_fps:.1f}, frames={source.motion.frame_count}, "
            f"loop={args.pkl_loop}, align_root={int(not args.pkl_no_align_root)})"
        )
        return source, description

    if args.source == "bvh_g1":
        if not args.bvh_file:
            raise ValueError("--bvh-file is required when --source bvh_g1")
        bvh_g1_ik_mode = _resolve_bvh_g1_ik_mode(args)
        retarget_config = _build_bvh_g1_retarget_config(
            args,
            ik_mode=bvh_g1_ik_mode,
        )
        source = BvhG1PlaybackSource(
            bvh_file=args.bvh_file,
            target_fps=args.bvh_fps,
            loop=args.bvh_loop,
            unit_scale=args.bvh_unit_scale,
            y_up_to_z_up=not args.bvh_no_y_up_to_z_up,
            local_root=args.bvh_g1_local_root,
            align_root=not args.bvh_g1_no_align_root,
            retarget_config=retarget_config,
            runtime_mode=args.bvh_g1_runtime_mode,
        )
        description = (
            f"retargeting BVH {args.bvh_file} to G1 joint references at "
            f"{source._playback_fps:.1f} Hz "
            f"(source_fps={_bvh_g1_source_fps(source):.1f}, frames={source._frame_count}, "
            f"loop={args.bvh_loop}, method={args.bvh_g1_method}, "
            f"runtime={args.bvh_g1_runtime_mode}, ik={bvh_g1_ik_mode}, "
            f"scale={args.bvh_g1_retarget_scale:.2f}, "
            f"lower={args.bvh_g1_lower_scale:.2f}, upper={args.bvh_g1_upper_scale:.2f}, "
            f"wrist={args.bvh_g1_wrist_scale:.2f}, "
            f"waist={args.bvh_g1_waist_scale:.2f}, "
            f"max_dq={args.bvh_g1_max_joint_velocity:.1f}rad/s, "
            f"max_step={args.bvh_g1_max_joint_step:.2f}rad, "
            f"alpha={args.bvh_g1_joint_filter_alpha:.2f}, "
            f"limit_margin={args.bvh_g1_joint_limit_margin:.2f}rad, "
            f"delta_limit={args.bvh_g1_joint_delta_limit_scale:.2f}x, "
            f"sign_fix={int(not args.bvh_g1_no_skeleton_sign_correction)}, "
            f"seg_dir={args.bvh_g1_segment_direction}, "
            f"axis_map={args.bvh_g1_axis_map}, "
            f"root_mode={args.bvh_g1_root_mode}, "
            f"root_max_w={args.bvh_g1_root_max_angular_velocity:.1f}rad/s, "
            f"root_tilt={args.bvh_g1_root_tilt_limit:.2f}rad, "
            f"root_min_z={args.bvh_g1_min_root_height:.2f}m, "
            f"body_fk={int(not args.bvh_g1_no_body_fk)}, "
            f"smpl_joints={args.bvh_g1_smpl_joints_source}, "
            f"align_root={int(not args.bvh_g1_no_align_root)})"
        )
        return source, description

    if args.source == "bvh_stream":
        bvh_g1_ik_mode = _resolve_bvh_g1_ik_mode(args)
        retarget_config = _build_bvh_g1_retarget_config(
            args,
            ik_mode=bvh_g1_ik_mode,
        )
        source = BvhStreamUdpSource(
            bind_host=args.bvh_stream_host,
            port=args.bvh_stream_port,
            packet_format=args.bvh_stream_format,
            recv_size=args.bvh_stream_recv_size,
            retarget_config=retarget_config,
            align_root=not args.bvh_g1_no_align_root,
        )
        description = (
            f"listening for BVH stream UDP on {args.bvh_stream_host}:{args.bvh_stream_port} "
            f"({args.bvh_stream_format}); retargeting streamed BVH frames to G1 joint references "
            f"(ik={bvh_g1_ik_mode}, scale={args.bvh_g1_retarget_scale:.2f}, "
            f"lower={args.bvh_g1_lower_scale:.2f}, upper={args.bvh_g1_upper_scale:.2f}, "
            f"axis_map={args.bvh_g1_axis_map}, body_fk={int(not args.bvh_g1_no_body_fk)}, "
            f"smpl_joints={args.bvh_g1_smpl_joints_source}, "
            f"align_root={int(not args.bvh_g1_no_align_root)})"
        )
        return source, description

    raise ValueError(f"unsupported source {args.source!r}")


def _resolve_bvh_g1_ik_mode(args: argparse.Namespace) -> str:
    mode = str(args.bvh_g1_ik_mode or "auto").strip().lower()
    if mode == "auto":
        return "analytic" if args.bvh_g1_runtime_mode == "online" else "numeric"
    return mode


def _bvh_g1_source_fps(source: BvhG1PlaybackSource) -> float:
    if source.motion is not None:
        return float(source.motion.source_fps)
    if source.retarget_context is not None:
        return float(source.retarget_context.source_fps)
    return 0.0


def _resolve_pose_window_size(args: argparse.Namespace) -> int:
    if args.pose_window_size is not None:
        window_size = int(args.pose_window_size)
    elif args.control_mode == "pose":
        window_size = POSE_CONTROL_WINDOW_SIZE
    else:
        window_size = POSE_DEBUG_WINDOW_SIZE

    if window_size <= 0:
        raise ValueError("--pose-window-size must be positive")
    return window_size


def _resolve_pose_filter_settings(args: argparse.Namespace) -> dict[str, float | bool]:
    settings = dict(POSE_FILTER_PROFILES[args.pose_filter_profile])
    if args.no_pose_reference_filter:
        settings = dict(POSE_FILTER_PROFILES["off"])

    cli_overrides = {
        "reference_alpha": args.pose_reference_alpha,
        "max_smpl_joint_speed_mps": args.pose_max_smpl_joint_speed,
        "max_smpl_pose_speed_radps": args.pose_max_smpl_pose_speed,
        "max_root_angular_speed_radps": args.pose_max_root_angular_speed,
        "max_joint_speed_radps": args.pose_max_joint_speed,
        "root_tilt_limit_rad": args.pose_root_tilt_limit,
    }
    for key, value in cli_overrides.items():
        if value is not None:
            settings[key] = float(value)
    return settings


def run_mocap_manager(args: argparse.Namespace) -> None:
    source, source_description = _create_source(args)
    retargeter = VR3PointRetargeter(
        calibrate_on_first_frame=not args.no_vr3pt_calibration,
        position_scale=args.vr3pt_scale,
        allow_bone_translation_vr=args.allow_bone_translation_vr,
        fk_calibration=not args.no_vr3pt_fk_calibration,
        require_fk_calibration=args.require_vr3pt_fk_calibration,
        enable_filter=not args.no_vr3pt_filter,
        position_alpha=args.vr3pt_position_alpha,
        orientation_alpha=args.vr3pt_orientation_alpha,
        max_position_speed=args.vr3pt_max_speed,
        max_position_accel=args.vr3pt_max_accel,
        max_angular_speed=args.vr3pt_max_angular_speed,
    )
    upper_body_ik = None
    if args.enable_upper_body_ik:
        upper_body_ik = UpperBodyIKRetargeter(
            iterations=args.upper_body_ik_iterations,
            damping=args.upper_body_ik_damping,
            position_weight=args.upper_body_ik_position_weight,
            orientation_weight=args.upper_body_ik_orientation_weight,
            posture_weight=args.upper_body_ik_posture_weight,
            step_size=args.upper_body_ik_step_size,
            max_joint_step=args.upper_body_ik_max_joint_step,
        )
    controls = LineControlSource(auto_start=not args.start_paused)
    visualizer = _create_vr3pt_visualizer(args)

    context = zmq.Context.instance()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:{args.zmq_port}")

    retargeter.preload()
    if upper_body_ik is not None:
        upper_body_ik.preload()
    source.start()
    time.sleep(args.publisher_warmup_s)

    pose_stream_enabled = args.enable_pose_stream or args.control_mode == "pose"
    pose_window_size = _resolve_pose_window_size(args)
    pose_filter = _resolve_pose_filter_settings(args)
    pose_publisher = PoseStreamPublisher(
        window_size=pose_window_size,
        protocol_version=args.pose_protocol_version,
        encoder_mode=_resolve_pose_encoder_mode(args),
        enable_reference_filter=bool(pose_filter["enable_reference_filter"]),
        reference_alpha=float(pose_filter["reference_alpha"]),
        max_smpl_joint_speed_mps=float(pose_filter["max_smpl_joint_speed_mps"]),
        max_smpl_pose_speed_radps=float(pose_filter["max_smpl_pose_speed_radps"]),
        max_root_angular_speed_radps=float(pose_filter["max_root_angular_speed_radps"]),
        max_joint_speed_radps=float(pose_filter["max_joint_speed_radps"]),
        root_tilt_limit_rad=float(pose_filter["root_tilt_limit_rad"]),
        root_yaw_only=args.pose_root_yaw_only,
    )
    pose_bootstrap_enabled = (
        pose_stream_enabled
        and args.control_mode == "pose"
        and not args.no_pose_bootstrap
        and pose_window_size > 1
    )
    control_uses_planner = args.control_mode == "planner"
    current_stream_mode = StreamMode.PLANNER_VR_3PT if control_uses_planner else StreamMode.POSE
    socket.send(
        build_command_message(
            start=(not args.start_paused) and control_uses_planner,
            stop=False,
            planner=control_uses_planner,
        )
    )
    print(
        f"[MocapManager] publishing on tcp://*:{args.zmq_port}; "
        f"control_mode={args.control_mode} pose_stream={int(pose_stream_enabled)} "
        f"pose_protocol=v{args.pose_protocol_version} pose_window={pose_window_size} "
        f"pose_encoder={args.pose_encoder_mode} pose_filter={args.pose_filter_profile} "
        f"root_yaw_only={int(args.pose_root_yaw_only)}; "
        f"{source_description}"
    )
    print("[MocapManager] stdin commands: start, pause, stop, mode N, move x y z, face x y z, dc, abort")

    dt = 1.0 / max(1.0, float(args.target_fps))
    last_log_s = 0.0

    try:
        while True:
            loop_start = time.time()
            control = controls.poll()
            if control.stop_requested:
                socket.send(
                    build_command_message(
                        start=False,
                        stop=True,
                        planner=control_uses_planner,
                    )
                )
                current_stream_mode = StreamMode.OFF
                _send_manager_state(socket, current_stream_mode, False, False)
                break

            frame = source.get_latest()
            vr_position = None
            vr_orientation = None
            upper_body_position = None
            upper_body_velocity = None
            pose_sent = False
            if frame is not None:
                frame_age_s = time.time() - frame.host_time_s
                if frame_age_s <= args.mocap_timeout_s:
                    target = retargeter.build_target(frame)
                    if target is not None:
                        vr_position = target.position
                        vr_orientation = target.orientation
                        if upper_body_ik is not None:
                            upper_body_target = upper_body_ik.build_target(target, frame.host_time_s)
                            if upper_body_target is not None:
                                upper_body_position = upper_body_target.position
                                upper_body_velocity = upper_body_target.velocity
                        if visualizer is not None:
                            vr_pose = _vr_arrays_to_pose(vr_position, vr_orientation)
                            visualizer.update_vr_poses(vr_pose)
                            visualizer.render()
                            if not visualizer.is_open:
                                print("[MocapManager] visualization window closed")
                                socket.send(
                                    build_command_message(
                                        start=False,
                                        stop=True,
                                        planner=control_uses_planner,
                                    )
                                )
                                current_stream_mode = StreamMode.OFF
                                _send_manager_state(socket, current_stream_mode, False, False)
                                break
                    if pose_stream_enabled and frame.full_body is not None:
                        if (
                            pose_bootstrap_enabled
                            and pose_publisher.sent_messages == 0
                            and pose_publisher.buffered_frames == 0
                        ):
                            pose_sent = pose_publisher.publish_bootstrap(
                                socket,
                                frame.full_body,
                                frame_index=frame.frame_index,
                                vr_position=vr_position,
                                vr_orientation=vr_orientation,
                                timestamp_s=frame.host_time_s,
                            )
                        else:
                            pose_sent = pose_publisher.publish(
                                socket,
                                frame.full_body,
                                frame_index=frame.frame_index,
                                vr_position=vr_position,
                                vr_orientation=vr_orientation,
                                timestamp_s=frame.host_time_s,
                            )

            start_allowed = control.enabled and (
                control_uses_planner
                or not pose_stream_enabled
                or pose_publisher.sent_messages > 0
            )
            socket.send(
                build_command_message(
                    start=start_allowed,
                    stop=False,
                    planner=control_uses_planner,
                )
            )

            if control_uses_planner:
                socket.send(
                    build_planner_message(
                        int(control.mode),
                        control.movement.tolist(),
                        control.facing.tolist(),
                        speed=control.speed,
                        height=control.height,
                        upper_body_position=(
                            upper_body_position.tolist() if upper_body_position is not None else None
                        ),
                        upper_body_velocity=(
                            upper_body_velocity.tolist() if upper_body_velocity is not None else None
                        ),
                        left_hand_position=np.zeros(7, dtype=np.float32).tolist(),
                        right_hand_position=np.zeros(7, dtype=np.float32).tolist(),
                        vr_3pt_position=vr_position.tolist() if vr_position is not None else None,
                        vr_3pt_orientation=vr_orientation.tolist() if vr_orientation is not None else None,
                    )
                )

            _send_manager_state(
                socket,
                current_stream_mode,
                control.toggle_data_collection,
                control.toggle_data_abort,
            )

            now = time.time()
            if now - last_log_s > args.log_interval_s:
                diag = source.diagnostics
                frame_desc = "none"
                if frame is not None:
                    source_frame_index = frame.metadata.get("source_frame_index")
                    source_frame_desc = (
                        f" source_frame={source_frame_index}"
                        if source_frame_index is not None and source_frame_index != frame.frame_index
                        else ""
                    )
                    probe_desc = ""
                    if frame.metadata.get("format") == "joint_probe":
                        probe_desc = (
                            f" probe={frame.metadata.get('joint_name')}["
                            f"{frame.metadata.get('joint_index')}]"
                            f"/mj{frame.metadata.get('mujoco_index')}"
                            f" offset={frame.metadata.get('offset_rad', 0.0):+.3f}rad"
                        )
                    frame_desc = (
                        f"frame={frame.frame_index} source_ts={frame.source_time_ns} "
                        f"age={time.time() - frame.host_time_s:.3f}s "
                        f"joints={len(frame.joints)} bones={len(frame.bones)}"
                        f"{source_frame_desc}"
                        f"{probe_desc}"
                    )
                vr_desc = "yes" if vr_position is not None else "no"
                pose_desc = "off"
                if pose_stream_enabled:
                    pose_desc = (
                        f"sent:{pose_publisher.sent_messages}"
                        if pose_sent
                        else f"buf:{pose_publisher.buffered_frames}/{pose_publisher.window_size}"
                    )
                    pose_desc += f" encoder={args.pose_encoder_mode}"
                    pose_metrics = pose_publisher.diagnostics
                    if pose_metrics:
                        pose_desc += (
                            f" q=[{pose_metrics.get('joint_pos_min', 0.0):.2f},"
                            f"{pose_metrics.get('joint_pos_max', 0.0):.2f}]"
                            f" dq_abs={pose_metrics.get('joint_vel_abs_max', 0.0):.2f}"
                            f" wrist_abs={pose_metrics.get('wrist_joint_pos_abs_max', 0.0):.2f}"
                            f" wrist_dq={pose_metrics.get('wrist_joint_vel_abs_max', 0.0):.2f}"
                            f" wrist_margin={pose_metrics.get('wrist_joint_limit_margin_min', 0.0):.2f}"
                            f" body_n={int(pose_metrics.get('body_pos_count', 1.0))}"
                            f" lower_dq={pose_metrics.get('lower_joint_default_delta_abs_max', 0.0):.2f}"
                            f" smpl_lz=[{pose_metrics.get('smpl_lower_z_min', 0.0):.2f},"
                            f"{pose_metrics.get('smpl_lower_z_max', 0.0):.2f}]"
                            f" smpl_lspan={pose_metrics.get('smpl_lower_span_m', 0.0):.2f}m"
                            f" smpl_lpose={pose_metrics.get('smpl_lower_pose_abs_max_rad', 0.0):.2f}rad"
                            f" root_z={pose_metrics.get('root_pos_z_m', 0.0):.3f}m"
                            f" body0_z={pose_metrics.get('body_pos_root_z_m', 0.0):.3f}m"
                            f" root_tilt={pose_metrics.get('root_tilt_rad', 0.0):.2f}rad"
                            f" smpl_lag={pose_metrics.get('pose_smpl_joint_lag_max_m', 0.0):.3f}m"
                            f" pose_lag={pose_metrics.get('pose_smpl_pose_lag_max_rad', 0.0):.2f}rad"
                            f" q_lag={pose_metrics.get('pose_joint_pos_lag_max_rad', 0.0):.2f}rad"
                            f" wrist_lag={pose_metrics.get('pose_wrist_joint_pos_lag_max_rad', 0.0):.2f}rad"
                            f" root_raw={pose_metrics.get('pose_root_tilt_raw_rad', pose_metrics.get('root_tilt_rad', 0.0)):.2f}rad"
                        )
                metrics_desc = ""
                metrics = retargeter.diagnostics
                if vr_position is not None and metrics:
                    metrics_desc = (
                        f" span={metrics.get('wrist_span_m', 0.0):.3f}m"
                        f" head_z={metrics.get('head_height_m', 0.0):.3f}m"
                        f" max_v={metrics.get('max_speed_mps', 0.0):.3f}m/s"
                        f" lag={metrics.get('max_filter_delta_m', 0.0):.3f}m"
                        f" fk={int(metrics.get('fk_calibrated', 0.0))}"
                    )
                ik_desc = ""
                if upper_body_ik is not None:
                    ik_metrics = upper_body_ik.diagnostics
                    if upper_body_position is not None and ik_metrics:
                        ik_desc = (
                            f" ik=1"
                            f" ik_err={ik_metrics.get('upper_body_ik_max_wrist_error_m', 0.0):.3f}m"
                            f" ik_dq={ik_metrics.get('upper_body_ik_max_default_delta_rad', 0.0):.3f}rad"
                            f" ik_active={int(ik_metrics.get('upper_body_ik_active_joint_count', 0.0))}"
                            f" ik_v={ik_metrics.get('upper_body_ik_max_velocity_radps', 0.0):.2f}rad/s"
                            f" ik_margin={ik_metrics.get('upper_body_ik_min_limit_margin_rad', 0.0):.3f}rad"
                        )
                    else:
                        ik_desc = " ik=0"
                print(
                    f"[MocapManager] recv_fps={diag['fps']:.1f} recv={diag['received_packets']} "
                    f"vr_3pt={vr_desc} pose={pose_desc}{metrics_desc}{ik_desc} "
                    f"dropped={diag['dropped_packets']} {frame_desc}"
                )
                if diag["last_error"]:
                    print(f"[MocapManager] last packet error: {diag['last_error']}")
                last_log_s = now

            sleep_s = dt - (time.time() - loop_start)
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        print("\n[MocapManager] stopping")
        socket.send(
            build_command_message(
                start=False,
                stop=True,
                planner=control_uses_planner,
            )
        )
    finally:
        if visualizer is not None:
            visualizer.close()
        source.stop()
        socket.close(0)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a mocap-backed teleop ZMQ manager.")
    parser.add_argument(
        "--source",
        choices=["mocopi", "bvh", "bvh_g1", "bvh_stream", "pkl", "joint_probe"],
        default="mocopi",
    )
    parser.add_argument(
        "--joint-probe-index",
        type=int,
        default=9,
        help=(
            "IsaacLab-order G1 joint index to probe when --source joint_probe. "
            "Default 9 is left_knee."
        ),
    )
    parser.add_argument(
        "--joint-probe-name",
        choices=G1_ISAACLAB_JOINT_NAMES,
        default=None,
        help="IsaacLab-order G1 joint name to probe; overrides --joint-probe-index.",
    )
    parser.add_argument(
        "--joint-probe-amplitude",
        type=float,
        default=0.3,
        help="Single-joint sine amplitude in radians when --source joint_probe.",
    )
    parser.add_argument(
        "--joint-probe-frequency",
        type=float,
        default=0.25,
        help="Single-joint sine frequency in Hz when --source joint_probe.",
    )
    parser.add_argument(
        "--joint-probe-fps",
        type=float,
        default=50.0,
        help="Reference publishing FPS for --source joint_probe.",
    )
    parser.add_argument("--mocopi-host", default="0.0.0.0", help="UDP bind host")
    parser.add_argument("--mocopi-port", type=int, default=MOCOPI_DEFAULT_PORT, help="UDP bind port")
    parser.add_argument(
        "--mocopi-format",
        choices=["auto", "binary", "json"],
        default="auto",
        help="Incoming UDP packet format. JSON is for bridge packets with vr_position/vr_orientation.",
    )
    parser.add_argument(
        "--bvh-stream-host",
        default="0.0.0.0",
        help="UDP bind host for --source bvh_stream.",
    )
    parser.add_argument(
        "--bvh-stream-port",
        type=int,
        default=BVH_STREAM_DEFAULT_PORT,
        help="UDP bind port for --source bvh_stream.",
    )
    parser.add_argument(
        "--bvh-stream-format",
        choices=["auto", "msgpack", "json"],
        default="auto",
        help="Incoming BVH stream packet format.",
    )
    parser.add_argument(
        "--bvh-stream-recv-size",
        type=int,
        default=262144,
        help="Maximum UDP packet size for --source bvh_stream.",
    )
    parser.add_argument("--bvh-file", help="BVH file to replay when --source bvh or bvh_g1")
    parser.add_argument("--bvh-loop", action="store_true", help="Loop BVH playback")
    parser.add_argument(
        "--bvh-fps",
        type=float,
        default=None,
        help="Target BVH playback FPS. Lower values stride through high-FPS BVH files.",
    )
    parser.add_argument(
        "--bvh-unit-scale",
        type=float,
        default=0.01,
        help="Scale BVH position units to meters. Use 0.01 for centimeter BVH files.",
    )
    parser.add_argument(
        "--bvh-no-y-up-to-z-up",
        action="store_true",
        help="Disable BVH Y-up to SONIC Z-up coordinate conversion.",
    )
    parser.add_argument(
        "--bvh-world-frame",
        action="store_true",
        help="Keep BVH global root translation instead of subtracting root position.",
    )
    parser.add_argument(
        "--bvh-g1-method",
        choices=["skeleton", "heuristic"],
        default="skeleton",
        help=(
            "Retargeting method for --source bvh_g1. skeleton projects BVH world "
            "rotations onto the G1 MJCF joints; heuristic keeps the older SMPL-axis-angle gains."
        ),
    )
    parser.add_argument(
        "--bvh-g1-retarget-scale",
        type=float,
        default=1.0,
        help="Global gain for realtime BVH-to-G1 joint retargeting when --source bvh_g1.",
    )
    parser.add_argument(
        "--bvh-g1-lower-scale",
        type=float,
        default=0.60,
        help="Lower-body gain for realtime BVH-to-G1 joint retargeting.",
    )
    parser.add_argument(
        "--bvh-g1-upper-scale",
        type=float,
        default=0.85,
        help="Upper-body gain for realtime BVH-to-G1 joint retargeting.",
    )
    parser.add_argument(
        "--bvh-g1-wrist-scale",
        type=float,
        default=0.55,
        help="Wrist gain for realtime BVH-to-G1 joint retargeting.",
    )
    parser.add_argument(
        "--bvh-g1-waist-scale",
        type=float,
        default=0.25,
        help="Waist gain for realtime BVH-to-G1 joint retargeting.",
    )
    parser.add_argument(
        "--bvh-g1-max-joint-velocity",
        type=float,
        default=6.0,
        help="Clamp realtime BVH-to-G1 joint velocities in rad/s. Use <=0 to disable.",
    )
    parser.add_argument(
        "--bvh-g1-max-joint-step",
        type=float,
        default=0.0,
        help=(
            "Clamp realtime BVH-to-G1 joint position changes in rad/frame. "
            "Defaults to max_joint_velocity / FPS; use <=0 with max velocity <=0 to disable."
        ),
    )
    parser.add_argument(
        "--bvh-g1-joint-filter-alpha",
        type=float,
        default=0.45,
        help=(
            "Optional low-pass alpha for realtime BVH-to-G1 joint targets. "
            "1.0 disables low-pass smoothing; lower values are smoother but laggier."
        ),
    )
    parser.add_argument(
        "--bvh-g1-joint-limit-margin",
        type=float,
        default=0.05,
        help="Keep realtime BVH-to-G1 joint targets this far inside joint limits in rad.",
    )
    parser.add_argument(
        "--bvh-g1-joint-delta-limit-scale",
        type=float,
        default=0.8,
        help=(
            "Scale the per-joint default-pose delta limits used by skeleton retargeting. "
            "Lower is more stable; higher preserves larger BVH motion."
        ),
    )
    parser.add_argument(
        "--bvh-g1-no-skeleton-sign-correction",
        action="store_true",
        help=(
            "Disable the side-specific sign correction used by skeleton retargeting for "
            "mirrored roll/yaw and elbow/wrist joints."
        ),
    )
    parser.add_argument(
        "--bvh-g1-segment-direction",
        choices=("off", "upper", "all"),
        default="all",
        help=(
            "Use BVH joint positions to align G1 limb segment directions before projecting "
            "skeleton rotations. upper affects arms only; all also affects legs."
        ),
    )
    parser.add_argument(
        "--bvh-g1-axis-map",
        choices=("identity", "bvh_y_forward"),
        default="bvh_y_forward",
        help=(
            "Map BVH root-local segment axes into G1 pelvis axes before direction retargeting. "
            "bvh_y_forward maps BVH -Y forward to G1 +X forward."
        ),
    )
    parser.add_argument(
        "--bvh-g1-runtime-mode",
        choices=("online", "precompute"),
        default="online",
        help=(
            "online retargets each BVH frame as it is replayed for low startup latency; "
            "precompute retargets the full BVH before publishing and is useful as a reference."
        ),
    )
    parser.add_argument(
        "--bvh-g1-ik-mode",
        choices=("auto", "numeric", "fast", "analytic", "off"),
        default="auto",
        help=(
            "Limb direction refinement for BVH-to-G1. auto uses analytic IK in online mode "
            "and numeric least-squares in precompute mode. fast is an experimental local-IK path."
        ),
    )
    parser.add_argument(
        "--bvh-g1-root-mode",
        choices=("source", "yaw", "locked"),
        default="yaw",
        help=(
            "Root orientation mode for realtime BVH-to-G1 playback. "
            "yaw keeps heading only, locked freezes the root orientation, source preserves BVH root."
        ),
    )
    parser.add_argument(
        "--bvh-g1-root-max-angular-velocity",
        type=float,
        default=0.0,
        help=(
            "Clamp realtime BVH-to-G1 root angular velocity in rad/s. "
            "Defaults to 0 to match bvh_stream single-frame retargeting; use >0 to smooth root yaw."
        ),
    )
    parser.add_argument(
        "--bvh-g1-root-tilt-limit",
        type=float,
        default=0.25,
        help="Clamp source-mode BVH root tilt in rad before FK. Use <=0 to disable.",
    )
    parser.add_argument(
        "--bvh-g1-min-root-height",
        type=float,
        default=0.74,
        help="Clamp realtime BVH-to-G1 root/body0 height in meters. Use <=0 to disable.",
    )
    parser.add_argument(
        "--bvh-g1-no-align-root",
        action="store_true",
        help="Disable first-frame root alignment for realtime BVH-to-G1 playback.",
    )
    parser.add_argument(
        "--bvh-g1-local-root",
        action="store_true",
        help="Use BVH root-local positions instead of preserving global root translation.",
    )
    parser.add_argument(
        "--bvh-g1-no-body-fk",
        action="store_true",
        help="Disable MJCF FK body_pos14 generation for realtime BVH-to-G1 playback.",
    )
    parser.add_argument(
        "--bvh-g1-smpl-joints-source",
        choices=("g1_fk", "skeleton"),
        default="g1_fk",
        help=(
            "SMPL joints sent in Sony/BVH POSE v3. g1_fk projects the validated v1 G1 FK "
            "keypoints into SMPL slots; skeleton keeps the raw BVH/mocopi skeleton joints."
        ),
    )
    parser.add_argument("--pkl-file", help="Robot-filtered G1 PKL file to replay when --source pkl")
    parser.add_argument("--pkl-loop", action="store_true", help="Loop robot PKL playback")
    parser.add_argument(
        "--pkl-fps",
        type=float,
        default=50.0,
        help="Target robot PKL playback FPS. The IsaacLab mocap path used 50 Hz.",
    )
    parser.add_argument(
        "--pkl-no-align-root",
        action="store_true",
        help="Disable first-frame root yaw/translation alignment for robot PKL playback.",
    )
    parser.add_argument("--zmq-port", type=int, default=5556, help="ZMQ PUB port for deploy side")
    parser.add_argument("--target-fps", type=float, default=20.0)
    parser.add_argument(
        "--control-mode",
        choices=["planner", "pose"],
        default="planner",
        help=(
            "Which deploy mode to select through the command topic. "
            "'planner' keeps PLANNER_VR_3PT behavior; 'pose' selects streamed-motion POSE."
        ),
    )
    parser.add_argument(
        "--enable-pose-stream",
        action="store_true",
        help=(
            "Publish the pose topic when the source provides full_body reference data. "
            "Automatically enabled when --control-mode pose is used."
        ),
    )
    parser.add_argument(
        "--pose-window-size",
        type=int,
        default=None,
        help=(
            "Number of full-body frames per pose topic message. "
            "Defaults to 80 in --control-mode pose and 5 for planner/debug pose streaming."
        ),
    )
    parser.add_argument(
        "--no-pose-bootstrap",
        action="store_true",
        help=(
            "Disable the initial hold-window POSE packet. By default pose mode sends one "
            "bootstrap window as soon as the first full-body frame arrives so deploy can start "
            "before a full real-time window has accumulated."
        ),
    )
    parser.add_argument(
        "--pose-protocol-version",
        type=int,
        choices=[1, 2, 3],
        default=3,
        help=(
            "POSE ZMQ protocol version. Use v1 for retargeted G1 joint PKL mode, "
            "v3 for deploy SMPL mode, and v2 only for debug."
        ),
    )
    parser.add_argument(
        "--allow-sony-pose-v3",
        action="store_true",
        help=(
            "Allow Sony/BVH G1 sources (--source bvh_g1 or bvh_stream) to publish the "
            "experimental SMPL POSE v3 line. Without this flag those sources stay on "
            "the validated POSE v1 + encoder_mode=g1 line."
        ),
    )
    parser.add_argument(
        "--pose-encoder-mode",
        choices=["g1", "smpl", "teleop"],
        default="smpl",
        help=(
            "Encoder observation mode requested from deploy for POSE playback. "
            "'g1' maps to mode 0 for retargeted robot joint PKL, 'smpl' maps to mode 2, "
            "and 'teleop' maps to mode 1 for controlled experiments."
        ),
    )
    parser.add_argument(
        "--allow-teleop-pose-experiment",
        action="store_true",
        help=(
            "Allow --pose-encoder-mode teleop in POSE playback. This path is unstable and should "
            "only be used for controlled A/B experiments."
        ),
    )
    parser.add_argument(
        "--bvh-lower-body-retarget-scale",
        type=float,
        default=None,
        help=(
            "Scale BVH SMPL leg rotations into G1 lower-body joint targets. "
            "Defaults to 0.0; start experiments around 0.1-0.2."
        ),
    )
    parser.add_argument(
        "--no-pose-reference-filter",
        action="store_true",
        help="Disable POSE SMPL/root reference smoothing. Equivalent to --pose-filter-profile off.",
    )
    parser.add_argument(
        "--pose-filter-profile",
        choices=["stable", "responsive", "off"],
        default="stable",
        help=(
            "POSE reference filter preset. 'stable' is conservative, 'responsive' follows BVH "
            "more closely, and 'off' sends raw references."
        ),
    )
    parser.add_argument(
        "--pose-reference-alpha",
        type=float,
        default=None,
        help=(
            "Override POSE reference smoothing alpha in [0, 1]. "
            "Higher values follow mocap more closely."
        ),
    )
    parser.add_argument(
        "--pose-max-smpl-joint-speed",
        type=float,
        default=None,
        help="Override maximum SMPL joint target speed in m/s before smoothing. Use <=0 to disable.",
    )
    parser.add_argument(
        "--pose-max-smpl-pose-speed",
        type=float,
        default=None,
        help=(
            "Override maximum per-joint SMPL axis-angle speed in rad/s before smoothing. "
            "Use <=0 to disable."
        ),
    )
    parser.add_argument(
        "--pose-max-root-angular-speed",
        type=float,
        default=None,
        help="Override maximum root orientation speed in rad/s before smoothing. Use <=0 to disable.",
    )
    parser.add_argument(
        "--pose-max-joint-speed",
        type=float,
        default=None,
        help="Override maximum streamed G1 joint target speed in rad/s before smoothing. Use <=0 to disable.",
    )
    parser.add_argument(
        "--pose-root-tilt-limit",
        type=float,
        default=None,
        help="Override root roll/pitch tilt clamp in radians. Use <=0 to disable.",
    )
    parser.add_argument(
        "--pose-root-yaw-only",
        action="store_true",
        help=(
            "Strip streamed root roll/pitch and keep yaw only for POSE SMPL mode. "
            "Use this to test whether BVH root tilt is destabilizing deploy."
        ),
    )
    parser.add_argument("--start-paused", action="store_true")
    parser.add_argument("--publisher-warmup-s", type=float, default=0.2)
    parser.add_argument("--log-interval-s", type=float, default=2.0)
    parser.add_argument(
        "--mocap-timeout-s",
        type=float,
        default=0.5,
        help="Do not send VR 3-point targets when the latest mocap packet is older than this.",
    )
    parser.add_argument(
        "--allow-bone-translation-vr",
        action="store_true",
        help=(
            "Use mocopi bone translation fields as VR 3-point positions. "
            "This is useful for bridge packets, but official binary packets usually need FK retargeting."
        ),
    )
    parser.add_argument(
        "--no-vr3pt-calibration",
        action="store_true",
        help="Disable first-frame position calibration for named-joint mocap frames.",
    )
    parser.add_argument(
        "--vr3pt-scale",
        type=float,
        default=1.0,
        help="Scale named-joint mocap positions before first-frame calibration.",
    )
    parser.add_argument(
        "--no-vr3pt-fk-calibration",
        action="store_true",
        help="Disable G1 FK-based wrist/head calibration and use position-only calibration.",
    )
    parser.add_argument(
        "--require-vr3pt-fk-calibration",
        action="store_true",
        help="Fail startup if G1 FK-based calibration cannot be loaded.",
    )
    parser.add_argument(
        "--no-vr3pt-filter",
        action="store_true",
        help="Disable VR 3-point smoothing and velocity limiting.",
    )
    parser.add_argument(
        "--vr3pt-position-alpha",
        type=float,
        default=0.45,
        help="Position smoothing alpha in [0, 1]. Higher values follow mocap more closely.",
    )
    parser.add_argument(
        "--vr3pt-orientation-alpha",
        type=float,
        default=0.45,
        help="Quaternion slerp smoothing alpha in [0, 1]. Higher values follow mocap more closely.",
    )
    parser.add_argument(
        "--vr3pt-max-speed",
        type=float,
        default=3.0,
        help="Maximum VR 3-point translation speed in m/s. Use <=0 to disable.",
    )
    parser.add_argument(
        "--vr3pt-max-accel",
        type=float,
        default=25.0,
        help="Maximum VR 3-point translation acceleration in m/s^2. Use <=0 to disable.",
    )
    parser.add_argument(
        "--vr3pt-max-angular-speed",
        type=float,
        default=8.0,
        help="Maximum VR 3-point angular speed in rad/s. Use <=0 to disable.",
    )
    parser.add_argument(
        "--enable-upper-body-ik",
        action="store_true",
        help=(
            "Solve and publish deploy-side upper_body_position/upper_body_velocity "
            "from VR 3-point wrist targets. Disabled by default."
        ),
    )
    parser.add_argument("--upper-body-ik-iterations", type=int, default=8)
    parser.add_argument("--upper-body-ik-damping", type=float, default=0.08)
    parser.add_argument("--upper-body-ik-position-weight", type=float, default=1.0)
    parser.add_argument("--upper-body-ik-orientation-weight", type=float, default=0.15)
    parser.add_argument("--upper-body-ik-posture-weight", type=float, default=0.03)
    parser.add_argument("--upper-body-ik-step-size", type=float, default=0.7)
    parser.add_argument(
        "--upper-body-ik-max-joint-step",
        type=float,
        default=0.08,
        help="Maximum upper-body IK joint update per solver iteration, in radians.",
    )
    parser.add_argument(
        "--visualize-vr3pt",
        action="store_true",
        help="Open a PyVista window that renders the generated VR 3-point targets.",
    )
    parser.add_argument(
        "--visualize-g1",
        action="store_true",
        help="Also render the G1 mesh in the VR 3-point visualization window.",
    )
    parser.add_argument("--visualize-width", type=int, default=1400)
    parser.add_argument("--visualize-height", type=int, default=900)
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    user_set_target_fps = _has_cli_option("--target-fps")
    user_set_pose_filter_profile = _has_cli_option("--pose-filter-profile")
    _validate_args(
        parser,
        args,
        user_set_target_fps=user_set_target_fps,
        user_set_pose_filter_profile=user_set_pose_filter_profile,
    )
    try:
        run_mocap_manager(args)
    except ValueError as exc:
        parser.error(str(exc))


def _validate_args(
    parser: argparse.ArgumentParser,
    args: argparse.Namespace,
    *,
    user_set_target_fps: bool,
    user_set_pose_filter_profile: bool,
) -> None:
    if args.source in {"pkl", "bvh_g1", "bvh_stream"}:
        if args.control_mode != "pose":
            parser.error(f"--source {args.source} requires --control-mode pose")
        if not user_set_target_fps:
            if args.source == "pkl":
                args.target_fps = args.pkl_fps
            elif args.source == "bvh_g1":
                args.target_fps = args.bvh_fps or 50.0
            else:
                args.target_fps = 50.0
    if args.source == "pkl":
        if args.pose_protocol_version != 1:
            parser.error("--source pkl requires --pose-protocol-version 1")
        if args.pose_encoder_mode != "g1":
            parser.error("--source pkl requires --pose-encoder-mode g1")
        if not user_set_pose_filter_profile:
            args.pose_filter_profile = "off"
    if args.source in {"bvh_g1", "bvh_stream"}:
        uses_g1_v1 = args.pose_protocol_version == 1 and args.pose_encoder_mode == "g1"
        uses_smpl_v3 = args.pose_protocol_version == 3 and args.pose_encoder_mode == "smpl"
        if uses_g1_v1:
            if not user_set_pose_filter_profile:
                args.pose_filter_profile = "off"
        elif uses_smpl_v3:
            if not args.allow_sony_pose_v3:
                parser.error(
                    f"--source {args.source} with POSE v3/smpl is experimental; "
                    "add --allow-sony-pose-v3 to keep the Sony v1 and v3 lines explicit"
                )
            if args.bvh_g1_smpl_joints_source == "g1_fk" and args.bvh_g1_no_body_fk:
                parser.error(
                    "--bvh-g1-smpl-joints-source g1_fk requires body FK; remove "
                    "--bvh-g1-no-body-fk or use --bvh-g1-smpl-joints-source skeleton"
                )
        else:
            parser.error(
                f"--source {args.source} supports either "
                "--pose-protocol-version 1 --pose-encoder-mode g1, or "
                "--pose-protocol-version 3 --pose-encoder-mode smpl --allow-sony-pose-v3"
            )
    if args.pose_encoder_mode == "teleop" and args.pose_protocol_version != 3:
        parser.error("--pose-encoder-mode teleop requires --pose-protocol-version 3")
    if args.pose_encoder_mode == "teleop" and not args.allow_teleop_pose_experiment:
        parser.error(
            "--pose-encoder-mode teleop is unstable for POSE playback; "
            "add --allow-teleop-pose-experiment only for controlled experiments"
        )
    if args.pose_encoder_mode == "g1" and args.pose_protocol_version != 1:
        parser.error("--pose-encoder-mode g1 requires --pose-protocol-version 1")


def _has_cli_option(option: str) -> bool:
    return any(arg == option or arg.startswith(f"{option}=") for arg in sys.argv[1:])


if __name__ == "__main__":
    main()
