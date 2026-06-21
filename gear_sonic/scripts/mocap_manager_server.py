"""Generic mocap teleop manager for non-PICO input sources.

This entry point publishes the same ZMQ topics consumed by the deploy side:
`command`, `planner`, and `manager_state`. It is intentionally separate from
`pico_manager_thread_server.py` so new motion-capture sources can be validated
without changing the PICO/XR control path.
"""

from __future__ import annotations

import argparse
import time
from enum import IntEnum

import numpy as np
import zmq

from gear_sonic.utils.teleop.controls import LineControlSource
from gear_sonic.utils.teleop.retarget import UpperBodyIKRetargeter, VR3PointRetargeter
from gear_sonic.utils.teleop.sources import (
    MOCOPI_DEFAULT_PORT,
    BvhPlaybackSource,
    MocopiUdpSource,
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


def _create_source(args: argparse.Namespace):
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
        source = BvhPlaybackSource(
            bvh_file=args.bvh_file,
            target_fps=args.bvh_fps,
            loop=args.bvh_loop,
            unit_scale=args.bvh_unit_scale,
            y_up_to_z_up=not args.bvh_no_y_up_to_z_up,
            body_local=not args.bvh_world_frame,
        )
        description = (
            f"playing BVH {args.bvh_file} at {source.motion.playback_fps:.1f} Hz "
            f"(source_fps={source.motion.source_fps:.1f}, stride={source.motion.frame_stride}, "
            f"loop={args.bvh_loop})"
        )
        return source, description

    raise ValueError(f"unsupported source {args.source!r}")


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
    pose_publisher = PoseStreamPublisher(
        window_size=pose_window_size,
        protocol_version=args.pose_protocol_version,
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
        f"pose_protocol=v{args.pose_protocol_version} pose_window={pose_window_size}; "
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
                        pose_sent = pose_publisher.publish(
                            socket,
                            frame.full_body,
                            frame_index=frame.frame_index,
                            vr_position=vr_position,
                            vr_orientation=vr_orientation,
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
                    frame_desc = (
                        f"frame={frame.frame_index} source_ts={frame.source_time_ns} "
                        f"age={time.time() - frame.host_time_s:.3f}s "
                        f"joints={len(frame.joints)} bones={len(frame.bones)}"
                        f"{source_frame_desc}"
                    )
                vr_desc = "yes" if vr_position is not None else "no"
                pose_desc = "off"
                if pose_stream_enabled:
                    pose_desc = (
                        f"sent:{pose_publisher.sent_messages}"
                        if pose_sent
                        else f"buf:{pose_publisher.buffered_frames}/{pose_publisher.window_size}"
                    )
                    pose_metrics = pose_publisher.diagnostics
                    if pose_metrics:
                        pose_desc += (
                            f" q=[{pose_metrics.get('joint_pos_min', 0.0):.2f},"
                            f"{pose_metrics.get('joint_pos_max', 0.0):.2f}]"
                            f" dq_abs={pose_metrics.get('joint_vel_abs_max', 0.0):.2f}"
                            f" lower_dq={pose_metrics.get('lower_joint_default_delta_abs_max', 0.0):.2f}"
                            f" smpl_lz=[{pose_metrics.get('smpl_lower_z_min', 0.0):.2f},"
                            f"{pose_metrics.get('smpl_lower_z_max', 0.0):.2f}]"
                            f" smpl_lspan={pose_metrics.get('smpl_lower_span_m', 0.0):.2f}m"
                            f" smpl_lpose={pose_metrics.get('smpl_lower_pose_abs_max_rad', 0.0):.2f}rad"
                            f" root_tilt={pose_metrics.get('root_tilt_rad', 0.0):.2f}rad"
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
    parser.add_argument("--source", choices=["mocopi", "bvh"], default="mocopi")
    parser.add_argument("--mocopi-host", default="0.0.0.0", help="UDP bind host")
    parser.add_argument("--mocopi-port", type=int, default=MOCOPI_DEFAULT_PORT, help="UDP bind port")
    parser.add_argument(
        "--mocopi-format",
        choices=["auto", "binary", "json"],
        default="auto",
        help="Incoming UDP packet format. JSON is for bridge packets with vr_position/vr_orientation.",
    )
    parser.add_argument("--bvh-file", help="BVH file to replay when --source bvh")
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
        "--pose-protocol-version",
        type=int,
        choices=[2, 3],
        default=3,
        help="POSE ZMQ protocol version. Use v3 for deploy SMPL mode; v2 is debug-only.",
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
    try:
        run_mocap_manager(args)
    except ValueError as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
