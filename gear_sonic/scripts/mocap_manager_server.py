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
from gear_sonic.utils.teleop.sources import MOCOPI_DEFAULT_PORT, MocapFrame, MocopiUdpSource, Pose7D
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
    build_command_message,
    build_planner_message,
    pack_pose_message,
)


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


def _find_joint(frame: MocapFrame, aliases: tuple[str, ...]) -> Pose7D | None:
    for alias in aliases:
        pose = frame.joints.get(alias)
        if pose is not None:
            return pose
    return None


def build_vr_3pt_from_frame(
    frame: MocapFrame,
    allow_bone_translation_vr: bool = False,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Extract robot-ready VR 3-point arrays from a canonical mocap frame."""
    if frame.direct_vr_position is not None:
        orientation = (
            frame.direct_vr_orientation
            if frame.direct_vr_orientation is not None
            else DEFAULT_VR_ORIENTATION.copy()
        )
        return frame.direct_vr_position.copy(), orientation.copy()

    left = _find_joint(frame, ("left_wrist", "left_hand", "l_hand", "left_controller"))
    right = _find_joint(frame, ("right_wrist", "right_hand", "r_hand", "right_controller"))
    head = _find_joint(frame, ("head", "neck", "neck_2", "head_tracker"))

    if allow_bone_translation_vr and (left is None or right is None):
        left = left or frame.bones.get(14)
        right = right or frame.bones.get(18)
        head = head or frame.bones.get(10) or frame.bones.get(9)

    if left is None or right is None:
        return None, None

    position = DEFAULT_VR_POSITION.copy()
    orientation = DEFAULT_VR_ORIENTATION.copy()
    position[:3] = left.position
    position[3:6] = right.position
    orientation[:4] = left.quat_wxyz
    orientation[4:8] = right.quat_wxyz

    if head is not None:
        position[6:9] = head.position
        orientation[8:12] = head.quat_wxyz

    return position, orientation


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


def run_mocap_manager(args: argparse.Namespace) -> None:
    source = MocopiUdpSource(
        bind_host=args.mocopi_host,
        port=args.mocopi_port,
        packet_format=args.mocopi_format,
    )
    controls = LineControlSource(auto_start=not args.start_paused)

    context = zmq.Context.instance()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:{args.zmq_port}")

    source.start()
    time.sleep(args.publisher_warmup_s)

    current_stream_mode = StreamMode.PLANNER_VR_3PT
    socket.send(build_command_message(start=not args.start_paused, stop=False, planner=True))
    print(
        f"[MocapManager] publishing planner data on tcp://*:{args.zmq_port}; "
        f"listening for mocopi UDP on {args.mocopi_host}:{args.mocopi_port} ({args.mocopi_format})"
    )
    print("[MocapManager] stdin commands: start, pause, stop, mode N, move x y z, face x y z, dc, abort")

    dt = 1.0 / max(1.0, float(args.target_fps))
    last_log_s = 0.0

    try:
        while True:
            loop_start = time.time()
            control = controls.poll()
            if control.stop_requested:
                socket.send(build_command_message(start=False, stop=True, planner=True))
                current_stream_mode = StreamMode.OFF
                _send_manager_state(socket, current_stream_mode, False, False)
                break

            if control.enabled:
                socket.send(build_command_message(start=True, stop=False, planner=True))
            else:
                socket.send(build_command_message(start=False, stop=False, planner=True))

            frame = source.get_latest()
            vr_position = None
            vr_orientation = None
            if frame is not None:
                frame_age_s = time.time() - frame.host_time_s
                if frame_age_s <= args.mocap_timeout_s:
                    vr_position, vr_orientation = build_vr_3pt_from_frame(
                        frame,
                        allow_bone_translation_vr=args.allow_bone_translation_vr,
                    )

            socket.send(
                build_planner_message(
                    int(control.mode),
                    control.movement.tolist(),
                    control.facing.tolist(),
                    speed=control.speed,
                    height=control.height,
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
                    frame_desc = (
                        f"frame={frame.frame_index} source_ts={frame.source_time_ns} "
                        f"age={time.time() - frame.host_time_s:.3f}s "
                        f"joints={len(frame.joints)} bones={len(frame.bones)}"
                    )
                vr_desc = "yes" if vr_position is not None else "no"
                print(
                    f"[MocapManager] recv_fps={diag['fps']:.1f} recv={diag['received_packets']} "
                    f"vr_3pt={vr_desc} "
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
        socket.send(build_command_message(start=False, stop=True, planner=True))
    finally:
        source.stop()
        socket.close(0)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a mocap-backed teleop ZMQ manager.")
    parser.add_argument("--source", choices=["mocopi"], default="mocopi")
    parser.add_argument("--mocopi-host", default="0.0.0.0", help="UDP bind host")
    parser.add_argument("--mocopi-port", type=int, default=MOCOPI_DEFAULT_PORT, help="UDP bind port")
    parser.add_argument(
        "--mocopi-format",
        choices=["auto", "binary", "json"],
        default="auto",
        help="Incoming UDP packet format. JSON is for bridge packets with vr_position/vr_orientation.",
    )
    parser.add_argument("--zmq-port", type=int, default=5556, help="ZMQ PUB port for deploy side")
    parser.add_argument("--target-fps", type=float, default=20.0)
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
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    if args.source != "mocopi":
        parser.error(f"unsupported source {args.source!r}")
    run_mocap_manager(args)


if __name__ == "__main__":
    main()
