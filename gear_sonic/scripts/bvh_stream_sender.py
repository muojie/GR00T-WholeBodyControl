"""Stream decoded BVH skeleton frames over UDP for mocap manager testing."""

from __future__ import annotations

import argparse
import json
import os.path as osp
import socket
import time
from typing import Any

import msgpack
import msgpack_numpy as mnp

from gear_sonic.utils.teleop.sources.bvh_source import load_bvh_motion
from gear_sonic.utils.teleop.sources.bvh_stream_source import (
    BVH_STREAM_DEFAULT_PORT,
    BVH_STREAM_FORMAT,
)


def _pack_payload(payload: dict[str, Any], packet_format: str) -> bytes:
    if packet_format == "msgpack":
        payload = dict(payload)
        payload["packet_format"] = "msgpack"
        return msgpack.packb(payload, default=mnp.encode, use_bin_type=True)
    if packet_format == "json":
        payload = dict(payload)
        payload["packet_format"] = "json"
        payload["world_positions"] = payload["world_positions"].tolist()
        payload["world_quat_wxyz"] = payload["world_quat_wxyz"].tolist()
        return json.dumps(payload, separators=(",", ":")).encode("utf-8")
    raise ValueError(f"unsupported packet format {packet_format!r}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Send a BVH skeleton as realtime UDP frames.")
    parser.add_argument("--bvh-file", required=True, help="BVH file to stream")
    parser.add_argument("--host", default="127.0.0.1", help="Destination host")
    parser.add_argument("--port", type=int, default=BVH_STREAM_DEFAULT_PORT, help="Destination UDP port")
    parser.add_argument(
        "--format",
        choices=("msgpack", "json"),
        default="msgpack",
        help="Packet encoding. msgpack is compact; json is useful for manual inspection.",
    )
    parser.add_argument("--loop", action="store_true", help="Loop after the last BVH frame")
    parser.add_argument(
        "--fps",
        type=float,
        default=None,
        help="Target stream FPS. Lower values stride through high-FPS BVH files.",
    )
    parser.add_argument(
        "--unit-scale",
        type=float,
        default=0.01,
        help="Scale BVH position units to meters. Use 0.01 for centimeter BVH files.",
    )
    parser.add_argument(
        "--no-y-up-to-z-up",
        action="store_true",
        help="Disable BVH Y-up to SONIC Z-up coordinate conversion before streaming.",
    )
    parser.add_argument(
        "--local-root",
        action="store_true",
        help="Subtract root translation before streaming. Default preserves BVH global root motion.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Stop after sending this many stream frames. Use 0 for no explicit limit.",
    )
    parser.add_argument(
        "--startup-delay-s",
        type=float,
        default=0.0,
        help="Sleep before sending the first packet so the receiver can bind.",
    )
    parser.add_argument(
        "--log-interval-s",
        type=float,
        default=1.0,
        help="Progress log interval in seconds. Use <=0 to disable.",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    motion = load_bvh_motion(
        args.bvh_file,
        target_fps=args.fps,
        unit_scale=args.unit_scale,
        y_up_to_z_up=not args.no_y_up_to_z_up,
        body_local=args.local_root,
        lower_body_retarget_scale=0.0,
    )
    frame_period_s = 1.0 / max(1.0, motion.playback_fps)
    address = (args.host, int(args.port))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(
        f"[BvhStreamSender] streaming {motion.path} to udp://{args.host}:{args.port} "
        f"format={args.format} fps={motion.playback_fps:.1f} source_fps={motion.source_fps:.1f} "
        f"stride={motion.frame_stride} frames={motion.frame_count} loop={args.loop}"
    )
    if args.startup_delay_s > 0.0:
        time.sleep(args.startup_delay_s)

    frame_idx = 0
    stream_frame_idx = 0
    next_tick_s = time.time()
    last_log_s = 0.0
    try:
        while True:
            payload = {
                "format": BVH_STREAM_FORMAT,
                "schema_version": 1,
                "path": motion.path,
                "motion_name": osp.splitext(osp.basename(motion.path))[0],
                "joint_names": motion.joint_names,
                "frame_index": int(stream_frame_idx),
                "source_frame_index": int(frame_idx),
                "source_fps": float(motion.source_fps),
                "fps": float(motion.playback_fps),
                "frame_stride": int(motion.frame_stride),
                "source_time_ns": time.time_ns(),
                "world_positions": motion.world_positions[frame_idx],
                "world_quat_wxyz": motion.world_quat_wxyz[frame_idx],
            }
            packet = _pack_payload(payload, args.format)
            sock.sendto(packet, address)

            stream_frame_idx += 1
            if args.max_frames > 0 and stream_frame_idx >= args.max_frames:
                break

            frame_idx += motion.frame_stride
            if frame_idx >= motion.frame_count:
                if not args.loop:
                    break
                frame_idx = frame_idx % motion.frame_count

            now = time.time()
            if args.log_interval_s > 0.0 and now - last_log_s >= args.log_interval_s:
                print(
                    f"[BvhStreamSender] sent={stream_frame_idx} source_frame={frame_idx} "
                    f"bytes={len(packet)}"
                )
                last_log_s = now

            next_tick_s += frame_period_s
            sleep_s = next_tick_s - time.time()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick_s = time.time()
    except KeyboardInterrupt:
        print("\n[BvhStreamSender] stopping")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
