"""Stream Sony mocopi saveBoneData JSON frames as raw UDP packets."""

from __future__ import annotations

import argparse
import json
import os.path as osp
import socket
import time
from pathlib import Path
from typing import Any

import msgpack

from gear_sonic.utils.teleop.sources.bvh_stream_source import BVH_STREAM_DEFAULT_PORT
from gear_sonic.utils.teleop.sources.sony_bonedata_json import (
    SONY_BONEDATA_DEFAULT_JOINTS_PER_FRAME,
    SONY_BONEDATA_JSON_FORMAT,
    load_sony_bonedata_json_raw,
)


def _pack_payload(payload: dict[str, Any], packet_format: str) -> bytes:
    payload = dict(payload)
    payload["packet_format"] = packet_format
    if packet_format == "msgpack":
        return msgpack.packb(payload, use_bin_type=True)
    if packet_format == "json":
        return json.dumps(payload, separators=(",", ":")).encode("utf-8")
    raise ValueError(f"unsupported packet format {packet_format!r}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Stream Sony mocopi saveBoneData JSON as raw sony_bonedata_json_v1 UDP frames. "
            "Coordinate-frame and quaternion conversion are handled by the receiver."
        )
    )
    parser.add_argument("--json-file", type=Path, required=True, help="saveBoneData JSON file")
    parser.add_argument("--host", default="127.0.0.1", help="Destination host")
    parser.add_argument("--port", type=int, default=BVH_STREAM_DEFAULT_PORT, help="Destination UDP port")
    parser.add_argument(
        "--format",
        choices=("msgpack", "json"),
        default="msgpack",
        help="Packet encoding. msgpack is recommended; json is useful for packet inspection.",
    )
    parser.add_argument("--fps", type=float, default=50.0, help="Playback/send FPS")
    parser.add_argument(
        "--source-fps",
        type=float,
        default=None,
        help="Original capture FPS. Defaults to --fps.",
    )
    parser.add_argument("--loop", action="store_true", help="Loop after the last JSON frame")
    parser.add_argument(
        "--joints-per-frame",
        type=int,
        default=SONY_BONEDATA_DEFAULT_JOINTS_PER_FRAME,
        help="Number of bones per frame in the flat name/position/rotation arrays.",
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
    parser.add_argument(
        "--motion-name",
        default=None,
        help="Diagnostic motion name. Defaults to the JSON file stem.",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    json_file = args.json_file.expanduser().resolve()
    source_fps = float(args.source_fps if args.source_fps is not None else args.fps)
    motion = load_sony_bonedata_json_raw(
        json_file,
        joints_per_frame=args.joints_per_frame,
        source_fps=source_fps,
        playback_fps=args.fps,
    )

    frame_period_s = 1.0 / max(1.0, motion.playback_fps)
    motion_name = args.motion_name or osp.splitext(osp.basename(motion.path))[0]
    address = (args.host, int(args.port))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(
        f"[SonyBoneDataJsonStreamSender] streaming raw {motion.path} to "
        f"udp://{args.host}:{args.port} format={args.format} "
        f"payload_format={SONY_BONEDATA_JSON_FORMAT} fps={motion.playback_fps:.1f} "
        f"source_fps={motion.source_fps:.1f} frames={motion.frame_count} "
        f"joints={len(motion.joint_names)} loop={args.loop}"
    )
    if args.startup_delay_s > 0.0:
        time.sleep(args.startup_delay_s)

    frame_idx = 0
    stream_frame_idx = 0
    next_tick_s = time.time()
    last_log_s = 0.0
    try:
        while True:
            payload = motion.frame_payload(
                frame_idx,
                stream_frame_idx=stream_frame_idx,
                motion_name=motion_name,
            )
            payload["source_time_ns"] = time.time_ns()
            packet = _pack_payload(payload, args.format)
            sock.sendto(packet, address)

            stream_frame_idx += 1
            if args.max_frames > 0 and stream_frame_idx >= args.max_frames:
                break

            frame_idx += 1
            if frame_idx >= motion.frame_count:
                if not args.loop:
                    break
                frame_idx = 0

            now = time.time()
            if args.log_interval_s > 0.0 and now - last_log_s >= args.log_interval_s:
                print(
                    f"[SonyBoneDataJsonStreamSender] sent={stream_frame_idx} "
                    f"source_frame={frame_idx} bytes={len(packet)}"
                )
                last_log_s = now

            next_tick_s += frame_period_s
            sleep_s = next_tick_s - time.time()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick_s = time.time()
    except KeyboardInterrupt:
        print("\n[SonyBoneDataJsonStreamSender] stopping")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
