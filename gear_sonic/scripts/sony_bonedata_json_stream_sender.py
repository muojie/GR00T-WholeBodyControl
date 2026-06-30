"""Stream Sony mocopi saveBoneData JSON frames as bvh_stream_v1 UDP packets."""

from __future__ import annotations

import argparse
import json
import math
import os.path as osp
import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import msgpack

from gear_sonic.utils.teleop.sources.bvh_stream_source import (
    BVH_STREAM_DEFAULT_PORT,
    BVH_STREAM_FORMAT,
)


DEFAULT_JOINTS_PER_FRAME = 27


@dataclass(frozen=True)
class SonyBoneDataMotion:
    path: str
    joint_names: list[str]
    world_positions: list[list[list[float]]]
    world_quat_wxyz: list[list[list[float]]]
    source_fps: float
    playback_fps: float

    @property
    def frame_count(self) -> int:
        return len(self.world_positions)


def _read_vec3(value: Any, *, position_scale: float) -> list[float]:
    if isinstance(value, dict):
        return [
            float(value["x"]) * position_scale,
            float(value["y"]) * position_scale,
            float(value["z"]) * position_scale,
        ]
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return [
            float(value[0]) * position_scale,
            float(value[1]) * position_scale,
            float(value[2]) * position_scale,
        ]
    raise ValueError(f"position must be dict x/y/z or length-3 list, got {value!r}")


def _normalize_quat_wxyz(quat: list[float]) -> list[float]:
    norm = math.sqrt(sum(float(v) * float(v) for v in quat))
    if norm < 1e-8 or not math.isfinite(norm):
        return [1.0, 0.0, 0.0, 0.0]
    return [float(v) / norm for v in quat]


def _read_quat_wxyz(value: Any, *, input_quat_order: str) -> list[float]:
    if isinstance(value, dict):
        quat = [
            float(value["w"]),
            float(value["x"]),
            float(value["y"]),
            float(value["z"]),
        ]
        return _normalize_quat_wxyz(quat)

    if isinstance(value, (list, tuple)) and len(value) >= 4:
        raw = [float(value[0]), float(value[1]), float(value[2]), float(value[3])]
        if input_quat_order == "xyzw":
            quat = [raw[3], raw[0], raw[1], raw[2]]
        elif input_quat_order == "wxyz":
            quat = raw
        else:
            raise ValueError(f"unsupported input quaternion order {input_quat_order!r}")
        return _normalize_quat_wxyz(quat)

    raise ValueError(f"rotation must be dict x/y/z/w or length-4 list, got {value!r}")


def load_sony_bonedata_json(
    json_file: Path,
    *,
    joints_per_frame: int,
    position_scale: float,
    input_quat_order: str,
    source_fps: float,
    playback_fps: float,
    local_root: bool,
) -> SonyBoneDataMotion:
    with json_file.open("r", encoding="utf-8-sig") as file:
        data = json.load(file)

    if not isinstance(data, dict):
        raise ValueError(f"{json_file} must contain a JSON object")
    for key in ("name", "position", "rotation"):
        if key not in data:
            raise ValueError(f"{json_file} missing required field {key!r}")

    names = data["name"]
    positions = data["position"]
    rotations = data["rotation"]
    if not isinstance(names, list) or not isinstance(positions, list) or not isinstance(rotations, list):
        raise ValueError("name, position, and rotation must all be lists")
    if len(names) != len(positions) or len(names) != len(rotations):
        raise ValueError(
            "name, position, and rotation must have the same length; "
            f"got {len(names)}, {len(positions)}, {len(rotations)}"
        )
    if joints_per_frame <= 0:
        raise ValueError("--joints-per-frame must be positive")
    if len(names) == 0 or len(names) % joints_per_frame != 0:
        raise ValueError(
            f"name length {len(names)} is not divisible by joints_per_frame={joints_per_frame}"
        )

    frame_count = len(names) // joints_per_frame
    joint_names = [str(name) for name in names[:joints_per_frame]]
    world_positions: list[list[list[float]]] = []
    world_quat_wxyz: list[list[list[float]]] = []

    for frame_idx in range(frame_count):
        start = frame_idx * joints_per_frame
        end = start + joints_per_frame
        frame_names = [str(name) for name in names[start:end]]
        if frame_names != joint_names:
            raise ValueError(
                f"joint names changed at frame {frame_idx}; "
                "bvh_stream_v1 expects stable joint order within one sender session"
            )

        frame_positions = [
            _read_vec3(value, position_scale=position_scale) for value in positions[start:end]
        ]
        if local_root:
            root = frame_positions[0]
            frame_positions = [
                [pos[0] - root[0], pos[1] - root[1], pos[2] - root[2]]
                for pos in frame_positions
            ]
        frame_quats = [
            _read_quat_wxyz(value, input_quat_order=input_quat_order)
            for value in rotations[start:end]
        ]
        world_positions.append(frame_positions)
        world_quat_wxyz.append(frame_quats)

    return SonyBoneDataMotion(
        path=str(json_file),
        joint_names=joint_names,
        world_positions=world_positions,
        world_quat_wxyz=world_quat_wxyz,
        source_fps=float(source_fps),
        playback_fps=float(playback_fps),
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
        description="Stream Sony mocopi saveBoneData JSON as bvh_stream_v1 UDP frames."
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
        default=DEFAULT_JOINTS_PER_FRAME,
        help="Number of bones per frame in the flat name/position/rotation arrays.",
    )
    parser.add_argument(
        "--position-scale",
        type=float,
        default=1.0,
        help="Scale positions before streaming. Use 1.0 when JSON is already meters.",
    )
    parser.add_argument(
        "--input-quat-order",
        choices=("xyzw", "wxyz"),
        default="xyzw",
        help="Quaternion order for list rotations. Dict rotations with x/y/z/w ignore this.",
    )
    parser.add_argument(
        "--local-root",
        action="store_true",
        help="Subtract each frame's root position from all joints before streaming.",
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
    motion = load_sony_bonedata_json(
        json_file,
        joints_per_frame=args.joints_per_frame,
        position_scale=args.position_scale,
        input_quat_order=args.input_quat_order,
        source_fps=source_fps,
        playback_fps=args.fps,
        local_root=args.local_root,
    )

    frame_period_s = 1.0 / max(1.0, motion.playback_fps)
    motion_name = args.motion_name or osp.splitext(osp.basename(motion.path))[0]
    address = (args.host, int(args.port))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(
        f"[SonyBoneDataJsonStreamSender] streaming {motion.path} to "
        f"udp://{args.host}:{args.port} format={args.format} fps={motion.playback_fps:.1f} "
        f"source_fps={motion.source_fps:.1f} frames={motion.frame_count} "
        f"joints={len(motion.joint_names)} loop={args.loop} "
        f"position_scale={args.position_scale:g} local_root={args.local_root}"
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
                "motion_name": motion_name,
                "joint_names": motion.joint_names,
                "frame_index": int(stream_frame_idx),
                "source_frame_index": int(frame_idx),
                "source_fps": float(motion.source_fps),
                "fps": float(motion.playback_fps),
                "frame_stride": 1,
                "source_time_ns": time.time_ns(),
                "world_positions": motion.world_positions[frame_idx],
                "world_quat_wxyz": motion.world_quat_wxyz[frame_idx],
            }
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
