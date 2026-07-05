"""Send Sony mocopi saveBoneData JSON frames to pico_manager_thread_server over UDP."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import time
from typing import Any

from gear_sonic.utils.teleop import input_readers


def _load_bonedata(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"BoneData JSON root must be an object: {path}")
    return data


def _source_fps(data: dict[str, Any], override: float | None) -> float:
    if override is not None:
        return float(override)
    for key in ("playbackFps", "playback_fps", "fps", "source_fps"):
        value = data.get(key)
        if value is not None:
            return float(value)
    return 50.0


def _frame_count(data: dict[str, Any]) -> int:
    names = data.get("name")
    positions = data.get("position")
    rotations = data.get("rotation")
    if not isinstance(names, list) or not names:
        raise ValueError("BoneData JSON requires non-empty list field 'name'")
    if not isinstance(positions, list) or not isinstance(rotations, list):
        raise ValueError("BoneData JSON requires list fields 'position' and 'rotation'")
    joint_count = len(names)
    return min(len(positions), len(rotations)) // joint_count


def _build_payload(
    data: dict[str, Any],
    *,
    json_file: Path,
    source_frame_index: int,
    stream_frame_index: int,
    send_fps: float,
    source_fps: float,
) -> dict[str, Any]:
    names = data["name"]
    joint_count = len(names)
    start = source_frame_index * joint_count
    end = start + joint_count
    return {
        "format": input_readers.SONY_BONEDATA_JSON_FORMAT,
        "name": names,
        "position": data["position"][start:end],
        "rotation": data["rotation"][start:end],
        "frame_index": stream_frame_index,
        "source_frame_index": source_frame_index,
        "timestamp_ns": int(stream_frame_index * 1_000_000_000 / send_fps),
        "source_time_ns": int(source_frame_index * 1_000_000_000 / source_fps),
        "fps": send_fps,
        "source_fps": source_fps,
        "path": str(json_file),
        "motion_name": data.get("motionName", data.get("name_id", json_file.stem)),
    }


def _encode_payload(payload: dict[str, Any], packet_format: str) -> bytes:
    if packet_format == "json":
        return json.dumps(payload, separators=(",", ":")).encode("utf-8")

    try:
        import msgpack
    except ImportError as exc:
        raise ImportError("msgpack is required for --format msgpack") from exc
    return msgpack.packb(payload, use_bin_type=True)


def _selected_frames(args: argparse.Namespace, frame_count: int, source_fps: float) -> range:
    start = max(0, int(args.start_frame))
    end = int(args.end_frame) if args.end_frame else frame_count
    if args.end_time_s is not None:
        end = min(end, start + int(float(args.end_time_s) * source_fps))
    end = min(frame_count, max(start, end))
    stride = max(1, int(args.frame_stride))
    if start >= end:
        raise ValueError(f"empty frame range: start={start}, end={end}, frame_count={frame_count}")
    return range(start, end, stride)


def _send_frame(
    sock: socket.socket,
    target: tuple[str, int],
    data: dict[str, Any],
    *,
    json_file: Path,
    source_frame_index: int,
    stream_frame_index: int,
    send_fps: float,
    source_fps: float,
    packet_format: str,
) -> int:
    payload = _build_payload(
        data,
        json_file=json_file,
        source_frame_index=source_frame_index,
        stream_frame_index=stream_frame_index,
        send_fps=send_fps,
        source_fps=source_fps,
    )
    packet = _encode_payload(payload, packet_format)
    sock.sendto(packet, target)
    return len(packet)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json-file", type=Path, required=True, help="Path to saveBoneData JSON")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Pico Manager UDP host")
    parser.add_argument(
        "--port",
        type=int,
        default=input_readers.SONY_BONEDATA_DEFAULT_PORT,
        help=f"Pico Manager UDP port (default: {input_readers.SONY_BONEDATA_DEFAULT_PORT})",
    )
    parser.add_argument("--format", choices=["msgpack", "json"], default="msgpack")
    parser.add_argument("--fps", type=float, default=None, help="Send FPS; defaults to source FPS")
    parser.add_argument("--source-fps", type=float, default=None, help="Override source JSON FPS")
    parser.add_argument("--loop", action="store_true", help="Loop selected frames")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N sent frames; 0 means no cap")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--end-frame", type=int, default=0, help="Exclusive end frame; 0 means all")
    parser.add_argument("--end-time-s", type=float, default=None, help="Optional source duration cap")
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument(
        "--warmup-s",
        type=float,
        default=1.0,
        help="Repeat the first selected frame for this many seconds before advancing",
    )
    parser.add_argument("--log-interval-s", type=float, default=2.0)
    args = parser.parse_args()

    data = _load_bonedata(args.json_file)
    frame_count = _frame_count(data)
    source_fps = _source_fps(data, args.source_fps)
    send_fps = float(args.fps) if args.fps is not None else source_fps
    if send_fps <= 0.0 or source_fps <= 0.0:
        raise ValueError("FPS values must be positive")

    frames = _selected_frames(args, frame_count, source_fps)
    target = (args.host, int(args.port))
    period = 1.0 / send_fps
    max_frames = max(0, int(args.max_frames))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(
        "Sony BoneData sender: "
        f"{args.json_file} frames={frame_count} selected={frames.start}:{frames.stop}:{frames.step} "
        f"source_fps={source_fps:.2f} send_fps={send_fps:.2f} target={target[0]}:{target[1]} "
        f"format={args.format}"
    )

    stream_frame_index = 0
    next_send = time.perf_counter()
    last_log = time.perf_counter()
    bytes_sent = 0
    selected_first = frames.start

    try:
        warmup_frames = int(max(0.0, float(args.warmup_s)) * send_fps)
        for _ in range(warmup_frames):
            bytes_sent += _send_frame(
                sock,
                target,
                data,
                json_file=args.json_file,
                source_frame_index=selected_first,
                stream_frame_index=stream_frame_index,
                send_fps=send_fps,
                source_fps=source_fps,
                packet_format=args.format,
            )
            stream_frame_index += 1
            next_send += period
            sleep_s = next_send - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            if max_frames and stream_frame_index >= max_frames:
                return

        while True:
            for source_frame_index in frames:
                bytes_sent += _send_frame(
                    sock,
                    target,
                    data,
                    json_file=args.json_file,
                    source_frame_index=source_frame_index,
                    stream_frame_index=stream_frame_index,
                    send_fps=send_fps,
                    source_fps=source_fps,
                    packet_format=args.format,
                )
                stream_frame_index += 1

                now = time.perf_counter()
                if args.log_interval_s > 0.0 and now - last_log >= args.log_interval_s:
                    print(
                        f"sent={stream_frame_index} source_frame={source_frame_index} "
                        f"bytes={bytes_sent}"
                    )
                    last_log = now

                if max_frames and stream_frame_index >= max_frames:
                    return

                next_send += period
                sleep_s = next_send - time.perf_counter()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)

            if not args.loop:
                return
    finally:
        sock.close()
        print(f"sender stopped: sent={stream_frame_index} bytes={bytes_sent}")


if __name__ == "__main__":
    main()
