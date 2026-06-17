"""Sony mocopi UDP receiver and packet parser.

The official mocopi app sends binary UDP packets on port 12351 by default.
This module also accepts a small JSON bridge format for integrations that
already compute robot-ready VR 3-point targets.
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from gear_sonic.utils.teleop.sources.base import MocapFrame, Pose7D, normalize_quat_wxyz


MOCOPI_DEFAULT_PORT = 12351
MOCOPI_MAX_BONES = 27

MOCOPI_BONE_NAMES = {
    0: "root",
    1: "torso_1",
    2: "torso_2",
    3: "torso_3",
    4: "torso_4",
    5: "torso_5",
    6: "torso_6",
    7: "torso_7",
    8: "neck_1",
    9: "neck_2",
    10: "head",
    11: "left_shoulder",
    12: "left_upper_arm",
    13: "left_lower_arm",
    14: "left_wrist",
    15: "right_shoulder",
    16: "right_upper_arm",
    17: "right_lower_arm",
    18: "right_wrist",
    19: "left_upper_leg",
    20: "left_lower_leg",
    21: "left_foot",
    22: "left_toes",
    23: "right_upper_leg",
    24: "right_lower_leg",
    25: "right_foot",
    26: "right_toes",
}


class MocopiPacketError(ValueError):
    """Raised when a mocopi UDP packet cannot be decoded."""


@dataclass
class _Chunk:
    name: str
    data: bytes
    end: int


def _read_chunk(packet: bytes, offset: int) -> _Chunk:
    if offset + 8 > len(packet):
        raise MocopiPacketError("truncated chunk header")
    size = int.from_bytes(packet[offset : offset + 4], "little", signed=False)
    name = packet[offset + 4 : offset + 8].decode("ascii", errors="replace")
    start = offset + 8
    end = start + size
    if end > len(packet):
        raise MocopiPacketError(f"chunk {name!r} extends beyond packet")
    return _Chunk(name=name, data=packet[start:end], end=end)


def _read_int(packet: bytes, offset: int, expected_name: str | None = None) -> tuple[str, int, int]:
    chunk = _read_chunk(packet, offset)
    if expected_name is not None and chunk.name != expected_name:
        raise MocopiPacketError(f"expected chunk {expected_name!r}, got {chunk.name!r}")
    return chunk.name, int.from_bytes(chunk.data, "little", signed=False), chunk.end


def _read_raw(packet: bytes, offset: int, expected_name: str | None = None) -> tuple[str, bytes, int]:
    chunk = _read_chunk(packet, offset)
    if expected_name is not None and chunk.name != expected_name:
        raise MocopiPacketError(f"expected chunk {expected_name!r}, got {chunk.name!r}")
    return chunk.name, chunk.data, chunk.end


def _read_vector7(
    packet: bytes, offset: int, expected_name: str | None = None
) -> tuple[str, tuple[float, ...], int]:
    chunk = _read_chunk(packet, offset)
    if expected_name is not None and chunk.name != expected_name:
        raise MocopiPacketError(f"expected chunk {expected_name!r}, got {chunk.name!r}")
    if len(chunk.data) != 28:
        raise MocopiPacketError(f"chunk {chunk.name!r} should contain 7 float32 values")
    return chunk.name, struct.unpack("<7f", chunk.data), chunk.end


def _mocopi_vector_to_pose(vector: tuple[float, ...]) -> Pose7D:
    """Convert mocopi's raw 7-float transform to the wxyz convention used here."""
    raw = np.asarray(vector, dtype=np.float32)
    position = raw[4:7].astype(np.float32)

    # The official Blender receiver remaps the raw quaternion this way before
    # applying per-bone axis corrections. Keep that base convention here; later
    # retargeting layers can add skeleton-specific offsets.
    quat_wxyz = np.array([-raw[3], -raw[0], raw[1], raw[2]], dtype=np.float32)
    return Pose7D(position=position, quat_wxyz=quat_wxyz)


def parse_mocopi_binary_packet(packet: bytes) -> MocapFrame:
    """Decode one official mocopi UDP packet into a canonical frame."""
    offset = 0
    metadata: dict[str, Any] = {"format": "mocopi-binary"}

    chunk = _read_chunk(packet, offset)
    if chunk.name != "head":
        raise MocopiPacketError(f"expected 'head' chunk, got {chunk.name!r}")
    offset = chunk.end

    name, data, offset = _read_raw(packet, offset, "ftyp")
    metadata[name] = data.decode("ascii", errors="replace").rstrip("\x00")
    name, version, offset = _read_int(packet, offset, "vrsn")
    metadata[name] = version

    chunk = _read_chunk(packet, offset)
    if chunk.name != "sndf":
        raise MocopiPacketError(f"expected 'sndf' chunk, got {chunk.name!r}")
    offset = chunk.end
    _, _, offset = _read_raw(packet, offset, "ipad")
    _, _, offset = _read_raw(packet, offset, "rcvp")

    chunk = _read_chunk(packet, offset)
    if chunk.name == "fram":
        offset = chunk.end
        for expected in ("fnum", "time", "uttm", "tmcd"):
            if offset + 8 > len(packet):
                break
            next_chunk = _read_chunk(packet, offset)
            if next_chunk.name != expected:
                break
            _, value, offset = _read_int(packet, offset, expected)
            metadata[expected] = value
        chunk = _read_chunk(packet, offset)

    if chunk.name != "btrs":
        raise MocopiPacketError(f"expected 'btrs' chunk, got {chunk.name!r}")
    offset = chunk.end

    bones: dict[int, Pose7D] = {}
    joints: dict[str, Pose7D] = {}
    for _ in range(MOCOPI_MAX_BONES):
        if offset + 8 > len(packet):
            break
        btdt = _read_chunk(packet, offset)
        if btdt.name != "btdt":
            break
        offset = btdt.end
        _, bone_id, offset = _read_int(packet, offset, "bnid")
        _, vector, offset = _read_vector7(packet, offset, "tran")
        pose = _mocopi_vector_to_pose(vector)
        bones[int(bone_id)] = pose
        joint_name = MOCOPI_BONE_NAMES.get(int(bone_id))
        if joint_name is not None:
            joints[joint_name] = pose

    return MocapFrame(
        source="sony_mocopi",
        host_time_s=time.time(),
        source_time_ns=metadata.get("uttm"),
        frame_index=metadata.get("fnum"),
        joints=joints,
        bones=bones,
        metadata=metadata,
    )


def _quat_from_json(value: Any, order: str = "wxyz") -> np.ndarray:
    quat = np.asarray(value, dtype=np.float32)
    if quat.shape != (4,):
        raise ValueError(f"quaternion must have shape (4,), got {quat.shape}")
    if order == "xyzw":
        quat = np.array([quat[3], quat[0], quat[1], quat[2]], dtype=np.float32)
    elif order != "wxyz":
        raise ValueError(f"unsupported quaternion order {order!r}")
    return normalize_quat_wxyz(quat)


def parse_mocopi_json_packet(packet: bytes) -> MocapFrame:
    """Decode a JSON bridge packet.

    Supported direct robot-ready fields:
      {"vr_position": [9 floats], "vr_orientation": [12 floats]}

    Supported named joint fields:
      {"joints": {"left_wrist": {"pos": [3], "quat": [4]}}}
    """
    try:
        payload = json.loads(packet.decode("utf-8"))
    except json.JSONDecodeError as exc:
        raise MocopiPacketError(f"invalid JSON packet: {exc}") from exc

    quat_order = payload.get("quat_order", "wxyz")
    joints: dict[str, Pose7D] = {}
    for name, value in payload.get("joints", {}).items():
        position = value.get("pos", value.get("position"))
        quat = value.get("quat_wxyz", value.get("quat", value.get("orientation")))
        joint_order = value.get("quat_order", quat_order)
        if position is None or quat is None:
            continue
        joints[str(name)] = Pose7D(position=position, quat_wxyz=_quat_from_json(quat, joint_order))

    bones: dict[int, Pose7D] = {}
    for key, value in payload.get("bones", {}).items():
        position = value.get("pos", value.get("position"))
        quat = value.get("quat_wxyz", value.get("quat", value.get("orientation")))
        bone_order = value.get("quat_order", quat_order)
        if position is None or quat is None:
            continue
        bone_id = int(key)
        pose = Pose7D(position=position, quat_wxyz=_quat_from_json(quat, bone_order))
        bones[bone_id] = pose
        joint_name = MOCOPI_BONE_NAMES.get(bone_id)
        if joint_name is not None and joint_name not in joints:
            joints[joint_name] = pose

    direct_vr_position = payload.get("vr_position")
    direct_vr_orientation = payload.get("vr_orientation")
    if direct_vr_position is None and "vr_pose" in payload:
        vr_pose = np.asarray(payload["vr_pose"], dtype=np.float32)
        if vr_pose.shape != (3, 7):
            raise MocopiPacketError(f"vr_pose must have shape (3, 7), got {vr_pose.shape}")
        direct_vr_position = vr_pose[:, :3].reshape(-1)
        direct_vr_orientation = vr_pose[:, 3:7].reshape(-1)

    return MocapFrame(
        source=str(payload.get("source", "sony_mocopi_json")),
        host_time_s=time.time(),
        source_time_ns=payload.get("source_time_ns", payload.get("timestamp_ns")),
        frame_index=payload.get("frame_index"),
        fps=float(payload.get("fps", 0.0)),
        joints=joints,
        bones=bones,
        direct_vr_position=direct_vr_position,
        direct_vr_orientation=direct_vr_orientation,
        metadata={"format": "json", **payload.get("metadata", {})},
    )


def parse_mocopi_packet(packet: bytes, packet_format: str = "auto") -> MocapFrame:
    """Decode a mocopi packet. packet_format is auto, binary, or json."""
    if packet_format == "auto":
        stripped = packet.lstrip()
        packet_format = "json" if stripped.startswith((b"{", b"[")) else "binary"
    if packet_format == "json":
        return parse_mocopi_json_packet(packet)
    if packet_format == "binary":
        return parse_mocopi_binary_packet(packet)
    raise ValueError(f"unsupported mocopi packet format {packet_format!r}")


class MocopiUdpSource:
    """Threaded UDP receiver for mocopi packets."""

    def __init__(
        self,
        bind_host: str = "0.0.0.0",
        port: int = MOCOPI_DEFAULT_PORT,
        packet_format: str = "auto",
        recv_size: int = 4096,
        socket_timeout_s: float = 0.5,
    ):
        self.bind_host = bind_host
        self.port = int(port)
        self.packet_format = packet_format
        self.recv_size = int(recv_size)
        self.socket_timeout_s = float(socket_timeout_s)

        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._lock = threading.Lock()
        self._latest: MocapFrame | None = None
        self._last_receive_time_s: float | None = None
        self._fps = 0.0
        self._received_packets = 0
        self._dropped_packets = 0
        self._last_error: str | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((self.bind_host, self.port))
        self._socket.settimeout(self.socket_timeout_s)
        self._running.set()
        self._thread = threading.Thread(target=self._run, name="MocopiUdpSource", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._socket is not None:
            self._socket.close()
            self._socket = None
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
                "fps": self._fps,
                "received_packets": self._received_packets,
                "dropped_packets": self._dropped_packets,
                "last_error": self._last_error,
                "has_frame": self._latest is not None,
            }

    def _run(self) -> None:
        assert self._socket is not None
        while self._running.is_set():
            try:
                packet, _ = self._socket.recvfrom(self.recv_size)
            except socket.timeout:
                continue
            except OSError:
                if self._running.is_set():
                    with self._lock:
                        self._last_error = "socket closed unexpectedly"
                break

            now = time.time()
            try:
                frame = parse_mocopi_packet(packet, self.packet_format)
            except Exception as exc:
                with self._lock:
                    self._dropped_packets += 1
                    self._last_error = str(exc)
                continue

            if self._last_receive_time_s is not None:
                dt = max(1e-6, now - self._last_receive_time_s)
                inst_fps = 1.0 / dt
                self._fps = inst_fps if self._fps <= 0.0 else 0.9 * self._fps + 0.1 * inst_fps
            self._last_receive_time_s = now
            if frame.fps <= 0.0:
                frame.fps = self._fps
            frame.host_time_s = now
            self._received_packets += 1
            frame.metadata["receive_sequence"] = self._received_packets
            frame.metadata["receive_time_s"] = now

            with self._lock:
                self._latest = frame
                self._last_error = None
