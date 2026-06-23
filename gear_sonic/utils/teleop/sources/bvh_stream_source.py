"""UDP BVH frame stream receiver for realtime BVH-to-G1 POSE testing."""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any

import msgpack
import msgpack_numpy as mnp
import numpy as np

from gear_sonic.utils.teleop.sources.base import (
    FullBodyReference,
    MocapFrame,
    Pose7D,
)
from gear_sonic.utils.teleop.sources.bvh_g1_source import (
    BvhG1RetargetConfig,
    prepare_bvh_g1_retarget_context_from_motion,
    retarget_bvh_g1_frame,
    update_bvh_g1_retarget_context_frame,
    _stabilize_joint_position_step,
)
from gear_sonic.utils.teleop.sources.bvh_source import (
    BvhMotion,
    SMPL_JOINT_SOURCE_KEYS,
    _resolve_selected_indices,
)


BVH_STREAM_DEFAULT_PORT = 12352
BVH_STREAM_FORMAT = "bvh_stream_v1"


def parse_bvh_stream_packet(packet: bytes, packet_format: str = "auto") -> dict[str, Any]:
    """Decode one BVH stream packet from msgpack or JSON."""
    normalized = str(packet_format or "auto").strip().lower()
    if normalized == "auto":
        normalized = "json" if packet.lstrip().startswith(b"{") else "msgpack"

    if normalized == "msgpack":
        payload = msgpack.unpackb(packet, object_hook=mnp.decode, raw=False)
    elif normalized == "json":
        payload = json.loads(packet.decode("utf-8"))
    else:
        raise ValueError(f"unsupported BVH stream packet format {packet_format!r}")

    if not isinstance(payload, dict):
        raise ValueError(f"BVH stream payload must be a dict, got {type(payload).__name__}")
    if payload.get("format") not in {BVH_STREAM_FORMAT, "bvh_stream"}:
        raise ValueError(f"unsupported BVH stream format {payload.get('format')!r}")
    return payload


class BvhStreamUdpSource:
    """Threaded UDP receiver that retargets streamed BVH skeleton frames to G1."""

    def __init__(
        self,
        bind_host: str = "0.0.0.0",
        port: int = BVH_STREAM_DEFAULT_PORT,
        packet_format: str = "auto",
        recv_size: int = 262144,
        socket_timeout_s: float = 0.5,
        retarget_config: BvhG1RetargetConfig | None = None,
        align_root: bool = True,
    ):
        self.bind_host = bind_host
        self.port = int(port)
        self.packet_format = packet_format
        self.recv_size = int(recv_size)
        self.socket_timeout_s = float(socket_timeout_s)
        self.retarget_config = retarget_config or BvhG1RetargetConfig()
        self.align_root = bool(align_root)

        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._lock = threading.Lock()
        self._latest: MocapFrame | None = None
        self._latest_payload: dict[str, Any] | None = None
        self._latest_payload_receive_time_s: float | None = None
        self._latest_payload_sequence: int = 0
        self._last_built_sequence: int = 0
        self._last_receive_time_s: float | None = None
        self._fps = 0.0
        self._received_packets = 0
        self._dropped_packets = 0
        self._last_error: str | None = None

        self._context = None
        self._joint_names: tuple[str, ...] | None = None
        self._last_joint_pos: np.ndarray | None = None
        self._last_output_joint_pos: np.ndarray | None = None
        self._last_source_frame_idx: int | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((self.bind_host, self.port))
        self._socket.settimeout(self.socket_timeout_s)
        self._running.set()
        self._thread = threading.Thread(target=self._run, name="BvhStreamUdpSource", daemon=True)
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
            payload = self._latest_payload
            receive_time_s = self._latest_payload_receive_time_s
            sequence = self._latest_payload_sequence
            if payload is None or receive_time_s is None:
                return self._latest
            if sequence == self._last_built_sequence:
                return self._latest

        try:
            frame = self._payload_to_frame(
                payload,
                receive_time_s=receive_time_s,
                receive_sequence=sequence,
            )
        except Exception as exc:
            with self._lock:
                self._dropped_packets += 1
                self._last_error = str(exc)
                return self._latest

        with self._lock:
            frame.metadata["receive_sequence"] = sequence
            frame.metadata["receive_time_s"] = receive_time_s
            self._latest = frame
            self._last_built_sequence = sequence
            self._last_error = None
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
                payload = parse_bvh_stream_packet(packet, self.packet_format)
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

            with self._lock:
                self._received_packets += 1
                self._latest_payload = payload
                self._latest_payload_receive_time_s = now
                self._latest_payload_sequence = self._received_packets
                self._last_error = None

    def _payload_to_frame(
        self,
        payload: dict[str, Any],
        *,
        receive_time_s: float,
        receive_sequence: int,
    ) -> MocapFrame:
        joint_names = tuple(str(name) for name in payload["joint_names"])
        positions = np.asarray(payload["world_positions"], dtype=np.float32)
        quats = np.asarray(payload["world_quat_wxyz"], dtype=np.float32)
        if positions.ndim == 3 and positions.shape[0] == 1:
            positions = positions[0]
        if quats.ndim == 3 and quats.shape[0] == 1:
            quats = quats[0]
        if positions.shape != (len(joint_names), 3):
            raise ValueError(
                f"world_positions must have shape ({len(joint_names)}, 3), got {positions.shape}"
            )
        if quats.shape != (len(joint_names), 4):
            raise ValueError(
                f"world_quat_wxyz must have shape ({len(joint_names)}, 4), got {quats.shape}"
            )

        source_fps = float(payload.get("source_fps") or payload.get("fps") or 0.0)
        playback_fps = float(payload.get("fps") or source_fps or 50.0)
        if self._context is None or self._joint_names != joint_names:
            self._context = prepare_bvh_g1_retarget_context_from_motion(
                BvhMotion(
                    path=str(payload.get("path", "bvh_stream")),
                    joint_names=list(joint_names),
                    world_positions=positions.reshape(1, len(joint_names), 3).astype(np.float32),
                    world_quat_wxyz=quats.reshape(1, len(joint_names), 4).astype(np.float32),
                    source_fps=source_fps or playback_fps,
                    source_frame_time_s=1.0 / max(1.0, source_fps or playback_fps),
                    frame_stride=1,
                    playback_fps=playback_fps,
                    selected_indices=_resolve_selected_indices(list(joint_names)),
                    smpl_source_indices=[
                        _resolve_selected_indices(list(joint_names)).get(source_key)
                        for source_key in SMPL_JOINT_SOURCE_KEYS
                    ],
                    lower_body_retarget_scale=0.0,
                ),
                align_root=self.align_root,
                retarget_config=self.retarget_config,
            )
            self._joint_names = joint_names
            self._last_joint_pos = None
            self._last_output_joint_pos = None
            self._last_source_frame_idx = None
        else:
            update_bvh_g1_retarget_context_frame(
                self._context,
                positions,
                quats,
                source_fps=source_fps,
                playback_fps=playback_fps,
            )

        source_frame_idx = int(payload.get("source_frame_index", payload.get("frame_index", receive_sequence)))
        stream_frame_idx = int(payload.get("frame_index", receive_sequence))
        raw_joint_pos = retarget_bvh_g1_frame(self._context, 0)
        joint_pos = _stabilize_joint_position_step(
            raw_joint_pos,
            self._last_output_joint_pos,
            fps=playback_fps,
            max_joint_velocity_radps=self.retarget_config.max_joint_velocity_radps,
            max_joint_step_rad=self.retarget_config.max_joint_step_rad,
            alpha=self.retarget_config.joint_filter_alpha,
        )
        joint_vel = np.zeros_like(joint_pos, dtype=np.float32)
        if self._last_joint_pos is not None and self._last_source_frame_idx is not None:
            frame_delta = source_frame_idx - self._last_source_frame_idx
            if frame_delta > 0:
                dt_s = max(1e-3, frame_delta / max(1.0, source_fps or playback_fps))
                joint_vel = ((joint_pos - self._last_joint_pos) / dt_s).astype(np.float32)
                max_vel = float(self.retarget_config.max_joint_velocity_radps)
                if max_vel > 0.0:
                    joint_vel = np.clip(joint_vel, -max_vel, max_vel).astype(np.float32)

        self._last_joint_pos = joint_pos.copy()
        self._last_output_joint_pos = joint_pos.copy()
        self._last_source_frame_idx = source_frame_idx

        body_pos = None
        if self.retarget_config.enable_body_fk:
            body_pos = self._context.fk.compute_body_pos14_world(
                joint_pos.reshape(1, 29),
                self._context.root_pos[0].reshape(1, 3),
                self._context.root_quat[0].reshape(1, 4),
            )[0]

        root_pose = Pose7D(
            position=self._context.root_pos[0],
            quat_wxyz=self._context.root_quat[0],
        )
        full_body = FullBodyReference(
            smpl_joints=np.zeros((24, 3), dtype=np.float32),
            smpl_pose=np.zeros((21, 3), dtype=np.float32),
            body_quat_w=self._context.root_quat[0],
            body_pos_w=self._context.root_pos[0],
            body_pos=body_pos,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            frame_index=stream_frame_idx,
        )

        return MocapFrame(
            source="bvh_stream",
            host_time_s=receive_time_s,
            source_time_ns=payload.get("source_time_ns"),
            frame_index=stream_frame_idx,
            fps=playback_fps,
            joints={"root": root_pose},
            full_body=full_body,
            metadata={
                "format": BVH_STREAM_FORMAT,
                "path": payload.get("path"),
                "motion_name": payload.get("motion_name"),
                "source_frame_index": source_frame_idx,
                "source_fps": source_fps,
                "packet_format": payload.get("packet_format"),
                "joint_count": len(joint_names),
            },
        )
