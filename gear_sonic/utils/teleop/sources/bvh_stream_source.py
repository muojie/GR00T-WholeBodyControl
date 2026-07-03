"""UDP BVH frame stream receiver for realtime BVH-to-G1 POSE testing."""

from __future__ import annotations

import json
import math
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
    g1_body_pos14_world_to_smpl_joints,
    prepare_bvh_g1_retarget_context_from_motion,
    retarget_bvh_g1_frame,
    update_bvh_g1_retarget_context_frame,
    _stabilize_joint_position_step,
)
from gear_sonic.utils.teleop.sources.bvh_source import (
    BvhMotion,
    SMPL_JOINT_SOURCE_KEYS,
    build_full_body_reference_from_skeleton_frame,
    _resolve_selected_indices,
)
from gear_sonic.utils.teleop.sources.sony_bonedata_json import (
    SONY_BONEDATA_JSON_FORMAT,
    convert_sony_bonedata_payload_to_bvh_stream_payload,
)


BVH_STREAM_DEFAULT_PORT = 12352
BVH_STREAM_FORMAT = "bvh_stream_v1"


def _quat_mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = (float(v) for v in a)
    bw, bx, by, bz = (float(v) for v in b)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float32,
    )


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
    if payload.get("format") not in {BVH_STREAM_FORMAT, "bvh_stream", SONY_BONEDATA_JSON_FORMAT}:
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
        bonedata_position_scale: float = 1.0,
        bonedata_input_quat_order: str = "xyzw",
        bonedata_rotation_mode: str = "input",
        bonedata_coordinate_frame: str = "sonic_zup",
        bonedata_local_root: bool = False,
    ):
        self.bind_host = bind_host
        self.port = int(port)
        self.packet_format = packet_format
        self.recv_size = int(recv_size)
        self.socket_timeout_s = float(socket_timeout_s)
        self.retarget_config = retarget_config or BvhG1RetargetConfig()
        self.align_root = bool(align_root)
        self.bonedata_position_scale = float(bonedata_position_scale)
        self.bonedata_input_quat_order = str(bonedata_input_quat_order)
        self.bonedata_rotation_mode = str(bonedata_rotation_mode)
        self.bonedata_coordinate_frame = str(bonedata_coordinate_frame)
        self.bonedata_local_root = bool(bonedata_local_root)

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
        if payload.get("format") == SONY_BONEDATA_JSON_FORMAT:
            payload = convert_sony_bonedata_payload_to_bvh_stream_payload(
                payload,
                output_format=BVH_STREAM_FORMAT,
                position_scale=self.bonedata_position_scale,
                input_quat_order=self.bonedata_input_quat_order,
                rotation_mode=self.bonedata_rotation_mode,
                coordinate_frame=self.bonedata_coordinate_frame,
                local_root=self.bonedata_local_root,
            )

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
        # Expose head/hand source joints so the VR 3-point retargeter (and thus
        # upper-body targets in planner mode) works on streamed skeletons too.
        # They are expressed relative to the skeleton root's horizontal position
        # and yaw ("operator on a treadmill"): the VR3pt calibration subtracts a
        # constant offset, so if the walking translation stayed in these poses
        # the arm targets would drift meters away from the robot as the person
        # walks and drag it over.
        frame_joints: dict[str, Pose7D] = {"root": root_pose}
        person_facing_yaw: float | None = None
        try:
            source_root_idx = joint_names.index("root")
        except ValueError:
            source_root_idx = None
        if source_root_idx is not None:
            anchor_pos = positions[source_root_idx]
            # Person facing from the shoulder line: the mocopi root quaternion's
            # +X is rotated a constant ~90 deg away from where the person
            # actually faces, which would rotate both the treadmill-local hand
            # frame and the planner facing command. The shoulder-line normal is
            # convention-free.
            anchor_yaw = None
            try:
                l_sh = positions[joint_names.index("l_up_arm")]
                r_sh = positions[joint_names.index("r_up_arm")]
                side = r_sh - l_sh
                if float(np.hypot(side[0], side[1])) > 1e-6:
                    anchor_yaw = math.atan2(float(side[0]), -float(side[1]))
            except ValueError:
                pass
            if anchor_yaw is None:
                aw, ax, ay, az = (float(v) for v in quats[source_root_idx])
                anchor_yaw = math.atan2(
                    2.0 * (aw * az + ax * ay), 1.0 - 2.0 * (ay * ay + az * az)
                )
            forward = np.array([math.cos(anchor_yaw), math.sin(anchor_yaw), 0.0])
            if self._context is not None and self._context.root0_inv is not None:
                forward_aligned = self._context.root0_inv.apply(forward)
            else:
                forward_aligned = forward
            person_facing_yaw = float(
                math.atan2(float(forward_aligned[1]), float(forward_aligned[0]))
            )
            cos_y = math.cos(-anchor_yaw)
            sin_y = math.sin(-anchor_yaw)
            yaw_inv_wxyz = np.array(
                [math.cos(-anchor_yaw / 2.0), 0.0, 0.0, math.sin(-anchor_yaw / 2.0)],
                dtype=np.float32,
            )
            for source_name in ("head", "l_hand", "r_hand"):
                try:
                    source_idx = joint_names.index(source_name)
                except ValueError:
                    continue
                rel = positions[source_idx] - np.array(
                    [anchor_pos[0], anchor_pos[1], 0.0], dtype=np.float32
                )
                local_pos = np.array(
                    [
                        cos_y * rel[0] - sin_y * rel[1],
                        sin_y * rel[0] + cos_y * rel[1],
                        rel[2],
                    ],
                    dtype=np.float32,
                )
                frame_joints[source_name] = Pose7D(
                    position=local_pos,
                    quat_wxyz=_quat_mul_wxyz(yaw_inv_wxyz, quats[source_idx]),
                )
        smpl_joints = None
        smpl_joints_source = str(self.retarget_config.smpl_joints_source or "skeleton").lower()
        if smpl_joints_source == "g1_fk" and body_pos is not None:
            smpl_joints = g1_body_pos14_world_to_smpl_joints(
                body_pos,
                self._context.root_pos[0],
                self._context.root_quat[0],
            )
        elif smpl_joints_source != "skeleton":
            raise ValueError(
                f"unsupported smpl_joints_source {self.retarget_config.smpl_joints_source!r}; "
                "expected 'g1_fk' or 'skeleton'"
            )
        full_body = build_full_body_reference_from_skeleton_frame(
            self._context.bvh_motion.joint_names,
            self._context.bvh_motion.world_positions[0],
            self._context.bvh_motion.world_quat_wxyz[0],
            frame_index=stream_frame_idx,
            smpl_joints=smpl_joints,
            body_quat_w=self._context.root_quat[0],
            body_pos_w=self._context.root_pos[0],
            body_pos=body_pos,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
        )

        return MocapFrame(
            source="bvh_stream",
            host_time_s=receive_time_s,
            source_time_ns=payload.get("source_time_ns"),
            frame_index=stream_frame_idx,
            fps=playback_fps,
            joints=frame_joints,
            full_body=full_body,
            metadata={
                "format": BVH_STREAM_FORMAT,
                "input_format": payload.get("input_format"),
                "path": payload.get("path"),
                "motion_name": payload.get("motion_name"),
                "source_frame_index": source_frame_idx,
                "source_fps": source_fps,
                "packet_format": payload.get("packet_format"),
                "joint_count": len(joint_names),
                "bonedata_coordinate_frame": payload.get("bonedata_coordinate_frame"),
                "bonedata_position_scale": payload.get("bonedata_position_scale"),
                "bonedata_input_quat_order": payload.get("bonedata_input_quat_order"),
                "bonedata_rotation_mode": payload.get("bonedata_rotation_mode"),
                "bonedata_local_root": payload.get("bonedata_local_root"),
                "person_facing_yaw": person_facing_yaw,
            },
        )
