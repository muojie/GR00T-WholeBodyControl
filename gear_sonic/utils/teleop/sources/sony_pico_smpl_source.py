"""Sony saveBoneData -> PICO SMPL stack source (route C, POSE v3 / encoder smpl).

Feeds raw mocopi ``sony_bonedata_json_v1`` UDP frames through the exact PICO
manager conversion functions (``compute_from_body_poses`` ->
``process_smpl_joints``) so the deploy side receives the same protocol v3 /
SMPL-encoder reference stream it gets from a PICO headset.

Conversion chain (validated offline by
``gear_sonic/scripts/validate_sony_bonedata_pico_smpl.py``):

    saveBoneData 27 bones, Unity left-handed Y-up, world poses
      -> zflip basis change (Unity left-handed -> XRT right-handed convention)
      -> 24 SMPL slots by bone name (rotations carried verbatim)
      -> PICO stack: global rotations -> SMPL local axis-angle + SMPL FK joints
      -> FullBodyReference (wrist joint_pos derived by the shared PICO wrist
         projection in FullBodyReference.__post_init__)

Unlike the ``bvh_stream`` route this source consumes the *raw* Unity-frame
payload and must NOT be combined with the sonic_zup BoneData conversion.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any

import msgpack
import msgpack_numpy as mnp
import numpy as np
from scipy.spatial.transform import Rotation as sRot

from gear_sonic.utils.teleop.sources.base import (
    FullBodyReference,
    MocapFrame,
    Pose7D,
)
from gear_sonic.utils.teleop.sources.sony_bonedata_json import (
    SONY_BONEDATA_JSON_FORMAT,
)

SONY_PICO_SOURCE_NAME = "sony_pico"

# SMPL parent table used by PoseStreamer in pico_manager_thread_server.py.
PICO_SMPL_PARENT_INDICES = [
    -1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 22, 23,
][:24]

# Basis change from Unity's left-handed Y-up world to the right-handed
# convention the XRT SDK delivers to the PICO stack. Positions transform as
# p' = B @ p and world rotations as R' = B @ R @ B^T (equivalently, quaternion
# components (x, y, z, w) -> (-x, -y, z, w)).
UNITY_TO_XRT_BASIS = np.diag([1.0, 1.0, -1.0])

# SMPL slot index -> accepted mocopi bone names (saveBoneData short names
# first, MOCOPI_BONE_NAMES long names as aliases). SMPL hand slots (22/23)
# reuse the wrist bone; their local rotation is dropped by the [:63] body_pose
# truncation inside the PICO stack.
SMPL_SLOT_BONE_NAMES: tuple[tuple[str, ...], ...] = (
    ("root",),                              # 0  pelvis
    ("l_up_leg", "left_upper_leg"),         # 1  L_hip
    ("r_up_leg", "right_upper_leg"),        # 2  R_hip
    ("torso_3",),                           # 3  spine1
    ("l_low_leg", "left_lower_leg"),        # 4  L_knee
    ("r_low_leg", "right_lower_leg"),       # 5  R_knee
    ("torso_5",),                           # 6  spine2
    ("l_foot", "left_foot"),                # 7  L_ankle
    ("r_foot", "right_foot"),               # 8  R_ankle
    ("torso_7",),                           # 9  spine3
    ("l_toes", "left_toes"),                # 10 L_foot
    ("r_toes", "right_toes"),               # 11 R_foot
    ("neck_1",),                            # 12 neck
    ("l_shoulder", "left_shoulder"),        # 13 L_collar
    ("r_shoulder", "right_shoulder"),       # 14 R_collar
    ("head",),                              # 15 head
    ("l_up_arm", "left_upper_arm"),         # 16 L_shoulder
    ("r_up_arm", "right_upper_arm"),        # 17 R_shoulder
    ("l_low_arm", "left_lower_arm"),        # 18 L_elbow
    ("r_low_arm", "right_lower_arm"),       # 19 R_elbow
    ("l_hand", "left_wrist"),               # 20 L_wrist
    ("r_hand", "right_wrist"),              # 21 R_wrist
    ("l_hand", "left_wrist"),               # 22 L_hand (reuse wrist)
    ("r_hand", "right_wrist"),              # 23 R_hand (reuse wrist)
)


def resolve_smpl_slot_bone_indices(joint_names: list[str]) -> np.ndarray:
    """Map the payload's bone-name list to the 24 SMPL slot source indices."""
    name_to_index = {str(name).strip().lower(): i for i, name in enumerate(joint_names)}
    indices = np.empty(len(SMPL_SLOT_BONE_NAMES), dtype=np.int64)
    missing: list[str] = []
    for slot, candidates in enumerate(SMPL_SLOT_BONE_NAMES):
        for candidate in candidates:
            if candidate in name_to_index:
                indices[slot] = name_to_index[candidate]
                break
        else:
            missing.append(candidates[0])
    if missing:
        raise ValueError(
            f"BoneData payload is missing bones required for SMPL slots: {missing}; "
            f"got joint names {joint_names}"
        )
    return indices


def read_bonedata_frame_arrays(
    payload: dict[str, Any],
    *,
    position_scale: float = 1.0,
    input_quat_order: str = "xyzw",
) -> tuple[list[str], np.ndarray, np.ndarray]:
    """Extract (names, positions (J,3), quats_xyzw (J,4)) from a raw payload."""
    names = payload.get("name", payload.get("joint_names"))
    positions = payload.get("position")
    rotations = payload.get("rotation")
    if not isinstance(names, list) or not names:
        raise ValueError("raw BoneData payload requires non-empty list field 'name'")
    if not isinstance(positions, list) or not isinstance(rotations, list):
        raise ValueError("raw BoneData payload requires list fields 'position' and 'rotation'")
    if len(positions) != len(names) or len(rotations) != len(names):
        raise ValueError(
            "raw BoneData payload lengths must match; "
            f"name={len(names)} position={len(positions)} rotation={len(rotations)}"
        )

    joint_names = [str(name) for name in names]
    pos = np.empty((len(names), 3), dtype=np.float64)
    quat = np.empty((len(names), 4), dtype=np.float64)
    for i, (p, r) in enumerate(zip(positions, rotations)):
        if isinstance(p, dict):
            pos[i] = (float(p["x"]), float(p["y"]), float(p["z"]))
        else:
            pos[i] = (float(p[0]), float(p[1]), float(p[2]))
        if isinstance(r, dict):
            quat[i] = (float(r["x"]), float(r["y"]), float(r["z"]), float(r["w"]))
        elif input_quat_order == "wxyz":
            quat[i] = (float(r[1]), float(r[2]), float(r[3]), float(r[0]))
        else:
            quat[i] = (float(r[0]), float(r[1]), float(r[2]), float(r[3]))
    pos *= float(position_scale)

    norms = np.linalg.norm(quat, axis=1, keepdims=True)
    bad = (norms[:, 0] < 1e-8) | ~np.isfinite(norms[:, 0])
    quat[bad] = (0.0, 0.0, 0.0, 1.0)
    norms[bad] = 1.0
    quat /= norms
    return joint_names, pos, quat


def bonedata_to_xrt_body_poses(
    positions: np.ndarray,
    quats_xyzw: np.ndarray,
    slot_indices: np.ndarray,
) -> np.ndarray:
    """Build the (24, 7) [x,y,z,qx,qy,qz,qw] array PicoReader would deliver."""
    basis = UNITY_TO_XRT_BASIS
    pos = positions[slot_indices] @ basis.T
    mats = sRot.from_quat(quats_xyzw[slot_indices]).as_matrix()
    mats = basis @ mats @ basis.T
    quat = sRot.from_matrix(mats).as_quat()
    return np.concatenate([pos, quat], axis=1)


class SonyPicoSmplConverter:
    """Run raw BoneData frames through the PICO manager SMPL functions."""

    def __init__(self) -> None:
        # Deferred import: pico_manager_thread_server pulls in the PICO/teleop
        # module tree, and importing it at sources package init time would
        # risk a partially-initialized circular import.
        from gear_sonic.scripts.pico_manager_thread_server import compute_from_body_poses
        import torch

        self._compute_from_body_poses = compute_from_body_poses
        self._device = torch.device("cpu")
        self._slot_indices: np.ndarray | None = None
        self._joint_names: tuple[str, ...] | None = None

    @property
    def slot_indices(self) -> np.ndarray | None:
        """Bone-row indices for the 24 SMPL slots, cached after first convert."""
        return self._slot_indices

    def preload(self) -> None:
        """Warm up the SMPL FK weights so the first live frame is not slow."""
        body_poses = np.zeros((24, 7), dtype=np.float64)
        body_poses[:, 6] = 1.0
        self._compute_from_body_poses(PICO_SMPL_PARENT_INDICES, self._device, body_poses)

    def convert(
        self,
        joint_names: list[str],
        positions: np.ndarray,
        quats_xyzw: np.ndarray,
        *,
        frame_index: int | None,
    ) -> FullBodyReference:
        names_key = tuple(joint_names)
        if self._slot_indices is None or self._joint_names != names_key:
            self._slot_indices = resolve_smpl_slot_bone_indices(joint_names)
            self._joint_names = names_key

        body_poses = bonedata_to_xrt_body_poses(positions, quats_xyzw, self._slot_indices)
        data = self._compute_from_body_poses(
            PICO_SMPL_PARENT_INDICES, self._device, body_poses
        )
        smpl_pose = (
            data["smpl_pose"].detach().cpu().numpy()[0][:63].reshape(21, 3).astype(np.float32)
        )
        smpl_joints = data["smpl_joints_local"].detach().cpu().numpy()[0].astype(np.float32)
        body_quat_w = data["global_orient_quat"].detach().cpu().numpy()[0].astype(np.float32)
        # joint_pos=None lets FullBodyReference derive the six G1 wrist joints
        # with the shared PICO projection (smpl_pose_to_g1_wrist_joint_pos).
        return FullBodyReference(
            smpl_joints=smpl_joints[:24],
            smpl_pose=smpl_pose,
            body_quat_w=body_quat_w,
            frame_index=frame_index,
        )


def parse_sony_pico_packet(packet: bytes, packet_format: str = "auto") -> dict[str, Any]:
    """Decode one raw sony_bonedata_json_v1 packet from JSON or msgpack."""
    normalized = str(packet_format or "auto").strip().lower()
    if normalized == "auto":
        normalized = "json" if packet.lstrip().startswith(b"{") else "msgpack"
    if normalized == "json":
        payload = json.loads(packet.decode("utf-8"))
    elif normalized == "msgpack":
        payload = msgpack.unpackb(packet, object_hook=mnp.decode, raw=False)
    else:
        raise ValueError(f"unsupported packet format {packet_format!r}")
    if not isinstance(payload, dict):
        raise ValueError(f"payload must be a dict, got {type(payload).__name__}")
    if payload.get("format") != SONY_BONEDATA_JSON_FORMAT:
        raise ValueError(
            f"--source {SONY_PICO_SOURCE_NAME} only accepts raw "
            f"{SONY_BONEDATA_JSON_FORMAT!r} packets, got {payload.get('format')!r}"
        )
    return payload


class SonyPicoSmplUdpSource:
    """Threaded UDP receiver feeding raw BoneData frames to the PICO stack."""

    def __init__(
        self,
        bind_host: str = "0.0.0.0",
        port: int = 12352,
        packet_format: str = "auto",
        recv_size: int = 262144,
        socket_timeout_s: float = 0.5,
        position_scale: float = 1.0,
        input_quat_order: str = "xyzw",
    ):
        self.bind_host = bind_host
        self.port = int(port)
        self.packet_format = packet_format
        self.recv_size = int(recv_size)
        self.socket_timeout_s = float(socket_timeout_s)
        self.position_scale = float(position_scale)
        self.input_quat_order = str(input_quat_order)

        self._converter = SonyPicoSmplConverter()
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
        self._convert_ms_ema = 0.0

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._converter.preload()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((self.bind_host, self.port))
        self._socket.settimeout(self.socket_timeout_s)
        self._running.set()
        self._thread = threading.Thread(target=self._run, name="SonyPicoSmplUdpSource", daemon=True)
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
            frame = self._payload_to_frame(payload, receive_time_s=receive_time_s)
        except Exception as exc:
            with self._lock:
                self._dropped_packets += 1
                self._last_error = str(exc)
                return self._latest

        with self._lock:
            frame.metadata["receive_sequence"] = sequence
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
                "convert_ms": round(self._convert_ms_ema, 2),
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
                payload = parse_sony_pico_packet(packet, self.packet_format)
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

    def _payload_to_frame(self, payload: dict[str, Any], *, receive_time_s: float) -> MocapFrame:
        joint_names, positions, quats_xyzw = read_bonedata_frame_arrays(
            payload,
            position_scale=self.position_scale,
            input_quat_order=self.input_quat_order,
        )
        frame_index = int(payload.get("frame_index", 0))
        convert_start = time.perf_counter()
        full_body = self._converter.convert(
            joint_names,
            positions,
            quats_xyzw,
            frame_index=frame_index,
        )
        convert_ms = (time.perf_counter() - convert_start) * 1000.0
        self._convert_ms_ema = (
            convert_ms if self._convert_ms_ema <= 0.0
            else 0.9 * self._convert_ms_ema + 0.1 * convert_ms
        )

        source_fps = float(payload.get("source_fps") or payload.get("fps") or 0.0)
        playback_fps = float(payload.get("fps") or source_fps or 50.0)
        root_row = int(self._converter.slot_indices[0])
        root_body_pose = bonedata_to_xrt_body_poses(
            positions, quats_xyzw, np.array([root_row], dtype=np.int64)
        )[0]
        root_pose = Pose7D(
            position=root_body_pose[:3],
            quat_wxyz=root_body_pose[[6, 3, 4, 5]],
        )

        return MocapFrame(
            source=SONY_PICO_SOURCE_NAME,
            host_time_s=receive_time_s,
            source_time_ns=payload.get("source_time_ns"),
            frame_index=frame_index,
            fps=playback_fps,
            joints={"root": root_pose},
            full_body=full_body,
            metadata={
                "format": SONY_BONEDATA_JSON_FORMAT,
                "path": payload.get("path"),
                "motion_name": payload.get("motion_name"),
                "source_frame_index": int(payload.get("source_frame_index", frame_index)),
                "source_fps": source_fps,
                "joint_count": len(joint_names),
                "convert_ms": round(convert_ms, 2),
            },
        )
