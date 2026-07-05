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

The source also accepts ``bvh_stream_v1`` packets (``bvh_stream_sender.py``
streaming a mocopi-skeleton BVH file), so recorded BVH takes the same
rotation-driven PICO line. BVH payloads carry world poses either in the SONIC
Z-up frame (sender default) or the raw BVH right-handed Y-up frame (sender
``--no-y-up-to-z-up``); ``bvh_input_frame`` selects the matching basis change
into the XRT convention. This assumes the BVH rest pose is a world-aligned
T-pose with identity world rotations (true for mocopi BVH exports) — the same
bind-frame assumption the saveBoneData line relies on.
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

# Basis changes into the XRT convention for the world frames a bvh_stream_v1
# payload can carry. Both compose zflip with the exact frame conversions used
# elsewhere in this package, so the three input frames land in the same XRT
# world:
#   sonic_zup: zflip @ inv(unity_yup -> sonic_zup)   (sender default)
#   bvh_yup:   zflip @ inv(unity_yup -> sonic_zup) @ (bvh_yup -> sonic_zup)
#              = diag(-1, 1, -1), i.e. the standard BVH<->Unity x-mirror
#              followed by zflip (sender --no-y-up-to-z-up)
SONY_PICO_BVH_INPUT_FRAMES = ("sonic_zup", "bvh_yup")
XRT_BASIS_BY_BVH_INPUT_FRAME: dict[str, np.ndarray] = {
    "sonic_zup": np.array(
        [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]], dtype=np.float64
    ),
    "bvh_yup": np.diag([-1.0, 1.0, -1.0]),
}

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


def read_bvh_stream_frame_arrays(
    payload: dict[str, Any],
) -> tuple[list[str], np.ndarray, np.ndarray]:
    """Extract (names, positions (J,3), quats_xyzw (J,4)) from a bvh_stream_v1 payload."""
    names = payload.get("joint_names")
    positions = payload.get("world_positions")
    quats_wxyz = payload.get("world_quat_wxyz")
    if not isinstance(names, list) or not names:
        raise ValueError("bvh_stream payload requires non-empty list field 'joint_names'")
    if positions is None or quats_wxyz is None:
        raise ValueError(
            "bvh_stream payload requires fields 'world_positions' and 'world_quat_wxyz'"
        )

    joint_names = [str(name) for name in names]
    pos = np.asarray(positions, dtype=np.float64)
    quat = np.asarray(quats_wxyz, dtype=np.float64)
    if pos.ndim == 3 and pos.shape[0] == 1:
        pos = pos[0]
    if quat.ndim == 3 and quat.shape[0] == 1:
        quat = quat[0]
    if pos.shape != (len(joint_names), 3):
        raise ValueError(
            f"world_positions must have shape ({len(joint_names)}, 3), got {pos.shape}"
        )
    if quat.shape != (len(joint_names), 4):
        raise ValueError(
            f"world_quat_wxyz must have shape ({len(joint_names)}, 4), got {quat.shape}"
        )

    quat_xyzw = quat[:, [1, 2, 3, 0]].copy()
    norms = np.linalg.norm(quat_xyzw, axis=1, keepdims=True)
    bad = (norms[:, 0] < 1e-8) | ~np.isfinite(norms[:, 0])
    quat_xyzw[bad] = (0.0, 0.0, 0.0, 1.0)
    norms[bad] = 1.0
    quat_xyzw /= norms
    return joint_names, pos, quat_xyzw


def bonedata_to_xrt_body_poses(
    positions: np.ndarray,
    quats_xyzw: np.ndarray,
    slot_indices: np.ndarray,
    basis: np.ndarray = UNITY_TO_XRT_BASIS,
) -> np.ndarray:
    """Build the (24, 7) [x,y,z,qx,qy,qz,qw] array PicoReader would deliver."""
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
        basis: np.ndarray = UNITY_TO_XRT_BASIS,
    ) -> FullBodyReference:
        names_key = tuple(joint_names)
        if self._slot_indices is None or self._joint_names != names_key:
            self._slot_indices = resolve_smpl_slot_bone_indices(joint_names)
            self._joint_names = names_key

        body_poses = bonedata_to_xrt_body_poses(
            positions, quats_xyzw, self._slot_indices, basis=basis
        )
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


# Accepted bvh_stream payload format markers (mirrors bvh_stream_source).
_BVH_STREAM_FORMATS = frozenset({"bvh_stream_v1", "bvh_stream"})


def parse_sony_pico_packet(packet: bytes, packet_format: str = "auto") -> dict[str, Any]:
    """Decode one raw sony_bonedata_json_v1 or bvh_stream_v1 packet."""
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
    payload_format = payload.get("format")
    if payload_format != SONY_BONEDATA_JSON_FORMAT and payload_format not in _BVH_STREAM_FORMATS:
        raise ValueError(
            f"--source {SONY_PICO_SOURCE_NAME} accepts raw {SONY_BONEDATA_JSON_FORMAT!r} "
            f"or bvh_stream_v1 packets, got {payload_format!r}"
        )
    return payload


class SonyPicoSmplUdpSource:
    """Threaded UDP receiver feeding BoneData or BVH-stream frames to the PICO stack."""

    def __init__(
        self,
        bind_host: str = "0.0.0.0",
        port: int = 12352,
        packet_format: str = "auto",
        recv_size: int = 262144,
        socket_timeout_s: float = 0.5,
        position_scale: float = 1.0,
        input_quat_order: str = "xyzw",
        bvh_input_frame: str = "sonic_zup",
    ):
        self.bind_host = bind_host
        self.port = int(port)
        self.packet_format = packet_format
        self.recv_size = int(recv_size)
        self.socket_timeout_s = float(socket_timeout_s)
        self.position_scale = float(position_scale)
        self.input_quat_order = str(input_quat_order)
        if bvh_input_frame not in XRT_BASIS_BY_BVH_INPUT_FRAME:
            raise ValueError(
                f"unsupported bvh_input_frame {bvh_input_frame!r}; "
                f"expected one of {SONY_PICO_BVH_INPUT_FRAMES}"
            )
        self.bvh_input_frame = str(bvh_input_frame)

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
        payload_format = payload.get("format")
        if payload_format in _BVH_STREAM_FORMATS:
            # bvh_stream_v1 world poses are canonical meters; position_scale
            # and input_quat_order only apply to raw Unity-frame BoneData.
            joint_names, positions, quats_xyzw = read_bvh_stream_frame_arrays(payload)
            basis = XRT_BASIS_BY_BVH_INPUT_FRAME[self.bvh_input_frame]
        else:
            joint_names, positions, quats_xyzw = read_bonedata_frame_arrays(
                payload,
                position_scale=self.position_scale,
                input_quat_order=self.input_quat_order,
            )
            basis = UNITY_TO_XRT_BASIS
        frame_index = int(payload.get("frame_index", 0))
        convert_start = time.perf_counter()
        full_body = self._converter.convert(
            joint_names,
            positions,
            quats_xyzw,
            frame_index=frame_index,
            basis=basis,
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
            positions, quats_xyzw, np.array([root_row], dtype=np.int64), basis=basis
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
                "format": payload_format,
                "path": payload.get("path"),
                "motion_name": payload.get("motion_name"),
                "source_frame_index": int(payload.get("source_frame_index", frame_index)),
                "source_fps": source_fps,
                "joint_count": len(joint_names),
                "convert_ms": round(convert_ms, 2),
                "bvh_input_frame": (
                    self.bvh_input_frame if payload_format in _BVH_STREAM_FORMATS else None
                ),
            },
        )
