"""Sony mocopi saveBoneData JSON helpers for raw-frame UDP streaming."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SONY_BONEDATA_JSON_FORMAT = "sony_bonedata_json_v1"
SONY_BONEDATA_DEFAULT_JOINTS_PER_FRAME = 27
SONY_BONEDATA_COORDINATE_FRAMES = (
    "sonic_zup",
    "left_handed_zup",
    "left_handed_yup",
    "zup_flip_xy",
)
SONY_BONEDATA_INPUT_QUAT_ORDERS = ("xyzw", "wxyz")
SONY_BONEDATA_ROTATION_MODES = ("input", "identity")


@dataclass(frozen=True)
class SonyBoneDataJsonMotion:
    path: str
    joint_names: list[str]
    frame_positions: list[list[Any]]
    frame_rotations: list[list[Any]]
    source_fps: float
    playback_fps: float

    @property
    def frame_count(self) -> int:
        return len(self.frame_positions)

    def frame_payload(
        self,
        frame_idx: int,
        *,
        stream_frame_idx: int,
        motion_name: str,
    ) -> dict[str, Any]:
        return {
            "format": SONY_BONEDATA_JSON_FORMAT,
            "schema_version": 1,
            "path": self.path,
            "motion_name": motion_name,
            "joints_per_frame": len(self.joint_names),
            "frame_index": int(stream_frame_idx),
            "source_frame_index": int(frame_idx),
            "source_fps": float(self.source_fps),
            "fps": float(self.playback_fps),
            "frame_stride": 1,
            "name": self.joint_names,
            "position": self.frame_positions[frame_idx],
            "rotation": self.frame_rotations[frame_idx],
        }


def load_sony_bonedata_json_raw(
    json_file: Path,
    *,
    joints_per_frame: int,
    source_fps: float,
    playback_fps: float,
) -> SonyBoneDataJsonMotion:
    """Load saveBoneData JSON and split it into raw per-frame payload slices."""
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
    if joints_per_frame <= 0:
        raise ValueError("--joints-per-frame must be positive")
    if len(names) == 0:
        raise ValueError("name must not be empty")
    if len(positions) != len(rotations):
        raise ValueError(
            "position and rotation must have the same length; "
            f"got {len(positions)}, {len(rotations)}"
        )
    if len(positions) == 0 or len(positions) % joints_per_frame != 0:
        raise ValueError(
            f"position length {len(positions)} is not divisible by "
            f"joints_per_frame={joints_per_frame}"
        )

    frame_count = len(positions) // joints_per_frame
    names_are_flat_frames = len(names) == len(positions)
    names_are_single_frame = len(names) == joints_per_frame
    if not names_are_flat_frames and not names_are_single_frame:
        raise ValueError(
            "name must either repeat per frame or contain one joint-name frame; "
            f"got name={len(names)}, position={len(positions)}, "
            f"joints_per_frame={joints_per_frame}"
        )

    joint_names = [str(name) for name in names[:joints_per_frame]]
    frame_positions: list[list[Any]] = []
    frame_rotations: list[list[Any]] = []
    for frame_idx in range(frame_count):
        start = frame_idx * joints_per_frame
        end = start + joints_per_frame
        if names_are_flat_frames:
            frame_names = [str(name) for name in names[start:end]]
            if frame_names != joint_names:
                raise ValueError(
                    f"joint names changed at frame {frame_idx}; "
                    "raw BoneData streaming expects stable joint order within one sender session"
                )
        frame_positions.append(list(positions[start:end]))
        frame_rotations.append(list(rotations[start:end]))

    return SonyBoneDataJsonMotion(
        path=str(json_file),
        joint_names=joint_names,
        frame_positions=frame_positions,
        frame_rotations=frame_rotations,
        source_fps=float(source_fps),
        playback_fps=float(playback_fps),
    )


def convert_sony_bonedata_payload_to_bvh_stream_payload(
    payload: dict[str, Any],
    *,
    output_format: str,
    position_scale: float,
    input_quat_order: str,
    rotation_mode: str,
    coordinate_frame: str,
    local_root: bool,
) -> dict[str, Any]:
    """Convert one raw BoneData UDP payload into canonical bvh_stream_v1 fields."""
    joint_names = _read_joint_names(payload)
    positions = payload.get("position")
    rotations = payload.get("rotation")
    if not isinstance(positions, list) or not isinstance(rotations, list):
        raise ValueError("raw BoneData payload requires list fields 'position' and 'rotation'")
    if len(positions) != len(joint_names) or len(rotations) != len(joint_names):
        raise ValueError(
            "raw BoneData payload lengths must match; "
            f"name={len(joint_names)} position={len(positions)} rotation={len(rotations)}"
        )

    frame_positions = [
        _convert_position(
            _read_vec3(value, position_scale=position_scale),
            coordinate_frame=coordinate_frame,
        )
        for value in positions
    ]
    if local_root:
        root = frame_positions[0]
        frame_positions = [
            [pos[0] - root[0], pos[1] - root[1], pos[2] - root[2]]
            for pos in frame_positions
        ]

    frame_quats = [
        _convert_quat_wxyz(
            _read_quat_wxyz(value, input_quat_order=input_quat_order),
            coordinate_frame=coordinate_frame,
        )
        for value in rotations
    ]
    if rotation_mode == "identity":
        frame_quats = [[1.0, 0.0, 0.0, 0.0] for _ in frame_quats]
    elif rotation_mode != "input":
        raise ValueError(f"unsupported BoneData rotation mode {rotation_mode!r}")

    converted = {
        "format": output_format,
        "schema_version": 1,
        "path": payload.get("path"),
        "motion_name": payload.get("motion_name"),
        "joint_names": joint_names,
        "frame_index": int(payload.get("frame_index", 0)),
        "source_frame_index": int(payload.get("source_frame_index", payload.get("frame_index", 0))),
        "source_fps": float(payload.get("source_fps") or payload.get("fps") or 0.0),
        "fps": float(payload.get("fps") or payload.get("source_fps") or 50.0),
        "frame_stride": int(payload.get("frame_stride", 1)),
        "source_time_ns": payload.get("source_time_ns"),
        "world_positions": frame_positions,
        "world_quat_wxyz": frame_quats,
        "input_format": payload.get("format"),
        "bonedata_coordinate_frame": coordinate_frame,
        "bonedata_position_scale": float(position_scale),
        "bonedata_input_quat_order": input_quat_order,
        "bonedata_rotation_mode": rotation_mode,
        "bonedata_local_root": bool(local_root),
    }
    if "packet_format" in payload:
        converted["packet_format"] = payload["packet_format"]
    return converted


def _read_joint_names(payload: dict[str, Any]) -> list[str]:
    names = payload.get("name", payload.get("joint_names"))
    if not isinstance(names, list) or not names:
        raise ValueError("raw BoneData payload requires non-empty list field 'name'")
    return [str(name) for name in names]


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


def _convert_position(position: list[float], *, coordinate_frame: str) -> list[float]:
    if coordinate_frame == "sonic_zup":
        return position
    if coordinate_frame == "left_handed_zup":
        return [-position[0], position[1], position[2]]
    if coordinate_frame == "left_handed_yup":
        return [-position[0], -position[2], position[1]]
    if coordinate_frame == "zup_flip_xy":
        return [-position[0], -position[1], position[2]]
    raise ValueError(f"unsupported BoneData coordinate frame {coordinate_frame!r}")


def _normalize_quat_wxyz(quat: list[float]) -> list[float]:
    norm = math.sqrt(sum(float(v) * float(v) for v in quat))
    if norm < 1e-8 or not math.isfinite(norm):
        return [1.0, 0.0, 0.0, 0.0]
    return [float(v) / norm for v in quat]


def _quat_wxyz_to_matrix(quat: list[float]) -> list[list[float]]:
    w, x, y, z = _normalize_quat_wxyz(quat)
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def _matrix_to_quat_wxyz(matrix: list[list[float]]) -> list[float]:
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        quat = [0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(max(0.0, 1.0 + m00 - m11 - m22)) * 2.0
        quat = [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s]
    elif m11 > m22:
        s = math.sqrt(max(0.0, 1.0 + m11 - m00 - m22)) * 2.0
        quat = [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s]
    else:
        s = math.sqrt(max(0.0, 1.0 + m22 - m00 - m11)) * 2.0
        quat = [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s]
    return _normalize_quat_wxyz(quat)


def _basis_change_quat_wxyz(quat: list[float], basis: list[list[float]]) -> list[float]:
    rotation = _quat_wxyz_to_matrix(quat)
    changed = [
        [
            sum(
                basis[row][k] * rotation[k][l] * basis[col][l]
                for k in range(3)
                for l in range(3)
            )
            for col in range(3)
        ]
        for row in range(3)
    ]
    return _matrix_to_quat_wxyz(changed)


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


def _convert_quat_wxyz(quat: list[float], *, coordinate_frame: str) -> list[float]:
    if coordinate_frame == "sonic_zup":
        return quat
    if coordinate_frame == "left_handed_zup":
        return _normalize_quat_wxyz([quat[0], quat[1], -quat[2], -quat[3]])
    if coordinate_frame == "left_handed_yup":
        return _basis_change_quat_wxyz(
            quat,
            [[-1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
        )
    if coordinate_frame == "zup_flip_xy":
        return _basis_change_quat_wxyz(
            quat,
            [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
        )
    raise ValueError(f"unsupported BoneData coordinate frame {coordinate_frame!r}")
