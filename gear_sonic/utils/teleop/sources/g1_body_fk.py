"""Lightweight G1 forward kinematics for streamed body-position references."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from gear_sonic.utils.teleop.sources.base import (
    G1_MUJOCO_TO_ISAACLAB_IDX,
    normalize_quat_wxyz,
)


SONIC_BODY_NAMES = (
    "pelvis",
    "left_hip_roll_link",
    "left_knee_link",
    "left_ankle_roll_link",
    "right_hip_roll_link",
    "right_knee_link",
    "right_ankle_roll_link",
    "torso_link",
    "left_shoulder_roll_link",
    "left_elbow_link",
    "left_wrist_yaw_link",
    "right_shoulder_roll_link",
    "right_elbow_link",
    "right_wrist_yaw_link",
)


DEFAULT_G1_MJCF_PATH = (
    Path(__file__).resolve().parents[3]
    / "data"
    / "assets"
    / "robot_description"
    / "mjcf"
    / "g1_29dof_rev_1_0.xml"
)


@dataclass(frozen=True)
class _BodyNode:
    name: str
    parent: int
    local_pos: np.ndarray
    local_quat_wxyz: np.ndarray
    joint_motor_idx: int | None
    joint_axis: np.ndarray | None


class G1BodyFk:
    """Compute SONIC's 14 body-position references from G1 29-DOF joint targets."""

    def __init__(self, mjcf_path: str | Path = DEFAULT_G1_MJCF_PATH):
        self.mjcf_path = Path(mjcf_path)
        self.nodes, self.body_name_to_idx = _load_body_tree(self.mjcf_path)
        self.sonic_body_indexes = np.asarray(
            [self.body_name_to_idx[name] for name in SONIC_BODY_NAMES],
            dtype=np.int64,
        )
        self.pelvis_idx = self.body_name_to_idx["pelvis"]

    def compute_body_pos14_world(
        self,
        joint_pos_isaaclab: np.ndarray,
        root_pos_w: np.ndarray,
        root_quat_wxyz: np.ndarray,
    ) -> np.ndarray:
        """Return ``(T, 14, 3)`` body positions in world coordinates."""
        joint_pos_isaaclab, root_pos_w, root_quat_wxyz = _validate_fk_inputs(
            joint_pos_isaaclab,
            root_pos_w,
            root_quat_wxyz,
        )
        joint_pos_mujoco = joint_pos_isaaclab[:, G1_MUJOCO_TO_ISAACLAB_IDX]
        out = np.empty((joint_pos_isaaclab.shape[0], len(SONIC_BODY_NAMES), 3), dtype=np.float32)
        for frame_idx in range(joint_pos_isaaclab.shape[0]):
            body_pos_w, _ = self._compute_world_frame(
                joint_pos_mujoco[frame_idx],
                root_pos_w[frame_idx],
                root_quat_wxyz[frame_idx],
            )
            out[frame_idx] = body_pos_w[self.sonic_body_indexes]
        return out.astype(np.float32)

    def compute_body_pos14_pelvis(
        self,
        joint_pos_isaaclab: np.ndarray,
        root_pos_w: np.ndarray,
        root_quat_wxyz: np.ndarray,
    ) -> np.ndarray:
        """Return ``(T, 14, 3)`` body positions in the pelvis frame."""
        joint_pos_isaaclab, root_pos_w, root_quat_wxyz = _validate_fk_inputs(
            joint_pos_isaaclab,
            root_pos_w,
            root_quat_wxyz,
        )
        joint_pos_mujoco = joint_pos_isaaclab[:, G1_MUJOCO_TO_ISAACLAB_IDX]
        out = np.empty((joint_pos_isaaclab.shape[0], len(SONIC_BODY_NAMES), 3), dtype=np.float32)
        for frame_idx in range(joint_pos_isaaclab.shape[0]):
            body_pos_w, body_rot_w = self._compute_world_frame(
                joint_pos_mujoco[frame_idx],
                root_pos_w[frame_idx],
                root_quat_wxyz[frame_idx],
            )
            selected = body_pos_w[self.sonic_body_indexes]
            pelvis_pos = body_pos_w[self.pelvis_idx]
            pelvis_rot_inv = body_rot_w[self.pelvis_idx].inv()
            rel_w = selected - pelvis_pos
            out[frame_idx] = pelvis_rot_inv.apply(rel_w.reshape(-1, 3)).reshape(-1, 3)
        return out.astype(np.float32)

    def _compute_world_frame(
        self,
        joint_pos_mujoco: np.ndarray,
        root_pos_w: np.ndarray,
        root_quat_wxyz: np.ndarray,
    ) -> tuple[np.ndarray, list[Rotation]]:
        body_pos_w = np.zeros((len(self.nodes), 3), dtype=np.float32)
        body_rot_w: list[Rotation] = [Rotation.identity() for _ in self.nodes]
        root_quat_wxyz = normalize_quat_wxyz(root_quat_wxyz)
        root_rot = Rotation.from_quat(root_quat_wxyz[[1, 2, 3, 0]])

        for idx, node in enumerate(self.nodes):
            if node.parent < 0:
                body_pos_w[idx] = np.asarray(root_pos_w, dtype=np.float32)
                body_rot_w[idx] = root_rot
                continue

            parent_rot = body_rot_w[node.parent]
            parent_pos = body_pos_w[node.parent]
            body_pos_w[idx] = parent_pos + parent_rot.apply(node.local_pos)

            local_rot = Rotation.from_quat(node.local_quat_wxyz[[1, 2, 3, 0]])
            if node.joint_motor_idx is not None and node.joint_axis is not None:
                angle = float(joint_pos_mujoco[node.joint_motor_idx])
                local_rot = local_rot * Rotation.from_rotvec(node.joint_axis * angle)
            body_rot_w[idx] = parent_rot * local_rot

        return body_pos_w, body_rot_w


def _load_body_tree(mjcf_path: Path) -> tuple[list[_BodyNode], dict[str, int]]:
    root = ET.parse(mjcf_path).getroot()
    motor_joint_order = [
        motor.attrib["joint"] for motor in root.findall("./actuator/motor") if "joint" in motor.attrib
    ]
    joint_to_motor_idx = {joint_name: idx for idx, joint_name in enumerate(motor_joint_order)}

    pelvis_body = None
    for worldbody in root.findall("worldbody"):
        for body in worldbody.findall("body"):
            if body.attrib.get("name") == "pelvis":
                pelvis_body = body
                break
        if pelvis_body is not None:
            break
    if pelvis_body is None:
        raise ValueError(f"could not find pelvis body in {mjcf_path}")

    nodes: list[_BodyNode] = []
    body_name_to_idx: dict[str, int] = {}

    def visit(body: ET.Element, parent_idx: int) -> None:
        name = body.attrib.get("name")
        if not name:
            return

        joint_motor_idx = None
        joint_axis = None
        for joint in body.findall("joint"):
            joint_type = joint.attrib.get("type", "hinge")
            joint_name = joint.attrib.get("name")
            if joint_type == "free" or joint_name not in joint_to_motor_idx:
                continue
            joint_motor_idx = joint_to_motor_idx[joint_name]
            joint_axis = _parse_vec3(joint.attrib.get("axis", "0 0 1"))
            norm = float(np.linalg.norm(joint_axis))
            if norm > 1e-8:
                joint_axis = (joint_axis / norm).astype(np.float32)
            else:
                joint_axis = np.array([0.0, 0.0, 1.0], dtype=np.float32)
            break

        idx = len(nodes)
        body_name_to_idx[name] = idx
        nodes.append(
            _BodyNode(
                name=name,
                parent=parent_idx,
                local_pos=_parse_vec3(body.attrib.get("pos", "0 0 0")),
                local_quat_wxyz=_parse_quat_wxyz(body.attrib.get("quat", "1 0 0 0")),
                joint_motor_idx=joint_motor_idx,
                joint_axis=joint_axis,
            )
        )
        for child in body.findall("body"):
            visit(child, idx)

    visit(pelvis_body, -1)
    missing = [name for name in SONIC_BODY_NAMES if name not in body_name_to_idx]
    if missing:
        raise ValueError(f"{mjcf_path} is missing SONIC body names: {missing}")
    return nodes, body_name_to_idx


def _parse_vec3(value: str) -> np.ndarray:
    parts = [float(part) for part in value.split()]
    if len(parts) != 3:
        raise ValueError(f"expected 3 floats, got {value!r}")
    return np.asarray(parts, dtype=np.float32)


def _parse_quat_wxyz(value: str) -> np.ndarray:
    parts = [float(part) for part in value.split()]
    if len(parts) != 4:
        raise ValueError(f"expected 4 floats, got {value!r}")
    return normalize_quat_wxyz(np.asarray(parts, dtype=np.float32))


def _validate_fk_inputs(
    joint_pos_isaaclab: np.ndarray,
    root_pos_w: np.ndarray,
    root_quat_wxyz: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    joint_pos_isaaclab = np.asarray(joint_pos_isaaclab, dtype=np.float32)
    root_pos_w = np.asarray(root_pos_w, dtype=np.float32)
    root_quat_wxyz = np.asarray(root_quat_wxyz, dtype=np.float32)
    if joint_pos_isaaclab.ndim != 2 or joint_pos_isaaclab.shape[1] != 29:
        raise ValueError(
            "joint_pos_isaaclab must have shape (T, 29), "
            f"got {joint_pos_isaaclab.shape}"
        )
    if root_pos_w.shape != (joint_pos_isaaclab.shape[0], 3):
        raise ValueError(
            f"root_pos_w must have shape ({joint_pos_isaaclab.shape[0]}, 3), "
            f"got {root_pos_w.shape}"
        )
    if root_quat_wxyz.shape != (joint_pos_isaaclab.shape[0], 4):
        raise ValueError(
            f"root_quat_wxyz must have shape ({joint_pos_isaaclab.shape[0]}, 4), "
            f"got {root_quat_wxyz.shape}"
        )
    return joint_pos_isaaclab, root_pos_w, root_quat_wxyz
