"""Utilities for converting mocap frames into deploy-side VR 3-point targets."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from gear_sonic.utils.teleop.sources.base import MocapFrame, Pose7D


DEFAULT_VR_POSITION = np.array(
    [
        0.0903,
        0.1615,
        -0.2411,
        0.1280,
        -0.1522,
        -0.2461,
        0.0241,
        -0.0081,
        0.4028,
    ],
    dtype=np.float32,
)
DEFAULT_VR_ORIENTATION = np.array(
    [
        0.7295,
        0.3145,
        0.5533,
        -0.2506,
        0.7320,
        -0.2639,
        0.5395,
        0.3217,
        0.9991,
        0.011,
        0.0402,
        -0.0002,
    ],
    dtype=np.float32,
)


@dataclass
class VR3PointTarget:
    position: np.ndarray
    orientation: np.ndarray


class VR3PointRetargeter:
    """Extract and lightly calibrate left wrist, right wrist, and head targets."""

    def __init__(
        self,
        calibrate_on_first_frame: bool = True,
        position_scale: float = 1.0,
        allow_bone_translation_vr: bool = False,
    ):
        self.calibrate_on_first_frame = bool(calibrate_on_first_frame)
        self.position_scale = float(position_scale)
        self.allow_bone_translation_vr = bool(allow_bone_translation_vr)
        self._position_offset: np.ndarray | None = None

    def reset(self) -> None:
        self._position_offset = None

    def build_target(self, frame: MocapFrame) -> VR3PointTarget | None:
        if frame.direct_vr_position is not None:
            orientation = (
                frame.direct_vr_orientation.copy()
                if frame.direct_vr_orientation is not None
                else DEFAULT_VR_ORIENTATION.copy()
            )
            return VR3PointTarget(position=frame.direct_vr_position.copy(), orientation=orientation)

        source_position, source_orientation = self._extract_from_joints(frame)
        if source_position is None:
            return None

        position = source_position * self.position_scale
        if self.calibrate_on_first_frame:
            if self._position_offset is None:
                self._position_offset = DEFAULT_VR_POSITION - position
            position = position + self._position_offset

        orientation = (
            source_orientation.copy()
            if source_orientation is not None
            else DEFAULT_VR_ORIENTATION.copy()
        )
        return VR3PointTarget(position=position.astype(np.float32), orientation=orientation)

    def _extract_from_joints(
        self, frame: MocapFrame
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        left = _find_joint(frame, ("left_wrist", "left_hand", "l_hand", "left_controller"))
        right = _find_joint(frame, ("right_wrist", "right_hand", "r_hand", "right_controller"))
        head = _find_joint(frame, ("head", "neck", "neck_2", "head_tracker"))

        if self.allow_bone_translation_vr and (left is None or right is None):
            left = left or frame.bones.get(14)
            right = right or frame.bones.get(18)
            head = head or frame.bones.get(10) or frame.bones.get(9)

        if left is None or right is None:
            return None, None

        position = DEFAULT_VR_POSITION.copy()
        orientation = DEFAULT_VR_ORIENTATION.copy()
        position[:3] = left.position
        position[3:6] = right.position
        orientation[:4] = left.quat_wxyz
        orientation[4:8] = right.quat_wxyz

        if head is not None:
            position[6:9] = head.position
            orientation[8:12] = head.quat_wxyz

        return position, orientation


def _find_joint(frame: MocapFrame, aliases: tuple[str, ...]) -> Pose7D | None:
    for alias in aliases:
        pose = frame.joints.get(alias)
        if pose is not None:
            return pose
    return None
