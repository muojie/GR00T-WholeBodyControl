"""Shared data types for teleoperation motion sources."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np


def _vector(name: str, value: Any, length: int) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float32)
    if arr.shape != (length,):
        raise ValueError(f"{name} must have shape ({length},), got {arr.shape}")
    return arr


def normalize_quat_wxyz(quat_wxyz: Any) -> np.ndarray:
    """Return a normalized wxyz quaternion, falling back to identity for bad input."""
    quat = _vector("quat_wxyz", quat_wxyz, 4)
    norm = float(np.linalg.norm(quat))
    if norm < 1e-8 or not np.isfinite(norm):
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    return (quat / norm).astype(np.float32)


@dataclass
class Pose7D:
    """Position plus quaternion in the repository's VR 3-point convention."""

    position: np.ndarray
    quat_wxyz: np.ndarray

    def __post_init__(self) -> None:
        self.position = _vector("position", self.position, 3)
        self.quat_wxyz = normalize_quat_wxyz(self.quat_wxyz)

    def as_pose7(self) -> np.ndarray:
        return np.concatenate((self.position, self.quat_wxyz)).astype(np.float32)


@dataclass
class MocapFrame:
    """Canonical frame passed from motion-capture sources to teleop managers."""

    source: str
    host_time_s: float
    source_time_ns: int | None = None
    frame_index: int | None = None
    fps: float = 0.0
    joints: dict[str, Pose7D] = field(default_factory=dict)
    bones: dict[int, Pose7D] = field(default_factory=dict)
    direct_vr_position: np.ndarray | None = None
    direct_vr_orientation: np.ndarray | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.direct_vr_position is not None:
            self.direct_vr_position = _vector("direct_vr_position", self.direct_vr_position, 9)
        if self.direct_vr_orientation is not None:
            self.direct_vr_orientation = _vector(
                "direct_vr_orientation", self.direct_vr_orientation, 12
            )


class MocapSource(Protocol):
    """Minimal interface for threaded or polled mocap sources."""

    def start(self) -> None:
        ...

    def stop(self) -> None:
        ...

    def get_latest(self) -> MocapFrame | None:
        ...
