"""Line-oriented control source for non-VR teleop managers."""

from __future__ import annotations

import select
import sys
from dataclasses import dataclass, field

import numpy as np


@dataclass
class PlannerControlState:
    enabled: bool = True
    stop_requested: bool = False
    mode: int = 0
    movement: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=np.float32))
    facing: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0], dtype=np.float32))
    speed: float = -1.0
    height: float = -1.0
    toggle_data_collection: bool = False
    toggle_data_abort: bool = False


class LineControlSource:
    """Poll stdin for simple text commands without taking over the terminal."""

    def __init__(self, auto_start: bool = True):
        self.state = PlannerControlState(enabled=auto_start)

    def poll(self) -> PlannerControlState:
        self.state.toggle_data_collection = False
        self.state.toggle_data_abort = False
        while self._line_ready():
            line = sys.stdin.readline()
            if not line:
                break
            self._apply(line.strip())
        return self.state

    def _line_ready(self) -> bool:
        try:
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        except (OSError, ValueError):
            return False
        return bool(readable)

    def _apply(self, line: str) -> None:
        if not line:
            return
        parts = line.split()
        cmd = parts[0].lower()

        if cmd in {"q", "quit", "exit", "stop"}:
            self.state.stop_requested = True
            self.state.enabled = False
        elif cmd in {"start", "resume", "on"}:
            self.state.enabled = True
        elif cmd in {"pause", "off"}:
            self.state.enabled = False
        elif cmd == "idle":
            self.state.mode = 0
            self.state.movement[:] = 0.0
            self.state.speed = -1.0
        elif cmd == "mode" and len(parts) >= 2:
            self.state.mode = int(parts[1])
        elif cmd == "move" and len(parts) >= 4:
            self.state.movement = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
        elif cmd == "face" and len(parts) >= 4:
            facing = np.array([float(parts[1]), float(parts[2]), float(parts[3])], dtype=np.float32)
            norm = float(np.linalg.norm(facing[:2]))
            if norm > 1e-6:
                facing[:2] /= norm
            self.state.facing = facing
        elif cmd == "speed" and len(parts) >= 2:
            self.state.speed = float(parts[1])
        elif cmd == "height" and len(parts) >= 2:
            self.state.height = float(parts[1])
        elif cmd in {"dc", "collect"}:
            self.state.toggle_data_collection = True
        elif cmd in {"abort", "da"}:
            self.state.toggle_data_abort = True
