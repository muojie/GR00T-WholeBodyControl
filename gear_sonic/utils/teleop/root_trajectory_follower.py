"""Online root-trajectory -> planner locomotion command follower.

Causal port of the BVH ``--follow-trajectory`` conversion
(branch ``bvh-trajectory-follow``, ``bvh_fullbody_pose_sender.py``
``_compute_root_trajectory``): the offline batch version differentiates the
whole root path up front; this version consumes streamed skeleton frames one
at a time and keeps the same behaviour — EMA-smoothed ground-plane velocity,
first-move heading alignment to +X, rate-limited stable heading, and a
loop-wrap guard for looped senders whose root teleports at the wrap.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


FACING_SOURCES = ("travel", "root_yaw")


@dataclass
class RootFollowCommand:
    mode: int
    movement: np.ndarray
    facing: np.ndarray
    speed: float
    moving: bool = False


@dataclass
class StreamRootTrajectoryFollower:
    """Convert streamed root poses into planner mode/movement/facing/speed.

    facing_source:
      - "travel": facing follows the direction of travel (matches the BVH
        follow-trajectory behaviour; no strafing, in-place turns are held).
      - "root_yaw": facing follows the reference root yaw (reproduces
        in-place turns and allows strafing/backstepping; experimental).
    """

    facing_source: str = "travel"
    speed_scale: float = 1.0
    speed_min: float = 0.1
    speed_max: float = 0.6
    move_threshold_mps: float = 0.08
    max_yaw_rate_radps: float = 1.5
    velocity_smooth_alpha: float = 0.2
    wrap_jump_m: float = 0.5
    idle_mode: int = 0
    walk_mode: int = 1

    _last_pos_xy: np.ndarray | None = field(default=None, init=False, repr=False)
    _last_time_s: float | None = field(default=None, init=False, repr=False)
    _last_frame_key: Any = field(default=None, init=False, repr=False)
    _vel_ema: np.ndarray = field(
        default_factory=lambda: np.zeros(2, dtype=np.float64), init=False, repr=False
    )
    _align_yaw: float | None = field(default=None, init=False, repr=False)
    _align_root_yaw: float = field(default=0.0, init=False, repr=False)
    _root_yaw_raw_prev: float | None = field(default=None, init=False, repr=False)
    _root_yaw_unwrapped: float = field(default=0.0, init=False, repr=False)
    _heading: float = field(default=0.0, init=False, repr=False)
    _last_cmd: RootFollowCommand = field(init=False, repr=False)
    _wrap_hold_frames: int = field(default=0, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.facing_source not in FACING_SOURCES:
            raise ValueError(
                f"unsupported facing_source {self.facing_source!r}; expected {FACING_SOURCES}"
            )
        self._last_cmd = self._idle_command()

    @property
    def diagnostics(self) -> dict[str, float]:
        return {
            "follow_speed_mps": float(self._last_cmd.speed if self._last_cmd.moving else 0.0),
            "follow_heading_deg": float(np.degrees(self._heading)),
            "follow_moving": float(self._last_cmd.moving),
            "follow_aligned": float(self._align_yaw is not None),
        }

    def update(
        self,
        root_pos: np.ndarray,
        root_quat_wxyz: np.ndarray,
        *,
        time_s: float,
        frame_key: Any = None,
    ) -> RootFollowCommand:
        """Feed one streamed root pose; returns the current planner command.

        Repeated frames (same frame_key) return the previous command unchanged.
        """
        if frame_key is not None and frame_key == self._last_frame_key:
            return self._last_cmd
        self._last_frame_key = frame_key

        pos_xy = np.asarray(root_pos, dtype=np.float64).reshape(-1)[:2]
        if self._last_pos_xy is None or self._last_time_s is None:
            self._last_pos_xy = pos_xy
            self._last_time_s = float(time_s)
            return self._last_cmd

        dt = float(time_s) - self._last_time_s
        if dt <= 0.0:
            return self._last_cmd
        dt = min(max(dt, 1e-3), 0.2)

        step = pos_xy - self._last_pos_xy
        self._last_pos_xy = pos_xy
        self._last_time_s = float(time_s)

        step_norm = float(np.linalg.norm(step))
        if step_norm > self.wrap_jump_m:
            # Looped sender wrapped (root teleports back to the clip start):
            # skip the velocity spike and coast on held state for a few frames.
            self._wrap_hold_frames = 5
            self._vel_ema[:] = 0.0
            return self._last_cmd
        if self._wrap_hold_frames > 0:
            self._wrap_hold_frames -= 1
            return self._last_cmd

        vel = step / dt
        alpha = float(np.clip(self.velocity_smooth_alpha, 0.0, 1.0))
        self._vel_ema = (1.0 - alpha) * self._vel_ema + alpha * vel
        raw_speed = float(np.linalg.norm(self._vel_ema))
        moving = raw_speed > self.move_threshold_mps

        # First-move alignment: lock the frame rotation so the first
        # meaningfully-moving direction maps to robot +X (robot starts facing +X).
        if self._align_yaw is None:
            if not moving:
                return self._last_cmd
            self._align_yaw = float(np.arctan2(self._vel_ema[1], self._vel_ema[0]))
            # Root-yaw facing needs its own zero so that at the first moving
            # frame facing == +X regardless of how the reference root yaw
            # relates to the direction of travel.
            self._align_root_yaw = self._root_yaw(root_quat_wxyz)

        cos_a = np.cos(-self._align_yaw)
        sin_a = np.sin(-self._align_yaw)
        vel_aligned = np.array(
            [
                cos_a * self._vel_ema[0] - sin_a * self._vel_ema[1],
                sin_a * self._vel_ema[0] + cos_a * self._vel_ema[1],
            ]
        )

        max_dyaw = self.max_yaw_rate_radps * dt
        if self.facing_source == "root_yaw":
            # Track the reference yaw on an unwrapped scale: a fast full spin
            # would otherwise lap the rate-limited heading and the wrapped
            # shortest-path delta would flip sign and oscillate.
            raw_yaw = self._root_yaw(root_quat_wxyz)
            if self._root_yaw_raw_prev is None:
                self._root_yaw_unwrapped = raw_yaw
            else:
                self._root_yaw_unwrapped += (
                    (raw_yaw - self._root_yaw_raw_prev + np.pi) % (2.0 * np.pi) - np.pi
                )
            self._root_yaw_raw_prev = raw_yaw
            heading_target = self._root_yaw_unwrapped - self._align_root_yaw
            delta = heading_target - self._heading
            self._heading += float(np.clip(delta, -max_dyaw, max_dyaw))
        elif moving:
            # Only steer while moving: near-zero velocity direction is noise and
            # chasing it makes the robot spin in place.
            heading_target = float(np.arctan2(vel_aligned[1], vel_aligned[0]))
            delta = (heading_target - self._heading + np.pi) % (2.0 * np.pi) - np.pi
            self._heading += float(np.clip(delta, -max_dyaw, max_dyaw))

        facing = np.array(
            [np.cos(self._heading), np.sin(self._heading), 0.0], dtype=np.float32
        )
        if not moving:
            cmd = RootFollowCommand(
                mode=self.idle_mode,
                movement=np.zeros(3, dtype=np.float32),
                facing=facing,
                speed=-1.0,
                moving=False,
            )
            self._last_cmd = cmd
            return cmd

        if self.facing_source == "root_yaw":
            direction = vel_aligned / max(raw_speed, 1e-8)
            movement = np.array([direction[0], direction[1], 0.0], dtype=np.float32)
        else:
            movement = facing.copy()

        speed = float(
            np.clip(raw_speed * self.speed_scale, self.speed_min, self.speed_max)
        )
        cmd = RootFollowCommand(
            mode=self.walk_mode,
            movement=movement,
            facing=facing,
            speed=speed,
            moving=True,
        )
        self._last_cmd = cmd
        return cmd

    def _idle_command(self) -> RootFollowCommand:
        return RootFollowCommand(
            mode=self.idle_mode,
            movement=np.zeros(3, dtype=np.float32),
            facing=np.array([1.0, 0.0, 0.0], dtype=np.float32),
            speed=-1.0,
            moving=False,
        )

    @staticmethod
    def _root_yaw(quat_wxyz: np.ndarray) -> float:
        quat = np.asarray(quat_wxyz, dtype=np.float64).reshape(-1)
        w, x, y, z = quat[0], quat[1], quat[2], quat[3]
        return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
