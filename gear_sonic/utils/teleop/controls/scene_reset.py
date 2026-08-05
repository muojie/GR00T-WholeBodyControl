"""Safety gates and DDS publishing for operator-requested Isaac scene resets."""

from __future__ import annotations

import math
import os
from typing import Sequence


class ExclusiveXLongPress:
    """Emit once when a fresh, neutral, X-only input is held long enough."""

    def __init__(
        self,
        hold_seconds: float = 2.0,
        input_max_age: float = 0.5,
        joystick_deadzone: float = 0.15,
    ) -> None:
        if not math.isfinite(hold_seconds) or hold_seconds <= 0.0:
            raise ValueError("hold_seconds must be finite and positive")
        if not math.isfinite(input_max_age) or input_max_age <= 0.0:
            raise ValueError("input_max_age must be finite and positive")
        if not math.isfinite(joystick_deadzone) or joystick_deadzone < 0.0:
            raise ValueError("joystick_deadzone must be finite and non-negative")

        self.hold_seconds = float(hold_seconds)
        self.input_max_age = float(input_max_age)
        self.joystick_deadzone = float(joystick_deadzone)
        self._hold_started_at: float | None = None
        self._latched = False

    @property
    def latched(self) -> bool:
        """Whether this X press has already emitted and still awaits release."""

        return self._latched

    def update(
        self,
        *,
        now: float,
        x_pressed: bool | None,
        other_face_pressed: bool,
        axes: Sequence[float],
        sample_monotonic: float | None,
    ) -> bool:
        """Advance the gate and return ``True`` exactly once per valid hold."""

        if x_pressed is None:
            # Unknown/malformed input cancels arming but is not proof that the
            # operator released X.  Preserve a post-trigger latch until an
            # explicit valid release is observed.
            self._hold_started_at = None
            return False

        if not x_pressed:
            self._hold_started_at = None
            self._latched = False
            return False

        if self._latched:
            return False

        sample_age = math.inf
        if sample_monotonic is not None:
            sample_age = float(now) - float(sample_monotonic)
        input_is_fresh = 0.0 <= sample_age <= self.input_max_age
        axes_are_neutral = len(axes) == 4 and all(
            math.isfinite(float(axis))
            and abs(float(axis)) <= self.joystick_deadzone
            for axis in axes
        )
        eligible = not other_face_pressed and axes_are_neutral and input_is_fresh
        if not eligible:
            self._hold_started_at = None
            return False

        if self._hold_started_at is None:
            self._hold_started_at = float(now)
            return False
        if float(now) - self._hold_started_at < self.hold_seconds:
            return False

        self._hold_started_at = None
        self._latched = True
        return True


class IsaacSceneResetPublisher:
    """Publish the existing full-scene reset command on Unitree DDS."""

    TOPIC = "rt/reset_pose/cmd"
    FULL_SCENE_CATEGORY = "2"

    def __init__(self, domain: int | None = None, interface: str | None = None) -> None:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

        resolved_domain = self.resolve_domain(domain)
        resolved_interface = self.resolve_interface(interface)
        if resolved_interface:
            initialized = ChannelFactoryInitialize(resolved_domain, resolved_interface)
        else:
            initialized = ChannelFactoryInitialize(resolved_domain)
        if initialized is False:
            raise RuntimeError(
                "failed to initialize Unitree DDS "
                f"domain={resolved_domain} interface={resolved_interface or 'auto'}"
            )

        self.domain = resolved_domain
        self.interface = resolved_interface
        self._string_type = String_
        self._publisher = ChannelPublisher(self.TOPIC, String_)
        self._publisher.Init()
        print(
            "[SceneReset] DDS publisher ready: "
            f"topic={self.TOPIC} domain={self.domain} "
            f"interface={self.interface or 'auto'}"
        )

    @staticmethod
    def resolve_domain(value: int | None) -> int:
        raw_value = os.environ.get("UNITREE_DDS_DOMAIN", "1") if value is None else value
        try:
            domain = int(raw_value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid Unitree DDS domain: {raw_value!r}") from exc
        if domain < 0:
            raise ValueError("Unitree DDS domain must be non-negative")
        return domain

    @staticmethod
    def resolve_interface(value: str | None) -> str | None:
        raw_value = os.environ.get("UNITREE_DDS_INTERFACE", "lo") if value is None else value
        interface = str(raw_value).strip()
        if interface.lower() == "auto":
            return None
        return interface or None

    def publish_full_scene_reset(self) -> bool:
        try:
            result = self._publisher.Write(
                self._string_type(data=self.FULL_SCENE_CATEGORY)
            )
            if result is False:
                raise RuntimeError("DDS writer returned false")
            print(
                "[SceneReset] published full-scene reset: "
                f"topic={self.TOPIC} category={self.FULL_SCENE_CATEGORY}"
            )
            return True
        except Exception as exc:
            print(f"[SceneReset] ERROR: failed to publish full-scene reset: {exc}")
            return False

    def close(self) -> None:
        try:
            self._publisher.Close()
        except Exception as exc:
            print(f"[SceneReset] WARNING: failed to close DDS publisher: {exc}")
