from __future__ import annotations

import os
import unittest
from unittest import mock

from gear_sonic.utils.teleop.controls.scene_reset import (
    ExclusiveXLongPress,
    IsaacSceneResetPublisher,
)


class ExclusiveXLongPressTest(unittest.TestCase):
    def setUp(self) -> None:
        self.gate = ExclusiveXLongPress(
            hold_seconds=2.0,
            input_max_age=0.5,
            joystick_deadzone=0.15,
        )

    def update(
        self,
        now: float,
        *,
        x: bool = True,
        other: bool = False,
        axes=(0.0, 0.0, 0.0, 0.0),
        sample_age: float = 0.0,
    ) -> bool:
        return self.gate.update(
            now=now,
            x_pressed=x,
            other_face_pressed=other,
            axes=axes,
            sample_monotonic=now - sample_age,
        )

    def test_emits_once_at_threshold_and_rearms_only_after_release(self) -> None:
        self.assertFalse(self.update(10.0))
        self.assertFalse(self.update(11.99))
        self.assertTrue(self.update(12.0))
        self.assertTrue(self.gate.latched)
        self.assertFalse(self.update(15.0))

        self.assertFalse(self.update(15.1, x=False))
        self.assertFalse(self.gate.latched)
        self.assertFalse(self.update(20.0))
        self.assertTrue(self.update(22.0))

    def test_other_face_button_cancels_the_hold(self) -> None:
        self.assertFalse(self.update(1.0))
        self.assertFalse(self.update(2.5, other=True))
        self.assertFalse(self.update(3.0))
        self.assertFalse(self.update(4.99))
        self.assertTrue(self.update(5.0))

    def test_non_neutral_or_non_finite_axis_cancels_the_hold(self) -> None:
        self.assertFalse(self.update(1.0))
        self.assertFalse(self.update(2.0, axes=(0.16, 0.0, 0.0, 0.0)))
        self.assertFalse(self.update(3.0))
        self.assertFalse(self.update(5.0, axes=(float("nan"), 0.0, 0.0, 0.0)))
        self.assertFalse(self.update(6.0))
        self.assertTrue(self.update(8.0))

    def test_stale_or_future_sample_cannot_trigger(self) -> None:
        self.assertFalse(self.update(1.0, sample_age=0.6))
        self.assertFalse(self.update(4.0, sample_age=0.6))
        self.assertFalse(self.update(5.0, sample_age=-0.1))
        self.assertFalse(self.update(6.0))
        self.assertTrue(self.update(8.0))

    def test_unknown_input_cancels_arming_without_releasing_a_latch(self) -> None:
        self.assertFalse(self.update(1.0))
        self.assertFalse(
            self.gate.update(
                now=2.0,
                x_pressed=None,
                other_face_pressed=False,
                axes=(),
                sample_monotonic=None,
            )
        )
        self.assertFalse(self.update(3.0))
        self.assertTrue(self.update(5.0))

        self.assertFalse(
            self.gate.update(
                now=6.0,
                x_pressed=None,
                other_face_pressed=False,
                axes=(),
                sample_monotonic=None,
            )
        )
        self.assertTrue(self.gate.latched)
        self.assertFalse(self.update(7.0))
        self.assertFalse(self.update(8.0, x=False))
        self.assertFalse(self.gate.latched)


class _FakeString:
    def __init__(self, data: str) -> None:
        self.data = data


class _FakePublisher:
    def __init__(self, result=True, error: Exception | None = None) -> None:
        self.result = result
        self.error = error
        self.messages = []

    def Write(self, message):
        if self.error is not None:
            raise self.error
        self.messages.append(message)
        return self.result


class IsaacSceneResetPublisherTest(unittest.TestCase):
    def make_publisher(self, fake: _FakePublisher) -> IsaacSceneResetPublisher:
        publisher = IsaacSceneResetPublisher.__new__(IsaacSceneResetPublisher)
        publisher._publisher = fake
        publisher._string_type = _FakeString
        return publisher

    def test_publishes_category_two_once(self) -> None:
        fake = _FakePublisher()
        publisher = self.make_publisher(fake)

        self.assertTrue(publisher.publish_full_scene_reset())
        self.assertEqual([message.data for message in fake.messages], ["2"])

    def test_publish_failure_is_reported_without_raising(self) -> None:
        publisher = self.make_publisher(
            _FakePublisher(error=RuntimeError("injected failure"))
        )
        self.assertFalse(publisher.publish_full_scene_reset())

    def test_domain_and_interface_resolve_from_environment(self) -> None:
        with mock.patch.dict(
            os.environ,
            {"UNITREE_DDS_DOMAIN": "7", "UNITREE_DDS_INTERFACE": "lo"},
        ):
            self.assertEqual(IsaacSceneResetPublisher.resolve_domain(None), 7)
            self.assertEqual(IsaacSceneResetPublisher.resolve_interface(None), "lo")
        self.assertIsNone(IsaacSceneResetPublisher.resolve_interface("auto"))


if __name__ == "__main__":
    unittest.main()
