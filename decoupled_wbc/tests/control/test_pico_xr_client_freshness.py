from __future__ import annotations

import threading
import unittest

from decoupled_wbc.control.teleop.device.pico.xr_client import XrClient
from gear_sonic.utils.teleop.controls.scene_reset import ExclusiveXLongPress


class XrClientFreshnessTest(unittest.TestCase):
    def make_udp_client(
        self,
        *,
        tracking: dict | None,
        received_at: float,
    ) -> XrClient:
        client = XrClient.__new__(XrClient)
        client.transport = "udp"
        client._udp_lock = threading.Lock()
        client._latest_tracking = tracking
        client._latest_monotonic_time = received_at
        return client

    def test_missing_device_timestamp_uses_stable_receive_time(self) -> None:
        client = self.make_udp_client(
            tracking={"Body": {"joints": [{}]}},
            received_at=1234.5,
        )

        self.assertEqual(client.get_timestamp_ns(), 1_234_500_000_000)
        self.assertEqual(client.get_timestamp_ns(), 1_234_500_000_000)
        self.assertEqual(client.get_latest_receive_monotonic_time(), 1234.5)

    def test_packet_header_timestamp_precedes_receive_time_fallback(self) -> None:
        client = self.make_udp_client(
            tracking={
                "Body": {"joints": [{}]},
                "_packet_timestamp_ms": 987_654,
            },
            received_at=1234.5,
        )

        self.assertEqual(client.get_timestamp_ns(), 987_654_000_000)

    def test_no_udp_packet_has_no_freshness_timestamp(self) -> None:
        client = self.make_udp_client(tracking=None, received_at=0.0)

        self.assertEqual(client.get_timestamp_ns(), 0)
        self.assertIsNone(client.get_latest_receive_monotonic_time())

    def test_reset_snapshot_is_atomic_and_requires_every_safety_field(self) -> None:
        complete_tracking = {
            "Controller": {
                "left": {
                    "primaryButton": True,
                    "secondaryButton": False,
                    "axisX": 0.01,
                    "axisY": -0.02,
                },
                "right": {
                    "primaryButton": False,
                    "secondaryButton": False,
                    "axisX": 0.03,
                    "axisY": -0.04,
                },
            }
        }
        client = self.make_udp_client(tracking=complete_tracking, received_at=42.0)

        self.assertEqual(
            client.get_scene_reset_input_snapshot(),
            {
                "a_pressed": False,
                "b_pressed": False,
                "x_pressed": True,
                "y_pressed": False,
                "axes": (0.01, -0.02, 0.03, -0.04),
                "received_monotonic": 42.0,
            },
        )

        del complete_tracking["Controller"]["right"]["axisY"]
        self.assertIsNone(client.get_scene_reset_input_snapshot())

    def test_held_x_cannot_finish_after_udp_disconnect(self) -> None:
        client = self.make_udp_client(
            tracking={
                "Controller": {
                    "left": {
                        "primaryButton": True,
                        "secondaryButton": False,
                        "axisX": 0.0,
                        "axisY": 0.0,
                    },
                    "right": {
                        "primaryButton": False,
                        "secondaryButton": False,
                        "axisX": 0.0,
                        "axisY": 0.0,
                    },
                }
            },
            received_at=10.0,
        )
        gate = ExclusiveXLongPress(hold_seconds=2.0, input_max_age=0.5)

        def update(now: float) -> bool:
            snapshot = client.get_scene_reset_input_snapshot()
            assert snapshot is not None
            return gate.update(
                now=now,
                x_pressed=snapshot["x_pressed"],
                other_face_pressed=(
                    snapshot["a_pressed"]
                    or snapshot["b_pressed"]
                    or snapshot["y_pressed"]
                ),
                axes=snapshot["axes"],
                sample_monotonic=snapshot["received_monotonic"],
            )

        self.assertFalse(update(10.0))
        self.assertFalse(update(10.4))
        self.assertFalse(update(10.6))
        self.assertFalse(update(12.1))


if __name__ == "__main__":
    unittest.main()
