from __future__ import annotations

import math
import unittest
from types import SimpleNamespace

from mavlink_manual_bridge import FAST_POSE_MAX_SOURCE_AGE_S, FastPoseCache


class FakeMessage(SimpleNamespace):
    def get_type(self) -> str:
        return str(self.message_type)


class FastPoseCacheTests(unittest.TestCase):
    def test_home_position_caches_local_z_reference(self) -> None:
        cache = FastPoseCache()
        cache.update(
            FakeMessage(
                message_type="HOME_POSITION",
                z=-1.25,
            ),
            1.0,
        )
        self.assertEqual(cache.home_z_down_m, -1.25)

    def test_direct_px4_quaternion_and_local_pose_form_payload(self) -> None:
        cache = FastPoseCache()
        cache.update(
            FakeMessage(
                message_type="LOCAL_POSITION_NED",
                x=1.0,
                y=-2.0,
                z=-8.5,
                vx=0.2,
                vy=-0.1,
                vz=0.0,
            ),
            10.0,
        )
        yaw_half = math.radians(90.0) * 0.5
        cache.update(
            FakeMessage(
                message_type="ATTITUDE_QUATERNION",
                q1=math.cos(yaw_half),
                q2=0.0,
                q3=0.0,
                q4=math.sin(yaw_half),
            ),
            10.01,
        )

        payload = cache.payload("UAV-02", 10.02)

        self.assertIsNotNone(payload)
        assert payload is not None
        self.assertEqual(payload["source"], "px4_mavlink")
        self.assertEqual(
            payload["local_position"]["x_north_m"],
            1.0,
        )
        self.assertAlmostEqual(
            payload["local_position"]["heading_rad"],
            math.pi / 2.0,
        )
        self.assertAlmostEqual(
            payload["quaternion_xyzw"][2],
            math.sin(yaw_half),
        )
        self.assertAlmostEqual(
            payload["quaternion_xyzw"][3],
            math.cos(yaw_half),
        )

    def test_direct_quaternion_is_not_overwritten_by_euler_fallback(self) -> None:
        cache = FastPoseCache()
        cache.update(
            FakeMessage(
                message_type="ATTITUDE_QUATERNION",
                q1=1.0,
                q2=0.0,
                q3=0.0,
                q4=0.0,
            ),
            5.0,
        )
        cache.update(
            FakeMessage(
                message_type="ATTITUDE",
                roll=0.0,
                pitch=0.0,
                yaw=1.0,
            ),
            5.01,
        )
        self.assertEqual(cache.quaternion_xyzw, (0.0, 0.0, 0.0, 1.0))

    def test_euler_fallback_uses_exact_quaternion_composition(self) -> None:
        cache = FastPoseCache()
        cache.update(
            FakeMessage(
                message_type="ATTITUDE",
                roll=math.radians(20.0),
                pitch=math.radians(-10.0),
                yaw=math.radians(30.0),
            ),
            7.0,
        )
        assert cache.quaternion_xyzw is not None
        norm = math.sqrt(
            sum(value * value for value in cache.quaternion_xyzw)
        )
        self.assertAlmostEqual(norm, 1.0)
        self.assertNotEqual(cache.quaternion_xyzw[0], 0.0)
        self.assertNotEqual(cache.quaternion_xyzw[1], 0.0)

    def test_stale_or_future_source_is_rejected(self) -> None:
        cache = FastPoseCache(
            position_ned_m=(0.0, 0.0, -8.0),
            velocity_ned_m_s=(0.0, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            local_received_monotonic_s=3.0,
            attitude_received_monotonic_s=3.0,
        )
        self.assertIsNone(
            cache.payload(
                "UAV-02",
                3.0 + FAST_POSE_MAX_SOURCE_AGE_S + 0.01,
            )
        )
        self.assertIsNone(cache.payload("UAV-02", 2.99))


if __name__ == "__main__":
    unittest.main()
