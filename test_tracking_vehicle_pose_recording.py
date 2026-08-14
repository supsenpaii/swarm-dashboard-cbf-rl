from __future__ import annotations

import math
import unittest

import main


class TrackingVehiclePoseRecordingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.drone_id = "UAV-02"
        main.tracking_vehicle_pose_buffers[self.drone_id].clear()

    def tearDown(self) -> None:
        main.tracking_vehicle_pose_buffers[self.drone_id].clear()

    def test_direct_quaternion_contract_is_preserved_and_normalized(self) -> None:
        half_yaw = math.radians(90.0) * 0.5
        main.record_tracking_vehicle_pose(
            self.drone_id,
            {
                "local_position": {
                    "x_north_m": 1.0,
                    "y_east_m": 2.0,
                    "z_down_m": -9.0,
                    "vx_m_s": 0.1,
                    "vy_m_s": 0.2,
                    "vz_m_s": 0.0,
                    "heading_rad": 0.0,
                },
                "quaternion_xyzw": [
                    0.0,
                    0.0,
                    2.0 * math.sin(half_yaw),
                    2.0 * math.cos(half_yaw),
                ],
            },
            12.5,
        )

        result = main.tracking_vehicle_pose_buffers[
            self.drone_id
        ].sample_at(12.5)

        self.assertTrue(result.valid)
        assert result.quaternion_xyzw is not None
        self.assertAlmostEqual(
            result.quaternion_xyzw[2],
            math.sin(half_yaw),
        )
        self.assertAlmostEqual(
            result.quaternion_xyzw[3],
            math.cos(half_yaw),
        )


if __name__ == "__main__":
    unittest.main()
