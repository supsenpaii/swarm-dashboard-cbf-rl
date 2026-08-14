import math
import unittest

from pose_time_sync import PoseSample, TimestampedPoseBuffer


def sample(timestamp, x, yaw_deg=0.0):
    half = math.radians(yaw_deg) / 2.0
    return PoseSample(
        timestamp_s=timestamp,
        position_ned_m=(x, 2.0 * x, -3.0),
        velocity_ned_m_s=(1.0, 2.0, 0.0),
        quaternion_xyzw=(0.0, 0.0, math.sin(half), math.cos(half)),
    )


class TimestampedPoseBufferTests(unittest.TestCase):
    def test_exact_timestamp(self):
        buffer = TimestampedPoseBuffer()
        buffer.append(sample(1.0, 4.0))
        result = buffer.sample_at(1.0)
        self.assertTrue(result.valid)
        self.assertEqual(result.position_ned_m, (4.0, 8.0, -3.0))
        self.assertEqual(result.reason, "exact")

    def test_interpolates_position_velocity_and_orientation(self):
        buffer = TimestampedPoseBuffer(max_interpolation_gap_s=2.0)
        buffer.append(sample(1.0, 0.0, 0.0))
        buffer.append(sample(2.0, 2.0, 90.0))
        result = buffer.sample_at(1.5)
        self.assertTrue(result.valid)
        self.assertTrue(result.interpolated)
        self.assertEqual(result.position_ned_m, (1.0, 2.0, -3.0))
        self.assertAlmostEqual(result.quaternion_xyzw[2], math.sin(math.pi / 8))
        self.assertAlmostEqual(result.quaternion_xyzw[3], math.cos(math.pi / 8))
        self.assertEqual(result.sample_offset_s, 0.0)
        self.assertEqual(result.lower_sample_delta_s, 0.5)
        self.assertEqual(result.upper_sample_delta_s, 0.5)

    def test_stale_nearest_sample_is_rejected(self):
        buffer = TimestampedPoseBuffer(max_nearest_age_s=0.1)
        buffer.append(sample(1.0, 0.0))
        result = buffer.sample_at(1.2)
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "sample_stale")
        self.assertAlmostEqual(result.sample_age_s, 0.2)
        self.assertAlmostEqual(result.sample_timestamp_s, 1.0)
        self.assertAlmostEqual(result.sample_offset_s, -0.2)

    def test_large_interpolation_gap_is_rejected(self):
        buffer = TimestampedPoseBuffer(max_interpolation_gap_s=0.2)
        buffer.append(sample(1.0, 0.0))
        buffer.append(sample(2.0, 1.0))
        result = buffer.sample_at(1.5)
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "interpolation_gap_too_large")

    def test_out_of_order_sample_is_rejected(self):
        buffer = TimestampedPoseBuffer()
        self.assertTrue(buffer.append(sample(2.0, 0.0)))
        self.assertFalse(buffer.append(sample(1.0, 0.0)))
        self.assertEqual(buffer.rejected_out_of_order, 1)

    def test_missing_samples_and_invalid_timestamp(self):
        buffer = TimestampedPoseBuffer()
        self.assertEqual(buffer.sample_at(1.0).reason, "missing_samples")
        self.assertEqual(buffer.sample_at(math.nan).reason, "invalid_timestamp")

    def test_invalid_quaternion_is_rejected(self):
        buffer = TimestampedPoseBuffer()
        with self.assertRaises(ValueError):
            buffer.append(
                PoseSample(1.0, (0, 0, 0), (0, 0, 0), (0, 0, 0, 0))
            )


if __name__ == "__main__":
    unittest.main()
