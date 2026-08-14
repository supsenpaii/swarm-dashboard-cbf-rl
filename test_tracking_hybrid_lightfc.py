import unittest

import numpy as np

import tracking_web  # Adds the configured lfc_gimbal_gazebo package to sys.path.
from tracking_hybrid import HybridMemoryTrackerService


BBox = tracking_web.BBox


@unittest.skipIf(BBox is None, "lfc_gimbal_gazebo tracking package unavailable")
class HybridLightFCGateTests(unittest.TestCase):
    def setUp(self):
        self.tracker = HybridMemoryTrackerService("KCF")

    def test_psr_separates_a_peak_from_a_flat_response(self):
        response = np.zeros((16, 16), dtype=np.float32)
        response[7, 8] = 1.0
        self.assertGreater(self.tracker._psr(response), 10.0)
        self.assertEqual(self.tracker._psr(np.ones((8, 8), np.float32)), 0.0)

    def test_shape_score_penalizes_large_bbox_change(self):
        reference = BBox(20.0, 30.0, 40.0, 20.0)
        same = BBox(21.0, 31.0, 40.0, 20.0)
        changed = BBox(20.0, 30.0, 75.0, 8.0)
        self.assertAlmostEqual(self.tracker._shape_score(same, reference), 1.0)
        self.assertLess(
            self.tracker._shape_score(changed, reference),
            self.tracker._shape_score(same, reference),
        )

    def test_implausible_scale_and_center_jump_are_rejected(self):
        reference = BBox(100.0, 100.0, 30.0, 20.0)
        valid, reason = self.tracker._plausible_transition(
            BBox(102.0, 101.0, 31.0, 21.0),
            reference,
            (117.0, 111.0),
        )
        self.assertTrue(valid)
        self.assertEqual(reason, "")

        valid, reason = self.tracker._plausible_transition(
            BBox(100.0, 100.0, 80.0, 20.0),
            reference,
            (117.0, 111.0),
        )
        self.assertFalse(valid)
        self.assertEqual(reason, "implausible_scale_change")

        valid, reason = self.tracker._plausible_transition(
            BBox(400.0, 300.0, 30.0, 20.0),
            reference,
            (117.0, 111.0),
        )
        self.assertFalse(valid)
        self.assertEqual(reason, "implausible_center_jump")

    def test_redetect_search_is_motion_local_then_expands_full_frame(self):
        gray = np.zeros((720, 1280), dtype=np.uint8)
        self.tracker._bbox = BBox(600.0, 330.0, 40.0, 30.0)
        self.tracker.lost_count = self.tracker.redetect_after
        search, origin, mode = self.tracker._redetect_search_region(
            gray,
            (620.0, 345.0),
            40,
            30,
        )
        self.assertEqual(mode, "motion_local")
        self.assertLess(search.shape[0], gray.shape[0])
        self.assertLess(search.shape[1], gray.shape[1])
        self.assertGreater(origin[0], 0)
        self.assertGreater(origin[1], 0)

        self.tracker.lost_count = self.tracker.redetect_full_after
        search, origin, mode = self.tracker._redetect_search_region(
            gray,
            (620.0, 345.0),
            40,
            30,
        )
        self.assertEqual(mode, "full_frame")
        self.assertEqual(search.shape, gray.shape)
        self.assertEqual(origin, (0, 0))

    def test_motion_direction_prefers_exit_velocity(self):
        self.tracker._bbox = BBox(100.0, 100.0, 20.0, 20.0)
        for _ in range(4):
            self.tracker.memory.update_velocity(5.0, 0.0)
        forward = self.tracker._direction_score(
            BBox(130.0, 100.0, 20.0, 20.0)
        )
        backward = self.tracker._direction_score(
            BBox(70.0, 100.0, 20.0, 20.0)
        )
        self.assertGreater(forward, backward)

    def test_diagnostics_identify_lightfc_gates_without_neural_core(self):
        diagnostics = self.tracker.diagnostics
        self.assertTrue(diagnostics["lightfc_robust_gates"])
        self.assertFalse(diagnostics["neural_core"])
        self.assertIn("psr", diagnostics)
        self.assertIn("ambiguous", diagnostics)
        self.assertIn("reject_reason", diagnostics)

    def test_synthetic_kcf_frame_passes_combined_quality_gates(self):
        rng = np.random.default_rng(7)
        patch = rng.integers(0, 256, (36, 48, 3), dtype=np.uint8)
        first = np.zeros((240, 320, 3), dtype=np.uint8)
        second = np.zeros_like(first)
        first[90:126, 110:158] = patch
        second[93:129, 115:163] = patch

        self.tracker.init(first, BBox(110.0, 90.0, 48.0, 36.0))
        result = self.tracker.update(second)

        self.assertEqual(result.state.value, "tracking")
        self.assertIsNotNone(result.bbox)
        self.assertFalse(result.redetecting)
        self.assertEqual(self.tracker.diagnostics["reject_reason"], "")


if __name__ == "__main__":
    unittest.main()
