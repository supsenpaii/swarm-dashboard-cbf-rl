import unittest

import cv2
import numpy as np

import tracking_web
from tracking_hybrid import (
    HybridMemoryTrackerService,
    MultiScaleBBoxRefiner,
)
from visual_follow_target import VisualBBoxFilter


BBox = tracking_web.BBox


@unittest.skipIf(BBox is None, "tracking package unavailable")
class MultiScaleBBoxRefinerTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(21)
        self.patch = rng.integers(0, 256, (40, 60, 3), dtype=np.uint8)
        self.center = (160, 120)
        self.base_bbox = BBox(130.0, 100.0, 60.0, 40.0)
        self.refiner = MultiScaleBBoxRefiner(
            minimum_psr=0.0,
            minimum_gain=0.0,
        )
        self.refiner.reset(self.patch)

    def frame(self, scale, center=None, distractor=False):
        center = center or self.center
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        patch = cv2.resize(
            self.patch,
            (round(60 * scale), round(40 * scale)),
        )
        x0 = round(center[0] - patch.shape[1] / 2)
        y0 = round(center[1] - patch.shape[0] / 2)
        frame[y0:y0 + patch.shape[0], x0:x0 + patch.shape[1]] = patch
        if distractor:
            frame[15:55, 20:80] = self.patch
        return frame

    def refine(self, scale, previous=None, center=None, **kwargs):
        previous = previous or self.base_bbox
        center = center or self.center
        raw = BBox.from_center(
            center[0], center[1], previous.w, previous.h
        )
        return self.refiner.refine(
            self.frame(scale, center, **kwargs),
            raw,
            previous,
        )

    def test_growing_target_increases_bbox_area_monotonically(self):
        previous = self.base_bbox
        areas = []
        for scale in (1.08, 1.15, 1.24):
            result = self.refine(scale, previous)
            self.assertTrue(result.accepted)
            previous = result.bbox
            areas.append(previous.w * previous.h)
        self.assertEqual(areas, sorted(areas))
        self.assertGreater(areas[-1], areas[0])

    def test_shrinking_target_decreases_bbox_area_monotonically(self):
        previous = self.base_bbox
        areas = []
        for scale in (0.92, 0.85):
            result = self.refine(scale, previous)
            self.assertTrue(result.accepted)
            previous = result.bbox
            areas.append(previous.w * previous.h)
        self.assertEqual(areas, sorted(areas, reverse=True))

    def test_translation_does_not_change_scale(self):
        result = self.refine(1.0, center=(176, 127))
        self.assertTrue(result.accepted)
        self.assertAlmostEqual(result.scale_ratio, 1.0)
        self.assertAlmostEqual(result.bbox.w, self.base_bbox.w)

    def test_translation_and_scale_are_refined_together(self):
        result = self.refine(1.15, center=(178, 130))
        self.assertTrue(result.accepted)
        self.assertGreater(result.bbox.w, self.base_bbox.w)
        self.assertAlmostEqual(result.bbox.cx, 178.0, delta=1.0)
        self.assertAlmostEqual(result.bbox.cy, 130.0, delta=1.0)

    def test_scale_outlier_is_rejected_by_physical_step_limit(self):
        refiner = MultiScaleBBoxRefiner(
            candidates=(1.0, 1.5),
            max_step_ratio=0.18,
            minimum_score=0.95,
        )
        refiner.reset(self.patch)
        result = refiner.refine(
            self.frame(1.5),
            self.base_bbox,
            self.base_bbox,
        )
        self.assertFalse(result.accepted)
        self.assertEqual(result.scale_ratio, 1.0)

    def test_remote_distractor_cannot_take_over_local_scale_search(self):
        result = self.refine(1.08, distractor=True)
        self.assertTrue(result.accepted)
        self.assertAlmostEqual(result.bbox.cx, self.center[0], delta=1.0)
        self.assertAlmostEqual(result.bbox.cy, self.center[1], delta=1.0)

    def test_ambiguous_scale_candidates_are_rejected(self):
        refiner = MultiScaleBBoxRefiner(
            candidates=(0.85, 1.0, 1.15),
            minimum_psr=100.0,
            minimum_gain=0.0,
        )
        refiner.reset(self.patch)
        result = refiner.refine(
            self.frame(1.15),
            self.base_bbox,
            self.base_bbox,
        )
        self.assertFalse(result.accepted)
        self.assertIn(result.reason, {
            "scale_psr_below_threshold",
            "ambiguous_scale_peaks",
        })

    def test_bbox_filter_converges_without_hiding_real_scale(self):
        filt = VisualBBoxFilter(
            size_alpha=0.20,
            max_size_alpha=0.60,
            stable_frames_required=1,
        )
        filt.update(
            cx=160, cy=120, width=60, height=40,
            timestamp_s=0.0, confidence=1.0,
        )
        widths = []
        for index in range(1, 6):
            state = filt.update(
                cx=160, cy=120, width=72, height=48,
                timestamp_s=index / 30.0, confidence=1.0,
            )
            widths.append(state.width)
        self.assertGreater(widths[0], 60.0)
        self.assertGreater(widths[-1], 70.0)
        self.assertEqual(widths, sorted(widths))

    def test_kcf_reinitializes_after_accepted_scale_change(self):
        tracker = HybridMemoryTrackerService("KCF")
        tracker.scale_update_every_n = 1
        tracker.scale_refiner.minimum_psr = 0.0
        tracker.scale_refiner.minimum_gain = 0.0
        tracker.init(self.frame(1.0), self.base_bbox)
        result = tracker.update(self.frame(1.15))
        self.assertEqual(result.state.value, "tracking")
        self.assertGreater(tracker.kcf_reinit_count, 0)
        self.assertGreater(result.bbox.w, self.base_bbox.w)

    def test_scale_refinement_is_not_scheduled_every_frame(self):
        tracker = HybridMemoryTrackerService("KCF")
        tracker.scale_update_every_n = 3
        tracker.init(self.frame(1.0), self.base_bbox)
        tracker.update(self.frame(1.0))
        self.assertFalse(tracker.diagnostics["scale_scheduled"])
        tracker.update(self.frame(1.0))
        self.assertFalse(tracker.diagnostics["scale_scheduled"])
        tracker.update(self.frame(1.0))
        self.assertTrue(tracker.diagnostics["scale_scheduled"])

    def test_low_confidence_does_not_update_scale_template(self):
        before = self.refiner.reference.copy()
        changed = np.zeros_like(self.patch)
        self.refiner.update_reference(changed, 0.2)
        self.assertTrue(np.array_equal(before, self.refiner.reference))


if __name__ == "__main__":
    unittest.main()
