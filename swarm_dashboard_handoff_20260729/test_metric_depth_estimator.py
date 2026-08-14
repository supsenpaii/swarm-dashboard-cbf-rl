import unittest

from metric_depth_estimator import MetricDepthEstimator


class MetricDepthEstimatorStabilityTest(unittest.TestCase):
    def estimator_with_distances(self, distances):
        estimator = MetricDepthEstimator(fx=205.5, fy=205.5)
        estimator.enabled = True
        estimator.every_n_frames = 1
        estimator.stable_samples_required = 3
        estimator.max_stable_step_m = 0.5
        estimator._ensure_loaded = lambda: True
        estimator._infer_depth = lambda frame: object()
        samples = iter(distances)
        estimator._distance_from_bbox = (
            lambda depth, bbox, width, height: (next(samples), 10.0, 0.9)
        )
        return estimator

    def test_first_metric_sample_is_not_ready(self):
        estimator = self.estimator_with_distances([12.0])
        frame = type("Frame", (), {"shape": (360, 640, 3)})()
        estimate = estimator.update(frame, object(), tracking_valid=True)
        self.assertTrue(estimate.valid)
        self.assertFalse(estimate.ready)
        self.assertEqual(estimate.reason, "stabilizing")
        self.assertEqual(estimate.stable_samples, 1)

    def test_consistent_metric_samples_become_ready(self):
        estimator = self.estimator_with_distances([12.0, 12.2, 12.1])
        frame = type("Frame", (), {"shape": (360, 640, 3)})()
        estimates = [
            estimator.update(frame, object(), tracking_valid=True)
            for _ in range(3)
        ]
        self.assertFalse(estimates[1].ready)
        self.assertTrue(estimates[2].ready)
        self.assertEqual(estimates[2].stable_samples, 3)

    def test_metric_jump_restarts_stability_gate(self):
        estimator = self.estimator_with_distances([12.0, 12.1, 14.0])
        frame = type("Frame", (), {"shape": (360, 640, 3)})()
        estimate = None
        for _ in range(3):
            estimate = estimator.update(frame, object(), tracking_valid=True)
        self.assertIsNotNone(estimate)
        self.assertFalse(estimate.ready)
        self.assertEqual(estimate.stable_samples, 1)


if __name__ == "__main__":
    unittest.main()
