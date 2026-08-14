from __future__ import annotations

import unittest

from cbf_uncertainty_flight_validation import configuration_failures
from flight_covariance_measurement import (
    analyze,
    distribution,
    finite_covariance,
    percentile,
    prearm_ok,
)
from mavlink_manual_bridge import FastPoseCache
from test_cbf_uncertainty_plumbing import NOW, odometry_message


class FlightCovarianceObservationTests(unittest.TestCase):
    def test_flight_validation_configuration_fails_closed(self) -> None:
        self.assertEqual(configuration_failures({"sigma": "0.1"}, {"sigma": "0.1"}), [])
        self.assertEqual(
            configuration_failures({"sigma": "0.0"}, {"sigma": "0.1"}),
            ["sigma:expected=0.1:actual=0.0"],
        )

    def test_odometry_observation_counts_age_and_reset_handoff(self) -> None:
        cache = FastPoseCache()
        cache.update(odometry_message(reset_counter=3), NOW)
        self.assertEqual(cache.odometry_sample_count, 1)
        self.assertEqual(cache.odometry_covariance_sample_count, 1)
        self.assertEqual(cache.position_covariance_age_ms(NOW), 0.0)
        cache.update(odometry_message(reset_counter=3), NOW + 0.02)
        self.assertEqual(cache.odometry_covariance_sample_count, 2)
        self.assertEqual(cache.position_covariance_age_ms(NOW + 0.05), 30.0)
        cache.update(odometry_message(reset_counter=4), NOW + 0.06)
        self.assertEqual(cache.odometry_reset_counter, 4)
        self.assertEqual(cache.odometry_covariance_sample_count, 3)
        self.assertEqual(cache.position_covariance_age_ms(NOW + 0.06), 0.0)

    def test_analysis_helpers_do_not_turn_invalid_covariance_into_zero(self) -> None:
        self.assertIsNone(finite_covariance(None))
        self.assertIsNone(finite_covariance([-1.0, 0.1, 0.1]))
        self.assertEqual(percentile([1.0, 2.0, 3.0], 0.5), 2.0)
        self.assertEqual(distribution([])["samples"], 0)
        self.assertIsNone(distribution([])["p95"])

    def test_prearm_gate_accepts_only_the_expected_candidate_configuration(self) -> None:
        state = {
            "armed": False,
            "position_covariance_enu_m2": [0.04, 0.041, 0.07],
            "position_covariance_map_complete": True,
            "covariance_age_ms": 10.0,
            "odometry_reset_counter": 12,
            "cbf_covariance_sigma": 0.10,
            "cbf_require_position_covariance": True,
        }
        rows = [{"drones": {drone: dict(state) for drone in ("UAV-01", "UAV-02")}}]

        self.assertEqual(
            prearm_ok(
                rows,
                expected_sigma=0.10,
                expected_require_covariance=True,
            ),
            (True, []),
        )
        self.assertFalse(prearm_ok(rows)[0])

    def test_candidate_analysis_requires_clean_safety_events(self) -> None:
        state = {
            "position_covariance_enu_m2": [0.04, 0.041, 0.07],
            "covariance_age_ms": 10.0,
            "odometry_reset_counter": 12,
            "odometry_sample_count": 100,
            "odometry_covariance_sample_count": 99,
            "cbf_margin_m": 0.5,
            "cbf_reason": "cbf_filtered",
            "cbf_intervened": False,
            "supervisor_stage": "normal",
            "watchdog_conditions": [],
            "sender_latched_abort": None,
            "cbf_covariance_sigma": 0.10,
            "cbf_require_position_covariance": True,
        }
        rows = [
            {
                "monotonic_s": float(index),
                "phase": phase,
                "drones": {drone: dict(state) for drone in ("UAV-01", "UAV-02")},
            }
            for index, phase in enumerate(
                ("GROUND", "TAKEOFF/CLIMB", "HOVER", "TRAJECTORY", "LANDING")
            )
        ]
        summary = analyze(
            rows,
            {"verdict": "FLIGHT_PASS"},
            expected_sigma=0.10,
            expected_require_covariance=True,
            require_clean_safety_events=True,
        )
        self.assertTrue(summary["measurement_data_valid"])

        rows[-1]["drones"]["UAV-01"]["cbf_margin_m"] = -0.01
        unsafe = analyze(
            rows,
            {"verdict": "FLIGHT_PASS"},
            expected_sigma=0.10,
            expected_require_covariance=True,
            require_clean_safety_events=True,
        )
        self.assertFalse(unsafe["measurement_data_valid"])


if __name__ == "__main__":
    unittest.main()
