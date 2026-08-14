from __future__ import annotations

import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from cbf_uncertainty_offline_sweep_v2 import load_trace
from cbf_uncertainty_sigma_sweep import _expected_completers, _feasible, scenarios, simulate


def _row(monotonic_s: float, phase: str, counter: int, raw: int, valid: int) -> dict:
    return {
        "monotonic_s": monotonic_s,
        "phase": phase,
        "drones": {
            drone: {
                "position_covariance_enu_m2": [0.04, 0.041, 0.07],
                "odometry_reset_counter": counter,
                "odometry_sample_count": raw,
                "odometry_covariance_sample_count": valid,
            }
            for drone in ("UAV-01", "UAV-02")
        },
    }


class FlightTraceReplayTests(unittest.TestCase):
    def test_loads_covariance_and_preserves_observed_reset(self) -> None:
        rows = [_row(10.0, "GROUND", 11, 10, 10), _row(10.1, "TAKEOFF/CLIMB", 12, 12, 11)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "samples.jsonl"
            path.write_text("\n".join(json.dumps(row) for row in rows), encoding="utf-8")
            trace = load_trace(path)

        self.assertEqual(len(trace["frames"]), 2)
        self.assertAlmostEqual(trace["sample_period_s"], 0.1)
        event = trace["reset_counter"]["UAV-01"]["events"][0]
        self.assertEqual(event["invalid_odometry_samples_in_interval"], 1)
        self.assertFalse(event["covariance_missing_at_observation"])

    def test_sigma_zero_trace_replay_is_feature_off_baseline(self) -> None:
        scenario = replace(
            next(item for item in scenarios() if item.name == "parallel_trajectory_baseline"),
            peer_age_ms=100.0,
        )
        frame = {
            "UAV-01": (0.04, 0.041, 0.07),
            "UAV-02": (0.042, 0.043, 0.072),
        }
        self.assertEqual(
            simulate(scenario, 0.0, with_covariance=False).as_dict(),
            simulate(
                scenario,
                0.0,
                covariance_frames=(frame,),
                covariance_sample_period_s=0.1,
            ).as_dict(),
        )

    def test_measured_max_profile_pins_current_feasibility_edge(self) -> None:
        scenario = replace(
            next(item for item in scenarios() if item.name == "crossing_validated"),
            peer_age_ms=100.0,
        )
        measured_max = ({
            "UAV-01": (0.04306085780262947, 0.04300913214683533, 0.07411964237689972),
            "UAV-02": (0.0433967150747776, 0.04333620145916939, 0.07437072694301605),
        },)
        results = {
            sigma: simulate(scenario, sigma, covariance_frames=measured_max).as_dict()
            for sigma in (0.10, 1.40, 1.50)
        }
        completers = _expected_completers(scenario)

        self.assertEqual(_feasible(results[0.10], completers), (True, True))
        self.assertEqual(_feasible(results[1.40], completers), (True, True))
        self.assertEqual(_feasible(results[1.50], completers), (False, False))
        self.assertIsNotNone(results[1.50]["first_infeasible_s"])


if __name__ == "__main__":
    unittest.main()
