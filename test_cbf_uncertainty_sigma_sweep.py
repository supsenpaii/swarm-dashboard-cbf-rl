"""Pins the two conclusions CBF_UNCERTAINTY_OFFLINE_SWEEP rests on.

Not a re-run of the sweep (that takes ~100 s and lives in
cbf_uncertainty_sigma_sweep.py): just the two properties that must not drift
silently, on the two cheapest scenarios that carry them.
"""

from __future__ import annotations

import unittest

from cbf_uncertainty_sigma_sweep import _expected_completers, _feasible, scenarios, simulate


def _scenario(name: str):
    for scenario in scenarios():
        if scenario.name == name:
            return scenario
    raise AssertionError(f"unknown scenario {name}")


class SigmaZeroIsTheBaselineTests(unittest.TestCase):
    def test_sigma_zero_reproduces_the_feature_off_run_exactly(self) -> None:
        """The whole sweep is only readable if sigma=0 with covariance
        published is indistinguishable from the flown, feature-off config."""
        scenario = _scenario("parallel_trajectory_baseline")
        off = simulate(scenario, 0.0, with_covariance=False).as_dict()
        zero = simulate(scenario, 0.0, with_covariance=True).as_dict()
        self.assertEqual(off, zero)
        self.assertEqual(zero["uncertainty_m"], 0.0)

    def test_zero_plant_lag_preserves_the_historical_replay(self) -> None:
        scenario = _scenario("parallel_trajectory_baseline")
        historical = simulate(scenario, 0.1).as_dict()
        explicit_zero = simulate(
            scenario,
            0.1,
            command_delay_s=0.0,
            velocity_time_constant_s=0.0,
        ).as_dict()
        self.assertEqual(historical, explicit_zero)

    def test_invalid_plant_lag_is_rejected(self) -> None:
        scenario = _scenario("parallel_trajectory_baseline")
        with self.assertRaisesRegex(ValueError, "command delay"):
            simulate(scenario, 0.1, command_delay_s=-0.01)
        with self.assertRaisesRegex(ValueError, "velocity time constant"):
            simulate(scenario, 0.1, velocity_time_constant_s=float("nan"))


class CrossingCliffTests(unittest.TestCase):
    """The validated crossing geometry is what binds the feasible region.

    The production-uncertainty geometry remains feasible through sigma=1.50,
    then crosses a discontinuous infeasibility edge before the historical
    default of 2.0. Production remains pinned to the separately selected 0.10.
    """

    def test_low_sigma_is_feasible_and_high_sigma_is_not(self) -> None:
        scenario = _scenario("crossing_validated")
        completers = _expected_completers(scenario)
        feasible = simulate(scenario, 1.5).as_dict()
        broken = simulate(scenario, 1.75).as_dict()

        self.assertEqual(_feasible(feasible, completers)[0], True)
        self.assertEqual(_feasible(broken, completers)[0], False)
        self.assertEqual(feasible["infeasible_frames"], 0)
        self.assertGreater(broken["infeasible_frames"], 0)

    def test_more_uncertainty_can_trigger_an_infeasible_discontinuity(self) -> None:
        scenario = _scenario("crossing_validated")
        feasible = simulate(scenario, 1.5).as_dict()
        broken = simulate(scenario, 1.75).as_dict()
        self.assertEqual(feasible["infeasible_frames"], 0)
        self.assertGreater(broken["infeasible_frames"], 0)
        self.assertLess(broken["min_margin_reported_m"], feasible["min_margin_reported_m"])

    def test_default_sigma_of_2_0_is_outside_the_feasible_region(self) -> None:
        scenario = _scenario("crossing_validated")
        row = simulate(scenario, 2.0).as_dict()
        self.assertFalse(_feasible(row, _expected_completers(scenario))[0])
        self.assertGreater(row["infeasible_frames"], 0)


if __name__ == "__main__":
    unittest.main()
