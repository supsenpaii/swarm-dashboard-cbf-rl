"""Pins the two conclusions CBF_UNCERTAINTY_OFFLINE_SWEEP rests on.

Not a re-run of the sweep (that takes ~100 s and lives in
cbf_uncertainty_sigma_sweep.py): just the two properties that must not drift
silently, on the two cheapest scenarios that carry them.
"""

from __future__ import annotations

import unittest

from cbf_uncertainty_sigma_sweep import (
    DEADLOCK_S,
    _expected_completers,
    _feasible,
    scenarios,
    simulate,
)


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

    # Re-measured 2026-08-18, when this scenario stopped modelling a vehicle
    # that changes velocity in a single 20 ms step and started carrying the
    # measured Sparrow response (0.860 s) and the airframe's MPC_ACC_HOR_MAX.
    #
    # The edge barely moved -- 1.5/1.75 became 1.6/1.7 -- but THE FAILURE MODE
    # CHANGED, and that is the result worth keeping. Against the instant
    # vehicle the barrier became unsatisfiable and the gate refused. Against a
    # real one it stays satisfiable far longer (no infeasible frame until 3.0)
    # and the pair stalls instead: 0.22 s of deadlock at 1.60, 8.74 s at 1.70,
    # 15.36 s at 2.00. Same edge, different thing breaking, and a stall is a
    # failure a flight crew sees while a refused frame is one they do not.
    #
    # Production remains pinned to the separately selected 0.10.

    def test_low_sigma_is_flyable_and_high_sigma_is_not(self) -> None:
        scenario = _scenario("crossing_validated")
        completers = _expected_completers(scenario)
        flyable = simulate(scenario, 1.6).as_dict()
        broken = simulate(scenario, 1.7).as_dict()

        self.assertEqual(_feasible(flyable, completers)[0], True)
        self.assertEqual(_feasible(broken, completers)[0], False)

    def test_the_edge_is_a_stall_not_a_refusal(self) -> None:
        """What actually fails first, now that the vehicle is a vehicle."""
        scenario = _scenario("crossing_validated")
        broken = simulate(scenario, 1.7).as_dict()

        self.assertEqual(broken["infeasible_frames"], 0)
        self.assertGreater(broken["longest_deadlock_s"], DEADLOCK_S)
        # And the barrier is still meeting its own requirement while stalled.
        self.assertGreater(broken["min_margin_reported_m"], 0.0)

    def test_refusal_needs_nearly_twice_the_uncertainty(self) -> None:
        scenario = _scenario("crossing_validated")
        self.assertEqual(simulate(scenario, 2.5).as_dict()["infeasible_frames"], 0)
        self.assertGreater(simulate(scenario, 3.0).as_dict()["infeasible_frames"], 0)

    def test_default_sigma_of_2_0_is_still_outside_the_flyable_region(self) -> None:
        """Unchanged verdict, different reason: a 15 s stall, not a refusal."""
        scenario = _scenario("crossing_validated")
        row = simulate(scenario, 2.0).as_dict()

        self.assertFalse(_feasible(row, _expected_completers(scenario))[0])
        self.assertEqual(row["infeasible_frames"], 0)
        self.assertGreater(row["longest_deadlock_s"], DEADLOCK_S)


if __name__ == "__main__":
    unittest.main()
