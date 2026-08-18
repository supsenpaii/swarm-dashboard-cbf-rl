"""Pin the production-uncertainty crossing flight geometry and its plant stress."""

from __future__ import annotations

import unittest
from dataclasses import replace

from cbf_uncertainty_sigma_sweep import Scenario, UAV_01, UAV_02, simulate
from trajectory_controller import LinearTrajectory
from two_uav_crossing_trajectory_flight import DEFAULT_TRAJECTORY_ENV, REQUIRED_CBF_ENV


def _vector(value: str) -> tuple[float, float, float]:
    result = tuple(float(part) for part in value.split(","))
    if len(result) != 3:
        raise ValueError("trajectory vector must have three components")
    return result  # type: ignore[return-value]


def _scenario(buffer_m: float | None = None) -> Scenario:
    geometry = {}
    trajectories = {}
    for drone in (UAV_01, UAV_02):
        tag = drone.replace("-", "_")
        start = _vector(DEFAULT_TRAJECTORY_ENV[f"SWARM_TRAJECTORY_{tag}_START_ENU_M"])
        end = _vector(DEFAULT_TRAJECTORY_ENV[f"SWARM_TRAJECTORY_{tag}_END_ENU_M"])
        speed = float(DEFAULT_TRAJECTORY_ENV[f"SWARM_TRAJECTORY_{tag}_SPEED_M_S"])
        geometry[drone] = start
        trajectories[drone] = LinearTrajectory(start, end, speed)
    return Scenario(
        name="production_uncertainty_crossing",
        note="flight-driver contract",
        horizon_s=120.0,
        spawn=geometry,
        trajectories=trajectories,
        command_latency_s=0.65,
        design_margin_buffer_m=(
            float(REQUIRED_CBF_ENV["SWARM_CBF_DESIGN_MARGIN_BUFFER_M"])
            if buffer_m is None
            else buffer_m
        ),
        peer_age_ms=100.0,
        # This test is named for the PLANT stress, so the plant has to be one:
        # the measured Sparrow horizontal response and the airframe's own
        # MPC_ACC_HOR_MAX. Left at the zero default the vehicle changes
        # velocity in a single 20 ms step, which is not a stress, it is a
        # cheat -- and it is the reason this file used to report a feasibility
        # cliff that a physical vehicle never reaches.
        response_time_constant_s=0.860,
        maximum_acceleration_m_s2=4.0,
    )


class CrossingGeometryBufferTests(unittest.TestCase):
    def _assert_safe_completion(
        self, result: dict[str, object], *, minimum_correction_m_s: float = 0.15
    ) -> None:
        self.assertEqual(result["infeasible_frames"], 0)
        self.assertEqual(result["hold_frames"], 0)
        self.assertTrue(all(value is not None for value in result["completed"].values()))
        self.assertTrue(all(stage == "normal" for stage in result["max_emergency_stage"].values()))
        self.assertGreaterEqual(result["min_margin_reported_m"], 0.3)
        self.assertGreaterEqual(result["min_physical_slack_m"], 0.6)
        # This floor exists to prove the barrier does something here at all,
        # not to grade how much. It was 0.2 against a vehicle that could be
        # redirected in one 20 ms step; a 4 m/s^2 airframe peaks at 0.186 on
        # the same encounter, which is the same intervention seen honestly.
        self.assertGreaterEqual(result["max_correction_m_s"], minimum_correction_m_s)

    def test_legs_genuinely_intersect_halfway(self) -> None:
        scenario = _scenario()
        points = []
        for drone in (UAV_01, UAV_02):
            trajectory = scenario.trajectories[drone]
            points.append(
                tuple(
                    (trajectory.start_enu_m[axis] + trajectory.end_enu_m[axis]) / 2.0
                    for axis in range(3)
                )
            )
        self.assertEqual(points, [(0.0, 8.0, 9.0), (0.0, 8.0, 9.0)])

    def test_production_uncertainty_has_real_intervention_and_margin(self) -> None:
        self._assert_safe_completion(simulate(_scenario(), 0.10).as_dict())

    def test_conservative_command_and_velocity_lag_remains_safe(self) -> None:
        result = simulate(
            _scenario(),
            0.10,
            command_delay_s=0.30,
            velocity_time_constant_s=0.75,
        ).as_dict()
        self._assert_safe_completion(result, minimum_correction_m_s=0.15)

    def test_unbuffered_geometry_does_not_clear_the_margin_gate(self) -> None:
        result = simulate(replace(_scenario(), design_margin_buffer_m=0.0), 0.10).as_dict()
        self.assertLess(result["min_margin_reported_m"], 0.3)


if __name__ == "__main__":
    unittest.main()
