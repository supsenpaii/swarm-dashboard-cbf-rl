"""Regression check: LATENCY_MEASUREMENT_FLIGHT's square-wave excitation must
not provoke CBF at all.

This flight exists to measure PX4's own step response so
`CbfConfig.command_latency_s` can finally be fit against real data (see
latency_measurement_flight.py's module docstring for the full chain of
investigation that led here). If CBF intervened during the excitation, the
measured velocity would be a CBF-corrected signal, not PX4's open-loop
response to the companion's own command -- exactly the confound this flight
is designed to avoid. This test simulates the real
`TrajectoryTrackingController` + `CbfCommandGate` (not reimplemented math,
same practice `test_formation_spawn_geometry.py` established) against
`latency_measurement_flight.DEFAULT_TRAJECTORY_ENV`'s actual numbers, so a
future edit to that constant cannot silently reintroduce CBF contamination
without this test catching it first.
"""

from __future__ import annotations

import unittest

from cbf_command_gate import CbfCommandGate, CbfConfig
from formation_controller import FormationConfig
from latency_measurement_flight import DEFAULT_TRAJECTORY_ENV
from trajectory_controller import SquareWaveVelocityTrajectory, TrajectoryTrackingController

# Below this, real PX4 tracking overshoot (measured: up to ~5.5% on prior
# flights, and CBF corrections up to ~1.9 m/s during genuine intervention on
# the crossing flights) could plausibly eat the remaining slack even though
# this simulation shows zero intervention. Not a CBF constant -- a check on
# this test's own conclusion, same role MINIMUM_ACCEPTABLE_MARGIN_M plays in
# test_formation_spawn_geometry.py.
MINIMUM_ACCEPTABLE_MARGIN_M = 1.0
SIMULATED_CYCLES = 3
DT_S = 0.05

UAV_01 = "UAV-01"
UAV_02 = "UAV-02"
# Static leader hover position in shared ENU -- the real value both prior
# trajectory flights actually flew to twice (see
# two_uav_trajectory_flight.py's DEFAULT_TRAJECTORY_ENV for the measured
# GPS-vs-local-NED z=9.0 reasoning this reuses).
LEADER_POSITION_ENU_M = (0.0, 0.0, 9.0)


def _parse_vector(value: str) -> tuple[float, float, float]:
    parts = tuple(float(part.strip()) for part in value.split(","))
    if len(parts) != 3:
        raise ValueError(f"expected three components, got {value!r}")
    return parts  # type: ignore[return-value]


class LatencyMeasurementGeometryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.assertEqual(DEFAULT_TRAJECTORY_ENV["SWARM_TRAJECTORY_UAV_02_KIND"], "square_wave")
        self.start = _parse_vector(DEFAULT_TRAJECTORY_ENV["SWARM_TRAJECTORY_UAV_02_START_ENU_M"])
        self.step_velocity = _parse_vector(
            DEFAULT_TRAJECTORY_ENV["SWARM_TRAJECTORY_UAV_02_STEP_VELOCITY_ENU_M_S"]
        )
        self.half_period_s = float(DEFAULT_TRAJECTORY_ENV["SWARM_TRAJECTORY_UAV_02_HALF_PERIOD_S"])

        self.trajectory = SquareWaveVelocityTrajectory(
            self.start, self.step_velocity, self.half_period_s
        )
        self.controller = TrajectoryTrackingController(
            UAV_02,
            self.trajectory,
            FormationConfig(position_gain_s_inv=0.6, maximum_velocity_m_s=2.0, arrival_radius_m=0.25),
        )
        self.cbf_gate = CbfCommandGate(
            UAV_02,
            (UAV_01,),
            CbfConfig(minimum_separation_m=4.0, barrier_gain_s_inv=2.0, maximum_velocity_m_s=2.0),
        )

    def _simulate(self, duration_s: float) -> tuple[float, float, int, int]:
        follower_pos = list(self.start)
        follower_vel = [0.0, 0.0, 0.0]
        minimum_distance = float("inf")
        minimum_margin = float("inf")
        interventions = 0
        infeasible = 0
        steps = int(duration_s / DT_S)
        for i in range(steps):
            t = i * DT_S
            swarm_state = {
                UAV_01: {
                    "valid": True,
                    "position_enu_m": list(LEADER_POSITION_ENU_M),
                    "velocity_enu_m_s": [0.0, 0.0, 0.0],
                    "message_age_ms": 25.0,
                },
                UAV_02: {
                    "valid": True,
                    "position_enu_m": list(follower_pos),
                    "velocity_enu_m_s": list(follower_vel),
                    "message_age_ms": 25.0,
                },
            }
            nominal = self.controller.command(t, swarm_state)
            command = self.cbf_gate.filter(nominal.velocity_enu_m_s, swarm_state)
            velocity = command.velocity_enu_m_s if command.active else (0.0, 0.0, 0.0)

            distance = sum(
                (follower_pos[k] - LEADER_POSITION_ENU_M[k]) ** 2 for k in range(3)
            ) ** 0.5
            minimum_distance = min(minimum_distance, distance)
            if command.minimum_margin_m is not None:
                minimum_margin = min(minimum_margin, command.minimum_margin_m)
            if command.reason == "cbf_constraints_infeasible":
                infeasible += 1
            if command.active and command.intervention_norm_m_s > 1e-6:
                interventions += 1

            for axis in range(3):
                follower_pos[axis] += velocity[axis] * DT_S
                follower_vel[axis] = velocity[axis]

        return minimum_distance, minimum_margin, interventions, infeasible

    def test_square_wave_never_provokes_cbf_intervention(self) -> None:
        duration = 2.0 * self.half_period_s * SIMULATED_CYCLES
        distance, margin, interventions, infeasible = self._simulate(duration)
        self.assertEqual(
            interventions,
            0,
            "expected zero CBF intervention during the excitation -- a "
            "measurement contaminated by CBF correction is not an open-loop "
            "PX4 step response. See this module's docstring.",
        )
        self.assertEqual(infeasible, 0)
        self.assertGreaterEqual(
            margin,
            MINIMUM_ACCEPTABLE_MARGIN_M,
            f"CBF minimum_margin_m ({margin:.3f} m) fell below the "
            f"{MINIMUM_ACCEPTABLE_MARGIN_M} m safety buffer (closest approach "
            f"{distance:.3f} m) -- DEFAULT_TRAJECTORY_ENV's geometry needs "
            "re-checking before flying.",
        )

    def test_excursion_points_away_from_the_leader(self) -> None:
        """The square wave's amplitude direction must only ever increase
        separation from the spawn distance, never close it -- see module
        docstring for why this, not a perpendicular or closing excursion,
        is what keeps CBF uninvolved by construction rather than by luck."""
        spawn_distance = sum(
            (self.start[k] - LEADER_POSITION_ENU_M[k]) ** 2 for k in range(3)
        ) ** 0.5
        peak = tuple(
            self.start[i] + self.step_velocity[i] * self.half_period_s for i in range(3)
        )
        peak_distance = sum(
            (peak[k] - LEADER_POSITION_ENU_M[k]) ** 2 for k in range(3)
        ) ** 0.5
        self.assertGreaterEqual(peak_distance, spawn_distance)


if __name__ == "__main__":
    unittest.main()
