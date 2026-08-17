from __future__ import annotations

import math
import unittest

from cbf_rl_env import CbfRlEnvConfig, CbfRlEnvironment, DRONE_IDS, OBSERVATION_FIELDS


SPAWN = {"UAV-01": (0.0, 0.0, 9.0), "UAV-02": (5.0, 0.0, 9.0)}
GOALS = {"UAV-01": (10.0, 0.0, 9.0), "UAV-02": (-5.0, 0.0, 9.0)}


class CbfRlEnvironmentTests(unittest.TestCase):
    def test_observation_contract_is_fixed_and_finite(self) -> None:
        observations = CbfRlEnvironment(GOALS).reset(SPAWN)

        self.assertEqual(len(OBSERVATION_FIELDS), 20)
        self.assertEqual(set(observations), set(DRONE_IDS))
        self.assertTrue(all(len(value) == 20 for value in observations.values()))
        self.assertTrue(
            all(math.isfinite(component) for value in observations.values() for component in value)
        )
        self.assertEqual(observations["UAV-01"][6:9], (5.0, 0.0, 0.0))
        self.assertEqual(observations["UAV-02"][6:9], (-5.0, 0.0, 0.0))

    def test_nominal_actions_can_only_advance_through_the_cbf(self) -> None:
        environment = CbfRlEnvironment(GOALS)
        environment.reset(SPAWN)

        _, _, _, _, info = environment.step(
            {"UAV-01": (1.0, 0.0, 0.0), "UAV-02": (-1.0, 0.0, 0.0)}
        )

        for drone in DRONE_IDS:
            self.assertTrue(info[drone]["cbf"]["active"])
            self.assertGreater(info[drone]["cbf"]["intervention_norm_m_s"], 0.0)
            self.assertNotEqual(
                info[drone]["nominal_velocity_enu_m_s"],
                info[drone]["safe_velocity_enu_m_s"],
            )
        separation = environment.positions["UAV-02"][0] - environment.positions["UAV-01"][0]
        self.assertGreater(separation, 4.0)

    def test_missing_covariance_fails_closed_without_motion(self) -> None:
        environment = CbfRlEnvironment(GOALS)
        environment.reset(
            SPAWN,
            covariance_by_drone={"UAV-01": (0.04, 0.04, 0.07), "UAV-02": None},
        )
        before = dict(environment.positions)

        observations, rewards, _, _, info = environment.step(
            {drone: (1.0, 0.0, 0.0) for drone in DRONE_IDS}
        )

        self.assertEqual(environment.positions, before)
        self.assertFalse(info["UAV-01"]["cbf"]["active"])
        self.assertFalse(info["UAV-02"]["cbf"]["active"])
        self.assertLess(rewards["UAV-01"], -2.0)
        self.assertEqual(observations["UAV-01"][-1], 0.0)
        self.assertEqual(observations["UAV-02"][-2], 0.0)

    def test_non_conflict_episode_reaches_both_goals(self) -> None:
        config = CbfRlEnvConfig(dt_s=0.1, maximum_steps=20, arrival_radius_m=0.11)
        environment = CbfRlEnvironment(
            {"UAV-01": (1.0, 0.0, 9.0), "UAV-02": (1.0, 10.0, 9.0)},
            config,
        )
        environment.reset(
            {"UAV-01": (0.0, 0.0, 9.0), "UAV-02": (0.0, 10.0, 9.0)}
        )

        terminated = truncated = False
        while not terminated and not truncated:
            _, _, terminated, truncated, _ = environment.step(
                {drone: (1.0, 0.0, 0.0) for drone in DRONE_IDS}
            )

        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertEqual(environment.reached, set(DRONE_IDS))

    def test_previous_arrival_does_not_terminate_after_leaving_goal(self) -> None:
        config = CbfRlEnvConfig(dt_s=0.1, maximum_steps=20, arrival_radius_m=0.11)
        environment = CbfRlEnvironment(
            {"UAV-01": (0.2, 0.0, 9.0), "UAV-02": (5.2, 10.0, 9.0)},
            config,
        )
        environment.reset(
            {"UAV-01": (0.0, 0.0, 9.0), "UAV-02": (5.0, 10.0, 9.0)}
        )
        environment.step({"UAV-01": (1.0, 0.0, 0.0), "UAV-02": (0.0, 0.0, 0.0)})

        _, _, terminated, _, _ = environment.step(
            {"UAV-01": (1.0, 0.0, 0.0), "UAV-02": (1.0, 0.0, 0.0)}
        )

        self.assertFalse(terminated)
        self.assertIn("UAV-01", environment.reached)

    def test_the_coordinator_mission_is_a_real_tracker_command(self) -> None:
        """The coordinator's `mission` argument, and the matrix's blind spot.

        Every role in ConflictCoordinator rebuilds its output from `mission`
        for the Sparrow config, so this vector -- not the policy -- is what
        the matrix certifies. It used to be a hand-written goal-direction
        vector, which is why 1260 cases passed a yield geometry that
        saturated at 5 m in flight: a bare direction has no position-feedback
        term for the lane change to be cancelled by.
        """
        config = CbfRlEnvConfig(
            maximum_acceleration_m_s2=4.0,
            response_time_constant_s=0.86,
            mission_speed_m_s=15.0,
        )
        environment = CbfRlEnvironment(
            {"UAV-01": (100.0, 0.0, 9.0), "UAV-02": (0.0, 50.0, 9.0)}, config
        )
        environment.reset({"UAV-01": (0.0, 0.0, 9.0), "UAV-02": (0.0, 0.0, 9.0)})

        # It ramps from rest rather than stepping to cruise, because it is the
        # same controller the companion installs and it obeys the same limit.
        first = environment._mission_velocity("UAV-01")
        self.assertEqual(first, (0.0, 0.0, 0.0))
        # 15 m/s at 4 m/s^2 is 3.75 s, and dt is 0.05 s.
        speeds = []
        for _ in range(100):
            environment.steps += 1
            speeds.append(environment._mission_velocity("UAV-01")[0])
        self.assertLess(speeds[0], 15.0)
        self.assertTrue(
            all(b >= a - 1e-9 for a, b in zip(speeds, speeds[1:])), speeds[:8]
        )
        self.assertAlmostEqual(speeds[-1], 15.0, places=3)

    def test_the_mission_carries_the_feedback_that_cancels_a_lane_change(self) -> None:
        """Displaced sideways, it pulls back -- which is the whole point.

        A yield pushes the vehicle off its leg; the tracker answers with
        position_gain_s_inv * cross-track, and that term re-enters the lane
        change weighted by forward_speed. Without it the matrix cannot see a
        yield saturate, however long it runs.
        """
        config = CbfRlEnvConfig(mission_speed_m_s=15.0, position_gain_s_inv=0.6)
        environment = CbfRlEnvironment(
            {"UAV-01": (300.0, 0.0, 9.0), "UAV-02": (0.0, 900.0, 9.0)}, config
        )
        environment.reset({"UAV-01": (0.0, 0.0, 9.0), "UAV-02": (0.0, 800.0, 9.0)})

        on_the_line = environment._mission_velocity("UAV-01")
        environment.positions["UAV-01"] = (150.0, 8.0, 9.0)
        displaced = environment._mission_velocity("UAV-01")

        self.assertAlmostEqual(on_the_line[1], 0.0, places=6)
        # 0.6 * -8 m of cross-track pulls back toward the leg. The pair is
        # then (15.0, -4.8), whose norm exceeds the 15 m/s ceiling, so the
        # command is scaled onto it and the lateral term lands at -4.57.
        self.assertLess(displaced[1], -4.0)
        self.assertAlmostEqual(displaced[1], -4.572, places=3)
        self.assertAlmostEqual(math.hypot(*displaced), 15.0, places=6)

    def test_it_holds_once_the_goal_is_reached(self) -> None:
        config = CbfRlEnvConfig(mission_speed_m_s=15.0)
        environment = CbfRlEnvironment(
            {"UAV-01": (100.0, 0.0, 9.0), "UAV-02": (0.0, 50.0, 9.0)}, config
        )
        environment.reset({"UAV-01": (0.0, 0.0, 9.0), "UAV-02": (0.0, 0.0, 9.0)})
        environment.positions["UAV-01"] = (100.0, 0.0, 9.0)

        self.assertEqual(environment._mission_velocity("UAV-01"), (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
