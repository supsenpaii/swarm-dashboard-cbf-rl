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

    def test_mission_velocity_points_at_the_goal_and_sheds_speed_into_it(self) -> None:
        """The coordinator's `mission` argument, which used to be the policy.

        Every role in ConflictCoordinator rebuilds its output from `mission`
        for the Sparrow config, so this vector -- not the policy -- is what
        the matrix actually certifies.
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

        far = environment._mission_velocity("UAV-01")
        self.assertAlmostEqual(far[0], 15.0)
        self.assertAlmostEqual(far[1], 0.0)

        environment.positions["UAV-01"] = (99.0, 0.0, 9.0)
        near = environment._mission_velocity("UAV-01")
        self.assertLess(near[0], 15.0)
        self.assertGreater(near[0], 0.0)

        environment.positions["UAV-01"] = (100.0, 0.0, 9.0)
        self.assertEqual(environment._mission_velocity("UAV-01"), (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
