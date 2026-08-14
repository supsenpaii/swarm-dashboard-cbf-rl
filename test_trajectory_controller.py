import math
import unittest

from formation_controller import FormationConfig
from trajectory_controller import (
    CircularTrajectory,
    ClosedPolylineTrajectory,
    LinearTrajectory,
    SquareWaveVelocityTrajectory,
    TrajectoryReference,
    TrajectoryTrackingController,
)


def state(position, velocity=(0.0, 0.0, 0.0), valid=True):
    return {
        "valid": valid,
        "position_enu_m": position,
        "velocity_enu_m_s": velocity,
    }


class LinearTrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.trajectory = LinearTrajectory((0.0, 0.0, 5.0), (10.0, 0.0, 5.0), speed_m_s=2.0)

    def test_reference_advances_at_the_configured_speed(self):
        reference = self.trajectory.reference(1.0)
        self.assertAlmostEqual(reference.position_enu_m[0], 2.0)
        self.assertAlmostEqual(reference.velocity_enu_m_s[0], 2.0)

    def test_reference_holds_at_the_endpoint_past_duration(self):
        reference = self.trajectory.reference(999.0)
        self.assertEqual(reference.position_enu_m, (10.0, 0.0, 5.0))
        self.assertEqual(reference.velocity_enu_m_s, (0.0, 0.0, 0.0))

    def test_is_finished_only_after_duration(self):
        self.assertFalse(self.trajectory.is_finished(0.0))
        self.assertTrue(self.trajectory.is_finished(5.0))

    def test_nearest_time_projects_onto_the_finite_leg(self):
        self.assertAlmostEqual(
            self.trajectory.nearest_time_s((6.0, 4.0, 5.0)), 3.0
        )
        self.assertEqual(self.trajectory.nearest_time_s((-4.0, 0.0, 5.0)), 0.0)
        self.assertEqual(self.trajectory.nearest_time_s((14.0, 0.0, 5.0)), 5.0)

    def test_rejects_zero_speed(self):
        with self.assertRaises(ValueError):
            LinearTrajectory((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), speed_m_s=0.0)


class CircularTrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.trajectory = CircularTrajectory((0.0, 0.0, 5.0), radius_m=10.0, angular_rate_rad_s=0.5)

    def test_starts_on_the_circle_at_start_angle(self):
        reference = self.trajectory.reference(0.0)
        self.assertAlmostEqual(reference.position_enu_m[0], 10.0)
        self.assertAlmostEqual(reference.position_enu_m[1], 0.0)
        self.assertAlmostEqual(reference.position_enu_m[2], 5.0)

    def test_velocity_is_tangential(self):
        reference = self.trajectory.reference(0.0)
        radial = (reference.position_enu_m[0], reference.position_enu_m[1], 0.0)
        tangential = (reference.velocity_enu_m_s[0], reference.velocity_enu_m_s[1], 0.0)
        dot = sum(radial[i] * tangential[i] for i in range(3))
        self.assertAlmostEqual(dot, 0.0, places=6)

    def test_never_finishes(self):
        self.assertFalse(self.trajectory.is_finished(0.0))
        self.assertFalse(self.trajectory.is_finished(1e9))

    def test_rejects_zero_angular_rate(self):
        with self.assertRaises(ValueError):
            CircularTrajectory((0.0, 0.0, 0.0), radius_m=1.0, angular_rate_rad_s=0.0)


class SquareWaveVelocityTrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.trajectory = SquareWaveVelocityTrajectory(
            (0.0, 0.0, 9.0), step_velocity_enu_m_s=(1.5, 0.0, 0.0), half_period_s=2.0
        )

    def test_starts_at_the_positive_step_velocity(self):
        reference = self.trajectory.reference(0.0)
        self.assertEqual(reference.position_enu_m, (0.0, 0.0, 9.0))
        self.assertEqual(reference.velocity_enu_m_s, (1.5, 0.0, 0.0))

    def test_position_advances_during_the_positive_half_period(self):
        reference = self.trajectory.reference(1.0)
        self.assertAlmostEqual(reference.position_enu_m[0], 1.5)
        self.assertEqual(reference.velocity_enu_m_s, (1.5, 0.0, 0.0))

    def test_velocity_flips_at_the_half_period(self):
        reference = self.trajectory.reference(2.0)
        self.assertAlmostEqual(reference.position_enu_m[0], 3.0)
        self.assertEqual(reference.velocity_enu_m_s, (-1.5, 0.0, 0.0))

    def test_position_returns_to_start_after_a_full_period(self):
        reference = self.trajectory.reference(4.0)
        self.assertAlmostEqual(reference.position_enu_m[0], 0.0, places=6)
        self.assertEqual(reference.velocity_enu_m_s, (1.5, 0.0, 0.0))

    def test_never_drifts_past_one_period_amplitude(self):
        for t in [i * 0.1 for i in range(200)]:
            position = self.trajectory.reference(t).position_enu_m
            self.assertGreaterEqual(position[0], -1e-9)
            self.assertLessEqual(position[0], 3.0 + 1e-9)

    def test_never_finishes(self):
        self.assertFalse(self.trajectory.is_finished(0.0))
        self.assertFalse(self.trajectory.is_finished(1e9))

    def test_repeats_every_full_period(self):
        first_cycle = self.trajectory.reference(0.5)
        second_cycle = self.trajectory.reference(4.5)
        self.assertAlmostEqual(first_cycle.position_enu_m[0], second_cycle.position_enu_m[0])
        self.assertEqual(first_cycle.velocity_enu_m_s, second_cycle.velocity_enu_m_s)

    def test_rejects_zero_half_period(self):
        with self.assertRaises(ValueError):
            SquareWaveVelocityTrajectory((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), half_period_s=0.0)

    def test_rejects_non_finite_step_velocity(self):
        with self.assertRaises(ValueError):
            SquareWaveVelocityTrajectory((0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0), half_period_s=1.0)


class TrajectoryTrackingControllerTests(unittest.TestCase):
    def setUp(self):
        self.trajectory = LinearTrajectory((0.0, 0.0, 5.0), (10.0, 0.0, 5.0), speed_m_s=2.0)
        self.controller = TrajectoryTrackingController(
            "UAV-01",
            self.trajectory,
            FormationConfig(position_gain_s_inv=0.5, maximum_velocity_m_s=3.0, arrival_radius_m=0.2),
        )

    def test_tracks_ahead_of_lagging_position(self):
        command = self.controller.command(2.0, {"UAV-01": state((0.0, 0.0, 5.0))})
        self.assertTrue(command.active)
        self.assertEqual(command.reason, "tracking_trajectory")
        self.assertGreater(command.velocity_enu_m_s[0], 2.0)

    def test_command_is_velocity_limited(self):
        command = self.controller.command(2.0, {"UAV-01": state((-100.0, 0.0, 5.0))})
        self.assertLessEqual(math.sqrt(sum(v * v for v in command.velocity_enu_m_s)), 3.0)

    def test_reports_trajectory_reached_once_finished_and_in_radius(self):
        command = self.controller.command(999.0, {"UAV-01": state((10.0, 0.0, 5.0))})
        self.assertEqual(command.reason, "trajectory_reached")
        self.assertEqual(command.velocity_enu_m_s, (0.0, 0.0, 0.0))

    def test_keeps_tracking_past_duration_if_still_out_of_radius(self):
        command = self.controller.command(999.0, {"UAV-01": state((5.0, 0.0, 5.0))})
        self.assertEqual(command.reason, "tracking_trajectory")
        self.assertGreater(command.velocity_enu_m_s[0], 0.0)

    def test_invalid_own_state_holds(self):
        command = self.controller.command(1.0, {"UAV-01": state((0.0, 0.0, 5.0), valid=False)})
        self.assertFalse(command.active)
        self.assertEqual(command.reason, "own_state_invalid")

    def test_negative_mission_elapsed_holds(self):
        command = self.controller.command(-1.0, {"UAV-01": state((0.0, 0.0, 5.0))})
        self.assertFalse(command.active)
        self.assertEqual(command.reason, "mission_elapsed_invalid")

    def test_optional_acceleration_limit_smooths_a_polygon_corner(self):
        trajectory = ClosedPolylineTrajectory(
            waypoints_enu_m=(
                (0.0, 0.0, 5.0),
                (20.0, 0.0, 5.0),
                (20.0, 20.0, 5.0),
                (0.0, 20.0, 5.0),
            ),
            speed_m_s=1.0,
        )
        controller = TrajectoryTrackingController(
            "UAV-01",
            trajectory,
            FormationConfig(maximum_velocity_m_s=2.0),
            maximum_acceleration_m_s2=0.5,
        )
        before_position = trajectory.reference(19.9).position_enu_m
        before = controller.command(
            19.9,
            {"UAV-01": state(before_position, velocity=(1.0, 0.0, 0.0))},
        )
        after_position = trajectory.reference(20.1).position_enu_m
        after = controller.command(
            20.1,
            {"UAV-01": state(after_position, velocity=(1.0, 0.0, 0.0))},
        )
        self.assertEqual(before.velocity_enu_m_s, (1.0, 0.0, 0.0))
        self.assertLessEqual(
            math.dist(after.velocity_enu_m_s, before.velocity_enu_m_s),
            0.100001,
        )

    def test_curvature_profile_keeps_a_15ms_square_turn_close(self):
        trajectory = ClosedPolylineTrajectory(
            waypoints_enu_m=(
                (0.0, 0.0, 9.0),
                (200.0, 0.0, 9.0),
                (200.0, 200.0, 9.0),
                (0.0, 200.0, 9.0),
            ),
            speed_m_s=15.0,
        )
        controller = TrajectoryTrackingController(
            "UAV-01",
            trajectory,
            FormationConfig(maximum_velocity_m_s=15.0),
            maximum_acceleration_m_s2=3.0,
            corner_tracking_tolerance_m=1.0,
        )
        position = [0.0, 0.0, 9.0]
        velocity = [0.0, 0.0, 0.0]
        errors = []
        speeds = []
        step_s = 0.05
        for step in range(800):
            command = controller.command(
                step * step_s,
                {"UAV-01": state(tuple(position), velocity=tuple(velocity))},
            )
            velocity = list(command.velocity_enu_m_s)
            position = [
                position[axis] + velocity[axis] * step_s for axis in range(3)
            ]
            errors.append(command.position_error_m or 0.0)
            speeds.append(math.dist((0.0, 0.0, 0.0), velocity))

        self.assertGreater(max(speeds), 14.9)
        self.assertLess(min(speeds[100:]), 4.0)
        self.assertLess(max(errors), 2.0)

    def test_polygon_progress_follows_position_instead_of_elapsed_time(self):
        trajectory = ClosedPolylineTrajectory(
            waypoints_enu_m=(
                (0.0, 0.0, 5.0),
                (20.0, 0.0, 5.0),
                (20.0, 20.0, 5.0),
                (0.0, 20.0, 5.0),
            ),
            speed_m_s=1.0,
        )
        controller = TrajectoryTrackingController(
            "UAV-01",
            trajectory,
            FormationConfig(maximum_velocity_m_s=2.0),
            maximum_acceleration_m_s2=0.5,
        )
        own_state = {"UAV-01": state((5.0, 0.0, 5.0), velocity=(0.0, 0.0, 0.0))}

        first = controller.command(5.0, own_state)
        much_later = controller.command(50.0, own_state)

        self.assertEqual(first.target_enu_m, much_later.target_enu_m)
        self.assertAlmostEqual(first.position_error_m, 0.0)
        self.assertLessEqual(math.dist((0.0, 0.0, 0.0), much_later.velocity_enu_m_s), 1.0)

    def test_polygon_speed_is_a_command_ceiling_not_only_reference_speed(self):
        trajectory = ClosedPolylineTrajectory(
            waypoints_enu_m=(
                (0.0, 0.0, 5.0),
                (20.0, 0.0, 5.0),
                (20.0, 20.0, 5.0),
                (0.0, 20.0, 5.0),
            ),
            speed_m_s=1.0,
        )
        controller = TrajectoryTrackingController(
            "UAV-01",
            trajectory,
            FormationConfig(position_gain_s_inv=0.6, maximum_velocity_m_s=2.0),
        )

        command = controller.command(
            100.0,
            {"UAV-01": state((5.0, -4.0, 5.0), velocity=(0.0, 0.0, 0.0))},
        )

        self.assertLessEqual(
            math.dist((0.0, 0.0, 0.0), command.velocity_enu_m_s), 1.0
        )
        self.assertAlmostEqual(command.position_error_m, 4.0)

    def test_acceleration_limit_starts_from_measured_velocity(self):
        controller = TrajectoryTrackingController(
            "UAV-01",
            self.trajectory,
            self.controller.config,
            maximum_acceleration_m_s2=0.5,
        )
        initial = controller.command(
            0.0,
            {"UAV-01": state((0.0, 0.0, 5.0), velocity=(0.2, 0.0, 0.0))},
        )
        self.assertEqual(initial.velocity_enu_m_s, (0.2, 0.0, 0.0))
        next_command = controller.command(
            0.2,
            {"UAV-01": state((0.04, 0.0, 5.0), velocity=(0.2, 0.0, 0.0))},
        )
        self.assertLessEqual(
            math.dist(next_command.velocity_enu_m_s, initial.velocity_enu_m_s),
            0.100001,
        )

    def test_rejects_invalid_acceleration_limit(self):
        for value in (0.0, -1.0, float("nan")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                TrajectoryTrackingController(
                    "UAV-01",
                    self.trajectory,
                    maximum_acceleration_m_s2=value,
                )

    def test_rejects_invalid_corner_tracking_tolerance(self):
        for value in (0.0, -1.0, float("nan")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                TrajectoryTrackingController(
                    "UAV-01",
                    self.trajectory,
                    corner_tracking_tolerance_m=value,
                )


class TrajectoryReferenceTests(unittest.TestCase):
    def test_rejects_non_finite_vectors(self):
        with self.assertRaises(ValueError):
            TrajectoryReference((float("nan"), 0.0, 0.0), (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()


class ClosedPolylineTrajectoryTest(unittest.TestCase):
    """A 20 m square at 2 m/s: 80 m perimeter, 40 s per lap."""

    SQUARE = ((0.0, 0.0, 9.0), (20.0, 0.0, 9.0), (20.0, 20.0, 9.0), (0.0, 20.0, 9.0))

    def _trajectory(self) -> ClosedPolylineTrajectory:
        return ClosedPolylineTrajectory(waypoints_enu_m=self.SQUARE, speed_m_s=2.0)

    def test_perimeter_and_lap_duration(self) -> None:
        trajectory = self._trajectory()
        self.assertAlmostEqual(trajectory.perimeter_m(), 80.0)
        self.assertAlmostEqual(trajectory.lap_duration_s(), 40.0)

    def test_walks_the_edges_at_the_requested_speed(self) -> None:
        trajectory = self._trajectory()
        # 5 s in: 10 m along the first edge, heading +East at full speed.
        reference = trajectory.reference(5.0)
        self.assertAlmostEqual(reference.position_enu_m[0], 10.0)
        self.assertAlmostEqual(reference.position_enu_m[1], 0.0)
        self.assertAlmostEqual(reference.velocity_enu_m_s[0], 2.0)
        self.assertAlmostEqual(reference.velocity_enu_m_s[1], 0.0)
        # 15 s in: past the first corner, 10 m up the second edge heading North.
        reference = trajectory.reference(15.0)
        self.assertAlmostEqual(reference.position_enu_m[0], 20.0)
        self.assertAlmostEqual(reference.position_enu_m[1], 10.0)
        self.assertAlmostEqual(reference.velocity_enu_m_s[1], 2.0)

    def test_laps_repeat_instead_of_finishing(self) -> None:
        trajectory = self._trajectory()
        self.assertFalse(trajectory.is_finished(1_000_000.0))
        for elapsed in (5.0, 12.5, 31.0):
            first = trajectory.reference(elapsed)
            third = trajectory.reference(elapsed + 80.0)
            for axis in range(3):
                self.assertAlmostEqual(first.position_enu_m[axis], third.position_enu_m[axis])
                self.assertAlmostEqual(
                    first.velocity_enu_m_s[axis], third.velocity_enu_m_s[axis]
                )

    def test_altitude_is_carried_by_the_waypoints(self) -> None:
        ramp = ((0.0, 0.0, 9.0), (20.0, 0.0, 12.0), (20.0, 20.0, 9.0))
        trajectory = ClosedPolylineTrajectory(waypoints_enu_m=ramp, speed_m_s=2.0)
        # The climbing edge is 3D arclength sqrt(20^2 + 3^2), not its ground
        # run, so 10 m flown along it is under half its climb.
        edge_m = math.hypot(20.0, 3.0)
        after = trajectory.reference(5.0)
        self.assertAlmostEqual(after.position_enu_m[2], 9.0 + 3.0 * (10.0 / edge_m))
        self.assertAlmostEqual(after.position_enu_m[0], 20.0 * (10.0 / edge_m))

    def test_refuses_a_degenerate_polygon(self) -> None:
        with self.assertRaises(ValueError):
            ClosedPolylineTrajectory(waypoints_enu_m=self.SQUARE[:2], speed_m_s=2.0)
        with self.assertRaises(ValueError):
            ClosedPolylineTrajectory(
                waypoints_enu_m=((0.0, 0.0, 9.0), (0.0, 0.0, 9.0), (5.0, 0.0, 9.0)),
                speed_m_s=2.0,
            )
        with self.assertRaises(ValueError):
            ClosedPolylineTrajectory(waypoints_enu_m=self.SQUARE, speed_m_s=0.0)


class ClosedPolylineEntryPointTest(unittest.TestCase):
    """Joining a loop must pick the nearest point on it, not waypoint zero."""

    SQUARE = ((0.0, 0.0, 9.0), (20.0, 0.0, 9.0), (20.0, 20.0, 9.0), (0.0, 20.0, 9.0))

    def setUp(self) -> None:
        self.trajectory = ClosedPolylineTrajectory(
            waypoints_enu_m=self.SQUARE, speed_m_s=2.0
        )

    def _entry_position(self, position):
        return self.trajectory.reference(
            self.trajectory.nearest_time_s(position)
        ).position_enu_m

    def test_a_point_beside_an_edge_projects_onto_that_edge(self) -> None:
        # Sitting 3 m outside the middle of the far (north) edge.
        entry = self._entry_position((10.0, 23.0, 9.0))
        self.assertAlmostEqual(entry[0], 10.0)
        self.assertAlmostEqual(entry[1], 20.0)

    def test_the_entry_point_is_the_true_minimum_over_the_whole_loop(self) -> None:
        for probe in ((-4.0, 10.0, 9.0), (25.0, 3.0, 9.0), (10.0, 10.0, 9.0), (1.0, -6.0, 9.0)):
            entry = self._entry_position(probe)
            best = min(
                math.dist(probe, self.trajectory.reference(step * 0.01).position_enu_m)
                for step in range(int(self.trajectory.lap_duration_s() * 100))
            )
            self.assertLessEqual(math.dist(probe, entry), best + 0.05)

    def test_a_vehicle_already_on_the_loop_enters_where_it_stands(self) -> None:
        standing = self.trajectory.reference(13.0).position_enu_m
        self.assertAlmostEqual(self.trajectory.nearest_time_s(standing), 13.0, places=3)
