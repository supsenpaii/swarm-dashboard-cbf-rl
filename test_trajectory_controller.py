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
    braking_speed_limit_m_s,
    corner_profile,
    fillet_radius_m,
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

    def test_on_the_line_it_commands_cruise_and_nothing_more(self):
        """A vehicle on its path is not behind: the leg is followed from
        measured progress, so there is no clock for it to fall behind.

        This used to assert the opposite -- that a vehicle sitting at the
        start with the mission clock at 2.0 s would be commanded to chase a
        reference 4 m ahead. That chase is exactly what pinned the 20 m/s rung
        at 2.4 m/s: the resulting along-track error tripped the companion's
        5 m runaway guard on every lap.
        """
        command = self.controller.command(2.0, {"UAV-01": state((0.0, 0.0, 5.0))})

        self.assertTrue(command.active)
        self.assertEqual(command.reason, "tracking_trajectory")
        self.assertAlmostEqual(command.velocity_enu_m_s[0], 2.0)
        self.assertAlmostEqual(command.velocity_enu_m_s[1], 0.0)

    def test_off_the_line_it_corrects_toward_the_line(self):
        # 3 m to the left of the leg, a third of the way along it.
        command = self.controller.command(0.0, {"UAV-01": state((3.0, 3.0, 5.0))})

        self.assertEqual(command.reason, "tracking_trajectory")
        self.assertAlmostEqual(command.velocity_enu_m_s[0], 2.0)   # cruise kept
        self.assertAlmostEqual(command.velocity_enu_m_s[1], -1.5)  # 0.5 * -3.0

    def test_the_mission_clock_no_longer_moves_the_reference(self):
        """The contract the 2026-08-17 change is really about."""
        def fresh():
            return TrajectoryTrackingController(
                "UAV-01", self.trajectory, self.controller.config
            )

        at_zero = fresh().command(0.0, {"UAV-01": state((3.0, 3.0, 5.0))})
        much_later = fresh().command(900.0, {"UAV-01": state((3.0, 3.0, 5.0))})

        self.assertEqual(at_zero.velocity_enu_m_s, much_later.velocity_enu_m_s)
        self.assertEqual(at_zero.reason, much_later.reason)

    def test_command_is_velocity_limited(self):
        command = self.controller.command(2.0, {"UAV-01": state((-100.0, 0.0, 5.0))})
        self.assertLessEqual(math.sqrt(sum(v * v for v in command.velocity_enu_m_s)), 3.0)

    def test_reports_trajectory_reached_once_finished_and_in_radius(self):
        command = self.controller.command(999.0, {"UAV-01": state((10.0, 0.0, 5.0))})
        self.assertEqual(command.reason, "trajectory_reached")
        self.assertEqual(command.velocity_enu_m_s, (0.0, 0.0, 0.0))

    def test_a_stale_clock_does_not_finish_a_leg_the_vehicle_is_halfway_down(self):
        # Renamed from "past duration": the clock reading 999 s no longer says
        # anything about how far along the leg the vehicle is. Half a leg from
        # the end is half a leg from the end, whatever the clock says.
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
        # 1.795 m before the fillet radius carried its cosine and before the
        # corner-exit pass existed, 0.476 m with both.
        self.assertLess(max(errors), 0.6)

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


class CornerExitProfileTests(unittest.TestCase):
    """The corner-exit pass is what keeps tracking speed-independent."""

    def fly_square(self, speed_m_s, tolerance_m=2.0, acceleration_m_s2=4.0):
        edge_m = 300.0
        trajectory = ClosedPolylineTrajectory(
            waypoints_enu_m=(
                (0.0, 0.0, 9.0),
                (edge_m, 0.0, 9.0),
                (edge_m, edge_m, 9.0),
                (0.0, edge_m, 9.0),
            ),
            speed_m_s=speed_m_s,
        )
        controller = TrajectoryTrackingController(
            "UAV-01",
            trajectory,
            FormationConfig(maximum_velocity_m_s=speed_m_s),
            maximum_acceleration_m_s2=acceleration_m_s2,
            corner_tracking_tolerance_m=tolerance_m,
        )
        step_s = 0.05
        position, velocity, errors, speeds = [0.0, 0.0, 9.0], [0.0, 0.0, 0.0], [], []
        for step in range(int(1.4 * 4 * edge_m / speed_m_s / step_s)):
            command = controller.command(
                step * step_s,
                {"UAV-01": state(tuple(position), velocity=tuple(velocity))},
            )
            velocity = list(command.velocity_enu_m_s)
            position = [position[a] + velocity[a] * step_s for a in range(3)]
            errors.append(command.position_error_m or 0.0)
            speeds.append(math.dist((0.0, 0.0, 0.0), velocity))
        return max(errors), max(speeds)

    def test_sparrow_holds_the_corner_budget_from_10_to_25_ms(self):
        for speed_m_s in (10.0, 15.0, 20.0, 25.0):
            with self.subTest(speed_m_s=speed_m_s):
                worst_m, fastest_m_s = self.fly_square(speed_m_s)
                # Without the exit pass this ran 1.21 m at 10 m/s and 3.03 m at
                # 25 m/s -- the error tracked speed instead of the tolerance.
                self.assertLess(worst_m, 2.0, "cross-track left the corner budget")
                self.assertGreater(
                    fastest_m_s, 0.99 * speed_m_s, "never reached cruise speed"
                )

    def test_tightening_the_tolerance_slows_the_corner(self):
        loose_error_m, _ = self.fly_square(25.0, tolerance_m=5.0)
        tight_error_m, _ = self.fly_square(25.0, tolerance_m=1.0)
        self.assertLess(tight_error_m, loose_error_m)


class FilletRadiusTests(unittest.TestCase):
    def test_arc_stays_inside_the_tolerance_at_every_turn_angle(self):
        """Measure the built arc instead of trusting the closed form."""
        for turn_deg in (15, 30, 60, 90, 120, 150):
            for tolerance_m in (0.5, 1.0, 2.0):
                turn_rad = math.radians(turn_deg)
                radius_m = fillet_radius_m(tolerance_m, turn_rad)
                # Vertex at the origin, bisector along +y: the arc centre sits
                # radius/cos(half) up the bisector.
                centre_y = radius_m / math.cos(0.5 * turn_rad)
                measured_m = min(
                    math.hypot(radius_m * math.cos(t), centre_y + radius_m * math.sin(t))
                    for t in (i * math.pi / 2000 for i in range(4001))
                )
                self.assertAlmostEqual(
                    measured_m,
                    tolerance_m,
                    places=3,
                    msg=f"{turn_deg} deg turn, {tolerance_m} m tolerance",
                )

    def test_sharper_turns_need_tighter_arcs(self):
        radii = [fillet_radius_m(1.0, math.radians(d)) for d in (30, 60, 90, 120, 150)]
        self.assertEqual(radii, sorted(radii, reverse=True))


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


class MissionSpeedPreviewTests(unittest.TestCase):
    def preview(self, waypoints, speed_m_s, tolerance_m=2.0):
        from trajectory_controller import mission_speed_preview

        trajectory = ClosedPolylineTrajectory(
            waypoints_enu_m=tuple(waypoints), speed_m_s=speed_m_s
        )
        return mission_speed_preview(
            trajectory,
            maximum_acceleration_m_s2=4.0,
            corner_tracking_tolerance_m=tolerance_m,
        )

    def test_a_long_lap_reaches_the_requested_cruise(self):
        report = self.preview(
            ((0.0, 0.0, 9.0), (300.0, 0.0, 9.0), (300.0, 300.0, 9.0), (0.0, 300.0, 9.0)),
            25.0,
        )
        self.assertTrue(report["reaches_requested_speed"])
        self.assertAlmostEqual(report["achievable_speed_m_s"], 25.0, places=2)

    def test_a_short_lap_reports_the_speed_it_can_actually_fly(self):
        """The operator asks for 25 m/s on a 20 m square and must be told no."""
        report = self.preview(
            ((0.0, 0.0, 9.0), (20.0, 0.0, 9.0), (20.0, 20.0, 9.0), (0.0, 20.0, 9.0)),
            25.0,
        )
        self.assertFalse(report["reaches_requested_speed"])
        self.assertLess(report["achievable_speed_m_s"], 12.0)
        self.assertEqual(report["requested_speed_m_s"], 25.0)

    def test_a_tighter_tolerance_slows_the_slowest_corner(self):
        square = ((0.0, 0.0, 9.0), (300.0, 0.0, 9.0), (300.0, 300.0, 9.0), (0.0, 300.0, 9.0))
        loose = self.preview(square, 25.0, tolerance_m=5.0)
        tight = self.preview(square, 25.0, tolerance_m=1.0)
        self.assertLess(tight["slowest_corner_m_s"], loose["slowest_corner_m_s"])


class ResponseLagCornerTests(unittest.TestCase):
    """A corner budget is geometric; a real vehicle spends part of it catching up.

    Measured on a 240 m square at 15 m/s with a 1 m tolerance and Sparrow's
    0.860 s response: 3.489 m of cross-track when the lag is ignored, 0.145 m
    when it is planned for. The aircraft was arriving at a 90 degree corner at
    5.06 m/s against a command of about 1 m/s, having never been given the
    distance to shed it.
    """

    def test_zero_lag_is_the_textbook_result(self) -> None:
        """Default behaviour is unchanged for every caller that has not
        measured its vehicle."""
        self.assertAlmostEqual(
            braking_speed_limit_m_s(2.0, 10.0, 4.0, 0.0),
            math.sqrt(2.0**2 + 1.4 * 4.0 * 10.0),
        )

    def test_lag_lowers_the_approach_speed(self) -> None:
        without = braking_speed_limit_m_s(2.0, 10.0, 4.0, 0.0)
        with_lag = braking_speed_limit_m_s(2.0, 10.0, 4.0, 0.86)

        self.assertLess(with_lag, without)
        # The root is the speed whose own lag distance still leaves enough
        # room to brake, so substituting it back reproduces the geometry.
        self.assertAlmostEqual(
            with_lag**2,
            2.0**2 + 1.4 * 4.0 * (10.0 - with_lag * 0.86),
            places=6,
        )

    def test_the_corner_speed_itself_is_capped_by_the_lag(self) -> None:
        _, without_m_s, _ = corner_profile(math.pi / 2, 240.0, 240.0, 15.0, 4.0, 1.0)
        _, with_lag_m_s, _ = corner_profile(
            math.pi / 2, 240.0, 240.0, 15.0, 4.0, 1.0, 0.86
        )

        # v * tau * sin(half) inside the tolerance, at a 90 degree turn.
        self.assertAlmostEqual(
            with_lag_m_s, 1.0 / (0.86 * math.sin(math.pi / 4)), places=6
        )
        self.assertLess(with_lag_m_s, without_m_s)

    def test_the_cap_scales_with_how_sharp_the_turn_is(self) -> None:
        """An angle-blind cap would hold a 1 degree bend to a right angle's
        speed -- and on a gentle bend the corner window covers nearly half the
        leg, so the whole mission would crawl."""
        speeds = [
            corner_profile(
                math.radians(degrees), 240.0, 240.0, 15.0, 4.0, 1.0, 0.86
            )[1]
            for degrees in (90.0, 45.0, 15.0, 1.0)
        ]

        self.assertEqual(speeds, sorted(speeds))
        self.assertLess(speeds[0], 2.0)
        # A 1 degree bend needs no help from the lag term at all.
        self.assertEqual(speeds[-1], 15.0)
