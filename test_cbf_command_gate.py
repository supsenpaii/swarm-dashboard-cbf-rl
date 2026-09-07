import unittest

from cbf_command_gate import CbfCommandGate, CbfConfig


def state(position, velocity=(0.0, 0.0, 0.0), *, valid=True, covariance=(0.0, 0.0, 0.0), age_ms=0.0):
    return {
        "valid": valid,
        "position_enu_m": position,
        "velocity_enu_m_s": velocity,
        "position_covariance_m2": covariance,
        "message_age_ms": age_ms,
    }


class CbfCommandGateTests(unittest.TestCase):
    def setUp(self):
        self.gate = CbfCommandGate(
            "UAV-02", ("UAV-01",),
            CbfConfig(minimum_separation_m=4.0, maximum_velocity_m_s=2.0, geofence_min_enu_m=(-20.0, -20.0, 0.0), geofence_max_enu_m=(20.0, 20.0, 20.0)),
        )

    def test_nominal_velocity_passes_when_safely_separated(self):
        command = self.gate.filter((1.0, 0.0, 0.0), {"UAV-02": state((0.0, 0.0, 5.0)), "UAV-01": state((10.0, 0.0, 5.0))})
        self.assertTrue(command.active)
        self.assertEqual(command.reason, "cbf_filtered")
        self.assertGreater(command.minimum_margin_m, 0.0)

    def test_head_on_velocity_is_modified_to_respect_separation(self):
        command = self.gate.filter((2.0, 0.0, 0.0), {"UAV-02": state((0.0, 0.0, 5.0)), "UAV-01": state((4.5, 0.0, 5.0), (-1.0, 0.0, 0.0))})
        self.assertTrue(command.active)
        self.assertLess(command.velocity_enu_m_s[0], 2.0)
        self.assertGreater(command.intervention_norm_m_s, 0.0)

    def test_stale_peer_fails_closed_to_hold(self):
        command = self.gate.filter((1.0, 0.0, 0.0), {"UAV-02": state((0.0, 0.0, 5.0)), "UAV-01": state((10.0, 0.0, 5.0), valid=False)})
        self.assertFalse(command.active)
        self.assertEqual(command.reason, "peer_state_invalid")

    def test_geofence_clamps_outward_command(self):
        command = self.gate.filter((2.0, 0.0, 0.0), {"UAV-02": state((19.9, 0.0, 5.0)), "UAV-01": state((0.0, 0.0, 5.0))})
        self.assertTrue(command.active)
        self.assertLessEqual(command.velocity_enu_m_s[0], 0.10001)

    def test_overlap_fails_closed(self):
        command = self.gate.filter((1.0, 0.0, 0.0), {"UAV-02": state((0.0, 0.0, 5.0)), "UAV-01": state((0.0, 0.0, 5.0))})
        self.assertFalse(command.active)
        self.assertEqual(command.reason, "peer_position_overlap")

    def test_reports_distance_and_required_separation_behind_the_margin(self):
        command = self.gate.filter(
            (0.0, 0.0, 0.0),
            {
                "UAV-02": state(
                    (0.0, 0.0, 5.0), covariance=(0.06, 0.06, 0.06)
                ),
                "UAV-01": state(
                    (5.1, 0.0, 5.0), covariance=(0.06, 0.06, 0.06)
                ),
            },
        )
        self.assertEqual(command.critical_peer_id, "UAV-01")
        self.assertAlmostEqual(command.critical_distance_m, 5.1)
        self.assertAlmostEqual(command.critical_required_separation_m, 5.2)
        self.assertAlmostEqual(command.minimum_margin_m, -0.1)

    def test_design_margin_buffer_commands_a_stronger_evasive_correction(self):
        """Solve-time-only buffer: a single filter() call reports margin
        against the CURRENT (unbuffered) distance regardless of the buffer --
        that reported number only diverges from the unbuffered case over
        time, as the buffered solve drives the vehicle to actually hold more
        distance in reserve (see design_margin_buffer_m's own docstring for
        the closed-loop investigation that established this; a multi-step
        simulation, not a single filter() call, is what shows the reported-
        margin effect). What a single call CAN show directly: the buffered
        solve targets a stricter boundary, so it must command a stronger
        evasive correction right now than the unbuffered solve does, for the
        identical input state."""
        scenario = {
            "UAV-02": state((0.0, 0.0, 5.0)),
            "UAV-01": state((4.5, 0.0, 5.0), (-1.0, 0.0, 0.0)),
        }
        baseline = CbfCommandGate(
            "UAV-02", ("UAV-01",),
            CbfConfig(minimum_separation_m=4.0, maximum_velocity_m_s=2.0, design_margin_buffer_m=0.0),
        ).filter((2.0, 0.0, 0.0), scenario)
        buffered = CbfCommandGate(
            "UAV-02", ("UAV-01",),
            CbfConfig(minimum_separation_m=4.0, maximum_velocity_m_s=2.0, design_margin_buffer_m=0.5),
        ).filter((2.0, 0.0, 0.0), scenario)

        self.assertTrue(baseline.active)
        self.assertTrue(buffered.active)
        self.assertGreater(buffered.intervention_norm_m_s, baseline.intervention_norm_m_s)
        # Reported margin is identical in a single snapshot -- it is a
        # function of the CURRENT distance, which neither call changed.
        self.assertAlmostEqual(buffered.minimum_margin_m, baseline.minimum_margin_m)

    def test_design_margin_buffer_defaults_to_zero_unbuffered_behavior(self):
        """Pins that CbfConfig()'s default leaves every already-validated
        milestone's CBF behavior exactly as it was before this field
        existed."""
        self.assertEqual(CbfConfig().design_margin_buffer_m, 0.0)

    def test_negative_design_margin_buffer_is_rejected(self):
        with self.assertRaises(ValueError):
            CbfConfig(design_margin_buffer_m=-0.1)

    def test_explicit_null_covariance_defaults_to_zero_uncertainty(self):
        # Real telemetry serializes missing covariance as JSON null, i.e. a
        # key present with value None, not an absent key.
        command = self.gate.filter(
            (1.0, 0.0, 0.0),
            {
                "UAV-02": state((0.0, 0.0, 5.0), covariance=None),
                "UAV-01": state((10.0, 0.0, 5.0), covariance=None),
            },
        )
        self.assertTrue(command.active)
        self.assertEqual(command.reason, "cbf_filtered")

    def test_high_speed_head_on_reserve_includes_latency_braking_and_tracking(self):
        gate = CbfCommandGate(
            "UAV-02",
            ("UAV-01",),
            CbfConfig(
                minimum_separation_m=20.0,
                maximum_velocity_m_s=10.0,
                covariance_sigma=0.0,
                command_latency_s=0.65,
                relative_braking_acceleration_m_s2=8.0,
                tracking_reserve_m=2.0,
            ),
        )
        command = gate.filter(
            (-10.0, 0.0, 0.0),
            {
                "UAV-02": state((60.0, 0.0, 20.0), (-10.0, 0.0, 0.0)),
                "UAV-01": state((0.0, 0.0, 20.0), (10.0, 0.0, 0.0)),
            },
        )

        # 20 m hard floor + 13 m latency + 25 m braking + 2 m reserve.
        self.assertAlmostEqual(command.critical_required_separation_m, 60.0)
        self.assertAlmostEqual(command.minimum_margin_m, 0.0)

    def test_high_speed_tangential_motion_does_not_add_braking_distance(self):
        gate = CbfCommandGate(
            "UAV-02",
            ("UAV-01",),
            CbfConfig(
                minimum_separation_m=20.0,
                maximum_velocity_m_s=10.0,
                covariance_sigma=0.0,
                command_latency_s=0.65,
                relative_braking_acceleration_m_s2=8.0,
                tracking_reserve_m=2.0,
            ),
        )
        command = gate.filter(
            (0.0, 10.0, 0.0),
            {
                "UAV-02": state((30.0, 0.0, 20.0), (0.0, 10.0, 0.0)),
                "UAV-01": state((0.0, 0.0, 20.0), (0.0, -10.0, 0.0)),
            },
        )

        self.assertAlmostEqual(command.critical_required_separation_m, 22.0)


if __name__ == "__main__":
    unittest.main()


class ReachableVelocityTests(unittest.TestCase):
    """Opt-in reachability: the solve may only pick a velocity the vehicle can take.

    Measured on the high-speed matrix with it enabled at 4 m/s^2 / 20 Hz:
    safety never degraded -- physical_safe and dynamic_safe stayed true in all
    280 -- but 178 cases stopped passing on liveness, with hold frames and 87
    failures to reach the goal. The 0.2 m/s of authority a frame buys is less
    than the correction the barrier asks for, so the solve reports infeasible
    rather than reaching. It stays off until a barrier that spreads the
    correction over frames justifies turning it on.
    """

    def config(self, **overrides):
        return CbfConfig(
            minimum_separation_m=4.0,
            maximum_velocity_m_s=2.0,
            geofence_min_enu_m=(-20.0, -20.0, 0.0),
            geofence_max_enu_m=(20.0, 20.0, 20.0),
            **overrides,
        )

    def test_off_by_default_leaves_the_command_unbounded_by_acceleration(self):
        gate = CbfCommandGate("UAV-02", ("UAV-01",), self.config())
        command = gate.filter(
            (2.0, 0.0, 0.0),
            {"UAV-02": state((0.0, 0.0, 5.0)), "UAV-01": state((15.0, 0.0, 5.0))},
        )
        self.assertTrue(command.active)
        # Standing start to full command in one frame, the legacy contract.
        self.assertAlmostEqual(command.velocity_enu_m_s[0], 2.0, places=6)

    def test_enabled_keeps_the_command_inside_one_frame_of_acceleration(self):
        gate = CbfCommandGate(
            "UAV-02",
            ("UAV-01",),
            self.config(maximum_acceleration_m_s2=4.0, control_period_s=0.05),
        )
        command = gate.filter(
            (2.0, 0.0, 0.0),
            {"UAV-02": state((0.0, 0.0, 5.0)), "UAV-01": state((15.0, 0.0, 5.0))},
        )
        self.assertTrue(command.active)
        self.assertAlmostEqual(command.velocity_enu_m_s[0], 0.2, places=6)

    def test_reachability_is_measured_from_the_vehicle_not_from_zero(self):
        gate = CbfCommandGate(
            "UAV-02",
            ("UAV-01",),
            self.config(maximum_acceleration_m_s2=4.0, control_period_s=0.05),
        )
        command = gate.filter(
            (2.0, 0.0, 0.0),
            {
                "UAV-02": state((0.0, 0.0, 5.0), velocity=(1.0, 0.0, 0.0)),
                "UAV-01": state((15.0, 0.0, 5.0)),
            },
        )
        self.assertAlmostEqual(command.velocity_enu_m_s[0], 1.2, places=6)

    def test_a_positive_acceleration_needs_a_positive_control_period(self):
        with self.assertRaises(ValueError):
            self.config(maximum_acceleration_m_s2=4.0, control_period_s=0.0)
