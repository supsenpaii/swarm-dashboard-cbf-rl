from __future__ import annotations

import re
import unittest
from pathlib import Path
from unittest import mock

from cbf_command_gate import CbfConfig
from companion_safety import CompanionSafetyMonitor
from formation_controller import FormationSlot
from trajectory_controller import LinearTrajectory


def state(
    position: tuple[float, float, float],
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
    *,
    valid: bool = True,
    age_ms: float = 20.0,
) -> dict:
    return {
        "position_enu_m": list(position),
        "velocity_enu_m_s": list(velocity),
        # Real peer packets carry a JSON null here when the sender has no GPS
        # accuracy estimate, so keep that shape rather than a tidy tuple.
        "position_covariance_m2": None,
        "message_age_ms": age_ms,
        "valid": valid,
    }


def monitor(drone_id: str = "UAV-02") -> CompanionSafetyMonitor:
    return CompanionSafetyMonitor(
        drone_id=drone_id,
        peer_ids=("UAV-01",) if drone_id == "UAV-02" else ("UAV-02",),
        leader_id="UAV-01",
        slots=(FormationSlot("UAV-02", (-10.0, 0.0, 0.0)),),
        cbf_config=CbfConfig(minimum_separation_m=4.0, barrier_gain_s_inv=2.0),
    )


class CompanionSafetyMonitorTests(unittest.TestCase):
    def test_follower_tracks_its_slot_when_everything_is_healthy(self) -> None:
        result = monitor().evaluate(
            {
                "UAV-01": state((0.0, 0.0, 10.0)),
                "UAV-02": state((-20.0, 0.0, 10.0)),
            }
        )

        self.assertTrue(result.nominal_active)
        self.assertEqual(result.nominal_reason, "tracking_slot")
        self.assertTrue(result.command.active)
        self.assertEqual(result.peer_ids_used, ("UAV-01",))

    def test_leader_without_a_slot_still_gets_the_barrier_enforced(self) -> None:
        """The server-side wiring skips CBF when the nominal is inactive, which
        would leave the leader with no collision avoidance at all."""
        result = monitor("UAV-01").evaluate(
            {
                "UAV-01": state((0.0, 0.0, 10.0)),
                "UAV-02": state((3.0, 0.0, 10.0)),
            }
        )

        self.assertFalse(result.nominal_active)
        self.assertEqual(result.nominal_reason, "formation_slot_unassigned")
        # CBF still ran: it saw a peer inside the 4 m separation and did not
        # simply pass the zero nominal through untouched.
        self.assertTrue(result.intervened)

    def test_zero_nominal_is_pushed_away_from_a_peer_inside_the_barrier(self) -> None:
        # 3.5 m of 4 m: recoverable, since h_dot + alpha*h >= 0 needs only
        # about 1.07 m/s of retreat, inside the 2 m/s limit.
        result = monitor("UAV-01").evaluate(
            {
                "UAV-01": state((0.0, 0.0, 10.0)),
                "UAV-02": state((3.5, 0.0, 10.0)),
            }
        )

        self.assertTrue(result.command.active)
        # Peer sits at +east, so a safe command must have a negative east
        # component: holding still violates h_dot + alpha*h >= 0 here.
        self.assertLess(result.command.velocity_enu_m_s[0], 0.0)
        self.assertGreater(result.command.intervention_norm_m_s, 0.0)

    def test_deep_incursion_is_infeasible_at_the_configured_speed_limit(self) -> None:
        """At 2 m the barrier demands roughly 6 m/s of retreat, which the 2 m/s
        maximum cannot deliver, so the filter must fail closed rather than
        emit a command it knows is insufficient."""
        result = monitor("UAV-01").evaluate(
            {
                "UAV-01": state((0.0, 0.0, 10.0)),
                "UAV-02": state((2.0, 0.0, 10.0)),
            }
        )

        self.assertFalse(result.command.active)
        self.assertEqual(result.command.reason, "cbf_constraints_infeasible")
        self.assertEqual(result.command.velocity_enu_m_s, (0.0, 0.0, 0.0))
        self.assertLess(result.command.minimum_margin_m, 0.0)

    def test_stale_peer_fails_closed_and_is_reported_as_missing(self) -> None:
        result = monitor().evaluate(
            {
                "UAV-01": state((0.0, 0.0, 10.0), valid=False),
                "UAV-02": state((-20.0, 0.0, 10.0)),
            }
        )

        self.assertFalse(result.command.active)
        self.assertEqual(result.command.velocity_enu_m_s, (0.0, 0.0, 0.0))
        self.assertEqual(result.peer_ids_missing, ("UAV-01",))
        self.assertTrue(result.intervened)

    def test_missing_peer_entirely_still_fails_closed(self) -> None:
        result = monitor().evaluate({"UAV-02": state((-20.0, 0.0, 10.0))})

        self.assertFalse(result.command.active)
        self.assertEqual(result.command.reason, "peer_state_invalid")

    def test_status_dict_never_claims_control_authority(self) -> None:
        payload = monitor().evaluate(
            {
                "UAV-01": state((0.0, 0.0, 10.0)),
                "UAV-02": state((-20.0, 0.0, 10.0)),
            }
        ).as_dict()

        self.assertEqual(payload["authority"], "shadow_companion_only")
        self.assertEqual(payload["source"], "companion_local_peer_to_peer")
        self.assertEqual(payload["cbf"]["authority"], "shadow_safety_gate_only")


class CompanionSafetyTrajectoryTests(unittest.TestCase):
    def _monitor(self) -> CompanionSafetyMonitor:
        return CompanionSafetyMonitor(
            drone_id="UAV-01",
            peer_ids=("UAV-02",),
            leader_id="UAV-01",
            slots=(),
            cbf_config=CbfConfig(minimum_separation_m=4.0, barrier_gain_s_inv=2.0),
            trajectory=LinearTrajectory((0.0, 0.0, 10.0), (10.0, 0.0, 10.0), speed_m_s=2.0),
        )

    def test_trajectory_is_inactive_until_station_keeping(self) -> None:
        result = self._monitor().evaluate(
            {
                "UAV-01": state((0.0, 0.0, 10.0)),
                "UAV-02": state((0.0, 20.0, 10.0)),
            },
            now_monotonic_s=100.0,
            station_keeping=False,
        )
        self.assertEqual(result.nominal_reason, "trajectory_inactive")
        self.assertFalse(result.nominal_active)

    def test_mission_clock_latches_on_the_station_keeping_transition(self) -> None:
        monitor = self._monitor()
        swarm_state = {
            "UAV-01": state((0.0, 0.0, 10.0)),
            "UAV-02": state((0.0, 20.0, 10.0)),
        }
        # Not station-keeping yet: clock must not be running.
        monitor.evaluate(swarm_state, now_monotonic_s=100.0, station_keeping=False)
        # Transition happens at t=105 -- this is mission_elapsed_s=0, not 105.
        first = monitor.evaluate(swarm_state, now_monotonic_s=105.0, station_keeping=True)
        second = monitor.evaluate(swarm_state, now_monotonic_s=107.0, station_keeping=True)
        self.assertEqual(first.nominal_reason, "tracking_trajectory")
        self.assertEqual(second.nominal_reason, "tracking_trajectory")
        # 2s further along a 2 m/s trajectory should have moved the reference
        # roughly 4m further east than at the transition instant.
        self.assertGreater(
            second.nominal_velocity_enu_m_s[0] - first.nominal_velocity_enu_m_s[0], -0.1
        )

    def test_trajectory_bypasses_altitude_hold_entirely(self) -> None:
        monitor = self._monitor()
        result = monitor.evaluate(
            {
                "UAV-01": state((0.0, 0.0, 10.0)),
                "UAV-02": state((0.0, 20.0, 10.0)),
            },
            now_monotonic_s=100.0,
            station_keeping=True,
        )
        self.assertEqual(result.nominal_reason, "tracking_trajectory")
        self.assertIsNone(monitor.altitude_hold.reference_altitude_m)

    def test_reference_start_is_exposed_without_needing_station_keeping(self) -> None:
        """The pre-flight alignment check has to read this while the vehicle
        is still in POSCTL, i.e. before station-keeping ever becomes true."""
        self.assertEqual(
            self._monitor().trajectory_reference_start_enu_m, (0.0, 0.0, 10.0)
        )

    def test_reference_start_is_none_without_a_trajectory(self) -> None:
        built = CompanionSafetyMonitor(
            drone_id="UAV-01",
            peer_ids=("UAV-02",),
            leader_id="UAV-01",
            slots=(),
        )
        self.assertIsNone(built.trajectory_reference_start_enu_m)

    def test_trajectory_still_gets_the_cbf_barrier_enforced(self) -> None:
        result = self._monitor().evaluate(
            {
                "UAV-01": state((0.0, 0.0, 10.0)),
                "UAV-02": state((2.0, 0.0, 10.0)),
            },
            now_monotonic_s=100.0,
            station_keeping=True,
        )
        self.assertFalse(result.command.active)
        self.assertEqual(result.command.reason, "cbf_constraints_infeasible")

    def test_linear_cbf_rl_observation_uses_the_trained_endpoint_goal(self) -> None:
        runtime = mock.Mock()
        runtime.evaluate_candidate.return_value = ({"valid": True}, None, None)
        monitor = CompanionSafetyMonitor(
            drone_id="UAV-01",
            peer_ids=("UAV-02",),
            leader_id="UAV-01",
            slots=(),
            cbf_config=CbfConfig(minimum_separation_m=4.0, barrier_gain_s_inv=2.0),
            trajectory=LinearTrajectory(
                (0.0, 0.0, 10.0), (140.0, 0.0, 10.0), speed_m_s=10.0
            ),
            cbf_rl_shadow=runtime,
        )

        monitor.evaluate(
            {
                "UAV-01": state((0.0, 0.0, 10.0)),
                "UAV-02": state((100.0, 0.0, 10.0)),
            },
            now_monotonic_s=100.0,
            station_keeping=True,
        )

        self.assertEqual(
            runtime.evaluate_candidate.call_args.kwargs["target_enu_m"],
            (140.0, 0.0, 10.0),
        )


class CompanionSafetyConfigTests(unittest.TestCase):
    def test_environment_matches_the_values_the_server_reads(self) -> None:
        environment = {
            "SWARM_FORMATION_LEADER_ID": "UAV-01",
            "SWARM_FORMATION_SLOT_UAV_02_ENU_M": "-7,2,0",
            "SWARM_CBF_MINIMUM_SEPARATION_M": "5.5",
            "SWARM_CBF_BARRIER_GAIN_S_INV": "1.25",
            "SWARM_CBF_COMMAND_LATENCY_S": "0.35",
            "SWARM_CBF_DESIGN_MARGIN_BUFFER_M": "0.5",
            "SWARM_CBF_GEOFENCE_MAX_ENU_M": "80,80,40",
        }
        with mock.patch.dict("os.environ", environment, clear=True):
            built = CompanionSafetyMonitor.from_environment(
                "UAV-02", ("UAV-01", "UAV-02")
            )

        self.assertEqual(built.gate.config.minimum_separation_m, 5.5)
        self.assertEqual(built.gate.config.barrier_gain_s_inv, 1.25)
        self.assertEqual(built.gate.config.command_latency_s, 0.35)
        self.assertEqual(built.gate.config.design_margin_buffer_m, 0.5)
        self.assertEqual(built.gate.config.geofence_max_enu_m, (80.0, 80.0, 40.0))
        self.assertEqual(built.formation.slots["UAV-02"].offset_enu_m, (-7.0, 2.0, 0.0))
        self.assertEqual(built.peer_ids, ("UAV-01",))

    def test_command_latency_s_defaults_to_the_unvalidated_010(self) -> None:
        """Pins the current default explicitly: 0.10s was never measured
        against real command-to-actuation latency (see .env comment on
        SWARM_CBF_COMMAND_LATENCY_S) and contributed 0.29-0.36m of
        required_margin at the worst frames of both aborted
        ACTIVE_CBF_CROSSING_TRAJECTORY flights. This test exists so a future
        change to the default is a deliberate, reviewed edit to this
        assertion -- not an accidental drift."""
        with mock.patch.dict("os.environ", {}, clear=True):
            built = CompanionSafetyMonitor.from_environment(
                "UAV-02", ("UAV-01", "UAV-02")
            )
        self.assertEqual(built.gate.config.command_latency_s, 0.10)

    def test_design_margin_buffer_defaults_to_zero(self) -> None:
        """Pins that every already-validated milestone's CBF behavior is
        unchanged unless a scenario explicitly opts into
        SWARM_CBF_DESIGN_MARGIN_BUFFER_M -- see CbfConfig.design_margin_buffer_m's
        docstring for why it exists."""
        with mock.patch.dict("os.environ", {}, clear=True):
            built = CompanionSafetyMonitor.from_environment(
                "UAV-02", ("UAV-01", "UAV-02")
            )
        self.assertEqual(built.gate.config.design_margin_buffer_m, 0.0)

    def test_configured_command_latency_is_at_least_the_measured_value(self) -> None:
        """Guards against silently reverting .env's SWARM_CBF_COMMAND_LATENCY_S
        to the unvalidated 0.10 default (or anything below it) without
        noticing.

        LATENCY_MEASUREMENT_FLIGHT (2026-08-10) measured 0.591-0.698s (median
        0.653s) across 8 independent step edges -- see the .env comment on
        SWARM_CBF_COMMAND_LATENCY_S for the full flight. Reads .env directly,
        same reasoning as test_formation_spawn_geometry.py: this must track
        whatever is actually configured, not a duplicated literal that could
        silently drift from it.
        """
        env_path = Path(__file__).resolve().parent / ".env"
        if not env_path.exists():
            self.skipTest("no .env in this checkout")
        match = re.search(
            r"^SWARM_CBF_COMMAND_LATENCY_S=(.+)$", env_path.read_text(encoding="utf-8"), re.MULTILINE
        )
        if match is None:
            self.skipTest("SWARM_CBF_COMMAND_LATENCY_S not set in .env")
        configured = float(match.group(1).strip())
        self.assertGreaterEqual(
            configured,
            0.591,
            "SWARM_CBF_COMMAND_LATENCY_S is below the minimum value measured "
            "by LATENCY_MEASUREMENT_FLIGHT across 8 real step edges -- "
            "re-run latency_measurement_flight.py before lowering this.",
        )

    def test_leader_gets_no_slot_but_keeps_every_peer(self) -> None:
        with mock.patch.dict("os.environ", {}, clear=True):
            built = CompanionSafetyMonitor.from_environment(
                "UAV-01", ("UAV-01", "UAV-02")
            )

        self.assertNotIn("UAV-01", built.formation.slots)
        self.assertEqual(built.peer_ids, ("UAV-02",))

    def test_unset_trajectory_kind_leaves_formation_nominal_untouched(self) -> None:
        with mock.patch.dict("os.environ", {}, clear=True):
            built = CompanionSafetyMonitor.from_environment(
                "UAV-01", ("UAV-01", "UAV-02")
            )

        self.assertIsNone(built.trajectory_tracking)

    def test_linear_trajectory_kind_is_parsed_from_environment(self) -> None:
        environment = {
            "SWARM_TRAJECTORY_UAV_01_KIND": "linear",
            "SWARM_TRAJECTORY_UAV_01_START_ENU_M": "0,0,10",
            "SWARM_TRAJECTORY_UAV_01_END_ENU_M": "20,0,10",
            "SWARM_TRAJECTORY_UAV_01_SPEED_M_S": "1.5",
        }
        with mock.patch.dict("os.environ", environment, clear=True):
            built = CompanionSafetyMonitor.from_environment(
                "UAV-01", ("UAV-01", "UAV-02")
            )

        self.assertIsNotNone(built.trajectory_tracking)
        reference = built.trajectory_tracking.trajectory.reference(0.0)
        self.assertEqual(reference.position_enu_m, (0.0, 0.0, 10.0))
        self.assertEqual(reference.velocity_enu_m_s, (1.5, 0.0, 0.0))

    def test_polyline_trajectory_kind_is_parsed_from_environment(self) -> None:
        environment = {
            "SWARM_TRAJECTORY_UAV_01_KIND": "polyline",
            "SWARM_TRAJECTORY_UAV_01_WAYPOINTS_ENU_M": (
                "0,0,30:240,0,30:240,240,30:0,240,30"
            ),
            "SWARM_TRAJECTORY_UAV_01_SPEED_M_S": "15",
            "SWARM_FORMATION_MAXIMUM_VELOCITY_M_S": "15",
        }
        with mock.patch.dict("os.environ", environment, clear=True):
            built = CompanionSafetyMonitor.from_environment(
                "UAV-01", ("UAV-01", "UAV-02")
            )

        trajectory = built.trajectory_tracking.trajectory
        self.assertEqual(len(trajectory.waypoints_enu_m), 4)
        self.assertEqual(trajectory.reference(0.0).position_enu_m, (0.0, 0.0, 30.0))

    def test_polyline_without_waypoints_fails_loud(self) -> None:
        with mock.patch.dict(
            "os.environ", {"SWARM_TRAJECTORY_UAV_01_KIND": "polyline"}, clear=True
        ):
            with self.assertRaises(ValueError):
                CompanionSafetyMonitor.from_environment("UAV-01", ("UAV-01", "UAV-02"))

    def test_unknown_trajectory_kind_fails_loud(self) -> None:
        with mock.patch.dict(
            "os.environ", {"SWARM_TRAJECTORY_UAV_01_KIND": "spiral"}, clear=True
        ):
            with self.assertRaises(ValueError):
                CompanionSafetyMonitor.from_environment("UAV-01", ("UAV-01", "UAV-02"))


if __name__ == "__main__":
    unittest.main()
