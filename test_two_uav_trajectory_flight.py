"""Alignment gate for TWO_UAV_TRAJECTORY_TRACKING.

The gate's whole job is to refuse a flight whose trajectory reference is
expressed in a different frame -- or simply a different place -- from where
the vehicles actually are, at the last moment that refusal is still free.
These tests pin that decision without a stack, which is also the only way to
exercise the refusal branches: a real flight that trips this gate is exactly
what the gate exists to prevent.
"""

from __future__ import annotations

import unittest
from unittest import mock

from two_uav_trajectory_flight import (
    AltitudeTrendHoverGate,
    INITIAL_ERROR_LIMIT_DEFAULT_M,
    Flight,
    FlightAbort,
    check_initial_frame_alignment,
    initial_error_limit_m,
)

LIMIT = 1.5


class AltitudeTrendHoverGateTests(unittest.TestCase):
    def test_stable_altitude_passes_despite_a_biased_velocity_field(self) -> None:
        gate = AltitudeTrendHoverGate()
        for second in range(5):
            reached = gate.reached(10.0 + 0.02 * (second % 2), now_s=float(second))
        self.assertTrue(reached)
        with mock.patch("two_uav_trajectory_flight.time.monotonic", return_value=4.5):
            self.assertTrue(
                gate({"altitude_m": 10.0, "vertical_velocity_m_s": -0.4})
            )

    def test_a_continuing_climb_does_not_count_as_hover(self) -> None:
        gate = AltitudeTrendHoverGate()
        for second in range(6):
            reached = gate.reached(8.0 + 0.25 * second, now_s=float(second))
        self.assertFalse(reached)

    def test_short_or_malformed_history_fails_closed(self) -> None:
        gate = AltitudeTrendHoverGate()
        self.assertFalse(gate.reached(None, now_s=0.0))
        self.assertFalse(gate.reached(float("nan"), now_s=1.0))
        self.assertFalse(gate.reached(10.0, now_s=2.0))


def vehicle_state(
    drone_id: str,
    *,
    actual=(0.0, 0.0, 9.0),
    reference=(0.0, 0.0, 9.0),
    companion_drone_id: str | None = "__same__",
) -> dict:
    return {
        "companion_drone_id": drone_id if companion_drone_id == "__same__" else companion_drone_id,
        "own_position_enu_m": None if actual is None else list(actual),
        "trajectory_reference_start_enu_m": None if reference is None else list(reference),
    }


def snapshot(**overrides) -> dict[str, dict]:
    base = {
        "UAV-01": vehicle_state("UAV-01"),
        "UAV-02": vehicle_state("UAV-02", actual=(-5.0, 2.0, 9.0), reference=(-5.0, 2.0, 9.0)),
    }
    base.update(overrides)
    return base


class InitialFrameAlignmentTests(unittest.TestCase):
    def test_small_error_passes(self) -> None:
        results, failure = check_initial_frame_alignment(
            snapshot(
                **{"UAV-01": vehicle_state("UAV-01", actual=(0.2, -0.1, 9.3))}
            ),
            limit_m=LIMIT,
        )
        self.assertIsNone(failure)
        self.assertTrue(all(result.ok for result in results))
        self.assertAlmostEqual(results[0].initial_tracking_error_m, 0.37416, places=4)

    def test_error_exactly_at_the_limit_passes(self) -> None:
        """Boundary is inclusive: only a STRICTLY greater error is refused.
        Pinned because a flight refused at exactly the configured limit would
        be indistinguishable from an off-by-one in the comparison."""
        results, failure = check_initial_frame_alignment(
            snapshot(**{"UAV-01": vehicle_state("UAV-01", actual=(0.0, 0.0, 9.0 + LIMIT))}),
            limit_m=LIMIT,
        )
        self.assertIsNone(failure)
        self.assertAlmostEqual(results[0].initial_tracking_error_m, LIMIT)
        self.assertTrue(results[0].ok)

    def test_error_just_over_the_limit_fails(self) -> None:
        results, failure = check_initial_frame_alignment(
            snapshot(
                **{"UAV-01": vehicle_state("UAV-01", actual=(0.0, 0.0, 9.0 + LIMIT + 0.001))}
            ),
            limit_m=LIMIT,
        )
        self.assertIsNotNone(failure)
        self.assertIn("trajectory_initial_position_mismatch:UAV-01", failure)
        self.assertFalse(results[0].ok)

    def test_uav01_frame_offset_fails(self) -> None:
        """A wrong ENU origin shows up as a large, roughly constant offset --
        the scenario this gate was added for."""
        _results, failure = check_initial_frame_alignment(
            snapshot(**{"UAV-01": vehicle_state("UAV-01", actual=(-43.0, -25.5, 9.0))}),
            limit_m=LIMIT,
        )
        self.assertIn("trajectory_initial_position_mismatch:UAV-01", failure)

    def test_uav02_frame_offset_fails(self) -> None:
        _results, failure = check_initial_frame_alignment(
            snapshot(
                **{
                    "UAV-02": vehicle_state(
                        "UAV-02", actual=(-48.0, -23.5, 9.0), reference=(-5.0, 2.0, 9.0)
                    )
                }
            ),
            limit_m=LIMIT,
        )
        self.assertIn("trajectory_initial_position_mismatch:UAV-02", failure)

    def test_both_vehicles_are_reported_not_just_the_first_failure(self) -> None:
        _results, failure = check_initial_frame_alignment(
            snapshot(
                **{
                    "UAV-01": vehicle_state("UAV-01", actual=(30.0, 0.0, 9.0)),
                    "UAV-02": vehicle_state(
                        "UAV-02", actual=(30.0, 2.0, 9.0), reference=(-5.0, 2.0, 9.0)
                    ),
                }
            ),
            limit_m=LIMIT,
        )
        self.assertIn("UAV-01", failure)
        self.assertIn("UAV-02", failure)

    def test_cross_identity_fails_before_any_position_is_trusted(self) -> None:
        """A stream serving UAV-02's state under UAV-01's key would otherwise
        pass every positional check against the wrong aircraft."""
        results, failure = check_initial_frame_alignment(
            snapshot(**{"UAV-01": vehicle_state("UAV-01", companion_drone_id="UAV-02")}),
            limit_m=LIMIT,
        )
        self.assertIn("trajectory_identity_mismatch:UAV-01", failure)
        self.assertIsNone(results[0].initial_tracking_error_m)
        self.assertIsNone(results[0].actual_start_enu_m)

    def test_missing_identity_fails(self) -> None:
        _results, failure = check_initial_frame_alignment(
            snapshot(**{"UAV-01": vehicle_state("UAV-01", companion_drone_id=None)}),
            limit_m=LIMIT,
        )
        self.assertIn("trajectory_identity_mismatch:UAV-01", failure)

    def test_trajectory_not_configured_fails(self) -> None:
        """No reference published means the companion is running the formation
        nominal, not a trajectory -- the flight would measure nothing."""
        _results, failure = check_initial_frame_alignment(
            snapshot(**{"UAV-01": vehicle_state("UAV-01", reference=None)}),
            limit_m=LIMIT,
        )
        self.assertIn("trajectory_not_configured:UAV-01", failure)

    def test_missing_own_position_fails_closed(self) -> None:
        _results, failure = check_initial_frame_alignment(
            snapshot(**{"UAV-01": vehicle_state("UAV-01", actual=None)}),
            limit_m=LIMIT,
        )
        self.assertIn("trajectory_initial_position_unavailable:UAV-01", failure)

    def test_malformed_position_fails_closed(self) -> None:
        for malformed in ([0.0, 0.0], "0,0,9", [0.0, 0.0, float("nan")], [0.0, 0.0, None]):
            with self.subTest(malformed=malformed):
                _results, failure = check_initial_frame_alignment(
                    {
                        "UAV-01": {
                            "companion_drone_id": "UAV-01",
                            "own_position_enu_m": malformed,
                            "trajectory_reference_start_enu_m": [0.0, 0.0, 9.0],
                        },
                        "UAV-02": vehicle_state(
                            "UAV-02", actual=(-5.0, 2.0, 9.0), reference=(-5.0, 2.0, 9.0)
                        ),
                    },
                    limit_m=LIMIT,
                )
                self.assertIn("trajectory_initial_position_unavailable:UAV-01", failure)

    def test_entirely_missing_vehicle_fails_closed(self) -> None:
        _results, failure = check_initial_frame_alignment(
            {"UAV-02": vehicle_state("UAV-02", actual=(-5.0, 2.0, 9.0), reference=(-5.0, 2.0, 9.0))},
            limit_m=LIMIT,
        )
        self.assertIn("trajectory_identity_mismatch:UAV-01", failure)


class InitialErrorLimitTests(unittest.TestCase):
    def test_default_when_unset(self) -> None:
        with mock.patch.dict("os.environ", {}, clear=True):
            self.assertEqual(initial_error_limit_m(), INITIAL_ERROR_LIMIT_DEFAULT_M)

    def test_operator_override_is_honoured(self) -> None:
        with mock.patch.dict(
            "os.environ", {"SWARM_TRAJECTORY_INITIAL_ERROR_MAX_M": "0.75"}, clear=True
        ):
            self.assertEqual(initial_error_limit_m(), 0.75)

    def test_nonsense_values_fall_back_rather_than_disabling_the_guard(self) -> None:
        for value in ("", "abc", "0", "-3", "nan"):
            with self.subTest(value=value):
                with mock.patch.dict(
                    "os.environ",
                    {"SWARM_TRAJECTORY_INITIAL_ERROR_MAX_M": value},
                    clear=True,
                ):
                    self.assertEqual(initial_error_limit_m(), INITIAL_ERROR_LIMIT_DEFAULT_M)

    def test_absurdly_large_override_is_clamped(self) -> None:
        with mock.patch.dict(
            "os.environ", {"SWARM_TRAJECTORY_INITIAL_ERROR_MAX_M": "1e9"}, clear=True
        ):
            self.assertEqual(initial_error_limit_m(), 50.0)


class GateStopsTheFlightTests(unittest.TestCase):
    """The gate has to stop the SEQUENCE, not merely report -- a refusal that
    still handed authority to PX4 would be worthless."""

    def _flight_with_telemetry(self, payload: dict) -> Flight:
        flight = Flight(hold_s=1.0, dry_run=True)
        flight.telemetry = lambda: payload  # type: ignore[method-assign]
        return flight

    @staticmethod
    def _payload(uav01_actual, uav02_actual) -> dict:
        def entry(drone_id, actual, reference):
            return {
                "companion_safety": {
                    "drone_id": drone_id,
                    "own_position_enu_m": list(actual),
                    "trajectory_reference_start_enu_m": list(reference),
                    "active_offboard_sender": {
                        "transmit_sink_attached": True,
                        "explicit_opt_in": True,
                        "stream_duration_s": 30.0,
                        "transmit_count": 500,
                        "max_transmit_gap_s": 0.2,
                    },
                    "active_offboard_frame": {"decision": "transmit"},
                    "active_offboard_conditions": [],
                    "nominal_reason": "trajectory_inactive",
                    "nominal_position_error_m": 0.1,
                    "station_keeping": False,
                    "intervened": False,
                    "cbf": {"active": True, "reason": "cbf_filtered", "minimum_margin_m": 1.3},
                }
            }

        return {
            "drones": {
                drone_id: {
                    "status": {"armed": True, "nav_state": 2, "failsafe": False},
                    "local_position": {
                        "z_down_m": -10.0,
                        "vz_m_s": 0.01,
                        "vx_m_s": 0.0,
                        "vy_m_s": 0.0,
                    },
                    "failsafe_flags": {"offboard_control_signal_lost": False},
                }
                for drone_id in ("UAV-01", "UAV-02")
            },
            "tracking_pose_streams": {
                "UAV-01": entry("UAV-01", uav01_actual, (0.0, 0.0, 9.0)),
                "UAV-02": entry("UAV-02", uav02_actual, (-5.0, 2.0, 9.0)),
            },
        }

    def test_misaligned_start_aborts_and_never_requests_offboard(self) -> None:
        flight = self._flight_with_telemetry(
            self._payload((40.0, 0.0, 9.0), (-5.0, 2.0, 9.0))
        )
        with self.assertRaises(FlightAbort) as raised:
            flight.verify_initial_frame_alignment(flight.snapshot())
        self.assertIn("trajectory_initial_position_mismatch:UAV-01", str(raised.exception))
        self.assertEqual(flight.commands, [])

    def test_aligned_start_passes_the_gate(self) -> None:
        flight = self._flight_with_telemetry(
            self._payload((0.1, 0.0, 9.1), (-5.0, 2.0, 8.9))
        )
        flight.verify_initial_frame_alignment(flight.snapshot())
        validated = [
            record
            for record in flight.records
            if record["step"] == "TRAJECTORY_INITIAL_FRAME_ALIGNMENT_VALIDATED"
        ]
        self.assertEqual(len(validated), 1)

    def test_dry_run_never_actuates_even_while_recording_the_command(self) -> None:
        """--dry-run must not reach PX4 at all: no subprocess, ever."""
        flight = self._flight_with_telemetry(
            self._payload((0.0, 0.0, 9.0), (-5.0, 2.0, 9.0))
        )
        with mock.patch("two_uav_trajectory_flight.subprocess.run") as run:
            flight.commander("UAV-01", "arm")
            flight.commander("UAV-01", "mode", "offboard")
        run.assert_not_called()
        self.assertEqual(flight.commands, ["UAV-01:arm", "UAV-01:mode offboard"])


if __name__ == "__main__":
    unittest.main()
