"""CBF_UNCERTAINTY_PLUMBING_VALIDATED.

Pins the ODOMETRY -> FastPoseCache -> own_swarm_state/peer_payload -> CBF path
for position covariance, and pins that the feature being off leaves the
already-flown behavior untouched. No flight behavior is enabled here:
`SWARM_CBF_UNCERTAINTY_SOURCE` defaults to off and covariance_sigma is not
touched (see CbfConfig.require_position_covariance for why enabling it is a
separate, sweep-gated decision).
"""

from __future__ import annotations

import json
import math
import unittest
from types import SimpleNamespace
from unittest import mock

from cbf_command_gate import (
    CbfCommandGate,
    CbfConfig,
    cbf_covariance_sigma,
    cbf_position_covariance_required,
    cbf_uncertainty_source_enabled,
)
from companion_safety import CompanionSafetyMonitor
from mavlink_manual_bridge import PEER_STATE_MAX_AGE_S, FastPoseCache
from peer_state import PeerStateRegistry
from swarm_state import GeodeticOrigin, ned_to_enu, ned_variance_to_enu


ORIGIN = GeodeticOrigin(47.397971057728974, 8.546163739800146, 0.0)
NOW = 100.0
# Diagonal measured on the real MAVLink link during the 2026-08-11 audit.
MEASURED_VARIANCE_NED_M2 = (0.0410, 0.0411, 0.0708)
FEATURE_ON = {"SWARM_CBF_UNCERTAINTY_SOURCE": "odometry"}
FEATURE_OFF = {"SWARM_CBF_UNCERTAINTY_SOURCE": "off"}


class FakeMessage(SimpleNamespace):
    def get_type(self) -> str:
        return str(self.message_type)


def odometry_message(
    *,
    variance=MEASURED_VARIANCE_NED_M2,
    reset_counter: int = 0,
    frame_id: int = 1,  # MAV_FRAME_LOCAL_NED
) -> FakeMessage:
    """PX4 shape: 21-entry packed upper triangle, off-diagonals NaN."""
    covariance = [float("nan")] * 21
    covariance[0], covariance[6], covariance[11] = variance
    return FakeMessage(
        message_type="ODOMETRY",
        frame_id=frame_id,
        reset_counter=reset_counter,
        pose_covariance=covariance,
    )


def primed_cache(*, variance=MEASURED_VARIANCE_NED_M2, at: float = NOW) -> FastPoseCache:
    cache = FastPoseCache()
    cache.position_ned_m = (1.0, 2.0, -10.0)
    cache.velocity_ned_m_s = (0.5, 0.0, 0.0)
    cache.global_position = (ORIGIN.latitude_deg, ORIGIN.longitude_deg, 10.0)
    cache.local_received_monotonic_s = at
    cache.global_received_monotonic_s = at
    cache.update(odometry_message(variance=variance), at)
    return cache


_SAME_AS_OWN = object()


def swarm_state(covariance, *, peer_covariance=_SAME_AS_OWN, valid: bool = True) -> dict:
    return {
        "UAV-01": {
            "position_enu_m": [0.0, 0.0, 10.0],
            "velocity_enu_m_s": [0.0, 0.0, 0.0],
            "position_covariance_m2": covariance,
            "message_age_ms": 20.0,
            "valid": valid,
        },
        "UAV-02": {
            "position_enu_m": [10.0, 0.0, 10.0],
            "velocity_enu_m_s": [0.0, 0.0, 0.0],
            "position_covariance_m2": (
                covariance if peer_covariance is _SAME_AS_OWN else peer_covariance
            ),
            "message_age_ms": 20.0,
            "valid": valid,
        },
    }


def gate(*, require: bool) -> CbfCommandGate:
    return CbfCommandGate(
        "UAV-01",
        ("UAV-02",),
        CbfConfig(require_position_covariance=require),
    )


class FeatureOffBaselineTests(unittest.TestCase):
    """1. Feature off preserves the behavior every passed flight ran on."""

    def test_flag_defaults_to_off_and_only_odometry_turns_it_on(self) -> None:
        with mock.patch.dict("os.environ", {}, clear=True):
            self.assertFalse(cbf_uncertainty_source_enabled())
        for value in ("off", "", "odometry_extra", "on", "true", "1"):
            with self.subTest(value=value):
                with mock.patch.dict(
                    "os.environ", {"SWARM_CBF_UNCERTAINTY_SOURCE": value}
                ):
                    self.assertFalse(cbf_uncertainty_source_enabled())
        with mock.patch.dict("os.environ", {"SWARM_CBF_UNCERTAINTY_SOURCE": " Odometry "}):
            self.assertTrue(cbf_uncertainty_source_enabled())

    def test_config_default_does_not_require_covariance(self) -> None:
        self.assertFalse(CbfConfig().require_position_covariance)

    def test_publish_sigma_and_require_are_independent_environment_knobs(self) -> None:
        environment = {
            "SWARM_CBF_UNCERTAINTY_SOURCE": "odometry",
            "SWARM_CBF_COVARIANCE_SIGMA": "0",
            "SWARM_CBF_REQUIRE_POSITION_COVARIANCE": "false",
        }
        with mock.patch.dict("os.environ", environment, clear=True):
            monitor = CompanionSafetyMonitor.from_environment(
                "UAV-01", ("UAV-01", "UAV-02")
            )
            self.assertTrue(cbf_uncertainty_source_enabled())
            self.assertEqual(cbf_covariance_sigma(), 0.0)
            self.assertFalse(cbf_position_covariance_required())
            self.assertEqual(monitor.gate.config.covariance_sigma, 0.0)
            self.assertFalse(monitor.gate.config.require_position_covariance)

    def test_producers_publish_no_covariance_when_off(self) -> None:
        with mock.patch.dict("os.environ", FEATURE_OFF):
            cache = primed_cache()
            self.assertEqual(cache.position_variance_ned_m2, MEASURED_VARIANCE_NED_M2)
            own = cache.own_swarm_state(NOW, ORIGIN, healthy=True)
            peer = cache.peer_payload("UAV-01", NOW, ORIGIN, 0, healthy=True)
            self.assertIsNone(own["position_covariance_m2"])
            self.assertIsNotNone(peer)
            self.assertIsNone(peer["position_covariance_m2"])

    def test_absent_covariance_still_reads_as_zero_when_off(self) -> None:
        absent = gate(require=False).filter((0.0, 0.0, 0.0), swarm_state(None))
        zeros = gate(require=False).filter(
            (0.0, 0.0, 0.0), swarm_state([0.0, 0.0, 0.0])
        )
        self.assertTrue(absent.active)
        self.assertEqual(absent.reason, "cbf_filtered")
        self.assertEqual(absent.minimum_margin_m, zeros.minimum_margin_m)
        self.assertEqual(absent.minimum_margin_m, 6.0)


class VarianceTransformTests(unittest.TestCase):
    """3. NED -> ENU for a diagonal variance."""

    def test_swaps_horizontal_axes_and_keeps_vertical_positive(self) -> None:
        self.assertEqual(ned_variance_to_enu((1.0, 2.0, 3.0)), (2.0, 1.0, 3.0))

    def test_vector_transform_would_have_produced_negative_variance(self) -> None:
        self.assertLess(ned_to_enu((1.0, 2.0, 3.0))[2], 0.0)
        self.assertGreater(ned_variance_to_enu((1.0, 2.0, 3.0))[2], 0.0)

    def test_units_stay_m2_end_to_end(self) -> None:
        with mock.patch.dict("os.environ", FEATURE_ON):
            covariance = primed_cache().position_covariance_enu_m2(NOW)
        north, east, down = MEASURED_VARIANCE_NED_M2
        self.assertEqual(covariance, (east, north, down))
        # Cross-check against the audit's independent ESTIMATOR_STATUS reading:
        # sqrt of the horizontal variances is the 0.29 m sigma it reported.
        self.assertAlmostEqual(
            math.sqrt(covariance[0] + covariance[1]), 0.2865, places=3
        )


class EndToEndPropagationTests(unittest.TestCase):
    """2 + 4. Real ODOMETRY value reaches own state, the wire, and back."""

    def test_own_swarm_state_carries_enu_variance(self) -> None:
        with mock.patch.dict("os.environ", FEATURE_ON):
            own = primed_cache().own_swarm_state(NOW, ORIGIN, healthy=True)
        north, east, down = MEASURED_VARIANCE_NED_M2
        self.assertTrue(own["valid"])
        self.assertEqual(own["position_covariance_m2"], (east, north, down))

    def test_peer_state_round_trip_over_the_wire(self) -> None:
        with mock.patch.dict("os.environ", FEATURE_ON):
            payload = primed_cache().peer_payload("UAV-02", NOW, ORIGIN, 7, healthy=True)
        north, east, down = MEASURED_VARIANCE_NED_M2
        self.assertEqual(payload["position_covariance_m2"], [east, north, down])

        registry = PeerStateRegistry(max_age_s=0.5)
        self.assertTrue(
            registry.ingest(json.loads(json.dumps(payload)), received_monotonic_s=NOW)
        )
        peer = registry.snapshot(NOW)["peers"]["UAV-02"]
        self.assertTrue(peer["valid"])
        self.assertEqual(peer["position_covariance_m2"], [east, north, down])

    def test_gate_consumes_the_round_tripped_covariance(self) -> None:
        with mock.patch.dict("os.environ", FEATURE_ON):
            payload = primed_cache().peer_payload("UAV-02", NOW, ORIGIN, 7, healthy=True)
        registry = PeerStateRegistry(max_age_s=0.5)
        registry.ingest(json.loads(json.dumps(payload)), received_monotonic_s=NOW)
        state = swarm_state(None)
        state["UAV-01"]["position_covariance_m2"] = list(
            payload["position_covariance_m2"]
        )
        state["UAV-02"] = {
            **registry.snapshot(NOW)["peers"]["UAV-02"],
            "position_enu_m": [10.0, 0.0, 10.0],
            "velocity_enu_m_s": [0.0, 0.0, 0.0],
        }
        command = gate(require=True).filter((0.0, 0.0, 0.0), state)
        self.assertTrue(command.active)
        # 10 m apart, d_min 4 m, sigma 2.0 * sqrt(2 * 0.1529 m2) = 1.106 m.
        self.assertAlmostEqual(command.minimum_margin_m, 6.0 - 1.106, places=3)


class MarginMonotonicityTests(unittest.TestCase):
    """5. More uncertainty must never mean more reported margin."""

    def test_margin_decreases_monotonically_with_covariance(self) -> None:
        margins = []
        for scale in (0.0, 0.05, 0.2, 1.0, 4.0):
            command = gate(require=True).filter(
                (0.0, 0.0, 0.0), swarm_state([scale, scale, scale])
            )
            self.assertIsNotNone(command.minimum_margin_m)
            margins.append(command.minimum_margin_m)
        self.assertEqual(margins, sorted(margins, reverse=True))
        self.assertEqual(margins[0], 6.0)
        self.assertLess(margins[-1], margins[0])

    def test_covariance_sigma_is_untouched_by_this_milestone(self) -> None:
        self.assertEqual(CbfConfig().covariance_sigma, 2.0)
        self.assertEqual(CbfConfig().minimum_separation_m, 4.0)
        self.assertEqual(CbfConfig().command_latency_s, 0.10)
        self.assertEqual(CbfConfig().design_margin_buffer_m, 0.0)


class FailClosedTests(unittest.TestCase):
    """6. Enabled + unusable covariance must hold, never substitute zero."""

    def test_missing_covariance_holds_when_required(self) -> None:
        command = gate(require=True).filter((0.0, 0.0, 0.0), swarm_state(None))
        self.assertFalse(command.active)
        self.assertEqual(command.reason, "nominal_or_self_state_invalid")

    def test_missing_peer_covariance_holds_when_required(self) -> None:
        state = swarm_state([0.1, 0.1, 0.1], peer_covariance=None)
        command = gate(require=True).filter((0.0, 0.0, 0.0), state)
        self.assertFalse(command.active)
        self.assertEqual(command.reason, "peer_state_invalid")

    def test_malformed_covariance_holds_whether_required_or_not(self) -> None:
        malformed = (
            [float("nan"), 0.1, 0.1],
            [float("inf"), 0.1, 0.1],
            [-0.1, 0.1, 0.1],
            [0.1, 0.1],
        )
        for covariance in malformed:
            for require in (True, False):
                with self.subTest(covariance=covariance, require=require):
                    command = gate(require=require).filter(
                        (0.0, 0.0, 0.0), swarm_state(covariance)
                    )
                    self.assertFalse(command.active)
                    self.assertEqual(command.reason, "nominal_or_self_state_invalid")

    def test_malformed_odometry_is_never_cached(self) -> None:
        for variance in (
            (float("nan"), 0.1, 0.1),
            (0.1, float("inf"), 0.1),
            (-0.1, 0.1, 0.1),
        ):
            with self.subTest(variance=variance):
                cache = primed_cache(variance=variance)
                self.assertIsNone(cache.position_variance_ned_m2)

    def test_wrong_frame_odometry_is_ignored(self) -> None:
        cache = FastPoseCache()
        for _ in range(2):
            cache.update(odometry_message(frame_id=0), NOW)  # MAV_FRAME_GLOBAL
        self.assertIsNone(cache.position_variance_ned_m2)

    def test_stale_covariance_is_withheld_and_then_holds(self) -> None:
        with mock.patch.dict("os.environ", FEATURE_ON):
            cache = primed_cache()
            stale_at = NOW + PEER_STATE_MAX_AGE_S + 0.01
            self.assertIsNotNone(cache.position_covariance_enu_m2(NOW))
            self.assertIsNone(cache.position_covariance_enu_m2(stale_at))
            # Position/velocity are kept fresh, so only covariance age is
            # under test here.
            cache.local_received_monotonic_s = stale_at
            cache.global_received_monotonic_s = stale_at
            own = cache.own_swarm_state(stale_at, ORIGIN, healthy=True)
        self.assertTrue(own["valid"])
        self.assertIsNone(own["position_covariance_m2"])
        command = gate(require=True).filter(
            (0.0, 0.0, 0.0), swarm_state(own["position_covariance_m2"])
        )
        self.assertFalse(command.active)

    def test_never_streamed_odometry_holds_when_required(self) -> None:
        cache = FastPoseCache()
        cache.velocity_ned_m_s = (0.0, 0.0, 0.0)
        cache.global_position = (ORIGIN.latitude_deg, ORIGIN.longitude_deg, 10.0)
        cache.local_received_monotonic_s = NOW
        cache.global_received_monotonic_s = NOW
        with mock.patch.dict("os.environ", FEATURE_ON):
            own = cache.own_swarm_state(NOW, ORIGIN, healthy=True)
        self.assertIsNone(own["position_covariance_m2"])


class ResetCounterTests(unittest.TestCase):
    """7. An EKF reset replaces, never carries, the previous covariance."""

    def test_counter_change_accepts_the_reset_samples_valid_variance(self) -> None:
        cache = primed_cache()
        self.assertEqual(cache.position_variance_ned_m2, MEASURED_VARIANCE_NED_M2)

        cache.update(odometry_message(variance=(9.0, 9.0, 9.0), reset_counter=12), NOW)
        self.assertEqual(cache.position_variance_ned_m2, (9.0, 9.0, 9.0))
        self.assertEqual(cache.odometry_reset_counter, 12)
        with mock.patch.dict("os.environ", FEATURE_ON):
            self.assertEqual(cache.position_covariance_enu_m2(NOW), (9.0, 9.0, 9.0))

    def test_counter_change_with_invalid_variance_drops_the_old_cache(self) -> None:
        cache = primed_cache()
        cache.update(
            odometry_message(variance=(float("nan"), 9.0, 9.0), reset_counter=12),
            NOW,
        )
        self.assertIsNone(cache.position_variance_ned_m2)
        self.assertEqual(cache.odometry_reset_counter, 12)

    def test_first_valid_sample_establishes_counter_and_covariance(self) -> None:
        cache = FastPoseCache()
        cache.update(odometry_message(reset_counter=3), NOW)
        self.assertEqual(cache.position_variance_ned_m2, MEASURED_VARIANCE_NED_M2)
        self.assertEqual(cache.odometry_reset_counter, 3)


if __name__ == "__main__":
    unittest.main()
