from __future__ import annotations

import os
import tempfile
import unittest
from unittest import mock

import mavlink_manual_bridge as bridge


class PeerStateConfigTests(unittest.TestCase):
    """Per-drone peer sockets, needed because SITL puts every companion on one host."""

    def test_absent_per_drone_variables_fall_back_to_shared_socket(self) -> None:
        with mock.patch.dict("os.environ", {}, clear=True):
            host, port, endpoints, error = bridge.peer_state_config_for("UAV-01")

        self.assertEqual(host, bridge.PEER_STATE_BIND_HOST)
        self.assertEqual(port, bridge.PEER_STATE_BIND_PORT)
        self.assertEqual(endpoints, bridge.PEER_STATE_ENDPOINTS)
        self.assertEqual(error, bridge.PEER_STATE_CONFIG_ERROR)

    def test_each_drone_binds_its_own_port_and_targets_the_other(self) -> None:
        environment = {
            "SWARM_PEER_STATE_BIND_PORT_UAV_01": "14670",
            "SWARM_PEER_STATE_ENDPOINTS_UAV_01": "127.0.0.1:14671",
            "SWARM_PEER_STATE_BIND_PORT_UAV_02": "14671",
            "SWARM_PEER_STATE_ENDPOINTS_UAV_02": "127.0.0.1:14670",
        }
        with mock.patch.dict("os.environ", environment, clear=True):
            first = bridge.peer_state_config_for("UAV-01")
            second = bridge.peer_state_config_for("UAV-02")

        self.assertEqual((first[1], first[2]), (14670, (("127.0.0.1", 14671),)))
        self.assertEqual((second[1], second[2]), (14671, (("127.0.0.1", 14670),)))
        self.assertEqual(first[3], "")
        self.assertEqual(second[3], "")
        # Neither drone may publish to its own bind port, or its registry would
        # ingest itself and the CBF peer set would contain the drone's own state.
        self.assertNotIn(("127.0.0.1", first[1]), first[2])
        self.assertNotIn(("127.0.0.1", second[1]), second[2])

    def test_drone_id_maps_to_a_shouty_underscore_suffix(self) -> None:
        self.assertEqual(bridge.peer_state_env_suffix("UAV-01"), "UAV_01")
        self.assertEqual(bridge.peer_state_env_suffix(" uav-02 "), "UAV_02")

    def test_invalid_per_drone_port_disables_that_drone_only(self) -> None:
        environment = {
            "SWARM_PEER_STATE_BIND_PORT_UAV_01": "99999",
            "SWARM_PEER_STATE_ENDPOINTS_UAV_01": "127.0.0.1:14671",
            "SWARM_PEER_STATE_BIND_PORT_UAV_02": "14671",
            "SWARM_PEER_STATE_ENDPOINTS_UAV_02": "127.0.0.1:14670",
        }
        with mock.patch.dict("os.environ", environment, clear=True):
            broken = bridge.peer_state_config_for("UAV-01")
            healthy = bridge.peer_state_config_for("UAV-02")

        self.assertTrue(broken[3])
        self.assertEqual(broken[2], ())
        self.assertEqual(healthy[3], "")
        self.assertEqual(healthy[1], 14671)


class PeerPayloadFreshnessTests(unittest.TestCase):
    """The worker timestamps its loop before refreshing the pose cache, so a
    just-received sample is fractionally newer than the query time."""

    def cache(self) -> bridge.FastPoseCache:
        cache = bridge.FastPoseCache()
        cache.position_ned_m = (1.0, 2.0, -10.0)
        cache.velocity_ned_m_s = (0.5, 0.0, 0.0)
        cache.global_position = (47.397971057728974, 8.546163739800146, 10.0)
        return cache

    def origin(self):
        from swarm_state import GeodeticOrigin

        return GeodeticOrigin(47.397971057728974, 8.546163739800146, 0.0)

    def test_sample_newer_than_query_time_still_publishes(self) -> None:
        cache = self.cache()
        cache.local_received_monotonic_s = 100.004
        cache.global_received_monotonic_s = 100.003

        payload = cache.peer_payload("UAV-01", 100.0, self.origin(), 0, healthy=True)

        self.assertIsNotNone(payload)
        self.assertEqual(payload["drone_id"], "UAV-01")
        self.assertEqual(payload["frame"], "ENU")

    def test_genuinely_stale_sample_is_still_withheld(self) -> None:
        cache = self.cache()
        cache.local_received_monotonic_s = 100.0
        cache.global_received_monotonic_s = 100.0

        stale_query = 100.0 + bridge.PEER_STATE_MAX_AGE_S + 0.1

        self.assertIsNone(
            cache.peer_payload("UAV-01", stale_query, self.origin(), 0, healthy=True)
        )

    def test_own_swarm_state_matches_that_freshness_rule(self) -> None:
        cache = self.cache()
        cache.local_received_monotonic_s = 100.004
        cache.global_received_monotonic_s = 100.003

        fresh = cache.own_swarm_state(100.0, self.origin(), healthy=True)
        stale = cache.own_swarm_state(
            100.0 + bridge.PEER_STATE_MAX_AGE_S + 0.1, self.origin(), healthy=True
        )

        self.assertTrue(fresh["valid"])
        self.assertEqual(fresh["reason"], "ok")
        self.assertEqual(fresh["message_age_ms"], 0.0)
        self.assertFalse(stale["valid"])
        self.assertEqual(stale["reason"], "telemetry_stale")

    def test_unhealthy_vehicle_is_invalid_even_when_fresh(self) -> None:
        cache = self.cache()
        cache.local_received_monotonic_s = 100.0
        cache.global_received_monotonic_s = 100.0

        result = cache.own_swarm_state(100.0, self.origin(), healthy=False)

        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "vehicle_unhealthy")



class HeartbeatLivenessTests(unittest.TestCase):
    """Liveness must be budgeted against PX4's 1 Hz HEARTBEAT, not against the
    position-sample freshness limit."""

    def test_default_peer_state_age_is_100_ms(self) -> None:
        self.assertEqual(bridge.PEER_STATE_MAX_AGE_S, 0.1)

    def test_heartbeat_timeout_exceeds_the_heartbeat_period(self) -> None:
        self.assertGreaterEqual(bridge.VEHICLE_HEARTBEAT_TIMEOUT_S, 2.0)
        self.assertGreater(
            bridge.VEHICLE_HEARTBEAT_TIMEOUT_S, bridge.PEER_STATE_MAX_AGE_S
        )

    def test_default_survives_a_single_missed_heartbeat(self) -> None:
        px4_heartbeat_period_s = 1.0

        self.assertGreater(
            bridge.VEHICLE_HEARTBEAT_TIMEOUT_S, 2.0 * px4_heartbeat_period_s
        )

if __name__ == "__main__":
    unittest.main()


class CompanionTelemetryBlockHookTests(unittest.TestCase):
    """SITL/test-only hook that freezes a companion's own FastPoseCache
    ingestion, distinct from main.py's server-side ingestion block."""

    def test_disabled_by_default(self) -> None:
        with mock.patch.object(bridge, "SWARM_TEST_COMPANION_TELEMETRY_BLOCK_FILE", ""):
            self.assertFalse(bridge.companion_telemetry_ingest_blocked("UAV-01"))

    def test_blocks_only_the_named_drone(self) -> None:
        fd, path = tempfile.mkstemp()
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write("UAV-01")
        try:
            with mock.patch.object(
                bridge, "SWARM_TEST_COMPANION_TELEMETRY_BLOCK_FILE", path
            ):
                self.assertTrue(bridge.companion_telemetry_ingest_blocked("UAV-01"))
                self.assertFalse(bridge.companion_telemetry_ingest_blocked("UAV-02"))
        finally:
            os.remove(path)

    def test_missing_file_fails_open_to_not_blocked(self) -> None:
        with mock.patch.object(
            bridge,
            "SWARM_TEST_COMPANION_TELEMETRY_BLOCK_FILE",
            "/nonexistent/path/for/test.txt",
        ):
            self.assertFalse(bridge.companion_telemetry_ingest_blocked("UAV-01"))
