import unittest

from swarm_state import GeodeticOrigin, SwarmStateStore, geodetic_to_enu, ned_to_enu


ORIGIN = GeodeticOrigin(10.0, 106.0, 15.0)


def payload(*, online=True):
    return {
        "online": online,
        "dashboard_received_ms": 1000,
        "global_position": {
            "latitude_deg": 10.00001,
            "longitude_deg": 106.00002,
            "altitude_msl_m": 20.0,
            "horizontal_accuracy_m": 2.0,
            "vertical_accuracy_m": 3.0,
        },
        "local_position": {"vx_m_s": 1.0, "vy_m_s": 2.0, "vz_m_s": 3.0},
    }


class SwarmStateTests(unittest.TestCase):
    def test_ned_vector_converts_to_enu(self):
        self.assertEqual(ned_to_enu((1.0, 2.0, 3.0)), (2.0, 1.0, -3.0))

    def test_geodetic_origin_maps_to_zero(self):
        self.assertEqual(geodetic_to_enu(10.0, 106.0, 15.0, ORIGIN), (0.0, 0.0, 0.0))

    def test_uses_global_position_not_vehicle_local_position_for_enu(self):
        store = SwarmStateStore(ORIGIN, max_message_age_s=0.5)
        state = store.ingest("UAV-01", payload(), received_monotonic_s=10.0)
        self.assertTrue(state.valid)
        self.assertGreater(state.position_enu_m[0], 2.0)
        self.assertGreater(state.position_enu_m[1], 1.0)
        self.assertEqual(state.velocity_enu_m_s, (2.0, 1.0, -3.0))
        self.assertEqual(state.position_covariance_m2, (4.0, 4.0, 9.0))

    def test_covariance_comes_from_gps_block(self):
        message = payload()
        message["global_position"].pop("horizontal_accuracy_m")
        message["global_position"].pop("vertical_accuracy_m")
        message["gps"] = {"fix_type": 3, "eph_m": 0.9, "epv_m": 1.78}
        state = SwarmStateStore(ORIGIN).ingest("UAV-01", message, received_monotonic_s=10.0)
        self.assertTrue(state.valid)
        self.assertEqual(state.position_covariance_m2, (0.81, 0.81, 1.78 * 1.78))

    def test_covariance_ignores_eph_in_global_position(self):
        message = payload()
        message["global_position"].pop("horizontal_accuracy_m")
        message["global_position"].pop("vertical_accuracy_m")
        message["global_position"].update({"eph_m": 5.0, "epv_m": 6.0})
        message["gps"] = {"eph_m": 2.0, "epv_m": 3.0}
        state = SwarmStateStore(ORIGIN).ingest("UAV-01", message, received_monotonic_s=10.0)
        self.assertEqual(state.position_covariance_m2, (4.0, 4.0, 9.0))

    def test_missing_or_malformed_accuracy_leaves_covariance_none(self):
        for gps in (None, {}, {"eph_m": 0.9}, {"eph_m": float("nan"), "epv_m": 1.0}, {"eph_m": -1.0, "epv_m": 1.0}, {"eph_m": "x", "epv_m": 1.0}):
            with self.subTest(gps=gps):
                message = payload()
                message["global_position"].pop("horizontal_accuracy_m")
                message["global_position"].pop("vertical_accuracy_m")
                if gps is not None:
                    message["gps"] = gps
                state = SwarmStateStore(ORIGIN).ingest("UAV-01", message, received_monotonic_s=10.0)
                self.assertTrue(state.valid)
                self.assertIsNone(state.position_covariance_m2)

    def test_stale_state_is_always_invalid(self):
        store = SwarmStateStore(ORIGIN, max_message_age_s=0.5)
        store.ingest("UAV-01", payload(), received_monotonic_s=10.0)
        state = store.snapshot(now_monotonic_s=10.51)["UAV-01"]
        self.assertFalse(state["valid"])
        self.assertEqual(state["reason"], "telemetry_stale")

    def test_missing_explicit_origin_is_invalid(self):
        store = SwarmStateStore(None)
        state = store.ingest("UAV-01", payload(), received_monotonic_s=10.0)
        self.assertFalse(state.valid)
        self.assertEqual(state.reason, "common_enu_origin_unconfigured")

    def test_offline_vehicle_is_invalid(self):
        store = SwarmStateStore(ORIGIN)
        state = store.ingest("UAV-01", payload(online=False), received_monotonic_s=10.0)
        self.assertFalse(state.valid)
        self.assertEqual(state.reason, "vehicle_offline")


if __name__ == "__main__":
    unittest.main()
