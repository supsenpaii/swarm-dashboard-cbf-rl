from __future__ import annotations

import unittest

from offboard_authority import (
    ACTIVE_FLIGHT_AUTHORIZED_VEHICLES,
    AUTHORITY_ENV_VAR,
    FIRST_ACTIVE_FLIGHT_DRONE_ID,
    FIRST_ACTIVE_FLIGHT_SYSTEM_ID,
    LEGACY_WRITER_FLAGS,
    AuthorityDecision,
    OffboardAuthority,
    companion_active_transmit_authorized,
    legacy_writer_permitted,
    resolve_authority,
)


def env(authority: str | None = None, **flags: str) -> dict[str, str]:
    result: dict[str, str] = {}
    if authority is not None:
        result[AUTHORITY_ENV_VAR] = authority
    result.update(flags)
    return result


class SingleWriterInvariantTests(unittest.TestCase):
    """active_offboard_writer_count <= 1, enforced structurally."""

    def test_decision_cannot_be_constructed_permitting_two_writers(self) -> None:
        with self.assertRaises(ValueError):
            AuthorityDecision(
                authority=OffboardAuthority.COMPANION_SAFETY,
                reason="",
                legacy_writer_permitted=True,
                companion_writer_permitted=True,
                enabled_legacy_flags=(),
            )

    def test_every_resolvable_configuration_permits_at_most_one_writer(self) -> None:
        configurations = [
            env(),
            env("disabled"),
            env("legacy_tracking"),
            env("companion_safety"),
            env("nonsense"),
            env(""),
            env("legacy_tracking", SWARM_TRACKING_OFFBOARD_ENABLED="true"),
            env("companion_safety", SWARM_TRACKING_OFFBOARD_ENABLED="true"),
            env("companion_safety", SWARM_LEGACY_AUTOMATION_ENABLED="true"),
            env(
                "legacy_tracking",
                SWARM_TRACKING_OFFBOARD_ENABLED="true",
                SWARM_ALLOW_RAW_ATTITUDE_OFFBOARD="true",
                SWARM_LEGACY_AUTOMATION_ENABLED="true",
                SWARM_SAFE_NATIVE_FOLLOW_MODE_REQUEST_ENABLED="true",
            ),
        ]
        for configuration in configurations:
            with self.subTest(configuration=configuration):
                decision = resolve_authority(configuration)
                self.assertLessEqual(decision.active_offboard_writer_count, 1)


class FailClosedTests(unittest.TestCase):
    def test_absent_authority_fails_closed(self) -> None:
        decision = resolve_authority(env())

        self.assertIs(decision.authority, OffboardAuthority.DISABLED)
        self.assertEqual(decision.reason, "authority_not_configured")
        self.assertEqual(decision.active_offboard_writer_count, 0)

    def test_unknown_authority_fails_closed(self) -> None:
        decision = resolve_authority(env("companion-safety"))  # wrong separator

        self.assertIs(decision.authority, OffboardAuthority.DISABLED)
        self.assertTrue(decision.reason.startswith("authority_unknown"))

    def test_empty_authority_fails_closed(self) -> None:
        decision = resolve_authority(env("   "))

        self.assertIs(decision.authority, OffboardAuthority.DISABLED)
        self.assertEqual(decision.reason, "authority_not_configured")

    def test_companion_authority_with_any_legacy_flag_fails_closed(self) -> None:
        """The exact ambiguity three independent flags could previously
        express. Resolving it toward either writer would be guessing."""
        for flag in LEGACY_WRITER_FLAGS:
            with self.subTest(flag=flag):
                decision = resolve_authority(env("companion_safety", **{flag: "true"}))

                self.assertIs(decision.authority, OffboardAuthority.DISABLED)
                self.assertIn("conflicts_with_legacy_flags", decision.reason)
                self.assertIn(flag, decision.reason)
                self.assertEqual(decision.active_offboard_writer_count, 0)

    def test_legacy_authority_without_any_legacy_flag_fails_closed(self) -> None:
        decision = resolve_authority(env("legacy_tracking"))

        self.assertIs(decision.authority, OffboardAuthority.DISABLED)
        self.assertEqual(decision.reason, "legacy_authority_without_any_legacy_flag")


class AuthoritySelectionTests(unittest.TestCase):
    def test_legacy_tracking_permits_only_the_legacy_writer(self) -> None:
        decision = resolve_authority(
            env("legacy_tracking", SWARM_TRACKING_OFFBOARD_ENABLED="true")
        )

        self.assertTrue(decision.legacy_writer_permitted)
        self.assertFalse(decision.companion_writer_permitted)
        self.assertEqual(decision.active_offboard_writer_count, 1)

    def test_companion_safety_permits_only_the_companion_writer(self) -> None:
        decision = resolve_authority(env("companion_safety"))

        self.assertFalse(decision.legacy_writer_permitted)
        self.assertTrue(decision.companion_writer_permitted)
        self.assertEqual(decision.active_offboard_writer_count, 1)

    def test_legacy_writer_is_inhibited_under_companion_authority(self) -> None:
        self.assertFalse(legacy_writer_permitted(env("companion_safety")))

    def test_companion_writer_is_inhibited_under_legacy_authority(self) -> None:
        allowed, reason = companion_active_transmit_authorized(
            "UAV-01", 1, env("legacy_tracking", SWARM_TRACKING_OFFBOARD_ENABLED="true")
        )

        self.assertFalse(allowed)
        self.assertIn("legacy_tracking", reason)

    def test_disabled_inhibits_both(self) -> None:
        environment = env("disabled")

        self.assertFalse(legacy_writer_permitted(environment))
        allowed, _ = companion_active_transmit_authorized("UAV-01", 1, environment)
        self.assertFalse(allowed)


class CompanionAuthorizationTests(unittest.TestCase):
    def test_uav01_with_system_id_1_is_authorized_under_companion_authority(self) -> None:
        allowed, reason = companion_active_transmit_authorized(
            "UAV-01", 1, env("companion_safety")
        )

        self.assertTrue(allowed)
        self.assertEqual(reason, "")

    def test_uav02_with_system_id_2_is_authorized_under_companion_authority(self) -> None:
        """Added for TWO_UAV_ACTIVE_SITL_FLIGHT."""
        allowed, reason = companion_active_transmit_authorized(
            "UAV-02", 2, env("companion_safety")
        )

        self.assertTrue(allowed)
        self.assertEqual(reason, "")

    def test_uav02_with_wrong_system_id_is_refused(self) -> None:
        for system_id in (1, 99):
            with self.subTest(system_id=system_id):
                allowed, reason = companion_active_transmit_authorized(
                    "UAV-02", system_id, env("companion_safety")
                )
                self.assertFalse(allowed)
                self.assertIn("vehicle_not_authorized", reason)

    def test_unlisted_drone_id_is_never_authorized(self) -> None:
        for drone_id, system_id in (("UAV-03", 3), ("UAV-01", 3), ("UAV-02", 1)):
            with self.subTest(drone_id=drone_id, system_id=system_id):
                allowed, reason = companion_active_transmit_authorized(
                    drone_id, system_id, env("companion_safety")
                )
                self.assertFalse(allowed)
                self.assertIn("vehicle_not_authorized", reason)

    def test_broadcast_system_id_zero_is_refused_even_for_uav01(self) -> None:
        """MAVLink target_system 0 is accepted by EVERY PX4 instance."""
        allowed, reason = companion_active_transmit_authorized(
            "UAV-01", 0, env("companion_safety")
        )

        self.assertFalse(allowed)
        self.assertEqual(reason, "broadcast_system_id_forbidden")

    def test_uav01_with_wrong_system_id_is_refused(self) -> None:
        allowed, reason = companion_active_transmit_authorized(
            "UAV-01", 2, env("companion_safety")
        )

        self.assertFalse(allowed)
        self.assertIn("vehicle_not_authorized", reason)

    def test_authorized_vehicles_set_is_hardcoded(self) -> None:
        """A config value must not be able to name a different vehicle."""
        allowed, _ = companion_active_transmit_authorized(
            "UAV-03",
            3,
            env("companion_safety", SWARM_FIRST_ACTIVE_DRONE_ID="UAV-03"),
        )

        self.assertFalse(allowed)
        self.assertEqual(FIRST_ACTIVE_FLIGHT_DRONE_ID, "UAV-01")
        self.assertEqual(FIRST_ACTIVE_FLIGHT_SYSTEM_ID, 1)
        self.assertEqual(
            ACTIVE_FLIGHT_AUTHORIZED_VEHICLES,
            frozenset({("UAV-01", 1), ("UAV-02", 2)}),
        )

    def test_matches_the_readiness_module_constants(self) -> None:
        """The two modules pin the same vehicle; drift would create a gap."""
        from one_uav_active_readiness import (
            FIRST_ACTIVE_FLIGHT_DRONE_ID as readiness_drone,
            FIRST_ACTIVE_FLIGHT_SYSTEM_ID as readiness_system,
        )

        self.assertEqual(FIRST_ACTIVE_FLIGHT_DRONE_ID, readiness_drone)
        self.assertEqual(FIRST_ACTIVE_FLIGHT_SYSTEM_ID, readiness_system)


class IntendedFirstFlightConfigTests(unittest.TestCase):
    def test_repository_env_keeps_every_legacy_offboard_flag_off(self) -> None:
        from pathlib import Path

        text = Path(".env").read_text(encoding="utf-8")
        for flag in LEGACY_WRITER_FLAGS:
            with self.subTest(flag=flag):
                self.assertIn(f"{flag}=false", text)

    def test_repository_env_authority_is_a_valid_value(self) -> None:
        from pathlib import Path

        line = [
            l for l in Path(".env").read_text(encoding="utf-8").splitlines()
            if l.startswith(f"{AUTHORITY_ENV_VAR}=")
        ]
        self.assertEqual(len(line), 1)
        value = line[0].split("=", 1)[1].strip()
        self.assertIn(value, {a.value for a in OffboardAuthority})


if __name__ == "__main__":
    unittest.main()


class LegacyWriterInterlockTests(unittest.TestCase):
    """The interlock lives inside the transmit methods, so every caller --
    including ones added later -- is covered by construction."""

    def worker(self):
        calls = []

        class FakeMav:
            def set_position_target_local_ned_send(self, *args):
                calls.append(args)

            def set_attitude_target_send(self, *args):
                calls.append(args)

            def command_long_send(self, *args):
                calls.append(args)

        class FakeConnection:
            mav = FakeMav()

        class FakeWorker:
            connection = FakeConnection()
            expected_system_id = 1
            legacy_offboard_refused_count = 0

        return FakeWorker(), calls

    def test_legacy_velocity_transmit_refused_under_companion_authority(self) -> None:
        from unittest import mock

        from mavlink_manual_bridge import MavlinkWorker

        worker, calls = self.worker()
        with mock.patch.dict(
            "os.environ", {AUTHORITY_ENV_VAR: "companion_safety"}, clear=False
        ):
            for flag in LEGACY_WRITER_FLAGS:
                import os

                os.environ.pop(flag, None)
            MavlinkWorker.send_offboard_setpoint(
                worker, 0.0, 0.0, 0.0, 0.0, 0.2, -0.1, -3.0, True
            )

        self.assertEqual(calls, [])
        self.assertEqual(worker.legacy_offboard_refused_count, 1)

    def test_legacy_attitude_transmit_refused_under_companion_authority(self) -> None:
        from unittest import mock

        from mavlink_manual_bridge import MavlinkWorker

        worker, calls = self.worker()
        with mock.patch.dict(
            "os.environ", {AUTHORITY_ENV_VAR: "companion_safety"}, clear=False
        ):
            import os

            for flag in LEGACY_WRITER_FLAGS:
                os.environ.pop(flag, None)
            MavlinkWorker.send_offboard_attitude_setpoint(worker, 0.0, 0.0, 0.0, 0.5)

        self.assertEqual(calls, [])
        self.assertEqual(worker.legacy_offboard_refused_count, 1)

    def test_offboard_mode_request_refused_but_posctl_allowed(self) -> None:
        """POSCTL is the recovery path every abort rule depends on and must
        stay available under any authority."""
        from unittest import mock

        from mavlink_manual_bridge import (
            PX4_MAIN_MODE_OFFBOARD,
            PX4_MAIN_MODE_POSCTL,
            MavlinkWorker,
        )

        worker, calls = self.worker()
        worker.visual_follow_exit_ack = ""
        worker.visual_follow_exit_ack_result = None
        worker.visual_follow_exit_ack_monotonic = 0.0
        worker.last_mode_request_monotonic = 0.0
        worker.drone_id = "UAV-01"

        with mock.patch.dict(
            "os.environ", {AUTHORITY_ENV_VAR: "companion_safety"}, clear=False
        ):
            import os

            for flag in LEGACY_WRITER_FLAGS:
                os.environ.pop(flag, None)
            offboard_ok = MavlinkWorker.request_px4_main_mode(
                worker, PX4_MAIN_MODE_OFFBOARD, "OFFBOARD"
            )
            self.assertFalse(offboard_ok)
            self.assertEqual(calls, [])
            MavlinkWorker.request_px4_main_mode(
                worker, PX4_MAIN_MODE_POSCTL, "POSITION"
            )

        self.assertEqual(len(calls), 1, "POSCTL recovery must remain available")

    def test_companion_flight_gate_can_request_offboard(self) -> None:
        from mavlink_manual_bridge import PX4_MAIN_MODE_OFFBOARD, MavlinkWorker

        worker, calls = self.worker()
        worker.drone_id = "UAV-01"
        worker.last_mode_request_monotonic = 0.0
        worker.companion_holds_offboard = lambda: True

        sent = MavlinkWorker.request_px4_main_mode(
            worker,
            PX4_MAIN_MODE_OFFBOARD,
            "MISSION OFFBOARD",
            companion_authorized=True,
        )

        self.assertTrue(sent)
        self.assertEqual(len(calls), 1)
