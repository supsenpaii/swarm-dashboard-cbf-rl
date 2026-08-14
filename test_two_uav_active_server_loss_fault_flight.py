"""Pins the outage-window guard in two_uav_active_server_loss_fault_flight.

That driver deliberately stops the backend it normally polls, so for the
duration of the fault its safety guard reads the companion's local JSONL
trace instead of `GET /api/drones`. That swap is the one genuinely new piece
of safety logic in the driver: everything else is `two_uav_active_flight.py`'s
already-reviewed sequence. If `guard_from_trace` misses a violation, the
flight has no guard at all during precisely the window the fault is active.

Every payload here is built by the REAL `CompanionSafetyStatus` serializer
via `test_offboard_abort_conditions.real_status_payload`, reused rather than
re-written for the reason that helper's own docstring gives: a hand-written
sample once agreed with a wrong key (`payload["command"]` vs `payload["cbf"]`)
and shipped a module that reported a phantom abort on every live frame. The
trace rows this guard parses are the same serializer's output, so they are
pinned the same way.
"""

from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import two_uav_active_server_loss_fault_flight as module
from one_uav_active_readiness import ABORT_MATRIX, abort_rule_for
from companion_server_loss_fault import read_trace, summarise
from test_offboard_abort_conditions import real_status_payload
from two_uav_active_server_loss_fault_flight import (
    TRACE_SILENCE_ABORT_S,
    Flight,
    FlightAbort,
)


def trace_row(
    *,
    margin_m: float | None = 1.0,
    latched_abort: str | None = None,
    conditions: tuple[str, ...] = (),
    cbf_reason: str = "cbf_filtered",
    peer_ids_used: bool = True,
) -> dict:
    """One line of the companion trace, shaped exactly as the bridge writes
    it: the real status payload plus the fields
    `MavlinkWorker.evaluate_companion_safety` adds before appending."""
    payload = real_status_payload(cbf_reason=cbf_reason)
    payload["cbf"]["minimum_margin_m"] = margin_m
    payload["evaluated_monotonic_s"] = 1000.0
    payload["active_offboard_conditions"] = list(conditions)
    payload["active_offboard_sender"] = {"latched_abort": latched_abort}
    if not peer_ids_used:
        payload["peer_ids_used"] = []
    return payload


class GuardFromTraceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.flight = Flight(
            baseline_s=1.0, outage_s=1.0, recovery_s=1.0, dry_run=True
        )

    def test_healthy_rows_do_not_abort(self) -> None:
        self.flight.guard_from_trace([trace_row(), trace_row(margin_m=0.2)])

    def test_negative_margin_aborts(self) -> None:
        with self.assertRaises(FlightAbort) as caught:
            self.flight.guard_from_trace([trace_row(), trace_row(margin_m=-0.01)])
        self.assertIn("cbf_separation_violated_during_outage", str(caught.exception))

    def test_latched_sender_abort_aborts(self) -> None:
        with self.assertRaises(FlightAbort) as caught:
            self.flight.guard_from_trace([trace_row(latched_abort="nan_or_inf_command")])
        self.assertIn("sender_latched_during_outage", str(caught.exception))

    def test_manual_intervention_condition_aborts(self) -> None:
        """`vertical_velocity_sign_mismatch` carries
        manual_intervention_required=True in ABORT_MATRIX -- read from the
        matrix here rather than hardcoded, so this stays true if the matrix
        is re-ordered."""
        condition = next(
            rule.condition for rule in ABORT_MATRIX if rule.manual_intervention_required
        )
        with self.assertRaises(FlightAbort) as caught:
            self.flight.guard_from_trace([trace_row(conditions=(condition,))])
        self.assertIn("manual_intervention_condition_during_outage", str(caught.exception))

    def test_recoverable_condition_does_not_abort(self) -> None:
        """The deliberate half of the design, not an oversight.
        `cbf_invalid_or_infeasible` means the vehicle holds zero velocity --
        ABORT_MATRIX gives it manual_intervention_required=False and
        recovery_allowed=True. Aborting a real armed flight on a condition
        whose own defined response is "hold still and recover" would be
        strictly less safe than letting it recover.
        """
        rule = abort_rule_for("cbf_invalid_or_infeasible")
        assert rule is not None
        self.assertFalse(rule.manual_intervention_required)
        self.assertTrue(rule.recovery_allowed)
        self.flight.guard_from_trace(
            [trace_row(conditions=("cbf_invalid_or_infeasible",), cbf_reason="peer_state_invalid")]
        )

    def test_unknown_condition_string_does_not_crash(self) -> None:
        """`abort_rule_for` returns None for anything not in the matrix. A
        guard that raised TypeError there would take down the flight from
        inside the guard itself, during the outage, which is the worst place
        in the whole sequence to hit an unhandled exception."""
        self.flight.guard_from_trace([trace_row(conditions=("not_a_real_condition",))])


class IncrementalTraceReadTests(unittest.TestCase):
    """`read_trace` must return only what is new and must never re-read the
    whole file. The first ACTIVE_FLIGHT_FAULT_INJECTION flight aborted
    because it did: re-reading a ~45,000-row trace twice a second stalled the
    bridge's own 20 Hz loop for 0.581 s, which tripped `setpoint_stream_gap`.
    A guard that perturbs the flight it is guarding is worse than no guard.
    """

    def setUp(self) -> None:
        handle = tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False)
        handle.close()
        self.path = Path(handle.name)
        self.addCleanup(os.unlink, handle.name)

    def append(self, *rows: dict) -> None:
        with self.path.open("a", encoding="utf-8") as handle:
            for row in rows:
                handle.write(json.dumps(row) + "\n")

    def test_returns_only_new_rows_and_reads_only_new_bytes(self) -> None:
        self.append(trace_row(margin_m=1.0), trace_row(margin_m=2.0))
        first, cursor = read_trace(self.path, 0)
        self.assertEqual(len(first), 2)
        self.assertEqual(cursor, self.path.stat().st_size)

        self.append(trace_row(margin_m=3.0))
        second, cursor = read_trace(self.path, cursor)
        self.assertEqual([r["cbf"]["minimum_margin_m"] for r in second], [3.0])
        self.assertEqual(cursor, self.path.stat().st_size)

        third, cursor = read_trace(self.path, cursor)
        self.assertEqual(third, [])

    def test_a_half_written_row_is_left_for_the_next_poll(self) -> None:
        """The bridge appends while this reads, so the tail can be torn. The
        partial row must be withheld, not silently dropped -- during the
        outage every row is a safety observation."""
        self.append(trace_row(margin_m=1.0))
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write('{"drone_id": "UAV-01", "cbf": {"minimum_ma')
        rows, cursor = read_trace(self.path, 0)
        self.assertEqual(len(rows), 1)

        with self.path.open("a", encoding="utf-8") as handle:
            handle.write('rgin_m": 4.0}}\n')
        rows, cursor = read_trace(self.path, cursor)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["cbf"]["minimum_margin_m"], 4.0)

    def test_missing_file_returns_the_cursor_unchanged(self) -> None:
        rows, cursor = read_trace(Path("/nonexistent/trace.jsonl"), 17)
        self.assertEqual(rows, [])
        self.assertEqual(cursor, 17)


class TraceSilenceTests(unittest.TestCase):
    """A trace that stops arriving is the one failure `guard_from_trace`
    cannot catch on its own: zero rows satisfy every check in it. During the
    outage that trace is the only thing watching two armed vehicles, so
    silence has to abort rather than read as all-clear."""

    def _flight(self, trace_path: str) -> Flight:
        flight = Flight(baseline_s=1.0, outage_s=1.0, recovery_s=1.0, dry_run=False)
        flight.trace_path = Path(trace_path)
        return flight

    def test_silent_trace_aborts(self) -> None:
        flight = self._flight("/nonexistent/companion/trace.jsonl")
        with mock.patch.object(module, "TRACE_SILENCE_ABORT_S", 0.0):
            with self.assertRaises(FlightAbort) as caught:
                flight._hold_via_trace(1.0)
        self.assertIn("companion_trace_silent_during_outage", str(caught.exception))

    def test_flowing_trace_does_not_abort(self) -> None:
        with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as handle:
            handle.write(json.dumps(trace_row()) + "\n")
            path = handle.name
        self.addCleanup(os.unlink, path)
        flight = self._flight(path)
        with mock.patch.object(module, "TRACE_SILENCE_ABORT_S", 0.0):
            rows = flight._hold_via_trace(0.1)
        self.assertEqual(len(rows), 1)


class OutageEvidenceTests(unittest.TestCase):
    """`fault_injection_evidence` decides PASS/FAIL from
    `companion_server_loss_fault.summarise`, which was written against the
    shadow test's trace. Pins that it still reads the real payload's keys --
    a silent KeyError-to-zero here would turn a failed fault into a pass."""

    def test_summarise_reads_real_trace_rows(self) -> None:
        summary = summarise([trace_row(), trace_row()])["UAV-01"]
        self.assertEqual(summary["samples"], 2)
        self.assertEqual(summary["filtered_fraction"], 1.0)
        self.assertEqual(summary["peers_used_fraction"], 1.0)
        self.assertEqual(summary["minimum_margin_m"], 1.0)

    def test_evidence_fails_when_companion_stopped_filtering(self) -> None:
        flight = Flight(baseline_s=1.0, outage_s=1.0, recovery_s=1.0, dry_run=False)
        flight.outage_rows = [
            trace_row(cbf_reason="peer_state_invalid", peer_ids_used=False)
        ] * 4
        evidence = flight.fault_injection_evidence()
        self.assertFalse(evidence["checks"]["companion_kept_filtering_during_outage"])
        self.assertFalse(evidence["checks"]["companion_kept_peers_during_outage"])
        self.assertEqual(evidence["conclusion"], "SERVER_LOSS_UNDER_REAL_ACTUATION_FAIL")

    def test_dry_run_never_claims_a_fault_injection_pass(self) -> None:
        """A dry run stops nothing and observes nothing, so it must not
        report the milestone's headline verdict."""
        evidence = Flight(
            baseline_s=1.0, outage_s=1.0, recovery_s=1.0, dry_run=True
        ).fault_injection_evidence()
        self.assertEqual(evidence["conclusion"], "DRY_RUN_NOT_EVALUATED")
        self.assertFalse(evidence["conclusion"].endswith("_PASS"))

    def test_evidence_fails_when_no_outage_rows_observed(self) -> None:
        """An empty trace means the driver watched nothing for the whole
        outage -- it must not read as a clean run."""
        flight = Flight(baseline_s=1.0, outage_s=1.0, recovery_s=1.0, dry_run=False)
        evidence = flight.fault_injection_evidence()
        self.assertFalse(evidence["checks"]["outage_rows_observed"])
        self.assertEqual(evidence["conclusion"], "SERVER_LOSS_UNDER_REAL_ACTUATION_FAIL")


if __name__ == "__main__":
    unittest.main()
