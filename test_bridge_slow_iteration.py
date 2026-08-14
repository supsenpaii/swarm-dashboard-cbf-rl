"""Pins the slow-iteration probe added to the companion bridge loop.

Five real flights on 2026-08-11 aborted on `setpoint_stream_gap` because the
bridge's 20 Hz loop stalled for 0.25-0.57 s. The traces those flights left
could bound the stall but not locate it, so this probe exists to answer
"where did the time go" on the next flight instead of the one after that. If
its arithmetic is wrong it will point at the wrong section, and the next
armed flight is spent chasing that instead -- which is the whole cost this
probe was added to avoid.
"""

from __future__ import annotations

import gc
import logging
import types
import unittest

from mavlink_manual_bridge import GcPauseProbe, MavlinkWorker


class SlowIterationReportTests(unittest.TestCase):
    def report(
        self,
        started: float,
        marks: list[tuple[str, float]],
        ended: float | None = None,
    ) -> str:
        """Calls the real method against a stand-in holding only what it
        reads, rather than building a MavlinkWorker (which opens sockets)."""
        worker = types.SimpleNamespace(drone_id="UAV-01")
        with self.assertLogs("mavlink-manual-bridge", level=logging.WARNING) as caught:
            MavlinkWorker.report_slow_iteration(
                worker, started, marks[-1][1] if ended is None else ended, marks
            )
        return caught.output[0]

    def test_each_section_is_measured_from_the_previous_mark(self) -> None:
        """The marks are absolute timestamps; the report must show per-section
        durations. Printing the raw deltas from loop start would make every
        section after a slow one look slow too."""
        line = self.report(
            100.0,
            [
                ("mavlink_drain", 100.01),
                ("companion_safety", 100.03),
                ("mqtt_publish", 100.33),
                ("peer_state_publish", 100.34),
            ],
        )
        self.assertIn("mavlink_drain=10ms", line)
        self.assertIn("companion_safety=20ms", line)
        self.assertIn("mqtt_publish=300ms", line)
        self.assertIn("peer_state_publish=10ms", line)
        self.assertIn("340ms", line)

    def test_time_outside_the_body_is_charged_to_sleep_and_scheduler(self) -> None:
        """The stall that matters may be in `stop_event.wait` -- the process
        descheduled, or the whole machine paused. The body-only view the
        first version of this probe had would have shown a fast loop and
        explained nothing, so the unaccounted remainder is named."""
        line = self.report(
            100.0, [("mavlink_drain", 100.01), ("control_output", 100.02)], ended=100.5
        )
        self.assertIn("sleep_and_scheduler=480ms", line)
        self.assertIn("500ms", line)

    def test_gc_pauses_inside_the_iteration_are_named_separately(self) -> None:
        """A GC pause is charged to whichever section was unlucky enough to be
        running, so it has to be reported on its own or it reads as that
        section being slow."""
        from mavlink_manual_bridge import GC_PAUSE_PROBE

        GC_PAUSE_PROBE.pauses.append((100.2, 0.3, 2))
        self.addCleanup(GC_PAUSE_PROBE.pauses.clear)
        line = self.report(100.0, [("mavlink_drain", 100.4)])
        self.assertIn("gen2:300ms", line)

    def test_gc_pauses_from_earlier_iterations_are_excluded(self) -> None:
        from mavlink_manual_bridge import GC_PAUSE_PROBE

        GC_PAUSE_PROBE.pauses.append((99.0, 0.3, 2))
        self.addCleanup(GC_PAUSE_PROBE.pauses.clear)
        self.assertIn("gc=none", self.report(100.0, [("mavlink_drain", 100.4)]))


class GcPauseProbeTests(unittest.TestCase):
    def test_records_a_real_collection(self) -> None:
        probe = GcPauseProbe()
        gc.callbacks.append(probe)
        try:
            gc.collect()
        finally:
            gc.callbacks.remove(probe)
        self.assertTrue(probe.pauses)
        _end, seconds, generation = probe.pauses[-1]
        self.assertGreaterEqual(seconds, 0.0)
        self.assertEqual(generation, 2)

    def test_only_the_last_few_pauses_are_kept(self) -> None:
        """Unbounded growth in a probe attached to a process that runs for
        hours would be a leak introduced by the diagnostic itself."""
        probe = GcPauseProbe(keep=3)
        for _ in range(10):
            probe("start", {})
            probe("stop", {"generation": 0})
        self.assertEqual(len(probe.pauses), 3)


if __name__ == "__main__":
    unittest.main()
