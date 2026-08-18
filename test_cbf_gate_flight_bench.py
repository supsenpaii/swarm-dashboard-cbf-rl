"""The bench's own credential: does it reproduce the flight it replays?

A tool that re-runs the gate against recorded states is only worth trusting to
the extent its rebuilt inputs match the ones the aircraft's gate actually had.
So the number that matters here is not the margin -- it is the error between
the bench's margin and the one the flight wrote down on the same frame.
"""

from __future__ import annotations

import os
import unittest
from pathlib import Path

from cbf_gate_flight_bench import bench, load_last_run, paired_frames
from sparrow_corridor_replay import load_env_profile

ROOT = Path(__file__).parent
LOG = ROOT / "artifacts" / "companion_safety_sparrow_active_20ms_corridor.jsonl"
PROFILE = "sparrow_active_20ms_corridor.env"


@unittest.skipUnless(LOG.exists(), f"{LOG.name} not present")
class GateFlightBenchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        saved = dict(os.environ)
        try:
            load_env_profile(ROOT / PROFILE)
            cls.report = bench(LOG)
        finally:
            os.environ.clear()
            os.environ.update(saved)

    def test_it_reproduces_the_flight_it_replays(self) -> None:
        for drone_id in ("UAV-01", "UAV-02"):
            error = self.report[drone_id]["reproduction_error_m"]
            with self.subTest(drone_id):
                # Measured 2026-08-18: 8 mm and 1 mm. The p95 is a metre
                # because the two vehicles log independently and a frame pair
                # can be up to 50 ms apart, which at 40 m/s of closing speed
                # is 2 m of geometry -- not gate disagreement.
                self.assertLess(error["median"], 0.05)
                self.assertLess(error["p95"], 2.0)

    def test_it_finds_the_breach_the_flight_report_missed(self) -> None:
        """The whole point.

        This log is three runs of a rung that signed FLIGHT_PASS. The driver
        polled at 1.9 Hz and reported a comfortable margin; the 20 Hz record
        says the pair went through their separation requirement.
        """
        for drone_id in ("UAV-01", "UAV-02"):
            with self.subTest(drone_id):
                self.assertLess(self.report[drone_id]["bench_minimum_margin_m"], 0.0)
                self.assertGreater(self.report[drone_id]["bench_breach_frames"], 0)

    def test_it_covers_the_whole_hold(self) -> None:
        self.assertGreater(self.report["frames"], 2000)
        self.assertGreater(self.report["duration_s"], 100.0)

    def test_only_the_last_run_is_read(self) -> None:
        """One log holds every run ever flown against a profile.

        Averaging them together would blend a fixed rung with the broken one
        that preceded it, so the split is load-bearing rather than tidiness.
        """
        run = load_last_run(LOG)
        times = [row["evaluated_monotonic_s"] for row in run]

        self.assertTrue(all(b >= a for a, b in zip(times, times[1:])))
        self.assertLess(max(times) - min(times), 600.0)
        self.assertGreater(len(paired_frames(run)), 1000)


if __name__ == "__main__":
    unittest.main()
