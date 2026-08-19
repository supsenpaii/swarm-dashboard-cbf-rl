"""The bench's own credential: does it reproduce the flight it replays?

A tool that re-runs the gate against recorded states is only worth trusting to
the extent its rebuilt inputs match the ones the aircraft's gate actually had.
So the number that matters here is not the margin -- it is the error between
the bench's margin and the one the flight wrote down on the same frame.

The fixture is a frozen copy of one historical run rather than the live
companion-safety log these tests used to read. That log is appended to by every
flight against the profile, so "the last run in it" changed the first time
somebody flew the rung again -- and after a stack reap the tail is idle
station-keeping with no paired frames at all, which failed the whole class in
setUpClass. A regression about a specific breach has to own the bytes it
asserts on.
"""

from __future__ import annotations

import gzip
import os
import shutil
import tempfile
import unittest
from pathlib import Path

from cbf_gate_flight_bench import bench, load_last_run, paired_frames
from sparrow_corridor_replay import load_env_profile

ROOT = Path(__file__).parent
FIXTURE = ROOT / "testdata" / "sparrow_20ms_breach_run.jsonl.gz"
PROFILE = "sparrow_active_20ms_corridor.env"


class GateFlightBenchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        # bench()/load_last_run() take a path and read it as text; decompress
        # once here rather than teaching them about compression they would
        # never meet on a real flight.
        handle, path = tempfile.mkstemp(suffix=".jsonl")
        os.close(handle)
        cls._log = Path(path)
        with gzip.open(FIXTURE, "rb") as source, cls._log.open("wb") as target:
            shutil.copyfileobj(source, target)

        saved = dict(os.environ)
        try:
            load_env_profile(ROOT / PROFILE)
            cls.report = bench(cls._log)
        finally:
            os.environ.clear()
            os.environ.update(saved)

    @classmethod
    def tearDownClass(cls) -> None:
        cls._log.unlink(missing_ok=True)

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

        This run signed FLIGHT_PASS. The driver polled at 1.9 Hz and reported a
        comfortable margin; the 20 Hz record says the pair went through their
        separation requirement, reaching -1.332 m.
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
        The fixture is a single run, so what this pins is that the splitter
        returns it whole and in order rather than clipping it.
        """
        run = load_last_run(self._log)
        times = [row["evaluated_monotonic_s"] for row in run]

        self.assertTrue(all(b >= a for a, b in zip(times, times[1:])))
        self.assertLess(max(times) - min(times), 600.0)
        self.assertGreater(len(paired_frames(run)), 1000)


if __name__ == "__main__":
    unittest.main()
