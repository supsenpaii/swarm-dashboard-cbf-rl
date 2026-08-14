"""Regression pins for the peer-age safety question.

The full 1 ms sweep is intentionally run only by cbf_peer_age_sweep.py. These
cheap points pin the production contract and prove that unbounded age is still
unsafe without treating a safer trajectory as permission to loosen 100 ms.
"""

from __future__ import annotations

import unittest

from cbf_peer_age_sweep import _run_at_age


class PeerAgeCrossingRegressionTests(unittest.TestCase):
    def test_production_100_ms_is_hard_and_strict_feasible(self) -> None:
        row = _run_at_age(100.0)
        self.assertTrue(row["hard_feasible"])
        self.assertTrue(row["strict_feasible"])
        self.assertGreater(row["min_margin_reported_m"], 0.0)

    def test_unbounded_peer_age_remains_hard_infeasible(self) -> None:
        row = _run_at_age(1000.0)
        self.assertFalse(row["hard_feasible"])
        self.assertGreater(row["infeasible_frames"], 0)


if __name__ == "__main__":
    unittest.main()
