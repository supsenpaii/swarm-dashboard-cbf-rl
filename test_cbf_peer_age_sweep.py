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
        """Re-measured 2026-08-18 at 4000 ms, and NOT an argument for 4000 ms.

        This pin used to sit at 1000 ms. It moved because the scenario stopped
        modelling a vehicle that changes velocity in one 20 ms step: an instant
        vehicle acts on a stale peer by putting itself somewhere a rate-limited
        one physically cannot reach, so staleness hurt it far sooner. Honestly
        modelled, the crossing survives 2000 ms and breaks at 4000 ms.

        Production stays at 100 ms. A simulation getting less pessimistic is
        not evidence about the link, and this file exists to say so.
        """
        survivable = _run_at_age(2000.0)
        row = _run_at_age(4000.0)

        self.assertTrue(survivable["hard_feasible"])
        self.assertFalse(row["hard_feasible"])
        self.assertGreater(row["infeasible_frames"], 0)
        self.assertLess(row["min_margin_reported_m"], 0.0)


if __name__ == "__main__":
    unittest.main()
