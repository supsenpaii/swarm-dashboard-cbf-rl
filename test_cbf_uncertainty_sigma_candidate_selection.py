from __future__ import annotations

import unittest

from cbf_uncertainty_sigma_candidate_selection import choose_candidate


class SigmaCandidateSelectionTests(unittest.TestCase):
    def test_selects_highest_eligible_candidate(self) -> None:
        rows = [
            {"sigma": 0.05, "eligible_for_flight_validation": True},
            {"sigma": 0.10, "eligible_for_flight_validation": True},
            {"sigma": 0.15, "eligible_for_flight_validation": False},
        ]
        self.assertEqual(choose_candidate(rows), 0.10)


if __name__ == "__main__":
    unittest.main()
