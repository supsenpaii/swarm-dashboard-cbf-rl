import unittest

from cbf_fault_injection import run_fault_injection


class CbfFaultInjectionTests(unittest.TestCase):
    def test_offline_fault_suite_passes_and_covers_required_cases(self):
        report = run_fault_injection()
        self.assertTrue(report["pass"])
        self.assertEqual(
            set(report["scenarios"]),
            {"head_on", "crossing", "formation_collapse", "peer_stale", "geofence", "packet_loss"},
        )


if __name__ == "__main__":
    unittest.main()
