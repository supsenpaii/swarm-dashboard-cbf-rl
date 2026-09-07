import unittest

from cbf_sitl_shadow_gate import evaluate_snapshots


def snapshot(*, cbf_active=True, intervention=0.0, hold_reason=None):
    filtered = {
        "active": cbf_active,
        "velocity_enu_m_s": [0.0, 0.0, 0.0],
        "intervention_norm_m_s": intervention,
    }
    if hold_reason:
        filtered["reason"] = hold_reason
    return {
        "swarm_state_origin_configured": True,
        "swarm_state": {
            "UAV-01": {"valid": True},
            "UAV-02": {"valid": True},
        },
        "formation": {
            "enabled": True,
            "commands": {"UAV-02": {"active": True}},
        },
        "cbf": {"enabled": True, "commands": {"UAV-02": filtered}},
    }


class CbfSitlShadowGateTests(unittest.TestCase):
    def test_valid_shadow_contract_passes(self):
        result = evaluate_snapshots([snapshot(intervention=0.2)], require_intervention=True)
        self.assertTrue(result["pass"])
        self.assertEqual(result["cbf_interventions"], 1)

    def test_hold_reason_can_be_required(self):
        result = evaluate_snapshots(
            [snapshot(cbf_active=False, hold_reason="peer_state_invalid")],
            required_hold_reasons=("peer_state_invalid",),
        )
        self.assertTrue(result["pass"])

    def test_missing_origin_or_disabled_shadow_fails(self):
        bad = snapshot()
        bad["swarm_state_origin_configured"] = False
        bad["cbf"]["enabled"] = False
        result = evaluate_snapshots([bad])
        self.assertFalse(result["pass"])
        self.assertIn("sample[0]: common_enu_origin_unconfigured", result["violations"])
        self.assertIn("sample[0]: cbf_shadow_disabled", result["violations"])


if __name__ == "__main__":
    unittest.main()
