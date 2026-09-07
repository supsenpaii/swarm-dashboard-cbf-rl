import unittest

from peer_state import PeerStateRegistry, make_peer_state, parse_endpoints


def packet(sequence=1, healthy=True):
    return make_peer_state(
        drone_id="UAV-02",
        sequence=sequence,
        position_enu_m=(1.0, 2.0, 3.0),
        velocity_enu_m_s=(0.1, 0.2, 0.3),
        healthy=healthy,
        timestamp_ms=1000,
    )


class PeerStateTests(unittest.TestCase):
    def test_endpoint_parser_rejects_invalid_input(self):
        self.assertEqual(parse_endpoints("10.0.0.2:14670, 10.0.0.3:14670"), (("10.0.0.2", 14670), ("10.0.0.3", 14670)))
        with self.assertRaises(ValueError):
            parse_endpoints("10.0.0.2")

    def test_state_is_valid_when_fresh_and_healthy(self):
        registry = PeerStateRegistry(max_age_s=0.5)
        self.assertTrue(registry.ingest(packet(), received_monotonic_s=10.0))
        state = registry.snapshot(now_monotonic_s=10.2)["peers"]["UAV-02"]
        self.assertTrue(state["valid"])
        self.assertEqual(state["frame"], "ENU")

    def test_sequence_gap_counts_packet_loss(self):
        registry = PeerStateRegistry()
        registry.ingest(packet(4), received_monotonic_s=10.0)
        registry.ingest(packet(7), received_monotonic_s=10.1)
        state = registry.snapshot(now_monotonic_s=10.2)["peers"]["UAV-02"]
        self.assertEqual(state["lost_packets"], 2)

    def test_stale_or_out_of_order_peer_is_invalid(self):
        registry = PeerStateRegistry(max_age_s=0.5)
        registry.ingest(packet(4), received_monotonic_s=10.0)
        self.assertFalse(registry.ingest(packet(4), received_monotonic_s=10.1))
        state = registry.snapshot(now_monotonic_s=10.6)["peers"]["UAV-02"]
        self.assertFalse(state["valid"])
        self.assertEqual(state["reason"], "peer_stale")


if __name__ == "__main__":
    unittest.main()
