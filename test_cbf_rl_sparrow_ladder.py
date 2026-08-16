"""Each certified speed rung keeps its own model, matrix and hash.

A model from one rung must never be flown at a higher one, so the pinning here
is the thing that makes the ladder mean anything.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from cbf_command_gate import CbfCommandGate
from cbf_rl_env import sparrow_20m_cbf_config
from cbf_rl_policy import MODEL_FORMAT_SPARROW_20M_V1, ProximityCbfRlPolicy
from cbf_rl_shadow import _validate_active_contract


ROOT = Path(__file__).parent
RUNGS = {
    10.0: ("cbf_rl_policy_sparrow_20m_10ms_v1.json", "cbf_rl_sparrow_20m_certification.json"),
    15.0: ("cbf_rl_policy_sparrow_20m_15ms_v1.json", "cbf_rl_sparrow_15ms_certification.json"),
}


@pytest.mark.parametrize("speed_m_s", sorted(RUNGS))
def test_rung_model_matches_its_certified_hash(speed_m_s):
    model_name, certificate_name = RUNGS[speed_m_s]
    certificate = json.loads((ROOT / certificate_name).read_text())
    model = ROOT / model_name

    assert certificate["policy"]["path"] == model_name
    assert hashlib.sha256(model.read_bytes()).hexdigest() == certificate["policy"]["sha256"]
    assert certificate["certification_scope"] == "offline_only_no_flight_authority"


@pytest.mark.parametrize("speed_m_s", sorted(RUNGS))
def test_rung_policy_carries_its_own_speed_contract(speed_m_s):
    model_name, _ = RUNGS[speed_m_s]
    policy = ProximityCbfRlPolicy.load(ROOT / model_name)

    assert policy.as_dict()["format"] == MODEL_FORMAT_SPARROW_20M_V1
    assert policy.vehicle_profile == "sparrow"
    assert policy.minimum_separation_m == 20.0
    assert policy.maximum_velocity_m_s == speed_m_s
    _validate_active_contract(
        CbfCommandGate("UAV-01", ("UAV-02",), sparrow_20m_cbf_config(speed_m_s)),
        policy,
    )


@pytest.mark.parametrize("speed_m_s", sorted(RUNGS))
def test_rung_matrix_passed_over_every_speed_up_to_the_rung(speed_m_s):
    _, certificate_name = RUNGS[speed_m_s]
    matrix = json.loads((ROOT / certificate_name).read_text())["matrix"]

    # 14 geometries (0..180 by 15, plus vertical) x 2 dynamics variants.
    assert matrix["case_count"] == int(speed_m_s) * 14 * 2
    assert matrix["maximum_speed_m_s"] == int(speed_m_s)
    assert matrix["failure_count"] == 0
    assert matrix["verdict"] == "PASS"
    assert matrix["minimum_distance_m"] >= 20.0
    assert matrix["minimum_dynamic_margin_m"] >= 0.0


def test_a_lower_rung_model_is_refused_at_a_higher_contract():
    """The rule the whole ladder exists to enforce."""
    slow = ProximityCbfRlPolicy.load(ROOT / RUNGS[10.0][0])
    with pytest.raises(ValueError):
        _validate_active_contract(
            CbfCommandGate("UAV-01", ("UAV-02",), sparrow_20m_cbf_config(15.0)),
            slow,
        )


def test_a_geometry_family_carries_its_own_certified_speed():
    """One verdict for the whole matrix hides the shape of the failure.

    Sparrow at 20 m/s reads FAIL on four cases that are all vertical head-on,
    while horizontal -- the geometry a drawn mission actually flies, since
    missions are polylines at one altitude -- is clean all the way up.
    """
    from cbf_rl_x500_safety_matrix import certified_speed_by_geometry

    results = [
        {"encounter_angle_deg": 90.0, "speed_m_s": 1.0, "success": True},
        {"encounter_angle_deg": 90.0, "speed_m_s": 2.0, "success": True},
        {"encounter_angle_deg": None, "speed_m_s": 1.0, "success": True},
        {"encounter_angle_deg": None, "speed_m_s": 2.0, "success": False},
    ]
    assert certified_speed_by_geometry(results) == {
        "horizontal": 2,
        "vertical_180deg": 1,
    }


def test_one_failing_case_caps_its_family_even_if_faster_ones_pass():
    """A rung only counts when every rung below it is clean."""
    from cbf_rl_x500_safety_matrix import certified_speed_by_geometry

    results = [
        {"encounter_angle_deg": 45.0, "speed_m_s": 1.0, "success": True},
        {"encounter_angle_deg": 45.0, "speed_m_s": 2.0, "success": False},
        {"encounter_angle_deg": 45.0, "speed_m_s": 3.0, "success": True},
    ]
    assert certified_speed_by_geometry(results)["horizontal"] == 1
