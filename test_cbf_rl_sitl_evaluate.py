import copy

from cbf_rl_sitl_evaluate import DRONES, evaluate


SHA = "a" * 64


def _active_row(drone_id: str) -> dict:
    cbf = {"active": True, "reason": "cbf_filtered", "minimum_margin_m": 0.5}
    shielded = [0.1, 0.2, 0.0]
    return {
        "drone_id": drone_id,
        "station_keeping": True,
        "nominal_reason": "tracking_slot",
        "authority": "shadow_companion_only",
        "nominal_source": "cbf_rl_active",
        "output_velocity_enu_m_s": shielded,
        "cbf": cbf,
        "emergency": {"stage": "normal"},
        "active_offboard_conditions": [],
        "active_offboard_sender": {"latched_abort": None},
        "active_offboard_frame": {"decision": "transmit", "transmitted": True},
        "cbf_rl_shadow": {
            "mode": "active",
            "valid": True,
            "model_sha256": SHA,
            "applied": True,
            "transmit_authority": True,
            "normalized_action": [0.1, 0.2, 0.0],
            "nominal_delta_norm_m_s": 0.1,
            "shielded_delta_norm_m_s": 0.2,
            "shielded_velocity_enu_m_s": shielded,
            "cbf": cbf,
        },
    }


def _shadow_row(drone_id: str) -> dict:
    row = _active_row(drone_id)
    row["nominal_reason"] = "tracking_trajectory"
    row["cbf_rl_shadow"].update(
        {"mode": "shadow", "applied": False, "transmit_authority": False}
    )
    return row


FLIGHT = {
    "verdict": "FLIGHT_PASS",
    "final_state": {drone: {"armed": False} for drone in DRONES},
}


def test_valid_shadow_flight_passes() -> None:
    report = evaluate(
        [_shadow_row(drone) for drone in DRONES],
        FLIGHT,
        SHA,
        minimum_trajectory_samples=1,
    )

    assert report["verdict"] == "PASS"
    assert report["per_drone"]["UAV-01"]["minimum_shadow_cbf_margin_m"] == 0.5


def test_circle_driver_final_safe_record_is_accepted() -> None:
    flight = {
        "verdict": "FLIGHT_PASS",
        "circle": {"radius_m": 5.0},
        "records": [
            {
                "event": "FINAL_SAFE_STATE",
                "armed": {drone: False for drone in DRONES},
            }
        ],
    }

    report = evaluate(
        [_shadow_row(drone) for drone in DRONES],
        flight,
        SHA,
        minimum_trajectory_samples=1,
    )

    assert report["verdict"] == "PASS"
    assert report["checks"]["landed_and_disarmed"]


def test_applied_shadow_output_fails() -> None:
    rows = [_shadow_row(drone) for drone in DRONES]
    rows[0]["cbf_rl_shadow"]["applied"] = True

    report = evaluate(rows, FLIGHT, SHA, minimum_trajectory_samples=1)

    assert report["verdict"] == "FAIL"
    assert not report["checks"]["UAV-01:never_applied_or_authorized"]


def test_active_evaluation_requires_real_transmission() -> None:
    rows = [_active_row(drone) for drone in DRONES]
    flight = {
        "verdict": "FLIGHT_PASS",
        "final_state": {drone: {"armed": False} for drone in DRONES},
    }

    report = evaluate(
        rows, flight, SHA, minimum_trajectory_samples=1, expected_mode="active"
    )
    assert report["verdict"] == "PASS"

    withheld = copy.deepcopy(rows)
    withheld[0]["active_offboard_frame"]["transmitted"] = False
    report = evaluate(
        withheld, flight, SHA, minimum_trajectory_samples=1, expected_mode="active"
    )
    assert report["verdict"] == "FAIL"
    assert not report["per_drone"]["UAV-01"]["checks"][
        "every_frame_transmitted"
    ]


def test_active_trajectory_requires_both_drones_to_reach_end() -> None:
    rows = [_active_row(drone) for drone in DRONES]
    for row in rows:
        row["nominal_reason"] = "tracking_trajectory"
    flight = {
        **FLIGHT,
        "expected_trajectory_env": {"SWARM_TRAJECTORY_UAV_01_KIND": "linear"},
        "records": [
            {
                "step": "TRAJECTORY_HOLD_COMPLETE",
                "drone_id": drone,
                "reached_trajectory_end": drone == "UAV-01",
            }
            for drone in DRONES
        ],
    }

    report = evaluate(
        rows, flight, SHA, minimum_trajectory_samples=1, expected_mode="active"
    )

    assert report["milestone"] == "CBF_RL_ACTIVE_TRAJECTORY"
    assert report["verdict"] == "FAIL"
    assert report["checks"]["UAV-01:trajectory_completed"]
    assert not report["checks"]["UAV-02:trajectory_completed"]
