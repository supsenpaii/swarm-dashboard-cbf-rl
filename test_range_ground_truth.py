from __future__ import annotations

import math

import pytest

from range_ground_truth import (
    RTK_GROUND_TRUTH_SOURCE,
    SIMULATION_GROUND_TRUTH_SOURCE,
    RangeGroundTruthRouter,
    RtkGroundTruthConfig,
    RtkGroundTruthProvider,
)


def telemetry(
    *,
    latitude_deg: float,
    longitude_deg: float,
    altitude_msl_m: float,
    timestamp_s: float = 10.0,
    fix_type: int = 6,
    eph_m: float = 0.02,
    epv_m: float = 0.03,
    yaw_deg: float = 0.0,
) -> dict:
    return {
        "online": True,
        "dashboard_received_monotonic_s": timestamp_s,
        "global_position": {
            "latitude_deg": latitude_deg,
            "longitude_deg": longitude_deg,
            "altitude_msl_m": altitude_msl_m,
        },
        "gps": {
            "fix_type": fix_type,
            "eph_m": eph_m,
            "epv_m": epv_m,
        },
        "attitude": {
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": yaw_deg,
        },
    }


def provider(samples: dict[str, dict], **config_values) -> RtkGroundTruthProvider:
    config = RtkGroundTruthConfig(
        camera_antenna_to_center_body_frd_m=config_values.get(
            "camera_lever", (0.0, 0.0, 0.0)
        ),
        target_antenna_to_center_body_frd_m=config_values.get(
            "target_lever", (0.0, 0.0, 0.0)
        ),
        lever_arm_uncertainty_m=0.01,
    )
    return RtkGroundTruthProvider(samples.get, config)


def test_rtk_fixed_provider_computes_3d_slant_range() -> None:
    latitude = 47.0
    north_m = 10.0
    latitude_delta = math.degrees(north_m / 6_378_137.0)
    samples = {
        "UAV-01": telemetry(
            latitude_deg=latitude,
            longitude_deg=8.0,
            altitude_msl_m=500.0,
        ),
        "UAV-02": telemetry(
            latitude_deg=latitude + latitude_delta,
            longitude_deg=8.0,
            altitude_msl_m=502.0,
            timestamp_s=10.03,
        ),
    }

    result = provider(samples).evaluate("UAV-01", 10.02, "UAV-02")

    assert result["available"]
    assert result["ground_truth_source"] == RTK_GROUND_TRUTH_SOURCE
    assert result["ground_truth_quality"] == "rtk_fixed"
    assert result["ground_truth_lever_arm_corrected"]
    assert result["ground_truth_time_offset_ms"] == pytest.approx(30.0)
    assert result["camera_to_target_center_m"] == pytest.approx(
        math.hypot(north_m, 2.0),
        abs=0.03,
    )
    assert result["ground_truth_uncertainty_m"] < 0.20


def test_rtk_provider_applies_body_frd_lever_arm() -> None:
    longitude_delta = math.degrees(10.0 / 6_378_137.0)
    samples = {
        "UAV-01": telemetry(
            latitude_deg=0.0,
            longitude_deg=0.0,
            altitude_msl_m=0.0,
            yaw_deg=90.0,
        ),
        "UAV-02": telemetry(
            latitude_deg=0.0,
            longitude_deg=longitude_delta,
            altitude_msl_m=0.0,
        ),
    }

    result = provider(samples, camera_lever=(1.0, 0.0, 0.0)).evaluate(
        "UAV-01",
        10.0,
        "UAV-02",
    )

    assert result["available"]
    assert result["camera_to_target_center_m"] == pytest.approx(9.0, abs=0.01)


@pytest.mark.parametrize(
    ("change", "reason"),
    (
        ({"fix_type": 5}, "UAV-02_rtk_not_fixed"),
        ({"timestamp_s": 10.2}, "UAV-02_telemetry_not_synchronized"),
        ({"eph_m": 0.3}, "rtk_ground_truth_uncertainty_too_large"),
    ),
)
def test_rtk_provider_rejects_low_quality_or_unsynchronized_labels(
    change: dict,
    reason: str,
) -> None:
    samples = {
        "UAV-01": telemetry(
            latitude_deg=47.0,
            longitude_deg=8.0,
            altitude_msl_m=500.0,
        ),
        "UAV-02": telemetry(
            latitude_deg=47.0001,
            longitude_deg=8.0,
            altitude_msl_m=500.0,
            **change,
        ),
    }

    result = provider(samples).evaluate("UAV-01", 10.0, "UAV-02")

    assert not result["available"]
    assert result["error"] == reason


def test_rtk_environment_requires_explicit_lever_arm_calibration(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv(
        "SWARM_RANGE_RTK_CAMERA_ANTENNA_TO_CENTER_BODY_FRD_M",
        raising=False,
    )
    monkeypatch.delenv(
        "SWARM_RANGE_RTK_TARGET_ANTENNA_TO_CENTER_BODY_FRD_M",
        raising=False,
    )
    monkeypatch.delenv(
        "SWARM_RANGE_RTK_LEVER_ARM_UNCERTAINTY_M",
        raising=False,
    )

    configured = RtkGroundTruthProvider.from_environment(lambda _: None)

    assert not configured.enabled
    assert "camera_antenna_to_center" in configured.load_error


def test_simulation_router_adds_locked_label_metadata() -> None:
    router = RangeGroundTruthRouter(
        SIMULATION_GROUND_TRUTH_SOURCE,
        lambda *_: {
            "available": True,
            "camera_to_target_center_m": 8.0,
            "pose_age_ms": 12.0,
        },
        RtkGroundTruthProvider(lambda _: None, None),
    )

    result = router("UAV-01", 10.0, 20.0, "UAV-02")

    assert result["ground_truth_source"] == SIMULATION_GROUND_TRUTH_SOURCE
    assert result["ground_truth_quality"] == "simulation_exact"
    assert result["ground_truth_uncertainty_m"] == 0.0
    assert result["ground_truth_time_offset_ms"] == 12.0
    assert result["ground_truth_lever_arm_corrected"]
    assert router.status()["available_count"] == 1
