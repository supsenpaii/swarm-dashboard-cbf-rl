from __future__ import annotations

from dataclasses import replace

from range_applicability_gate import (
    RangeApplicabilityGate,
    RangeApplicabilityInputs,
)


def valid_inputs() -> RangeApplicabilityInputs:
    return RangeApplicabilityInputs(
        image_ray_x=0.0,
        image_ray_y=-0.25,
        target_bearing_down=0.02,
        camera_optical_axis_down=0.17,
        calibration_condition_number=3500.0,
        calibration_residual_m_inv=0.002,
        calibration_inlier_fraction=0.70,
        anchor_spatial_coverage_fraction=0.50,
        target_anchor_extrapolation_iqr=0.0,
        ray_range_relative_std=0.12,
    )


def test_centered_well_calibrated_range_is_applicable() -> None:
    gate = RangeApplicabilityGate(mode="active")

    result = gate.evaluate(valid_inputs())

    assert result.applicable
    assert result.measurement_usable
    assert not result.enforced
    assert gate.status()["applicable_count"] == 1


def test_oblique_image_ray_is_shadowed_or_enforced_by_mode() -> None:
    oblique = replace(valid_inputs(), image_ray_x=-0.77)
    shadow_gate = RangeApplicabilityGate(mode="shadow")
    active_gate = RangeApplicabilityGate(mode="active")

    shadow = shadow_gate.evaluate(oblique)
    active = active_gate.evaluate(oblique)

    assert not shadow.applicable
    assert shadow.measurement_usable
    assert not shadow.enforced
    assert shadow.reason == "image_ray_x_outside_validated_envelope"
    assert not active.applicable
    assert not active.measurement_usable
    assert active.enforced
    assert active_gate.status()["enforced_rejection_count"] == 1


def test_calibration_health_and_uncertainty_fail_closed() -> None:
    gate = RangeApplicabilityGate(mode="active")

    poor_coverage = gate.evaluate(
        replace(valid_inputs(), anchor_spatial_coverage_fraction=0.10)
    )
    uncertain = gate.evaluate(
        replace(valid_inputs(), ray_range_relative_std=0.50)
    )

    assert poor_coverage.reason == "anchor_spatial_coverage_too_low"
    assert not poor_coverage.measurement_usable
    assert uncertain.reason == "range_relative_uncertainty_too_large"
    assert not uncertain.measurement_usable


def test_range_model_risk_geometry_is_refused_not_merely_flagged() -> None:
    # |image_ray_x| = 0.20 sits inside this gate's 0.45 envelope but above the
    # learned model's 0.13 known-risk threshold, so before this rule the frame
    # was accepted with nothing but an inflated sigma to signal the risk.
    risky = replace(
        valid_inputs(),
        image_ray_x=0.20,
        range_model_risk_geometry=True,
    )
    gate = RangeApplicabilityGate(mode="active")

    assert gate.evaluate(replace(risky, range_model_risk_geometry=False)).applicable

    result = gate.evaluate(risky)

    assert not result.applicable
    assert not result.measurement_usable
    assert result.enforced
    assert result.reason == "range_model_known_risk_geometry"


def test_range_model_risk_geometry_defaults_off_for_existing_callers() -> None:
    # The learned model is opt-in; callers that never enable it must see no
    # behaviour change from this field existing.
    assert valid_inputs().range_model_risk_geometry is False
    assert RangeApplicabilityGate(mode="active").evaluate(valid_inputs()).applicable


def test_range_model_risk_geometry_is_observable_without_enforcement() -> None:
    risky = replace(valid_inputs(), range_model_risk_geometry=True)
    shadow = RangeApplicabilityGate(mode="shadow")

    result = shadow.evaluate(risky)

    assert not result.applicable
    assert result.reason == "range_model_known_risk_geometry"
    # Shadow mode reports the rejection without withholding the measurement.
    assert result.measurement_usable
    assert shadow.status()["enforced_rejection_count"] == 0
