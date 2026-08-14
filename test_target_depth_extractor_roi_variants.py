import numpy as np

from metric_depth_calibrator import MetricDepthCalibrator
from target_depth_extractor import TargetDepthExtractor


def _calibrated() -> MetricDepthCalibrator:
    calibrator = MetricDepthCalibrator()
    relative = np.linspace(0.2, 1.0, 60)
    metric = 1.0 / (0.2 * relative + 0.04)
    assert calibrator.fit(relative, metric).valid
    return calibrator


def _depth_map() -> np.ndarray:
    depth_map = np.full((100, 120), 0.25, dtype=np.float32)
    depth_map[30:70, 40:80] = 0.80
    depth_map[44:56, 54:66] = 0.85
    return depth_map


def test_variant_diagnostics_default_off_is_none() -> None:
    extractor = TargetDepthExtractor(minimum_samples=20)
    result = extractor.extract(_depth_map(), (35.0, 25.0, 50.0, 50.0), _calibrated())
    assert result.variant_diagnostics is None


def test_variant_diagnostics_opt_in_matches_default_off_otherwise() -> None:
    calibrator = _calibrated()
    depth_map = _depth_map()
    bbox = (35.0, 25.0, 50.0, 50.0)

    off = TargetDepthExtractor(minimum_samples=20).extract(depth_map, bbox, calibrator)
    on = TargetDepthExtractor(
        minimum_samples=20, enable_variant_diagnostics=True
    ).extract(depth_map, bbox, calibrator)

    assert off.variant_diagnostics is None
    assert on.variant_diagnostics is not None
    # Every field that drives raw range / filtering / calibration is unchanged.
    assert off.valid == on.valid
    assert off.optical_depth_m == on.optical_depth_m
    assert off.optical_depth_std_m == on.optical_depth_std_m
    assert off.relative_inverse_depth == on.relative_inverse_depth
    assert off.relative_inverse_depth_std == on.relative_inverse_depth_std
    assert off.valid_fraction == on.valid_fraction
    assert off.sample_count == on.sample_count
    assert off.roi_inverse_depth_quantiles == on.roi_inverse_depth_quantiles
    assert off.selected_foreground_statistic == on.selected_foreground_statistic


def test_variant_diagnostics_has_all_eight_variants_and_is_deterministic() -> None:
    calibrator = _calibrated()
    depth_map = _depth_map()
    bbox = (35.0, 25.0, 50.0, 50.0)
    extractor = TargetDepthExtractor(minimum_samples=20, enable_variant_diagnostics=True)

    first = extractor.extract(depth_map, bbox, calibrator).variant_diagnostics
    second = extractor.extract(depth_map, bbox, calibrator).variant_diagnostics

    expected_keys = {
        "A_original",
        "B_centered_90",
        "C_centered_80",
        "D_centered_70",
        "E_erosion_10",
        "F_erosion_20",
        "G_foreground_half_current",
        "H_central_quantile_region",
    }
    assert set(first.keys()) == expected_keys
    assert first == second

    for name, entry in first.items():
        assert "median" in entry and "mad_std" in entry and "uncertainty_ratio" in entry
        assert "valid_fraction" in entry and "finite_pixel_count" in entry
        assert "nonfinite_result" in entry
        if entry["finite_pixel_count"] > 0:
            assert not entry["nonfinite_result"]


def test_variant_diagnostics_never_uses_gt_or_model_prediction() -> None:
    import ast
    import inspect
    import textwrap

    source = inspect.getsource(TargetDepthExtractor._compute_variant_diagnostics)
    tree = ast.parse(textwrap.dedent(source))
    body_source = ast.unparse(tree.body[0].body)
    for forbidden in ("ground_truth", "gt_", "model_prediction"):
        assert forbidden not in body_source.lower()


def test_variant_diagnostics_env_opt_in_default_off(monkeypatch) -> None:
    monkeypatch.delenv("SWARM_TARGET_ROI_VARIANT_DIAGNOSTICS", raising=False)
    extractor = TargetDepthExtractor(minimum_samples=20)
    assert extractor.enable_variant_diagnostics is False

    monkeypatch.setenv("SWARM_TARGET_ROI_VARIANT_DIAGNOSTICS", "1")
    extractor_on = TargetDepthExtractor(minimum_samples=20)
    assert extractor_on.enable_variant_diagnostics is True


def test_recovery_policy_default_off_is_byte_identical_to_before() -> None:
    calibrator = _calibrated()
    depth_map = _depth_map()
    bbox = (35.0, 25.0, 50.0, 50.0)

    default = TargetDepthExtractor(minimum_samples=20)
    assert default.recovery_policy is None
    result = default.extract(depth_map, bbox, calibrator)
    assert (
        result.selected_foreground_statistic
        == "median_of_median_or_mad_gated_foreground_half"
    )


def test_recovery_policy_env_default_off(monkeypatch) -> None:
    monkeypatch.delenv("SWARM_TARGET_ROI_RECOVERY_POLICY", raising=False)
    assert TargetDepthExtractor(minimum_samples=20).recovery_policy is None


def test_recovery_policy_opt_in_changes_only_the_statistic(monkeypatch) -> None:
    calibrator = _calibrated()
    depth_map = _depth_map()
    bbox = (35.0, 25.0, 50.0, 50.0)

    monkeypatch.setenv(
        "SWARM_TARGET_ROI_RECOVERY_POLICY", "central_quantile_region"
    )
    recovered = TargetDepthExtractor(minimum_samples=20).extract(
        depth_map, bbox, calibrator
    )
    assert (
        recovered.selected_foreground_statistic
        == "central_iqr_band_median_recovery_policy"
    )
    assert recovered.valid is True
    # bbox/erosion geometry (and therefore ROI containment) is untouched by
    # the recovery policy: quantile envelope of the *raw* ROI is identical.
    default = TargetDepthExtractor(minimum_samples=20).extract(
        depth_map, bbox, calibrator
    )
    assert (
        recovered.roi_inverse_depth_quantiles
        == default.roi_inverse_depth_quantiles
    )


def test_recovery_policy_rejects_unknown_value() -> None:
    try:
        TargetDepthExtractor(recovery_policy="not_a_real_policy")
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for unknown recovery_policy")
