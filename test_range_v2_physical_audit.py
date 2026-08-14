import math
from pathlib import Path

import pytest

from range_v2_physical_audit import (
    FrameCalibrationAuditCache,
    affine_metric_depth,
    configuration_fingerprint,
    enu_to_ned,
    fit_affine_inverse_depth,
    frd_to_flu,
    image_ray_to_sensor_flu,
    optical_depth_to_slant,
    quaternion_multiply_xyzw,
    ray_scale,
    rotate_vector_xyzw,
    transform_point,
    _verify_checksum,
)


def quaternion_axis_angle(axis, degrees):
    half = math.radians(degrees) / 2.0
    scale = math.sin(half)
    return axis[0] * scale, axis[1] * scale, axis[2] * scale, math.cos(half)


def test_center_pixel_ray_scale_is_one():
    assert ray_scale(0.0, 0.0) == 1.0


def test_symmetric_left_right_rays():
    left = image_ray_to_sensor_flu(-0.4, 0.2)
    right = image_ray_to_sensor_flu(0.4, 0.2)
    assert left[0] == pytest.approx(right[0])
    assert left[1] == pytest.approx(-right[1])
    assert left[2] == pytest.approx(right[2])
    assert ray_scale(-0.4, 0.2) == pytest.approx(ray_scale(0.4, 0.2))


def test_known_optical_depth_to_slant_conversion():
    assert optical_depth_to_slant(4.0, 0.75, 0.0) == pytest.approx(5.0)


def test_known_camera_extrinsics():
    yaw_90 = quaternion_axis_angle((0.0, 0.0, 1.0), 90.0)
    point = transform_point((10.0, 20.0, 30.0), yaw_90, (1.0, 0.0, 0.0))
    assert point == pytest.approx((10.0, 21.0, 30.0))


def test_enu_ned_and_frd_flu_conversions():
    assert enu_to_ned((1.0, 2.0, 3.0)) == (2.0, 1.0, -3.0)
    assert frd_to_flu((1.0, 2.0, 3.0)) == (1.0, -2.0, -3.0)
    assert image_ray_to_sensor_flu(0.0, 0.0) == (1.0, -0.0, -0.0)


def test_synthetic_camera_level_pitch_yaw_and_image_directions():
    center = image_ray_to_sensor_flu(0.0, 0.0)
    assert rotate_vector_xyzw(center, (0.0, 0.0, 0.0, 1.0)) == pytest.approx((1, 0, 0))
    yaw_90 = quaternion_axis_angle((0, 0, 1), 90)
    yaw_minus_90 = quaternion_axis_angle((0, 0, 1), -90)
    assert rotate_vector_xyzw(center, yaw_90) == pytest.approx((0, 1, 0), abs=1e-12)
    assert rotate_vector_xyzw(center, yaw_minus_90) == pytest.approx((0, -1, 0), abs=1e-12)
    pitch_down = quaternion_axis_angle((0, 1, 0), 10)
    assert rotate_vector_xyzw(center, pitch_down)[2] < 0.0
    assert image_ray_to_sensor_flu(-0.2, 0.0)[1] > 0.0
    assert image_ray_to_sensor_flu(0.2, 0.0)[1] < 0.0
    assert image_ray_to_sensor_flu(0.0, -0.2)[2] > 0.0
    assert image_ray_to_sensor_flu(0.0, 0.2)[2] < 0.0


def test_quaternion_composition_order():
    yaw = quaternion_axis_angle((0, 0, 1), 90)
    pitch = quaternion_axis_angle((0, 1, 0), 90)
    composed = quaternion_multiply_xyzw(yaw, pitch)
    sequential = rotate_vector_xyzw(rotate_vector_xyzw((1, 0, 0), pitch), yaw)
    assert rotate_vector_xyzw((1, 0, 0), composed) == pytest.approx(sequential)
    reverse = quaternion_multiply_xyzw(pitch, yaw)
    assert rotate_vector_xyzw((1, 0, 0), reverse) != pytest.approx(sequential)


def test_affine_inverse_depth_synthetic_recovery():
    q = [1.0, 2.0, 3.0, 4.0]
    a, b = 0.2, 0.1
    depths = [1.0 / (a * item + b) for item in q]
    recovered_a, recovered_b = fit_affine_inverse_depth(q, depths)
    assert recovered_a == pytest.approx(a)
    assert recovered_b == pytest.approx(b)
    depth, denominator = affine_metric_depth(2.5, recovered_a, recovered_b)
    assert denominator == pytest.approx(0.6)
    assert depth == pytest.approx(1.0 / 0.6)


@pytest.mark.parametrize("scale,offset", [(1e-9, -1e-9), (0.1, -0.1)])
def test_invalid_or_near_zero_denominator(scale, offset):
    with pytest.raises(ValueError, match="calibration_denominator_invalid"):
        affine_metric_depth(1.0, scale, offset)


def test_group_session_reset_and_no_cross_frame_reuse():
    cache = FrameCalibrationAuditCache()
    cache.put("run-a", 2, 10, 0.1, 0.2)
    assert cache.get("run-a", 2, 10) == (0.1, 0.2)
    with pytest.raises(ValueError, match="cross_frame_or_context"):
        cache.get("run-a", 2, 11)
    with pytest.raises(ValueError, match="cross_frame_or_context"):
        cache.get("run-b", 2, 10)
    cache.reset()
    with pytest.raises(ValueError, match="cross_frame_or_context"):
        cache.get("run-a", 2, 10)


def test_configuration_fingerprint_is_deterministic_and_order_independent():
    first = configuration_fingerprint({"resolution": [480, 270], "fx": 299.84})
    second = configuration_fingerprint({"fx": 299.84, "resolution": [480, 270]})
    assert first == second
    assert first != configuration_fingerprint({"resolution": [640, 360], "fx": 299.84})


def test_nonfinite_ray_and_transform_fail_closed():
    with pytest.raises(ValueError):
        ray_scale(math.nan, 0.0)
    with pytest.raises(ValueError):
        rotate_vector_xyzw((1, 0, 0), (0, 0, 0, 0))


def test_source_checksum_mismatch_aborts(tmp_path: Path):
    source = tmp_path / "source.jsonl"
    source.write_text("locked\n", encoding="utf-8")
    with pytest.raises(ValueError, match="checksum_mismatch:source"):
        _verify_checksum(source, "0" * 64, "source")
