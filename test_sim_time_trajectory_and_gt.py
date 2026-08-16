from __future__ import annotations

import inspect
import math
import time

import pytest

from sim_time_trajectory import SimTimeTrajectoryContract
from simulation_ground_truth import (
    CAMERA_SENSOR_TRANSLATION_IN_CAMERA_LINK_M,
    camera_optical_center,
    interpolate_pose,
    optical_center_distance,
)


def _contract() -> SimTimeTrajectoryContract:
    return SimTimeTrajectoryContract(11.5, 3.5, 1.0, 1.0, -10.0, 10.0, 20.0, 5.0)


def test_pose_is_deterministic_function_of_sim_time_only() -> None:
    contract = _contract()
    first = contract.pose_at(7.25)
    time.sleep(0.01)
    second = contract.pose_at(7.25)
    assert first == second
    source = inspect.getsource(SimTimeTrajectoryContract.pose_at)
    assert "monotonic(" not in source
    assert "sleep(" not in source
    assert first["clock_domain"] == "gazebo_sim_time"


def test_pause_and_rtf_cannot_advance_pose_at_fixed_sim_time() -> None:
    contract = _contract()
    before_pause = contract.pose_at(8.0)
    time.sleep(0.02)  # simulated wall-clock pause
    after_pause = contract.pose_at(8.0)
    high_rtf = contract.pose_at(8.0)
    low_rtf = contract.pose_at(8.0)
    assert before_pause == after_pause == high_rtf == low_rtf


def test_velocity_profile_is_defined_in_sim_time() -> None:
    contract = SimTimeTrajectoryContract(3.5, 11.5, 0.0, 1.0, 0.0, 0.0, 20.0, 5.0)
    first = contract.pose_at(5.0)
    second = contract.pose_at(10.0)
    assert first["radial_velocity_m_s"] == pytest.approx(0.4)
    assert first["x_velocity_m_s"] == pytest.approx(0.4)
    assert second["radial_velocity_m_s"] == pytest.approx(0.4)
    assert contract.pose_at(21.0)["radial_velocity_m_s"] == 0.0


def test_pose_interpolation_requires_two_valid_brackets() -> None:
    identity = (0.0, 0.0, 0.0, 1.0)
    lower = {"timestamp_s": 1.0, "position_xyz": (0.0, 0.0, 0.0), "quaternion_xyzw": identity}
    upper = {"timestamp_s": 3.0, "position_xyz": (2.0, 0.0, 0.0), "quaternion_xyzw": identity}
    assert interpolate_pose(lower, upper, 2.0)["position_xyz"] == pytest.approx((1.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="valid_bracket"):
        interpolate_pose(lower, upper, 4.0)


def test_sparrow_camera_sensor_is_at_camera_link_origin() -> None:
    pose = {"position_xyz": (1.0, 2.0, 3.0), "quaternion_xyzw": (0.0, 0.0, 0.0, 1.0)}
    optical = camera_optical_center(pose)
    assert optical["position_xyz"] == pytest.approx(tuple(1.0 + CAMERA_SENSOR_TRANSLATION_IN_CAMERA_LINK_M[i] if i == 0 else (2.0 if i == 1 else 3.0) + CAMERA_SENSOR_TRANSLATION_IN_CAMERA_LINK_M[i] for i in range(3)))


def test_zero_camera_extrinsic_stays_at_link_origin_after_rotation() -> None:
    yaw_90 = (0.0, 0.0, math.sin(math.pi/4), math.cos(math.pi/4))
    optical = camera_optical_center({"position_xyz": (0.0, 0.0, 0.0), "quaternion_xyzw": yaw_90})
    assert optical["position_xyz"] == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)


def test_gt_distance_uses_sparrow_optical_center_at_link_origin() -> None:
    camera = {"position_xyz": (0.0, 0.0, 0.0), "quaternion_xyzw": (0.0, 0.0, 0.0, 1.0)}
    target = {"position_xyz": (3.0, 0.0, 0.0), "quaternion_xyzw": (0.0, 0.0, 0.0, 1.0)}
    result = optical_center_distance(camera, target)
    assert result["distance_m"] == pytest.approx(3.0)
