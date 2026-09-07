"""Pure Gazebo-sim-time pose interpolation and optical-center GT geometry."""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any, Mapping


GT_CONTRACT_VERSION = "gazebo_actual_pose_optical_center_v2"
CAMERA_SENSOR_TRANSLATION_IN_CAMERA_LINK_M = (0.0, 0.0, 0.0)
CAMERA_SENSOR_QUATERNION_XYZW = (0.0, 0.0, 0.0, 1.0)
TARGET_REFERENCE = "sparrow_gimbal_1_model_origin"
_CONTRACT = {
    "camera_extrinsic_quaternion_xyzw": CAMERA_SENSOR_QUATERNION_XYZW,
    "camera_extrinsic_translation_m": CAMERA_SENSOR_TRANSLATION_IN_CAMERA_LINK_M,
    "clock": "gazebo_sim_time",
    "distance": "norm(target_actual_model_origin-camera_actual_optical_center)",
    "interpolation": "strict_two_sample_bracket_linear_position_slerp_orientation",
    "target_reference": TARGET_REFERENCE,
    "version": GT_CONTRACT_VERSION,
}
GT_CONTRACT_SHA256 = hashlib.sha256(
    json.dumps(_CONTRACT, sort_keys=True, separators=(",", ":")).encode()
).hexdigest()


def quaternion_multiply(first: tuple[float, ...], second: tuple[float, ...]) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def rotate_vector(quaternion: tuple[float, ...], vector: tuple[float, ...]) -> tuple[float, float, float]:
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return (
        vx + w*tx + (y*tz-z*ty),
        vy + w*ty + (z*tx-x*tz),
        vz + w*tz + (x*ty-y*tx),
    )


def quaternion_slerp(first: tuple[float, ...], second: tuple[float, ...], ratio: float) -> tuple[float, float, float, float]:
    a = [float(value) for value in first]
    b = [float(value) for value in second]
    dot = sum(x*y for x, y in zip(a, b))
    if dot < 0.0:
        b = [-value for value in b]
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        value = [x + ratio*(y-x) for x, y in zip(a, b)]
    else:
        theta = math.acos(dot)
        scale = math.sin(theta)
        value = [
            math.sin((1.0-ratio)*theta)/scale*x + math.sin(ratio*theta)/scale*y
            for x, y in zip(a, b)
        ]
    norm = math.sqrt(sum(component*component for component in value))
    if norm <= 1e-12:
        raise ValueError("interpolated_quaternion_invalid")
    return tuple(component/norm for component in value)  # type: ignore[return-value]


def interpolate_pose(lower: Mapping[str, Any], upper: Mapping[str, Any], timestamp_s: float) -> dict[str, tuple[float, ...]]:
    lower_t, upper_t, query = float(lower["timestamp_s"]), float(upper["timestamp_s"]), float(timestamp_s)
    if not lower_t <= query <= upper_t or upper_t <= lower_t:
        raise ValueError("pose_interpolation_requires_valid_bracket")
    ratio = (query-lower_t)/(upper_t-lower_t)
    lower_position, upper_position = lower["position_xyz"], upper["position_xyz"]
    return {
        "position_xyz": tuple(float(lower_position[i]) + ratio*(float(upper_position[i])-float(lower_position[i])) for i in range(3)),
        "quaternion_xyzw": quaternion_slerp(tuple(lower["quaternion_xyzw"]), tuple(upper["quaternion_xyzw"]), ratio),
    }


def camera_optical_center(camera_link_pose: Mapping[str, Any]) -> dict[str, tuple[float, ...]]:
    link_position = tuple(float(value) for value in camera_link_pose["position_xyz"])
    link_quaternion = tuple(float(value) for value in camera_link_pose["quaternion_xyzw"])
    offset = rotate_vector(link_quaternion, CAMERA_SENSOR_TRANSLATION_IN_CAMERA_LINK_M)
    return {
        "position_xyz": tuple(link_position[i] + offset[i] for i in range(3)),
        "quaternion_xyzw": quaternion_multiply(link_quaternion, CAMERA_SENSOR_QUATERNION_XYZW),
    }


def optical_center_distance(camera_link_pose: Mapping[str, Any], target_pose: Mapping[str, Any]) -> dict[str, Any]:
    optical = camera_optical_center(camera_link_pose)
    target = tuple(float(value) for value in target_pose["position_xyz"])
    vector = tuple(target[i]-optical["position_xyz"][i] for i in range(3))
    distance = math.sqrt(sum(value*value for value in vector))
    return {"camera_optical_center_xyz": optical["position_xyz"], "camera_optical_quaternion_xyzw": optical["quaternion_xyzw"], "target_reference_xyz": target, "distance_m": distance}
