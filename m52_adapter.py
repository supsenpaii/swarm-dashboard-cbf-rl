from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from visual_follow_target import CameraRayProjector


@dataclass(frozen=True)
class AnchorDiagnostic:
    grid_index: int
    pixel_u: float
    pixel_v: float
    accepted: bool
    reason: str
    ray_ned_unit: tuple[float, float, float] | None = None
    ground_slant_range_m: float | None = None
    metric_optical_depth_m: float | None = None
    relative_inverse_depth: float | None = None
    normalized_image_x: float | None = None
    normalized_image_y: float | None = None
    sensor_forward_component: float | None = None
    local_sample_count: int = 0
    local_inverse_depth_mean: float | None = None
    local_inverse_depth_std: float | None = None
    local_inverse_depth_cv: float | None = None
    target_bbox_exclusion_result: str = "not_evaluated"
    contribution_to_fit: str = "not_in_fit"

    def as_dict(self) -> dict[str, float | int | bool | str | list[float] | None]:
        return {
            "grid_index": self.grid_index,
            "pixel_u": self.pixel_u,
            "pixel_v": self.pixel_v,
            "accepted": self.accepted,
            "reason": self.reason,
            "ray_ned_unit": (
                None if self.ray_ned_unit is None else list(self.ray_ned_unit)
            ),
            "ground_slant_range_m": self.ground_slant_range_m,
            "metric_optical_depth_m": self.metric_optical_depth_m,
            "relative_inverse_depth": self.relative_inverse_depth,
            "normalized_image_x": self.normalized_image_x,
            "normalized_image_y": self.normalized_image_y,
            "sensor_forward_component": self.sensor_forward_component,
            "local_sample_count": self.local_sample_count,
            "local_inverse_depth_mean": self.local_inverse_depth_mean,
            "local_inverse_depth_std": self.local_inverse_depth_std,
            "local_inverse_depth_cv": self.local_inverse_depth_cv,
            "target_bbox_exclusion_result": (
                self.target_bbox_exclusion_result
            ),
            "contribution_to_fit": self.contribution_to_fit,
        }


@dataclass(frozen=True)
class GroundAnchors:
    relative_inverse_depth: np.ndarray
    metric_optical_depth_m: np.ndarray
    pixel_uv: np.ndarray
    candidate_count: int = 0
    accepted_ground_anchor_count: int = 0
    rejected_reason_counts: tuple[tuple[str, int], ...] = ()
    relative_inverse_depth_quantiles: tuple[float, ...] = ()
    metric_depth_quantiles_m: tuple[float, ...] = ()
    spatial_bin_count: int = 0
    spatial_coverage_fraction: float = 0.0
    diagnostics: tuple[AnchorDiagnostic, ...] = ()


class M52GroundAnchorAdapter:
    """Use M52 ray/ground intersections as metric calibration anchors."""

    def __init__(
        self,
        projector: CameraRayProjector,
        *,
        grid_columns: int = 12,
        grid_rows: int = 8,
        minimum_range_m: float = 1.0,
        maximum_range_m: float = 100.0,
        minimum_ground_image_fraction: float = 0.45,
        maximum_local_inverse_depth_cv: float = 0.30,
    ) -> None:
        self.projector = projector
        self.grid_columns = max(4, int(grid_columns))
        self.grid_rows = max(3, int(grid_rows))
        self.minimum_range_m = max(0.1, float(minimum_range_m))
        self.maximum_range_m = max(
            self.minimum_range_m,
            float(maximum_range_m),
        )
        self.minimum_ground_image_fraction = max(
            0.25,
            min(0.90, float(minimum_ground_image_fraction)),
        )
        self.maximum_local_inverse_depth_cv = max(
            0.05,
            float(maximum_local_inverse_depth_cv),
        )

    def anchors(
        self,
        inverse_depth: np.ndarray,
        *,
        camera_position_ned_m: Sequence[float],
        camera_quaternion_xyzw: Sequence[float],
        ground_down_m: float,
        excluded_bbox_xywh: Sequence[float] | None = None,
    ) -> GroundAnchors:
        height, width = inverse_depth.shape[:2]
        camera = np.asarray(camera_position_ned_m, dtype=np.float64)
        if camera.shape != (3,) or not np.all(np.isfinite(camera)):
            raise ValueError("camera position is invalid")
        xs = np.linspace(0.08 * width, 0.92 * width, self.grid_columns)
        ys = np.linspace(0.12 * height, 0.95 * height, self.grid_rows)
        excluded = (
            tuple(float(value) for value in excluded_bbox_xywh)
            if excluded_bbox_xywh is not None
            else None
        )
        q_values: list[float] = []
        metric_depths: list[float] = []
        pixels: list[tuple[float, float]] = []
        rejection_counts: dict[str, int] = {}
        candidate_count = 0
        diagnostics: list[AnchorDiagnostic] = []

        def reject(reason: str) -> None:
            rejection_counts[reason] = rejection_counts.get(reason, 0) + 1

        for row_index, v in enumerate(ys):
            for column_index, u in enumerate(xs):
                grid_index = row_index * self.grid_columns + column_index
                if excluded is not None:
                    x, y, box_width, box_height = excluded
                    margin_x = 0.08 * box_width
                    margin_y = 0.08 * box_height
                    if (
                        x - margin_x <= u <= x + box_width + margin_x
                        and y - margin_y <= v <= y + box_height + margin_y
                    ):
                        reject("excluded_target_bbox")
                        diagnostics.append(
                            AnchorDiagnostic(
                                grid_index,
                                float(u),
                                float(v),
                                False,
                                "excluded_target_bbox",
                                target_bbox_exclusion_result=(
                                    "inside_excluded_target_bbox_with_margin"
                                ),
                            )
                        )
                        continue
                exclusion_result = (
                    "outside_excluded_target_bbox_with_margin"
                    if excluded is not None
                    else "no_target_bbox_provided"
                )
                ray = np.asarray(
                    self.projector.pixel_to_ned_ray(
                        float(u),
                        float(v),
                        width,
                        height,
                        camera_quaternion_xyzw,
                    ),
                    dtype=np.float64,
                )
                if ray[2] <= 1e-4:
                    reject("ray_not_ground_facing")
                    diagnostics.append(
                        AnchorDiagnostic(
                            grid_index,
                            float(u),
                            float(v),
                            False,
                            "ray_not_ground_facing",
                            ray_ned_unit=tuple(float(value) for value in ray),
                            target_bbox_exclusion_result=exclusion_result,
                        )
                    )
                    continue
                ray_range = (float(ground_down_m) - camera[2]) / ray[2]
                if not self.minimum_range_m <= ray_range <= self.maximum_range_m:
                    reject("ground_range_outside_limits")
                    diagnostics.append(
                        AnchorDiagnostic(
                            grid_index,
                            float(u),
                            float(v),
                            False,
                            "ground_range_outside_limits",
                            ray_ned_unit=tuple(float(value) for value in ray),
                            ground_slant_range_m=float(ray_range),
                            target_bbox_exclusion_result=exclusion_result,
                        )
                    )
                    continue
                candidate_count += 1
                # A downward ray is necessary but not sufficient evidence of
                # ground. Without a semantic mask, accept only the conservative
                # lower image region and locally coherent inverse depth. This
                # rejects sky/upper façades and isolated object pixels.
                if float(v) < self.minimum_ground_image_fraction * height:
                    reject("outside_conservative_ground_region")
                    diagnostics.append(
                        AnchorDiagnostic(
                            grid_index,
                            float(u),
                            float(v),
                            False,
                            "outside_conservative_ground_region",
                            ray_ned_unit=tuple(float(value) for value in ray),
                            ground_slant_range_m=float(ray_range),
                            target_bbox_exclusion_result=exclusion_result,
                        )
                    )
                    continue
                normalized_x = (float(u) - width / 2.0) / self.projector.fx
                normalized_y = (float(v) - height / 2.0) / self.projector.fy
                sensor_forward_component = 1.0 / np.sqrt(
                    1.0 + normalized_x**2 + normalized_y**2
                )
                optical_depth = ray_range * sensor_forward_component
                ui = max(0, min(width - 1, int(round(u))))
                vi = max(0, min(height - 1, int(round(v))))
                relative = float(inverse_depth[vi, ui])
                local = np.asarray(
                    inverse_depth[
                        max(0, vi - 1):min(height, vi + 2),
                        max(0, ui - 1):min(width, ui + 2),
                    ],
                    dtype=np.float64,
                )
                local = local[np.isfinite(local) & (local > 0.0)]
                local_mean = (
                    None if local.size == 0 else float(np.mean(local))
                )
                local_std = (
                    None if local.size == 0 else float(np.std(local))
                )
                local_cv = (
                    float("inf")
                    if local.size < 4
                    else float(local_std / max(1e-9, local_mean))
                )
                accepted = bool(
                    np.isfinite(relative)
                    and relative > 0.0
                    and np.isfinite(optical_depth)
                    and optical_depth > 0.0
                    and local_cv <= self.maximum_local_inverse_depth_cv
                )
                if accepted:
                    q_values.append(relative)
                    metric_depths.append(float(optical_depth))
                    pixels.append((float(u), float(v)))
                    reason = "accepted_ground_anchor"
                elif not np.isfinite(relative) or relative <= 0.0:
                    reason = "invalid_relative_inverse_depth"
                    reject(reason)
                elif not np.isfinite(optical_depth) or optical_depth <= 0.0:
                    reason = "invalid_metric_ground_depth"
                    reject(reason)
                else:
                    reason = "local_inverse_depth_incoherent"
                    reject(reason)
                diagnostics.append(
                    AnchorDiagnostic(
                        grid_index,
                        float(u),
                        float(v),
                        accepted,
                        reason,
                        ray_ned_unit=tuple(float(value) for value in ray),
                        ground_slant_range_m=float(ray_range),
                        metric_optical_depth_m=float(optical_depth),
                        relative_inverse_depth=(
                            float(relative) if np.isfinite(relative) else None
                        ),
                        normalized_image_x=float(normalized_x),
                        normalized_image_y=float(normalized_y),
                        sensor_forward_component=float(sensor_forward_component),
                        local_sample_count=int(local.size),
                        local_inverse_depth_mean=local_mean,
                        local_inverse_depth_std=local_std,
                        local_inverse_depth_cv=(
                            float(local_cv) if np.isfinite(local_cv) else None
                        ),
                        target_bbox_exclusion_result=exclusion_result,
                        contribution_to_fit=(
                            "included_in_affine_fit"
                            if accepted
                            else "not_in_fit"
                        ),
                    )
                )
        q_array = np.asarray(q_values, dtype=np.float64)
        depth_array = np.asarray(metric_depths, dtype=np.float64)
        pixel_array = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
        quantile_levels = (0.0, 0.05, 0.25, 0.5, 0.75, 0.95, 1.0)
        q_quantiles = (
            tuple(float(value) for value in np.quantile(q_array, quantile_levels))
            if q_array.size
            else ()
        )
        depth_quantiles = (
            tuple(
                float(value)
                for value in np.quantile(depth_array, quantile_levels)
            )
            if depth_array.size
            else ()
        )
        occupied_bins: set[tuple[int, int]] = set()
        spatial_columns = 4
        spatial_rows = 3
        for u, v in pixels:
            column = min(
                spatial_columns - 1,
                max(0, int(spatial_columns * u / max(1.0, float(width)))),
            )
            row = min(
                spatial_rows - 1,
                max(0, int(spatial_rows * v / max(1.0, float(height)))),
            )
            occupied_bins.add((column, row))
        return GroundAnchors(
            relative_inverse_depth=q_array,
            metric_optical_depth_m=depth_array,
            pixel_uv=pixel_array,
            candidate_count=candidate_count,
            accepted_ground_anchor_count=len(q_values),
            rejected_reason_counts=tuple(sorted(rejection_counts.items())),
            relative_inverse_depth_quantiles=q_quantiles,
            metric_depth_quantiles_m=depth_quantiles,
            spatial_bin_count=len(occupied_bins),
            spatial_coverage_fraction=(
                len(occupied_bins) / float(spatial_columns * spatial_rows)
            ),
            diagnostics=tuple(diagnostics),
        )
