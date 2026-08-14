from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from metric_depth_calibrator import MetricDepthCalibrator

# Collection-only ROI shape variants for offline recovery-policy comparison.
# "scale" variants rescale width/height around the bbox center; "erosion"
# variants inset each edge by a fixed fraction of that edge's own dimension.
# Kept separate from the erosion_fraction actually used to drive raw range.
_ROI_SHAPE_VARIANTS: tuple[tuple[str, float | None, float | None], ...] = (
    ("A_original", 1.0, None),
    ("B_centered_90", 0.90, None),
    ("C_centered_80", 0.80, None),
    ("D_centered_70", 0.70, None),
    ("E_erosion_10", None, 0.10),
    ("F_erosion_20", None, 0.20),
)


@dataclass(frozen=True)
class TargetDepth:
    valid: bool
    reason: str
    optical_depth_m: float | None = None
    optical_depth_std_m: float | None = None
    relative_inverse_depth: float | None = None
    relative_inverse_depth_std: float | None = None
    valid_fraction: float = 0.0
    sample_count: int = 0
    roi_inverse_depth_quantiles: tuple[float, ...] = ()
    selected_foreground_statistic: str | None = None
    variant_diagnostics: dict | None = None


class TargetDepthExtractor:
    """Robust foreground depth from an eroded bbox, never a center pixel."""

    def __init__(
        self,
        *,
        erosion_fraction: float = 0.18,
        minimum_samples: int = 24,
        minimum_valid_fraction: float = 0.25,
        enable_variant_diagnostics: bool | None = None,
        recovery_policy: str | None = None,
    ) -> None:
        self.erosion_fraction = max(0.0, min(0.4, erosion_fraction))
        self.minimum_samples = max(4, int(minimum_samples))
        self.minimum_valid_fraction = max(
            0.05,
            min(1.0, minimum_valid_fraction),
        )
        self.enable_variant_diagnostics = (
            bool(enable_variant_diagnostics)
            if enable_variant_diagnostics is not None
            else os.environ.get(
                "SWARM_TARGET_ROI_VARIANT_DIAGNOSTICS", ""
            ).strip().lower()
            in ("1", "true", "yes", "on")
        )
        # Collection-development-only recovery policy, default off. Chosen
        # from artifacts/core_range_3_12m/dynamic_repair_and_replay/recovery_policy.json.
        # Never changes bbox/erosion geometry, calibration, the temporal
        # filter or its inverse_depth_relative_gate — only which pixels
        # within the existing ROI are used for the foreground statistic.
        self.recovery_policy = (
            recovery_policy
            if recovery_policy is not None
            else os.environ.get("SWARM_TARGET_ROI_RECOVERY_POLICY", "").strip()
        ) or None
        if self.recovery_policy not in (None, "central_quantile_region"):
            raise ValueError(
                f"unknown recovery_policy: {self.recovery_policy!r}"
            )

    def extract(
        self,
        inverse_depth: np.ndarray,
        bbox_xywh: Sequence[float],
        calibrator: MetricDepthCalibrator,
    ) -> TargetDepth:
        height, width = inverse_depth.shape[:2]
        x, y, box_width, box_height = (
            float(value) for value in bbox_xywh
        )
        inset_x = box_width * self.erosion_fraction
        inset_y = box_height * self.erosion_fraction
        x0 = max(0, min(width - 1, int(round(x + inset_x))))
        y0 = max(0, min(height - 1, int(round(y + inset_y))))
        x1 = max(x0 + 1, min(width, int(round(x + box_width - inset_x))))
        y1 = max(y0 + 1, min(height, int(round(y + box_height - inset_y))))
        roi = np.asarray(inverse_depth[y0:y1, x0:x1], dtype=np.float64)
        total = int(roi.size)
        values = roi[np.isfinite(roi) & (roi > 0.0)]
        roi_quantiles = (
            tuple(
                float(value)
                for value in np.quantile(
                    values,
                    (0.0, 0.10, 0.25, 0.50, 0.75, 0.90, 1.0),
                )
            )
            if values.size
            else ()
        )
        valid_fraction = float(values.size / max(1, total))
        variant_diagnostics = (
            self._compute_variant_diagnostics(
                inverse_depth, (x, y, box_width, box_height)
            )
            if self.enable_variant_diagnostics
            else None
        )
        if (
            values.size < self.minimum_samples
            or valid_fraction < self.minimum_valid_fraction
        ):
            return TargetDepth(
                False,
                "sparse_target_roi",
                valid_fraction=valid_fraction,
                sample_count=int(values.size),
                roi_inverse_depth_quantiles=roi_quantiles,
                selected_foreground_statistic=self._statistic_label(),
                variant_diagnostics=variant_diagnostics,
            )
        foreground, statistic_name = self._select_foreground(values)
        relative = float(np.median(foreground))
        depths = calibrator.metric_depth(foreground)
        depths = depths[np.isfinite(depths) & (depths > 0.0)]
        if depths.size < max(3, self.minimum_samples // 4):
            return TargetDepth(
                False,
                "target_metric_depth_invalid",
                relative_inverse_depth=relative,
                relative_inverse_depth_std=max(
                    1e-6,
                    1.4826 * float(
                        np.median(np.abs(foreground - relative))
                    ),
                ),
                valid_fraction=valid_fraction,
                sample_count=int(depths.size),
                roi_inverse_depth_quantiles=roi_quantiles,
                selected_foreground_statistic=statistic_name,
                variant_diagnostics=variant_diagnostics,
            )
        optical_depth = float(np.median(depths))
        depth_mad = float(np.median(np.abs(depths - optical_depth)))
        depth_std = max(0.10, 1.4826 * depth_mad)
        inverse_mad = float(np.median(np.abs(foreground - relative)))
        inverse_std = max(1e-6, 1.4826 * inverse_mad)
        return TargetDepth(
            True,
            "ok",
            optical_depth,
            depth_std,
            relative,
            inverse_std,
            valid_fraction,
            int(depths.size),
            roi_quantiles,
            statistic_name,
            variant_diagnostics,
        )

    def _statistic_label(self) -> str:
        if self.recovery_policy == "central_quantile_region":
            return "central_iqr_band_median_recovery_policy"
        return "median_of_median_or_mad_gated_foreground_half"

    def _select_foreground(
        self, values: np.ndarray
    ) -> tuple[np.ndarray, str]:
        """Select the pixel subset used for the target depth statistic.

        Default (recovery_policy=None): unchanged production behaviour —
        MAD-gated near foreground half. Opt-in "central_quantile_region":
        the recovery policy chosen in recovery_policy.json, which fixed
        the 0% raw-range availability observed at near-start lateral-left
        receding without touching bbox geometry or the temporal gate.
        """
        if self.recovery_policy == "central_quantile_region":
            if values.size == 0:
                return values, self._statistic_label()
            q25, q75 = np.quantile(values, (0.25, 0.75))
            band = values[(values >= q25) & (values <= q75)]
            if band.size < self.minimum_samples // 2:
                band = values
            return band, self._statistic_label()
        median = float(np.median(values))
        foreground = values[values >= median]
        center = float(np.median(foreground))
        mad = float(np.median(np.abs(foreground - center)))
        if mad > 1e-9:
            foreground = foreground[
                np.abs(foreground - center) <= 3.5 * 1.4826 * mad
            ]
        if foreground.size < self.minimum_samples // 2:
            foreground = values
        return foreground, self._statistic_label()

    def _compute_variant_diagnostics(
        self,
        inverse_depth: np.ndarray,
        bbox_xywh: Sequence[float],
    ) -> dict:
        """Collection-only, deterministic ROI-shape comparison (opt-in).

        Computes robust median/MAD-based stats for each candidate ROI shape
        (A-F) plus the two foreground-selection statistics currently in use
        (G: MAD-gated foreground half, at this extractor's erosion_fraction;
        H: central IQR-band statistic, at the same bbox). Never uses GT range
        or model prediction; never affects raw range, filtering or the
        returned TargetDepth fields above when this method isn't called.
        """
        height, width = inverse_depth.shape[:2]
        x, y, box_width, box_height = (float(v) for v in bbox_xywh)
        results: dict[str, dict] = {}

        def _bounds(scale: float | None, erosion: float | None) -> tuple[int, int, int, int]:
            if scale is not None:
                new_w = box_width * scale
                new_h = box_height * scale
                cx = x + box_width / 2.0
                cy = y + box_height / 2.0
                bx, by, bw, bh = cx - new_w / 2.0, cy - new_h / 2.0, new_w, new_h
            else:
                inset_x = box_width * float(erosion)
                inset_y = box_height * float(erosion)
                bx, by = x + inset_x, y + inset_y
                bw, bh = box_width - 2 * inset_x, box_height - 2 * inset_y
            x0 = max(0, min(width - 1, int(round(bx))))
            y0 = max(0, min(height - 1, int(round(by))))
            x1 = max(x0 + 1, min(width, int(round(bx + bw))))
            y1 = max(y0 + 1, min(height, int(round(by + bh))))
            return x0, y0, x1, y1

        def _stats(values: np.ndarray, total: int) -> dict:
            valid_fraction = float(values.size / max(1, total))
            if values.size == 0:
                return {
                    "finite_pixel_count": 0,
                    "valid_fraction": valid_fraction,
                    "median": None,
                    "mad_std": None,
                    "uncertainty_ratio": None,
                    "nonfinite_result": True,
                }
            median = float(np.median(values))
            mad_std = max(1e-6, 1.4826 * float(np.median(np.abs(values - median))))
            ratio = (mad_std / median) if median > 0.0 else None
            return {
                "finite_pixel_count": int(values.size),
                "valid_fraction": valid_fraction,
                "median": median,
                "mad_std": mad_std,
                "uncertainty_ratio": ratio,
                "nonfinite_result": not (
                    np.isfinite(median) and np.isfinite(mad_std)
                ),
            }

        for name, scale, erosion in _ROI_SHAPE_VARIANTS:
            x0, y0, x1, y1 = _bounds(scale, erosion)
            roi = np.asarray(inverse_depth[y0:y1, x0:x1], dtype=np.float64)
            values = roi[np.isfinite(roi) & (roi > 0.0)]
            entry = _stats(values, int(roi.size))
            entry["bbox_px"] = [x0, y0, x1 - x0, y1 - y0]
            results[name] = entry

        # G: current production foreground-half + MAD gate, at this
        # extractor's configured erosion_fraction bbox.
        x0, y0, x1, y1 = _bounds(None, self.erosion_fraction)
        roi = np.asarray(inverse_depth[y0:y1, x0:x1], dtype=np.float64)
        values = roi[np.isfinite(roi) & (roi > 0.0)]
        if values.size:
            median = float(np.median(values))
            foreground = values[values >= median]
            center = float(np.median(foreground))
            mad = float(np.median(np.abs(foreground - center)))
            if mad > 1e-9:
                foreground = foreground[
                    np.abs(foreground - center) <= 3.5 * 1.4826 * mad
                ]
            g_stats = _stats(foreground, int(roi.size))
        else:
            g_stats = _stats(values, int(roi.size))
        g_stats["bbox_px"] = [x0, y0, x1 - x0, y1 - y0]
        results["G_foreground_half_current"] = g_stats

        # H: central IQR-band statistic at the same default bbox.
        if values.size:
            q25, q75 = np.quantile(values, (0.25, 0.75))
            band = values[(values >= q25) & (values <= q75)]
            h_stats = _stats(band, int(roi.size))
        else:
            h_stats = _stats(values, int(roi.size))
        h_stats["bbox_px"] = [x0, y0, x1 - x0, y1 - y0]
        results["H_central_quantile_region"] = h_stats

        return results
