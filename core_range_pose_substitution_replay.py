"""Offline replay that swaps the PX4 camera altitude for the simulator truth.

Ground anchors get their metric depth from `(ground_down_m - camera_z) / ray_down`
(`m52_adapter.py`), so the calibration scale -- and with it every reported range --
is proportional to the camera altitude the pipeline believes. That altitude comes
from `extrinsics.camera_position_ned_m`, the PX4 estimate, which wandered between
0.375 m and 2.833 m across the 2026-08-06 corpus while the simulator held the
observer at 1.0 m.

This replay re-fits calibration from the recorded anchors with the true altitude
substituted and re-derives the range, to establish whether the association between
altitude error and range error is causal. It reads artifacts only and writes
nothing back into them.

Substituting simulator truth is a diagnostic, not a fix: real flight has no such
ground truth to substitute.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from statistics import median
from typing import Any

import numpy as np

from metric_depth_calibrator import MetricDepthCalibrator


def raw_rows(root: Path) -> list[dict[str, Any]]:
    rows = []
    for path in sorted(root.glob("physical_diagnostics*.jsonl")):
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(row, dict) and row.get("stage") == "raw_range_computed":
                rows.append(row)
    rows.sort(key=lambda row: int(row.get("frame_index", 0)))
    return rows


def true_camera_altitude(row: dict[str, Any]) -> float | None:
    diagnostics = (row.get("ground_truth") or {}).get("provider_diagnostics") or {}
    position = diagnostics.get("camera_position_xyz")
    if not isinstance(position, (list, tuple)) or len(position) != 3:
        return None
    return float(position[2])


def anchor_arrays(
    row: dict[str, Any],
    altitude_m: float,
    ground_down_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Rebuild the calibrator inputs for one frame at a chosen camera altitude.

    Only the anchors the adapter already accepted are reused. Re-deciding
    acceptance is impossible offline because rays rejected before sampling carry
    no `relative_inverse_depth`, so this isolates the scale effect rather than
    reproducing the adapter end to end.
    """

    inverse_depths: list[float] = []
    metric_depths: list[float] = []
    for point in (row.get("anchors") or {}).get("per_grid_point") or []:
        if not point.get("accepted"):
            continue
        ray = point.get("ray_ned_unit")
        forward = point.get("sensor_forward_component")
        relative = point.get("relative_inverse_depth")
        if not ray or forward is None or relative is None:
            continue
        ray_down = float(ray[2])
        if abs(ray_down) < 1e-9:
            continue
        ray_range = (ground_down_m + altitude_m) / ray_down
        if not math.isfinite(ray_range) or ray_range <= 0.0:
            continue
        inverse_depths.append(float(relative))
        metric_depths.append(ray_range * float(forward))
    return np.asarray(inverse_depths, dtype=np.float64), np.asarray(
        metric_depths, dtype=np.float64
    )


def target_range(row: dict[str, Any], scale: float, offset: float) -> float | None:
    measured = (row.get("inverse_depth_filter") or {}).get("filtered_inverse_depth")
    raw_range = row.get("raw_range") or {}
    image_x = raw_range.get("image_ray_x")
    image_y = raw_range.get("image_ray_y")
    if measured is None or image_x is None or image_y is None:
        return None
    inverse_metric = scale * float(measured) + offset
    if not math.isfinite(inverse_metric) or inverse_metric <= 0.0:
        return None
    optical_depth = 1.0 / inverse_metric
    return optical_depth * math.sqrt(1.0 + float(image_x) ** 2 + float(image_y) ** 2)


def replay(root: Path, use_true_altitude: bool) -> dict[str, Any]:
    calibrator = MetricDepthCalibrator()
    errors: list[float] = []
    ratios: list[float] = []
    produced = 0
    skipped = 0
    for row in raw_rows(root):
        ground_truth = row.get("ground_truth") or {}
        distance = ground_truth.get("distance_m")
        config = ((row.get("anchors") or {}).get("adapter_config")) or {}
        ground_down_m = float(config.get("ground_down_m", 0.0))
        if use_true_altitude:
            altitude = true_camera_altitude(row)
        else:
            position = (row.get("extrinsics") or {}).get("camera_position_ned_m")
            altitude = -float(position[2]) if position else None
        if altitude is None or distance is None:
            skipped += 1
            continue
        inverse_depths, metric_depths = anchor_arrays(row, altitude, ground_down_m)
        if inverse_depths.size == 0:
            skipped += 1
            continue
        fit = calibrator.fit(inverse_depths, metric_depths)
        if not fit.valid or fit.scale is None or fit.offset is None:
            skipped += 1
            continue
        estimate = target_range(row, fit.scale, fit.offset)
        if estimate is None:
            skipped += 1
            continue
        produced += 1
        errors.append(estimate - float(distance))
        ratios.append(estimate / float(distance))
    if not errors:
        return {"frames": 0, "skipped": skipped}
    absolute = [abs(value) for value in errors]
    return {
        "frames": produced,
        "skipped": skipped,
        "signed_bias_m": median(errors),
        "mae_m": sum(absolute) / len(absolute),
        "median_abs_error_m": median(absolute),
        "range_ratio_median": median(ratios),
    }


def recorded_baseline(root: Path) -> dict[str, Any]:
    """What the pipeline actually reported, for replay-fidelity checking."""

    errors: list[float] = []
    ratios: list[float] = []
    for row in raw_rows(root):
        distance = (row.get("ground_truth") or {}).get("distance_m")
        reported = (row.get("raw_range") or {}).get("physics_slant_range_m")
        if distance is None or reported is None:
            continue
        errors.append(float(reported) - float(distance))
        ratios.append(float(reported) / float(distance))
    if not errors:
        return {"frames": 0}
    absolute = [abs(value) for value in errors]
    return {
        "frames": len(errors),
        "signed_bias_m": median(errors),
        "mae_m": sum(absolute) / len(absolute),
        "median_abs_error_m": median(absolute),
        "range_ratio_median": median(ratios),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("roots", nargs="+", type=Path)
    parser.add_argument("--output-json", type=Path)
    arguments = parser.parse_args()

    results = []
    for root in arguments.roots:
        if not (root / "physical_diagnostics.jsonl").exists():
            continue
        results.append(
            {
                "root": str(root),
                "recorded": recorded_baseline(root),
                "replay_px4_altitude": replay(root, use_true_altitude=False),
                "replay_true_altitude": replay(root, use_true_altitude=True),
            }
        )

    print(f"{'run':38s} {'recorded':>10s} {'replay_px4':>11s} {'replay_true':>12s}   (median |error| m)")
    for result in results:
        name = Path(result["root"]).name[:38]
        recorded = result["recorded"].get("median_abs_error_m")
        px4 = result["replay_px4_altitude"].get("median_abs_error_m")
        true = result["replay_true_altitude"].get("median_abs_error_m")
        print(
            f"{name:38s} "
            f"{'n/a' if recorded is None else f'{recorded:10.3f}'} "
            f"{'n/a' if px4 is None else f'{px4:11.3f}'} "
            f"{'n/a' if true is None else f'{true:12.3f}'}"
        )

    usable = [
        result
        for result in results
        if result["replay_px4_altitude"].get("median_abs_error_m") is not None
        and result["replay_true_altitude"].get("median_abs_error_m") is not None
    ]
    if usable:
        px4_errors = [r["replay_px4_altitude"]["median_abs_error_m"] for r in usable]
        true_errors = [r["replay_true_altitude"]["median_abs_error_m"] for r in usable]
        improved = sum(1 for a, b in zip(px4_errors, true_errors) if b < a)
        print(
            f"\nruns={len(usable)}  median |error|: "
            f"PX4 altitude {median(px4_errors):.3f} m -> true altitude {median(true_errors):.3f} m  "
            f"({improved}/{len(usable)} runs improved)"
        )

    if arguments.output_json:
        arguments.output_json.write_text(
            json.dumps(results, indent=2, sort_keys=True), encoding="utf-8"
        )


if __name__ == "__main__":
    main()
