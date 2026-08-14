"""Train a distance-from-visual-cues regressor on the 2026-08-07 Stage 2A corpus.

Two feature sets are trained and compared honestly on group-disjoint holdout:

- `size_agnostic`: MiDaS relative-depth statistics, M52 ground-anchor
  calibration outputs, target ray geometry, and the current pipeline's own
  `physics_slant_range_m` as a prior. None of these depend on the tracked
  object having a known physical size, so a model trained on this set should
  transfer to objects other than the one in this corpus.
- `with_bbox`: adds bounding-box pixel size/aspect. This feature carries by
  far the strongest distance signal in this corpus (see
  docs/CORE_RANGE_STAGE_2A_FAILURE_AUDIT_20260806.md section 9), but it
  implicitly encodes this specific target's physical size. A model trained
  on it will not generalise to differently sized objects; it is reported
  separately, not blended into the size-agnostic result.

Evaluation is 4-fold group-disjoint cross-validation (GroupKFold on
`group_id`), which is the only way twelve captured scenarios can give an
honest read on generalisation across unseen geometry -- a random per-frame
split would leak the same scenario into train and test and repeat the
overfitting failure mode already documented in the frozen XGBoost residual
candidates under artifacts/range_residual_xgboost_candidate_*/.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np
import xgboost as xgb
from sklearn.linear_model import Ridge
from sklearn.model_selection import GroupKFold

WORKSPACE = Path(__file__).resolve().parent
CORPUS = WORKSPACE / "artifacts/core_range_3_12m/targeted_dynamic_pilot_stage2a_20260807"
OUT = WORKSPACE / "artifacts/core_range_3_12m/stage2a_relative_range_model"
SEED = 52
DISTANCE_BOUNDS = (3.0, 12.0)

SIZE_AGNOSTIC_FEATURES = (
    "target_raw_inverse_depth", "target_raw_inverse_depth_std",
    "target_roi_optical_depth_m", "target_roi_optical_depth_std_m",
    "target_valid_fraction", "target_sample_count",
    "filtered_inverse_depth", "inverse_depth_std",
    "calibration_scale", "calibration_offset", "calibration_anchor_count",
    "calibration_inlier_count", "calibration_residual_m_inv",
    "calibration_condition_number", "calibration_stable",
    "calibration_parameter_uncertainty", "anchor_inverse_depth_span",
    "image_ray_x", "image_ray_y", "camera_altitude_px4_m",
    "gimbal_pitch_deg", "physics_slant_range_m",
)
BBOX_FEATURES = (
    "bbox_width_px", "bbox_height_px", "bbox_area_px2",
    "bbox_scale_px", "bbox_aspect_ratio", "tracking_score",
)


def load_group_spec(root: Path) -> dict[str, float]:
    plan = json.loads((root / "collection_plan_pilot.json").read_text())
    return {s["group_id"]: float(s["gimbal_pitch_deg"]) for s in plan["sessions"]}


def extract_rows(root: Path) -> list[dict]:
    pitch_by_group = load_group_spec(root)
    rows: list[dict] = []
    for session_dir in sorted((root / "raw_sessions").glob("*/")):
        diag = session_dir / "physical_diagnostics.jsonl"
        if not diag.exists():
            continue
        scenario_group = re.sub(r"_attempt_\d+$", "", session_dir.name)
        for line in diag.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            if record.get("stage") != "raw_range_computed":
                continue
            gt = (record.get("ground_truth") or {}).get("distance_m")
            td = record.get("target_depth") or {}
            idf = record.get("inverse_depth_filter") or {}
            cal = (record.get("calibration") or {}).get("applied") or {}
            bbox = (record.get("bbox") or {}).get("xywh_px")
            raw_range = record.get("raw_range") or {}
            extrinsics = record.get("extrinsics") or {}
            position = extrinsics.get("camera_position_ned_m")
            quantiles = cal.get("anchor_inverse_depth_quantiles") or []
            if gt is None or not bbox or position is None:
                continue
            width, height = float(bbox[2]), float(bbox[3])
            features = {
                "target_raw_inverse_depth": td.get("raw_relative_inverse_depth"),
                "target_raw_inverse_depth_std": td.get("raw_relative_inverse_depth_std"),
                "target_roi_optical_depth_m": td.get("raw_roi_optical_depth_m"),
                "target_roi_optical_depth_std_m": td.get("raw_roi_optical_depth_std_m"),
                "target_valid_fraction": td.get("valid_fraction"),
                "target_sample_count": td.get("sample_count"),
                "filtered_inverse_depth": idf.get("filtered_inverse_depth"),
                "inverse_depth_std": idf.get("inverse_depth_std"),
                "calibration_scale": cal.get("filtered_scale"),
                "calibration_offset": cal.get("filtered_offset"),
                "calibration_anchor_count": cal.get("anchor_count"),
                "calibration_inlier_count": cal.get("inlier_count"),
                "calibration_residual_m_inv": cal.get("residual_m_inv"),
                "calibration_condition_number": cal.get("condition_number"),
                "calibration_stable": 1.0 if cal.get("stable") else 0.0,
                "calibration_parameter_uncertainty": cal.get("parameter_uncertainty"),
                "anchor_inverse_depth_span": (
                    (quantiles[5] - quantiles[1]) if len(quantiles) == 7 else None
                ),
                "image_ray_x": raw_range.get("image_ray_x"),
                "image_ray_y": raw_range.get("image_ray_y"),
                "camera_altitude_px4_m": -float(position[2]),
                "gimbal_pitch_deg": pitch_by_group.get(scenario_group),
                "physics_slant_range_m": raw_range.get("physics_slant_range_m"),
                "bbox_width_px": width,
                "bbox_height_px": height,
                "bbox_area_px2": width * height,
                "bbox_scale_px": (width * height) ** 0.5,
                "bbox_aspect_ratio": width / height if height > 1e-6 else None,
                "tracking_score": (record.get("bbox") or {}).get("tracking_score"),
            }
            rows.append({
                "group_id": scenario_group,
                "distance_m": float(gt),
                "features": features,
            })
    return rows


def build_matrix(
    rows: list[dict], feature_names: tuple[str, ...],
    medians: dict[str, float] | None = None,
) -> tuple[np.ndarray, dict[str, float]]:
    raw = np.asarray(
        [[np.nan if r["features"].get(n) is None else float(r["features"][n]) for n in feature_names]
         for r in rows],
        dtype=np.float64,
    )
    if medians is None:
        medians = {n: float(np.nanmedian(raw[:, i])) for i, n in enumerate(feature_names)}
    missing = np.isnan(raw)
    filled = raw.copy()
    for i, name in enumerate(feature_names):
        filled[np.isnan(filled[:, i]), i] = medians[name]
    return np.hstack([filled, missing.astype(np.float64)]), medians


def lateral_offset_weights(rows: list[dict], multiplier: float = 5.0) -> np.ndarray:
    """Upweight scenarios with large |lateral_offset_m| during training.

    Centred scenarios (lateral_offset ~= 0) dominate plain MSE training
    because their frame-level signal is more stable, which starves the
    lateral-offset scenarios of influence even though those are exactly the
    geometries with the known negative-slope (wrong-direction) failure mode.
    5x upweighting measurably shrank the wrongness in the three affected
    groups (~60% reduction in |slope| for the two worst) without hurting
    overall MAE -- see docs/CORE_RANGE_STAGE_2A_FAILURE_AUDIT_20260806.md
    section 10.6. It does not flip their slope positive; it is a partial
    mitigation, not a fix.
    """

    plan = json.loads((CORPUS / "collection_plan_pilot.json").read_text())
    lateral = {s["group_id"]: abs(float(s["lateral_offset_m"])) for s in plan["sessions"]}
    return np.asarray([
        multiplier if lateral.get(row["group_id"], 0.0) > 0.1 else 1.0
        for row in rows
    ])


def evaluate(rows: list[dict], feature_names: tuple[str, ...], label: str) -> dict:
    groups = np.asarray([r["group_id"] for r in rows])
    y = np.asarray([r["distance_m"] for r in rows], dtype=np.float64)
    n_splits = min(4, len(set(groups)))
    splitter = GroupKFold(n_splits=n_splits)
    oof_ridge = np.full(len(rows), np.nan)
    oof_xgb = np.full(len(rows), np.nan)
    for train_idx, test_idx in splitter.split(rows, y, groups):
        train_rows = [rows[i] for i in train_idx]
        test_rows = [rows[i] for i in test_idx]
        train_x, medians = build_matrix(train_rows, feature_names)
        test_x, _ = build_matrix(test_rows, feature_names, medians=medians)
        train_y = y[train_idx]
        sample_weight = lateral_offset_weights(train_rows)

        mean, std = train_x.mean(axis=0), train_x.std(axis=0)
        std[std < 1e-9] = 1.0
        ridge = Ridge(alpha=1.0)
        ridge.fit((train_x - mean) / std, train_y, sample_weight=sample_weight)
        oof_ridge[test_idx] = ridge.predict((test_x - mean) / std)

        booster = xgb.XGBRegressor(
            n_estimators=200, max_depth=4, learning_rate=0.05,
            subsample=0.8, colsample_bytree=0.8, reg_lambda=2.0,
            random_state=SEED,
        )
        booster.fit(train_x, train_y, sample_weight=sample_weight)
        oof_xgb[test_idx] = booster.predict(test_x)

    def metrics(pred: np.ndarray, truth: np.ndarray) -> dict:
        pred = np.clip(pred, *DISTANCE_BOUNDS)
        err = pred - truth
        return {
            "mae_m": float(np.mean(np.abs(err))),
            "bias_m": float(np.median(err)),
            "rmse_m": float(np.sqrt(np.mean(err ** 2))),
            "p95_abs_error_m": float(np.percentile(np.abs(err), 95)),
        }

    per_group_xgb = {}
    for gid in sorted(set(groups)):
        mask = groups == gid
        per_group_xgb[gid] = metrics(oof_xgb[mask], y[mask])

    print(f"\n=== {label} (n={len(rows)}, {len(feature_names)} features, "
          f"{n_splits}-fold group-disjoint) ===")
    for name, pred in (("ridge", oof_ridge), ("xgboost", oof_xgb)):
        m = metrics(pred, y)
        print(f"  {name:8s} MAE={m['mae_m']:.3f}m  bias={m['bias_m']:+.3f}m  "
              f"RMSE={m['rmse_m']:.3f}m  p95={m['p95_abs_error_m']:.3f}m")
    print("  xgboost per-group:")
    for gid, m in per_group_xgb.items():
        print(f"    {gid:38s} MAE={m['mae_m']:.3f}m  bias={m['bias_m']:+.3f}m  n={int((groups==gid).sum())}")

    return {
        "label": label, "n_rows": len(rows), "n_groups": len(set(groups)),
        "feature_names": list(feature_names),
        "ridge_oof": metrics(oof_ridge, y), "xgboost_oof": metrics(oof_xgb, y),
        "xgboost_per_group": per_group_xgb,
    }


def baseline_current_pipeline(rows: list[dict]) -> dict:
    y = np.asarray([r["distance_m"] for r in rows], dtype=np.float64)
    pred = np.asarray([r["features"]["physics_slant_range_m"] for r in rows], dtype=np.float64)
    valid = np.isfinite(pred)
    pred = np.clip(pred[valid], *DISTANCE_BOUNDS)
    err = pred - y[valid]
    return {
        "mae_m": float(np.mean(np.abs(err))), "bias_m": float(np.median(err)),
        "rmse_m": float(np.sqrt(np.mean(err ** 2))),
        "p95_abs_error_m": float(np.percentile(np.abs(err), 95)),
        "n": int(valid.sum()),
    }


def train_final_model(
    rows: list[dict], feature_names: tuple[str, ...], name: str,
) -> None:
    x, medians = build_matrix(rows, feature_names)
    y = np.asarray([r["distance_m"] for r in rows], dtype=np.float64)
    booster = xgb.XGBRegressor(
        n_estimators=200, max_depth=4, learning_rate=0.05,
        subsample=0.8, colsample_bytree=0.8, reg_lambda=2.0,
        random_state=SEED,
    )
    booster.fit(x, y, sample_weight=lateral_offset_weights(rows))
    OUT.mkdir(parents=True, exist_ok=True)
    booster.save_model(str(OUT / f"{name}.xgb.json"))
    (OUT / f"{name}.medians.json").write_text(
        json.dumps({"feature_names": list(feature_names), "medians": medians}, indent=2),
        encoding="utf-8",
    )


def main() -> None:
    rows = extract_rows(CORPUS)
    print(f"Loaded {len(rows)} rows across {len(set(r['group_id'] for r in rows))} groups")

    baseline = baseline_current_pipeline(rows)
    print(f"\n=== BASELINE: current physics_slant_range_m, no model (n={baseline['n']}) ===")
    print(f"  MAE={baseline['mae_m']:.3f}m  bias={baseline['bias_m']:+.3f}m  "
          f"RMSE={baseline['rmse_m']:.3f}m  p95={baseline['p95_abs_error_m']:.3f}m")

    result_size_agnostic = evaluate(rows, SIZE_AGNOSTIC_FEATURES, "size_agnostic")
    result_with_bbox = evaluate(rows, SIZE_AGNOSTIC_FEATURES + BBOX_FEATURES, "with_bbox (target-size-dependent)")

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "evaluation.json").write_text(
        json.dumps({
            "baseline_physics_slant_range": baseline,
            "size_agnostic": result_size_agnostic,
            "with_bbox": result_with_bbox,
        }, indent=2),
        encoding="utf-8",
    )
    train_final_model(rows, SIZE_AGNOSTIC_FEATURES, "size_agnostic")
    train_final_model(rows, SIZE_AGNOSTIC_FEATURES + BBOX_FEATURES, "with_bbox")
    print(f"\nSaved evaluation.json and model artifacts to {OUT}")


if __name__ == "__main__":
    main()
