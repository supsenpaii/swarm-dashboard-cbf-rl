"""Inference for the Stage 2A learned relative-range model.

Trained by `core_range_stage2a_relative_range_model.py` on the 2026-08-07
Stage 2A corpus (12 scenario groups, 1551 frames). Group-disjoint
cross-validated MAE was 0.88 m in 3-12 m using only size-agnostic features
(no bounding-box size, so it does not encode this corpus's one tracked
object's physical dimensions) -- see
artifacts/core_range_3_12m/stage2a_relative_range_model/evaluation.json.

This module only loads the model and runs inference; it does not decide
whether the live pipeline should use it (that switch lives in
metric_target_fusion.py, opt-in via SWARM_METRIC_TARGET_RANGE_MODEL_ENABLED).
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

MODEL_DIR = Path(__file__).resolve().parent / "artifacts/core_range_3_12m/stage2a_relative_range_model"
DISTANCE_BOUNDS = (3.0, 12.0)

# Cross-validated RMSE for the size_agnostic model (evaluation.json ->
# size_agnostic.xgboost_oof.rmse_m). Used as a conservative, fixed
# uncertainty estimate since the point-prediction model has no native
# per-sample variance. Read once at import time so a re-trained model's
# evaluation.json is picked up without editing this file.
try:
    _EVALUATION = json.loads((MODEL_DIR / "evaluation.json").read_text())
    SIZE_AGNOSTIC_RANGE_STD_M = float(
        _EVALUATION["size_agnostic"]["xgboost_oof"]["rmse_m"]
    )
except (FileNotFoundError, KeyError, ValueError):
    SIZE_AGNOSTIC_RANGE_STD_M = 1.5

# Three of twelve training groups have negative within-scenario slope (the
# model reports range moving opposite to the true trend) -- confirmed and
# left unfixed after four independent attempts (temporal smoothing, a
# rank:pairwise objective, cross-checking against physics_slant_range_m; see
# evaluation.json's "known_risk_2026_08_07" and the session transcript).
# All three affected groups share |lateral_offset_m| = 0.75, which shows up
# at inference as a target well off the camera's boresight. Frames in that
# geometry aren't guaranteed wrong, but the corpus that would prove they're
# fine doesn't exist -- inflating uncertainty here is honest under-confidence,
# not a fix.
KNOWN_RISK_IMAGE_RAY_ABS_THRESHOLD = 0.13
KNOWN_RISK_RANGE_STD_MULTIPLIER = 2.5

_model = None
_feature_names: tuple[str, ...] | None = None
_medians: dict[str, float] | None = None


def _load(name: str = "size_agnostic") -> None:
    global _model, _feature_names, _medians
    if _model is not None:
        return
    import xgboost as xgb

    booster = xgb.XGBRegressor()
    booster.load_model(str(MODEL_DIR / f"{name}.xgb.json"))
    spec = json.loads((MODEL_DIR / f"{name}.medians.json").read_text())
    _model = booster
    _feature_names = tuple(spec["feature_names"])
    _medians = {k: float(v) for k, v in spec["medians"].items()}


def feature_names() -> tuple[str, ...]:
    """Feature names in the exact order the loaded model expects."""
    _load()
    assert _feature_names is not None
    return _feature_names


def is_known_risk_geometry(features: dict[str, float | None]) -> bool:
    """Flag frames matching the geometry of the three unfixed negative-slope
    groups (large horizontal target bearing off boresight). See the module
    docstring constants above for what this does and does not mean.

    Only the horizontal component (`image_ray_x`) separates the affected
    groups from the rest of the corpus. `image_ray_y` sits around 0.18-0.20
    in every scenario regardless of risk, driven by the fixed downward
    gimbal pitch used throughout Stage 2A -- including it made this fire on
    100% of frames in every group, not just the three at risk.
    """

    image_ray_x = features.get("image_ray_x")
    return (
        image_ray_x is not None
        and abs(float(image_ray_x)) >= KNOWN_RISK_IMAGE_RAY_ABS_THRESHOLD
    )


def predict_range_m(features: dict[str, float | None]) -> float | None:
    """Predict target range in metres from a size-agnostic feature dict.

    `features` must use the same keys as `feature_names()`; missing or None
    values are median-imputed exactly as during training. Returns None only
    if the model artifacts are unavailable.
    """
    try:
        _load()
    except Exception:
        return None
    assert _model is not None and _feature_names is not None and _medians is not None

    raw = np.asarray(
        [
            np.nan if features.get(name) is None else float(features[name])  # type: ignore[arg-type]
            for name in _feature_names
        ],
        dtype=np.float64,
    ).reshape(1, -1)
    missing = np.isnan(raw)
    filled = raw.copy()
    for index, name in enumerate(_feature_names):
        if missing[0, index]:
            filled[0, index] = _medians[name]
    row = np.hstack([filled, missing.astype(np.float64)])
    prediction = float(_model.predict(row)[0])
    return float(np.clip(prediction, *DISTANCE_BOUNDS))
