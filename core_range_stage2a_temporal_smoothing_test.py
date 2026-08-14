"""Test whether temporal smoothing of out-of-fold predictions restores slope.

Two failure modes look identical in a single-frame slope measurement but
require opposite fixes:

- Noise around a correctly-scaled signal: smoothing over time reduces
  variance and should raise the within-group slope toward 1.0.
- Systematic shrinkage (the conditional mean itself is compressed toward
  the group mean): smoothing only removes zero-mean noise, so it cannot
  raise the slope -- the smoothed sequence stays shrunk, just less jagged.

This applies moving-average and EMA smoothers of several windows directly
to the already-computed out-of-fold predictions (same ones behind the 0.26
median within-group slope reported earlier) and re-measures slope, MAE and
RMSE per window. It does not retrain anything.
"""

from __future__ import annotations

import numpy as np

from core_range_stage2a_relative_range_model import DISTANCE_BOUNDS, extract_rows, CORPUS
from core_range_stage2a_slope_calibration import out_of_fold_predictions
from core_range_stage2a_relative_range_model import SIZE_AGNOSTIC_FEATURES


def moving_average(x: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return x.copy()
    kernel = np.ones(window) / window
    padded = np.concatenate([np.full(window - 1, x[0]), x])
    return np.convolve(padded, kernel, mode="valid")


def ema(x: np.ndarray, alpha: float) -> np.ndarray:
    out = np.empty_like(x)
    out[0] = x[0]
    for i in range(1, len(x)):
        out[i] = alpha * x[i] + (1.0 - alpha) * out[i - 1]
    return out


def slope(pred: np.ndarray, truth: np.ndarray) -> float:
    variance = float(np.sum((truth - truth.mean()) ** 2))
    if variance <= 1e-12:
        return float("nan")
    return float(np.sum((truth - truth.mean()) * (pred - pred.mean())) / variance)


def main() -> None:
    rows = extract_rows(CORPUS)
    oof, truth, groups = out_of_fold_predictions(rows, SIZE_AGNOSTIC_FEATURES)

    configs: list[tuple[str, callable]] = [
        ("raw (no smoothing)", lambda x: x),
        ("moving_avg w=5", lambda x: moving_average(x, 5)),
        ("moving_avg w=10", lambda x: moving_average(x, 10)),
        ("moving_avg w=20", lambda x: moving_average(x, 20)),
        ("moving_avg w=40", lambda x: moving_average(x, 40)),
        ("ema alpha=0.30", lambda x: ema(x, 0.30)),
        ("ema alpha=0.15", lambda x: ema(x, 0.15)),
        ("ema alpha=0.05", lambda x: ema(x, 0.05)),
    ]

    print(f"{'config':22s} {'median slope':>13s} {'MAE':>8s} {'RMSE':>8s}")
    print("-" * 55)
    for label, fn in configs:
        slopes, errs = [], []
        for gid in sorted(set(groups)):
            mask = groups == gid
            order = np.argsort(np.arange(mask.sum()))  # rows already time-ordered per group
            p = oof[mask][order]
            t = truth[mask][order]
            smoothed = fn(p)
            s = slope(smoothed, t)
            if s == s:
                slopes.append(s)
            clipped = np.clip(smoothed, *DISTANCE_BOUNDS)
            errs.extend(list(np.abs(clipped - t)))
        errs = np.asarray(errs)
        print(f"{label:22s} {np.median(slopes):13.3f} {errs.mean():8.3f} "
              f"{np.sqrt((errs**2).mean()):8.3f}")

    print("\nNếu slope không tăng đáng kể khi tăng cửa sổ làm mượt -> thiên lệch hệ")
    print("thống (shrinkage), không phải nhiễu. Nếu slope tăng rõ theo cửa sổ ->")
    print("phần lớn là nhiễu, làm mượt là hướng đúng.")


if __name__ == "__main__":
    main()
