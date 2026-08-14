"""CORE_RANGE_STABLE_RELATIVE_TRACKING_3_12M, Phase 4/6/7 driver: train the
3 precommitted direct-XGBoost candidates, apply per-fold affine and
isotonic calibration, write prediction_rows.csv with raw + both calibrated
outputs per candidate for the downstream alpha-beta filter stage.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

import core_range_stable_relative_tracking as s

OUT = s.OUT


def main() -> None:
    combined_rows, fold_map = s.load_corpus()
    precommit = json.loads((OUT / "training_precommit.json").read_text())

    weight_fns = {
        "CANDIDATE_A_GROUP_BALANCED": s.weights_group_balanced,
        "CANDIDATE_B_GROUP_AND_DISTANCE_BALANCED": s.weights_group_and_distance_balanced,
        "CANDIDATE_C_MONOTONIC_RAY_SCALE": s.weights_group_balanced,
    }

    prediction_rows = []
    raw_metrics_rows = []
    calibration_params_all = {}

    for name, spec in precommit["candidates"].items():
        print(f"=== training {name} ===", flush=True)
        monotone = None
        if spec["monotone_constraints"]:
            monotone = tuple(-1 if f == "ray_scale" else 0 for f in s.STABLE_FEATURES)

        trained = s.train_direct(
            combined_rows, fold_map, s.STABLE_FEATURES, spec["hyperparameters"],
            weight_fns[name], monotone, seed=52,
        )
        oof = trained["oof"]
        train_preds = s.fold_train_predictions(combined_rows, fold_map, trained)
        affine_pred, affine_params = s.apply_affine_calibration(combined_rows, fold_map, oof, train_preds)
        isotonic_pred, isotonic_params = s.apply_isotonic_calibration(combined_rows, fold_map, oof, train_preds)
        calibration_params_all[name] = {"affine": affine_params, "isotonic": isotonic_params}

        for fold in range(3):
            trained["boosters"][fold].save_model(str(OUT / "models" / f"{name}_fold_{fold}.json"))
            train_groups = sorted({r["group_id"] for r in combined_rows if fold_map[r["group_id"]] != fold})
            (OUT / "models" / f"{name}_fold_{fold}_preprocessing.json").write_text(
                json.dumps(trained["preprocessors"][fold].as_json(train_groups), indent=2, sort_keys=True) + "\n"
            )

        gt = np.asarray([float(r["ground_truth_range_m"]) for r in combined_rows])
        for label, pred in (("none", oof), ("affine", affine_pred), ("isotonic", isotonic_pred)):
            abs_err = np.abs(pred - gt)
            raw_metrics_rows.append({
                "candidate": name, "calibration": label,
                "mae_m": round(float(np.mean(abs_err)), 4),
                "medae_m": round(float(np.median(abs_err)), 4),
                "p90_m": round(float(np.percentile(abs_err, 90)), 4),
                "bias_m": round(float(np.mean(pred - gt)), 4),
                "std_m": round(float(np.std(pred - gt)), 4),
            })

        for i, row in enumerate(combined_rows):
            prediction_rows.append({
                "candidate": name, "group_id": row["group_id"], "domain": row["domain"],
                "scenario_type": row["scenario_type"], "fold": fold_map[row["group_id"]],
                "measurement_timestamp_s": row["measurement_timestamp_s"],
                "ground_truth_range_m": row["ground_truth_range_m"],
                "raw_physical_range_m": row["raw_physical_range_m"],
                "pred_none_m": float(oof[i]), "pred_affine_m": float(affine_pred[i]),
                "pred_isotonic_m": float(isotonic_pred[i]),
            })
        print(json.dumps(raw_metrics_rows[-3:], indent=2), flush=True)

    s.write_csv(OUT / "prediction_rows.csv", prediction_rows)
    s.write_csv(OUT / "raw_candidate_metrics.csv", raw_metrics_rows)

    # calibration provenance / checksums
    calib_dir = OUT / "calibration"
    calib_dir.mkdir(exist_ok=True)
    for name, params in calibration_params_all.items():
        (calib_dir / f"{name}_affine.json").write_text(json.dumps(params["affine"], indent=2))
        (calib_dir / f"{name}_isotonic.json").write_text(json.dumps(params["isotonic"], indent=2))
    print("done")


if __name__ == "__main__":
    main()
