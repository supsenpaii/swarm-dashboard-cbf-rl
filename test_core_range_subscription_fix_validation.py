from __future__ import annotations

import csv
import json
import sys

import core_range_subscription_fix_validation_analyze as analysis


def write_run(root, config_id, repeat, values):
    run = root / "runs" / f"{config_id}_run{repeat}"
    run.mkdir(parents=True)
    with (run / "per_second_metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("elapsed_s", "gazebo_rtf"))
        writer.writeheader()
        for second, value in enumerate(values):
            writer.writerow({"elapsed_s": second, "gazebo_rtf": value})
    (run / "exit_code.txt").write_text("0\n")


def test_analyzer_separates_fix_gate_from_causal_gate(tmp_path, monkeypatch):
    passing = [1.0] * 600
    failing = [1.0] * 596 + [0.1] * 4
    for repeat in range(1, 4):
        write_run(tmp_path, "S6_on_demand", repeat, passing)
        write_run(tmp_path, "S5", repeat, failing)

    monkeypatch.setattr(sys, "argv", ["analyze", "--root", str(tmp_path)])
    assert analysis.main() == 0
    decision = json.loads((tmp_path / "validation_decision.json").read_text())
    assert decision["fix_runtime_gate_pass"] is True
    assert decision["camera_eager_cause_confirmed"] is True


def test_analyzer_fails_closed_for_incomplete_matrix(tmp_path, monkeypatch):
    write_run(tmp_path, "S6_on_demand", 1, [1.0] * 600)
    monkeypatch.setattr(sys, "argv", ["analyze", "--root", str(tmp_path)])
    assert analysis.main() == 1
    decision = json.loads((tmp_path / "validation_decision.json").read_text())
    assert decision["conclusion"] == "VALIDATION_INCOMPLETE"
