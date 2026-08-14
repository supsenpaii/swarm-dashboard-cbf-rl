from pathlib import Path

import core_range_rtf_stutter_root_cause as rtf


def rows(values):
    return [{"elapsed_s": i, "gazebo_rtf": value} for i, value in enumerate(values)]


def test_rtf_event_detection_and_streak_counting():
    events = rtf.detect_events(rows([1, 1, .2, .2, .2, 1]), "r")
    assert len(events) == 1
    assert events[0]["sample_count"] == 3
    assert events[0]["gate_failing"] is True


def test_half_second_samples_collapse_to_one_second_worst_case():
    source = [{"elapsed_s": 1.0, "gazebo_rtf": 1}, {"elapsed_s": 1.49, "gazebo_rtf": .2}]
    assert rtf.one_second_samples(source)[0]["rtf"] == .2


def test_event_window_extraction():
    source = rows([1.0] * 100)
    event = {"event_id": "e", "start_elapsed_s": 30, "end_elapsed_s": 32}
    selected = rtf.extract_event_window(source, event)
    assert selected[0]["elapsed_s"] == 15
    assert selected[-1]["elapsed_s"] == 62


def test_config_matrix_completeness():
    assert rtf.validate_config_matrix([{"config_id": value} for value in "ABCDEFGH"])
    assert not rtf.validate_config_matrix([{"config_id": value} for value in "ABCDEFG"])


def test_process_metric_parsing():
    assert rtf.parse_process_metric({"gz_sim_cpu_pct": "12.5"}, "gz_sim", "cpu_pct") == 12.5
    assert rtf.parse_process_metric({}, "gz_sim", "cpu_pct") is None


def test_no_overlap_between_runs():
    assert rtf.runs_do_not_overlap([{"start_wall_s": 1, "end_wall_s": 2}, {"start_wall_s": 2, "end_wall_s": 3}])
    assert not rtf.runs_do_not_overlap([{"start_wall_s": 1, "end_wall_s": 3}, {"start_wall_s": 2, "end_wall_s": 4}])


def test_precommit_before_results(tmp_path: Path):
    result = tmp_path / "result"
    result.write_text("x")
    assert rtf.precommit_before_results(result.stat().st_mtime - 1, [result])
    assert not rtf.precommit_before_results(result.stat().st_mtime + 1, [result])


def test_root_cause_evidence_rule_requires_two_pairs_and_mechanism():
    pair = {"before_pass": True, "after_fail": True, "single_change": True}
    assert rtf.root_cause_rule([pair, pair], True)
    assert not rtf.root_cause_rule([pair], True)
    assert not rtf.root_cause_rule([pair, pair], False)


def test_post_fix_three_soak_gate():
    good = {"complete": True, "rtf_gate_pass": True, "fps_latency_integrity_pass": True}
    assert rtf.post_fix_soak_gate([good, good, good])
    assert not rtf.post_fix_soak_gate([good, good])


def test_six_smoke_gate():
    good = [{"scenario": name, "all_gates_pass": True} for name in ("approaching", "approaching", "receding", "receding", "stop_and_hold", "stop_and_hold")]
    assert rtf.six_smoke_gate(good)
    assert not rtf.six_smoke_gate(good[:-1])


def test_candidate_b_checksum_non_interference():
    before = {"candidate": "B", "feature_order": ["x"]}
    after = {"candidate": "B", "feature_order": ["x"], "max_abs_diff_m": 0.0}
    assert rtf.candidate_non_interference(before, after)


def test_no_controller_px4_effect():
    scope = {key: False for key in ("arm", "takeoff", "offboard", "follow_target", "shadow_controller", "dataset_collection")}
    assert rtf.scope_has_no_controller_effect(scope)
    scope["offboard"] = True
    assert not rtf.scope_has_no_controller_effect(scope)


def test_rollback_contract():
    assert rtf.rollback_is_explicit({"rollback_command_or_env": "unset X", "production_default_changed": False})
    assert not rtf.rollback_is_explicit({"production_default_changed": False})
