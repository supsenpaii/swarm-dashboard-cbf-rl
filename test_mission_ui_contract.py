"""Mission controls must use the companion path, never legacy OFFBOARD MAP."""

from pathlib import Path


INDEX = Path(__file__).parent / "static" / "index.html"


def test_mission_ui_has_explicit_start_stop_and_readiness_controls():
    source = INDEX.read_text(encoding="utf-8")
    for token in (
        'id="mission-start-button"',
        'id="mission-stop-button"',
        'id="mission-readiness"',
        'type:"mission_start"',
        'type:"mission_stop"',
    ):
        assert token in source


def test_mission_instructions_do_not_tell_the_operator_to_use_legacy_offboard():
    source = INDEX.read_text(encoding="utf-8")
    assert "Không bấm MAP POINT · LEGACY để chạy quỹ đạo" in source
    assert "vẽ → GỬI → mới bật OFFBOARD" not in source


def test_mission_speed_comes_from_the_runtime_config():
    source = INDEX.read_text(encoding="utf-8")
    for token in (
        'id="mission-profile"',
        "applyMissionConfig(m.mission_config)",
        'input.max=String(maximum)',
        'input.value=String(fallback)',
    ):
        assert token in source
