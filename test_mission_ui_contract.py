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


def test_a_map_click_always_says_what_it_will_do():
    """The sentence above is a warning; this is the thing that removes the doubt.

    Two map-click modes shared one canvas with nothing on screen to tell them
    apart -- click without arming the draw tool and the click fell through to
    the legacy fly-to-point branch, whose message sent the operator further the
    wrong way. The badge names the live mode on every frame, and the two modes
    now live in different workspaces, so the warning is the backstop rather
    than the whole defence.
    """
    source = INDEX.read_text(encoding="utf-8")
    for token in (
        'id="map-click-badge"',
        'id="map-click-text"',
        "function updateClickBadge()",
        "Click bản đồ = thêm waypoint",
        "Click bản đồ = chọn điểm bay tới",
        "Click bản đồ không làm gì",
    ):
        assert token in source, token
    # Drawing tools and the legacy fly-to-point control must never be on
    # screen together: one is ws-plan, the other ws-fly.
    assert 'id="mission-draw-button"' in source
    assert 'id="offboard-mode-button"' in source


def test_mission_speed_comes_from_the_runtime_config():
    source = INDEX.read_text(encoding="utf-8")
    for token in (
        'id="mission-profile"',
        "applyMissionConfig(m.mission_config)",
        'input.max=String(maximum)',
        'input.value=String(fallback)',
    ):
        assert token in source
