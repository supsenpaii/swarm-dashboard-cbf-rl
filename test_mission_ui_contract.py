"""Mission controls must use the companion path, never legacy OFFBOARD MAP."""

import re
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
    assert "Do not use MAP POINT · LEGACY to run a route" in source
    assert "draw → SEND → then enable OFFBOARD" not in source


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
        "Map click = add waypoint",
        "Map click = pick destination point",
        "Map click does nothing",
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


def test_the_plan_workspace_can_pick_which_vehicle_the_mission_goes_to():
    """The FLY control bar owns the only other vehicle tabs, and it is hidden here.

    Before this, opening PLAN left `selectedDroneId` at whatever FLY had last
    set, with nothing on screen naming it -- so a route drawn for one vehicle
    could be sent to the other with no visible cue. The picker reuses the
    existing `.tab-button[data-drone-id]` contract so one click updates both
    sets, and it reports the *other* vehicle's route state too.
    """
    source = INDEX.read_text(encoding="utf-8")
    for token in (
        'class="ws ws-plan"',
        'class="tab-button drone-pick active" data-drone-id="UAV-01"',
        'class="tab-button drone-pick" data-drone-id="UAV-02"',
        'id="pick-note-UAV-01"',
        'id="pick-note-UAV-02"',
        'id="mission-send-target"',
    ):
        assert token in source, token
    # The send button must name its destination, not just say "send".
    assert "SEND MISSION TO <span id=\"mission-send-target\">" in source


def test_the_dashboard_never_reaches_a_cdn():
    """A ground station must render with no internet: tiles, fonts and Leaflet
    all come from our own FastAPI. This is not hypothetical -- the router on the
    development network resolves every openstreetmap.org name to 127.0.0.1, so a
    live tile layer renders nothing at all here."""
    source = INDEX.read_text(encoding="utf-8")
    assert "unpkg.com" not in source
    assert "fonts.googleapis.com" not in source
    assert "fonts.gstatic.com" not in source
    assert '/static/vendor/leaflet/leaflet.js' in source
    assert '/static/vendor/fonts/fonts.css' in source
    assert "GridBackdrop" in source
    assert '"/static/tiles/{z}/{x}/{y}.png"' in source
    external = set(re.findall(r"https?://([a-z0-9.-]+)", source))
    assert external == set(), external
