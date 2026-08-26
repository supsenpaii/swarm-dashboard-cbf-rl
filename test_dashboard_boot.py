"""The dashboard's first script must reach connectWebSocket().

The page is two classic scripts sharing one global scope, and the second one
defines the safety strip, the mission stepper and the map-click badge. The
first one calls into them. That call order is a real trap: the first script's
last statement is

    updateModeUi();renderTelemetry();updateGimbalUi();updateTrackingUi();connectWebSocket();

so a bare cross-script call inside updateModeUi() throws ReferenceError before
the second script has been evaluated, the whole statement dies, and the socket
is never opened. The page then loads perfectly and shows "WebSocket
disconnected" forever -- which is exactly what shipped on 2026-08-19, with the
backend healthy and serving GET / 200 the whole time.

Grep, not execution: the failure is a NAME resolved too early, which is visible
in the source and cheap to pin.
"""

import re
from html.parser import HTMLParser
from pathlib import Path


INDEX = Path(__file__).parent / "static" / "index.html"

# Defined in the SECOND script; anything the first script calls must be guarded.
LATE_FUNCTIONS = ("updateClickBadge", "updateSafetyUi", "updateStepper")


def _scripts() -> list[str]:
    return re.findall(r"<script>(.*?)</script>", INDEX.read_text(encoding="utf-8"), re.S)


def test_the_page_has_exactly_two_inline_scripts() -> None:
    assert len(_scripts()) == 2


def test_the_first_script_never_calls_a_late_function_by_bare_name() -> None:
    first = _scripts()[0]
    for name in LATE_FUNCTIONS:
        bare = re.findall(rf"(?<![.\w]){name}\s*\(", first)
        assert not bare, (
            f"{name}() is called by bare name in the first script; it is not "
            "defined until the second one runs. Use globalThis."
            f"{name}?.() instead."
        )
        assert f"globalThis.{name}?.(" in first, (
            f"{name} is expected to be called from the first script through "
            "globalThis with an optional call"
        )


def test_the_second_script_defines_them() -> None:
    second = _scripts()[1]
    for name in LATE_FUNCTIONS:
        assert f"function {name}(" in second


def test_connect_websocket_is_the_last_thing_the_first_script_does() -> None:
    first = _scripts()[0]
    assert "connectWebSocket()" in first
    tail = first.rstrip().splitlines()[-1]
    assert "connectWebSocket()" in tail, (
        "connectWebSocket() must stay on the boot line; anything that throws "
        "earlier on that line takes the socket down with it"
    )


def test_the_fly_to_point_panel_is_not_nested_inside_the_takeoff_panel() -> None:
    """Two mutually exclusive modes cannot share a parent that one of them hides.

    `#fly-point-section` used to be a child of `#takeoff-control-section`,
    and updateModeUi() sets the child visible when `offboard` and the parent
    visible when `takeoff` -- conditions that are never both true. Measured in a
    browser: FLY TO POINT had a 0x0 bounding box in *every* mode, so MAP POINT
    could select a point on the map and never let anyone act on it.
    """
    source = INDEX.read_text(encoding="utf-8")

    class Ancestry(HTMLParser):
        """Records the open-element ids surrounding each id we care about."""

        def __init__(self) -> None:
            super().__init__(convert_charrefs=True)
            self.stack: list[str | None] = []
            self.ancestors: dict[str, list[str]] = {}

        def handle_starttag(self, tag, attrs):
            identifier = dict(attrs).get("id")
            if identifier:
                self.ancestors[identifier] = [
                    a for a in self.stack if a is not None
                ]
            if tag not in ("br", "img", "input", "meta", "link", "hr"):
                self.stack.append(identifier)

        def handle_endtag(self, tag):
            if self.stack:
                self.stack.pop()

    parser = Ancestry()
    parser.feed(source)
    assert "takeoff-control-section" not in parser.ancestors.get(
        "fly-point-section", []
    ), "fly-to-point panel is still inside the takeoff panel"
    # Takeoff must not be revived by setting display:block over .control-group.
    assert 'style.display=takeoff?"block"' not in source
    # Fly-to-point is no longer a flight mode, so nothing may gate it at all.
    assert 'getElementById("fly-point-section").style.display' not in source


def test_the_safety_layer_reads_the_payload_the_server_actually_sends() -> None:
    """`tracking` is the tracking-manager status object, not a per-drone map.

    safetyOf() read `snapshot.tracking[droneId].companion_safety`, which is
    always undefined, so the safety strip, the coordination role chips and the
    barrier-demand breakdown rendered a dash on every frame the project has ever
    run. The data is per drone under `tracking_pose_streams`, and the WebSocket
    snapshot has to carry it -- it only ever went out on /api/drones.
    """
    source = INDEX.read_text(encoding="utf-8")
    assert "snapshot.tracking_pose_streams" in source
    assert "snapshot.tracking[droneId]" not in source

    server = (INDEX.parent.parent / "main.py").read_text(encoding="utf-8")
    assert "def tracking_pose_stream_snapshot(" in server
    # Both transports must use the one builder, and the socket must send it.
    assert server.count("tracking_pose_stream_snapshot(") >= 3
    assert '"tracking_pose_streams": (' in server


def test_the_hard_floor_is_read_from_config_not_pinned_in_the_page() -> None:
    """The bar tick and the barrier breakdown used a literal 20.0 while the
    thresholds panel showed the live `minimum_separation_m`. On a profile whose
    floor is not 20 m the same quantity appeared twice, with two values."""
    source = INDEX.read_text(encoding="utf-8")
    assert "HARD_FLOOR_M" not in source
    assert "hardFloorM = Number(config.minimum_separation_m)" in source


def test_the_server_refuses_the_action_that_opens_the_second_writer() -> None:
    """Removing the button is not enough -- the allowlist is the actual gate.

    `offboard_map` (and its legacy alias `enable_offboard`) makes the ROS 2 node
    set `offboard_enabled` and stream position setpoints, which then race the
    companion's 20 Hz velocity stream inside PX4. The browser no longer sends
    it, but any MQTT or WebSocket client could, so the refusal has to live on
    the server where every client passes through.
    """
    server = (INDEX.parent.parent / "main.py").read_text(encoding="utf-8")
    allowlist = server[server.index("ALLOWED_ACTIONS = {"):]
    allowlist = allowlist[: allowlist.index("}")]
    assert '"offboard_map"' not in allowlist
    assert '"enable_offboard"' not in allowlist
    # The one-point path has no handler left to reach either.
    assert "async def handle_goto_global(" not in server
    assert '"goto_global"' not in server


def test_the_server_refuses_two_points_that_can_never_both_be_reached() -> None:
    """The pair check has to run BEFORE the mission goes onto MQTT.

    Two hold points closer than the station-keeping floor is not a conflict CBF
    can resolve -- the pair converges, meets the barrier and hovers short of
    both points, with the trajectory reporting finished the whole time because
    `LinearTrajectory.is_finished` is a clock and not a position check. And
    unlike `validate_mission`, this one cannot be re-run per drone on the
    bridge: it is a property of the PAIR, and each companion knows only its own
    mission. The server is the only place that sees both.
    """
    server = (INDEX.parent.parent / "main.py").read_text(encoding="utf-8")
    handler = server[server.index("async def handle_mission_path("):]
    handler = handler[: handler.index("async def handle_mission_action(")]
    assert "unreachable_destinations(pending)" in handler
    assert handler.index("unreachable_destinations(pending)") < handler.index(
        "publish_control_message("
    ), "the pair check must refuse before the mission is published to MQTT"
    # The radius the check uses is the one the map draws with -- one source.
    assert '"station_keeping_separation_m": limits.station_keeping_separation_m' in server
