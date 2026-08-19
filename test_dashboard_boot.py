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
