"""Generate a world SDF overlay that adds the sim-time trajectory system plugin.

GZ_SIM_SERVER_CONFIG_PATH is only consulted by gz-sim as a fallback for
worlds that declare no system plugins of their own. PX4's default.sdf
already declares Physics/UserCommands/SceneBroadcaster/etc directly, so a
custom server.config is silently never loaded (confirmed empirically: no
Configure() call, no subscriber on the command topic, no log line, even at
--verbose=4). The only reliable way to add a system plugin to that world is
to declare it in the SDF itself. This script copies the source world file
and injects a <plugin> element for the trajectory system, leaving the
original PX4 file untouched.
"""
from __future__ import annotations

import sys
from pathlib import Path

PLUGIN_ELEMENT = (
    '    <plugin name="swarm::simulation::SimTimeTrajectorySystem" '
    'filename="libswarm_sim_time_trajectory.so"/>\n'
)


def build_overlay(source_world: Path, destination: Path) -> None:
    text = source_world.read_text(encoding="utf-8-sig")
    if PLUGIN_ELEMENT.strip() in text:
        destination.write_text(text, encoding="utf-8")
        return
    marker = "<gui "
    index = text.find(marker)
    if index == -1:
        marker = "</world>"
        index = text.find(marker)
        if index == -1:
            raise ValueError(f"no insertion point found in {source_world}")
    text = text[:index] + PLUGIN_ELEMENT + text[index:]
    destination.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: build_overlay_world.py SOURCE_WORLD_SDF DESTINATION_SDF", file=sys.stderr)
        raise SystemExit(2)
    build_overlay(Path(sys.argv[1]), Path(sys.argv[2]))
