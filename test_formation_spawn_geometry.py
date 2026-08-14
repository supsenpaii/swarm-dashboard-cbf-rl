"""Regression check: the configured SITL spawn pose must not create a
near-miss with the configured formation slot.

Measured, not assumed: TWO_UAV_ACTIVE_SITL_FLIGHT's first two real flights
(2026-08-10) both showed CBF's minimum_margin_m dropping close to zero
(0.032 m and 0.097 m against a 4.0 m SWARM_CBF_MINIMUM_SEPARATION_M
requirement) while UAV-02 tracked its formation slot. Simulating the real
DeterministicFormationController + CbfCommandGate (not reimplemented math)
against the then-configured spawn pose reproduced this within noise: 4.472 m
closest approach, 0.222 m margin, from a straight-line P-controller path that
happened to swing close to the leader on its way to a slot in a very
different direction (5 m north spawn vs. a slot 10 m west of the leader).

Two things were ruled out as fixes by the same simulation, both important
enough to pin here so nobody re-tries them:

  - Raising SWARM_CBF_MINIMUM_SEPARATION_M makes it WORSE, not better: the
    old spawn was only 5.0 m apart, so raising the requirement (swept 4.0 to
    8.0) pushes the margin negative by 4.5 m, starts producing occasional
    cbf_constraints_infeasible frames by 5.5 m, and by 6.0 m the follower is
    permanently stuck at zero velocity (infeasible on every single frame,
    never reaching its slot) -- a bigger problem than the thin margin it was
    meant to fix.
  - Raising SWARM_CBF_BARRIER_GAIN_S_INV does nothing: swept 2.0 to 32.0,
    identical minimum_margin_m every time. A minimally-invasive CBF filter
    converges to the constraint boundary regardless of gain; gain does not
    add slack.

The only lever that actually worked was geometry: spawning the follower
roughly along its own approach axis instead of perpendicular to it. This
test encodes that property so it is checked automatically against whatever
spawn pose and slot offset are actually configured, rather than trusting
the one geometry that happened to be measured once.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

from cbf_command_gate import CbfCommandGate, CbfConfig
from formation_controller import DeterministicFormationController, FormationConfig, FormationSlot

# Below this, a real vehicle's tracking error (measured: ~5.5% horizontal
# overshoot on the actual flights, see the 2026-08-10 handoff) could plausibly
# eat the remaining slack. Not a CBF constant -- a check on this test's own
# conclusion, independent of the production config.
MINIMUM_ACCEPTABLE_MARGIN_M = 0.5
MAXIMUM_STEPS = 3000
DT_S = 0.05


def _read_env_float(path: Path, name: str, default: float) -> float:
    if not path.exists():
        return default
    match = re.search(rf"^{re.escape(name)}=(.+)$", path.read_text(encoding="utf-8"), re.MULTILINE)
    return float(match.group(1).strip()) if match else default


def _read_env_vector(path: Path, name: str, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if not path.exists():
        return default
    match = re.search(rf"^{re.escape(name)}=(.+)$", path.read_text(encoding="utf-8"), re.MULTILINE)
    if not match:
        return default
    parts = tuple(float(part.strip()) for part in match.group(1).split(","))
    if len(parts) != 3:
        raise ValueError(f"{name} requires three components")
    return parts  # type: ignore[return-value]


def _read_bash_default_pose(path: Path, shell_var: str) -> tuple[float, float, float]:
    """Extracts `x,y,z,...` from `var="${SHELL_VAR:-x,y,z,roll,pitch,yaw}"` in
    run_all.sh. Regex over bash rather than a duplicated Python constant, so
    this test cannot silently drift from what actually gets spawned."""
    text = path.read_text(encoding="utf-8")
    match = re.search(rf'\$\{{{re.escape(shell_var)}:-([^}}]+)\}}"', text)
    if not match:
        raise AssertionError(f"could not find {shell_var} default in {path}")
    parts = match.group(1).split(",")
    if len(parts) != 6:
        raise AssertionError(f"{shell_var} default does not have 6 pose components: {match.group(1)!r}")
    x, y, z = (float(part) for part in parts[:3])
    return x, y, z


def _simulate_closing_margin(
    *,
    leader_spawn_enu: tuple[float, float, float],
    follower_spawn_enu: tuple[float, float, float],
    slot_offset_enu_m: tuple[float, float, float],
    formation_config: FormationConfig,
    cbf_config: CbfConfig,
) -> tuple[float, float, float | None]:
    """Kinematic replay of the real formation + CBF classes.

    Position integrates the CBF-filtered velocity directly (v * dt), which is
    an idealization -- real PX4 tracking lags and overshoots the commanded
    velocity (measured: ~5.5% on the real flights). That is exactly why
    MINIMUM_ACCEPTABLE_MARGIN_M is a buffer above zero, not the razor's edge
    this same simulation showed the old geometry sitting on.
    """
    leader_id, follower_id = "UAV-01", "UAV-02"
    formation = DeterministicFormationController(
        leader_id, (FormationSlot(follower_id, slot_offset_enu_m),), formation_config
    )
    cbf_gate = CbfCommandGate(follower_id, (leader_id,), cbf_config)

    leader_pos = list(leader_spawn_enu)
    leader_vel = [0.0, 0.0, 0.0]
    follower_pos = list(follower_spawn_enu)
    follower_vel = [0.0, 0.0, 0.0]

    minimum_distance = float("inf")
    minimum_margin = float("inf")

    for step in range(MAXIMUM_STEPS):
        swarm_state = {
            leader_id: {
                "valid": True,
                "position_enu_m": list(leader_pos),
                "velocity_enu_m_s": list(leader_vel),
                "message_age_ms": 25.0,
            },
            follower_id: {
                "valid": True,
                "position_enu_m": list(follower_pos),
                "velocity_enu_m_s": list(follower_vel),
                "message_age_ms": 25.0,
            },
        }
        nominal = formation.command(follower_id, swarm_state)
        command = cbf_gate.filter(nominal.velocity_enu_m_s, swarm_state)
        velocity = command.velocity_enu_m_s if command.active else (0.0, 0.0, 0.0)

        distance = sum((follower_pos[i] - leader_pos[i]) ** 2 for i in range(3)) ** 0.5
        minimum_distance = min(minimum_distance, distance)
        if command.minimum_margin_m is not None:
            minimum_margin = min(minimum_margin, command.minimum_margin_m)

        for axis in range(3):
            follower_pos[axis] += velocity[axis] * DT_S
            follower_vel[axis] = velocity[axis]

        if nominal.reason == "slot_reached":
            return minimum_distance, minimum_margin, round(step * DT_S, 2)

    return minimum_distance, minimum_margin, None


class FormationSpawnGeometryTests(unittest.TestCase):
    def setUp(self) -> None:
        repo_root = Path(__file__).resolve().parent
        self.run_all_sh = repo_root / "run_all.sh"
        self.env_path = repo_root / ".env"

        self.leader_spawn = _read_bash_default_pose(self.run_all_sh, "SWARM_UAV_01_MODEL_POSE")
        self.follower_spawn = _read_bash_default_pose(self.run_all_sh, "SWARM_UAV_02_MODEL_POSE")
        self.slot_offset = _read_env_vector(
            self.env_path, "SWARM_FORMATION_SLOT_UAV_02_ENU_M", (-10.0, 0.0, 0.0)
        )
        self.formation_config = FormationConfig(
            position_gain_s_inv=_read_env_float(self.env_path, "SWARM_FORMATION_POSITION_GAIN_S_INV", 0.6),
            maximum_velocity_m_s=_read_env_float(self.env_path, "SWARM_FORMATION_MAXIMUM_VELOCITY_M_S", 2.0),
            arrival_radius_m=_read_env_float(self.env_path, "SWARM_FORMATION_ARRIVAL_RADIUS_M", 0.25),
        )
        self.cbf_config = CbfConfig(
            minimum_separation_m=_read_env_float(self.env_path, "SWARM_CBF_MINIMUM_SEPARATION_M", 4.0),
            barrier_gain_s_inv=_read_env_float(self.env_path, "SWARM_CBF_BARRIER_GAIN_S_INV", 2.0),
            maximum_velocity_m_s=_read_env_float(self.env_path, "SWARM_CBF_MAXIMUM_VELOCITY_M_S", 2.0),
        )

    def test_configured_spawn_reaches_the_slot_with_a_safe_margin(self) -> None:
        distance, margin, reached_at_s = _simulate_closing_margin(
            leader_spawn_enu=self.leader_spawn,
            follower_spawn_enu=self.follower_spawn,
            slot_offset_enu_m=self.slot_offset,
            formation_config=self.formation_config,
            cbf_config=self.cbf_config,
        )
        self.assertIsNotNone(
            reached_at_s,
            "follower never reached its slot within the simulated horizon -- "
            "likely cbf_constraints_infeasible from spawning already too close "
            "to satisfy SWARM_CBF_MINIMUM_SEPARATION_M",
        )
        self.assertGreaterEqual(
            margin,
            MINIMUM_ACCEPTABLE_MARGIN_M,
            f"CBF minimum_margin_m ({margin:.3f} m) fell below the "
            f"{MINIMUM_ACCEPTABLE_MARGIN_M} m safety buffer during the "
            f"configured spawn-to-slot approach (closest approach "
            f"{distance:.3f} m) -- the spawn pose likely sends the follower's "
            f"straight-line path close to the leader before it reaches its "
            f"slot. See this module's docstring for what does and does not fix that.",
        )

    def test_raising_minimum_separation_is_not_the_fix_for_this_geometry(self) -> None:
        """Pins the finding that motivated fixing geometry instead: at the
        *previous* (perpendicular) spawn, tightening the CBF constant broke
        feasibility rather than helping. Uses a fixed, historical spawn --
        not `self.follower_spawn` -- because this asserts a property of the
        old geometry that justified not choosing that fix, not a property of
        whatever spawn is configured today."""
        historical_perpendicular_spawn = (0.0, 5.0, 0.0)
        _distance, _margin, reached_at_s = _simulate_closing_margin(
            leader_spawn_enu=self.leader_spawn,
            follower_spawn_enu=historical_perpendicular_spawn,
            slot_offset_enu_m=(-10.0, 0.0, 0.0),
            formation_config=FormationConfig(
                position_gain_s_inv=0.6,
                maximum_velocity_m_s=2.0,
                arrival_radius_m=0.25,
            ),
            cbf_config=CbfConfig(
                minimum_separation_m=6.0,
                barrier_gain_s_inv=self.cbf_config.barrier_gain_s_inv,
                maximum_velocity_m_s=2.0,
            ),
        )
        self.assertIsNone(
            reached_at_s,
            "expected raising minimum_separation_m to 6.0 against the old "
            "perpendicular spawn to make the follower permanently stuck "
            "(cbf_constraints_infeasible) -- if this now succeeds, the "
            "'raising the constant doesn't work' finding in this module's "
            "docstring needs re-checking, not silently dropping",
        )


if __name__ == "__main__":
    unittest.main()
