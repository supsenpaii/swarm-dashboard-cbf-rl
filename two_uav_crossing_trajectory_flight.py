#!/usr/bin/env python3
"""ACTIVE_CBF_CROSSING_TRAJECTORY: fly UAV-01 AND UAV-02 on trajectories that
actually cross, so CBF has to do real work instead of confirming a baseline
where it barely touches anything.

Succeeds `two_uav_trajectory_flight.py` (TWO_UAV_TRAJECTORY_TRACKING's own
reviewed, non-crossing, non-conflict-baseline record -- measured intervention
rate 0.0000 there) rather than editing it. The precheck/guard/abort/alignment
shape is unchanged and untouched: none of it assumed non-crossing geometry,
so it needed no changes to be correct here too. Same downstream chain,
unchanged per this milestone's own roadmap: `u_nom -> local CBF ->
EmergencySupervisor -> PX4`.

THIS SCRIPT DOES NOT CONFIGURE THE TRAJECTORIES ITSELF
--------------------------------------------------------
Trajectory selection is companion-side, environment-driven
(`SWARM_TRAJECTORY_<DRONE_ID>_KIND` and friends, read once when the stack
starts -- see companion_safety.py's `_trajectory_env`), exactly like
`SWARM_FORMATION_SLOT_UAV_02_ENU_M` already is. A flight driver process
cannot reach into an already-running worker's config. `DEFAULT_TRAJECTORY_ENV`
below is the pair this driver's guards/measurements assume; `--print-env`
prints it so an operator can diff it against their actual `.env` before
flying, and `precheck()` aborts early with a clear diagnosis rather than
silently holding station if the live system never entered trajectory mode.

WHY THESE TWO DEFAULT LEGS SPECIFICALLY
------------------------------------------
Both start at each vehicle's own real hover position (UAV-01 (0,0), UAV-02
(-5,2) per run_all.sh, same pair TWO_UAV_TRAJECTORY_TRACKING used) so the
initial-frame-alignment gate below passes on real numbers, not luck. UAV-01
heads due north 20 m; UAV-02 heads roughly southeast, chosen so the two
paths pass close together early in the flight -- simulated WITHOUT CBF (not
reimplemented math: the real TrajectoryTrackingController with no gate),
closest approach was 3.331 m at t=1.65s, inside the 4.0 m
SWARM_CBF_MINIMUM_SEPARATION_M -- a genuine conflict, not a near-miss by
construction. WITH the real CbfCommandGate: closest approach 4.423 m,
minimum_margin_m +0.045 m, zero infeasible frames, both still reach
"trajectory_reached", and CBF's own commanded correction against the nominal
peaks at 1.479 m/s over ~8% of frames -- an order of magnitude above this
project's own measured PX4 tracking noise floor (~0.1 m/s), so a real flight
can actually attribute the correction to CBF rather than noise.

A PERFECTLY SYNCHRONIZED HEAD-ON CROSSING WAS TRIED AND REJECTED
--------------------------------------------------------------------
The first design was geometrically symmetric: both vehicles starting their
mission clock at the same instant, same speed, paths crossing at a single
point equidistant from both starts. Simulated over a 120 s horizon, neither
vehicle ever reached its endpoint -- a logjam, not a bug. A reactive,
minimally-invasive CBF has no right-of-way rule; two vehicles arriving at a
conflict point simultaneously push each other apart symmetrically forever
rather than one yielding. The legs below are deliberately asymmetric (UAV-02
crosses the conflict region well after UAV-01 does, by construction of the
geometry -- not by an explicit per-vehicle clock offset, which nothing in
the live system currently implements) specifically to break that symmetry.

WHY MARGIN IS THIN HERE ON PURPOSE, NOT A REGRESSION
---------------------------------------------------------
TWO_UAV_TRAJECTORY_TRACKING's spawn-geometry fix chased margin UP to ~1.3 m by
making CBF barely need to act. Sweeping this crossing's own geometry the same
way here (`crossing_trajectory_sim.py`/`crossing_trajectory_sim2.py`,
scratchpad) turned up a hard structural finding: margin and correction
strength trade off along a curve, but a comfortable margin (>=0.2 m) and a
clearly measurable correction (>=0.3 m/s) never coexist for this geometry --
minimally-invasive CBF converges to the constraint boundary whenever it is
actually the thing keeping two vehicles apart (same "gain does not add
slack" finding from the spawn-geometry investigation, now confirmed to apply
to margin itself). A thin `minimum_margin_m` during active intervention is
therefore the EXPECTED signature of CBF genuinely working, not a fault to
tune away -- what matters is that it never goes negative and never goes
infeasible.

FIRST REAL ATTEMPT (phi=-30, margin +0.045 simulated) FAILED IN FLIGHT --
NOT A BUG, A REAL MARGIN EXCURSION THE SIMULATION UNDERESTIMATED
------------------------------------------------------------------------
That attempt's own trace: physical separation never dropped below 4.193 m
(safely above the 4.0 m floor -- no hard-limit violation), but the strict
`minimum_margin_m` (which bakes in a latency buffer scaling with closing
speed) reached -0.149 m on UAV-02 during a 1.92 m/s peak correction, and the
driver's `cbf_separation_violated` guard correctly aborted both vehicles
1.03 s after trajectory mode engaged. The idealized simulation (position =
integral of COMMANDED velocity, no PX4 tracking lag) predicted +0.045 m; real
flight measured -0.149 m -- a ~0.19 m gap that only shows up once CBF is
correcting HARD and FAST, which the earlier non-crossing baseline's
near-zero intervention never exercised.

THE RETRY, AND THE TRADEOFF IT MAKES EXPLICIT
--------------------------------------------------
The genuine-hard-limit-violation region (closest approach <4.0 m with no
CBF at all) only exists for phi roughly <= -48 deg, and margin inside that
region tops out around 0.07 m regardless of exactly where -- thin by the
same structural property above, not a search failure. `DEFAULT_TRAJECTORY_ENV`
below instead uses phi=-60 deg, OUTSIDE that region (without CBF, closest
approach is 4.312 m -- never actually violates the 4.0 m floor). CBF still
does real, measurable, non-trivial work here (correction peaks 0.385 m/s,
~5.6% of frames, both ~4x this project's measured PX4 noise floor) because
its own required-margin formula is stricter than the raw floor -- this
demonstrates CBF acting PREVENTIVELY to hold the latency-inclusive margin
positive, not recovering from an already-violated hard limit. That is a
real, deliberate change from the first attempt's design, made explicit here
rather than silently: less severe as a "collision saved" demonstration, but
the correction magnitude (~0.385 m/s vs. the failed attempt's 0.945-1.9 m/s)
is small enough that the same real-world excess that cost ~0.19 m at high
correction should cost markedly less here -- to be MEASURED on this flight,
not assumed.

WHY NO LEADER/FOLLOWER SEQUENCING IS REQUIRED ANYMORE
---------------------------------------------------------
`two_uav_active_flight.py` armed/took off/engaged the leader before the
follower because the follower's nominal command depended on the leader's
*live* position. A trajectory's nominal depends only on wall-clock-relative
mission time and its own vehicle's position -- never on a peer's state. Only
CBF (the safety layer, not the nominal) still reads peer state. UAV-01 is
still sequenced first below purely for deterministic, readable logs, not
because of any data dependency.

EACH RUN NEEDS A FRESHLY RESTARTED STACK
-------------------------------------------
The trajectory starts are ABSOLUTE shared-ENU coordinates, so they are only
valid from the spawn poses run_all.sh actually places the vehicles at. A
previous run that moved a vehicle -- even an aborted one -- leaves it parked
somewhere else, and it will take off from there next time. Measured: an
attempt that held OFFBOARD for ~1 s pushed UAV-01 2.05 m east, and the very
next run was correctly refused by the alignment gate below (2.047 m > 1.5 m)
before it could fly a trajectory it no longer started at. Restart the stack
between runs; the gate is what makes forgetting safe rather than silent.

MEASUREMENT, NOT JUST A GUARD
------------------------------
`nominal_position_error_m` (tracking error against `p_ref(t)`) is new
telemetry added for this milestone (see companion_safety.py's
`CompanionSafetyStatus.nominal_position_error_m`) -- neither the formation
nor the altitude-hold flights had anything to measure tracking error against.
It is reported, not guarded on: CBF is what's safety-critical here, same as
before, so `cbf_minimum_margin_m` keeps its abort-on-violation guard;
tracking error is instrumentation for judging PASS at the end.

ATTEMPT 3 (2026-08-10, same day, later session) -- NOT YET FLOWN
----------------------------------------------------------------------
Both attempts above assumed `CbfConfig.command_latency_s`'s unvalidated
0.10s default. A dedicated `latency_measurement_flight.py` measured it for
real: median 0.653s across 8 clean step edges. Re-simulating BOTH earlier
geometries with the corrected value showed they are not just thin anymore --
they are badly infeasible (margins -0.409m and -2.274m, the second with 175
infeasible frames out of ~3000). A systematic offline sweep (speed x angle x
spawn-separation scale, real `CbfCommandGate`+`TrajectoryTrackingController`)
then found something structural, not a missed parameter: for this whole
geometry family, EVERY point with a comfortable margin has zero CBF
intervention, and margin collapses to ~0.000-0.03m at every point where
intervention turns on -- because a minimally-invasive CBF, when a
constraint is genuinely active, converges its solution exactly onto the
constraint boundary by construction. The two real flights' small "positive"
simulated margins were most likely discretization artifacts of that same
zero-convergence property, not a real buffer.

The fix is `CbfConfig.design_margin_buffer_m` (new field, default 0.0,
`cbf_command_gate.py`): a solve-time-only buffer added to the CBF's internal
constraint target, while the REPORTED `minimum_margin_m` stays anchored to
the true, unbuffered required_margin -- so a positive buffer makes the
solver hold real distance in reserve instead of hovering on the boundary.
Verified NOT a universal "bigger is safer" knob (see that field's own
docstring for the closing-speed feedback loop that makes it actively worse
at >=1.0 m/s per vehicle): the geometry below is redesigned around a MUCH
slower closing speed specifically so the buffer is stable.

`DEFAULT_TRAJECTORY_ENV` below is updated to UAV-01=1.0 m/s /
UAV-02=0.4 m/s (same phi=-60 heading as the retry -- geometry direction was
never the problem, closing speed was). This additionally requires
`SWARM_CBF_DESIGN_MARGIN_BUFFER_M=0.4` in `.env`, which `--print-env` now
also prints; this value is deliberately NOT part of `.env`'s permanent CBF
baseline (unsafe at the higher closing speeds other milestones use), so it
must be opted into by hand before this specific flight and removed after,
same operational discipline `SWARM_TRAJECTORY_*` already requires. Simulated
with the real classes at these numbers (`test_crossing_geometry_buffer.py`):
margin 0.383m, max intervention 0.740 m/s (about 7x the ~0.1 m/s PX4 noise
floor), 0 infeasible frames, closest physical approach 5.355m -- the widest,
most robust margin any crossing design in this investigation has produced.
Confirmed this is the SAME early-transient phenomenon both real aborted
flights hit (closest approach ~1s into trajectory mode, not a later
geometric path intersection -- an analytic check on the unbuffered case
matches the two real flights' own abort timing, ~1.03s after engaging), not
an easier, different scenario.

UAV-02's leg alone takes ~50s at 0.4 m/s (`--hold-s` default raised to 55.0
here to let it complete); `SWARM_FIRST_FLIGHT_MAX_DURATION_S=170` is
required in `.env` for this flight and is also in `--print-env`'s output
now (`REQUIRED_ENVELOPE_ENV`). `SWARM_CBF_DESIGN_MARGIN_BUFFER_M=0.4` is
NOT part of `.env`'s permanent CBF baseline (see `REQUIRED_CBF_ENV` and
`design_margin_buffer_m`'s own docstring for why) -- both must be set by
hand before this flight and unset after, same discipline
`SWARM_TRAJECTORY_*` already requires.

ATTEMPT 4 (2026-08-11) -- CURRENT PRODUCTION-UNCERTAINTY CONTRACT
-----------------------------------------------------------------
Attempt 3 passed before live covariance was enabled, but its minimum reported
margin was only 0.053 m. With the now-production `covariance_sigma=0.10`, that
reserve is consumed and the first shadow-only retry correctly aborted at
-0.07 m; the RL output was never applied. The current legs below supersede the
historical geometry above. They start at the same spawn poses, genuinely
intersect at (0,8,9), and then diverge to endpoints separated by 5.385 m.
UAV-01=0.40 m/s and UAV-02=0.15 m/s deliberately stagger arrival at the
intersection, avoiding the known symmetric deadlock without adding a
right-of-way controller.

The production-equivalent replay at peer age 100 ms and covariance sigma 0.10
reports minimum margin 0.354 m, physical slack 0.696 m, and a 0.259 m/s peak
CBF correction. A conservative plant stress (300 ms command delay plus a
0.75 s velocity time constant) still completes both legs with 0.397 m margin,
0.661 m physical slack, zero infeasible frames, and normal supervisor stages.
`test_crossing_geometry_buffer.py` pins both results. The 0.4 m design buffer
remains a flight-specific temporary setting, not a production default.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from two_uav_active_readiness import FlightEnvelope, PX4_FACTS, WarmupContract

API_URL = "http://127.0.0.1:8000/api/drones"
COMMANDER = (
    "/mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander"
)
UAV_01 = "UAV-01"
UAV_02 = "UAV-02"
DRONE_IDS: tuple[str, ...] = (UAV_01, UAV_02)
PX4_INSTANCE: dict[str, str] = {UAV_01: "0", UAV_02: "1"}

NAV_STATE_OFFBOARD = PX4_FACTS["nav_state_offboard"]["value"]
NAV_STATE_POSCTL = PX4_FACTS["nav_state_posctl"]["value"]

TRAJECTORY_NOMINAL_REASONS = {"tracking_trajectory", "trajectory_reached"}

# What this driver's guards/measurements assume is configured in the live
# stack's .env before ./run_all.sh starts. Not enforced by this process --
# see the module docstring.
# The x/y starts are each vehicle's Gazebo spawn, which IS a shared-ENU
# coordinate here and not an assumption: .env's SWARM_ENU_ORIGIN_* equals the
# world's own <spherical_coordinates> block in default.sdf, with
# world_frame_orientation ENU. (.env.example documents why a mismatch there
# silently corrupts every separation, and how to re-derive it.)
#
# The z start is 9.0 because shared-ENU altitude is NOT the ~10 m the vehicle
# reports locally: measured altitude_reference_m was 8.973 (UAV-01) and 8.810
# (UAV-02) on the geometry-fix flight, and 9.452 on the trace flight -- the
# known GPS-vs-local-NED offset. 9.0 sits inside that measured spread; the
# alignment gate below is what confirms it per flight instead of trusting it.
#
# UAV-02's end point ((5.0,-15.32) -- a -60 deg heading from due east) is
# unchanged from the retry: geometry DIRECTION was never the problem, closing
# SPEED was (see "ATTEMPT 4" in the module docstring). UAV-01=0.40 m/s /
# UAV-02=0.15 m/s is what keeps
# SWARM_CBF_DESIGN_MARGIN_BUFFER_M's solve-time buffer stable instead of
# feeding back into a worse margin. Do not "clean up" either the heading or
# the speeds to round numbers without re-running the simulation -- both were
# found by sweeping this exact geometry, not a formula.
DEFAULT_TRAJECTORY_ENV: dict[str, str] = {
    "SWARM_TRAJECTORY_UAV_01_KIND": "linear",
    "SWARM_TRAJECTORY_UAV_01_START_ENU_M": "0,0,9",
    "SWARM_TRAJECTORY_UAV_01_END_ENU_M": "0,16,9",
    "SWARM_TRAJECTORY_UAV_01_SPEED_M_S": "0.4",
    "SWARM_TRAJECTORY_UAV_02_KIND": "linear",
    "SWARM_TRAJECTORY_UAV_02_START_ENU_M": "-5,2,9",
    "SWARM_TRAJECTORY_UAV_02_END_ENU_M": "5,14,9",
    "SWARM_TRAJECTORY_UAV_02_SPEED_M_S": "0.15",
}

# Not a trajectory variable, but equally required in .env before this flight
# -- see "ATTEMPT 4" in the module docstring for why this value is safe ONLY
# at the closing speed DEFAULT_TRAJECTORY_ENV configures above, and must NOT
# be added to .env's permanent CBF baseline.
REQUIRED_CBF_ENV: dict[str, str] = {
    "SWARM_CBF_DESIGN_MARGIN_BUFFER_M": "0.4",
}

# UAV-02's leg takes ~104s at 0.15 m/s, which does not fit the default 120s
# (SWARM_FIRST_FLIGHT_MAX_DURATION_S) alongside arm/takeoff/land overhead
# (~90-100s measured on prior two-vehicle flights). Also required in .env.
REQUIRED_ENVELOPE_ENV: dict[str, str] = {
    "SWARM_FIRST_FLIGHT_MAX_DURATION_S": "240",
}


# ENGINEERING GUARD, not a derived physical constant. Bounds chosen from
# measured numbers rather than picked round:
#
#   floor -- shared ENU altitude is GPS-derived while the driver's own
#     altitude_m is LOCAL_POSITION_NED, and the two disagree by a measured
#     ~0.91 m at hover (geometry-fix flight: altitude_reference_m 8.973 vs
#     entry_altitude_m 9.88; the 2026-08-10 trace flight root-caused this as
#     GPS/baro fusion lag, not a bug). Add hover wander and a limit below
#     ~1.2 m would abort healthy flights.
#   ceiling -- must stay far under the scale of a real frame error. The
#     vehicles spawn 5.39 m apart and a wrong ENU origin would be tens of
#     metres out, so 1.5 m still refuses those decisively.
#
# It deliberately permits brief velocity saturation (with k_p 0.6 and
# v_ref 1.5 m/s, saturation begins around 0.83 m of aligned error). That is
# acceptable: the cap is FlightEnvelope's own limit and CBF still bounds the
# result. This guard exists to catch a WRONG FRAME, not to enforce perfect
# alignment.
INITIAL_ERROR_LIMIT_DEFAULT_M = 1.5


def initial_error_limit_m() -> float:
    """Operator override for the alignment limit, clamped to a sane band."""
    try:
        value = float(
            os.environ.get(
                "SWARM_TRAJECTORY_INITIAL_ERROR_MAX_M",
                str(INITIAL_ERROR_LIMIT_DEFAULT_M),
            )
        )
    except (TypeError, ValueError):
        return INITIAL_ERROR_LIMIT_DEFAULT_M
    if not math.isfinite(value) or value <= 0.0:
        return INITIAL_ERROR_LIMIT_DEFAULT_M
    return min(value, 50.0)


@dataclass(frozen=True)
class AlignmentResult:
    drone_id: str
    actual_start_enu_m: tuple[float, float, float] | None
    reference_start_enu_m: tuple[float, float, float] | None
    initial_tracking_error_m: float | None
    ok: bool
    reason: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "drone_id": self.drone_id,
            "actual_start_enu": (
                [round(v, 3) for v in self.actual_start_enu_m]
                if self.actual_start_enu_m is not None
                else None
            ),
            "reference_start_enu": (
                [round(v, 3) for v in self.reference_start_enu_m]
                if self.reference_start_enu_m is not None
                else None
            ),
            "initial_tracking_error_m": (
                round(self.initial_tracking_error_m, 3)
                if self.initial_tracking_error_m is not None
                else None
            ),
            "ok": self.ok,
            "reason": self.reason,
        }


def _finite_vector3(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        vector = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return None
    return vector if all(math.isfinite(item) for item in vector) else None  # type: ignore[return-value]


def check_initial_frame_alignment(
    snapshot: dict[str, dict[str, Any]], *, limit_m: float
) -> tuple[tuple[AlignmentResult, ...], str | None]:
    """Compares where each vehicle actually is against where its trajectory
    says it starts, both in the SHARED ENU frame the trajectory controller
    itself consumes -- never LOCAL_POSITION_NED, whose origin is per-vehicle
    and not the frame any of this is expressed in.

    Pure and snapshot-driven so the decision is testable without a stack.
    Returns per-vehicle results plus an abort reason (None when all pass).

    Boundary semantics: an error EQUAL to the limit passes; only a strictly
    greater error fails.
    """
    results: list[AlignmentResult] = []
    failures: list[str] = []
    for drone_id in DRONE_IDS:
        state = snapshot.get(drone_id) or {}

        claimed = state.get("companion_drone_id")
        if claimed != drone_id:
            results.append(
                AlignmentResult(
                    drone_id, None, None, None, False,
                    f"identity_mismatch:{claimed!r}",
                )
            )
            failures.append(f"trajectory_identity_mismatch:{drone_id}:{claimed!r}")
            continue

        reference = _finite_vector3(state.get("trajectory_reference_start_enu_m"))
        if reference is None:
            results.append(
                AlignmentResult(drone_id, None, None, None, False, "trajectory_not_configured")
            )
            failures.append(f"trajectory_not_configured:{drone_id}")
            continue

        actual = _finite_vector3(state.get("own_position_enu_m"))
        if actual is None:
            results.append(
                AlignmentResult(
                    drone_id, None, reference, None, False,
                    "shared_enu_position_unavailable",
                )
            )
            failures.append(f"trajectory_initial_position_unavailable:{drone_id}")
            continue

        error = math.sqrt(sum((actual[i] - reference[i]) ** 2 for i in range(3)))
        within = error <= limit_m
        results.append(
            AlignmentResult(
                drone_id, actual, reference, error, within,
                "aligned" if within else "initial_error_exceeds_limit",
            )
        )
        if not within:
            failures.append(
                f"trajectory_initial_position_mismatch:{drone_id}:"
                f"{error:.3f}m>{limit_m:.3f}m"
            )

    return tuple(results), (",".join(failures) if failures else None)


class FlightAbort(Exception):
    """Raised on any failed verification. Always leads to POSCTL + land, both vehicles."""


class Flight:
    def __init__(self, hold_s: float, dry_run: bool) -> None:
        self.hold_s = hold_s
        self.dry_run = dry_run
        self.envelope = FlightEnvelope.from_environment()
        self.warmup = WarmupContract()
        self.records: list[dict[str, Any]] = []
        self.started = time.monotonic()
        self.commands: list[str] = []

    # -- telemetry --------------------------------------------------------

    def telemetry(self) -> dict[str, Any]:
        result = subprocess.run(
            ["curl", "-fsS", "-m", "5", API_URL],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            raise FlightAbort("telemetry_unavailable")
        return json.loads(result.stdout)

    def vehicle(
        self, drone_id: str, payload: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        payload = payload or self.telemetry()
        drone = payload["drones"][drone_id]
        stream = payload["tracking_pose_streams"][drone_id]
        safety = stream.get("companion_safety") or {}
        local = drone.get("local_position") or {}
        cbf = safety.get("cbf") or {}
        return {
            "drone_id": drone_id,
            # Which vehicle the companion pipeline BELIEVES it is evaluating.
            # Compared against drone_id by the alignment gate: a stream
            # serving one vehicle's state under another's key would otherwise
            # let every other check pass against the wrong aircraft.
            "companion_drone_id": safety.get("drone_id"),
            "own_position_enu_m": safety.get("own_position_enu_m"),
            "trajectory_reference_start_enu_m": safety.get(
                "trajectory_reference_start_enu_m"
            ),
            "armed": drone["status"].get("armed"),
            "nav_state": drone["status"].get("nav_state"),
            "failsafe": drone["status"].get("failsafe"),
            "altitude_m": (
                None if local.get("z_down_m") is None else -float(local["z_down_m"])
            ),
            "vertical_velocity_m_s": (
                None if local.get("vz_m_s") is None else -float(local["vz_m_s"])
            ),
            "speed_m_s": (
                None
                if local.get("vx_m_s") is None
                else (local["vx_m_s"] ** 2 + local["vy_m_s"] ** 2) ** 0.5
            ),
            "offboard_signal_lost": (drone.get("failsafe_flags") or {}).get(
                "offboard_control_signal_lost"
            ),
            "sender": safety.get("active_offboard_sender") or {},
            "frame": safety.get("active_offboard_frame") or {},
            "conditions": safety.get("active_offboard_conditions") or [],
            "output_velocity_enu_m_s": safety.get("output_velocity_enu_m_s"),
            "nominal_reason": safety.get("nominal_reason"),
            "nominal_position_error_m": safety.get("nominal_position_error_m"),
            "station_keeping": safety.get("station_keeping"),
            "cbf_active": cbf.get("active"),
            "cbf_reason": cbf.get("reason"),
            "cbf_intervened": safety.get("intervened"),
            # Own measurement of the pair's separation margin -- the same
            # value CBF itself filters against (see two_uav_active_flight.py
            # for why this is not re-derived from local_position instead).
            "cbf_minimum_margin_m": cbf.get("minimum_margin_m"),
        }

    def snapshot(self) -> dict[str, dict[str, Any]]:
        payload = self.telemetry()
        return {drone_id: self.vehicle(drone_id, payload) for drone_id in DRONE_IDS}

    def log(self, step: str, **fields: Any) -> None:
        record = {"t_s": round(time.monotonic() - self.started, 2), "step": step, **fields}
        self.records.append(record)
        print(json.dumps(record), flush=True)

    # -- commands ---------------------------------------------------------

    def commander(self, drone_id: str, *arguments: str) -> str:
        command = [COMMANDER, "--instance", PX4_INSTANCE[drone_id], *arguments]
        self.commands.append(f"{drone_id}:" + " ".join(arguments))
        if self.dry_run:
            self.log("WOULD_COMMAND", drone_id=drone_id, command=" ".join(arguments))
            return ""
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        self.log(
            "COMMAND", drone_id=drone_id, command=" ".join(arguments), rc=result.returncode
        )
        return result.stdout

    def await_vehicle_state(
        self,
        drone_id: str,
        description: str,
        predicate,
        timeout_s: float,
        poll_s: float = 0.5,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        last: dict[str, Any] = {}
        while time.monotonic() < deadline:
            snapshot = self.snapshot()
            self.guard(snapshot)
            last = snapshot[drone_id]
            if predicate(last):
                self.log("CONFIRMED", drone_id=drone_id, what=description, **self.brief(last))
                return last
            time.sleep(poll_s)
        raise FlightAbort(f"timeout_waiting_for:{drone_id}:{description}")

    @staticmethod
    def brief(state: dict[str, Any]) -> dict[str, Any]:
        return {
            "armed": state["armed"],
            "nav_state": state["nav_state"],
            "alt_m": None if state["altitude_m"] is None else round(state["altitude_m"], 2),
            "decision": state["frame"].get("decision"),
            "tx": state["sender"].get("transmit_count"),
            "nominal_reason": state["nominal_reason"],
            "tracking_error_m": (
                None
                if state["nominal_position_error_m"] is None
                else round(state["nominal_position_error_m"], 2)
            ),
            "cbf_margin_m": (
                None
                if state["cbf_minimum_margin_m"] is None
                else round(state["cbf_minimum_margin_m"], 2)
            ),
        }

    # -- continuous safety ------------------------------------------------

    def guard(self, snapshot: dict[str, dict[str, Any]]) -> None:
        """Same shape as two_uav_active_flight.py's guard: per-vehicle checks
        for both, joint CBF margin check. Tracking error is NOT guarded here
        -- CBF is the safety layer; tracking error is instrumentation."""
        if time.monotonic() - self.started > self.envelope.maximum_test_duration_s:
            raise FlightAbort("maximum_test_duration_exceeded")
        for drone_id, state in snapshot.items():
            if state["failsafe"]:
                raise FlightAbort(f"px4_failsafe:{drone_id}")
            latched = state["sender"].get("latched_abort")
            if latched:
                raise FlightAbort(f"sender_latched:{drone_id}:{latched}")
            altitude = state["altitude_m"]
            if altitude is not None and altitude > self.envelope.geofence_max_enu_m[2]:
                raise FlightAbort(f"geofence_altitude_exceeded:{drone_id}")
            speed = state["speed_m_s"]
            if (
                speed is not None
                and speed > self.envelope.maximum_horizontal_velocity_m_s + 0.5
            ):
                raise FlightAbort(f"horizontal_speed_exceeded:{drone_id}:{speed:.2f}")
        margins = [
            state["cbf_minimum_margin_m"]
            for state in snapshot.values()
            if state["cbf_minimum_margin_m"] is not None
        ]
        if margins and min(margins) < 0.0:
            raise FlightAbort(f"cbf_separation_violated:{min(margins):.2f}")

    def verify_initial_frame_alignment(self, snapshot: dict[str, dict[str, Any]]) -> None:
        """TRAJECTORY_INITIAL_FRAME_ALIGNMENT_VALIDATED.

        Reports every vehicle's numbers before deciding, so a refusal is
        diagnosable from the log alone rather than needing a re-run.
        """
        limit_m = initial_error_limit_m()
        results, failure = check_initial_frame_alignment(snapshot, limit_m=limit_m)
        self.log(
            "INITIAL_FRAME_ALIGNMENT",
            limit_m=limit_m,
            **{result.drone_id: result.as_dict() for result in results},
        )
        if failure:
            raise FlightAbort(failure)
        self.log("TRAJECTORY_INITIAL_FRAME_ALIGNMENT_VALIDATED", limit_m=limit_m)

    def await_trajectory_mode_engaged(self, timeout_s: float = 8.0) -> dict[str, dict[str, Any]]:
        """Trajectory mode is a TRANSITION to wait for, not an instantaneous
        property of having requested OFFBOARD.

        Measured on the first real attempt: this driver confirms OFFBOARD from
        `drones[].status.nav_state`, but the companion derives its own
        `station_keeping` -- which is what actually activates the trajectory
        nominal -- from the bridge worker's `last_px4_main_mode`, fed by a
        different MAVLink stream on its own cadence. UAV-01 had caught up
        0.52 s after its mode request and reported `tracking_trajectory`;
        UAV-02, checked 0.52 s after its own request, had not, and a
        single-shot check aborted a flight that was in fact healthy.

        Polling instead of sleeping keeps every guard live throughout, and a
        genuinely unconfigured stack still fails: its nominal_reason stays a
        formation one forever and the timeout fires. The stronger check
        against that -- a published trajectory reference -- has already run
        before OFFBOARD, in the alignment gate.
        """
        deadline = time.monotonic() + timeout_s
        snapshot: dict[str, dict[str, Any]] = {}
        while time.monotonic() < deadline:
            snapshot = self.snapshot()
            self.guard(snapshot)
            pending = [
                drone_id
                for drone_id, state in snapshot.items()
                if state["nominal_reason"] not in TRAJECTORY_NOMINAL_REASONS
            ]
            if not pending:
                return snapshot
            time.sleep(0.25)
        self.verify_trajectory_mode_engaged(snapshot)
        return snapshot

    def verify_trajectory_mode_engaged(self, snapshot: dict[str, dict[str, Any]]) -> None:
        """Checked once, right after both vehicles enter OFFBOARD. Companion-
        side trajectory selection is environment-driven and set BEFORE this
        process starts (see module docstring) -- if an operator forgot to set
        it, the live system silently falls back to formation/altitude-hold,
        which would otherwise produce a flight that "passes" while measuring
        nothing this milestone cares about."""
        misconfigured = [
            drone_id
            for drone_id, state in snapshot.items()
            if state["nominal_reason"] not in TRAJECTORY_NOMINAL_REASONS
        ]
        if misconfigured:
            raise FlightAbort(
                "trajectory_not_configured:" + ",".join(misconfigured)
                + " -- see DEFAULT_TRAJECTORY_ENV / --print-env"
            )

    # -- sequence -----------------------------------------------------------

    def precheck(self) -> None:
        snapshot = self.snapshot()
        failures: list[str] = []
        for drone_id, state in snapshot.items():
            sender = state["sender"]
            if state["armed"] is not False:
                failures.append(f"{drone_id}_not_disarmed")
            if state["offboard_signal_lost"] is not False:
                failures.append(f"{drone_id}_px4_does_not_see_the_setpoint_stream")
            if not sender.get("transmit_sink_attached"):
                failures.append(f"{drone_id}_no_transmit_sink")
            if not sender.get("explicit_opt_in"):
                failures.append(f"{drone_id}_no_explicit_opt_in")
            duration = sender.get("stream_duration_s")
            if duration is None or duration < self.warmup.minimum_duration_s:
                failures.append(f"{drone_id}_warmup_duration_insufficient:{duration}")
            if sender.get("transmit_count", 0) < self.warmup.minimum_valid_samples:
                failures.append(f"{drone_id}_warmup_samples_insufficient")
            if sender.get("max_transmit_gap_s", 9.9) > self.warmup.maximum_gap_s:
                failures.append(f"{drone_id}_warmup_gap:{sender.get('max_transmit_gap_s')}")
            if state["conditions"]:
                failures.append(f"{drone_id}_abort_conditions_active:{state['conditions']}")
        if failures:
            raise FlightAbort("precheck:" + ",".join(failures))
        self.log(
            "PRECHECK_PASS",
            **{drone_id: self.brief(state) for drone_id, state in snapshot.items()},
        )

    def run(self) -> str:
        self.log(
            "START",
            envelope_hover_m=self.envelope.hover_altitude_m,
            max_duration_s=self.envelope.maximum_test_duration_s,
            expected_trajectory_env=DEFAULT_TRAJECTORY_ENV,
            expected_cbf_env=REQUIRED_CBF_ENV,
            expected_envelope_env=REQUIRED_ENVELOPE_ENV,
        )
        self.precheck()

        self.commander(UAV_01, "arm")
        self.await_vehicle_state(UAV_01, "armed", lambda s: s["armed"] is True, timeout_s=10.0)
        self.commander(UAV_02, "arm")
        self.await_vehicle_state(UAV_02, "armed", lambda s: s["armed"] is True, timeout_s=10.0)

        def hover_reached(state: dict[str, Any]) -> bool:
            return (
                state["altitude_m"] is not None
                and state["altitude_m"] > 7.0
                and abs(state["vertical_velocity_m_s"] or 9.9) < 0.3
            )

        self.commander(UAV_01, "takeoff")
        self.await_vehicle_state(UAV_01, "hover_reached", hover_reached, timeout_s=45.0)
        self.commander(UAV_02, "takeoff")
        self.await_vehicle_state(UAV_02, "hover_reached", hover_reached, timeout_s=45.0)

        # THE GATE. Runs while BOTH vehicles hover under PX4's own POSCTL,
        # before either hands control to the companion. Checking after
        # OFFBOARD would be too late to be preventive: trajectory mode only
        # activates with station-keeping, so a mismatched p_ref(0) would
        # already be commanding a saturated velocity toward the wrong place
        # by the time the first sample arrived.
        #
        # Both vehicles are armed and hovering at this point, which cannot be
        # avoided -- on the ground their shared-ENU altitude is ~0 against a
        # ~9 m reference, so every check would fail spuriously. Arming alone
        # grants the companion nothing; OFFBOARD is where authority actually
        # transfers, and that is what this gate stands in front of, for BOTH
        # vehicles at once.
        self.verify_initial_frame_alignment(self.snapshot())

        self.commander(UAV_01, "mode", "offboard")
        self.await_vehicle_state(
            UAV_01,
            "offboard_engaged",
            lambda s: s["nav_state"] == NAV_STATE_OFFBOARD,
            timeout_s=10.0,
        )
        self.commander(UAV_02, "mode", "offboard")
        self.await_vehicle_state(
            UAV_02,
            "offboard_engaged",
            lambda s: s["nav_state"] == NAV_STATE_OFFBOARD,
            timeout_s=10.0,
        )

        entry = self.await_trajectory_mode_engaged()
        self.log(
            "TRAJECTORY_MODE_CONFIRMED",
            **{drone_id: self.brief(state) for drone_id, state in entry.items()},
        )

        deadline = time.monotonic() + self.hold_s
        samples: dict[str, list[dict[str, Any]]] = {drone_id: [] for drone_id in DRONE_IDS}
        while time.monotonic() < deadline:
            snapshot = self.snapshot()
            self.guard(snapshot)
            for drone_id, state in snapshot.items():
                if state["nav_state"] != NAV_STATE_OFFBOARD:
                    raise FlightAbort(f"px4_exited_offboard_unexpectedly:{drone_id}")
                samples[drone_id].append(state)
            time.sleep(0.5)

        for drone_id in DRONE_IDS:
            drone_samples = samples[drone_id]
            errors = [
                s["nominal_position_error_m"]
                for s in drone_samples
                if s["nominal_position_error_m"] is not None
            ]
            speeds = [s["speed_m_s"] for s in drone_samples if s["speed_m_s"] is not None]
            margins = [
                s["cbf_minimum_margin_m"]
                for s in drone_samples
                if s["cbf_minimum_margin_m"] is not None
            ]
            interventions = sum(1 for s in drone_samples if s["cbf_intervened"])
            self.log(
                "TRAJECTORY_HOLD_COMPLETE",
                drone_id=drone_id,
                seconds=self.hold_s,
                max_tracking_error_m=round(max(errors), 3) if errors else None,
                final_tracking_error_m=round(errors[-1], 3) if errors else None,
                max_horizontal_speed_m_s=round(max(speeds), 3) if speeds else None,
                min_cbf_margin_m=round(min(margins), 3) if margins else None,
                cbf_intervention_rate=(
                    round(interventions / len(drone_samples), 3) if drone_samples else None
                ),
                total_samples=len(drone_samples),
                nominal_reasons=sorted({str(s["nominal_reason"]) for s in drone_samples}),
                reached_trajectory_end=any(
                    s["nominal_reason"] == "trajectory_reached" for s in drone_samples
                ),
            )

        self.commander(UAV_01, "mode", "posctl")
        self.await_vehicle_state(
            UAV_01,
            "posctl_restored",
            lambda s: s["nav_state"] == NAV_STATE_POSCTL,
            timeout_s=10.0,
        )
        self.commander(UAV_02, "mode", "posctl")
        self.await_vehicle_state(
            UAV_02,
            "posctl_restored",
            lambda s: s["nav_state"] == NAV_STATE_POSCTL,
            timeout_s=10.0,
        )

        self.commander(UAV_01, "land")
        self.await_vehicle_state(
            UAV_01, "disarmed", lambda s: s["armed"] is False, timeout_s=90.0
        )
        self.commander(UAV_02, "land")
        self.await_vehicle_state(
            UAV_02, "disarmed", lambda s: s["armed"] is False, timeout_s=90.0
        )
        return "FLIGHT_PASS"

    def abort(self, reason: str) -> None:
        self.log("ABORT", reason=reason)
        for drone_id in DRONE_IDS:
            try:
                self.commander(drone_id, "mode", "posctl")
            except Exception as error:  # noqa: BLE001 - abort must not raise
                self.log("ABORT_INCOMPLETE", drone_id=drone_id, stage="posctl", error=str(error))
        time.sleep(1.0)
        for drone_id in DRONE_IDS:
            try:
                state = self.vehicle(drone_id)
                if state["armed"]:
                    self.commander(drone_id, "land")
            except Exception as error:  # noqa: BLE001 - abort must not raise
                self.log("ABORT_INCOMPLETE", drone_id=drone_id, stage="land", error=str(error))
        self.log(
            "ABORT_DONE",
            note="POSCTL is the strongest implemented action; a human must take it from here",
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hold-s", type=float, default=110.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output", default="artifacts/two_uav_crossing_trajectory_flight.json")
    parser.add_argument(
        "--print-env",
        action="store_true",
        help="Print the .env block this driver's guards assume, then exit.",
    )
    arguments = parser.parse_args()

    if arguments.print_env:
        for key, value in {**DEFAULT_TRAJECTORY_ENV, **REQUIRED_CBF_ENV, **REQUIRED_ENVELOPE_ENV}.items():
            print(f"{key}={value}")
        return 0

    flight = Flight(hold_s=arguments.hold_s, dry_run=arguments.dry_run)
    verdict = "FLIGHT_FAIL"
    try:
        verdict = flight.run()
    except FlightAbort as error:
        flight.abort(str(error))
        verdict = f"FLIGHT_ABORTED:{error}"
    except KeyboardInterrupt:
        flight.abort("operator_interrupt")
        verdict = "FLIGHT_ABORTED:operator_interrupt"

    try:
        final = {drone_id: flight.vehicle(drone_id) for drone_id in DRONE_IDS}
        final_state = {drone_id: Flight.brief(state) for drone_id, state in final.items()}
        sender = {drone_id: state["sender"] for drone_id, state in final.items()}
    except FlightAbort as error:
        final_state = {"error": str(error)}
        sender = {}
    result = {
        "verdict": verdict,
        "records": flight.records,
        "commands_issued": flight.commands,
        "final_state": final_state,
        "sender": sender,
        "expected_trajectory_env": DEFAULT_TRAJECTORY_ENV,
        "expected_cbf_env": REQUIRED_CBF_ENV,
        "expected_envelope_env": REQUIRED_ENVELOPE_ENV,
    }
    output = Path(arguments.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print()
    print(
        json.dumps(
            {"verdict": verdict, "final": result["final_state"]},
            indent=2,
        )
    )
    return 0 if verdict == "FLIGHT_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
