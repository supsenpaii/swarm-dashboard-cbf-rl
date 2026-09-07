"""Detects a MavlinkWorker loop that has stopped advancing, from outside it.

`offboard_abort_conditions.py`'s REACHABILITY section names the gap this
closes: `mavlink_connection_loss` and `companion_safety_loop_stops` are
derived inside `evaluate_companion_safety()`, so a worker whose loop has
actually frozen -- the blocking MAVLink read never returning -- can never run
the code that would report its own freeze. Measured directly: SIGSTOP on one
UAV's PX4 froze that worker's `evaluated_monotonic_s` at 28484.764 for the
full 7 s of the fault while the peer kept evaluating at 20 Hz.

This module is the watcher outside it. It is handed each worker's last known
evaluation timestamp plus a *separately sourced* current time -- the caller's
own clock, never anything read from the worker -- so a frozen worker cannot
also freeze the check that detects it. That separation is the entire
mechanism; nothing here talks to MAVLink, PX4, or a vehicle.

Scope, stated plainly: this restores observability, not protection. PX4's own
COM_OF_LOSS_T already falls back to Position mode once a worker actually stops
streaming setpoints -- a vehicle-side safeguard this module does not change
and does not need to. What was missing was a process able to say "worker X
has not advanced in N seconds" instead of staying silent. The
`companion_safety_loop_stops` / `mavlink_connection_loss` ABORT_MATRIX
conditions themselves remain unreachable from a genuinely frozen worker's own
sender, for the same reason they always were: that sender is code the frozen
loop cannot run either.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class WorkerLivenessSample:
    """One worker's self-reported progress, read at one instant.

    `companion_safety_enabled` distinguishes "never evaluated because the
    feature is off" (no peer-state transport, so evaluate_companion_safety
    never runs at all) from "stopped evaluating". Both leave
    `last_evaluation_monotonic_s` at `None`, but only the second is a stall --
    conflating them would flag every companion-safety-disabled worker as
    stalled forever.
    """

    drone_id: str
    companion_safety_enabled: bool
    last_evaluation_monotonic_s: float | None


@dataclass(frozen=True)
class WorkerLivenessResult:
    drone_id: str
    stalled: bool
    reason: str
    stalled_for_s: float | None


def check_worker_liveness(
    samples: tuple[WorkerLivenessSample, ...],
    *,
    now_monotonic_s: float,
    stall_after_s: float,
) -> tuple[WorkerLivenessResult, ...]:
    """Pure per-worker liveness check. No state, no logging, no I/O.

    `now_monotonic_s` must come from the caller's own clock read, not from
    anything the worker reported -- that is what lets this detect a worker
    whose own clock reads have stopped happening.
    """
    results = []
    for sample in samples:
        if not sample.companion_safety_enabled:
            results.append(
                WorkerLivenessResult(
                    sample.drone_id, False, "companion_safety_disabled", None
                )
            )
            continue
        if sample.last_evaluation_monotonic_s is None:
            # Enabled but has not produced a first evaluation yet (still
            # connecting). Not a stall -- there is nothing to have stalled.
            results.append(
                WorkerLivenessResult(
                    sample.drone_id, False, "awaiting_first_evaluation", None
                )
            )
            continue
        age_s = now_monotonic_s - sample.last_evaluation_monotonic_s
        results.append(
            WorkerLivenessResult(
                sample.drone_id,
                age_s > stall_after_s,
                "evaluation_loop_stalled" if age_s > stall_after_s else "ok",
                age_s,
            )
        )
    return tuple(results)


class WorkerLivenessWatchdog:
    """Edge-triggered wrapper around `check_worker_liveness`.

    A caller polling every second would otherwise re-report the same stall
    every tick for as long as the fault lasts. This tracks each drone's
    previous stalled/healthy state and surfaces only the ticks where it
    changed, including the very first poll (so a fresh boot always announces
    an initial healthy/unknown state rather than staying silent until the
    first fault).
    """

    def __init__(self, stall_after_s: float) -> None:
        self._stall_after_s = stall_after_s
        self._previously_stalled: dict[str, bool] = {}

    def poll(
        self,
        samples: tuple[WorkerLivenessSample, ...],
        *,
        now_monotonic_s: float,
    ) -> tuple[tuple[WorkerLivenessResult, ...], tuple[WorkerLivenessResult, ...]]:
        """Returns (all results this tick, results whose stalled state changed)."""
        results = check_worker_liveness(
            samples,
            now_monotonic_s=now_monotonic_s,
            stall_after_s=self._stall_after_s,
        )
        transitions = []
        for result in results:
            previous = self._previously_stalled.get(result.drone_id)
            if previous != result.stalled:
                transitions.append(result)
            self._previously_stalled[result.drone_id] = result.stalled
        return results, tuple(transitions)
