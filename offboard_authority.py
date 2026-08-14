"""Single-writer interlock for the PX4 OFFBOARD command path.

Two independent OFFBOARD writers exist in this codebase:

1. the legacy tracking-driven writer in `mavlink_manual_bridge.py`
   (`send_offboard_setpoint` / `send_offboard_attitude_setpoint` /
   `request_px4_main_mode`), and
2. the future companion-safety active sender, which does not exist yet.

Three independent legacy feature flags are not a mutual-exclusion guarantee:
they can all be true at once, and nothing in the type system stops it. This
module replaces "several flags that happen to be false" with one authority
that can name at most one writer, and makes every ambiguous or unparseable
configuration fail closed to `DISABLED`.

The invariant this module exists to enforce:

    active_offboard_writer_count <= 1

`resolve_authority` is the only way to obtain an authority. It never raises:
an unknown string, an empty value, or a legacy flag contradicting the
selected authority all resolve to `DISABLED` with a recorded reason, because
a misconfigured system must not fall back to "whatever was enabled before".
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import os
from typing import Any


class OffboardAuthority(str, Enum):
    DISABLED = "disabled"
    LEGACY_TRACKING = "legacy_tracking"
    COMPANION_SAFETY = "companion_safety"


AUTHORITY_ENV_VAR = "SWARM_OFFBOARD_AUTHORITY"

# Legacy feature flags that can each independently drive the legacy writer.
# Any of them being enabled while authority is COMPANION_SAFETY is a
# contradiction, not a preference to be silently resolved.
LEGACY_WRITER_FLAGS = (
    "SWARM_TRACKING_OFFBOARD_ENABLED",
    "SWARM_ALLOW_RAW_ATTITUDE_OFFBOARD",
    "SWARM_LEGACY_AUTOMATION_ENABLED",
    "SWARM_SAFE_NATIVE_FOLLOW_MODE_REQUEST_ENABLED",
)

# The ONE_UAV_ACTIVE_SITL_FLIGHT vehicle. Hardcoded rather than configurable,
# and duplicated from one_uav_active_readiness on purpose: this module must be
# able to refuse a vehicle without importing the readiness contract, and the
# two constants are pinned equal by test. one_uav_active_readiness.py's own
# gate still checks only this single vehicle -- it is that milestone's
# contract specifically and is unaffected by ACTIVE_FLIGHT_AUTHORIZED_VEHICLES
# below.
FIRST_ACTIVE_FLIGHT_DRONE_ID = "UAV-01"
FIRST_ACTIVE_FLIGHT_SYSTEM_ID = 1

# Every (drone_id, px4_system_id) pair `companion_active_transmit_authorized`
# will ever approve. Hardcoded rather than configurable -- see
# test_authorized_identity_is_hardcoded and
# build_active_offboard_setpoint_sender's docstring for why a vehicle's
# authorization must stay a committed source change, never a runtime flag.
# Added to for TWO_UAV_ACTIVE_SITL_FLIGHT; UAV-02's system id (2) is
# VEHICLES["UAV-02"]["system_id"] in mavlink_manual_bridge.py, duplicated
# rather than imported for the same reason as the pair above.
ACTIVE_FLIGHT_AUTHORIZED_VEHICLES: frozenset[tuple[str, int]] = frozenset(
    {
        (FIRST_ACTIVE_FLIGHT_DRONE_ID, FIRST_ACTIVE_FLIGHT_SYSTEM_ID),
        ("UAV-02", 2),
    }
)


def _flag_enabled(value: str | None) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


@dataclass(frozen=True)
class AuthorityDecision:
    authority: OffboardAuthority
    reason: str
    legacy_writer_permitted: bool
    companion_writer_permitted: bool
    enabled_legacy_flags: tuple[str, ...]

    def __post_init__(self) -> None:
        # The invariant, asserted at construction so no code path can build a
        # decision that permits two writers.
        if self.legacy_writer_permitted and self.companion_writer_permitted:
            raise ValueError("two offboard writers permitted simultaneously")

    @property
    def active_offboard_writer_count(self) -> int:
        return int(self.legacy_writer_permitted) + int(
            self.companion_writer_permitted
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "authority": self.authority.value,
            "reason": self.reason,
            "legacy_writer_permitted": self.legacy_writer_permitted,
            "companion_writer_permitted": self.companion_writer_permitted,
            "active_offboard_writer_count": self.active_offboard_writer_count,
            "enabled_legacy_flags": list(self.enabled_legacy_flags),
        }


def _disabled(reason: str, enabled_flags: tuple[str, ...] = ()) -> AuthorityDecision:
    return AuthorityDecision(
        authority=OffboardAuthority.DISABLED,
        reason=reason,
        legacy_writer_permitted=False,
        companion_writer_permitted=False,
        enabled_legacy_flags=enabled_flags,
    )


def resolve_authority(environment: dict[str, str] | None = None) -> AuthorityDecision:
    """Resolve the single OFFBOARD writer authority. Never raises.

    Fails closed to DISABLED on: absent value, unknown value, or a legacy
    flag enabled while COMPANION_SAFETY is selected. The last case is the
    important one -- it is exactly the ambiguity that three independent flags
    could previously express, and resolving it in favour of either writer
    would be guessing at intent.
    """
    env = os.environ if environment is None else environment
    raw = str(env.get(AUTHORITY_ENV_VAR, "")).strip().lower()
    enabled_flags = tuple(
        name for name in LEGACY_WRITER_FLAGS if _flag_enabled(env.get(name))
    )

    if not raw:
        return _disabled("authority_not_configured", enabled_flags)

    try:
        authority = OffboardAuthority(raw)
    except ValueError:
        return _disabled(f"authority_unknown:{raw}", enabled_flags)

    if authority is OffboardAuthority.DISABLED:
        return _disabled("authority_disabled", enabled_flags)

    if authority is OffboardAuthority.LEGACY_TRACKING:
        if not enabled_flags:
            # Selecting the legacy writer while every legacy flag is off
            # would authorize a writer that cannot actually produce anything.
            return _disabled("legacy_authority_without_any_legacy_flag", enabled_flags)
        return AuthorityDecision(
            authority=authority,
            reason="",
            legacy_writer_permitted=True,
            companion_writer_permitted=False,
            enabled_legacy_flags=enabled_flags,
        )

    # COMPANION_SAFETY
    if enabled_flags:
        return _disabled(
            "companion_authority_conflicts_with_legacy_flags:"
            + ",".join(enabled_flags),
            enabled_flags,
        )
    return AuthorityDecision(
        authority=authority,
        reason="",
        legacy_writer_permitted=False,
        companion_writer_permitted=True,
        enabled_legacy_flags=enabled_flags,
    )


def legacy_writer_permitted(environment: dict[str, str] | None = None) -> bool:
    """Interlock consulted by the legacy writer immediately before transmit.

    Deliberately re-resolved at the call site rather than cached at import:
    the legacy path already reads its own flags at import time, and a stale
    cached authority is precisely the failure this interlock exists to
    prevent.
    """
    return resolve_authority(environment).legacy_writer_permitted


def companion_active_transmit_authorized(
    drone_id: str,
    px4_system_id: int,
    environment: dict[str, str] | None = None,
) -> tuple[bool, str]:
    """Whether the companion active sender may transmit for this vehicle.

    All of the following must hold: the authority names companion_safety,
    system id 0 is refused explicitly because MAVLink treats it as a
    broadcast that every PX4 instance accepts, and the (drone_id,
    px4_system_id) pair is one of the hardcoded ACTIVE_FLIGHT_AUTHORIZED_VEHICLES.
    """
    decision = resolve_authority(environment)
    if not decision.companion_writer_permitted:
        return False, decision.reason or f"authority_is:{decision.authority.value}"
    if int(px4_system_id) == 0:
        return False, "broadcast_system_id_forbidden"
    if (drone_id, int(px4_system_id)) not in ACTIVE_FLIGHT_AUTHORIZED_VEHICLES:
        return False, f"vehicle_not_authorized:{drone_id}:{px4_system_id}"
    return True, ""
