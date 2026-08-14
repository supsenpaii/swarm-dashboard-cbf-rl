"""Frozen, offline-only derived labels for Range Estimation V2.

This module deliberately has no runtime integration.  It maps trustworthy
ground-truth slant range to the canonical labels frozen by R1 while keeping
validity independent from physical distance.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any


SPEC_ID = "range_v2_near_core_far_r1_v001"
DERIVED_LABEL_SCHEMA_VERSION = "range_v2_derived_labels_r2_v001"

NEAR_CORE_BOUNDARY_M = 3.0
CORE_FAR_BOUNDARY_M = 12.0
NEAR_CERTAIN_MAX_M = 2.7
NEAR_CORE_TRANSITION_MAX_M = 3.3
CORE_CERTAIN_MAX_M = 11.7
CORE_FAR_TRANSITION_MAX_M = 12.3

UNKNOWN = "unknown"


@dataclass(frozen=True)
class DerivedDistanceLabel:
    """Distance-only label view; it never asserts Range V2 validity."""

    distance_zone_gt: str
    boundary_band: str
    is_boundary_transition: bool | None
    core_bin: str
    hard_zone_label_eligible: bool
    core_residual_label_eligible: bool
    core_residual_transition_weight_factor: float | None
    label_reason: str

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def _trustworthy_distance(
    distance_m: float | None,
    *,
    ground_truth_trustworthy: bool,
) -> float | None:
    if not ground_truth_trustworthy or distance_m is None:
        return None
    try:
        value = float(distance_m)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value) or value <= 0.0:
        return None
    return value


def canonical_distance_zone(
    distance_m: float | None,
    *,
    ground_truth_trustworthy: bool,
) -> str:
    """Return the exact R1 canonical zone without transition relabeling."""

    distance = _trustworthy_distance(
        distance_m,
        ground_truth_trustworthy=ground_truth_trustworthy,
    )
    if distance is None:
        return UNKNOWN
    if distance < NEAR_CORE_BOUNDARY_M:
        return "near"
    if distance <= CORE_FAR_BOUNDARY_M:
        return "core"
    return "far"


def boundary_band(
    distance_m: float | None,
    *,
    ground_truth_trustworthy: bool,
) -> str:
    distance = _trustworthy_distance(
        distance_m,
        ground_truth_trustworthy=ground_truth_trustworthy,
    )
    if distance is None:
        return UNKNOWN
    if distance < NEAR_CERTAIN_MAX_M:
        return "near_certain"
    if distance < NEAR_CORE_TRANSITION_MAX_M:
        return "near_core_transition"
    if distance <= CORE_CERTAIN_MAX_M:
        return "core_certain"
    if distance <= CORE_FAR_TRANSITION_MAX_M:
        return "core_far_transition"
    return "far_certain"


def core_bin(
    distance_m: float | None,
    *,
    ground_truth_trustworthy: bool,
) -> str:
    distance = _trustworthy_distance(
        distance_m,
        ground_truth_trustworthy=ground_truth_trustworthy,
    )
    if distance is None or not NEAR_CORE_BOUNDARY_M <= distance <= CORE_FAR_BOUNDARY_M:
        return UNKNOWN
    # The last interval is [11, 12]; all others are [lower, upper).
    lower = 11 if distance == CORE_FAR_BOUNDARY_M else int(math.floor(distance))
    lower = max(3, min(11, lower))
    return f"{lower}-{lower + 1}m"


def derive_distance_label(
    distance_m: float | None,
    *,
    ground_truth_trustworthy: bool,
) -> DerivedDistanceLabel:
    zone = canonical_distance_zone(
        distance_m,
        ground_truth_trustworthy=ground_truth_trustworthy,
    )
    band = boundary_band(
        distance_m,
        ground_truth_trustworthy=ground_truth_trustworthy,
    )
    bin_name = core_bin(
        distance_m,
        ground_truth_trustworthy=ground_truth_trustworthy,
    )
    if zone == UNKNOWN:
        return DerivedDistanceLabel(
            distance_zone_gt=UNKNOWN,
            boundary_band=UNKNOWN,
            is_boundary_transition=None,
            core_bin=UNKNOWN,
            hard_zone_label_eligible=False,
            core_residual_label_eligible=False,
            core_residual_transition_weight_factor=None,
            label_reason="ground_truth_unavailable_or_untrusted",
        )

    is_transition = band in {
        "near_core_transition",
        "core_far_transition",
    }
    core_eligible = zone == "core"
    return DerivedDistanceLabel(
        distance_zone_gt=zone,
        boundary_band=band,
        is_boundary_transition=is_transition,
        core_bin=bin_name,
        hard_zone_label_eligible=not is_transition,
        core_residual_label_eligible=core_eligible,
        core_residual_transition_weight_factor=(
            0.5 if core_eligible and is_transition else 1.0 if core_eligible else None
        ),
        label_reason="canonical_range_v2_distance_label",
    )
