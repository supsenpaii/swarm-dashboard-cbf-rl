import pytest

from range_v2_labels import derive_distance_label


@pytest.mark.parametrize(
    ("distance_m", "zone", "band", "transition", "bin_name"),
    (
        (2.69, "near", "near_certain", False, "unknown"),
        (2.70, "near", "near_core_transition", True, "unknown"),
        (2.99, "near", "near_core_transition", True, "unknown"),
        (3.00, "core", "near_core_transition", True, "3-4m"),
        (3.29, "core", "near_core_transition", True, "3-4m"),
        (3.30, "core", "core_certain", False, "3-4m"),
        (11.70, "core", "core_certain", False, "11-12m"),
        (11.71, "core", "core_far_transition", True, "11-12m"),
        (12.00, "core", "core_far_transition", True, "11-12m"),
        (12.01, "far", "core_far_transition", True, "unknown"),
        (12.30, "far", "core_far_transition", True, "unknown"),
        (12.31, "far", "far_certain", False, "unknown"),
    ),
)
def test_frozen_zone_and_transition_boundaries(
    distance_m,
    zone,
    band,
    transition,
    bin_name,
):
    label = derive_distance_label(
        distance_m,
        ground_truth_trustworthy=True,
    )
    assert label.distance_zone_gt == zone
    assert label.boundary_band == band
    assert label.is_boundary_transition is transition
    assert label.core_bin == bin_name


def test_transition_changes_weight_not_canonical_core_label():
    transition = derive_distance_label(3.1, ground_truth_trustworthy=True)
    certain = derive_distance_label(8.0, ground_truth_trustworthy=True)
    assert transition.distance_zone_gt == certain.distance_zone_gt == "core"
    assert transition.hard_zone_label_eligible is False
    assert transition.core_residual_transition_weight_factor == 0.5
    assert certain.hard_zone_label_eligible is True
    assert certain.core_residual_transition_weight_factor == 1.0


@pytest.mark.parametrize("distance_m", (None, float("nan"), -1.0, 8.0))
def test_untrusted_ground_truth_is_unknown(distance_m):
    label = derive_distance_label(
        distance_m,
        ground_truth_trustworthy=False,
    )
    assert label.distance_zone_gt == "unknown"
    assert label.is_boundary_transition is None
    assert label.hard_zone_label_eligible is False
    assert label.core_residual_label_eligible is False


def test_distance_label_does_not_contain_validity_label():
    label = derive_distance_label(2.99, ground_truth_trustworthy=True)
    assert "validity_gt" not in label.as_dict()

