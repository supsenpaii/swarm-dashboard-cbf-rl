import json
from pathlib import Path

import pytest

from core_range_priority_collection_audit import (
    bin_contains,
    load_effective_manifest,
    percentile,
)


def test_percentile_is_deterministic_and_interpolated() -> None:
    assert percentile([4.0, 1.0, 3.0, 2.0], 0.5) == 2.5
    assert percentile([1.0, 2.0, 3.0], 0.9) == pytest.approx(2.8)


def test_bin_assignment_is_left_closed_right_open() -> None:
    assert bin_contains("10-11m", 10.0)
    assert bin_contains("10-11m", 10.999)
    assert not bin_contains("10-11m", 11.0)


def test_amendment_source_checksum_mismatch_aborts(tmp_path: Path) -> None:
    manifest = tmp_path / "manifest.json"
    amendment = tmp_path / "amendment.json"
    manifest.write_text(
        json.dumps({"collection_id": "c1", "scenarios": []}), encoding="utf-8"
    )
    amendment.write_text(
        json.dumps(
            {
                "parent_collection_id": "c1",
                "parent_manifest_sha256": "not-the-source-checksum",
                "changes": {},
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="collection_amendment_checksum_mismatch"):
        load_effective_manifest(manifest, amendment)
