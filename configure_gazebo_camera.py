#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import os
import re
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class CameraProfile:
    name: str
    sensor_name: str
    width: int
    height: int
    horizontal_fov_rad: float
    update_rate_hz: float

    @classmethod
    def load(cls, path: str | Path) -> "CameraProfile":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        profile = cls(
            name=str(payload["name"]),
            sensor_name=str(payload.get("sensor_name", "camera")),
            width=int(payload["width"]),
            height=int(payload["height"]),
            horizontal_fov_rad=float(payload["horizontal_fov_rad"]),
            update_rate_hz=float(payload.get("update_rate_hz", 30.0)),
        )
        profile.validate()
        return profile

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera dimensions must be positive")
        if not 0.0 < self.horizontal_fov_rad < math.pi:
            raise ValueError("horizontal_fov_rad must be in (0, pi)")
        if self.update_rate_hz <= 0.0:
            raise ValueError("update_rate_hz must be positive")

    def diagnostics(self) -> dict[str, Any]:
        focal = self.width / (
            2.0 * math.tan(self.horizontal_fov_rad / 2.0)
        )
        vertical_fov = 2.0 * math.atan(self.height / (2.0 * focal))
        return {
            **asdict(self),
            "fx": focal,
            "fy": focal,
            "cx": self.width / 2.0,
            "cy": self.height / 2.0,
            "vertical_fov_rad": vertical_fov,
            "horizontal_fov_deg": math.degrees(self.horizontal_fov_rad),
            "vertical_fov_deg": math.degrees(vertical_fov),
            "megapixels": self.width * self.height / 1_000_000.0,
        }


def _sensor_block(text: str, sensor_name: str) -> tuple[int, int, str]:
    pattern = re.compile(
        rf"<sensor\b(?=[^>]*\bname=[\"']{re.escape(sensor_name)}[\"'])"
        rf"[^>]*>.*?</sensor>",
        re.DOTALL,
    )
    matches = list(pattern.finditer(text))
    if len(matches) != 1:
        raise ValueError(
            f"expected exactly one sensor named {sensor_name!r}, "
            f"found {len(matches)}"
        )
    match = matches[0]
    return match.start(), match.end(), match.group(0)


def _tag_value(block: str, tag: str) -> str:
    matches = re.findall(rf"<{tag}>\s*([^<]+?)\s*</{tag}>", block)
    if len(matches) != 1:
        raise ValueError(f"expected one <{tag}> in camera sensor")
    return matches[0].strip()


def inspect_camera_sdf(
    path: str | Path,
    sensor_name: str = "camera",
) -> dict[str, Any]:
    text = Path(path).read_text(encoding="utf-8")
    _, _, block = _sensor_block(text, sensor_name)
    return {
        "sensor_name": sensor_name,
        "width": int(_tag_value(block, "width")),
        "height": int(_tag_value(block, "height")),
        "horizontal_fov_rad": float(_tag_value(block, "horizontal_fov")),
        "update_rate_hz": float(_tag_value(block, "update_rate")),
    }


def _replace_tag(block: str, tag: str, value: str) -> str:
    pattern = re.compile(rf"(<{tag}>\s*)[^<]+?(\s*</{tag}>)")
    updated, count = pattern.subn(rf"\g<1>{value}\g<2>", block)
    if count != 1:
        raise ValueError(f"expected one <{tag}> in camera sensor")
    return updated


def apply_camera_profile(
    sdf_path: str | Path,
    profile: CameraProfile,
) -> dict[str, Any]:
    path = Path(sdf_path)
    text = path.read_text(encoding="utf-8")
    start, end, block = _sensor_block(text, profile.sensor_name)
    updated = _replace_tag(
        block,
        "horizontal_fov",
        f"{profile.horizontal_fov_rad:.9g}",
    )
    updated = _replace_tag(updated, "width", str(profile.width))
    updated = _replace_tag(updated, "height", str(profile.height))
    updated = _replace_tag(
        updated,
        "update_rate",
        f"{profile.update_rate_hz:.9g}",
    )
    output = text[:start] + updated + text[end:]
    if output != text:
        mode = path.stat().st_mode
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as stream:
            stream.write(output)
            temporary_path = Path(stream.name)
        os.chmod(temporary_path, mode)
        os.replace(temporary_path, path)
    return inspect_camera_sdf(path, profile.sensor_name)


def profile_matches_sdf(
    sdf_path: str | Path,
    profile: CameraProfile,
) -> bool:
    current = inspect_camera_sdf(sdf_path, profile.sensor_name)
    return bool(
        current["width"] == profile.width
        and current["height"] == profile.height
        and math.isclose(
            current["horizontal_fov_rad"],
            profile.horizontal_fov_rad,
            rel_tol=0.0,
            abs_tol=1e-9,
        )
        and math.isclose(
            current["update_rate_hz"],
            profile.update_rate_hz,
            rel_tol=0.0,
            abs_tol=1e-9,
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate or apply a metric-depth Gazebo camera profile."
    )
    parser.add_argument("profile", type=Path)
    parser.add_argument("sdf", type=Path)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--check", action="store_true")
    action.add_argument("--apply", action="store_true")
    args = parser.parse_args()

    profile = CameraProfile.load(args.profile)
    before = inspect_camera_sdf(args.sdf, profile.sensor_name)
    if args.apply:
        after = apply_camera_profile(args.sdf, profile)
    else:
        after = before
    matches = profile_matches_sdf(args.sdf, profile)
    print(
        json.dumps(
            {
                "profile": profile.diagnostics(),
                "before": before,
                "after": after,
                "matches": matches,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if matches else 2


if __name__ == "__main__":
    raise SystemExit(main())
