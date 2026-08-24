#!/usr/bin/env python3
"""Download a local map-tile pack so the dashboard renders a map with no internet.

The ground station must work in the field, and it already had to: this machine's
router resolves every openstreetmap.org name to 127.0.0.1, so the live tile layer
was dead on arrival. openstreetmap.de serves the same standard style and is not
blocked; tiles are fetched once, here, then served by our own FastAPI at
/static/tiles.

Defaults cover the ENU origin in .env. Re-run with --lat/--lon for another site.

    python3 fetch_map_tiles.py                     # default site, ~9 MB
    python3 fetch_map_tiles.py --lat 10.8 --lon 106.6 --radius-m 4000
"""
from __future__ import annotations

import argparse
import math
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

STYLE_URL = "https://{s}.tile.openstreetmap.de/{z}/{x}/{y}.png"
SUBDOMAINS = "abc"
USER_AGENT = "sparrow-swarm-dashboard/1.0 (offline tile pack; contact: local operator)"
OUT_ROOT = Path(__file__).parent / "static" / "tiles"


def deg2tile(lat: float, lon: float, zoom: int) -> tuple[int, int]:
    n = 2.0**zoom
    x = int((lon + 180.0) / 360.0 * n)
    rad = math.radians(lat)
    y = int((1.0 - math.asinh(math.tan(rad)) / math.pi) / 2.0 * n)
    return x, y


def bounding_box(lat: float, lon: float, radius_m: float) -> tuple[float, float, float, float]:
    dlat = radius_m / 111_320.0
    dlon = radius_m / (111_320.0 * math.cos(math.radians(lat)))
    return lat - dlat, lon - dlon, lat + dlat, lon + dlon


def fetch(url: str, destination: Path, retries: int = 3) -> bool:
    request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    for attempt in range(retries):
        try:
            with urllib.request.urlopen(request, timeout=20) as response:
                payload = response.read()
        except (urllib.error.URLError, TimeoutError) as error:
            if attempt == retries - 1:
                print(f"  FAIL {url}: {error}", file=sys.stderr)
                return False
            time.sleep(1.5 * (attempt + 1))
            continue

        # A truncated tile renders as a broken image and looks like a code bug
        # months later; refuse it here instead.
        if len(payload) < 100:
            print(f"  FAIL {url}: {len(payload)} bytes", file=sys.stderr)
            return False

        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(payload)
        return True
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lat", type=float, default=47.397971057728974)
    parser.add_argument("--lon", type=float, default=8.546163739800146)
    parser.add_argument("--radius-m", type=float, default=1250.0)
    parser.add_argument("--minimum-zoom", type=int, default=13)
    parser.add_argument("--maximum-zoom", type=int, default=18)
    parser.add_argument("--output", type=Path, default=OUT_ROOT)
    arguments = parser.parse_args()

    south, west, north, east = bounding_box(arguments.lat, arguments.lon, arguments.radius_m)
    print(f"site   {arguments.lat:.6f}, {arguments.lon:.6f}  ±{arguments.radius_m:.0f} m")
    print(f"zoom   {arguments.minimum_zoom}-{arguments.maximum_zoom}")
    print(f"output {arguments.output}")

    wanted: list[tuple[int, int, int]] = []
    for zoom in range(arguments.minimum_zoom, arguments.maximum_zoom + 1):
        x_min, y_max = deg2tile(south, west, zoom)
        x_max, y_min = deg2tile(north, east, zoom)
        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                wanted.append((zoom, x, y))

    print(f"tiles  {len(wanted)}")
    downloaded = skipped = failed = 0
    for index, (zoom, x, y) in enumerate(wanted):
        destination = arguments.output / str(zoom) / str(x) / f"{y}.png"
        if destination.exists() and destination.stat().st_size > 100:
            skipped += 1
            continue
        url = STYLE_URL.format(s=SUBDOMAINS[index % len(SUBDOMAINS)], z=zoom, x=x, y=y)
        if fetch(url, destination):
            downloaded += 1
        else:
            failed += 1
        time.sleep(0.02)
        if (index + 1) % 100 == 0:
            print(f"  {index + 1}/{len(wanted)}  ok={downloaded} cached={skipped} fail={failed}")

    print(f"done   downloaded={downloaded} cached={skipped} failed={failed}")
    return 1 if failed and downloaded == 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
