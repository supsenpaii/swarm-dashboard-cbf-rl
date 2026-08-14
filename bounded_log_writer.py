"""Bounded binary-safe stdout/stderr sink for supervised runtime processes."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import sys
import time


IDLE_COALESCE_S = 0.05


def stream_to_rotating_files(
    source,
    path: Path,
    *,
    maximum_bytes: int,
    chunk_bytes: int = 64 * 1024,
    fsync_on_rotate: bool = True,
    fsync_trace_path: Path | None = None,
) -> dict[str, int]:
    maximum = max(1024, int(maximum_bytes))
    chunk_size = max(256, int(chunk_bytes))
    path.parent.mkdir(parents=True, exist_ok=True)
    rotated = path.with_suffix(path.suffix + ".1")
    written = 0
    rotations = 0
    fsync_count = 0
    fsync_trace = None
    fsync_writer = None
    if fsync_trace_path is not None:
        fsync_trace_path.parent.mkdir(parents=True, exist_ok=True)
        fsync_trace = fsync_trace_path.open("w", newline="", encoding="utf-8")
        fsync_writer = csv.DictWriter(
            fsync_trace,
            fieldnames=(
                "wall_clock_s", "monotonic_s", "event", "duration_s",
                "rotation_count", "bytes_in_segment",
            ),
        )
        fsync_writer.writeheader()
        fsync_trace.flush()

    def sync(event: str) -> None:
        nonlocal fsync_count
        started = time.monotonic()
        os.fsync(stream.fileno())
        finished = time.monotonic()
        fsync_count += 1
        if fsync_writer is not None and fsync_trace is not None:
            fsync_writer.writerow({
                "wall_clock_s": round(time.time(), 6),
                "monotonic_s": round(finished, 6),
                "event": event,
                "duration_s": round(finished - started, 9),
                "rotation_count": rotations,
                "bytes_in_segment": written,
            })
            fsync_trace.flush()

    stream = path.open("wb")
    # `read`, not `read1`, would block until a full chunk_bytes had accumulated
    # -- so a process that logs only on notable events (the mavlink bridge
    # emits a few KB across a whole session) wrote NOTHING readable until it
    # exited. Measured 2026-08-11: a 20-minute bridge run left a 0-byte log
    # while the process was alive. That makes the log useless for watching a
    # run, and loses it entirely if the run is killed rather than shut down --
    # which is exactly the case a log is most wanted for. `read1` returns
    # whatever has arrived, still bounded by chunk_size.
    read_available = getattr(source, "read1", source.read)
    try:
        while True:
            chunk = read_available(chunk_size)
            if not chunk:
                break
            if not isinstance(chunk, bytes):
                raise TypeError("bounded_log_source_must_be_binary")
            offset = 0
            while offset < len(chunk):
                remaining = maximum - written
                if remaining <= 0:
                    stream.flush()
                    if fsync_on_rotate:
                        sync("rotation")
                    stream.close()
                    if rotated.exists():
                        rotated.unlink()
                    os.replace(path, rotated)
                    stream = path.open("wb")
                    written = 0
                    rotations += 1
                    remaining = maximum
                selected = chunk[offset : offset + remaining]
                stream.write(selected)
                written += len(selected)
                offset += len(selected)
            # Flush only; the fsync stays on rotation. Without this the
            # 8 KB file buffer holds the same output back that read1 just
            # stopped the pipe from holding.
            stream.flush()
            if len(chunk) * 16 < chunk_size:
                # read1 returns the instant a single byte lands, so on a
                # continuous stream this loop spins on tiny chunks: measured
                # 2026-08-11, the two PX4 writers (an ANSI-escape firehose)
                # burned 96% of a core EACH, starving the very simulator
                # whose logs they were writing. Pausing only when the last
                # read came back nearly empty is self-regulating -- a busy
                # stream refills to a full chunk during the pause and never
                # sleeps again, an idle one costs one wakeup per interval --
                # and it bounds write latency to this interval, which is
                # what the read1 change was for.
                time.sleep(IDLE_COALESCE_S)
        stream.flush()
        sync("shutdown")
    finally:
        stream.close()
        if fsync_trace is not None:
            fsync_trace.close()
    return {
        "current_bytes": written,
        "rotations": rotations,
        "fsync_count": fsync_count,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", required=True, type=Path)
    parser.add_argument("--max-bytes", required=True, type=int)
    parser.add_argument(
        "--fsync-on-rotate",
        choices=("true", "false"),
        default="true",
        help="fsync each full segment before rotation; shutdown is always synced",
    )
    parser.add_argument("--fsync-trace", type=Path)
    args = parser.parse_args()
    stream_to_rotating_files(
        sys.stdin.buffer,
        args.path,
        maximum_bytes=args.max_bytes,
        fsync_on_rotate=args.fsync_on_rotate == "true",
        fsync_trace_path=args.fsync_trace,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
