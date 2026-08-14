import time
from io import BytesIO
from pathlib import Path

from bounded_log_writer import stream_to_rotating_files


def test_small_log_is_preserved_exactly(tmp_path: Path) -> None:
    payload = b"line one\nline two\n"
    result = stream_to_rotating_files(
        BytesIO(payload), tmp_path / "runtime.log", maximum_bytes=4096
    )
    assert (tmp_path / "runtime.log").read_bytes() == payload
    assert not (tmp_path / "runtime.log.1").exists()
    assert result == {
        "current_bytes": len(payload), "rotations": 0, "fsync_count": 1,
    }


def test_large_log_is_bounded_and_keeps_latest_two_segments(tmp_path: Path) -> None:
    payload = bytes(range(256)) * 100
    path = tmp_path / "runtime.log"
    result = stream_to_rotating_files(
        BytesIO(payload), path, maximum_bytes=4096, chunk_bytes=777
    )
    rotated = path.with_suffix(".log.1")
    assert path.stat().st_size <= 4096
    assert rotated.stat().st_size == 4096
    retained = rotated.read_bytes() + path.read_bytes()
    assert retained == payload[-len(retained):]
    assert result["rotations"] >= 1
    assert result["fsync_count"] == result["rotations"] + 1


def test_binary_nul_bytes_are_not_treated_as_text(tmp_path: Path) -> None:
    payload = b"prefix\x00middle\xffsuffix"
    path = tmp_path / "runtime.log"
    stream_to_rotating_files(BytesIO(payload), path, maximum_bytes=4096)
    assert path.read_bytes() == payload


def test_rotation_fsync_can_be_disabled_but_shutdown_is_durable(tmp_path: Path) -> None:
    payload = bytes(range(256)) * 100
    path = tmp_path / "runtime.log"
    trace = tmp_path / "fsync.csv"
    result = stream_to_rotating_files(
        BytesIO(payload), path, maximum_bytes=4096, chunk_bytes=777,
        fsync_on_rotate=False, fsync_trace_path=trace,
    )
    assert result["rotations"] >= 1
    assert result["fsync_count"] == 1
    rows = trace.read_text(encoding="utf-8").splitlines()
    assert len(rows) == 2
    assert ",shutdown," in rows[1]


def test_output_is_readable_before_the_source_closes(tmp_path):
    """A supervised process that logs a few KB across a whole session left a
    0-byte file for its entire run, because the reader waited for a full
    64 KB chunk and the writer then buffered another 8 KB. The log was only
    readable after a clean shutdown -- and empty after a kill."""
    import os
    import threading

    path = tmp_path / "live.log"
    read_fd, write_fd = os.pipe()
    reader = os.fdopen(read_fd, "rb")
    done = threading.Event()
    worker = threading.Thread(
        target=lambda: (
            stream_to_rotating_files(reader, path, maximum_bytes=1 << 20),
            done.set(),
        ),
        daemon=True,
    )
    worker.start()
    with os.fdopen(write_fd, "wb", buffering=0) as source:
        source.write(b"first line, far short of a chunk\n")
        for _ in range(200):
            if path.exists() and path.stat().st_size:
                break
            time.sleep(0.01)
        assert path.read_bytes() == b"first line, far short of a chunk\n"
    worker.join(timeout=5)
    assert done.is_set()


def test_a_firehose_does_not_spin_the_reader(tmp_path):
    """read1 returns the instant one byte lands, so writing a continuous
    stream in tiny pieces once span the loop at ~96% of a core per writer --
    enough to starve the simulator whose logs it was writing. The pause is
    skipped when reads come back full, so throughput must not collapse."""
    import os
    import threading

    path = tmp_path / "firehose.log"
    read_fd, write_fd = os.pipe()
    reader = os.fdopen(read_fd, "rb")
    result = {}
    worker = threading.Thread(
        target=lambda: result.update(
            stream_to_rotating_files(reader, path, maximum_bytes=1 << 24)
        ),
        daemon=True,
    )
    worker.start()
    payload = b"x" * 4096
    started = time.process_time()
    with os.fdopen(write_fd, "wb", buffering=0) as source:
        for _ in range(256):
            source.write(payload)
    worker.join(timeout=30)
    cpu_seconds = time.process_time() - started
    assert path.stat().st_size == 256 * len(payload)
    # Generous: the spinning version burned a full core for the whole run.
    assert cpu_seconds < 2.0, f"reader burned {cpu_seconds:.2f}s of CPU"
