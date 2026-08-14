"""Opt-in fine-grained stage instrumentation in MidasSmallAdapter.infer().

See docs/CORE_RANGE_LIVE_STACK_CONTENTION_REPORT.md. These tests avoid
loading real MiDaS weights: they bypass `load()` by pre-populating the
adapter's `_torch`/`_model`/`_device` attributes with a tiny stand-in model,
since `load()` returns immediately once `_model is not None`.
"""

from __future__ import annotations

import numpy as np
import pytest
import torch

from depth_model_adapter import MidasSmallAdapter

EXPECTED_STAGE_KEYS = {
    "frame_copy_decode_s",
    "host_to_device_transfer_s",
    "gpu_preprocess_resize_normalize_s",
    "model_forward_s",
    "output_resize_interpolation_s",
    "device_to_host_transfer_s",
    "numpy_postprocess_s",
}


class _StubMidasModel(torch.nn.Module):
    def forward(self, tensor: torch.Tensor) -> torch.Tensor:
        return tensor.mean(dim=1)


class _FailingMidasModel(torch.nn.Module):
    def forward(self, tensor: torch.Tensor) -> None:
        return None


def _make_adapter(*, device_name: str, profile_stages: bool, model: torch.nn.Module) -> MidasSmallAdapter:
    adapter = MidasSmallAdapter(input_size=32)
    adapter._profile_stages = profile_stages
    adapter._torch = torch
    adapter._device = torch.device(device_name)
    adapter._model = model.to(adapter._device).eval()
    return adapter


def _frame() -> np.ndarray:
    rng = np.random.default_rng(52)
    return rng.integers(0, 255, (18, 24, 3), dtype=np.uint8)


def test_profiling_disabled_by_default_leaves_last_profiling_stages_none() -> None:
    adapter = _make_adapter(device_name="cpu", profile_stages=False, model=_StubMidasModel())
    assert adapter.last_profiling_stages is None
    depth_map = adapter.infer(_frame())
    assert depth_map.source == "midas_small"
    assert adapter.last_profiling_stages is None


def test_profiling_enabled_populates_all_expected_stage_keys() -> None:
    adapter = _make_adapter(device_name="cpu", profile_stages=True, model=_StubMidasModel())
    adapter.infer(_frame())
    stages = adapter.last_profiling_stages
    assert stages is not None
    assert set(stages.keys()) == EXPECTED_STAGE_KEYS
    for name, duration_s in stages.items():
        assert duration_s >= 0.0, name


def test_profiling_stage_dict_cleared_not_stale_after_a_failing_call() -> None:
    adapter = _make_adapter(device_name="cpu", profile_stages=True, model=_StubMidasModel())
    adapter.infer(_frame())
    assert adapter.last_profiling_stages is not None

    adapter._model = _FailingMidasModel().to(adapter._device).eval()
    with pytest.raises(ValueError):
        adapter.infer(_frame())
    assert adapter.last_profiling_stages is None


def test_env_var_enables_profile_stages_flag(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("SWARM_DEPTH_PROFILE_STAGES", "1")
    assert MidasSmallAdapter()._profile_stages is True

    monkeypatch.delenv("SWARM_DEPTH_PROFILE_STAGES", raising=False)
    assert MidasSmallAdapter()._profile_stages is False


@pytest.mark.skipif(not torch.cuda.is_available(), reason="requires a CUDA device")
def test_cuda_synchronize_only_called_when_profiling_enabled(monkeypatch: pytest.MonkeyPatch) -> None:
    calls = {"count": 0}
    real_synchronize = torch.cuda.synchronize

    def counting_synchronize(*args: object, **kwargs: object) -> None:
        calls["count"] += 1
        real_synchronize(*args, **kwargs)

    monkeypatch.setattr(torch.cuda, "synchronize", counting_synchronize)

    adapter = _make_adapter(device_name="cuda", profile_stages=False, model=_StubMidasModel())
    adapter.infer(_frame())
    assert calls["count"] == 0

    adapter = _make_adapter(device_name="cuda", profile_stages=True, model=_StubMidasModel())
    adapter.infer(_frame())
    assert calls["count"] > 0
    assert set(adapter.last_profiling_stages.keys()) == EXPECTED_STAGE_KEYS
