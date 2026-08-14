from __future__ import annotations

import math
import os
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import numpy as np


@dataclass(frozen=True)
class DepthMap:
    """A relative inverse-depth map aligned with the input image."""

    inverse_depth: np.ndarray
    source: str


class DepthModelAdapter(ABC):
    """Small interface kept outside TrackingManager for testability."""

    @abstractmethod
    def load(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def infer(self, frame_bgr: np.ndarray) -> DepthMap:
        raise NotImplementedError


class CallableDepthAdapter(DepthModelAdapter):
    """Adapter used by replay/tests and by externally managed model runtimes."""

    def __init__(
        self,
        infer_fn: Callable[[np.ndarray], np.ndarray],
        *,
        source: str = "callable",
    ) -> None:
        self._infer_fn = infer_fn
        self._source = source

    def load(self) -> None:
        return

    def infer(self, frame_bgr: np.ndarray) -> DepthMap:
        inverse_depth = np.asarray(self._infer_fn(frame_bgr), dtype=np.float32)
        if inverse_depth.shape != frame_bgr.shape[:2]:
            raise ValueError("depth output must be aligned with the input frame")
        if not np.any(np.isfinite(inverse_depth)):
            raise ValueError("depth output contains no finite values")
        return DepthMap(inverse_depth=inverse_depth, source=self._source)


class MidasSmallAdapter(DepthModelAdapter):
    """Local-only MiDaS-small adapter.

    Loading is deliberately local-only. A tracking session must never pause to
    clone a repository or download model weights. Prepare the MiDaS repository
    and checkpoint beforehand and point the two environment variables below at
    them.
    """

    def __init__(
        self,
        *,
        repository_path: str | None = None,
        checkpoint_path: str | None = None,
        model_name: str | None = None,
        device: str | None = None,
        input_size: int | None = None,
    ) -> None:
        local_model_root = Path(__file__).resolve().parent / "models"
        configured_repository = (
            repository_path
            or os.environ.get("SWARM_MIDAS_REPOSITORY", "").strip()
        )
        configured_checkpoint = (
            checkpoint_path
            or os.environ.get("SWARM_MIDAS_CHECKPOINT", "").strip()
        )
        self.repository_path = Path(
            configured_repository
            or local_model_root / "MiDaS"
        ).expanduser()
        self.checkpoint_path = Path(
            configured_checkpoint
            or local_model_root / "midas_v21_small_256.pt"
        ).expanduser()
        self.model_name = (
            model_name
            or os.environ.get("SWARM_MIDAS_MODEL", "MiDaS_small")
        ).strip()
        self.device_name = (
            device
            or os.environ.get("SWARM_MIDAS_DEVICE", "auto")
        ).strip().lower()
        try:
            configured_size = int(
                input_size
                or os.environ.get("SWARM_MIDAS_INPUT_SIZE", "256")
            )
        except ValueError:
            configured_size = 256
        self.input_size = max(128, min(512, configured_size))
        self._torch: Any = None
        self._model: Any = None
        self._device: Any = None
        # Opt-in only: see docs/CORE_RANGE_LIVE_STACK_CONTENTION_REPORT.md.
        # Never enabled unless this env var is explicitly set, so production
        # inference never pays for the extra torch.cuda.synchronize() calls
        # below (those calls would otherwise mask real CUDA-asynchronous
        # timing by forcing a sync point that is not there by default).
        self._profile_stages = (
            os.environ.get("SWARM_DEPTH_PROFILE_STAGES", "0").strip() == "1"
        )
        self.last_profiling_stages: dict[str, float] | None = None

    def load(self) -> None:
        if self._model is not None:
            return
        if not self.repository_path.is_dir():
            raise FileNotFoundError(
                "SWARM_MIDAS_REPOSITORY must point to a local MiDaS checkout"
            )
        if not self.checkpoint_path.is_file():
            raise FileNotFoundError(
                "SWARM_MIDAS_CHECKPOINT must point to local model weights"
            )

        import torch

        # Fixed input resolution, repeated every inference call: letting
        # cuDNN autotune and cache the fastest conv algorithm for that one
        # shape is output-preserving (same math, different implementation)
        # and measured ~2.3x faster in isolation. See
        # docs/CORE_RANGE_DEPTH_PIPELINE_LATENCY_REPORT.md.
        torch.backends.cudnn.benchmark = True

        self._torch = torch
        if self.model_name.lower() == "midas_small":
            hub_root = Path(torch.hub.get_dir())
            efficientnet_repository = (
                hub_root / "rwightman_gen-efficientnet-pytorch_master"
            )
            efficientnet_checkpoint = (
                hub_root
                / "checkpoints"
                / "tf_efficientnet_lite3-b733e338.pth"
            )
            if (
                not efficientnet_repository.is_dir()
                or not efficientnet_checkpoint.is_file()
            ):
                raise FileNotFoundError(
                    "MiDaS-small EfficientNet dependency is not preloaded "
                    "in the torch hub cache"
                )
        if self.device_name == "auto":
            self._device = torch.device(
                "cuda" if torch.cuda.is_available() else "cpu"
            )
        else:
            self._device = torch.device(self.device_name)
        model = torch.hub.load(
            str(self.repository_path),
            self.model_name,
            source="local",
            pretrained=False,
            trust_repo=True,
        )
        checkpoint = torch.load(
            str(self.checkpoint_path),
            map_location="cpu",
            weights_only=False,
        )
        if isinstance(checkpoint, dict):
            checkpoint = checkpoint.get(
                "state_dict",
                checkpoint.get("model", checkpoint),
            )
        if not isinstance(checkpoint, dict):
            raise ValueError("MiDaS checkpoint does not contain a state dict")
        cleaned = {
            str(key).removeprefix("module."): value
            for key, value in checkpoint.items()
        }
        model.load_state_dict(cleaned, strict=False)
        model.to(self._device)
        model.eval()
        self._model = model

    def infer(self, frame_bgr: np.ndarray) -> DepthMap:
        self.load()
        assert self._torch is not None
        assert self._model is not None
        height, width = frame_bgr.shape[:2]
        torch = self._torch
        profiling = self._profile_stages
        is_cuda = self._device is not None and self._device.type == "cuda"
        stages: dict[str, float] | None = {} if profiling else None
        # Cleared up front: if this call raises partway through, a stale
        # dict from a previous successful call must not be attributed to
        # this (failed) call.
        self.last_profiling_stages = None

        def _sync() -> None:
            # Only forced in profiling mode: torch.cuda ops are
            # asynchronous, so without a synchronize() here a stage's
            # wall-clock duration would just measure how long it took to
            # enqueue the kernel, not how long the GPU took to run it. Never
            # called when profiling is off (the production default).
            if profiling and is_cuda:
                torch.cuda.synchronize()

        def _stage(name: str, started_at: float) -> None:
            if stages is not None:
                stages[name] = time.perf_counter() - started_at

        start = time.perf_counter()
        rgb = np.ascontiguousarray(frame_bgr[:, :, ::-1])
        _stage("frame_copy_decode_s", start)

        start = time.perf_counter()
        tensor = (
            torch.from_numpy(rgb)
            .permute(2, 0, 1)
            .unsqueeze(0)
            .to(self._device, dtype=torch.float32)
            / 255.0
        )
        _sync()
        _stage("host_to_device_transfer_s", start)

        start = time.perf_counter()
        mean = torch.tensor(
            (0.485, 0.456, 0.406),
            device=self._device,
        ).view(1, 3, 1, 1)
        std = torch.tensor(
            (0.229, 0.224, 0.225),
            device=self._device,
        ).view(1, 3, 1, 1)
        tensor = (tensor - mean) / std
        tensor = torch.nn.functional.interpolate(
            tensor,
            size=(self.input_size, self.input_size),
            mode="bicubic",
            align_corners=False,
        )
        _sync()
        _stage("gpu_preprocess_resize_normalize_s", start)

        with torch.inference_mode():
            start = time.perf_counter()
            prediction = self._model(tensor)
            if isinstance(prediction, dict):
                prediction = prediction.get(
                    "predicted_depth",
                    prediction.get("out"),
                )
            if isinstance(prediction, (tuple, list)):
                prediction = prediction[0]
            if prediction is None:
                raise ValueError("MiDaS returned no depth output")
            while prediction.ndim > 3:
                prediction = prediction[:, 0]
            if prediction.ndim == 2:
                prediction = prediction.unsqueeze(0)
            _sync()
            _stage("model_forward_s", start)

            start = time.perf_counter()
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=(height, width),
                mode="bicubic",
                align_corners=False,
            )[0, 0]
            _sync()
            _stage("output_resize_interpolation_s", start)

        start = time.perf_counter()
        inverse_depth = prediction.float().cpu().numpy().astype(
            np.float32,
            copy=False,
        )
        _stage("device_to_host_transfer_s", start)

        start = time.perf_counter()
        finite = np.isfinite(inverse_depth)
        if not np.any(finite):
            raise ValueError("MiDaS returned no finite depth")
        minimum = float(np.nanpercentile(inverse_depth[finite], 0.5))
        inverse_depth = np.where(
            finite,
            np.maximum(inverse_depth - min(0.0, minimum), 1e-6),
            np.nan,
        ).astype(np.float32, copy=False)
        _stage("numpy_postprocess_s", start)

        self.last_profiling_stages = stages
        return DepthMap(inverse_depth=inverse_depth, source="midas_small")


def create_depth_adapter_from_environment() -> DepthModelAdapter:
    backend = os.environ.get(
        "SWARM_METRIC_TARGET_DEPTH_BACKEND",
        "midas_small",
    ).strip().lower()
    if backend in {"midas", "midas_small"}:
        return MidasSmallAdapter()
    raise ValueError(f"unsupported depth backend: {backend}")
