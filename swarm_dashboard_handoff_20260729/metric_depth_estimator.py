from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class MetricDepthEstimate:
    valid: bool
    ready: bool
    reason: str
    distance_raw_m: float | None
    distance_filtered_m: float | None
    optical_depth_m: float | None
    roi_valid_fraction: float
    source: str
    frame_index: int
    stable_samples: int


class MetricDepthEstimator:
    """Optional monocular metric depth adapter for tracking range control."""

    def __init__(
        self,
        *,
        fx: float,
        fy: float,
        cx: float | None = None,
        cy: float | None = None,
    ) -> None:
        self.enabled = os.environ.get(
            "SWARM_METRIC_DEPTH_ENABLED",
            "false",
        ).strip().lower() in {"1", "true", "yes", "on"}
        self.backend = os.environ.get(
            "SWARM_METRIC_DEPTH_BACKEND",
            "unidepth",
        ).strip().lower()
        self.model_name = os.environ.get(
            "SWARM_METRIC_DEPTH_MODEL",
            "lpiccinelli/unidepth-v2-vitl14",
        ).strip()
        self.device_name = os.environ.get(
            "SWARM_METRIC_DEPTH_DEVICE",
            "auto",
        ).strip().lower()
        try:
            self.every_n_frames = max(
                1,
                min(30, int(os.environ.get("SWARM_METRIC_DEPTH_EVERY_N", "3"))),
            )
            self.ema_alpha = max(
                0.01,
                min(1.0, float(os.environ.get("SWARM_METRIC_DEPTH_EMA_ALPHA", "0.30"))),
            )
            self.min_distance_m = max(
                0.05,
                float(os.environ.get("SWARM_METRIC_DEPTH_MIN_M", "0.5")),
            )
            self.max_distance_m = max(
                self.min_distance_m,
                float(os.environ.get("SWARM_METRIC_DEPTH_MAX_M", "80.0")),
            )
            self.min_valid_fraction = max(
                0.01,
                min(
                    1.0,
                    float(os.environ.get("SWARM_METRIC_DEPTH_MIN_VALID_FRACTION", "0.20")),
                ),
            )
            self.stable_samples_required = max(
                2,
                min(
                    20,
                    int(os.environ.get("SWARM_METRIC_DEPTH_STABLE_SAMPLES", "5")),
                ),
            )
            self.max_stable_step_m = max(
                0.05,
                min(
                    5.0,
                    float(os.environ.get("SWARM_METRIC_DEPTH_MAX_STABLE_STEP_M", "0.50")),
                ),
            )
        except ValueError:
            self.every_n_frames = 3
            self.ema_alpha = 0.30
            self.min_distance_m = 0.5
            self.max_distance_m = 80.0
            self.min_valid_fraction = 0.20
            self.stable_samples_required = 5
            self.max_stable_step_m = 0.50
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = cx
        self.cy = cy
        self._model: Any = None
        self._torch: Any = None
        self._np: Any = None
        self._device: Any = None
        self._frame_index = 0
        self._filtered_distance_m: float | None = None
        self._previous_raw_distance_m: float | None = None
        self._stable_samples = 0
        self._last_estimate = MetricDepthEstimate(
            False, False, "disabled" if not self.enabled else "not_loaded",
            None, None, None, 0.0, "metric_depth", 0, 0,
        )
        self._load_error = ""

    @property
    def available(self) -> bool:
        return bool(self.enabled and self._model is not None and not self._load_error)

    @property
    def load_error(self) -> str:
        return self._load_error

    @property
    def last_estimate(self) -> MetricDepthEstimate:
        return self._last_estimate

    def preload(self) -> bool:
        """Load model weights before tracking starts.

        Keeping this outside the first bbox update prevents a model download or
        GPU initialization from blocking the tracking stream while the user is
        interacting with the dashboard.
        """
        if not self.enabled:
            return False
        loaded = self._ensure_loaded()
        if loaded:
            self._last_estimate = MetricDepthEstimate(
                False,
                False,
                "preloaded",
                None,
                None,
                None,
                0.0,
                "metric_depth",
                0,
                0,
            )
        return loaded

    def reset(self) -> None:
        self._frame_index = 0
        self._filtered_distance_m = None
        self._previous_raw_distance_m = None
        self._stable_samples = 0
        self._last_estimate = MetricDepthEstimate(
            False, False, "disabled" if not self.enabled else "not_loaded",
            None, None, None, 0.0, "metric_depth", 0, 0,
        )

    def update(self, frame_bgr: Any, bbox: Any, *, tracking_valid: bool) -> MetricDepthEstimate:
        self._frame_index += 1
        if not self.enabled:
            return self._invalid("disabled")
        if not tracking_valid or bbox is None:
            return self._invalid("tracking_not_confirmed")
        if self._frame_index % self.every_n_frames != 0 and self._last_estimate.valid:
            return self._last_estimate
        if not self._ensure_loaded():
            reason = f"unavailable:{self._load_error}" if self._load_error else "unavailable"
            return self._invalid(reason)

        try:
            depth = self._infer_depth(frame_bgr)
            distance, optical_depth, valid_fraction = self._distance_from_bbox(
                depth,
                bbox,
                frame_bgr.shape[1],
                frame_bgr.shape[0],
            )
        except Exception as error:
            return self._invalid(f"inference_failed:{error}")

        if valid_fraction < self.min_valid_fraction:
            return self._invalid("sparse_roi", roi_valid_fraction=valid_fraction)
        if not self.min_distance_m <= distance <= self.max_distance_m:
            return self._invalid("distance_out_of_range", roi_valid_fraction=valid_fraction)

        filtered = (
            distance
            if self._filtered_distance_m is None
            else (
                (1.0 - self.ema_alpha) * self._filtered_distance_m
                + self.ema_alpha * distance
            )
        )
        stable_sample = bool(
            self._previous_raw_distance_m is None
            or abs(distance - self._previous_raw_distance_m) <= self.max_stable_step_m
        )
        self._stable_samples = self._stable_samples + 1 if stable_sample else 1
        self._previous_raw_distance_m = distance
        ready = self._stable_samples >= self.stable_samples_required
        self._filtered_distance_m = filtered
        self._last_estimate = MetricDepthEstimate(
            True,
            ready,
            "ready" if ready else "stabilizing",
            distance,
            filtered,
            optical_depth,
            valid_fraction,
            "metric_depth",
            self._frame_index,
            self._stable_samples,
        )
        return self._last_estimate

    def _ensure_loaded(self) -> bool:
        if self._model is not None:
            return True
        if self._load_error:
            return False
        if self.backend != "unidepth":
            self._load_error = f"unsupported backend {self.backend}"
            return False
        try:
            import numpy as np
            import torch
            from unidepth.models import UniDepthV2

            self._np = np
            self._torch = torch
            if self.device_name == "auto":
                self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            else:
                self._device = torch.device(self.device_name)
            self._model = UniDepthV2.from_pretrained(self.model_name).to(self._device)
            self._model.eval()
            return True
        except Exception as error:
            self._load_error = str(error)
            self._model = None
            return False

    def _infer_depth(self, frame_bgr: Any) -> Any:
        assert self._model is not None
        assert self._np is not None
        assert self._torch is not None
        rgb = frame_bgr[:, :, ::-1].copy()
        tensor = self._torch.from_numpy(rgb).permute(2, 0, 1).to(self._device)
        camera = self._torch.tensor(
            [
                [self.fx, 0.0, self.cx if self.cx is not None else frame_bgr.shape[1] / 2.0],
                [0.0, self.fy, self.cy if self.cy is not None else frame_bgr.shape[0] / 2.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=self._torch.float32,
            device=self._device,
        )
        with self._torch.no_grad():
            try:
                prediction = self._model.infer(tensor, camera)
            except TypeError:
                prediction = self._model.infer(tensor)
        depth = prediction["depth"] if isinstance(prediction, dict) else prediction
        depth = depth.detach().float().cpu().numpy()
        return self._np.squeeze(depth)

    def _distance_from_bbox(
        self,
        depth: Any,
        bbox: Any,
        frame_width: int,
        frame_height: int,
    ) -> tuple[float, float, float]:
        assert self._np is not None
        if depth.shape[:2] != (frame_height, frame_width):
            raise ValueError("depth map size does not match frame")
        x1 = float(bbox.x)
        y1 = float(bbox.y)
        x2 = float(bbox.x + bbox.w)
        y2 = float(bbox.y + bbox.h)
        u1 = max(0, min(frame_width - 1, int(x1 + 0.35 * (x2 - x1))))
        u2 = max(u1 + 1, min(frame_width, int(x1 + 0.65 * (x2 - x1))))
        v1 = max(0, min(frame_height - 1, int(y1 + 0.35 * (y2 - y1))))
        v2 = max(v1 + 1, min(frame_height, int(y1 + 0.65 * (y2 - y1))))
        roi = depth[v1:v2, u1:u2]
        valid = self._np.isfinite(roi) & (roi > 0.0)
        valid_fraction = float(valid.mean()) if roi.size else 0.0
        if not valid.any():
            raise ValueError("no valid depth in bbox roi")
        z = float(self._np.median(roi[valid]))
        u = float(bbox.cx)
        v = float(bbox.cy)
        cx = frame_width / 2.0 if self.cx is None else float(self.cx)
        cy = frame_height / 2.0 if self.cy is None else float(self.cy)
        x = (u - cx) * z / self.fx
        y = (v - cy) * z / self.fy
        distance = math.sqrt(x * x + y * y + z * z)
        return distance, z, valid_fraction

    def _invalid(
        self,
        reason: str,
        *,
        roi_valid_fraction: float = 0.0,
    ) -> MetricDepthEstimate:
        self._last_estimate = MetricDepthEstimate(
            False,
            False,
            reason,
            None,
            self._filtered_distance_m,
            None,
            roi_valid_fraction,
            "metric_depth",
            self._frame_index,
            0,
        )
        self._stable_samples = 0
        return self._last_estimate
