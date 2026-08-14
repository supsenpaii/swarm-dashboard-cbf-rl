from __future__ import annotations

import os
from typing import Any

import cv2
import numpy as np

from lfc_gimbal_gazebo.models.types import BBox, TrackResult, TrackState
from lfc_gimbal_gazebo.utils.kalman import KalmanCV
from lfc_gimbal_gazebo.utils.memory_bank import TemplateMemory


class HybridMemoryTrackerService:
    """Fast KCF tracking with Kalman prediction and appearance re-identification."""

    def __init__(self, algorithm: str = "KCF") -> None:
        self.algorithm = algorithm.upper()
        self.redetect_after = self._env_int("SWARM_REID_AFTER", 5, 1, 60)
        self.redetect_interval = self._env_int(
            "SWARM_REID_INTERVAL", 2, 1, 30
        )
        self.reacquire_confirm = self._env_int(
            "SWARM_REID_CONFIRM", 2, 1, 10
        )
        self.track_reid_threshold = self._env_float(
            "SWARM_REID_TRACK_THRESHOLD", 0.38, 0.0, 1.0
        )
        self.redetect_reid_threshold = self._env_float(
            "SWARM_REID_THRESHOLD", 0.48, 0.0, 1.0
        )
        self.redetect_score_threshold = self._env_float(
            "SWARM_REID_SCORE_THRESHOLD", 0.58, 0.0, 1.0
        )
        self.memory_update_threshold = self._env_float(
            "SWARM_MEMORY_UPDATE_THRESHOLD", 0.72, 0.0, 1.0
        )
        self.reset()

    @staticmethod
    def _env_int(name: str, default: int, minimum: int, maximum: int) -> int:
        try:
            value = int(os.environ.get(name, str(default)))
        except ValueError:
            value = default
        return max(minimum, min(maximum, value))

    @staticmethod
    def _env_float(
        name: str,
        default: float,
        minimum: float,
        maximum: float,
    ) -> float:
        try:
            value = float(os.environ.get(name, str(default)))
        except ValueError:
            value = default
        return max(minimum, min(maximum, value))

    def reset(self) -> None:
        self._tracker: Any = None
        self._bbox: BBox | None = None
        self._template: Any = None
        self._template_size: tuple[int, int] | None = None
        self.kalman = KalmanCV()
        self.memory = TemplateMemory()
        self.frame_idx = 0
        self.lost_count = 0
        self.reacquire_count = 0
        self.pending_bbox: BBox | None = None
        self.last_reid_score = 0.0
        self.last_match_score = 0.0
        self.state = TrackState.IDLE

    @property
    def active(self) -> bool:
        return self._tracker is not None and self._bbox is not None

    @property
    def diagnostics(self) -> dict[str, Any]:
        velocity = self.kalman.vel
        return {
            "kalman": True,
            "memory": True,
            "reid": True,
            "memory_entries": len(self.memory.temporal_features),
            "reid_score": round(self.last_reid_score, 3),
            "match_score": round(self.last_match_score, 3),
            "lost_count": self.lost_count,
            "kalman_velocity_px": [round(velocity[0], 2), round(velocity[1], 2)],
        }

    def init(self, frame: Any, bbox: BBox) -> None:
        clipped = self._clip_bbox(bbox, frame.shape)
        crop = self._crop(frame, clipped)
        if crop is None:
            raise ValueError("Bounding box does not contain a valid target image")

        self.reset()
        self._bbox = clipped
        self._template = crop.copy()
        self._template_size = (crop.shape[1], crop.shape[0])
        self.memory.update(self._appearance_feature(crop), self._memory_crop(crop))
        self.kalman.start(clipped.cx, clipped.cy)
        self._init_kcf(frame, clipped)
        self.state = TrackState.TRACKING

    def update(self, frame: Any) -> TrackResult:
        if not self.active:
            raise RuntimeError("Tracker is not initialized")

        self.frame_idx += 1
        predicted = self.kalman.predict()
        success, raw_bbox = self._tracker.update(frame)

        if success:
            candidate = self._clip_bbox(BBox(*map(float, raw_bbox)), frame.shape)
            crop = self._crop(frame, candidate)
            reid_score = self._appearance_similarity(crop)
            self.last_reid_score = reid_score
            motion_score = self._motion_score(candidate, predicted, frame.shape)
            self.last_match_score = 0.78 * reid_score + 0.22 * motion_score

            allow_weak_start = self.frame_idx <= 2 and reid_score >= 0.25
            if self.last_match_score >= self.track_reid_threshold or allow_weak_start:
                self._accept_tracking(frame, candidate, crop, reid_score)
                return self._result(self.last_match_score)

            self.memory.add_blacklist(
                candidate.x, candidate.y, candidate.w, candidate.h
            )

        return self._handle_missing(frame, predicted)

    def _accept_tracking(
        self,
        frame: Any,
        bbox: BBox,
        crop: Any,
        reid_score: float,
    ) -> None:
        previous = self._bbox
        self._bbox = bbox
        self.kalman.correct(bbox.cx, bbox.cy)
        if previous is not None:
            self.memory.update_velocity(bbox.cx - previous.cx, bbox.cy - previous.cy)
        if (
            crop is not None
            and reid_score >= self.memory_update_threshold
            and self.frame_idx % 5 == 0
        ):
            self.memory.update(
                self._appearance_feature(crop),
                self._memory_crop(crop),
            )
        self.lost_count = 0
        self.reacquire_count = 0
        self.pending_bbox = None
        self.memory.exit_pos = None
        self.state = TrackState.TRACKING

    def _handle_missing(self, frame: Any, predicted: Any) -> TrackResult:
        self.lost_count += 1
        self.kalman.damp(0.92)
        if predicted is not None and self._bbox is not None:
            self._bbox = self._clip_bbox(
                BBox.from_center(
                    predicted[0],
                    predicted[1],
                    self._bbox.w,
                    self._bbox.h,
                ),
                frame.shape,
            )

        if self.memory.exit_pos is None and self._bbox is not None:
            velocity = self.memory.get_avg_velocity()
            self.memory.set_exit(
                self._bbox.cx,
                self._bbox.cy,
                velocity[0],
                velocity[1],
            )

        if self.lost_count < self.redetect_after:
            self.state = TrackState.OCCLUDED
            return self._result(self.last_match_score)

        self.state = TrackState.LOST
        if self.lost_count % self.redetect_interval != 0:
            return self._result(self.last_match_score, redetecting=True)

        candidate = self._redetect(frame, predicted)
        if candidate is None:
            self.reacquire_count = 0
            self.pending_bbox = None
            return self._result(self.last_match_score, redetecting=True)

        if self.pending_bbox is not None and self._same_target(
            self.pending_bbox, candidate
        ):
            self.reacquire_count += 1
        else:
            self.pending_bbox = candidate
            self.reacquire_count = 1

        self._bbox = candidate
        if self.reacquire_count < self.reacquire_confirm:
            return self._result(self.last_match_score, redetecting=True)

        self._init_kcf(frame, candidate)
        self.kalman.start(candidate.cx, candidate.cy)
        self.lost_count = 0
        self.reacquire_count = 0
        self.pending_bbox = None
        self.memory.exit_pos = None
        self.state = TrackState.TRACKING
        return self._result(self.last_match_score)

    def _redetect(self, frame: Any, predicted: Any) -> BBox | None:
        if self._template is None or self._template_size is None:
            return None

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        base_width, base_height = self._template_size
        candidates: list[tuple[float, BBox]] = []

        for scale in (0.75, 0.9, 1.0, 1.12, 1.28):
            width = max(8, int(round(base_width * scale)))
            height = max(8, int(round(base_height * scale)))
            if width >= frame.shape[1] or height >= frame.shape[0]:
                continue
            template = cv2.resize(
                self._template,
                (width, height),
                interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR,
            )
            template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            response = cv2.matchTemplate(
                frame_gray, template_gray, cv2.TM_CCOEFF_NORMED
            )
            for _ in range(3):
                _, template_score, _, location = cv2.minMaxLoc(response)
                if not np.isfinite(template_score):
                    break
                bbox = BBox(float(location[0]), float(location[1]), width, height)
                crop = self._crop(frame, bbox)
                reid_score = self._appearance_similarity(crop)
                motion_score = self._motion_score(bbox, predicted, frame.shape)
                exit_score = 1.0 if self.memory.check_exit_return(
                    bbox.cx,
                    bbox.cy,
                    radius=max(width, height) * 5.0,
                ) else 0.0
                combined = (
                    0.42 * float(template_score)
                    + 0.43 * reid_score
                    + 0.10 * motion_score
                    + 0.05 * exit_score
                )
                if (
                    reid_score >= self.redetect_reid_threshold
                    and not self.memory.is_blacklisted(
                        bbox.x, bbox.y, bbox.w, bbox.h
                    )
                ):
                    candidates.append((combined, bbox))
                x0 = max(0, location[0] - width // 2)
                y0 = max(0, location[1] - height // 2)
                x1 = min(response.shape[1], location[0] + width // 2 + 1)
                y1 = min(response.shape[0], location[1] + height // 2 + 1)
                response[y0:y1, x0:x1] = -1.0

        if not candidates:
            return None
        score, bbox = max(candidates, key=lambda item: item[0])
        self.last_match_score = float(score)
        crop = self._crop(frame, bbox)
        self.last_reid_score = self._appearance_similarity(crop)
        if score < self.redetect_score_threshold:
            return None
        return self._clip_bbox(bbox, frame.shape)

    def _appearance_similarity(self, crop: Any) -> float:
        if crop is None:
            return 0.0
        return float(
            np.clip(
                self.memory.match(
                    self._appearance_feature(crop),
                    self._memory_crop(crop),
                ),
                0.0,
                1.0,
            )
        )

    @staticmethod
    def _appearance_feature(crop: Any) -> Any:
        resized = cv2.resize(crop, (24, 24), interpolation=cv2.INTER_AREA)
        lab = cv2.cvtColor(resized, cv2.COLOR_BGR2LAB).astype(np.float32)
        feature = lab.reshape(-1)
        feature -= feature.mean()
        feature /= feature.std() + 1e-6
        return feature

    @staticmethod
    def _memory_crop(crop: Any) -> Any:
        return cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)

    @staticmethod
    def _motion_score(bbox: BBox, predicted: Any, shape: Any) -> float:
        if predicted is None:
            return 1.0
        diagonal = float(np.hypot(shape[1], shape[0]))
        distance = float(np.hypot(bbox.cx - predicted[0], bbox.cy - predicted[1]))
        return float(np.exp(-4.0 * distance / max(diagonal, 1.0)))

    @staticmethod
    def _same_target(first: BBox, second: BBox) -> bool:
        distance = float(np.hypot(first.cx - second.cx, first.cy - second.cy))
        radius = max(18.0, 0.75 * max(first.w, first.h, second.w, second.h))
        return distance <= radius

    def _init_kcf(self, frame: Any, bbox: BBox) -> None:
        factories = {
            "KCF": getattr(cv2, "TrackerKCF_create", None),
            "CSRT": getattr(cv2, "TrackerCSRT_create", None),
            "MOSSE": getattr(getattr(cv2, "legacy", None), "TrackerMOSSE_create", None),
        }
        factory = factories.get(self.algorithm)
        if factory is None:
            raise RuntimeError(f"Unsupported OpenCV tracker: {self.algorithm}")
        tracker = factory()
        values = tuple(
            int(round(value)) for value in (bbox.x, bbox.y, bbox.w, bbox.h)
        )
        initialized = tracker.init(frame, values)
        if initialized is False:
            raise RuntimeError(f"OpenCV {self.algorithm} could not initialize")
        self._tracker = tracker

    @staticmethod
    def _crop(frame: Any, bbox: BBox) -> Any:
        height, width = frame.shape[:2]
        x0 = max(0, min(width - 1, int(np.floor(bbox.x))))
        y0 = max(0, min(height - 1, int(np.floor(bbox.y))))
        x1 = max(x0 + 1, min(width, int(np.ceil(bbox.x + bbox.w))))
        y1 = max(y0 + 1, min(height, int(np.ceil(bbox.y + bbox.h))))
        crop = frame[y0:y1, x0:x1]
        return crop if crop.size else None

    @staticmethod
    def _clip_bbox(bbox: BBox, shape: Any) -> BBox:
        height, width = shape[:2]
        box_width = float(np.clip(bbox.w, 6.0, width))
        box_height = float(np.clip(bbox.h, 6.0, height))
        x = float(np.clip(bbox.x, 0.0, max(0.0, width - box_width)))
        y = float(np.clip(bbox.y, 0.0, max(0.0, height - box_height)))
        return BBox(x, y, box_width, box_height)

    def _result(self, score: float, redetecting: bool = False) -> TrackResult:
        return TrackResult(
            state=self.state,
            bbox=self._bbox,
            score=float(np.clip(score, 0.0, 1.0)),
            frame_idx=self.frame_idx,
            redetecting=redetecting,
        )
