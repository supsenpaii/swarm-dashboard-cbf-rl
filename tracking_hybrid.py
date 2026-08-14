from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from lfc_gimbal_gazebo.models.types import BBox, TrackResult, TrackState
from lfc_gimbal_gazebo.utils.kalman import KalmanCV
from lfc_gimbal_gazebo.utils.memory_bank import TemplateMemory


@dataclass(frozen=True)
class ScaleRefinementResult:
    bbox: BBox
    candidate_bbox: BBox
    accepted: bool
    reason: str
    scale_ratio: float
    score: float
    psr: float
    correlation_score: float
    appearance_score: float
    texture_score: float
    ambiguous: bool


class MultiScaleBBoxRefiner:
    """Small local scale search around the KCF/Kalman target centre.

    KCF remains responsible for translation.  Each candidate only changes the
    previous accepted width/height, so this never becomes an expensive
    full-frame detector.  A fixed, high-confidence reference prevents a large
    background crop from becoming the new target template.
    """

    def __init__(
        self,
        *,
        enabled: bool = True,
        candidates: tuple[float, ...] = (
            0.92, 1.0, 1.08,
        ),
        minimum_score: float = 0.48,
        minimum_psr: float = 0.35,
        minimum_gain: float = 0.012,
        max_step_ratio: float = 0.18,
        ambiguity_ratio: float = 0.985,
    ) -> None:
        clean = sorted({
            float(value) for value in candidates
            if np.isfinite(value) and 0.5 <= float(value) <= 1.8
        })
        if 1.0 not in clean:
            clean.append(1.0)
            clean.sort()
        self.enabled = bool(enabled)
        self.candidates = tuple(clean)
        self.minimum_score = float(np.clip(minimum_score, 0.0, 1.0))
        self.minimum_psr = max(0.0, float(minimum_psr))
        self.minimum_gain = max(0.0, float(minimum_gain))
        self.max_step_ratio = max(0.01, float(max_step_ratio))
        self.ambiguity_ratio = float(np.clip(ambiguity_ratio, 0.8, 1.0))
        self.reference: Any = None
        self.reference_texture = 0.0

    def reset(self, reference_crop: Any | None = None) -> None:
        self.reference = None
        self.reference_texture = 0.0
        if reference_crop is not None and getattr(reference_crop, "size", 0):
            self.reference = reference_crop.copy()
            self.reference_texture = self._texture(reference_crop)

    def refine(
        self,
        frame: Any,
        center_bbox: BBox,
        previous_bbox: BBox,
        *,
        appearance_similarity: Any = None,
    ) -> ScaleRefinementResult:
        held = BBox.from_center(
            center_bbox.cx,
            center_bbox.cy,
            previous_bbox.w,
            previous_bbox.h,
        )
        if not self.enabled:
            return self._invalid(held, "disabled")
        if self.reference is None:
            return self._invalid(held, "missing_reference")

        height, width = frame.shape[:2]
        ranked: list[
            tuple[float, int, BBox, float, float, float, float, float]
        ] = []
        for index, scale in enumerate(self.candidates):
            if abs(scale - 1.0) > self.max_step_ratio + 1e-9:
                continue
            local = self._local_candidate(
                frame,
                center_bbox.cx,
                center_bbox.cy,
                previous_bbox.w * scale,
                previous_bbox.h * scale,
            )
            if local is None:
                continue
            candidate, correlation, spatial_psr = local
            if (
                candidate.x < 0.0
                or candidate.y < 0.0
                or candidate.x + candidate.w > width
                or candidate.y + candidate.h > height
            ):
                continue
            crop = HybridMemoryTrackerService._crop(frame, candidate)
            if crop is None:
                continue
            appearance = (
                float(np.clip(appearance_similarity(crop), 0.0, 1.0))
                if callable(appearance_similarity)
                else correlation
            )
            texture = self._texture_similarity(crop)
            continuity = float(np.exp(-0.40 * abs(np.log(scale))))
            score = (
                0.45 * correlation
                + 0.25 * appearance
                + 0.10 * texture
                + 0.10 * continuity
                + 0.10 * min(1.0, spatial_psr / 8.0)
            )
            ranked.append(
                (
                    float(score),
                    index,
                    candidate,
                    correlation,
                    appearance,
                    texture,
                    scale,
                    spatial_psr,
                )
            )

        if not ranked:
            return self._invalid(held, "no_in_frame_candidate")
        ranked.sort(key=lambda item: item[0], reverse=True)
        best = ranked[0]
        psr = best[7]
        unity = min(ranked, key=lambda item: abs(item[6] - 1.0))

        separated_second = next(
            (
                item for item in ranked[1:]
                if abs(item[1] - best[1]) > 1
            ),
            None,
        )
        ambiguous = bool(
            separated_second is not None
            and best[0] > 1e-6
            and separated_second[0] / best[0] >= self.ambiguity_ratio
            and abs(best[6] - 1.0) >= 0.07
        )
        reason = ""
        if best[0] < self.minimum_score:
            reason = "scale_score_below_threshold"
        elif psr < self.minimum_psr and abs(best[6] - 1.0) >= 0.07:
            reason = "scale_psr_below_threshold"
        elif ambiguous:
            reason = "ambiguous_scale_peaks"
        elif (
            abs(best[6] - 1.0) >= 0.025
            and best[0] < unity[0] + self.minimum_gain
        ):
            reason = "insufficient_scale_gain"

        if reason:
            return ScaleRefinementResult(
                bbox=held,
                candidate_bbox=best[2],
                accepted=False,
                reason=reason,
                scale_ratio=1.0,
                score=best[0],
                psr=psr,
                correlation_score=best[3],
                appearance_score=best[4],
                texture_score=best[5],
                ambiguous=ambiguous,
            )
        return ScaleRefinementResult(
            bbox=best[2],
            candidate_bbox=best[2],
            accepted=True,
            reason="accepted",
            scale_ratio=best[6],
            score=best[0],
            psr=psr,
            correlation_score=best[3],
            appearance_score=best[4],
            texture_score=best[5],
            ambiguous=False,
        )

    def _local_candidate(
        self,
        frame: Any,
        center_x: float,
        center_y: float,
        candidate_width: float,
        candidate_height: float,
    ) -> tuple[BBox, float, float] | None:
        """Align a scale candidate within a small KCF-centred search window."""
        frame_height, frame_width = frame.shape[:2]
        width = max(6, int(round(candidate_width)))
        height = max(6, int(round(candidate_height)))
        margin_x = max(4, int(round(width * 0.18)))
        margin_y = max(4, int(round(height * 0.18)))
        x0 = max(0, int(round(center_x - width / 2.0)) - margin_x)
        y0 = max(0, int(round(center_y - height / 2.0)) - margin_y)
        x1 = min(frame_width, int(round(center_x + width / 2.0)) + margin_x)
        y1 = min(frame_height, int(round(center_y + height / 2.0)) + margin_y)
        search = frame[y0:y1, x0:x1]
        if search.shape[1] < width or search.shape[0] < height:
            return None
        template = cv2.resize(
            self.reference,
            (width, height),
            interpolation=(
                cv2.INTER_AREA
                if width < self.reference.shape[1]
                else cv2.INTER_LINEAR
            ),
        )
        search_gray = cv2.cvtColor(search, cv2.COLOR_BGR2GRAY)
        template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        response = cv2.matchTemplate(
            search_gray,
            template_gray,
            cv2.TM_CCOEFF_NORMED,
        )
        _, peak, _, location = cv2.minMaxLoc(response)
        if not np.isfinite(peak):
            return None
        bbox = BBox(
            float(x0 + location[0]),
            float(y0 + location[1]),
            float(width),
            float(height),
        )
        correlation = float(np.clip(0.5 + 0.5 * peak, 0.0, 1.0))
        psr = HybridMemoryTrackerService._psr(response)
        return bbox, correlation, psr

    def update_reference(self, crop: Any, confidence: float) -> None:
        if (
            crop is None
            or not getattr(crop, "size", 0)
            or confidence < 0.78
            or self.reference is None
        ):
            return
        resized = cv2.resize(
            crop,
            (self.reference.shape[1], self.reference.shape[0]),
            interpolation=cv2.INTER_AREA,
        )
        self.reference = cv2.addWeighted(
            self.reference,
            0.92,
            resized,
            0.08,
            0.0,
        )
        self.reference_texture = self._texture(self.reference)

    def _invalid(self, bbox: BBox, reason: str) -> ScaleRefinementResult:
        return ScaleRefinementResult(
            bbox=bbox,
            candidate_bbox=bbox,
            accepted=False,
            reason=reason,
            scale_ratio=1.0,
            score=0.0,
            psr=0.0,
            correlation_score=0.0,
            appearance_score=0.0,
            texture_score=0.0,
            ambiguous=False,
        )

    def _correlation(self, crop: Any) -> float:
        resized = cv2.resize(
            crop,
            (self.reference.shape[1], self.reference.shape[0]),
            interpolation=(
                cv2.INTER_AREA
                if crop.shape[0] >= self.reference.shape[0]
                else cv2.INTER_LINEAR
            ),
        )
        reference_gray = cv2.cvtColor(self.reference, cv2.COLOR_BGR2GRAY)
        crop_gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        value = float(
            cv2.matchTemplate(
                crop_gray,
                reference_gray,
                cv2.TM_CCOEFF_NORMED,
            )[0, 0]
        )
        if not np.isfinite(value):
            return 0.0
        return float(np.clip(0.5 + 0.5 * value, 0.0, 1.0))

    @staticmethod
    def _texture(crop: Any) -> float:
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
        return float(np.mean(cv2.magnitude(gx, gy)) / 255.0)

    def _texture_similarity(self, crop: Any) -> float:
        texture = self._texture(crop)
        scale = max(0.02, self.reference_texture)
        return float(np.exp(-abs(texture - self.reference_texture) / scale))


class HybridMemoryTrackerService:
    """Fast KCF tracking with the robust gates used by the LightFC pipeline.

    The LightFC package is the source of the Kalman, temporal appearance
    memory, PSR, distractor and motion-guided re-detection ideas used here.
    Keeping KCF as the per-frame correlation engine avoids making the dashboard
    depend on an unavailable ONNX model while preserving the package's safety
    gates around bbox publication.
    """

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
            "SWARM_REID_TRACK_THRESHOLD", 0.62, 0.0, 1.0
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
        self.redetect_full_after = self._env_int(
            "SWARM_REID_FULL_FRAME_AFTER", 16, 1, 300
        )
        self.redetect_psr_threshold = self._env_float(
            "SWARM_REID_PSR_THRESHOLD", 3.8, 0.0, 100.0
        )
        self.redetect_ambiguity_ratio = self._env_float(
            "SWARM_REID_AMBIGUITY_RATIO", 0.82, 0.0, 1.0
        )
        self.redetect_direction_weight = self._env_float(
            "SWARM_REID_DIRECTION_WEIGHT", 0.08, 0.0, 0.5
        )
        self.local_search_factor = self._env_float(
            "SWARM_REID_LOCAL_SEARCH_FACTOR", 6.0, 2.0, 20.0
        )
        self.max_scale_change = self._env_float(
            "SWARM_TRACKING_MAX_SCALE_CHANGE", 1.8, 1.05, 5.0
        )
        self.max_center_jump_ratio = self._env_float(
            "SWARM_TRACKING_MAX_CENTER_JUMP_RATIO", 2.5, 0.5, 10.0
        )
        self.scale_update_every_n = self._env_int(
            "SWARM_TRACKING_SCALE_UPDATE_EVERY_N", 3, 1, 30
        )
        self.scale_kcf_reinit_ratio = self._env_float(
            "SWARM_TRACKING_SCALE_KCF_REINIT_RATIO", 0.05, 0.01, 0.5
        )
        raw_scale_candidates = os.environ.get(
            "SWARM_TRACKING_SCALE_CANDIDATES",
            "0.92,1.0,1.08",
        )
        try:
            scale_candidates = tuple(
                float(value.strip())
                for value in raw_scale_candidates.split(",")
                if value.strip()
            )
        except ValueError:
            scale_candidates = (0.92, 1.0, 1.08)
        self.scale_refiner = MultiScaleBBoxRefiner(
            enabled=os.environ.get(
                "SWARM_TRACKING_SCALE_REFINER_ENABLED", "true"
            ).strip().lower() in {"1", "true", "yes", "on"},
            candidates=scale_candidates,
            minimum_score=self._env_float(
                "SWARM_TRACKING_SCALE_MIN_SCORE", 0.68, 0.0, 1.0
            ),
            minimum_psr=self._env_float(
                "SWARM_TRACKING_SCALE_MIN_PSR", 3.5, 0.0, 20.0
            ),
            max_step_ratio=self._env_float(
                "SWARM_TRACKING_SCALE_MAX_STEP_RATIO", 0.18, 0.01, 0.8
            ),
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
        self.last_psr = 0.0
        self.last_shape_score = 0.0
        self.last_motion_score = 0.0
        self.last_candidate_count = 0
        self.last_ambiguous = False
        self.last_reject_reason = ""
        self.redetect_mode = "idle"
        self.raw_tracker_bbox: BBox | None = None
        self.scale_candidate_bbox: BBox | None = None
        self.refined_bbox: BBox | None = None
        self.last_scale_ratio = 1.0
        self.last_scale_score = 0.0
        self.last_scale_psr = 0.0
        self.last_scale_accepted = False
        self.last_scale_reason = "not_initialized"
        self.last_scale_scheduled = False
        self.last_scale_refinement_ms = 0.0
        self.last_correlation_score = 0.0
        self.last_texture_score = 0.0
        self.last_final_quality_score = 0.0
        self.kcf_reinit_count = 0
        self.scale_refiner.reset()
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
            "psr": round(self.last_psr, 3),
            "shape_score": round(self.last_shape_score, 3),
            "motion_score": round(self.last_motion_score, 3),
            "correlation_score": round(self.last_correlation_score, 3),
            "appearance_score": round(self.last_reid_score, 3),
            "texture_score": round(self.last_texture_score, 3),
            "final_quality_score": round(
                self.last_final_quality_score, 3
            ),
            "candidate_count": self.last_candidate_count,
            "ambiguous": self.last_ambiguous,
            "reject_reason": self.last_reject_reason,
            "redetect_mode": self.redetect_mode,
            "lightfc_robust_gates": True,
            "neural_core": False,
            "lost_count": self.lost_count,
            "kalman_velocity_px": [round(velocity[0], 2), round(velocity[1], 2)],
            "raw_tracker_bbox": self._bbox_list(self.raw_tracker_bbox),
            "scale_candidate_bbox": self._bbox_list(
                self.scale_candidate_bbox
            ),
            "refined_bbox": self._bbox_list(self.refined_bbox),
            "raw_tracker_area_px2": self._bbox_area(self.raw_tracker_bbox),
            "scale_candidate_area_px2": self._bbox_area(
                self.scale_candidate_bbox
            ),
            "refined_area_px2": self._bbox_area(self.refined_bbox),
            "scale_ratio": round(self.last_scale_ratio, 4),
            "scale_score": round(self.last_scale_score, 3),
            "scale_psr": round(self.last_scale_psr, 3),
            "scale_accepted": self.last_scale_accepted,
            "scale_reject_reason": (
                "" if self.last_scale_accepted else self.last_scale_reason
            ),
            "scale_refiner_enabled": self.scale_refiner.enabled,
            "scale_update_every_n": self.scale_update_every_n,
            "scale_scheduled": self.last_scale_scheduled,
            "scale_refinement_ms": round(self.last_scale_refinement_ms, 3),
            "kcf_reinit_count": self.kcf_reinit_count,
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
        self.scale_refiner.reset(crop)
        self.raw_tracker_bbox = clipped
        self.scale_candidate_bbox = clipped
        self.refined_bbox = clipped
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
            raw_candidate = self._clip_bbox(
                BBox(*map(float, raw_bbox)), frame.shape
            )
            self.raw_tracker_bbox = raw_candidate
            scale_result = self.scale_refiner._invalid(
                BBox.from_center(
                    raw_candidate.cx,
                    raw_candidate.cy,
                    self._bbox.w,
                    self._bbox.h,
                ),
                "not_scheduled",
            )
            self.last_scale_scheduled = bool(
                self.frame_idx % self.scale_update_every_n == 0
            )
            self.last_scale_refinement_ms = 0.0
            if self.last_scale_scheduled:
                scale_started = time.perf_counter()
                scale_result = self.scale_refiner.refine(
                    frame,
                    raw_candidate,
                    self._bbox,
                    appearance_similarity=self._appearance_similarity,
                )
                self.last_scale_refinement_ms = (
                    time.perf_counter() - scale_started
                ) * 1000.0
            candidate = self._clip_bbox(scale_result.bbox, frame.shape)
            self.scale_candidate_bbox = scale_result.candidate_bbox
            self.refined_bbox = candidate
            self.last_scale_ratio = scale_result.scale_ratio
            self.last_scale_score = scale_result.score
            self.last_scale_psr = scale_result.psr
            self.last_psr = scale_result.psr
            self.last_scale_accepted = scale_result.accepted
            self.last_scale_reason = scale_result.reason
            self.last_correlation_score = scale_result.correlation_score
            self.last_texture_score = scale_result.texture_score
            crop = self._crop(frame, candidate)
            reid_score = self._appearance_similarity(crop)
            self.last_reid_score = reid_score
            motion_score = self._motion_score(
                candidate,
                predicted,
                frame.shape,
                self._bbox,
            )
            shape_score = self._shape_score(candidate, self._bbox)
            self.last_motion_score = motion_score
            self.last_shape_score = shape_score
            scale_quality = (
                scale_result.score
                if scale_result.accepted
                else reid_score
            )
            self.last_match_score = float(np.clip(
                0.36 * reid_score
                + 0.24 * motion_score
                + 0.15 * shape_score
                + 0.25 * scale_quality,
                0.0,
                1.0,
            ))
            self.last_final_quality_score = self.last_match_score
            plausible, reject_reason = self._plausible_transition(
                candidate,
                self._bbox,
                predicted,
            )

            allow_weak_start = bool(
                self.frame_idx <= 2
                and reid_score >= 0.25
                and (
                    not self.last_scale_scheduled
                    or (
                        scale_result.accepted
                        and scale_result.score
                        >= self.scale_refiner.minimum_score
                    )
                )
            )
            if plausible and (
                self.last_match_score >= self.track_reid_threshold
                or allow_weak_start
            ):
                self.last_reject_reason = ""
                self.redetect_mode = "tracking"
                self._accept_tracking(frame, candidate, crop, reid_score)
                scale_change = max(
                    abs(candidate.w / max(raw_candidate.w, 1e-6) - 1.0),
                    abs(candidate.h / max(raw_candidate.h, 1e-6) - 1.0),
                )
                if (
                    scale_result.accepted
                    and scale_change >= self.scale_kcf_reinit_ratio
                ):
                    self._init_kcf(frame, candidate)
                    self.kcf_reinit_count += 1
                return self._result(self.last_match_score)

            self.last_reject_reason = (
                reject_reason
                if not plausible
                else "tracking_quality_below_threshold"
            )
            self.memory.add_blacklist(
                candidate.x, candidate.y, candidate.w, candidate.h
            )
        else:
            self.last_reject_reason = "correlation_tracker_lost"

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
            if self.last_scale_accepted:
                self.scale_refiner.update_reference(crop, reid_score)
        self.lost_count = 0
        self.reacquire_count = 0
        self.pending_bbox = None
        self.memory.exit_pos = None
        self.last_ambiguous = False
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
            self.last_reject_reason = "missing_template"
            return None

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        base_width, base_height = self._template_size
        search_gray, search_origin, search_mode = self._redetect_search_region(
            frame_gray,
            predicted,
            base_width,
            base_height,
        )
        self.redetect_mode = search_mode
        candidates: list[tuple[float, BBox, float, float]] = []
        response_peaks: list[tuple[float, BBox]] = []
        self.last_candidate_count = 0
        self.last_ambiguous = False
        self.last_psr = 0.0

        for scale in (0.75, 0.9, 1.0, 1.12, 1.28):
            width = max(8, int(round(base_width * scale)))
            height = max(8, int(round(base_height * scale)))
            if width >= search_gray.shape[1] or height >= search_gray.shape[0]:
                continue
            template = cv2.resize(
                self._template,
                (width, height),
                interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR,
            )
            template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            response = cv2.matchTemplate(
                search_gray, template_gray, cv2.TM_CCOEFF_NORMED
            )
            response_psr = self._psr(response)
            for _ in range(3):
                _, template_score, _, location = cv2.minMaxLoc(response)
                if not np.isfinite(template_score):
                    break
                bbox = BBox(
                    float(location[0] + search_origin[0]),
                    float(location[1] + search_origin[1]),
                    width,
                    height,
                )
                crop = self._crop(frame, bbox)
                reid_score = self._appearance_similarity(crop)
                motion_score = self._motion_score(
                    bbox,
                    predicted,
                    frame.shape,
                    self._bbox,
                )
                shape_score = self._shape_score(bbox, self._bbox)
                direction_score = self._direction_score(bbox)
                exit_score = 1.0 if self.memory.check_exit_return(
                    bbox.cx,
                    bbox.cy,
                    radius=max(width, height) * 5.0,
                ) else 0.0
                combined = (
                    0.29 * float(template_score)
                    + 0.34 * reid_score
                    + 0.12 * motion_score
                    + 0.08 * shape_score
                    + 0.05 * exit_score
                    + self.redetect_direction_weight * direction_score
                    + 0.04 * min(1.0, response_psr / 8.0)
                )
                plausible, _ = self._plausible_transition(
                    bbox,
                    self._bbox,
                    predicted,
                    redetecting=True,
                )
                if (
                    plausible
                    and response_psr >= self.redetect_psr_threshold
                    and reid_score >= self.redetect_reid_threshold
                    and not self.memory.is_blacklisted(
                        bbox.x, bbox.y, bbox.w, bbox.h
                    )
                ):
                    candidates.append(
                        (combined, bbox, float(template_score), response_psr)
                    )
                response_peaks.append((float(template_score), bbox))
                x0 = max(0, location[0] - width // 2)
                y0 = max(0, location[1] - height // 2)
                x1 = min(response.shape[1], location[0] + width // 2 + 1)
                y1 = min(response.shape[0], location[1] + height // 2 + 1)
                response[y0:y1, x0:x1] = -1.0

        self.last_candidate_count = len(candidates)
        ranked_peaks = sorted(response_peaks, key=lambda item: item[0], reverse=True)
        strongest = ranked_peaks[0] if ranked_peaks else None
        separated_second = None
        if strongest is not None:
            for peak in ranked_peaks[1:]:
                separation = float(
                    np.hypot(
                        peak[1].cx - strongest[1].cx,
                        peak[1].cy - strongest[1].cy,
                    )
                )
                if separation > 0.5 * min(
                    strongest[1].w,
                    strongest[1].h,
                    peak[1].w,
                    peak[1].h,
                ):
                    separated_second = peak
                    break
        self.last_ambiguous = bool(
            strongest is not None
            and separated_second is not None
            and strongest[0] > 1e-6
            and separated_second[0] / strongest[0]
            >= self.redetect_ambiguity_ratio
        )
        if not candidates:
            self.last_reject_reason = (
                "ambiguous_reidentification"
                if self.last_ambiguous
                else "no_reidentification_candidate"
            )
            return None
        score, bbox, _, psr = max(candidates, key=lambda item: item[0])
        self.last_match_score = float(score)
        self.last_psr = float(psr)
        crop = self._crop(frame, bbox)
        self.last_reid_score = self._appearance_similarity(crop)
        if score < self.redetect_score_threshold:
            self.last_reject_reason = "reidentification_score_below_threshold"
            return None
        if self.last_ambiguous and self._motion_score(
            bbox,
            predicted,
            frame.shape,
            self._bbox,
        ) < 0.55:
            self.last_reject_reason = "ambiguous_reidentification"
            return None
        self.last_reject_reason = ""
        return self._clip_bbox(bbox, frame.shape)

    def _redetect_search_region(
        self,
        frame_gray: Any,
        predicted: Any,
        base_width: int,
        base_height: int,
    ) -> tuple[Any, tuple[int, int], str]:
        if self.lost_count >= self.redetect_full_after:
            return frame_gray, (0, 0), "full_frame"

        if predicted is not None:
            center_x, center_y = float(predicted[0]), float(predicted[1])
        elif self._bbox is not None:
            center_x, center_y = self._bbox.cx, self._bbox.cy
        else:
            return frame_gray, (0, 0), "full_frame"

        half = int(
            round(
                0.5
                * self.local_search_factor
                * max(base_width, base_height, 8)
            )
        )
        height, width = frame_gray.shape[:2]
        x0 = max(0, int(round(center_x)) - half)
        y0 = max(0, int(round(center_y)) - half)
        x1 = min(width, int(round(center_x)) + half)
        y1 = min(height, int(round(center_y)) + half)
        if x1 - x0 <= base_width or y1 - y0 <= base_height:
            return frame_gray, (0, 0), "full_frame"
        return frame_gray[y0:y1, x0:x1], (x0, y0), "motion_local"

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
    def _motion_score(
        bbox: BBox,
        predicted: Any,
        shape: Any,
        reference: BBox | None = None,
    ) -> float:
        if predicted is None:
            return 1.0
        diagonal = (
            float(np.hypot(reference.w, reference.h)) * 4.0
            if reference is not None
            else float(np.hypot(shape[1], shape[0]))
        )
        distance = float(np.hypot(bbox.cx - predicted[0], bbox.cy - predicted[1]))
        return float(np.exp(-distance / max(diagonal, 1.0)))

    @staticmethod
    def _shape_score(bbox: BBox, reference: BBox | None) -> float:
        if reference is None:
            return 1.0
        width_ratio = max(
            bbox.w / max(reference.w, 1e-6),
            reference.w / max(bbox.w, 1e-6),
        )
        height_ratio = max(
            bbox.h / max(reference.h, 1e-6),
            reference.h / max(bbox.h, 1e-6),
        )
        aspect_ratio = max(
            (bbox.w / max(bbox.h, 1e-6))
            / max(reference.w / max(reference.h, 1e-6), 1e-6),
            (reference.w / max(reference.h, 1e-6))
            / max(bbox.w / max(bbox.h, 1e-6), 1e-6),
        )
        change = width_ratio * height_ratio * np.sqrt(aspect_ratio)
        return float(np.exp(-0.65 * max(0.0, change - 1.0)))

    def _plausible_transition(
        self,
        bbox: BBox,
        reference: BBox | None,
        predicted: Any,
        *,
        redetecting: bool = False,
    ) -> tuple[bool, str]:
        if reference is None:
            return True, ""
        width_change = max(
            bbox.w / max(reference.w, 1e-6),
            reference.w / max(bbox.w, 1e-6),
        )
        height_change = max(
            bbox.h / max(reference.h, 1e-6),
            reference.h / max(bbox.h, 1e-6),
        )
        scale_limit = self.max_scale_change * (1.35 if redetecting else 1.0)
        if max(width_change, height_change) > scale_limit:
            return False, "implausible_scale_change"
        anchor = (
            predicted
            if predicted is not None
            else (reference.cx, reference.cy)
        )
        jump = float(np.hypot(bbox.cx - anchor[0], bbox.cy - anchor[1]))
        target_diagonal = max(1.0, float(np.hypot(reference.w, reference.h)))
        jump_limit = self.max_center_jump_ratio * target_diagonal
        if redetecting:
            jump_limit *= max(1.0, min(3.0, self.lost_count / 4.0))
        if jump > jump_limit:
            return False, "implausible_center_jump"
        return True, ""

    def _direction_score(self, bbox: BBox) -> float:
        velocity = self.memory.get_avg_velocity()
        speed = float(np.hypot(velocity[0], velocity[1]))
        if speed < 0.4 or self._bbox is None:
            return 0.5
        displacement = (
            bbox.cx - self._bbox.cx,
            bbox.cy - self._bbox.cy,
        )
        distance = float(np.hypot(displacement[0], displacement[1]))
        if distance < 1e-6:
            return 0.5
        alignment = (
            displacement[0] * velocity[0]
            + displacement[1] * velocity[1]
        ) / (distance * speed)
        return float(np.clip(0.5 + 0.5 * alignment, 0.0, 1.0))

    @staticmethod
    def _psr(response: Any) -> float:
        finite = np.asarray(response, dtype=np.float32)
        finite = finite[np.isfinite(finite)]
        if finite.size < 2:
            return 0.0
        return float(
            (float(np.max(finite)) - float(np.mean(finite)))
            / (float(np.std(finite)) + 1e-6)
        )

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

    @staticmethod
    def _bbox_list(bbox: BBox | None) -> list[float] | None:
        return bbox.as_list() if bbox is not None else None

    @staticmethod
    def _bbox_area(bbox: BBox | None) -> float | None:
        return (
            round(float(bbox.w * bbox.h), 2)
            if bbox is not None
            else None
        )
