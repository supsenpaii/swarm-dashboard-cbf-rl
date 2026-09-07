import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from bearing_target_estimator import TargetEstimate
from tracking_web import (
    TrackingManager,
    apply_gimbal_pitch_direction,
    apply_gimbal_yaw_direction,
    camera_angular_error_deg,
    predict_bbox_center_px,
)
from follow_workflow import FollowWorkflowState
from visual_follow_target import BBoxMotionSafetyEstimator


class TrackingPointingBoundaryTests(unittest.TestCase):
    def test_gimbal_yaw_direction_is_explicitly_configurable(self):
        requested = math.radians(12.0)

        self.assertAlmostEqual(
            apply_gimbal_yaw_direction(requested, inverted=True),
            -requested,
        )
        self.assertAlmostEqual(
            apply_gimbal_yaw_direction(requested, inverted=False),
            requested,
        )

    def test_gimbal_pitch_direction_is_explicitly_configurable(self):
        requested = math.radians(10.0)

        self.assertAlmostEqual(
            apply_gimbal_pitch_direction(requested, inverted=True),
            -requested,
        )
        self.assertAlmostEqual(
            apply_gimbal_pitch_direction(requested, inverted=False),
            requested,
        )

    def test_bbox_selection_keeps_pointing_authority_until_follow_requested(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.visual_follow_feature_enabled = True
        manager.native_visual_follow_enabled = True
        manager.apparent_size_follow_enabled = False
        manager.visual_follow_requested = False
        manager.visual_follow_active = False
        manager.follow_workflow = SimpleNamespace(
            state=FollowWorkflowState.READY_FOR_FOLLOW
        )
        manager.drone_id = "UAV-02"
        manager._update_visual_follow_safely_locked = Mock()
        manager._update_body_yaw_locked = Mock()
        manager._stop_follow_locked = Mock()
        manager._update_motion_locked = Mock()
        manager._disable_body_yaw_locked = Mock()
        manager._update_native_precenter_yaw_locked = Mock()

        result = SimpleNamespace()
        manager._update_after_pointing_locked(
            SimpleNamespace(),
            result,
            1.0,
            0.025,
            True,
        )

        manager._update_visual_follow_safely_locked.assert_called_once()
        manager._update_body_yaw_locked.assert_called_once_with(
            result,
            1.0,
            0.025,
            True,
        )
        manager._update_motion_locked.assert_called_once_with(
            result,
            1.0,
            True,
        )
        manager._update_native_precenter_yaw_locked.assert_not_called()
        manager._disable_body_yaw_locked.assert_not_called()

    def test_dataset_collection_runs_metric_path_without_motion_authority(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.metric_target_fusion = SimpleNamespace(
            dataset_collection_enabled=True,
        )
        manager.visual_follow_feature_enabled = False
        manager.native_visual_follow_enabled = False
        manager.drone_id = "UAV-01"
        manager._update_visual_follow_safely_locked = Mock()
        manager._disable_body_yaw_locked = Mock()
        manager._stop_follow_locked = Mock()
        manager._stop_motion_locked = Mock()

        frame = SimpleNamespace()
        result = SimpleNamespace()
        manager._update_after_pointing_locked(
            frame,
            result,
            1.0,
            0.025,
            True,
        )

        manager._update_visual_follow_safely_locked.assert_called_once_with(
            frame,
            result,
            1.0,
            True,
        )
        manager._disable_body_yaw_locked.assert_called_once_with("UAV-01")
        manager._stop_follow_locked.assert_called_once_with(
            "UAV-01",
            "dataset_collection",
        )
        manager._stop_motion_locked.assert_called_once_with(
            "UAV-01",
            "dataset_collection",
        )
        self.assertEqual(
            manager.relative_visual_follow_state,
            "dataset_collection",
        )

    def test_bbox_selection_starts_relative_follow_while_metric_is_acquiring(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.visual_follow_feature_enabled = True
        manager.native_visual_follow_enabled = True
        manager.apparent_size_follow_enabled = True
        manager.visual_follow_requested = False
        manager.visual_follow_active = False
        manager.follow_reference_warmup_until_monotonic = 0.5
        manager.visual_motion_gate_reason = "ready"
        manager.follow_command_forward = 0.0
        manager.follow_block_reason = ""
        manager.pointing_guard_reason = "pointing_guard_valid"
        manager.follow_workflow = SimpleNamespace(
            state=FollowWorkflowState.ACQUIRING_RANGE
        )
        manager.drone_id = "UAV-02"
        manager._update_visual_follow_safely_locked = Mock()
        manager._update_body_yaw_locked = Mock()
        manager._update_follow_locked = Mock()
        manager._update_motion_locked = Mock()
        manager._stop_follow_locked = Mock()
        manager._disable_body_yaw_locked = Mock()

        result = SimpleNamespace()
        manager._update_after_pointing_locked(
            SimpleNamespace(),
            result,
            1.0,
            0.025,
            True,
        )

        manager._update_visual_follow_safely_locked.assert_called_once()
        manager._update_follow_locked.assert_called_once_with(
            result,
            1.0,
            0.025,
            True,
        )
        manager._update_motion_locked.assert_called_once_with(
            result,
            1.0,
            True,
        )
        manager._stop_follow_locked.assert_not_called()
        self.assertEqual(
            manager.relative_visual_follow_state,
            "tracking",
        )
        self.assertFalse(manager.visual_follow_requested)

    def test_relative_follow_waits_for_selection_scale_warmup(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.visual_follow_feature_enabled = True
        manager.native_visual_follow_enabled = True
        manager.apparent_size_follow_enabled = True
        manager.visual_follow_requested = False
        manager.visual_follow_active = False
        manager.follow_reference_warmup_until_monotonic = 2.0
        manager.pointing_guard_reason = "pointing_guard_valid"
        manager.follow_workflow = SimpleNamespace(
            state=FollowWorkflowState.ACQUIRING_RANGE
        )
        manager.drone_id = "UAV-02"
        manager._update_visual_follow_safely_locked = Mock()
        manager._update_body_yaw_locked = Mock()
        manager._update_follow_locked = Mock()
        manager._update_motion_locked = Mock()
        manager._stop_follow_locked = Mock()

        manager._update_after_pointing_locked(
            SimpleNamespace(),
            SimpleNamespace(),
            1.0,
            0.025,
            True,
        )

        manager._update_follow_locked.assert_not_called()
        manager._stop_follow_locked.assert_called_once_with(
            "UAV-02",
            "reference_warmup",
        )
        manager._update_motion_locked.assert_called_once()
        self.assertEqual(
            manager.relative_visual_follow_block_reason,
            "reference_warmup",
        )

    def test_ttc_gate_blocks_only_forward_relative_command(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.visual_follow_feature_enabled = True
        manager.native_visual_follow_enabled = True
        manager.apparent_size_follow_enabled = True
        manager.visual_follow_requested = False
        manager.visual_follow_active = False
        manager.follow_reference_warmup_until_monotonic = 0.0
        manager.visual_motion_gate_reason = "ttc_block"
        manager.follow_command_forward = 0.0
        manager.follow_block_reason = ""
        manager.pointing_guard_reason = "pointing_guard_valid"
        manager.follow_workflow = SimpleNamespace(
            state=FollowWorkflowState.ACQUIRING_RANGE
        )
        manager.drone_id = "UAV-02"
        manager._update_visual_follow_safely_locked = Mock()
        manager._update_body_yaw_locked = Mock()

        def command_forward(*_args):
            manager.follow_command_forward = 0.2
            manager.follow_state = "moving_forward"

        manager._update_follow_locked = Mock(side_effect=command_forward)
        manager._update_motion_locked = Mock()
        manager._stop_follow_locked = Mock()

        manager._update_after_pointing_locked(
            SimpleNamespace(),
            SimpleNamespace(),
            1.0,
            0.025,
            True,
        )

        self.assertEqual(manager.follow_command_forward, 0.0)
        self.assertEqual(manager.follow_state, "ttc_block")
        self.assertEqual(manager.follow_block_reason, "ttc_block")

    def test_hold_keeps_gimbal_but_cannot_reacquire_offboard_body_yaw(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.visual_follow_feature_enabled = True
        manager.native_visual_follow_enabled = True
        manager.apparent_size_follow_enabled = False
        manager.visual_follow_requested = False
        manager.visual_follow_active = False
        manager.follow_workflow = SimpleNamespace(
            state=FollowWorkflowState.HOLD
        )
        manager.drone_id = "UAV-02"
        manager._update_visual_follow_safely_locked = Mock()
        manager._update_body_yaw_locked = Mock()
        manager._stop_follow_locked = Mock()
        manager._update_motion_locked = Mock()
        manager._stop_motion_locked = Mock()
        manager._disable_body_yaw_locked = Mock()

        manager._update_after_pointing_locked(
            SimpleNamespace(),
            SimpleNamespace(),
            1.0,
            0.025,
            True,
        )

        manager._update_visual_follow_safely_locked.assert_called_once()
        manager._disable_body_yaw_locked.assert_called_once_with("UAV-02")
        manager._stop_motion_locked.assert_called_once_with(
            "UAV-02",
            "follow_hold",
        )
        manager._update_body_yaw_locked.assert_not_called()
        manager._update_motion_locked.assert_not_called()

    def test_authorized_boundary_loss_uses_brief_grace_then_enters_hold(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.drone_id = "UAV-02"
        manager.visual_target_command = Mock(return_value={"ok": True})
        manager.visual_follow_requested = True
        manager.visual_follow_active = False
        manager.visual_target_stream_active = True
        manager.visual_follow_state = "follow_prestream"
        manager.visual_follow_last_stop_reason = ""
        manager.visual_follow_last_stop_monotonic = None
        manager.visual_follow_boundary_invalid_since_monotonic = None
        manager.visual_follow_last_publish_monotonic = 1.0
        manager.visual_follow_ready_since_monotonic = 1.0
        manager.visual_follow_center_since_monotonic = 1.0
        manager.visual_follow_handover_started_monotonic = 1.0
        manager.visual_follow_handover_blend = 1.0
        manager.visual_follow_center_error_deg = 2.0
        manager.visual_follow_error = ""
        manager.visual_follow_ready = True
        manager.visual_follow_dropout_grace_s = 1.5
        manager._stop_bootstrap_motion_locked = Mock()
        manager.follow_workflow = SimpleNamespace(
            state=FollowWorkflowState.FOLLOW_PRESTREAM,
            transition=Mock(),
        )

        manager._stop_visual_follow_at_boundary_locked("no_bbox", 2.0)

        self.assertTrue(manager.visual_follow_requested)
        self.assertTrue(manager.visual_target_stream_active)
        self.assertEqual(manager.visual_follow_state, "boundary_grace")
        manager.follow_workflow.transition.assert_not_called()

        manager._stop_visual_follow_at_boundary_locked("no_bbox", 2.51)

        self.assertTrue(manager.visual_follow_requested)
        manager._stop_visual_follow_at_boundary_locked("no_bbox", 3.51)

        self.assertFalse(manager.visual_follow_requested)
        self.assertFalse(manager.visual_follow_active)
        self.assertFalse(manager.visual_target_stream_active)
        self.assertEqual(manager.visual_follow_last_stop_reason, "no_bbox")
        manager._stop_bootstrap_motion_locked.assert_called_once_with("no_bbox")
        manager.follow_workflow.transition.assert_called_once_with(
            FollowWorkflowState.HOLD,
            "no_bbox",
            timestamp_s=3.51,
            force=True,
        )
        manager.visual_target_command.assert_called_once_with(
            "UAV-02",
            False,
            None,
            "no_bbox",
        )

    def test_angular_error_is_resolution_invariant(self):
        reference = camera_angular_error_deg(
            center_x_px=420.0,
            center_y_px=130.0,
            frame_width_px=640,
            frame_height_px=360,
            calibrated_fx_px=205.5,
            calibrated_fy_px=205.5,
            calibration_width_px=640,
            calibration_height_px=360,
        )
        doubled = camera_angular_error_deg(
            center_x_px=840.0,
            center_y_px=260.0,
            frame_width_px=1280,
            frame_height_px=720,
            calibrated_fx_px=205.5,
            calibrated_fy_px=205.5,
            calibration_width_px=640,
            calibration_height_px=360,
        )

        self.assertAlmostEqual(reference[0], doubled[0], places=9)
        self.assertAlmostEqual(reference[1], doubled[1], places=9)
        self.assertAlmostEqual(
            reference[0],
            math.degrees(math.atan2(100.0, 205.5)),
            places=9,
        )

    def test_bbox_prediction_is_latency_and_frame_clamped(self):
        predicted_x, predicted_y, horizon = predict_bbox_center_px(
            center_x_px=630.0,
            center_y_px=10.0,
            velocity_x_px_per_frame=20.0,
            velocity_y_px_per_frame=-20.0,
            source_fps=30.0,
            latency_s=0.5,
            maximum_horizon_s=0.1,
            frame_width_px=640,
            frame_height_px=360,
        )

        self.assertEqual(predicted_x, 639.0)
        self.assertEqual(predicted_y, 0.0)
        self.assertAlmostEqual(horizon, 0.1)

    def test_bbox_motion_integration_uses_geometric_bbox_scale(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.visual_bbox_filter_state = "ready"
        manager.bbox_motion_safety_estimator = BBoxMotionSafetyEstimator(
            stable_frames_required=1
        )
        manager.follow_min_score = 0.70
        manager.gimbal_error = ""
        manager.visual_bbox_stable_frames = 1
        manager.visual_bbox_filter = SimpleNamespace(stable_frames_required=1)
        manager.visual_range_raw_m = None
        manager.visual_range_filtered_m = None
        manager.visual_bbox_scale_px = None
        manager.visual_range_quality = 0.0
        manager.visual_ttc_s = None
        manager.follow_distance_m = None
        manager.follow_distance_source = "none"
        manager.visual_follow_ready = False
        manager.visual_follow_state = "idle"
        manager.visual_follow_error = ""
        manager.visual_follow_requested = False
        manager.visual_follow_active = False
        manager._update_bearing_target_locked = lambda *_args: TargetEstimate(
            timestamp_s=1.0,
            state="BOOTSTRAPPING",
            valid=False,
            reason="insufficient_baseline",
        )
        manager._stop_visual_follow_locked = lambda _reason: None

        frame = SimpleNamespace(shape=(360, 640, 3))
        bbox = SimpleNamespace(
            x=270.0,
            y=150.0,
            w=100.0,
            h=64.0,
            cx=320.0,
            cy=182.0,
        )
        result = SimpleNamespace(
            state=SimpleNamespace(value="tracking"),
            score=0.99,
            bbox=bbox,
        )

        manager._update_visual_follow_locked(frame, result, 1.0, True)

        self.assertAlmostEqual(
            manager.visual_bbox_scale_px,
            math.sqrt(bbox.w * bbox.h),
        )
        self.assertEqual(manager.visual_follow_state, "bootstrapping")

    def test_estimator_exception_does_not_reset_pointing_controller(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.gimbal_controller = Mock()
        manager.gimbal_error = ""
        manager.target_estimator_error = ""
        manager.visual_follow_error = ""
        manager.visual_follow_ready = True
        manager.visual_follow_state = "ready"
        manager.visual_follow_requested = False
        manager.visual_follow_active = False
        manager._update_visual_follow_locked = Mock(
            side_effect=ValueError("synthetic estimator fault")
        )
        manager._stop_visual_follow_locked = Mock()

        manager._update_visual_follow_safely_locked(
            SimpleNamespace(),
            SimpleNamespace(),
            1.0,
            True,
        )

        manager.gimbal_controller.reset.assert_not_called()
        self.assertEqual(manager.gimbal_error, "")
        self.assertEqual(manager.visual_follow_state, "degraded")
        self.assertIn("synthetic estimator fault", manager.visual_follow_error)

    def test_body_yaw_gate_does_not_deadlock_on_bbox_stabilization(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.tracker = object()
        manager._tracker_diagnostics_locked = lambda: {}
        manager.gimbal_error = ""
        manager.visual_bbox_filter = SimpleNamespace(stable_frames_required=10)
        manager.visual_bbox_stable_frames = 0
        result = SimpleNamespace(
            state=SimpleNamespace(value="tracking"),
            score=0.99,
            bbox=SimpleNamespace(),
            redetecting=False,
        )

        yaw_allowed, yaw_reason = manager._tracking_motion_gate_locked(
            result,
            True,
            0.70,
            require_bbox_stable=False,
        )
        translation_allowed, translation_reason = (
            manager._tracking_motion_gate_locked(result, True, 0.70)
        )

        self.assertTrue(yaw_allowed)
        self.assertEqual(yaw_reason, "")
        self.assertFalse(translation_allowed)
        self.assertEqual(translation_reason, "bbox_stabilizing")


if __name__ == "__main__":
    unittest.main()
