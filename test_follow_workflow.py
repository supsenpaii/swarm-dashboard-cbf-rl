import unittest
import threading
import os
from types import SimpleNamespace
from unittest.mock import patch

from bearing_target_estimator import TargetEstimate
from follow_workflow import (
    FollowWorkflowMachine,
    FollowWorkflowState,
    SelectionSnapshot,
)
from tracking_web import (
    TrackingManager,
    lateral_bootstrap_unit_ned,
    px4_follow_mode_confirmed,
)
from visual_follow_target import CameraRayProjector, TargetStateFilter


class FollowWorkflowMachineTests(unittest.TestCase):
    def test_auto_start_environment_override_is_ignored(self) -> None:
        with patch.dict(
            os.environ,
            {"SWARM_VISUAL_FOLLOW_AUTO_START": "true"},
        ):
            manager = TrackingManager(None, set())
        self.assertFalse(manager.visual_follow_auto_start)
        self.assertFalse(manager.visual_follow_auto_start_pending)

    def test_explicit_pointing_then_user_authorized_bootstrap(self) -> None:
        workflow = FollowWorkflowMachine()
        session_id = workflow.begin_tracking(10.0)
        workflow.select_target(
            snapshot_valid=True,
            reason="bbox_selected",
            timestamp_s=11.0,
        )
        workflow.transition(
            FollowWorkflowState.POINTING_STABLE,
            "pointing_guard_valid",
            timestamp_s=12.0,
        )
        workflow.transition(
            FollowWorkflowState.READY_FOR_FOLLOW,
            "pointing_stable_hold_complete",
            timestamp_s=13.0,
        )

        self.assertEqual(session_id, 1)
        self.assertEqual(
            workflow.state,
            FollowWorkflowState.READY_FOR_FOLLOW,
        )
        self.assertNotEqual(
            workflow.state,
            FollowWorkflowState.RGB_BOOTSTRAP,
        )

        workflow.transition(
            FollowWorkflowState.RGB_BOOTSTRAP,
            "user_authorized_follow",
            timestamp_s=14.0,
            timeout_s=4.0,
        )
        self.assertEqual(workflow.state, FollowWorkflowState.RGB_BOOTSTRAP)
        self.assertEqual(workflow.status(15.0)["timeout_remaining_s"], 3.0)

    def test_invalid_transition_is_rejected(self) -> None:
        workflow = FollowWorkflowMachine()
        workflow.begin_tracking(1.0)
        with self.assertRaisesRegex(ValueError, "invalid workflow transition"):
            workflow.transition(
                FollowWorkflowState.FOLLOWING,
                "must_not_skip_handover",
                timestamp_s=2.0,
            )

    def test_follow_confirmation_requires_ack_and_nav_state(self) -> None:
        self.assertFalse(px4_follow_mode_confirmed(19, "pending"))
        self.assertFalse(px4_follow_mode_confirmed(2, "accepted"))
        self.assertTrue(px4_follow_mode_confirmed(19, "accepted"))

    def test_new_bbox_gets_a_new_target_session(self) -> None:
        workflow = FollowWorkflowMachine()
        tracking_session = workflow.begin_tracking(1.0)
        first_target_session = workflow.begin_target_session()
        second_target_session = workflow.begin_target_session()
        self.assertEqual(tracking_session, 1)
        self.assertEqual(first_target_session, 2)
        self.assertEqual(second_target_session, 3)

    def test_selection_snapshot_is_immutable(self) -> None:
        snapshot = SelectionSnapshot(
            session_id=4,
            selection_timestamp_s=20.0,
            selection_frame_index=99,
            selection_vehicle_position_ned=(1.0, 2.0, -5.0),
            selection_camera_position_ned=(1.1, 2.0, -5.1),
            selection_vehicle_global_position=(10.0, 106.0, 15.0),
            selection_camera_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            selection_bearing_ned=(1.0, 0.0, 0.0),
            selection_bbox=(10.0, 20.0, 30.0, 40.0),
            selection_bbox_scale_px=(30.0 * 40.0) ** 0.5,
            selection_heading_rad=0.5,
            selection_altitude_m=5.0,
            selection_pose_age_ms=4.0,
            selection_gimbal_age_ms=6.0,
            valid=True,
            reason="ok",
        )
        with self.assertRaises(AttributeError):
            snapshot.selection_frame_index = 100  # type: ignore[misc]

        status = snapshot.status()
        self.assertEqual(status["selection_vehicle_position_ned"], (1.0, 2.0, -5.0))
        self.assertEqual(status["selection_bbox_scale_px"], (1200.0) ** 0.5)

    def test_selection_snapshot_uses_pose_at_selected_frame_timestamp(self) -> None:
        requested_timestamps = []

        def pose_provider(_drone_id, timestamp_s):
            requested_timestamps.append(timestamp_s)
            return {
                "available": True,
                "vehicle_position_ned_m": (1.0, 2.0, -8.0),
                "camera_position_ned_m": (1.1, 2.0, -8.1),
                "synchronized_global_position": (10.0, 106.0, 18.0),
                "camera_quaternion_xyzw": (0.0, 0.0, 0.0, 1.0),
                "synchronized_heading_rad": 0.25,
                "telemetry_age_ms": 5.0,
                "camera_age_ms": 7.0,
            }

        manager = TrackingManager.__new__(TrackingManager)
        manager.current_source_timestamp_s = 42.5
        manager.source_frame_index = 123
        manager.follow_workflow = FollowWorkflowMachine()
        manager.follow_workflow.begin_tracking(40.0)
        manager.drone_id = "UAV-02"
        manager.pose_provider = pose_provider
        manager.selection_pose_max_age_ms = 120.0
        manager.selection_gimbal_max_age_ms = 120.0
        manager.visual_ray_projector = CameraRayProjector(200.0, 200.0)
        bbox = SimpleNamespace(
            x=270.0,
            y=150.0,
            w=100.0,
            h=64.0,
            cx=320.0,
            cy=182.0,
        )

        snapshot = manager._capture_selection_snapshot_locked(
            bbox,
            640,
            360,
        )

        self.assertTrue(snapshot.valid)
        self.assertEqual(requested_timestamps, [42.5])
        self.assertEqual(snapshot.selection_frame_index, 123)
        self.assertEqual(
            snapshot.selection_vehicle_position_ned,
            (1.0, 2.0, -8.0),
        )
        self.assertEqual(snapshot.selection_timestamp_s, 42.5)

    def test_stale_selection_snapshot_is_reacquired_without_bbox_reset(self) -> None:
        manager = TrackingManager.__new__(TrackingManager)
        manager.follow_workflow = FollowWorkflowMachine()
        manager.follow_workflow.transition(
            FollowWorkflowState.POINTING,
            "selection_gimbal_pose_stale",
            timestamp_s=1.0,
            force=True,
        )
        manager.selection_snapshot = SelectionSnapshot(
            session_id=manager.follow_workflow.session_id,
            selection_timestamp_s=1.0,
            selection_frame_index=1,
            selection_vehicle_position_ned=None,
            selection_camera_position_ned=None,
            selection_vehicle_global_position=None,
            selection_camera_quaternion_xyzw=None,
            selection_bearing_ned=None,
            selection_bbox=(270.0, 150.0, 100.0, 64.0),
            selection_bbox_scale_px=80.0,
            selection_heading_rad=None,
            selection_altitude_m=None,
            selection_pose_age_ms=4.0,
            selection_gimbal_age_ms=141.0,
            valid=False,
            reason="selection_gimbal_pose_stale",
        )
        manager.current_source_timestamp_s = 2.0
        manager.source_frame_index = 2
        manager.drone_id = "UAV-02"
        manager.selection_pose_max_age_ms = 120.0
        manager.selection_gimbal_max_age_ms = 120.0
        manager.visual_ray_projector = CameraRayProjector(200.0, 200.0)
        manager.pose_provider = lambda _drone_id, _timestamp_s: {
            "available": True,
            "vehicle_position_ned_m": (1.0, 2.0, -8.0),
            "camera_position_ned_m": (1.1, 2.0, -8.1),
            "synchronized_global_position": (10.0, 106.0, 18.0),
            "camera_quaternion_xyzw": (0.0, 0.0, 0.0, 1.0),
            "synchronized_heading_rad": 0.25,
            "telemetry_age_ms": 5.0,
            "camera_age_ms": 7.0,
        }
        manager._pointing_follow_guard_locked = (
            lambda _frame, _result, _gimbal_ok: (False, "test_stop")
        )
        manager.visual_follow_center_since_monotonic = None
        manager.visual_follow_ready_since_monotonic = None
        manager.visual_follow_ready = False
        bbox = SimpleNamespace(
            x=270.0,
            y=150.0,
            w=100.0,
            h=64.0,
            cx=320.0,
            cy=182.0,
        )
        result = SimpleNamespace(bbox=bbox)

        manager._update_pointing_workflow_locked(
            SimpleNamespace(shape=(360, 640, 3)),
            result,
            2.0,
            True,
        )

        self.assertTrue(manager.selection_snapshot.valid)
        self.assertIs(result.bbox, bbox)
        self.assertEqual(
            manager.selection_snapshot.selection_frame_index,
            2,
        )

    def test_initial_distance_uses_selection_not_post_bootstrap_pose(self) -> None:
        manager = TrackingManager.__new__(TrackingManager)
        manager.selection_snapshot = SelectionSnapshot(
            session_id=1,
            selection_timestamp_s=1.0,
            selection_frame_index=2,
            selection_vehicle_position_ned=(0.0, 0.0, -8.0),
            selection_camera_position_ned=(0.1, 0.0, -8.0),
            selection_vehicle_global_position=(10.0, 106.0, 18.0),
            selection_camera_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            selection_bearing_ned=(1.0, 0.0, 0.0),
            selection_bbox=(1.0, 2.0, 30.0, 40.0),
            selection_bbox_scale_px=(1200.0) ** 0.5,
            selection_heading_rad=0.0,
            selection_altitude_m=8.0,
            selection_pose_age_ms=1.0,
            selection_gimbal_age_ms=1.0,
            valid=True,
            reason="ok",
        )
        estimate = TargetEstimate(
            timestamp_s=2.0,
            state="VALID",
            valid=True,
            reason="ready",
            position_ned_m=(0.0, 10.0, -7.0),
            range_std_m=0.4,
        )

        self.assertTrue(manager._save_initial_safe_distance_locked(estimate))
        self.assertAlmostEqual(manager.initial_safe_distance_horizontal_m, 10.0)
        self.assertAlmostEqual(
            manager.initial_safe_distance_slant_m,
            101.0 ** 0.5,
        )
        self.assertAlmostEqual(manager.initial_vertical_offset_m, 1.0)
        self.assertAlmostEqual(manager.initial_follow_angle_deg, -90.0)
        self.assertEqual(
            manager.initial_follow_angle_source,
            "selection_target_to_vehicle_bearing",
        )
        self.assertEqual(manager.initial_safe_distance_lock_count, 1)
        first_lock_timestamp = manager.initial_safe_distance_lock_timestamp_s
        second = TargetEstimate(
            timestamp_s=3.0,
            state="VALID",
            valid=True,
            reason="ready",
            position_ned_m=(0.0, 15.0, -7.0),
            range_std_m=0.3,
        )
        self.assertTrue(manager._save_initial_safe_distance_locked(second))
        self.assertAlmostEqual(manager.initial_safe_distance_horizontal_m, 10.0)
        self.assertEqual(manager.initial_safe_distance_lock_count, 1)
        self.assertEqual(
            manager.initial_safe_distance_lock_timestamp_s,
            first_lock_timestamp,
        )

    def test_selection_anchor_initializes_without_locking_safe_distance(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.selection_snapshot = SelectionSnapshot(
            session_id=1,
            selection_timestamp_s=1.0,
            selection_frame_index=2,
            selection_vehicle_position_ned=(0.0, 0.0, -8.0),
            selection_camera_position_ned=(0.1, 0.0, -8.1),
            selection_vehicle_global_position=(10.0, 106.0, 18.0),
            selection_camera_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            selection_bearing_ned=(1.0, 0.0, 0.0),
            selection_bbox=(1.0, 2.0, 30.0, 40.0),
            selection_bbox_scale_px=(1200.0) ** 0.5,
            selection_heading_rad=0.0,
            selection_altitude_m=8.0,
            selection_pose_age_ms=1.0,
            selection_gimbal_age_ms=1.0,
            valid=True,
            reason="ok",
        )
        manager.follow_distance_target_m = 10.0
        manager.provisional_target_filter = TargetStateFilter()
        manager.provisional_target_estimate = TargetEstimate(
            timestamp_s=0.0,
            state="PROVISIONAL",
            valid=False,
            reason="not_initialized",
        )
        manager.provisional_target_initial_slant_range_m = None
        manager.provisional_target_range_m = None
        manager.provisional_target_update_count = 0
        manager.provisional_target_accepted_count = 0
        manager.provisional_target_source = "none"
        manager.initial_safe_distance_horizontal_m = None
        manager.initial_safe_distance_slant_m = None
        manager.initial_safe_distance_lock_timestamp_s = None
        manager.initial_safe_distance_lock_count = 0
        manager.initial_safe_distance_source = "none"
        manager.initial_vertical_offset_m = None
        manager.initial_bbox_scale_px = None
        manager.initial_range_std_m = None

        first = manager._initialize_provisional_target_locked(2.0)
        second = manager._initialize_provisional_target_locked(3.0)

        self.assertTrue(first.valid)
        self.assertIs(second, first)
        self.assertIsNone(manager.initial_safe_distance_horizontal_m)
        self.assertIsNone(manager.initial_safe_distance_slant_m)
        self.assertEqual(manager.initial_safe_distance_lock_count, 0)
        self.assertEqual(manager.initial_safe_distance_source, "none")
        self.assertEqual(
            manager.initial_bbox_scale_px,
            manager.selection_snapshot.selection_bbox_scale_px,
        )
        self.assertEqual(first.velocity_ned_m_s, (0.0, 0.0, 0.0))
        self.assertFalse(first.velocity_valid)

    def test_initial_distance_rejects_invalid_estimate(self) -> None:
        manager = TrackingManager.__new__(TrackingManager)
        manager.initial_safe_distance_horizontal_m = None
        manager.metric_target_fusion = SimpleNamespace(
            operational=True,
            ready_for_safe_distance_lock=True,
        )
        manager.selection_snapshot = SelectionSnapshot(
            session_id=1,
            selection_timestamp_s=1.0,
            selection_frame_index=2,
            selection_vehicle_position_ned=(0.0, 0.0, -8.0),
            selection_camera_position_ned=(0.1, 0.0, -8.0),
            selection_vehicle_global_position=(10.0, 106.0, 18.0),
            selection_camera_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            selection_bearing_ned=(1.0, 0.0, 0.0),
            selection_bbox=(1.0, 2.0, 30.0, 40.0),
            selection_bbox_scale_px=(1200.0) ** 0.5,
            selection_heading_rad=0.0,
            selection_altitude_m=8.0,
            selection_pose_age_ms=1.0,
            selection_gimbal_age_ms=1.0,
            valid=True,
            reason="ok",
        )
        invalid = TargetEstimate(
            timestamp_s=2.0,
            state="INVALID",
            valid=False,
            reason="stale",
            position_ned_m=(10.0, 0.0, -8.0),
            range_std_m=0.4,
        )

        self.assertFalse(manager._save_initial_safe_distance_locked(invalid))
        self.assertIsNone(manager.initial_safe_distance_horizontal_m)

    def test_manual_authorization_rejects_selection_anchored_target(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.lock = threading.Lock()
        manager.visual_follow_feature_enabled = True
        manager.active = True
        manager.tracker = SimpleNamespace(active=True)
        manager.selection_snapshot = SimpleNamespace(valid=True)
        manager.visual_follow_requested = False
        manager.visual_follow_state = "target_initialized"
        manager.visual_follow_error = ""
        manager.initial_safe_distance_horizontal_m = 10.0
        manager.metric_target_fusion = SimpleNamespace(operational=True)
        manager.provisional_target_source = "selection_anchor"
        manager.provisional_target_metric_confirmed = False
        manager.target_estimate = TargetEstimate(
            timestamp_s=2.0,
            state="PROVISIONAL",
            valid=True,
            reason="selection_anchor_tracking",
            position_ned_m=(10.0, 0.0, -8.0),
            velocity_ned_m_s=(0.0, 0.0, 0.0),
            range_std_m=3.0,
        )
        manager.follow_workflow = FollowWorkflowMachine()
        manager.follow_workflow.transition(
            FollowWorkflowState.TARGET_INITIALIZED,
            "selection_anchor_initialized",
            force=True,
        )
        manager.status = lambda: {"ok": True}

        with self.assertRaisesRegex(RuntimeError, "converged measured target"):
            manager.set_visual_follow_requested(True)

        self.assertFalse(manager.visual_follow_requested)

    def test_metric_handover_keeps_velocity_invalid_until_complete(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.metric_target_fusion = SimpleNamespace(
            ready_for_safe_distance_lock=True,
            active_metric_source="multiview",
        )
        manager.visual_follow_handover_s = 2.0
        manager.provisional_target_metric_handover_started_s = None
        manager.provisional_target_metric_handover_origin_ned = None
        manager.provisional_target_metric_blend = 0.0
        manager.provisional_target_metric_confirmed = False
        manager.provisional_target_source = "selection_anchor_cv2"
        provisional = TargetEstimate(
            timestamp_s=1.0,
            state="PROVISIONAL",
            valid=True,
            reason="ready",
            position_ned_m=(10.0, 0.0, -8.0),
            velocity_ned_m_s=(0.0, 0.0, 0.0),
            covariance=((9.0, 0.0, 0.0),) * 3,
            range_m=10.0,
            range_std_m=3.0,
        )
        metric = TargetEstimate(
            timestamp_s=1.0,
            state="METRIC_FUSION",
            valid=True,
            reason="ready",
            position_ned_m=(14.0, 2.0, -8.0),
            velocity_ned_m_s=(1.0, 0.0, 0.0),
            covariance=(
                (0.25, 0.0, 0.0),
                (0.0, 0.25, 0.0),
                (0.0, 0.0, 0.25),
            ),
            range_m=14.2,
            range_std_m=0.5,
            velocity_valid=True,
        )

        start = manager._select_follow_target_estimate_locked(
            provisional, metric, 1.0
        )
        middle = manager._select_follow_target_estimate_locked(
            provisional, metric, 2.0
        )
        complete = manager._select_follow_target_estimate_locked(
            provisional, metric, 3.1
        )

        self.assertEqual(start.position_ned_m, provisional.position_ned_m)
        self.assertEqual(middle.position_ned_m, (12.0, 1.0, -8.0))
        self.assertFalse(middle.velocity_valid)
        self.assertEqual(middle.velocity_ned_m_s, (0.0, 0.0, 0.0))
        self.assertIs(complete, metric)
        self.assertTrue(manager.provisional_target_metric_confirmed)

    def test_bootstrap_direction_is_local_ned_not_body_heading(self) -> None:
        right = lateral_bootstrap_unit_ned((0.6, 0.8, 0.0), "right")
        left = lateral_bootstrap_unit_ned((0.6, 0.8, 0.0), "left")
        self.assertAlmostEqual(right[0], -0.8)
        self.assertAlmostEqual(right[1], 0.6)
        self.assertAlmostEqual(left[0], 0.8)
        self.assertAlmostEqual(left[1], -0.6)
        self.assertAlmostEqual(0.6 * right[0] + 0.8 * right[1], 0.0)

    def test_bootstrap_travel_origin_uses_authorization_pose_not_selection(self):
        requested_timestamps = []

        def pose_provider(_drone_id, timestamp_s):
            requested_timestamps.append(timestamp_s)
            return {
                "available": True,
                "vehicle_position_ned_m": (2.0, -3.0, -8.5),
                "telemetry_age_ms": 4.0,
                "camera_age_ms": 6.0,
            }

        manager = TrackingManager.__new__(TrackingManager)
        manager.drone_id = "UAV-02"
        manager.current_source_timestamp_s = 50.0
        manager.pose_provider = pose_provider
        manager.selection_pose_max_age_ms = 200.0
        manager.selection_gimbal_max_age_ms = 120.0

        position = manager._bootstrap_start_position_locked()

        self.assertEqual(position, (2.0, -3.0, -8.5))
        self.assertEqual(requested_timestamps, [50.0])

    def test_metric_target_can_skip_rgb_bootstrap(self) -> None:
        machine = FollowWorkflowMachine()
        machine.transition(
            FollowWorkflowState.READY_FOR_FOLLOW,
            "metric_ready",
            timestamp_s=1.0,
            force=True,
        )
        machine.transition(
            FollowWorkflowState.TARGET_3D_READY,
            "user_authorized_metric_target",
            timestamp_s=1.1,
        )
        self.assertEqual(
            machine.state,
            FollowWorkflowState.TARGET_3D_READY,
        )

    def test_serialized_workflow_uses_canonical_visual_follow_names(self) -> None:
        machine = FollowWorkflowMachine()
        machine.transition(
            FollowWorkflowState.ACQUIRING_RANGE,
            "bbox_stable",
            timestamp_s=1.0,
            force=True,
        )
        self.assertEqual(machine.status(1.0)["state"], "ACQUIRING_RANGE")
        machine.transition(
            FollowWorkflowState.TARGET_INITIALIZED,
            "range_converged",
            timestamp_s=2.0,
        )
        self.assertEqual(machine.status(2.0)["state"], "TARGET_INITIALIZED")
        machine.transition(
            FollowWorkflowState.PRESTREAMING_TARGET,
            "stream_started",
            timestamp_s=3.0,
        )
        self.assertEqual(machine.status(3.0)["state"], "PRESTREAMING_TARGET")
        machine.transition(
            FollowWorkflowState.REQUESTING_FOLLOW_MODE,
            "prestream_complete",
            timestamp_s=4.0,
        )
        self.assertEqual(
            machine.status(4.0)["state"],
            "REQUESTING_FOLLOW_MODE",
        )

    def test_start_follow_preserves_tracker_and_selected_bbox(self) -> None:
        manager = TrackingManager.__new__(TrackingManager)
        manager.lock = threading.Lock()
        manager.visual_follow_feature_enabled = True
        manager.active = True
        manager.tracker = SimpleNamespace(active=True, reset=lambda: None)
        manager.selection_snapshot = SimpleNamespace(valid=True)
        manager.visual_follow_requested = False
        manager.visual_follow_state = "target_initialized"
        manager.visual_follow_error = ""
        manager.initial_safe_distance_horizontal_m = 10.0
        manager.metric_target_fusion = SimpleNamespace(
            operational=True,
            ready_for_safe_distance_lock=True,
        )
        manager.provisional_target_source = "midas_metric"
        manager.provisional_target_metric_confirmed = True
        manager.target_estimate = TargetEstimate(
            timestamp_s=2.0,
            state="METRIC_FUSION",
            valid=True,
            reason="ok",
            position_ned_m=(10.0, 0.0, 0.0),
            velocity_ned_m_s=(0.0, 0.0, 0.0),
            range_std_m=0.3,
        )
        manager.follow_workflow = FollowWorkflowMachine()
        manager.follow_workflow.transition(
            FollowWorkflowState.TARGET_INITIALIZED,
            "range_converged",
            force=True,
        )
        selected_bbox = [10.0, 20.0, 30.0, 40.0]
        manager.selected_bbox = selected_bbox
        manager.status = lambda: {"ok": True}

        status = manager.set_visual_follow_requested(True)

        self.assertEqual(status, {"ok": True})
        self.assertTrue(manager.visual_follow_requested)
        self.assertIs(manager.selected_bbox, selected_bbox)
        self.assertIsNotNone(manager.tracker)

    def test_follow_abort_enters_hold_without_resetting_bbox_tracker(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.follow_workflow = FollowWorkflowMachine()
        manager.follow_workflow.transition(
            FollowWorkflowState.FOLLOWING,
            "nav19_confirmed",
            force=True,
        )
        tracker = SimpleNamespace(active=True)
        selected_bbox = [10.0, 20.0, 30.0, 40.0]
        stopped = []
        manager.tracker = tracker
        manager.selected_bbox = selected_bbox
        manager.visual_follow_ready = True
        manager.visual_follow_error = ""
        manager._stop_bootstrap_motion_locked = stopped.append
        manager._stop_visual_follow_locked = stopped.append

        manager._abort_follow_workflow_locked(
            "manual_control_loss_timeout",
            5.0,
        )

        self.assertIs(manager.tracker, tracker)
        self.assertTrue(manager.tracker.active)
        self.assertIs(manager.selected_bbox, selected_bbox)
        self.assertEqual(
            manager.follow_workflow.state,
            FollowWorkflowState.HOLD,
        )
        self.assertEqual(stopped, ["manual_control_loss_timeout"] * 2)

    def test_authorize_measured_target_preserves_selected_bbox(self) -> None:
        manager = TrackingManager.__new__(TrackingManager)
        manager.visual_follow_requested = False
        manager.visual_follow_auto_start_pending = True
        manager.visual_follow_state = "acquiring_range"
        manager.visual_follow_error = "target_depth_invalid"
        manager.provisional_target_source = "midas_metric"
        manager.provisional_target_metric_confirmed = True
        manager.metric_target_fusion = SimpleNamespace(
            operational=True,
            ready_for_safe_distance_lock=True,
        )
        manager.initial_safe_distance_horizontal_m = None
        manager.initial_safe_distance_lock_count = 0
        manager.selection_snapshot = SelectionSnapshot(
            session_id=1,
            selection_timestamp_s=1.0,
            selection_frame_index=2,
            selection_vehicle_position_ned=(0.0, 0.0, -8.0),
            selection_camera_position_ned=(0.1, 0.0, -8.0),
            selection_vehicle_global_position=(10.0, 106.0, 18.0),
            selection_camera_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            selection_bearing_ned=(1.0, 0.0, 0.0),
            selection_bbox=(12.0, 24.0, 36.0, 48.0),
            selection_bbox_scale_px=(36.0 * 48.0) ** 0.5,
            selection_heading_rad=0.0,
            selection_altitude_m=8.0,
            selection_pose_age_ms=1.0,
            selection_gimbal_age_ms=1.0,
            valid=True,
            reason="ok",
        )
        manager.target_estimate = TargetEstimate(
            timestamp_s=2.0,
            state="VALID",
            valid=True,
            reason="ready",
            position_ned_m=(10.0, 0.0, -8.0),
            range_std_m=0.4,
        )
        manager.follow_workflow = FollowWorkflowMachine()
        manager.follow_workflow.transition(
            FollowWorkflowState.TARGET_INITIALIZED,
            "range_converged",
            force=True,
        )
        selected_bbox = [12.0, 24.0, 36.0, 48.0]
        manager.selected_bbox = selected_bbox

        manager._authorize_metric_follow_locked(
            "auto_authorized_metric_target",
            timestamp_s=3.0,
        )

        self.assertTrue(manager.visual_follow_requested)
        self.assertFalse(manager.visual_follow_auto_start_pending)
        self.assertEqual(manager.visual_follow_state, "target_initialized")
        self.assertEqual(manager.visual_follow_error, "")
        self.assertEqual(
            manager.follow_workflow.reason,
            "auto_authorized_metric_target",
        )
        self.assertIs(manager.selected_bbox, selected_bbox)


if __name__ == "__main__":
    unittest.main()
