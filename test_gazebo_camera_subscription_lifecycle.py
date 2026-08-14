from __future__ import annotations

import main


class FakeNode:
    def __init__(self) -> None:
        self.subscribed: list[str] = []
        self.unsubscribed: list[str] = []

    def subscribe(self, _message_type, topic, _callback):
        self.subscribed.append(topic)
        return True

    def unsubscribe(self, topic):
        self.unsubscribed.append(topic)
        return True


def bridge_with_camera(monkeypatch):
    monkeypatch.setenv("SWARM_GAZEBO_SUBSCRIPTION_PROFILE", "production")
    bridge = main.GazeboDashboardBridge()
    bridge.node = FakeNode()
    bridge.started = True
    bridge.camera_topics["UAV-01"] = "/camera/uav01"
    bridge.camera_callbacks["UAV-01"] = lambda _message: None
    return bridge


def test_tracking_subscribes_camera_on_demand_and_releases_it(monkeypatch):
    bridge = bridge_with_camera(monkeypatch)

    bridge.set_tracking_drone("UAV-01")
    assert bridge.camera_subscribed == {"UAV-01"}
    assert bridge.node.subscribed == ["/camera/uav01"]

    bridge.set_tracking_drone(None)
    assert bridge.camera_subscribed == set()
    assert bridge.node.unsubscribed == ["/camera/uav01"]


def test_preview_holds_subscription_until_generator_closes(monkeypatch):
    bridge = bridge_with_camera(monkeypatch)
    bridge.frames["UAV-01"] = b"jpeg"
    bridge.frame_versions["UAV-01"] = 1

    stream = bridge.mjpeg_frames("UAV-01")
    assert b"jpeg" in next(stream)
    assert bridge.camera_clients["UAV-01"] == 1
    assert bridge.camera_subscribed == {"UAV-01"}

    stream.close()
    assert bridge.camera_clients["UAV-01"] == 0
    assert bridge.camera_subscribed == set()


def test_preview_and_tracking_share_one_subscription(monkeypatch):
    bridge = bridge_with_camera(monkeypatch)
    bridge.frames["UAV-01"] = b"jpeg"
    bridge.frame_versions["UAV-01"] = 1
    bridge.set_tracking_drone("UAV-01")

    stream = bridge.mjpeg_frames("UAV-01")
    next(stream)
    stream.close()
    assert bridge.camera_subscribed == {"UAV-01"}
    assert bridge.node.unsubscribed == []

    bridge.set_tracking_drone(None)
    assert bridge.camera_subscribed == set()
    assert bridge.node.unsubscribed == ["/camera/uav01"]


def test_production_keeps_non_camera_subscriptions_enabled(monkeypatch):
    bridge = bridge_with_camera(monkeypatch)
    assert bridge._subscription_enabled("body_imu_uav01") is True
    assert bridge._subscription_enabled("camera_imu_uav01") is True
    assert bridge._subscription_enabled("front_lidar") is True


def test_zero_and_single_profiles_remain_isolated(monkeypatch):
    monkeypatch.setenv("SWARM_GAZEBO_SUBSCRIPTION_PROFILE", "zero")
    assert main.GazeboDashboardBridge()._subscription_enabled("front_lidar") is False
    monkeypatch.setenv("SWARM_GAZEBO_SUBSCRIPTION_PROFILE", "single")
    monkeypatch.setenv("SWARM_GAZEBO_SUBSCRIPTION_TOPICS", "front_lidar")
    bridge = main.GazeboDashboardBridge()
    assert bridge._subscription_enabled("front_lidar") is True
    assert bridge._subscription_enabled("body_imu_uav01") is False
