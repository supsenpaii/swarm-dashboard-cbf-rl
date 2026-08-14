#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


UAVS = [
    {
        "suffix": "uav_01",
        "drone_id": "UAV-01",
        "px4_namespace": "/px4_0",
        "ros_topic": "/swarm/uav_01/telemetry_json",
        "mqtt_topic": "swarm/UAV-01/telemetry/state",
    },
    {
        "suffix": "uav_02",
        "drone_id": "UAV-02",
        "px4_namespace": "/px4_1",
        "ros_topic": "/swarm/uav_02/telemetry_json",
        "mqtt_topic": "swarm/UAV-02/telemetry/state",
    },
]

UAV_01_CONTROL_REMAPPINGS = [
    (
        f"/fmu/in/{topic_name}",
        f"/px4_0/fmu/in/{topic_name}",
    )
    for topic_name in (
        "manual_control_input",
        "offboard_control_mode",
        "trajectory_setpoint",
        "vehicle_command",
    )
] + [
    (
        f"/fmu/out/{topic_name}",
        f"/px4_0/fmu/out/{topic_name}",
    )
    for topic_name in (
        "vehicle_local_position",
        "vehicle_status",
        "vehicle_command_ack",
    )
]


def generate_launch_description() -> LaunchDescription:
    actions = []

    for uav in UAVS:
        actions.append(
            Node(
                package="swarm_telemetry",
                executable="telemetry_node",
                name=f"telemetry_{uav['suffix']}",
                output="screen",
                parameters=[
                    {
                        "drone_id": uav["drone_id"],
                        "px4_namespace": uav["px4_namespace"],
                        "output_topic": uav["ros_topic"],
                    }
                ],
            )
        )
        actions.append(
            Node(
                package="swarm_telemetry",
                executable="mqtt_bridge",
                name=f"mqtt_bridge_{uav['suffix']}",
                output="screen",
                parameters=[
                    {
                        "drone_id": uav["drone_id"],
                        "broker_host": "127.0.0.1",
                        "broker_port": 1883,
                        "ros_topic": uav["ros_topic"],
                        "mqtt_topic": uav["mqtt_topic"],
                    }
                ],
            )
        )

    actions.append(
        Node(
            package="swarm_telemetry",
            executable="web_control_node",
            name="web_control_node",
            output="screen",
            remappings=UAV_01_CONTROL_REMAPPINGS,
        )
    )

    return LaunchDescription(actions)
