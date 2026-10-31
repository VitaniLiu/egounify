"""Launch file for the Xynova hand node.

Usage:
  ros2 launch xynova_hand xynova_hand.launch.py
  ros2 launch xynova_hand xynova_hand.launch.py dry_run:=true
  ros2 launch xynova_hand xynova_hand.launch.py config_file:=/abs/path/to/hand.yaml
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_cfg = PathJoinSubstitution(
        [
            get_package_share_directory("xynova_hand"),
            "config",
            "default_hand.yaml",
        ]
    )

    args = [
        DeclareLaunchArgument(
            "dry_run", default_value="false", description="If true, do not touch hardware."
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=default_cfg,
            description="YAML overriding parameters + joint calibration.",
        ),
        DeclareLaunchArgument(
            "command_topic",
            default_value="joint_command",
            description="Subscribed sensor_msgs/JointState topic.",
        ),
        DeclareLaunchArgument(
            "state_topic",
            default_value="joint_states",
            description="Published sensor_msgs/JointState topic.",
        ),
        DeclareLaunchArgument(
            "tactile_topic",
            default_value="tactile_forces",
            description="Published sensor_msgs/Image topic containing 15 six-axis wrenches.",
        ),
        DeclareLaunchArgument(
            "state_publish_hz", default_value="100.0", description="State publishing rate."
        ),
        DeclareLaunchArgument(
            "tactile_publish_hz",
            default_value="100.0",
            description="Tactile publishing rate.",
        ),
        DeclareLaunchArgument(
            "enable_tactile",
            default_value="false",
            description="Read and publish the 15 six-axis tactile wrenches.",
        ),
        DeclareLaunchArgument(
            "namespace", default_value="xynova_hand", description="ROS 2 node namespace."
        ),
    ]

    node_params = [
        {
            "dry_run": LaunchConfiguration("dry_run"),
            "config_file": LaunchConfiguration("config_file"),
            "command_topic": LaunchConfiguration("command_topic"),
            "state_topic": LaunchConfiguration("state_topic"),
            "tactile_topic": LaunchConfiguration("tactile_topic"),
            "state_publish_hz": LaunchConfiguration("state_publish_hz"),
            "tactile_publish_hz": LaunchConfiguration("tactile_publish_hz"),
            "enable_tactile": LaunchConfiguration("enable_tactile"),
        }
    ]

    node = Node(
        package="xynova_hand",
        executable="xynova_hand_node",
        name="xynova_hand_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=node_params,
        emulate_tty=True,
    )

    return LaunchDescription([*args, node])
