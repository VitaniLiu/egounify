from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _driver(*, side, enabled, serial, params_file):
    return Node(
        package="wuji_hand_driver",
        executable="wuji-hand-2-driver",
        name=f"lychee_{side}_wuji_hand_2_driver",
        output="screen",
        condition=IfCondition(enabled),
        parameters=[
            params_file,
            {"hand_side": side, "serial": serial},
        ],
    )


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("wuji_hand_driver"), "config", "wuji_hand_2.yaml"]
    )
    params_file = LaunchConfiguration("params_file")
    enable_left = LaunchConfiguration("enable_left")
    enable_right = LaunchConfiguration("enable_right")
    left_serial = LaunchConfiguration("left_serial")
    right_serial = LaunchConfiguration("right_serial")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Wuji Hand 2 driver ROS parameter file.",
            ),
            DeclareLaunchArgument(
                "enable_left",
                default_value="false",
                description="Connect and start the left Wuji Hand 2.",
            ),
            DeclareLaunchArgument(
                "enable_right",
                default_value="true",
                description="Connect and start the right Wuji Hand 2.",
            ),
            DeclareLaunchArgument(
                "left_serial",
                default_value="",
                description="Left hand serial; empty discovers by handedness.",
            ),
            DeclareLaunchArgument(
                "right_serial",
                default_value="",
                description="Right hand serial; empty discovers by handedness.",
            ),
            _driver(
                side="left",
                enabled=enable_left,
                serial=left_serial,
                params_file=params_file,
            ),
            _driver(
                side="right",
                enabled=enable_right,
                serial=right_serial,
                params_file=params_file,
            ),
        ]
    )
