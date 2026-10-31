"""Safe no-hardware entry point for the complete teleoperation pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    launch_file = os.path.join(
        get_package_share_directory("manus_sharpa_teleop"),
        "launch",
        "teleop.launch.py",
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    "use_fake_hardware": "true",
                    "manus_client_executable": "",
                    "retarget_backend": "analytic",
                }.items(),
            )
        ]
    )
