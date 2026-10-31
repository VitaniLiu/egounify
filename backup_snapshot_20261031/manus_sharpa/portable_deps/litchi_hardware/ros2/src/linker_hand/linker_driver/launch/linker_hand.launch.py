from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [
            get_package_share_directory("linker_driver"),
            "config",
            "o30i.yaml",
        ]
    )
    arguments = [
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("side", default_value="right"),
        DeclareLaunchArgument("namespace", default_value="linker/right"),
        DeclareLaunchArgument("command_topic", default_value="joint_command"),
        DeclareLaunchArgument("state_topic", default_value="joint_states"),
    ]
    node = Node(
        package="linker_driver",
        executable="linker_hand_node",
        name="linker_hand_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                "config_file": LaunchConfiguration("config_file"),
                "side": LaunchConfiguration("side"),
                "command_topic": LaunchConfiguration("command_topic"),
                "state_topic": LaunchConfiguration("state_topic"),
            }
        ],
    )
    return LaunchDescription([*arguments, node])
