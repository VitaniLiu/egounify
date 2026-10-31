from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    enable_left = LaunchConfiguration("enable_left")
    enable_right = LaunchConfiguration("enable_right")
    default_wuji_params = PathJoinSubstitution(
        [FindPackageShare("wuji_hand_driver"), "config", "wuji_hand_2.yaml"]
    )

    arguments = [
        DeclareLaunchArgument("enable_left", default_value="true"),
        DeclareLaunchArgument("enable_right", default_value="true"),
        DeclareLaunchArgument(
            "manus_backend",
            default_value="native",
            description="MANUS backend: native or zmq.",
        ),
        DeclareLaunchArgument(
            "manus_endpoint", default_value="tcp://127.0.0.1:2044"
        ),
        DeclareLaunchArgument("manus_manage_client", default_value="true"),
        DeclareLaunchArgument(
            "manus_client_executable",
            default_value=EnvironmentVariable(
                "MANUS_CLIENT_EXECUTABLE", default_value=""
            ),
        ),
        DeclareLaunchArgument(
            "manus_client_working_directory",
            default_value=EnvironmentVariable(
                "MANUS_CLIENT_WORKING_DIRECTORY", default_value=""
            ),
        ),
        DeclareLaunchArgument("manus_native_library", default_value=""),
        DeclareLaunchArgument("manus_calibration_directory", default_value=""),
        DeclareLaunchArgument("manus_native_host_discovery_s", default_value="1"),
        DeclareLaunchArgument("manus_native_loopback_only", default_value="false"),
        DeclareLaunchArgument("manus_publish_rate_hz", default_value="60.0"),
        DeclareLaunchArgument("max_input_age_s", default_value="0.25"),
        DeclareLaunchArgument("smoothing_alpha", default_value="0.35"),
        DeclareLaunchArgument(
            "max_joint_velocity_rad_s", default_value="12.0"
        ),
        DeclareLaunchArgument(
            "pinky_max_joint_velocity_rad_s", default_value="5.0"
        ),
        DeclareLaunchArgument("max_filter_dt_s", default_value="0.05"),
        DeclareLaunchArgument("left_wuji_serial", default_value=""),
        DeclareLaunchArgument("right_wuji_serial", default_value=""),
        DeclareLaunchArgument(
            "wuji_params_file", default_value=default_wuji_params
        ),
    ]

    manus = Node(
        package="manus_driver",
        executable="manus_driver",
        name="manus_driver",
        output="screen",
        parameters=[
            {
                "use_fake_hardware": False,
                "backend": LaunchConfiguration("manus_backend"),
                "endpoint": LaunchConfiguration("manus_endpoint"),
                "manage_client": LaunchConfiguration("manus_manage_client"),
                "client_executable": LaunchConfiguration(
                    "manus_client_executable"
                ),
                "client_working_directory": LaunchConfiguration(
                    "manus_client_working_directory"
                ),
                "native_library": LaunchConfiguration("manus_native_library"),
                "native_host_discovery_s": LaunchConfiguration(
                    "manus_native_host_discovery_s"
                ),
                "native_loopback_only": LaunchConfiguration(
                    "manus_native_loopback_only"
                ),
                "calibration_directory": LaunchConfiguration(
                    "manus_calibration_directory"
                ),
                "publish_rate_hz": LaunchConfiguration(
                    "manus_publish_rate_hz"
                ),
                "publish_left": enable_left,
                "publish_right": enable_right,
            }
        ],
    )

    retarget = Node(
        package="manus_wuji_teleop",
        executable="retarget_node",
        name="manus_wuji_retarget",
        output="screen",
        parameters=[
            {
                "enable_left": enable_left,
                "enable_right": enable_right,
                "max_input_age_s": LaunchConfiguration("max_input_age_s"),
                "smoothing_alpha": LaunchConfiguration("smoothing_alpha"),
                "max_joint_velocity_rad_s": LaunchConfiguration(
                    "max_joint_velocity_rad_s"
                ),
                "pinky_max_joint_velocity_rad_s": LaunchConfiguration(
                    "pinky_max_joint_velocity_rad_s"
                ),
                "nominal_input_rate_hz": LaunchConfiguration(
                    "manus_publish_rate_hz"
                ),
                "max_filter_dt_s": LaunchConfiguration("max_filter_dt_s"),
            }
        ],
    )

    wuji = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("wuji_hand_driver"),
                    "launch",
                    "wuji_hand_2.launch.py",
                ]
            )
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("wuji_params_file"),
            "enable_left": enable_left,
            "enable_right": enable_right,
            "left_serial": LaunchConfiguration("left_wuji_serial"),
            "right_serial": LaunchConfiguration("right_wuji_serial"),
        }.items(),
    )

    return LaunchDescription([*arguments, manus, retarget, wuji])
