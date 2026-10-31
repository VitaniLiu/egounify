from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    replay_config = os.path.join(
        get_package_share_directory('gento_replay'),
        'config',
        'gento_replay.yaml',
    )
    input_topic = LaunchConfiguration('input_topic')
    mink_request_topic = LaunchConfiguration('mink_request_topic')
    playback_bag = LaunchConfiguration('playback_bag')
    ui_port = LaunchConfiguration('ui_port')

    # Single source of truth for the namespace: APEX_ROS_NAMESPACE (written to
    # /etc/apex/apex.env), same default 'tj' as every other service launch.
    # gento_replay is included by apex_backend/teleop_replay.launch.py, which
    # does NOT push a namespace, so this launch must push it once itself —
    # otherwise gento_replay's control/ik_request/replay lands at root and never
    # matches the ik_request_mux's /tj/control/ik_request/replay.
    ros_namespace = LaunchConfiguration('apex_ros_namespace')

    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='control/ik_request/vr'),
        DeclareLaunchArgument('mink_request_topic', default_value='control/ik_request/replay'),
        DeclareLaunchArgument('playback_bag', default_value=''),
        DeclareLaunchArgument('ui_port', default_value='8012'),
        DeclareLaunchArgument(
            'apex_ros_namespace',
            default_value=os.environ.get('APEX_ROS_NAMESPACE', 'tj').strip().strip('/'),
            description='ROS namespace for the Apex robot graph',
        ),
        GroupAction(actions=[
            PushRosNamespace(ros_namespace),
            Node(
                package='gento_replay',
                executable='gento_replay_node',
                name='gento_replay_node',
                output='screen',
                parameters=[
                    replay_config,
                    {
                        'input_topic': input_topic,
                        'mink_request_topic': mink_request_topic,
                        'joint_cmd_mux_service': 'control/ik_request_mux/select',
                        'idle_joint_cmd_source': 1,
                        'replay_joint_cmd_source': 3,
                        'playback_bag': playback_bag,
                        'ros_namespace': ros_namespace,
                    },
                ],
            ),
            Node(
                package='gento_replay',
                executable='gento_replay_ui_node',
                name='gento_replay_ui_node',
                output='screen',
                parameters=[{
                    'ui_port': ParameterValue(ui_port, value_type=int),
                    'service_timeout': 30.0,
                }],
            ),
        ]),
    ])
