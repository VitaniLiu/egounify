from launch import LaunchDescription
from launch_ros.actions import Node
import os
import pwd

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    username = os.environ.get('SUDO_USER') or os.environ.get('USER') or os.environ.get('LOGNAME') or pwd.getpwuid(os.getuid()).pw_name
    storage_root = os.environ.get('BAG_STORAGE_ROOT', os.path.join('/media', username, 'BAG_STORAGE'))
    record_storage_path = os.path.join(storage_root, 'recorded_bags')
    playback_path = os.path.join(storage_root, 'recorded_bags')
    recording_playback_config = os.path.join(
        get_package_share_directory('recording_playback_nodes_py'),
        'config',
        'recording_playback.yaml',
    )
    return LaunchDescription([
        Node(
            package='robot_setting',
            executable='robot_mode_service.py',
            name='robot_mode_service',
            output='screen',
            parameters=[recording_playback_config, {
                'bag_directory': playback_path,
            }]
        ),
        Node(
            package='recording_playback_nodes_py',
            executable='rd_playback_node',
            name='playback_node',
            output='screen',
            parameters=[recording_playback_config],
        ),
        Node(
            package='recording_playback_nodes_py',
            executable='data_bag_recorder',
            name='data_bag_recorder',
            parameters=[recording_playback_config, {
                'record_bags_storage_dir': record_storage_path,
            }],
            arguments=[
                "--headless"
            ]
        ),
    ])
