from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    waypoint_saver_config = os.path.join(
        get_package_share_directory('waypoint_manager'),
        'config',
        'maker_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='waypoint_manager',
            executable='waypoint_maker',
            name='waypoint_maker_node',
            output='screen',
            remappings=[('/current_pose', '/initialpose')],
            parameters=[waypoint_saver_config]
        ),
    ])