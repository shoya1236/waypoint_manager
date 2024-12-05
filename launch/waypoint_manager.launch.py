from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set the paths to the waypoint CSV and the configuration file
    waypoints_csv_path = '/path/to/your/waypoints.csv'
    config_file_path = os.path.join(
        get_package_share_directory('waypoint_manager'),
        'config',
        'manager_config.yaml'
    )

    # Path to the voice_manager launch file
    voice_manager_launch_path = os.path.join(
        get_package_share_directory('voice_manager'),
        'launch',
        'voice_manager.launch.py'
    )

    # Declare the argument for enabling voice_manager
    voice_manager_arg = DeclareLaunchArgument(
        'enable_voice_manager',
        default_value='false',
        description='Whether to launch the voice_manager package'
    )

    return LaunchDescription([
        # Argument declaration
        voice_manager_arg,

        # Waypoint manager node
        Node(
            package='waypoint_manager',
            executable='waypoint_manager',
            name='waypoint_manager_node',
            output='screen',
            parameters=[config_file_path, {
                'waypoints_csv': waypoints_csv_path,
            }]
        ),

        # Waypoint visualizer node
        Node(
            package='waypoint_manager',
            executable='waypoint_visualizer',
            name='waypoint_visualizer_node',
            output='screen',
            parameters=[config_file_path, {
                'waypoints_csv': waypoints_csv_path,
            }]
        ),

        # Waypoint skipper node
        Node(
            package='waypoint_manager',
            executable='waypoint_skipper',
            name='waypoint_skipper_node',
            output='screen',
            remappings=[('/current_pose', '/current_pose'),
                        ('/scan', '/scan')],
            parameters=[config_file_path, {
                'waypoints_csv': waypoints_csv_path,
            }]
        ),

        # Conditional launch of voice_manager
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(voice_manager_launch_path),
                condition=IfCondition(LaunchConfiguration('enable_voice_manager'))
            )
        ]),
    ])
