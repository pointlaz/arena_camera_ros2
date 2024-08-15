import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    share_path = get_package_share_directory("arena_camera_node")
    config_path = os.path.join(share_path, "config")

    return LaunchDescription([
        DeclareLaunchArgument(
            'respawn',
            default_value='false',
            description='Whether to respawn the node if it crashes'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Whether to run the node in debug mode'
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='arena_camera_node',
            description='Name of the node'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(config_path, 'default.yaml'),
            description='Path to the config file'
        ),
        DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='Whether to open RViz'
        ),
        Node(
            package='arena_camera_node',
            executable='start',
            name=LaunchConfiguration('node_name'),
            parameters=[LaunchConfiguration('config_file')],
            output='screen'
        ),
    ])

