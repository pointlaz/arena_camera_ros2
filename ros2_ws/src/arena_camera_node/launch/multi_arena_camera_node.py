from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():
    share_path = get_package_share_directory("arena_camera_node")
    config_path = os.path.join(share_path, "config")
               
    return LaunchDescription([
        DeclareLaunchArgument('respawn', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('node_name', default_value='arena_camera_node'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(config_path, 'default.yaml')),
        DeclareLaunchArgument('device_user_id_0', default_value='camera_ToF_0'),
        DeclareLaunchArgument('device_user_id_1', default_value='camera_ToF_1'),
        DeclareLaunchArgument('camera_frame_0', default_value='arena_camera_0'),
        DeclareLaunchArgument('camera_frame_1', default_value='arena_camera_1'),
        DeclareLaunchArgument('launch_prefix', default_value='', condition=UnlessCondition(LaunchConfiguration('debug'))),
        DeclareLaunchArgument('launch_prefix', default_value='gdb -ex run --args', condition=IfCondition(LaunchConfiguration('debug'))),

        Node(
            package='arena_camera_node',
            executable='start',
            name=[LaunchConfiguration('node_name'), '_0'],
            output='screen',
            respawn=LaunchConfiguration('respawn'),
            parameters=[LaunchConfiguration('config_file'),
                        {
                            "device_user_id": LaunchConfiguration('device_user_id_0'),
                            "camera_frame": LaunchConfiguration('camera_frame_0'),
                        }, 
            ]
            # prefix=LaunchConfiguration('launch_prefix'),
        ),
        Node(
            package='arena_camera_node',
            executable='start',
            name=[LaunchConfiguration('node_name'), '_1'],
            output='screen',
            respawn=LaunchConfiguration('respawn'),
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    "device_user_id": LaunchConfiguration('device_user_id_1'),
                    "camera_frame": LaunchConfiguration('camera_frame_1'),
                },
            ]

            # prefix=LaunchConfiguration('launch_prefix'),
        ),
    ])