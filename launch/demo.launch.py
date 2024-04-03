import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sam_test'),
        # 'config',
        'param.yaml'
    )
    return LaunchDescription([
        Node(
            package='polylidar_ros2',
            namespace ='lidar_3d_node',
            executable='lidar_3d_node',
            remappings=[('/input_cloud', '/velodyne_points')],
            # parameters=[config]
        ),
    ])