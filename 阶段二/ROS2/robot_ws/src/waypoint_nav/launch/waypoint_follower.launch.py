"""单独起路点导航节点（Nav2 已在另一个终端跑起来的前提下）：

    ros2 launch waypoint_nav waypoint_follower.launch.py

换一套路点：
    ros2 launch waypoint_nav waypoint_follower.launch.py \\
        points_file:=/path/to/my_points.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_points = os.path.join(
        get_package_share_directory('waypoint_nav'),
        'config', 'waypoint_config.yaml')

    points_file = LaunchConfiguration('points_file', default=default_points)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'points_file', default_value=points_file,
            description='路点表的参数文件绝对路径'),
        DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='用 Gazebo 的仿真时钟'),

        Node(
            package='waypoint_nav',
            executable='waypoint_follower',
            # 不能叫 waypoint_follower：Nav2 自己也起了个同名节点，会串参数
            name='waypoint_navigator',
            output='screen',
            parameters=[points_file, {'use_sim_time': use_sim_time}],
        ),
    ])
