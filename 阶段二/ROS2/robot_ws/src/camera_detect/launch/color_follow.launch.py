"""启动颜色跟随节点。仿真已经在另一个终端跑起来的前提下：

    ros2 launch camera_detect color_follow.launch.py

要换一组阈值就复制一份 yaml 改，再：

    ros2 launch camera_detect color_follow.launch.py params_file:=/path/to/my.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('camera_detect'),
        'config', 'color_follow.yaml')

    params_file = LaunchConfiguration('params_file', default=default_params)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=params_file,
            description='颜色跟随节点的参数文件绝对路径'),
        DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='用 Gazebo 的仿真时钟（打桩测试时设为 false）'),

        Node(
            package='camera_detect',
            executable='color_follow',
            name='color_follow',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
