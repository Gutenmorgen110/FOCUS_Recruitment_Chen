"""启动自主探索节点。

用法（仿真已在另一个终端跑起来的前提下）：
    ros2 launch exploration exploration.launch.py

打桩测试（没有 Gazebo、没有 /clock）时把 use_sim_time 关掉：
    ros2 launch exploration exploration.launch.py use_sim_time:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('exploration')
    default_params = os.path.join(pkg_share, 'config', 'exploration.yaml')

    # 用 LaunchConfiguration 做延迟求值，这样上层 launch 可以替换这两个值
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=default_params)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='用 Gazebo 的仿真时钟（打桩测试时设为 false）'),
        DeclareLaunchArgument(
            'params_file', default_value=params_file,
            description='探索节点的参数文件绝对路径'),

        Node(
            package='exploration',
            executable='exploration',
            name='exploration',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
