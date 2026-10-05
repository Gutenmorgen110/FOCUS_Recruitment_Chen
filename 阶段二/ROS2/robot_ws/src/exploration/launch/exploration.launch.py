"""启动自主探索节点。

用法（仿真已在另一个终端跑起来的前提下）：
    ros2 launch exploration exploration.launch.py

日志默认输出到“启动时所在目录/records”下，可用 log_dir 覆盖：
    ros2 launch exploration exploration.launch.py log_dir:=/tmp/exp_logs

打桩测试（没有 Gazebo、没有 /clock）：
    ros2 launch exploration exploration.launch.py use_sim_time:=false
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 日志目录：默认 <cwd>/records，启动时创建
    default_log_dir = os.path.join(os.getcwd(), 'records')
    os.makedirs(default_log_dir, exist_ok=True)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    log_dir      = LaunchConfiguration('log_dir',      default=default_log_dir)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='用 Gazebo 的仿真时钟（打桩测试时设为 false）'),
        DeclareLaunchArgument(
            'log_dir', default_value=log_dir,
            description='日志输出目录（默认为当前目录下的 records/）'),

        # 关键：让 ros2 把日志写到我们指定的目录
        SetEnvironmentVariable('ROS_LOG_DIR', log_dir),

        Node(
            package='exploration',
            executable='exploration',
            name='exploration',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])