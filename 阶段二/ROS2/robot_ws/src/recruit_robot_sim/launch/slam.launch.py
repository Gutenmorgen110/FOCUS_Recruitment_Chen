"""Launch Gazebo, the robot, SLAM Toolbox, and the SLAM RViz view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('recruit_robot_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    start_rviz = LaunchConfiguration('rviz')
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'rviz': start_rviz,
            'rviz_config': os.path.join(pkg_share, 'rviz', 'slam.rviz'),
        }.items(),
    )
    slam_toolbox = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        parameters=[os.path.join(pkg_share, 'config', 'slam_params.yaml'),
                    {'use_sim_time': use_sim_time}],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        bringup,
        TimerAction(period=15.0, actions=[slam_toolbox]),
    ])
