"""Launch the Gazebo world, robot, sensors, TF, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('recruit_robot_sim')
    gazebo_share = get_package_share_directory('gazebo_ros')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    start_rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen',
    )
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': os.path.join(pkg_share, 'worlds', 'my_worlds.world')}.items(),
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )
    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'recruit_robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.02', '-timeout', '120'],
        output='screen',
    )
    rviz = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}], output='screen',
        condition=IfCondition(start_rviz),
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo graphical client'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg_share, 'rviz', 'basic.rviz')),
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        TimerAction(period=5.0, actions=[spawn_robot]),
        TimerAction(period=7.0, actions=[rviz]),
    ])
