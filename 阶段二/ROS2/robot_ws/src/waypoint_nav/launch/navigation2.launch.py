"""起 Nav2（AMCL 定位 + 代价地图 + 规划/控制）和 RViz。

迁移自 chapt7 的 fishbot_navigation2/launch/navigation2.launch.py，
只改了包名和默认地图。用法（Gazebo 那边已经跑起来的前提下）：

    ros2 launch waypoint_nav navigation2.launch.py

换成别的图不用改代码，给参数就行：

    ros2 launch waypoint_nav navigation2.launch.py \\
        map:=/home/chen/code/FOCUS/阶段二/ROS2/robot_ws/src/recruit_robot_sim/maps/new1_map.yaml

跑完在 RViz 里用 "2D Pose Estimate" 点一下机器人的真实位置和朝向，
再用 "2D Goal Pose" 点目标；也可以另开一个终端跑路点节点：
    ros2 launch waypoint_nav waypoint_follower.launch.py
"""

import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    waypoint_nav_dir = get_package_share_directory('waypoint_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz',
                                   'nav2_default_view.rviz')

    # finals_map 中段没建到，必须配 allow_unknown: true 才规划得过去。
    # 同目录下还放了一份完整覆盖的 new1_map，换图不用改代码：
    #   map:=<share>/waypoint_nav/maps/new1_map.yaml
    default_map = os.path.join(waypoint_nav_dir, 'maps', 'finals_map.yaml')

    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    start_rviz = launch.substitutions.LaunchConfiguration(
        'rviz', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=default_map)
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file',
        default=os.path.join(waypoint_nav_dir, 'config', 'nav2_params.yaml'))

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument(
            'map', default_value=map_yaml_path,
            description='FULL path to map file to load'),
        launch.actions.DeclareLaunchArgument(
            'params_file', default_value=nav2_param_path,
            description='FULL path to Nav2 params file'),
        launch.actions.DeclareLaunchArgument(
            'rviz', default_value=start_rviz,
            description='Start RViz'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                nav2_bringup_dir, '/launch', '/bringup_launch.py'
            ]),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=launch.conditions.IfCondition(start_rviz),
        ),
    ])
