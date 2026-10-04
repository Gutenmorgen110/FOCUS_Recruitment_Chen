from setuptools import find_packages, setup

package_name = 'waypoint_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch',
            ['launch/navigation2.launch.py',
             'launch/waypoint_follower.launch.py']),
        ('share/' + package_name + '/config',
            ['config/nav2_params.yaml',
             'config/waypoint_config.yaml']),
        # 图放这里自带一份。recruit_robot_sim 的 CMakeLists 没装 maps/ 目录，
        # 直接从那边引用会指向不存在的路径。
        ('share/' + package_name + '/maps',
            ['maps/finals_map.yaml', 'maps/finals_map.pgm',
             'maps/new1_map.yaml', 'maps/new1_map.pgm']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chen',
    maintainer_email='16491766+yingle_chen@user.noreply.gitee.com',
    description='Nav2 路点导航',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_follower = waypoint_nav.waypoint_follower:main',
        ],
    },
)
