# Robot Recruitment Starter Project

This is a ROS 2 Humble and Gazebo Classic starter project for first- and second-year students. It supplies a working differential-drive robot, LiDAR, RGB camera, TF, odometry, RViz, and SLAM Toolbox. It deliberately does not contain autonomous exploration or colour-following solutions.

## Environment and dependencies

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11 and `gazebo_ros_pkgs`
- `slam_toolbox`
- `teleop_twist_keyboard`
- OpenCV and `cv_bridge`

On a computer with ROS 2 Humble installed, install the runtime packages:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-slam-toolbox \\
  ros-humble-teleop-twist-keyboard ros-humble-rqt-image-view \\
  ros-humble-cv-bridge python3-opencv
```

## Build

From the workspace root (the directory containing `recruit_robot_sim`):

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Start the simulation

Terminal 1 starts Gazebo, the robot, LiDAR, RGB camera, odometry, TF, and RViz:

```bash
source install/setup.bash
ros2 launch recruit_robot_sim bringup.launch.py
```

Terminal 1 can instead start SLAM Toolbox as well. RViz then shows the map while the robot moves:

```bash
source install/setup.bash
ros2 launch recruit_robot_sim slam.launch.py
```

In terminal 2, use the keyboard to drive. The robot remains stationary until a publisher sends commands to `/cmd_vel`.

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive slowly around the complete arena to build a map. Save a completed map with a path of your choice:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \\
  "{name: {data: '/tmp/my_map'}}"
```

## Important topics and TF

| Topic | Meaning |
| --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` velocity command for the differential drive |
| `/scan` | 360-degree `sensor_msgs/msg/LaserScan` from the LiDAR |
| `/odom` | `nav_msgs/msg/Odometry` from the drive plugin |
| `/camera/image_raw` | RGB `sensor_msgs/msg/Image` from the Gazebo camera |
| `/map` | `nav_msgs/msg/OccupancyGrid` published by SLAM Toolbox after `slam.launch.py` starts |

The main transform chain is:

```text
map
 ↓
odom
 ↓
base_footprint
 ↓
base_link
 ├── lidar_link
 └── camera_link
      ↓
      camera_optical_frame
```

`map → odom` is supplied by SLAM Toolbox only when SLAM is running. The other transforms come from the robot model and Gazebo drive plugin.

## Check the camera

Start the base simulation, then run either command in another terminal:

```bash
ros2 topic list
ros2 run rqt_image_view rqt_image_view
```

Choose `/camera/image_raw` in rqt_image_view. Gazebo also contains distinct red, green, and blue obstacles so that the camera has visible colour targets.

## Recruitment tasks

1. **Run the base project.** Use keyboard teleoperation and inspect LiDAR, odometry, TF, and camera data.
2. **Design your own robot and environment.** Change the provided Xacro and World, or create/import your own. The final submission must not use the original robot and World unchanged.
3. **Manual SLAM.** Use SLAM Toolbox and keyboard control to map your own environment.
4. **SLAM tuning.** After successfully running SLAM, tune the parameters in config/slam_params.yaml according to your robot and environment. You may adjust map resolution, LiDAR range, scan matching, map update thresholds, loop closure, or other relevant parameters. Compare the maps before and after tuning, and briefly explain the parameters you changed and how they affected mapping quality.
5. **Autonomous SLAM.** Write your own `exploration.py` node. Subscribe to `/scan` and publish `geometry_msgs/msg/Twist` to `/cmd_vel`. A simple solution can compare front, left, and right ranges, stop or reverse near obstacles, and choose a turn direction. You may add multi-region sensing, recovery after getting stuck, random turns, speed control, odometry-based stuck detection, visited-area records, or a completion condition.
6. **OpenCV colour following.** Write your own `color_follow.py`. Subscribe to `/camera/image_raw`, convert it with `cv_bridge`, use HSV thresholding and contours to find a red, green, or blue target, then publish `/cmd_vel`. Steer from the image-centre error; slow down or stop when the target occupies a large image area; stop or search slowly when it disappears.
7. **Nav2 autonomous navigation (Bonus).** Save the map created in Task 3 or Task 5 as `.pgm` and `.yaml` files, then configure Nav2 for your own robot and environment. Use AMCL or SLAM Toolbox localization, set the robot's initial pose in RViz, and send navigation goals with **Nav2 Goal** or **2D Goal Pose**.

   Your Nav2 configuration should match your robot rather than directly using all provided default parameters. At minimum, check or tune:

   - robot footprint or robot radius;
   - maximum linear and angular velocities;
   - acceleration and deceleration limits;
   - global and local costmap size and resolution;
   - obstacle and inflation layers;
   - inflation radius and cost scaling factor;
   - planner and controller parameters;
   - goal tolerances and recovery behaviours.

   Demonstrate that the robot can autonomously reach at least three goal poses in the saved map without manually publishing velocity commands. The route should include at least one turn and one obstacle or narrow passage. Record the planned path, local costmap, robot trajectory, and final navigation result in RViz.

   Briefly explain the Nav2 data flow from the goal pose to `/cmd_vel`, the difference between the global planner and local controller, and the purpose of the global and local costmaps.

The starter contains only the sensor and motion interfaces. It does not include `exploration.py`, `color_follow.py`, path planning, autonomous navigation, or a ready-made exploration strategy.

## Package layout

```text
recruit_robot_sim/
├── config/slam_params.yaml
├── launch/bringup.launch.py
├── launch/slam.launch.py
├── rviz/basic.rviz
├── rviz/slam.rviz
├── scripts/README.md
├── urdf/robot.urdf.xacro
└── worlds/example.world
```
