# Student code area

Put your own ROS 2 Python packages or scripts here only if your team chooses to organize them this way. This starter intentionally contains no executable solution for autonomous exploration or colour following.

Suggested files to create yourself:

- `exploration.py`: subscribe to `/scan` (and optionally `/odom`) and publish `Twist` messages to `/cmd_vel`.
- `color_follow.py`: subscribe to `/camera/image_raw`, use `cv_bridge` and OpenCV HSV segmentation, and publish `Twist` messages to `/cmd_vel`.

Make each program stop the robot by publishing a zero `Twist` when it exits.
