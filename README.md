# ROS2 Workspace for Autonomous Vehicle Practical Course

This repository contains the ROS2 workspace (ros2_ws) developed as part of the Autonomous Vehicle Practical Course at Tallinn University of Technology. The workspace includes custom packages and scripts designed for autonomous navigation, obstacle avoidance, and mapping using the TurtleBot3 robot in a simulated Gazebo environment.

## Run Locally

Clone the project

```bash
	git clone https://github.com/kuuiii/av_course.git
```

Navigate to the ROS2 workspace and build the package:

```bash
	cd ros2_ws
	colcon build --symlink-install
	source ~/.bashrc
```

Launch the Simulation World

```bash
	echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
	source ~/.bashrc
```

To launch the custom mapping node along with the simulation environment:

```bash
	ros2 launch my_robot_controler mapping_launch.py
```

