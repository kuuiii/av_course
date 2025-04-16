ROS2 Workspace for Autonomous Vehicle Practical Course

Overview

This repository contains the ROS2 workspace (ros2_ws) developed as part of the Autonomous Vehicle Practical Course at Tallinn University of Technology. The workspace includes custom packages and scripts designed for autonomous navigation, obstacle avoidance, and mapping using the TurtleBot3 robot in a simulated Gazebo environment.

Repository Structure

ros2_ws/
├── src/
│   └── my_robot_controller/
│       ├── launch/            # Launch files for nodes
│       ├── my_robot_controller/           # Python scripts (ROS2 nodes)
│       ├── map/               # Generated map files
│       └── package.xml        # ROS2 package metadata and dependencies
├── README.md                  # This README file
└── [other directories or files]

Setup and Installation

Prerequisites

ROS2 (Foxy/Humble or later)

Gazebo Simulator

TurtleBot3 packages

Installation Steps

Clone this repository:

git clone github.com/kuuiii/av_course

Navigate to your ROS2 workspace and build the package:

cd ros2_ws
colcon build --symlink-install

Source the workspace:

source install/setup.bash

Running the Mapping Node

To launch the custom mapping node along with the simulation environment:

ros2 launch my_robot_controler mapping_launch.py

This command will:

Start Gazebo with the custom world.

Launch the TurtleBot3 model.

Activate the autonomous mapping node.

Generated Maps

Generated occupancy grid maps are stored in the map/ directory within the my_robot_controler package.

Documentation and Reporting

Detailed documentation, including the process description, images, and conclusions, can be found in the project documentation submitted separately as part of the Autonomous Vehicle Practical Course.
