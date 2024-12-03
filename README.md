## ENPM702 RWA5
This project controls a TurtleBot using ROS2 by systematically moving it through a series of waypoints

## Table of Contents

- [Installation](#installation)
- [Run the nodes](#run-the-nodes)

## Installation
To install this project, you need to build and source two packages: "bot_waypoint_msgs" and "group2_rwa5". 
Run the following commands in the terminal:

Enter the workspace in which the packages are downloaded:
```bash
cd /path_to_workspace/
```

Build the waypoint message package which contains the IDL for defining the interface "BotWaypoint"
```bash
colcon build --packages-select bot_waypoint_msgs
```

Build the main package which contains the nodes for publishing the waypoint and reaching it
```bash
colcon build --packages-select group2_rwa5
```

Source the workspace
```bash
source install/setup.bash
```

## Run the nodes
First open a new terminal and source the underlay and launch the Turtlebot with Gazebo in an empty world:
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Then run the waypoint_publisher and waypoint_reacher nodes from a different terminal (make sure to build the package and source the overlay first)
``` bash
ros2 run group2_rwa5 group2_rwa5
```