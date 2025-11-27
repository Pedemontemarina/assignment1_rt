# assignment1_rt

This package is part of the *Research Track I* course assignment.  
It contains three ROS2 nodes that implement:

- The spawn of a second turtle called **turtle2**
- A simple **text-based user interface** to control one of the turtles (`turtle1` or `turtle2`)
- A **distance monitoring node** that checks the relative position between the turtles and applies safety constraints

---

## 1. Package Overview

The package `assignment1_rt` contains:

- `node1`: **turtle_spawn** 
  Spawns a second turtle, was provided together with the requests of the assignment

- `node2`: **UI_node**  
  Allows the user to choose which turtle to move and at what velocity.  
  Sends the command for **1 second**, then stops the robot.

- `node3`: **Distance_node**  
  Computes and publishes the distance between `turtle1` and `turtle2`.  
  Stops the moving turtle if:
  - The turtles get **too close** (configurable threshold)  
  - The turtle approaches the **world boundaries** (x or y > 10.0, x or y < 1.0)

---

## 2. Directory Structure

ASSIGNMENT1_RT
.
├── CMakeLists.txt
├── README.md
├── include
│   └── assignment1_rt
├── launch
│   └── assignment_launch.py
├── package.xml
├── scripts
│   ├── distance.py
│   └── turtle_spawn.py
├── src
│   └── user_interface.cpp
└── start_all.sh

## 3. Installation

Clone this repository into your ROS2 workspace:

```bash
cd ~/ros_ws/src
git clone <repository-url> assignment1_rt
cd ..
colcon build 
```
Source your workspace:
```bash
source install/setup.bash
```

## 4. Usage

Start the entire system (turtlesim + nodes) with the provided bash script:

```bash 
./start_all.sh
```
This script calls the ROS2 launch file: assignment_launch.py

## 5. Nodes Description

1) UI Node (node1) 

Written in C++, it uses rclcpp and geometry_msgs/msg/twist.hpp. It implements a TurtleController class that publishes velocity commands on the turtle1/cmd_vel and turtle2/cmd_vel topics.

The node reads user commands in a continuous loop, validating both the turtle name and the velocity values. Each command is published for 1 second, after which the node sends a stop command (linear and angular velocities set to zero).

The node uses private publishers and handles input errors, allowing the user to enter new commands indefinitely.

2) Distance Node (node2)

Written in Python, it subscribes to both turtles’ positions to compute the Euclidean distance between them. Then it publishes it as a std_msgs/Float32 message on the topic /turtles_distance.

A periodic timer is used to check both the distance and apply safety rules such as:

- Stopping the moving turtle if the turtles are getting closer than a threshold.

- Stopping the moving turtle if it approaches the map boundaries.

To achieve this, a function determines which turtle is currently moving by checking subscriptions to their velocity topics.

When a turtle is stopped due to safety constraints, the node can also command it to move backward at a constant speed, untill it is in safe position.

The node makes extensive use of logging to report the current distance (if it has changed from the previous one) and any warnings when safety rules are triggered.


## 6. Launch File

The provided launch file starts and configures all the necessary nodes for the assignment:

1) turtlesim_node – Starts the main Turtlesim simulator.

2) turtle_spawn (Python) – Spawns the second turtle if it is not already present.

3) UI Node (C++) – Launches the user interface node for controlling turtle velocity.

4) Distance Node (Python) – Launches the node that monitors the turtles’ positions, computes the distance, and enforces safety rules.

Nodes can be run also individually using the following command (eachone on a different terminal)
```bash
ros2 run assignment1_rt <node_name>
```

## 7. Requirements

ROS2 Humble / Foxy / Galactic

turtlesim package installed

## 8. Author

Pedemonte Marina
Research Track I – Assignment 1

## 9. Notes

Thresholds and velocity limits can be adjusted in the code.

The package is designed for execution inside a ROS2 workspace.