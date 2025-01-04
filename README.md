# Research Track I - Second Assignment Part I
This repository contains the assignment work for the **Research Track I** course, completed by:  
**Rubin Khadka Chhetri**  
**ID: 6558048**

## Table of Contents
- [Introduction](#introduction)
- [Node and Launch File Details](#node-and-launch-file-details)
    - [Action Client Node](#action-client-node)
    - [Service Node](#service-node)
    - [Launch File](#launch-file-coordinate-control)
- [Repository Structure](#repository-structure)
- [Getting Started (Read Before Action)](#getting-started-read-before-action)
    - [Prerequisites](#prerequisites)
    - [Setup](#setup)
- [Launching Nodes](#launching-nodes)
- [Implementation Details](#implementation-details)
- [Summary](#summary)

## Introduction

This project implements a ROS-based system for controlling a robot in a Gazebo simulation. It demonstrates the use of Action Clients and Service Nodes for setting target coordinates and retrieving them dynamically. The simulation enables interaction through user inputs and showcases communication between nodes using ROS.

This repository includes a ROS package with the following components:
 - **Action Client Node**: Handles target setting and interaction with the action server.
 - **Service Node**: Provides a service to retrieve the last target coordinates.
 - **Launch File**: Launches all the nodes.

**Note**:
- This assignment is completed using both **Python** and **C++**. 

## Node and Launch File Details

### Action Client Node

The **Action Client Node** is responsible for interacting with the Action Server in a client-server architecture.
This node is responsible for:
 - Sending target coordinates to the action server.
 - Handling user inputs.
    - Prompts user to provide target `x` and `y` coordinates for robot to move to.
    - Allows user to cancel goal, while the robot is moving.
 - Subscribing to `/odom` topic.
 - Publishing the robot's current position and velocity to a custom ROS topic `/robot_status`.

Key Features:
 - Receives feedback and monitors the goal status from the action server.
 - Dynamically updates target coordinates based on user inputs.

### Service node

This node provides a service to:
 - Retrieve the last target coordinates set by the user.

Key Features:
 - Listens for service calls (`/get_last_target`) and responds with the last target coordinates.
 - Maintains consistency with the Action Client Node through parameters.

### Launch File

The launch file simplifies the process of starting the nodes. It:
 - Launches the Action Client Node and Service Node simultaneously.
 - Outputs relevant logs and feedback directly to the terminal for easier debugging and monitoring.

## Repository Structure
The root of this repository is the package folder, which contains all necessary files and scripts for running the assignment nodes. When cloning the repository for the first time, place it directly in the `src` folder of your ROS workspace.

### Folder and File Overview
- **`/launch`**: Contains launch files
    - `coordinate_control_cpp.launch`: Launch file for C++ version
    - `coordinate_control_py.launch`: Launch file for the Python version, launching both nodes simultaneously.

- **`/msg`**: Contains custom message definitions.
    - `robot_status.msg`: Defines the custom message to publish the robot's position and velocity.

- **`/scripts`**: Contains Python scripts for nodes.
    - `action_client_node.py`: Python implementation of the action client node.
    - `service_node.py`: Python implementation of the service node.

- **`/src`**:  Contains C++ source files.
    - `action_client_node.cpp`: Main C++ code for the action client node.
    - `service_node.cpp`: C++ code for the service node fetching the last known target.

- **`/srv`**: Contains custom service definitions.
    - `get_last_target.srv`: Defines the service for retrieving the last target coordinates.

- **`/CMakeLists.txt`**: Specifies the package build rules and dependencies.

- **`/package.xml`**: Lists dependencies and package metadata.

- **`/README.md`**: This file (Documentation).

## Getting Started (Read Before Action)

### Prerequisites
Before proceeding, make sure that **`ROS Noetic`** is installed on your system.<br>
If you haven’t set up ROS yet, refer to the official installation guide for ROS Noetic on Ubuntu: <br>
(https://wiki.ros.org/noetic/Installation/Ubuntu) <br>

Additionally, you’ll need **`Python 3`** and **`python-is-python3`** package to run this project. Ensure these dependencies are installed on your system. If they are not installed, you can do so by running the following commands:
```bash
sudo apt-get update
sudo apt-get install python3
sudo apt-get install python-is-python3
```
Once you've completed the installation, you can proceed to set up your ROS workspace.

### Setup 

#### 1. Set up your ROS workspace
Create a new workspace (or use an existing one) and navigate to its `src` directory:
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

#### 2. Clone the `assignment_2_2024` package
Before cloning this repository, you need to get the `assignment_2_2024` package, which is responsible for starting the simulation environment and the action server. Clone the `assignment_2_2024` repository into your workspace’s `src` folder:
```bash
git clone https://github.com/CarmineD8/assignment_2_2024.git
```

#### 3. Clone this repository
Now that the `assignment_2_2024` is cloned, clone this repository into your workspace’s `src` folder:
```bash
git clone https://github.com/Rubin-unige/assignment2_rt_part1.git
```

#### 4. Build the workspace
Once both repositories are cloned, navigate back to the root of your workspace and build the packages using `catkin_make`:
```bash
cd ~/ros_ws
catkin_make
```

#### 5. Add the Workspace to your ROS Environment
To ensure that your workspace is sourced automatically every time you start a new terminal session, add it to your `.bashrc` file:
```bash
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 6. Source the workspace
After building and sourcing, source the workspace manually for the first time in the current terminal session:
```bash
source ~/ros_ws/devel/setup.bash
```

## Launching Nodes

#### 1. Launch the `assignment_2_2024` Package

Before launching your nodes, ensure that the simulation environment is running. To start the simulation in Gazebo and Rviz, run the following command:
```bash
roslaunch assignment_2_2024 assignment1.launch
```
This will spawn the robot in the simulation environment and start the action server, which the Action Client Node will interact with. <br>
**Note**: Please wait for everything to load properly before proceeding.

#### 2. Launch the Action Client Node and Service Node

After the simulation environment is running, you can proceed to launch either the **C++** or **Python** version of the nodes. 

#### Running the C++ Version
To launch the C++ version of the nodes, use the following command:
```bash
roslaunch assignment2_rt_part1 coordinate_control_cpp.launch
```
#### Running the Python Version
To launch the Python version of the nodes, use the following command:
```bash
roslaunch assignment2_rt_part1 coordinate_control_py.launch
```
This will launch both the Action Client Node and the Service Node simultaneously.

#### 3. Running ROS Service to Get Last Coordinates
After the Action Client Node is running, you can retrieve the last set target coordinates via the following service call:
```bash
rosservice call /get_last_target 
```
This will fetch the last target coordinates that were set for the robot, based on the most recent input from the user.

#### 4. Stopping the nodes

To stop the nodes, simply press `Ctrl+C` in the terminal where each nodes are running . This will terminate the nodes and stop the simulation.

## Implementation Details

### Action Client Node

### Service Node
The **Service Node** shares a similar structure in both **Python** and **C++**, with consistent core logic across implementations. Here, the **C++** version is used to explain the implementation.

#### 1. Subscribe to the `/reaching_goal/goal` topic
The Service Node subscribes to the /reaching_goal/goal topic to track the robot's current target coordinates. 
```cpp
ros::Subscriber sub = nh.subscribe("/reaching_goal/goal", 10, planningCallback);
```
The node extracts the `x` and `y` coordinates from the incoming messages and stores them in variables (`last_target_x` and `last_target_y`) to keep track of the most recent goal.
```cpp
void planningCallback(const assignment_2_2024::PlanningActionGoal::ConstPtr &msg){
    last_target_x = msg->goal.target_pose.pose.position.x;  // Access x coordinate
    last_target_y = msg->goal.target_pose.pose.position.y;  // Access y coordinate
}
```
This ensures the Service Node always has the latest target coordinates for responding to service calls or other queries.

#### 2. Publish the last target coordinate
The Service Node provides a ROS service that responds with the last target coordinates (`last_target_x` and `last_target_y`) when queried. 
The service is implemented using the ROS service mechanism. It listens for requests on a specific service topic `/get_last_target`.
```cpp
ros::ServiceServer service = nh.advertiseService("get_last_target", handle_get_last_target);
```
The callback function handles incoming service requests. It fills the response with the current values of `last_target_x` and `last_target_y`:
```cpp
bool handle_get_last_target(assignment2_rt_part1::get_last_target::Request &req,
    assignment2_rt_part1::get_last_target::Response &res){
    res.x = last_target_x;
    res.y = last_target_y;
    return true;  // Return success
}
```
Once the service is running, it can be called from the terminal using:
```bash
rosservice call /get_last_target
```
### Launch file

## Summary