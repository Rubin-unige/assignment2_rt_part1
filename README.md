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
    - `coordinate_control_py.launch`: Launch file that starts the Action Client Node and Service Node simultaneously.

- **`/msg`**: Contains custom message definitions.
    - `robot_status.msg`: Defines the custom message to publish the robot's position and velocity.

- **`/scripts`**: Contains Python scripts used for the nodes in this project.
    - `action_client_node.py`: Python version of action client node.
    - `service_node.py`: Python version of service node.

- **`/src`**: Contains C++ source files.
    - `action_client_node.cpp`: The main action client code.
    - `service_node.cpp`: Service node that fetches last known target.

- **`/srv`**: Contains custom service definitions.
    - `get_last_target.srv`: Defines the service for retrieving the last target coordinates.

- **`/CMakeLists.txt`**: Specifies the package build rules and dependencies.

- **`/package.xml`**: Lists dependencies and package metadata.

- **`/README.md`**: This file (Documentation).


## Getting Started (Read Before Action)
## Launching Nodes
## Implementation Details
## Summary