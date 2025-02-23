#!/usr/bin/env python

"""
.. module:: action_client_node
   :platform: Unix
   :synopsis: Python module for controlling the robot using ROS actions.
.. moduleauthor:: Rubin Khadka Chhetri

This node acts as an action client for the ROS navigation stack. It allows the user to send goals (target coordinates) to the robot and cancel them if needed. The node also subscribes to the `/odom` topic to monitor the robot's position and velocity, and publishes this information to the `/robot_status` topic.

Subscribes to:
    - `/odom` (Odometry): The robot's current position and velocity.

Publishes to:
    - `/robot_status` (robot_status): The robot's current position and velocity.

Actions:
    - `/reaching_goal` (PlanningAction): The action server for sending goals to the robot.

Dependencies:
    - ``assignment_2_2024`` package
"""

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal, PlanningFeedback
from nav_msgs.msg import Odometry
from assignment2_rt_part1.msg import robot_status

# Global variables to store the robot's position and velocity
current_x = 0.0
"""float: The current x-coordinate of the robot."""
current_y = 0.0
"""float: The current y-coordinate of the robot."""
vel_x = 0.0
"""float: The linear velocity of the robot in the x-direction."""
vel_z = 0.0
"""float: The angular velocity of the robot around the z-axis."""
cancel_goal_flag = False
"""bool: Flag to indicate if the current goal should be canceled."""
goal_active = False
"""bool: Flag to indicate if a goal is currently active."""

def odom_callback(msg):
    """
    Callback function to update the robot's position and velocity from the `/odom` topic.

    Args:
        msg (Odometry): The message containing the robot's current position and velocity.
    """
    global current_x, current_y, vel_x, vel_z
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    vel_x = msg.twist.twist.linear.x
    vel_z = msg.twist.twist.angular.z

def feedback_callback(feedback):
    """
    Callback function to handle feedback from the action server.

    Args:
        feedback (PlanningFeedback): The feedback message from the action server.
    """
    if feedback.stat == "Target reached!":
        rospy.loginfo("Goal Reached!! Press 'Enter' to continue!!")

def send_goal(client, target_x, target_y):
    """
    Send a goal to the action server.

    Args:
        client (SimpleActionClient): The action client used to communicate with the action server.
        target_x (float): The target x-coordinate for the robot.
        target_y (float): The target y-coordinate for the robot.
    """
    global goal_active
    goal = PlanningGoal()  # Create a goal instance
    goal.target_pose.pose.position.x = target_x  # Set the target x-coordinate
    goal.target_pose.pose.position.y = target_y  # Set the target y-coordinate
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0
    rospy.loginfo(f"Sending goal: x={target_x}, y={target_y}")
   
    # Send the goal to the action server and listen for feedback
    client.send_goal(goal, done_cb=None, active_cb=None, feedback_cb=feedback_callback)
    goal_active = True  # Mark the goal as active

def cancel_goal(client):
    """
    Cancel the current goal if the cancel flag is set and the robot is moving.

    Args:
        client (SimpleActionClient): The action client used to communicate with the action server.
    """
    global cancel_goal_flag, goal_active
    if cancel_goal_flag and goal_active:
        client.cancel_goal()

def action_client_node():
    """
    Main function for the action client node.

    This node allows the user to send goals to the robot and cancel them if needed.
    It also publishes the robot's current position and velocity to the `/robot_status` topic.
    """
    global cancel_goal_flag, goal_active
    
    rospy.init_node('action_client_node')
    
    # Create an action client to communicate with the action server
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    client.wait_for_server()
    rospy.loginfo("Connected to action server")

    # Subscribe to /odom to get robot's position and velocity
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Publisher to send robot's position and velocity to the topic
    pub = rospy.Publisher('/robot_status', robot_status, queue_size=10)
    
    while not rospy.is_shutdown():
        try:
            # Get user input for target coordinates
            target_x = float(input("Enter target x coordinate: "))
            target_y = float(input("Enter target y coordinate: "))

            # Send the goal to the action server
            send_goal(client, target_x, target_y)

            # Monitor the goal state and robot's position while the goal is in progress
            while not rospy.is_shutdown() and client.get_state() not in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
                # Check for user input to cancel the goal
                user_input = input("Robot running!! Enter 'cancel' to cancel the current goal: ").strip().lower()
                if user_input == 'cancel':
                    cancel_goal_flag = True
                    cancel_goal(client)
                    break
                    
                # Publish robot's current position and velocity
                pos_vel_msg = robot_status()
                pos_vel_msg.x = current_x
                pos_vel_msg.y = current_y
                pos_vel_msg.vel_x = vel_x
                pos_vel_msg.vel_z = vel_z
                pub.publish(pos_vel_msg)

                rospy.sleep(0.5)  # Sleep for 500 ms to avoid spamming messages

            # Check the outcome of the goal
            if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Please Enter new coordinates !!")
            elif cancel_goal_flag:
                rospy.loginfo("Goal was canceled by the user.")
            else:
                rospy.loginfo("Goal did not succeed. Status: %d", client.get_state())

            cancel_goal_flag = False
            goal_active = False  # Reset goal_active after processing the goal

        except ValueError:
            rospy.logerr("Invalid input! Please enter valid numbers for the target coordinates.")
        
        # After handling the goal or cancelation, return to accepting new coordinates
        cancel_goal_flag = False  # Reset the cancel flag before the next iteration
        rospy.sleep(0.5)  # Sleep for 1 second before allowing another input


if __name__ == '__main__':
    try:
        action_client_node()  # Run the Action Client
    except rospy.ROSInterruptException:
        rospy.loginfo("Action client node terminated.")