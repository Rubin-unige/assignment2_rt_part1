#!/usr/bin/env python

"""
.. module:: action_client_node
   :platform: Unix
   :synopsis: Python ROS node for controlling robot navigation via actionlib.

.. moduleauthor:: Rubin Khadka Chhetri

This module implements an **action client** for the ROS navigation stack. It allows users to:
    - Send **target coordinates** (goals) to the robot.
    - Cancel ongoing goals.
    - Monitor robot status (position/velocity) via `/odom`.
    - Publish robot status to `/robot_status`.

Subscribes to:
    - **/odom** (`nav_msgs/Odometry`): Current robot position and velocity.

Publishes to:
    - **/robot_status** (`assignment2_rt_part1/robot_status`): Custom message with robot state.

Actions:
    - **/reaching_goal** (`assignment_2_2024/PlanningAction`): Action server for navigation.

Dependencies:
    - ``rospy``
    - ``actionlib``
    - ``assignment_2_2024``
    - ``assignment2_rt_part1``
"""

import rospy
import actionlib
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import PlanningAction, PlanningGoal, PlanningFeedback
from assignment2_rt_part1.msg import robot_status

# Global variables
current_x = 0.0
"""float: Current x-coordinate of the robot (meters)."""
current_y = 0.0
"""float: Current y-coordinate of the robot (meters)."""
vel_x = 0.0
"""float: Linear velocity in x-direction (m/s)."""
vel_z = 0.0
"""float: Angular velocity around z-axis (rad/s)."""
cancel_goal_flag = False
"""bool: Flag to request goal cancellation."""
goal_active = False
"""bool: Flag indicating if a goal is active."""

def odom_callback(msg):
    """
    Callback for `/odom` subscriber. Updates robot pose and velocity.

    Args:
        msg (Odometry): Incoming message with robot state.
    """
    global current_x, current_y, vel_x, vel_z
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    vel_x = msg.twist.twist.linear.x
    vel_z = msg.twist.twist.angular.z

def feedback_callback(feedback):
    """
    Handles feedback from the action server.

    Args:
        feedback (PlanningFeedback): Feedback message containing status.
    """
    if feedback.stat == "Target reached!":
        rospy.loginfo("Goal reached! Press 'Enter' to continue.")

def send_goal(client, target_x, target_y):
    """
    Sends a navigation goal to the action server.

    Args:
        client (SimpleActionClient): Action client connected to `/reaching_goal`.
        target_x (float): Target x-coordinate (meters).
        target_y (float): Target y-coordinate (meters).
    """
    global goal_active
    goal = PlanningGoal()
    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.position.y = target_y
    goal.target_pose.pose.orientation.w = 1.0  # Neutral orientation
    rospy.loginfo(f"Sending goal: ({target_x}, {target_y})")
    client.send_goal(goal, feedback_cb=feedback_callback)
    goal_active = True

def cancel_goal(client):
    """
    Cancels the current goal if requested.

    Args:
        client (SimpleActionClient): Action client to send cancel request.
    """
    global cancel_goal_flag, goal_active
    if cancel_goal_flag and goal_active:
        client.cancel_goal()
        rospy.loginfo("Goal cancellation requested.")

def action_client_node():
    """
    Main function for the action client node.

    Handles:
    - User input for goals.
    - Goal cancellation.
    - Robot status publishing.
    """
    global cancel_goal_flag, goal_active

    rospy.init_node('action_client_node')
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    client.wait_for_server()
    rospy.loginfo("Action server connected.")

    rospy.Subscriber('/odom', Odometry, odom_callback)
    pub = rospy.Publisher('/robot_status', robot_status, queue_size=10)

    while not rospy.is_shutdown():
        try:
            # Get user input
            target_x = float(input("Enter target x: "))
            target_y = float(input("Enter target y: "))
            send_goal(client, target_x, target_y)

            # Monitor goal progress
            while not rospy.is_shutdown() and client.get_state() not in [
                actionlib.GoalStatus.SUCCEEDED,
                actionlib.GoalStatus.ABORTED,
                actionlib.GoalStatus.REJECTED
            ]:
                user_input = input("Enter 'cancel' to abort: ").strip().lower()
                if user_input == 'cancel':
                    cancel_goal_flag = True
                    cancel_goal(client)
                    break

                # Publish status
                msg = robot_status()
                msg.x = current_x
                msg.y = current_y
                msg.vel_x = vel_x
                msg.vel_z = vel_z
                pub.publish(msg)
                rospy.sleep(0.5)

            # Post-goal handling
            if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Ready for new coordinates.")
            elif cancel_goal_flag:
                rospy.loginfo("Goal canceled.")
            else:
                rospy.logwarn(f"Goal failed: {client.get_state()}")

            cancel_goal_flag = goal_active = False

        except ValueError:
            rospy.logerr("Invalid input! Use numbers only.")

if __name__ == '__main__':
    try:
        action_client_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")