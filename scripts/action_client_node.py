#!/usr/bin/env python

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction
from nav_msgs.msg import Odometry
from assignment2_rt_part1.msg import robot_status
import threading

# Global variables to store the robot's position and velocity
current_x = 0.0
current_y = 0.0
vel_x = 0.0
vel_y = 0.0
cancel_goal_flag = False  # Flag to cancel the goal

def odom_callback(msg):
    """Callback function to update robot's position and velocity from /odom."""
    global current_x, current_y, vel_x, vel_y
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    vel_x = msg.twist.twist.linear.x
    vel_y = msg.twist.twist.linear.y

def feedback_callback(feedback):
    """Feedback callback to handle updates from the action server."""
    rospy.loginfo(f"Feedback: {feedback.stat}")

def send_goal(client, target_x, target_y):
    """Send the goal to the action server."""
    goal = PlanningGoal()  # Create a goal instance
    goal.target.x = target_x  # Set the target x-coordinate
    goal.target.y = target_y  # Set the target y-coordinate
    rospy.loginfo(f"Sending goal: x={target_x}, y={target_y}")
    
    # Send the goal to the action server and listen for feedback
    client.send_goal(goal, feedback_cb=feedback_callback)

def cancel_goal(client):
    """Cancel the goal if the cancel flag is set."""
    global cancel_goal_flag
    if cancel_goal_flag:
        rospy.loginfo("Canceling goal!")
        client.cancel_goal()
        cancel_goal_flag = False  # Reset the cancel flag

def user_input_thread():
    """Thread to listen for user input (e.g., cancel command)."""
    global cancel_goal_flag
    while not rospy.is_shutdown():
        user_input = raw_input("Enter 'cancel' to cancel the current goal: ")
        if user_input.lower() == 'cancel':
            cancel_goal_flag = True  # Set the cancel flag when user types 'cancel'

def action_client_node():
    """Main action client node."""
    global cancel_goal_flag
    
    rospy.init_node('action_client_node')
    
    # Create an action client to communicate with the action server
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    client.wait_for_server()
    rospy.loginfo("Connected to action server")

    # Subscribe to /odom to get robot's position and velocity
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Publisher to send robot's position and velocity to the topic
    pub = rospy.Publisher('/robot_status', robot_status, queue_size=10)

    # Start a separate thread to handle user input
    input_thread = threading.Thread(target=user_input_thread)
    input_thread.daemon = True  # Make the thread a daemon so it stops when the main program exits
    input_thread.start()

    while not rospy.is_shutdown():
        try:
            # Get user input for target coordinates
            target_x = float(input("Enter target x coordinate: "))
            target_y = float(input("Enter target y coordinate: "))

            # Send the goal to the action server
            send_goal(client, target_x, target_y)

            # Monitor the goal state and robot's position while the goal is in progress
            while not client.get_state().is_done():
                cancel_goal(client)  # Check if the user wants to cancel the goal

                # Publish robot's current position and velocity
                pos_vel_msg = PositionVelocity()
                pos_vel_msg.x = current_x
                pos_vel_msg.y = current_y
                pos_vel_msg.vel_x = vel_x
                pos_vel_msg.vel_y = vel_y
                pub.publish(pos_vel_msg)

                rospy.sleep(0.5)  # Sleep for 100 ms to avoid spamming messages

            # After the goal is completed, log feedback
            if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached!")
            else:
                rospy.loginfo("Goal did not succeed!")

        except ValueError:
            rospy.logerr("Invalid input! Please enter a valid number for the target coordinates.")
        
        rospy.sleep(1)  # Sleep for 1 second before allowing another input

if __name__ == '__main__':
    try:
        action_client_node()  # Run the Action Client
    except rospy.ROSInterruptException:
        pass
