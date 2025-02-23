#!/usr/bin/env python

"""
This node provides a service to retrieve the last target coordinates sent to the robot. It subscribes to the `/reaching_goal/goal` topic to monitor the target coordinates and stores them in global variables. When the `get_last_target` service is called, it returns the last target coordinates.

Subscribes to:
    - `/reaching_goal/goal` (PlanningActionGoal): The goal message containing the target coordinates.

Services:
    - `get_last_target` (get_last_target): Returns the last target coordinates (x, y).

Dependencies:
    - ``assignment_2_2024`` package 
"""

import rospy
from assignment2_rt_part1.srv import get_last_target, get_last_targetResponse
from assignment_2_2024.msg import PlanningActionGoal
from geometry_msgs.msg import PoseStamped

# Global variable to store last target coordinates
last_target_x = 0.0
"""float: The last target x-coordinate."""

last_target_y = 0.0
"""float: The last target y-coordinate."""

def planning_callback(msg):
    """
    Callback function to update the last target coordinates from the `/reaching_goal/goal` topic.

    Args:
        msg (PlanningActionGoal): The message containing the target coordinates.
    """
    global last_target_x, last_target_y
    # Access the target pose from the goal and update the coordinates
    last_target_x = msg.goal.target_pose.pose.position.x
    last_target_y = msg.goal.target_pose.pose.position.y
    rospy.loginfo(f"Updated last target coordinates: x={last_target_x}, y={last_target_y}")

def handle_get_last_target(req):
    """
    Service callback to return the last target coordinates.

    Args:
        req (get_last_targetRequest): The service request (empty in this case).

    Returns:
        get_last_targetResponse: The response containing the last target coordinates (x, y).
    """
    rospy.loginfo("Service called: Returning last target coordinates.")
    return get_last_targetResponse(last_target_x, last_target_y)

def service_node():
    """
    Main function for the service node.

    This node initializes the ROS node, subscribes to the `/reaching_goal/goal` topic to monitor target coordinates,
    and provides the `get_last_target` service to return the last target coordinates.
    """
    rospy.init_node('service_node')
    rospy.loginfo("Service node started.")

    # Subscribe to the /reaching_goal/goal topic (PlanningActionGoal)
    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, planning_callback)

    # Define the service with the name 'get_last_target' and the handler
    rospy.Service('get_last_target', get_last_target, handle_get_last_target)

    rospy.loginfo("Service 'get_last_target' is ready")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        service_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Service node terminated.")