#!/usr/bin/env python

"""
.. module:: target_coordinates_service
   :platform: Unix
   :synopsis: ROS service node to track and provide last navigation target coordinates.

This node:
    - Subscribes to `/reaching_goal/goal` to monitor target coordinates
    - Provides a `get_last_target` service to retrieve the last received coordinates

Subscribes to:
    - `/reaching_goal/goal` (assignment_2_2024/PlanningActionGoal)

Services:
    - `get_last_target` (assignment2_rt_part1/get_last_target)
    
Dependencies:
    - ``assignment_2_2024``
    - ``assignment2_rt_part1``
"""

import rospy
from assignment2_rt_part1.srv import get_last_target, get_last_targetResponse
from assignment_2_2024.msg import PlanningActionGoal

# Global storage for last target
last_target_x = 0.0
"""float: Last received x-coordinate (meters)."""

last_target_y = 0.0
"""float: Last received y-coordinate (meters)."""

def planning_callback(msg):
    """
    Updates last target coordinates from action goal messages.

    Args:
        msg (PlanningActionGoal): Incoming goal message containing target pose.
    """
    global last_target_x, last_target_y
    last_target_x = msg.goal.target_pose.pose.position.x
    last_target_y = msg.goal.target_pose.pose.position.y
    rospy.logdebug(f"New target received: ({last_target_x:.2f}, {last_target_y:.2f})")

def handle_get_last_target():
    """
    Service handler that returns the last stored target coordinates.

    Returns:
        get_last_targetResponse: Service response with x,y coordinates
    """
    return get_last_targetResponse(last_target_x, last_target_y)

def service_node():
    """
    Initializes and runs the target coordinates service node.
    """
    rospy.init_node('target_coordinates_service')
    
    # Setup subscriber and service
    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, planning_callback)
    rospy.Service('get_last_target', get_last_target, handle_get_last_target)
    
    rospy.loginfo("Target coordinates service ready")
    rospy.spin()

if __name__ == '__main__':
    try:
        service_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Service shutdown complete")