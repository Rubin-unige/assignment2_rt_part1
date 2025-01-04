#!/usr/bin/env python

import rospy
from assignment2_rt_part1.srv import get_last_target, get_last_targetResponse
from assignment_2_2024.msg import PlanningGoal

# Global variable to store last target coordinates
last_target_x = 0.0
last_target_y = 0.0

def planning_callback(msg):
    """Callback function to update the last target coordinates from the PlanningGoal."""
    global last_target_x, last_target_y
    # Assuming PlanningGoal contains x and y as part of the goal
    last_target_x = msg.target_x
    last_target_y = msg.target_y

def handle_get_last_target(req):
    """Service callback to return the last target coordinates."""
    return get_last_targetResponse(last_target_x, last_target_y)

def service_node():
    """Service node to provide the last target coordinates."""
    rospy.init_node('service_node')

    # Subscribe to the /reaching_goal/goal topic
    rospy.Subscriber('/reaching_goal/goal', PlanningGoal, planning_callback)

    # Define the service with the name 'get_last_target' and the handler
    rospy.Service('get_last_target', get_last_target, handle_get_last_target)

    rospy.loginfo("Service 'get_last_target' is ready and subscribing to /reaching_goal/goal.")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        service_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Target service node terminated.")
