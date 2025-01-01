#!/usr/bin/env python

import rospy
from assignment2_rt_part1.srv import get_last_target, get_last_targetResponse

def handle_get_last_target(req):
    """Service callback to return the last target coordinates."""
     # Read the target coordinates from ROS parameters
    last_target_x = rospy.get_param('last_target_x', 0.0)  # Default value if not set
    last_target_y = rospy.get_param('last_target_y', 0.0)  # Default value if not set
    rospy.loginfo("Returning last target coordinates: x={}, y={}".format(last_target_x, last_target_y))
    return get_last_targetResponse(last_target_x, last_target_y)

def service_node():
    """Service node to provide the last target coordinates."""
    rospy.init_node('service_node')

    # Define the service with the name 'get_last_target' and the handler
    rospy.Service('get_last_target', get_last_target, handle_get_last_target)

    rospy.loginfo("Service 'get_last_target' is ready.")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        service_node() 
    except rospy.ROSInterruptException:
        rospy.loginfo("Target service node terminated.")
