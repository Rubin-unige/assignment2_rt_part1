#include <ros/ros.h>
#include <assignment2_rt_part1/get_last_target.h>

// Service callback function
bool handle_get_last_target(assignment2_rt_part1::get_last_target::Request &req,
    assignment2_rt_part1::get_last_target::Response &res)
{
    // Read the target coordinates from ROS parameters
    double last_target_x = ros::param::param("last_target_x", 0.0); 
    double last_target_y = ros::param::param("last_target_y", 0.0);

    // Set the response with the target coordinates
    res.x = last_target_x;
    res.y = last_target_y;

    return true;  // Return success
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "service_node");
    ros::NodeHandle nh;

    // Define the service with the name 'get_last_target' and the callback function
    ros::ServiceServer service = nh.advertiseService("get_last_target", handle_get_last_target);

    ROS_INFO("Service 'get_last_target' is ready.");

    // Keep the node running and wait for requests
    ros::spin();

    return 0;
}
