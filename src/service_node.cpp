#include <ros/ros.h>
#include <assignment2_rt_part1/get_last_target.h>
#include <assignment_2_2024/PlanningActionGoal.h>  
#include <geometry_msgs/PoseStamped.h>

// Global variables to store the last target coordinates
double last_target_x = 0.0;
double last_target_y = 0.0;

// Callback function for receiving PlanningActionGoal messages
void planningCallback(const assignment_2_2024::PlanningActionGoal::ConstPtr &msg)
{
    // Access the pose from target_pose and update the coordinates
    last_target_x = msg->goal.target_pose.pose.position.x;  // Access x coordinate
    last_target_y = msg->goal.target_pose.pose.position.y;  // Access y coordinate
}

// Service callback function
bool handle_get_last_target(assignment2_rt_part1::get_last_target::Request &req,
    assignment2_rt_part1::get_last_target::Response &res)
{
    // Set the response with the last target coordinates
    res.x = last_target_x;
    res.y = last_target_y;

    return true;  // Return success
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "service_node");
    ros::NodeHandle nh;

    // Subscribe to the /reaching_goal/goal topic (PlanningActionGoal)
    ros::Subscriber sub = nh.subscribe("/reaching_goal/goal", 10, planningCallback);

    // Define the service with the name 'get_last_target' and the callback function
    ros::ServiceServer service = nh.advertiseService("get_last_target", handle_get_last_target);

    ROS_INFO("Service 'get_last_target' is ready.");

    // Keep the node running and wait for requests
    ros::spin();

    return 0;
}
