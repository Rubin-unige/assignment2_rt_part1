#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2024/PlanningAction.h>
#include <assignment2_rt_part1/robot_status.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <iostream>
#include <limits>

// Global variables to store current position and velocity
double current_x = 0.0, current_y = 0.0, vel_x = 0.0, vel_z = 0.0;
bool cancel_goal_flag = false; // Flag to handle cancelation
bool goal_active = false;

// Function declarations
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
bool getValidCoordinate(const std::string& prompt, double& coordinate);
void sendGoal(actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>& ac, double target_x, double target_y);
void cancelGoal(actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>& ac);

// Main function
int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "action_client_node");
    ros::NodeHandle nh;

    // Set up action client
    actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> ac("/reaching_goal", true);
    ac.waitForServer();
    ROS_INFO("Action Server Started !!!");

    // Subscribe to /odom to get position and velocity
    ros::Subscriber sub_odom = nh.subscribe("/odom", 10, odomCallback);

    // Publisher for position and velocity
    ros::Publisher pub_pos_vel = nh.advertise<assignment2_rt_part1::robot_status>("/robot_status", 10);

    while (ros::ok()) {
        try {
            double target_x, target_y;

            // Get valid user coordinates for the goal
            if (!getValidCoordinate("Enter target x:", target_x)) {
                continue;
            }

            if (!getValidCoordinate("Enter target y:", target_y)) {
                continue;
            }

            // Send the goal to the action server
            sendGoal(ac, target_x, target_y);

            // Monitor the goal state and robot's position while the goal is in progress
            while (!ac.getState().isDone() && ros::ok()) {
                // Check for cancel command from user input
                std::string user_input;
                std::cout << "Robot Moving!!! Enter 'cancel' to cancel the goal: ";
                std::cin >> user_input;

                if (user_input == "cancel") {
                    cancel_goal_flag = true;
                    cancelGoal(ac);
                    std::cin.clear(); // Clear error
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    break;
                }

                // Publish robot's current position and velocity
                assignment2_rt_part1::robot_status pos_vel_msg;
                pos_vel_msg.x = current_x;
                pos_vel_msg.y = current_y;
                pos_vel_msg.vel_x = vel_x;
                pos_vel_msg.vel_z = vel_z;
                pub_pos_vel.publish(pos_vel_msg);

                ros::Duration(0.5).sleep();  // Sleep for 500 ms to avoid spamming messages
            }

            // Check the outcome of the goal
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Goal reached successfully!");
            } else if (cancel_goal_flag) {
                ROS_INFO("Goal was canceled by the user.");
            } else {
                ROS_INFO("Goal did not succeed. Status: %d", ac.getState().state_);
            }

            cancel_goal_flag = false;  // Reset cancel flag
            goal_active = false;

        } catch (const std::invalid_argument& e) {
        }

        ros::Duration(1).sleep();  // Sleep for 1 second before accepting new input
    }

    ros::shutdown();
    return 0;
}

// Callback to update the robot's position and velocity from /odom
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
    vel_x = msg->twist.twist.linear.x;
    vel_z = msg->twist.twist.angular.z;
}

// Function to get valid user input for coordinates
bool getValidCoordinate(const std::string& prompt, double& coordinate) {
    std::string input;
    while (true) {
        std::cout << prompt << " ";
        std::getline(std::cin, input);

        try {
            size_t pos;
            coordinate = std::stod(input, &pos);  // Try converting to double
            
            // Check if the whole string was consumed by std::stod
            if (pos == input.length()) {
                return true;  // Valid number, no leftover characters
            } else {
                std::cerr << "Invalid input! Please enter a valid number." << std::endl;
            }
        } catch (const std::invalid_argument&) {
            std::cerr << "Invalid input! Please enter a valid number." << std::endl;
        }
    }
}

void sendGoal(actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>& ac, double target_x, double target_y) {
    assignment_2_2024::PlanningGoal goal;  // Create a goal instance
    goal.target_pose.pose.position.x = target_x;  // Set the target x-coordinate
    goal.target_pose.pose.position.y = target_y;  // Set the target y-coordinate
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    // Log the goal
    ROS_INFO("Sending goal: x=%.2f, y=%.2f", target_x, target_y);

    // Set the target coordinates as ROS parameters
    ros::param::set("last_target_x", target_x);
    ros::param::set("last_target_y", target_y);

    // Send the goal to the action server and listen for feedback
    ac.sendGoal(goal);  // Set up feedback callback
    goal_active = true;  // Mark the goal as active
}

void cancelGoal(actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>& ac) {
    if (cancel_goal_flag && goal_active) {
        // Cancel the active goal if the cancel flag is set
        ac.cancelGoal();
    }
}