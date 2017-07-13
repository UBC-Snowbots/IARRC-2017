/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#ifndef DRAG_RACE_NODE_DRAG_RACE_H
#define DRAG_RACE_NODE_DRAG_RACE_H

// STD Includes
#include <iostream>
#include <vector>

// ROS Includes
// TODO: Sort me for neatness
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

// SB Includes
#include <sb_utils.h>
#include <LidarObstacleManager.h>
#include <DragRaceController.h>

class DragRaceNode {
public:
    DragRaceNode(int argc, char **argv, std::string node_name);

private:
    // TODO: Doc comment
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan);

    void greenLightCallBack(const std_msgs::Bool &green_light_detected);

    // Manages obstacles, including the cones and wall
    LidarObstacleManager obstacle_manager;

    // Manages line handling and movement
    DragRaceController drag_race_controller;


    // How far lines can be before being considered invalid
    double max_distance_from_robot_accepted;

    // How far from the target line the robot should be
    double target_distance;

    // Where the target line is
    bool line_to_the_right;

    // Velocity limits
    double angular_vel_cap;
    double linear_vel_cap;

    // Scaling
    double theta_scaling_multiplier;
    double angular_speed_multiplier;
    double linear_speed_multiplier;

    // Traffic light detection
    int minimum_green_recognised_count;
    int green_count_recognised;

    // Subscribes to the LaserScan
    ros::Subscriber scan_subscriber;
    // Subscribes to traffic light detection
    ros::Subscriber traffic_light_subscriber;

    // Publishes Twist messages to control the robot
    ros::Publisher twist_publisher;
    // Publishes the obstacles so we can see them in RViz
    ros::Publisher cone_debug_publisher;
    // Publishes the cone lines so we can see them in RViz
    ros::Publisher cone_line_debug_publisher;
    // Publishes the cone line we're using to determine the twist message
    // so we can see it in RViz
    ros::Publisher best_line_debug_publisher;
};

#endif //DRAG_RACE_NODE_DRAG_RACE_H
