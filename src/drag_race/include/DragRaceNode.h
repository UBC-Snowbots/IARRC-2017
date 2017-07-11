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

// ROS Includes
// TODO: Sort me for neatness
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

// SB Includes
#include <sb_utils.h>
#include <LidarObstacleManager.h>

class DragRaceNode {
public:
    DragRaceNode(int argc, char **argv, std::string node_name);

    // TODO: doc comment
    // TODO: TEST ME!
    static geometry_msgs::Twist determineDesiredMotion(sensor_msgs::LaserScan& scan);

private:
    // TODO: Doc comment
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);

    // Manages obstacles, including the cones and wall
    LidarObstacleManager obstacle_manager;

    // Subscribes to the LaserScan
    ros::Subscriber scan_subscriber;
    // Publishes Twist messages to control the robot
    ros::Publisher twist_publisher;
    // Publishes the obstacles so we can see them in RViz
    ros::Publisher obstacle_debug_publisher;
};
#endif //DRAG_RACE_NODE_DRAG_RACE_H
