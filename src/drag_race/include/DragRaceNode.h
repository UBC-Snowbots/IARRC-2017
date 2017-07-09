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

    /**
     * Determines the optimal movement to stay within target distance of
     * the given line.
     *
     * @param longestConeLine
     * @param targetDistance
     * @param lineToTheRight
     * @param theta_scaling_multiplier
     * @param angular_speed_multiplier
     * @param linear_speed_multiplier
     * @param angular_vel_cap
     * @param linear_vel_cap
     * @return the optimal angular and linear acceleration.
     */
    static geometry_msgs::Twist determineDesiredMotion(LineOfBestFit *longestConeLine, double targetDistance,
                                                       bool lineToTheRight, double theta_scaling_multiplier,
                                                       double angular_speed_multiplier, double linear_speed_multiplier,
                                                       double angular_vel_cap, double linear_vel_cap);

private:
    // TODO: Doc comment
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan);

    /**
     * Finds the minimum distance from given line and the origin.
     *
     * @param line
     * @return the minimum distance from given line and the origin.
     */
    static double determineDistanceFromLine(LineOfBestFit *line);

    // Manages obstacles, including the cones and wall
    LidarObstacleManager *obstacle_manager;

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

    // Subscribes to the LaserScan
    ros::Subscriber scan_subscriber;
    ros::Publisher twist_publisher;
};

#endif //DRAG_RACE_NODE_DRAG_RACE_H
