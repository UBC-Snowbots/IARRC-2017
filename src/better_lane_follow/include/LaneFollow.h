/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Gets angle of lane lines point of intersection
 *              relative to the robot and broadcasts a
 *              recommended Twist message.
 * Usage:
 * Subscribes to:
 * Publishes to:
 */

#ifndef LANE_FOLLOW_H
#define LANE_FOLLOW_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <LineDetect.h>
#include "sb_utils.h"

class LaneFollow {

public:

    /**
     * Constructor
     */
    LaneFollow(int argc, char **argv, std::string node_name);

private:

    /**
     * Callback for the filtered image
     *
     * @param address of filtered image matrix
     */
    void subscriberCallBack(const sensor_msgs::ImageConstPtr &msg);

    // Angle of POI of detected lane lines relative to the robot
    int angle_theta;

    /**
     * Computes the average of 1/x and sqrt(y), multiplied by their respective scaling factors
     *
     * @param x
     * @param y
     * @param x_scale the value to multiply the x value by
     * @param y_scale the value to multiply the y value by
     *
     * @return the average of: [x_scale * 1/x] and [y_scale * sqrt(y)]
     */
    double magicFunction(double x, double y, double x_scale, double y_scale);

    // Instantiate LineDetect class which generates the lane lines
    LineDetect ld;

    // Velocity limits
    double angular_vel_cap;
    double linear_vel_cap;

    // Scaling
    double angular_speed_multiplier;
    double linear_speed_multiplier;

    /**
     * Subscribes to the raw camera image node
     */
    ros::Subscriber image_sub;

    /**
     * Publishes the filtered image
     */
    ros::Publisher filter_pub;

    /**
     * Publishes the recommended twist message
     */
    ros::Publisher twist_pub;

};

#endif

