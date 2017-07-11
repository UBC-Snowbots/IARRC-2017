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

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <LineDetect.h>
#include "sb_utils.h"

// temp headers
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

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

    // Moving away from line variables
    double target_x_distance;
    double target_y_distance;

    /**
     * Subscribes to the raw camera image node
     */
    image_transport::Subscriber image_sub;

    /**
     * Publishes the filtered image
     */
    image_transport::Publisher filter_pub;

    /**
     * Publishes the recommended twist message
     */
    ros::Publisher twist_pub;

    /**
     * Converts ros::sensor_msgs::Image into a cv::Mat
     *
     * @param message to be converted
     */
    cv::Mat rosToMat(const sensor_msgs::Image::ConstPtr& image);
};

#endif

