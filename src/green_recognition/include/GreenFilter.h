/*
 * Created By: Robyn Castro
 * Created On: June 15, 2017
 * Description: Anything green is mapped to white, everything
 *              else is mapped to black.
 *
 * Usage:
 * Subscribes to: camera // TODO: Determine topic names
 * Publishes to:
 */

#ifndef GREEN_FILTER_H
#define GREEN_FILTER_H

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <ros/package.h>

// STD
#include <vector>
#include <string>

// I/O
#include <iostream>
#include <stdio.h>
#include <fstream>

// Snowbots
#include <ManualFilter.h>
#include <sb_utils.h>

using namespace cv;

class GreenFilter {

public:

    /**
     * Empty Constructor for testing purposes
     */
    GreenFilter();

    /**
     * Constructor
     */
    GreenFilter(int argc, char **argv, std::string node_name);

private:

    /**
     * Callback for the filtered image
     *
     * @param address of filtered image matrix
     */
    void subscriberCallBack(const sensor_msgs::Image::ConstPtr& image);

    void setUpFilter();

    Mat rosToMat(const sensor_msgs::Image::ConstPtr& image);

    void check_if_image_exist(const cv::Mat &img, const std::string &path);

    /**
     * Subscribes to the raw camera image node
     */
    ros::Subscriber image_sub;

    /**
     * Publishes the filtered image
     */
    ros::Publisher filter_pub;

    //Frequency handling
    ros::Time last_published;
    ros::Duration publish_interval;


    // The name and size of the display window
    std::string displayWindowName;

    //Filters and their variables
    snowbotsFilter filter;
    std::string mfilter_file;
    double frequency;
    int width , height;
    int x1, x2, x3, x4, y1, y2, y3, y4;
    std::vector<cv::Point2f> orig_points;
    std::vector<cv::Point2f> dst_points;

    // Whether or not we've received the first image
    bool receivedFirstImage;

    //Debug and calibration variables
    bool showWindow;
    bool isCalibratingManually;




};

#endif