/*
 * Created By: Robyn Castro
 * Created On: June 15, 2017
 * Description: Determines whether or not green is seen on
 *              the screen then publishes a recommended twist
 *              message.
 * Usage:
 * Subscribes to: camera // TODO: Determine topic names
 * Publishes to:
 */

#ifndef GREEN_RECOGNITION_H
#define GREEN_RECOGNITION_H

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// ROS
#include <ros/ros.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// STD
#include <vector>
#include <string>

using namespace cv;

class GreenRecognition {

public:

    /**
     * Constructor
     */
    GreenRecognition(int argc, char **argv, std::string node_name);

private:

    /**
     * Callback for the filtered image
     *
     * @param address of filtered image matrix
     */
    void subscriberCallBack(const sensor_msgs::Image::ConstPtr& image);

    int findObjects(const Mat &filtered_image);
    Mat rosToMat(const sensor_msgs::Image::ConstPtr& image);
    /**
     * Subscribes to the filtered camera image node
     */
    ros::Subscriber image_sub;

    /**
     * Publishes the recommended twist message
     */
    ros::Publisher twist_pub;

};

#endif