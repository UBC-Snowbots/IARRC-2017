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
#include <image_transport/subscriber.h>

// ROS
#include <ros/ros.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// STD
#include <vector>
#include <string>

// Snowbots
#include <sb_utils.h>

using namespace cv;

class GreenRecognition {

public:

    // Constructors used for testing
    GreenRecognition(std::string &image_path);

    GreenRecognition();

    /**
     * Constructor
     */
    GreenRecognition(int argc, char **argv, std::string node_name);

    /**
     * Counts the number of circles found in the image.
     *
     * @param image to be parsed
     */
    int countCircles(const Mat &filtered_image);

private:

    /**
     * Callback for the filtered image
     *
     * @param address of filtered image matrix
     */
    void subscriberCallBack(const sensor_msgs::Image::ConstPtr &image);

    /**
     * Converts ros::sensor_msgs::Image into a cv::Mat
     *
     * @param message to be converted
     */
    Mat rosToMat(const sensor_msgs::Image::ConstPtr &image);

    /**
     *  Displays a window with the detected objects being circled
     */
    void showFilteredObjectsWindow(const Mat &filtered_image, std::vector<cv::Point2i> center,
                                   std::vector<int> radii);

    /**
     * Determines whether path contains an image.
     *
     */
    void check_if_image_exist(const cv::Mat &img, const std::string &path);

    /**
     * Subscribes to the filtered camera image node
     */
    image_transport::Subscriber image_sub;

    /**
     * Publishes the recommended twist message
     */
    ros::Publisher boolean_pub;

    // Minimum radius needed to be considered an object
    int minTargetRadius;

    // Show window for debugging purposes
    bool showWindow;

};

#endif