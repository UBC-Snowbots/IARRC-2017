/*
 * Created By: Robyn Castro
 * Created On: June 17, 2017
 * Description: Determines whether there is green being seen by the
 *              camera.
 */

#include <ros/ros.h>
#include <GreenRecognition.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

class Twist;

using namespace cv;

GreenRecognition::GreenRecognition(int argc, char **argv, std::string node_name) {

    // Setup handles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup subscriber
    std::string image_topic = "/robot/line_detect/camera_image"; // TODO: Get topic name
    int refresh_rate = 10;
    ros::Subscriber image_sub = nh.subscribe(image_topic, refresh_rate,
                                             &GreenRecognition::subscriberCallBack, this);

    // Setup publishers
    std::string twist_topic = "/robot/lane_follow/twist_message"; // TODO: Decide on topic names
    uint32_t queue_size = 1;

    ros::Publisher twist_pub = private_nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);

}

