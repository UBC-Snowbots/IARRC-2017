/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Gets angle of point of intersection of lane lines
 *              relative to the robot and broadcasts a
 *              recommended Twist message.
 */

#include <ros/ros.h>
#include <LaneFollow.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

class Twist;

using namespace cv;

LaneFollow::LaneFollow(int argc, char** argv, std::string node_name) {

    // Setup handles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup subscriber
    std::string image_topic_name = "/robot/line_detect/camera_image";
    int refresh_rate = 10;
    ros::Subscriber image_sub = nh.subscribe(image_topic_name, refresh_rate,
                                             &LaneFollow::subscriberCallBack, this);

    // Setup publishers
    std::string filter_topic_name = "/robot/lane_follow/filtered_image";
    std::string twist_topic_name = "/robot/lane_follow/twist_message";
    uint32_t queue_size = 1;

    ros::Publisher filter_pub = private_nh.advertise<cv::Mat>(image_topic_name, queue_size);
    ros::Publisher twist_pub = private_nh.advertise<geometry_msgs::Twist>(twist_topic_name, queue_size);

    ros::Rate loop_rate(10);

    int angle_theta = 0;

    /*while (ros::ok()) {

        // ...
        ros::spinOnce();
        loop_rate.sleep();
    }*/
}

void LaneFollow::subscriberCallBack(const Mat& img) {

    // ...

    // Publish filtered image to an arbitrary topic
    filter_pub.publish(img);

    // Calculate angle of POI of lane lines and broadcast a recommended Twist message
    angle_theta = LaneFollow::angleDetermine(const Mat& img);

    geometry_msgs::Twist stayInLane;

    // ...

    // Publish recommended Twist message
    twist_pub.publish(stayInLane);
}

double LaneFollow::magicFunction(double x, double y, double x_scale, double y_scale){
    return (1/fabs(x)*x_scale + sqrt(fabs(y))*y_scale)/2;
}

int angleDetermine(const Mat& img) {
    // ...

}
