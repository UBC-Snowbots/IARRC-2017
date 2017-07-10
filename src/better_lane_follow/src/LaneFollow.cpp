/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Gets angle of point of intersection of lane lines
 *              relative to the robot and broadcasts a
 *              recommended Twist message.
 */

#include <LaneFollow.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

class Twist;

using namespace cv;

LaneFollow::LaneFollow(int argc, char **argv, std::string node_name) {

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

    ros::Publisher filter_pub = private_nh.advertise<sensor_msgs::Image>(image_topic_name, queue_size);
    ros::Publisher twist_pub = private_nh.advertise<geometry_msgs::Twist>(twist_topic_name, queue_size);

    ros::Rate loop_rate(10);

    // Get Params
    SB_getParam(nh, "angular_vel_cap", angular_vel_cap, 1.0);
    SB_getParam(nh, "linear_vel_cap", linear_vel_cap, 1.0);
    SB_getParam(nh, "angular_speed_multiplier", angular_speed_multiplier, 1.0);
    SB_getParam(nh, "linear_speed_multiplier", linear_speed_multiplier, 1.0);

    /*while (ros::ok()) {

        // ...
        ros::spinOnce();
        loop_rate.sleep();
    }*/
}

void LaneFollow::subscriberCallBack(const sensor_msgs::ImageConstPtr &msg) {

    // The command to return
    geometry_msgs::Twist stayInLane;

    // Set components we don't care about to 0
    stayInLane.linear.y = 0;
    stayInLane.linear.z = 0;
    stayInLane.angular.x = 0;
    stayInLane.angular.y = 0;

    // TODO: Populate the vector with poly fit.
    std::vector<Polynomial> boundaryLines;
    double angle_heading = 0;

    // Head to the middle of the line if 2 lines exist
    if (boundaryLines.size() >= 2) {
        Point intersectionPoint = LineDetect::getIntersection(boundaryLines[0], boundaryLines[1]);
        angle_heading = LineDetect::getAngleFromOriginToPoint(intersectionPoint);
    }
    // Head parallel to the line of only 1 line exists
    else if (boundaryLines.size() == 1) {
        angle_heading = atan(boundaryLines[0].c);
    }
    // If no lines are seen go straight (See initialization)

    // Figure out how fast we should turn
    stayInLane.angular.z = pow(angle_heading, 2.0) * angular_speed_multiplier;

    // Limit the angular speed
    if(stayInLane.angular.z > angular_vel_cap)
        stayInLane.angular.z = angular_vel_cap * stayInLane.angular.z / fabs(stayInLane.angular.z);

    // Figure out how fast we should move forward
    stayInLane.linear.x = linear_speed_multiplier / fabs(stayInLane.angular.z);

    // Limit the linear speed
    if(stayInLane.linear.x > linear_vel_cap)
        stayInLane.linear.x = linear_vel_cap;

    twist_pub.publish(stayInLane);
}

double LaneFollow::magicFunction(double x, double y, double x_scale, double y_scale) {
    return (1 / fabs(x) * x_scale + sqrt(fabs(y)) * y_scale) / 2;
}

