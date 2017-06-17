/*
 * Created By: Robyn Castro
 * Created On: June 17, 2017
 * Description: Determines whether or not green is seen on
 *              the screen then publishes a recommended twist
 *              message.
 */

#include <GreenRecognition.h>

using namespace cv;
using namespace cv_bridge;

GreenRecognition::GreenRecognition(int argc, char **argv, std::string node_name) {

    // Setup handles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup subscriber
    std::string image_topic = "/robot/line_detect/camera_image"; // TODO: Get topic name
    int refresh_rate = 10;
    image_sub = nh.subscribe(image_topic, refresh_rate,
                                             &GreenRecognition::subscriberCallBack, this);

    // Setup publishers
    std::string twist_topic = "/robot/lane_follow/twist_message"; // TODO: Decide on topic names
    uint32_t queue_size = 1;

    twist_pub = private_nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);

}

void GreenRecognition::subscriberCallBack(const sensor_msgs::Image::ConstPtr& image) {

    geometry_msgs::Twist twist_message;

    // Make sure all values inside twist_message are 0
    twist_message.angular.x = 0;
    twist_message.angular.y = 0;
    twist_message.angular.z = 0;
    twist_message.linear.x = 0;
    twist_message.linear.y = 0;
    twist_message.linear.z =0;

    // If something is seen tell the robot to move
    int numObjects = findObjects(rosToMat(image));
    if(numObjects > 0)
        twist_message.linear.x = 1;

    twist_pub.publish(twist_message);
}

Mat GreenRecognition::rosToMat(const sensor_msgs::Image::ConstPtr& image) {
    CvImagePtr imagePtr;
    imagePtr = toCvCopy(image, image->encoding);
    return imagePtr->image;
}

int GreenRecognition::findObjects(const Mat &filtered_image) {

    cv::findContours( filtered_image.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    size_t count = contours.size();

    for( int i=0; i<count; i++)
    {
        cv::Point2f c;
        float r;
        cv::minEnclosingCircle( contours[i], c, r);

    }

    // TODO: FIX THIS FUNCTION
}
