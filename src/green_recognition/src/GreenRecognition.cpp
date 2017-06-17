/*
 * Created By: Robyn Castro
 * Created On: June 17, 2017
 * Description: Determines whether or not green is seen on
 *              the screen then publishes a recommended twist
 *              message.
 */

#include <GreenRecognition.h>

using namespace cv;
using namespace std;
using namespace cv_bridge;

GreenRecognition::GreenRecognition(std::string image_path) {

    cv::Mat bgr_image = imread(image_path);

    // Check if the image can be loaded
    check_if_image_exist(bgr_image, image_path);

    findObjects(bgr_image);

}
GreenRecognition::GreenRecognition(int argc, char **argv, std::string node_name) {

    // Setup handles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup subscriber
    std::string image_topic = "/robot/vision/filtered_image";
    int refresh_rate = 10;
    image_sub = nh.subscribe(image_topic, refresh_rate,
                                             &GreenRecognition::subscriberCallBack, this);

    // Setup publishers
    std::string twist_topic = private_nh.resolveName("command");
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

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<cv::Point2i> center;
    cv::Mat bwImage;
    cv::cvtColor(filtered_image, bwImage, CV_RGB2GRAY);
    vector<int> radii;

    cv::findContours( bwImage.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    size_t count = contours.size();
    int minTargetRadius = 50;

    for( int i=0; i<count; i++)
    {
        cv::Point2f c;
        float r;
        cv::minEnclosingCircle( (Mat) contours[i], c, r);

        if (r >= minTargetRadius) {
            center.push_back(c);
            radii.push_back(r);
        }

    }

    size_t center_count = center.size();
    cv::Scalar red(255,0,0);

    for( int i = 0; i < center_count; i++)
    {
        cv::circle(filtered_image, center[i], radii[i], red, 3);
    }

    namedWindow("Threshold lower image", WINDOW_AUTOSIZE);
    imshow("Threshold lower image", filtered_image);

    waitKey(0);

    return center.size();
}

void GreenRecognition::check_if_image_exist(const cv::Mat &img, const std::string &path) {
    if(img.empty()) {
        std::cout << "Error! Unable to load image: " << path << std::endl;
        std::exit(-1);
    }
}