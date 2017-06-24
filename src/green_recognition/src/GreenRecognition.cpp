/*
 * Created By: Robyn Castro
 * Created On: June 17, 2017
 * Description: Determines whether or not green is seen on
 *              the screen then publishes a recommended twist
 *              message.
 */

#include <GreenRecognition.h>
#include <image_transport/image_transport.h>

using namespace cv;
using namespace std;
using namespace cv_bridge;

GreenRecognition::GreenRecognition(std::string &image_path) {

    cv::Mat bgr_image = imread(image_path);

    // Check if the image can be loaded
    check_if_image_exist(bgr_image, image_path);

    minTargetRadius = 20;
    countObjects(bgr_image);

    waitKey(0);

}

GreenRecognition::GreenRecognition() {
    minTargetRadius = 20;
}

GreenRecognition::GreenRecognition(int argc, char **argv, std::string node_name) {

    // Setup handles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup image transport
    image_transport::ImageTransport it(nh);

    // Setup subscriber
    std::string image_topic = "/robot/vision/filtered_image";
    int refresh_rate = 10;
    image_sub = it.subscribe<GreenRecognition>(image_topic, refresh_rate,
                                               &GreenRecognition::subscriberCallBack, this);

    // Setup publishers
    std::string twist_topic = private_nh.resolveName("command");
    uint32_t queue_size = 1;
    twist_pub = private_nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);

    // Get some params
    SB_getParam(private_nh, "minimum_target_radius", minTargetRadius, 50);
    SB_getParam(private_nh, "show_image_window", showWindow, true);

}

void GreenRecognition::subscriberCallBack(const sensor_msgs::Image::ConstPtr &image) {

    geometry_msgs::Twist twist_message;

    // Make sure all values inside twist_message are 0
    twist_message.angular.x = 0;
    twist_message.angular.y = 0;
    twist_message.angular.z = 0;
    twist_message.linear.x = 0;
    twist_message.linear.y = 0;
    twist_message.linear.z = 0;

    // If something is seen tell the robot to move
    int numObjects = countObjects(rosToMat(image));
    if (numObjects > 0)
        twist_message.linear.x = 1;

    twist_pub.publish(twist_message);
}

Mat GreenRecognition::rosToMat(const sensor_msgs::Image::ConstPtr &image) {
    CvImagePtr imagePtr;
    imagePtr = toCvCopy(image, image->encoding);
    return imagePtr->image;
}

int GreenRecognition::countObjects(const Mat &filtered_image) {

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<cv::Point2i> center;
    vector<int> radii;

    // Convert grayscale image to black and white image
    cv::Mat bwImage;
    try {
        cv::cvtColor(filtered_image, bwImage, CV_RGB2GRAY);
    } catch(cv::Exception) {
        bwImage = filtered_image;
    }
    cv::findContours(bwImage.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    size_t count = contours.size();

    for (int i = 0; i < count; i++) {
        cv::Point2f c;
        float r;
        cv::minEnclosingCircle((Mat) contours[i], c, r);

        // Only count circles with large enough radius
        if (r >= minTargetRadius) {
            center.push_back(c);
            radii.push_back(r);
        }
    }

    // Displays a window with the detected objects being circled
    showFilteredObjectsWindow(filtered_image, center, radii);

    return center.size();
}

void GreenRecognition::showFilteredObjectsWindow(const Mat &filtered_image, std::vector<cv::Point2i> center,
                                                 std::vector<int> radii) {
    size_t center_count = center.size();
    cv::Scalar green(0, 255, 0);

    // Draw green circles around object
    for (int i = 0; i < center_count; i++) {
        cv::circle(filtered_image, center[i], radii[i], green, 3);
    }

    namedWindow("Filtered Objects", WINDOW_AUTOSIZE);
    imshow("Filtered Objects", filtered_image);

}

void GreenRecognition::check_if_image_exist(const cv::Mat &img, const std::string &path) {
    if (img.empty()) {
        std::cout << "Error! Unable to load image: " << path << std::endl;
        std::exit(-1);
    }
}