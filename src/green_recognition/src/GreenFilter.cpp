/*
 * Created By: Robyn Castro
 * Created On: June 17, 2017
 * Description: Filters out everything but green.
 *
 */

//#include <ros/ros.h>
#include <GreenFilter.h>

using namespace cv;
using namespace cv_bridge;
GreenFilter::GreenFilter() {

    std::string path_image = "/home/robyncastro/IARRC-2017/src/green_recognition/test/images/moreCircles.jpg";
    cv::Mat bgr_image = imread(path_image);

    // Check if the image can be loaded
    check_if_image_exist(bgr_image, path_image);

    filterImage(bgr_image);
}

GreenFilter::GreenFilter(int argc, char **argv, std::string node_name) {

    // Setup handles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup subscriber
    std::string image_topic = "/robot/line_detect/camera_image"; // TODO: Get topic name
    int refresh_rate = 10;
    image_sub = nh.subscribe(image_topic, refresh_rate,
                                             &GreenFilter::subscriberCallBack, this);

    // Setup publishers
    std::string twist_topic = "/robot/lane_follow/twist_message"; // TODO: Decide on topic names
    uint32_t queue_size = 1;

    filter_pub = private_nh.advertise<sensor_msgs::Image>(twist_topic, queue_size);

}

void GreenFilter::subscriberCallBack(const sensor_msgs::Image::ConstPtr& image) {


    // Filter out non-green colors
    Mat filteredImage = filterImage(rosToMat(image));

    cv_bridge::CvImage out_msg;
    out_msg.header   = image->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
    out_msg.image    = filteredImage; // Your cv::Mat

    // Publish recommended Twist message
    filter_pub.publish(out_msg.toImageMsg());
}

Mat GreenFilter::rosToMat(const sensor_msgs::Image::ConstPtr& image) {
    CvImagePtr imagePtr;
    imagePtr = toCvCopy(image, image->encoding);
    return imagePtr->image;
}

Mat GreenFilter::filterImage(const Mat &raw_image) {

    Mat orig_image = raw_image.clone();

    medianBlur(raw_image, raw_image, 3);

    // Convert input image to HSV
    Mat hsv_image;
    cvtColor(raw_image, hsv_image, COLOR_BGR2HSV);

    int sensitivity = 15;

    // Threshold the HSV image, keep only the red pixels
    Mat green_hue_range;
    inRange(hsv_image, Scalar(60 - sensitivity, 100, 100), Scalar(60 + sensitivity, 255, 255), green_hue_range);

    // Show images
    namedWindow("Threshold lower image", WINDOW_AUTOSIZE);
    imshow("Original Image", orig_image);
    imshow("Threshold lower image", green_hue_range);

    waitKey(0);


    return green_hue_range;
}

void GreenFilter::check_if_image_exist(const cv::Mat &img, const std::string &path) {
    if(img.empty()) {
        std::cout << "Error! Unable to load image: " << path << std::endl;
        std::exit(-1);
    }
}