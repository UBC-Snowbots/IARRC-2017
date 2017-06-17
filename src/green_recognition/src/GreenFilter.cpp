/*
 * Created By: Robyn Castro
 * Created On: June 17, 2017
 * Description: Filters out everything but green.
 *
 */

#include <GreenFilter.h>

using namespace cv;
using namespace std;
using namespace cv_bridge;

GreenFilter::GreenFilter(int argc, char **argv, std::string node_name) {

    displayWindowName = "Snowbots - GreenFilter";
    receivedFirstImage = false;

    //ROS
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Set topics
    std::string image_topic = "/robot/vision/raw_image";
    std::string output_topic = "/robot/vision/filtered_image";
    ROS_INFO("Image (Subscribe) Topic: %s", image_topic.c_str());
    ROS_INFO("Output (Publish) Topic: %s", output_topic.c_str());

    // Setup subscriber
    int refresh_rate = 10;
    image_sub = nh.subscribe(image_topic, refresh_rate,
                                             &GreenFilter::subscriberCallBack, this);

    // Setup publishers
    std::string twist_topic = "/robot/lane_follow/twist_message"; // TODO: Decide on topic names
    uint32_t queue_size = 1;

    filter_pub = private_nh.advertise<sensor_msgs::Image>(output_topic, queue_size);

    // Get some params (not all though, we wait until we have an image to get IPM ones)
    SB_getParam(private_nh, "update_frequency", frequency, 5.0);
    SB_getParam(private_nh, "config_file", mfilter_file, ros::package::getPath("green_recognition") + "/launch/filter_init.txt");
    SB_getParam(private_nh, "show_image_window", showWindow, true);
    SB_getParam(private_nh, "show_calibration_window", isCalibratingManually, false);

    setUpFilter();
}

void GreenFilter::subscriberCallBack(const sensor_msgs::Image::ConstPtr& image) {


    // Filter out non-green colors
    Mat filteredImage;
    filter.filterImage(rosToMat(image), filteredImage);

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

void GreenFilter::setUpFilter() {

    //Sets up filter update frequency
    publish_interval = ros::Duration(1 / frequency);
    last_published = ros::Time::now();

    //Check for filter initialization file
    ROS_INFO("Looking for filter file at: %s", mfilter_file.c_str());
    fstream filter_file(mfilter_file, ios::in);
    string line;
    bool filter_set = false;
    if (filter_file.is_open()){
        if (getline(filter_file, line)){
            istringstream iss(line);
            int lh, hh, ls, hs, lv, hv;
            if (iss >> lh >> hh >> ls >> hs >> lv >> hv) {
                ROS_INFO("Filter file found");
                ROS_INFO("Filter initializing with: %d %d %d %d %d %d", lh, hh, ls, hs, lv ,hv);
                filter = snowbotsFilter(lh, hh, ls, hs, lv, hv);
                filter_set = true;
            }
        }
    }
    if (!filter_set){
        ROS_INFO("Filter file not found");
        ROS_INFO("Filter initialized with default values");
        filter = snowbotsFilter(0, 155, 0, 155, 150, 255);
    }
    filter_file.close();

    ROS_WARN("Waiting for first image");
}

void GreenFilter::check_if_image_exist(const cv::Mat &img, const std::string &path) {
    if(img.empty()) {
        std::cout << "Error! Unable to load image: " << path << std::endl;
        std::exit(-1);
    }
}