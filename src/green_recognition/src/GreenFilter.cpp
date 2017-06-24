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

    // Setup image transport
    image_transport::ImageTransport it(nh);

    // Setup subscriber
    int refresh_rate = 10;
    image_sub = it.subscribe<GreenFilter>(image_topic, refresh_rate,
                                          &GreenFilter::subscriberCallBack, this);
    // Setup publisher
    uint32_t queue_size = 1;
    filter_pub = it.advertise(output_topic, queue_size);

    // Get some params (not all though, we wait until we have an image to get the rest)
    SB_getParam(private_nh, "update_frequency", frequency, 5.0);
    SB_getParam(private_nh, "config_file", mfilter_file,
                ros::package::getPath("green_recognition") + "/launch/filter_init.txt");
    SB_getParam(private_nh, "show_image_window", showWindow, true);
    SB_getParam(private_nh, "show_calibration_window", isCalibratingManually, false);

    setUpFilter();
}

void GreenFilter::subscriberCallBack(const sensor_msgs::Image::ConstPtr &image) {

    // Filter out non-green colors
    Mat filteredImage;
    imageInput = rosToMat(image);
    filter.filterImage(imageInput, filteredImage);
    filterOutput = filteredImage;

    if (!receivedFirstImage) {
        ROS_INFO("First image received!");
        ros::NodeHandle private_nh("~");
        SB_getParam(private_nh, "image_width", image_width, (int) image->width);
        SB_getParam(private_nh, "image_height", image_height, (int) image->height);
        receivedFirstImage = true;
    }

    // If enough time has passed update filter and show image
    if ((ros::Time::now() - last_published) > publish_interval) {
        last_published = ros::Time::now();
        if (showWindow)
            showRawAndFilteredImageWindow();
        updateFilter();
    }

    // Outputs the image
    sensor_msgs::ImagePtr output_message = cv_bridge::CvImage(std_msgs::Header(), "mono8",
                                                              filteredImage).toImageMsg();
    // Publish recommended Twist message
    filter_pub.publish(output_message);
}

Mat GreenFilter::rosToMat(const sensor_msgs::Image::ConstPtr &image) {
    CvImagePtr imagePtr;
    imagePtr = toCvCopy(image, image->encoding);
    return imagePtr->image;
}

void GreenFilter::setUpFilter() {

    //Sets up filter update frequency
    publish_interval = ros::Duration(1 / frequency);
    last_published = ros::Time::now();

    // Check for filter initialization file
    ROS_INFO("Looking for filter file at: %s", mfilter_file.c_str());
    fstream filter_file(mfilter_file, ios::in);
    string line;
    bool filter_set = false;
    if (filter_file.is_open()) {
        if (getline(filter_file, line)) {
            istringstream iss(line);
            int lh, hh, ls, hs, lv, hv;
            if (iss >> lh >> hh >> ls >> hs >> lv >> hv) {
                ROS_INFO("Filter file found");
                ROS_INFO("Filter initializing with: %d %d %d %d %d %d", lh, hh, ls, hs, lv, hv);
                filter = snowbotsFilter(lh, hh, ls, hs, lv, hv);
                filter_set = true;
            }
        }
    }
    if (!filter_set) {
        ROS_INFO("Filter file not found");
        ROS_INFO("Filter initialized with default values");
        filter = snowbotsFilter(0, 155, 0, 155, 150, 255);
    }
    filter_file.close();

    ROS_WARN("Waiting for first image");
}

void GreenFilter::updateFilter() {

    // Color filter calibration
    if (isCalibratingManually)
        filter.manualCalibration();

    int a = waitKey(20);
    // Press 'm' to calibrate manually, press m again to save
    if (a == 109) {
        if (!isCalibratingManually) {
            ROS_INFO("Beginning manual calibration");
        } else {
            ROS_INFO("Ending manual calibration");
            ROS_INFO("Saving filter state in %s", mfilter_file.c_str());
            fstream filter_file(mfilter_file, ios::trunc | ios::out);
            filter_file << filter.getValues();
            cout << filter.getValues();
            filter_file.close();
            filter.stopManualCalibration();
        }
        isCalibratingManually = !isCalibratingManually;
    }// Press 's' to show/unshow window
    else if (a == 115) {
        showWindow = !showWindow;
        if (!showWindow) destroyWindow(displayWindowName);
    }
}

void GreenFilter::showRawAndFilteredImageWindow() {
    // Create one big mat for all our images
    cv::Size main_window_size(image_width * 2, image_height * 2);
    cv::Size sub_window_size(main_window_size.width / 2, main_window_size.height / 2);
    cv::Mat main_image(main_window_size, CV_8UC3);

    // Copy all our images to the big Mat we just created (in 4 sub-windows)

    // Image 1
    cv::Mat image1Roi(main_image, cv::Rect(0, 0, sub_window_size.width, sub_window_size.height));
    resize(imageInput, image1Roi, sub_window_size);

    // Image 2
    cv::Mat image2Roi(main_image, cv::Rect(sub_window_size.width, 0, sub_window_size.width, sub_window_size.height));
    resize(filterOutput, image2Roi, sub_window_size);

    // Image 3 & 4: We only have 2 images, so just make the last two all black to stop flickering
    cv::Mat allBlack(sub_window_size, CV_8UC3, 3);

    // Image 3
    cv::Mat image3Roi(main_image, cv::Rect(0, sub_window_size.height, sub_window_size.width, sub_window_size.height));
    resize(allBlack, image3Roi, sub_window_size);

    // Image 4
    cv::Mat image4Roi(main_image, cv::Rect(sub_window_size.width, sub_window_size.height, sub_window_size.width,
                                           sub_window_size.height));
    resize(allBlack, image4Roi, sub_window_size);
}

void GreenFilter::check_if_image_exist(const cv::Mat &img, const std::string &path) {
    if (img.empty()) {
        std::cout << "Error! Unable to load image: " << path << std::endl;
        std::exit(-1);
    }
}