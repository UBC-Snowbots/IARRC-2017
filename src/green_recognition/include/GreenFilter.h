/*
 * Created By: Robyn Castro
 * Created On: June 15, 2017
 * Description: Anything green is mapped to white, everything
 *              else is mapped to black.
 *
 * Usage:
 * Subscribes to: camera // TODO: Determine topic names
 * Publishes to:
 */

#ifndef GREEN_FILTER_H
#define GREEN_FILTER_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>

using namespace cv;

class GreenFilter {

public:

    /**
     * Empty Constructor for testing purposes
     */
    GreenFilter();

    /**
     * Constructor
     */
    GreenFilter(int argc, char **argv, std::string node_name);

private:

    /**
     * Callback for the filtered image
     *
     * @param address of filtered image matrix
     */
    void subscriberCallBack(const Mat &img);

    /**
     * Filters the green out of the image
     *
     * @param raw_image
     */
    Mat filterImage(const Mat &raw_image);

    void check_if_image_exist(const cv::Mat &img, const std::string &path);

//    /**
//     * Subscribes to the raw camera image node
//     */
//    ros::Subscriber image_sub;
//
//    /**
//     * Publishes the filtered image
//     */
//    ros::Publisher filter_pub;



};

#endif