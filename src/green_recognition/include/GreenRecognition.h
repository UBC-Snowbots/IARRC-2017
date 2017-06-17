/*
 * Created By: Robyn Castro
 * Created On: June 15, 2017
 * Description: Determines whether or not green is seen on
 *              the screen then publishes a recommended twist
 *              message.
 * Usage:
 * Subscribes to: camera // TODO: Determine topic names
 * Publishes to:
 */

#ifndef GREEN_RECOGNITION_H
#define GREEN_RECOGNITION_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>

class GreenRecognition {

public:

    /**
     * Constructor
     */
    GreenRecognition(int argc, char **argv, std::string node_name);

private:

    /**
     * Callback for the filtered image
     *
     * @param address of filtered image matrix
     */
    void subscriberCallBack(const Mat &img);

    /**
     * Subscribes to the filtered camera image node
     */
    ros::Subscriber image_sub;

    /**
     * Publishes the recommended twist message
     */
    ros::Publisher twist_pub;

};

#endif