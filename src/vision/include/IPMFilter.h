/*
 * Created by: Robyn Castro
 * Created On: July 1, 2017
 * Description: Takes in an image applies inverse perspective mapping to it
 *
 */

#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

class IPMFilter {

public:

    /**
     * Initializes the corners of the filter
     */
    IPMFilter(float ipm_base_width, float ipm_top_width,
              float ipm_base_displacement, float ipm_top_displacement,
              float image_height, float image_width);

    /**
     * Filters an image according to ipm
     *
     * @param input the frame being filtered
     * @param output the output image
     */
    void filterImage(const cv::Mat &input, cv::Mat &output);
private:

    /**
     * Initializator
     *
     * @params the appropriate HSV ranges
     */
    void createFilter(float ipm_base_width, float ipm_top_width,
                      float ipm_base_displacement, float ipm_top_displacement
                      float image_height, float image_width);

    // Corners of the portion of the image to be filtered
    int x1, y1;
    int x2, y2;
    int x3, y3;
    int x4, y4;



    //Filters and their variables
    IPM ipm;
    std::vector<cv::Point2f> orig_points;
    std::vector<cv::Point2f> dst_points;

};