/*
 * Created by: Raad Khan
 * Created On: April 23, 2017
 * Description: Takes in an image feed and generates lane lines.
 * Usage:
 * Subscribes to:
 * Publishes to:
 */

#ifndef LANE_FOLLOW_LINEDETECT_H
#define LANE_FOLLOW_LINEDETECT_H

#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>

struct Polynomial2D {
    int d1;
    int d2;
    int d3;
};

struct Point {
    int x;
    int y;
};

// TODO refactor this to a class with functions for getting the left and right sides
struct Window {
    int center;
    int width;
};

class LineDetect {

public:
    /**
     * Constructor
     */
    LineDetect();

    // TODO doc functions

    std::vector<Polynomial2D> getLines(cv::Mat& binaryImage);

    std::vector<int> getHistogram(cv::Mat& windowImage);

    // TODO typedef histogram

    std::pair<int, int> getHistogramPeak(std::vector<int> histogram);

    Polynomial2D constructPolyLine(std::vector<Point> points);

    cv::Mat& getWindowMat(Window window, int verticalSliceIndex);

private:

    int white;

    int initialLineDetectThreshold;

    int initialWindowWidth;

    int numVerticalSlice;

};

#endif
