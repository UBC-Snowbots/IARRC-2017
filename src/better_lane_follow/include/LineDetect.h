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
};

struct Point {
    int x;
    int y;
};

typedef std::vector<int> intVec;

class Window {

public:
    int center;
    int width;

    int getLeftSide() {
        return (center - window.width/2);
    }
    int getRightSide() {
        return (center + window.width/2);
    }

};

class LineDetect {

public:
    /**
     * Constructor
     */
    LineDetect();

    // TODO doc functions

    std::vector<Polynomial2D> getLines(cv::Mat& filteredImage);

    std::vector<int> getHistogram(cv::Mat& image);

    std::pair<int, int> getHistogramPeak(std::vector<int> histogram);

    std::vector<Polynomial2D> constructPolyLine(std::vector<Point> anchors, double accuracy);

    cv::Mat& getWindowSlice(cv::Mat& image, Window window, int verticalSliceIndex);

private:
    int white;

    int initialLineDetectThreshold;

    int initialWindowWidth;

    int numVerticalSlice;

    float degree;

};

#endif
