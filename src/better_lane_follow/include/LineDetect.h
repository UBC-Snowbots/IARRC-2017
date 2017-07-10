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

struct Polynomial {
    double a;
    double b;
    double c;
    double d;
};

struct Point {
    double x;
    double y;
};

typedef std::vector<int> intVec;

class Window {

public:
    int center;
    int width;

    int getLeftSide() {
        return (center - width/2);
    }
    int getRightSide() {
        return (center + width/2);
    }

};

class LineDetect {

public:
    /**
     * Constructor
     */
    LineDetect();

    // TODO doc functions

    std::vector<Polynomial> getLines(cv::Mat& filteredImage);

    intVec getHistogram(cv::Mat& image);

    cv::Mat getWindowSlice(cv::Mat& image, Window window, int verticalSliceIndex);

    std::pair<int, int> getHistogramPeakPosition(intVec histogram);

    Polynomial fitPolyLine(std::vector<Point> points, int order);

    Point getIntersection(Polynomial leftLine, Polynomial rightLine);
private:
    int white;

    int initialLineDetectThreshold;

    int windowWidth;

    int numVerticalSlice;

    int degree;
};

#endif
