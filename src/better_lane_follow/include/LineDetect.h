/*
 * Created by: Raad Khan
 * Created On: July 1, 2017
 * Description: Takes in an image feed and generates lane lines.
 * Usage: LaneFollow node instantiates this class to generate lane lines
 */

#ifndef LANE_FOLLOW_LINEDETECT_H
#define LANE_FOLLOW_LINEDETECT_H

#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>

using namespace cv;

struct Polynomial {
    // ax^3 + bx^2 + cx + d
    double a;
    double b;
    double c;
    double d;
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

    std::vector<Polynomial> getLines(Mat& filteredImage);

    intVec getHistogram(Mat& image);

    Mat getWindowSlice(Mat& image, Window window, int verticalSliceIndex);

    std::pair<int, int> getHistogramPeakPosition(intVec histogram);

    Polynomial fitPolyLine(std::vector<Point2d> points, int order);

    static Point2d getIntersection(Polynomial leftLine, Polynomial rightLine);

    static Point2d moveAwayFromLine(Polynomial line, double targetXDistance, double targetYDistance);

    static double getAngleFromOriginToPoint(Point2d point);

    static double cubicFormula(double a, double b, double c, double d);

private:
    int white;

    int initialLineDetectThreshold;

    int windowWidth;

    int numVerticalSlice;

    int degree;
};

#endif