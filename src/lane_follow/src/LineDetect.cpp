/*------------------------------------------------------------------------------------------*\
Lane Detection

General idea and some code modified from:
chapter 7 of Computer Vision Programming using the OpenCV Library.
by Robert Laganiere, Packt Publishing, 2011.

This program is free software; permission is hereby granted to use, copy, modify,
and distribute this source code, or portions thereof, for any purpose, without fee,
subject to the restriction that the copyright notice may not be removed
or altered from any source or altered source distribution.
The software is released on an as-is basis and without any warranties of any kind.
In particular, the software is not guaranteed to be fault-tolerant or free from failure.
The author disclaims all warranties with regard to this software, any use,
and any consequent failure, is purely the responsibility of the user.

Copyright (C) 2013 Jason Dorweiler, www.transistor.io

Notes:

Add up number on lines that are found within a threshold of a given rho, theta and
use that to determine a score.  Only lines with a good enough score are kept.

Calculation for the distance of the car from the center.  This should also determine
if the road in turning.  We might not want to be in the center of the road for a turn.
	
Several other parameters can be played with: min vote on houghp, line distance and gap.  Some
type of feed back loop might be good to self tune these parameters.

We are still finding the Road, i.e. both left and right lanes.  we Need to set it up to find the
yellow divider line in the middle.

Added filter on theta angle to reduce horizontal and vertical lines.

Added image ROI to reduce false lines from things like trees/power lines
\*------------------------------------------------------------------------------------------*/

/*
 * Created by: Raad Khan
 * Created On: April 23, 2017
 * Description: Takes in an image feed and generates lane lines.
 * Usage:
 * Subscribes to:
 * Publishes to:
 */

#include <LineDetect.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

using namespace cv;

void LineDetect::setAccResolution(double dRho, double dTheta) {

    deltaRho = dRho;
    deltaTheta = dTheta;
}

void LineDetect::setMinVote(int minv) {

    minVote = minv;
}

void LineDetect::setLineLengthAndGap(double length, double gap) {

    minLength = length;
    maxGap = gap;
}

void LineDetect::setShift(int imgShift) {

    shift = imgShift;
}

std::vector<cv::Vec4i> LineDetect::findLines(cv::Mat& image) {

    std::vector<cv::Vec4i> lines;

    // namedWindow("Raw Image");
    // imshow("Raw Image", image);

    cv::Mat imageROI = getGrayROI(image);
    cv::Mat imageContours = getBinaryMap(imageROI);

    setAccResolution(1, CV_PI/180);
    setMinVote(80);
    setLineLengthAndGap(30, 10);

    cv::HoughLinesP(imageContours, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);

    // Display the detected line image
    if (showSteps) {
        Mat imageLines(imageContours.size(), CV_8U, Scalar(0));
        drawDetectedLines(imageLines, cv::Scalar(255), lines);

        namedWindow("Lines");
        imshow("Lines", imageLines);
    }

    if (showSteps)
        waitKey(10);

    return lines;
}

void LineDetect::drawDetectedLines(cv::Mat& image, cv::Scalar color, std::vector<cv::Vec4i> lines) {
    // Draw the lines
    auto it = lines.begin();

    while (it != lines.end()) {

        cv::Point pt1((*it)[0], (*it)[1] + shift);
        cv::Point pt2((*it)[2], (*it)[3] + shift);

        cv::line(image, pt1, pt2, color, 6);
        std::cout << "HoughP line: ("<< pt1 <<"," << pt2 << ")\n";
        ++it;
    }
}

/**
std::vector<cv::Vec4i> LineDetect::removeLinesOfInconsistentOrientations(
        const cv::Mat &orientations, double percentage, double delta) {

    std::vector<cv::Vec4i>::iterator it = lines.begin();

    // check all lines
    while (it != lines.end()) {

        // end points
        int x1 = (*it)[0];
        int y1 = (*it)[1];
        int x2 = (*it)[2];
        int y2 = (*it)[3];

        // line orientation + 90 degrees to get the parallel line
        double orientation1 = atan2(static_cast<double>(y1 - y2), static_cast<double>(x1 - x2)) + M_PI/2;
        if (orientation1 > M_PI) orientation1 = orientation1 - 2*M_PI;

        double orientation2 = atan2(static_cast<double>(y2 - y1), static_cast<double>(x2 - x1)) + M_PI/2;
        if (orientation2 > M_PI) orientation2 = orientation2 - 2*M_PI;

        // for all points on the line
        cv::LineIterator lit(orientations, cv::Point(x1, y1), cv::Point(x2, y2));
        int i, count = 0;
        for (i = 0, count = 0; i < lit.count; i++, ++lit) {

            float orientation = *(reinterpret_cast<float*>(*lit));

            // is line orientation similar to gradient orientation?
            if (std::min(fabs(orientation - orientation1), fabs(orientation - orientation2)) < delta)
                count++;
        }

        double consistency = count / static_cast<double>(i);

        // set to zero lines of inconsistent orientation
        if (consistency < percentage)
            (*it)[0] = (*it)[1] = (*it)[2] = (*it)[3] = 0;

        ++it;
    }

    return lines;
}
**/

cv::Mat LineDetect::getGrayROI(cv::Mat& image) {
    // set the ROI for the image
    Mat gray;
    cvtColor(image, gray, CV_RGB2GRAY);
    Rect roi(0, image.cols/3, image.cols-1, image.rows - image.cols/3);
    Mat ROI = image(roi);

    // display the image
    if (showSteps) {
        namedWindow("ROI Image");
        imshow("ROI Image", ROI);
    }

    return ROI;
}

cv::Mat LineDetect::getBinaryMap(cv::Mat& imageROI) {
    // Canny algorithm
    cv::Mat contours;
    Canny(imageROI, contours, 50, 250, 3);
    cv::Mat contoursInv;
    threshold(contours, contoursInv, 128, 255, THRESH_BINARY_INV);

    houghVoteAdjust(contours, imageROI);

    // display Canny image
    if (showSteps) {
        namedWindow("Contours");
        imshow("Contours", contours);

        namedWindow("Contours Inverse");
        imshow("Contours Inverse", contoursInv);
    }

    return contours;
}

void LineDetect::houghVoteAdjust(cv::Mat& contours, cv::Mat& imageROI) {
    std::vector<Vec2f> lines;

    // all lines are lost so reset
    if (houghVote < 1 or lines.size() > 2)
        houghVote = 200;
    else
        houghVote += 25;

    while (lines.size() < 5 && houghVote > 0) {
        HoughLines(contours, lines, 1, M_PI / 180, houghVote);
        houghVote -= 5;
    }

    std::cout << houghVote << "\n";
    result = Mat(imageROI.size(), CV_8U, Scalar(255));
    imageROI.copyTo(result);

}

cv::Mat LineDetect::drawLines(std::vector<cv::Vec4i> lines, cv::Mat& imageContours) {
    // Draw the lines
    auto it = lines.begin();
    Mat hough(imageContours.size(), CV_8U, Scalar(0));
    while (it != lines.end()) {
        // first element is distance rho
        float rho = (*it)[0];
        // second element is angle theta
        float theta = (*it)[1];

        // filter to remove vertical and horizontal lines
        if (theta > 0.09 && theta < 1.48 || theta < 3.14 && theta > 1.66) {
            // point of intersection of the line with first row
            Point pt1(rho / cos(theta), 0);
            // point of intersection of the line with last row
            Point pt2((rho - result.rows * sin(theta)) / cos(theta), result.rows);
            // draw a white line
            line(result, pt1, pt2, Scalar(255), 8);
            line(hough, pt1, pt2, Scalar(255), 8);
        }

        std::cout << "line: (" << rho << "," << theta << ")\n";
        ++it;
    }

    return result;
}