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
\*------------------------------------------------------------------------------------------*/

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

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>

class LineDetect {

public:

    /**
     * Constructor
     */
    LineDetect(int argc, char** argv, std::string node_name);

    /**
     * Default accumulator resolution is 1 pixel by 1 degree
     *
     * no gap, no minimum length
     */
    LineDetect() : deltaRho(1),
                   deltaTheta(M_PI/180),
                   minVote(10),
                   minLength(0.0),
                   maxGap(0.0) {}

    /**
     * Wrapper function which runs LineDetect algorithm to generate
     * the filtered image with detected lane lines
     *
     * @return filtered image matrix
     */
     cv::Mat getFilteredImage();

    /**
     * Sets the resolution of the accumulator
     *
     * @param resolution to set in polar coordinates,
     * rho and theta
     *
     */
	void setAccResolution(double dRho, double dTheta);

    /**
     * Sets the minimum number of votes
     *
     * @param minimum number of votes
     */
	void setMinVote(int minv);

    /**
     * Sets line length and line gap
     *
     * @param line length and line gap
     */
	void setLineLengthAndGap(double length, double gap);

    /**
     * Sets image shift
     *
     * @param magnitude of the image shift
     */
	void setShift(int imgShift);

    /**
     * Applies probabilistic Hough transform
     *
     * @param address of Canny image matrix
     *
     * @return lines created by probabilistic Hough transform
     */
	std::vector<cv::Vec4i> findLines(cv::Mat& binary);

    /**
     * Draws the detected lines on an image
     *
     * @param address of raw image matrix, pixel color value of the lines
     */
	void drawDetectedLines(cv::Mat &image, cv::Scalar color=cv::Scalar(255));

    /**
     * Eliminates lines that do not have an orientation equals to the ones
     * specified in the input matrix of orientations
     *
     * At least the given percentage of pixels on the line must be within
     * plus or minus delta of the corresponding orientation
     *
     * @param lines of consistent orientations
     *
     * @return address of consistent orientations matrix,
     * percentage of pixels on the line within the orientation,
     * within some upper and lower limit, delta
     */
	std::vector<cv::Vec4i> removeLinesOfInconsistentOrientations(
            const cv::Mat &orientations, double percentage, double delta);

    /**
     * Reads in the image to process from the terminal
     */
    void getVideo();

    /**
     * Gets binary map of image ROI using Canny algorithm
     */
    void getBinaryMap();

    /**
     * Increases houghVote by 25 for the next frame if we found some lines
     *
     * so we don't miss other lines that may crop up in the next frame but
     * at the same time we don't want to start the feedback loop from scratch.
     */
    void houghVoteAdjust();

    /**
     * Draws lane lines on the image ROI
     */
    void drawLines();

    /**
     * Performs a bitwise AND operation on both Hough images
     *
     * Regular Hough transform does not find endpoints and
     * probabilistic Hough transform finds endpoints but several
     * other unwanted lines, so bitwise AND to output the ideal lines
     */
    void andHoughPHough();

private:

    cv::Mat image;

    cv::Mat imgROI;

    cv::Mat contours;
    cv::Mat contoursInv;

    cv::Mat hough;
    cv::Mat houghP;

    // vector containing the end points
    // of the detected lines
    std::vector<cv::Vec4i> lines;

    // accumulator resolution parameters
    double deltaRho;
    double deltaTheta;

    // minimum number of votes that a line
    // must receive before being considered
    int minVote;

    // min length for a line
    double minLength;

    // max allowed gap along the line
    double maxGap;

    // distance to shift the drawn lines down when using a ROI
    int shift;

    bool showSteps;
};

#endif