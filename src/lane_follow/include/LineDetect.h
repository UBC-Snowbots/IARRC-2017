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
 * Description: Analyzes an image and detects lane lines, following them accordingly.
 * Usage:
 * Subscribes to:
 * Publishes to:
 */

#ifndef LANE_FOLLOW_LINEDETECT_H
#define LANE_FOLLOW_LINEDETECT_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <string>

class LineDetect {

public:

    LineDetect(int argc, char** argv, std::string node_name);
    /**
     * Constructor
     */
    LineDetect();
    /**
     * Default accumulator resolution is 1 pixel by 1 degree
     *
     * no gap, no minimum length
     */
	LineFinder() : deltaRho(1),
                   deltaTheta(PI/180),
                   minVote(10),
                   minLength(0.0),
                   maxGap(0.0) {}
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
	// Set the minimum number of votes
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
     * Applies probabilistic hough transform
     *
     * @param
     *
     * @return
     */
	std::vector<cv::Vec4i> findLines(cv::Mat& binary);
    /**
     * Draws the detected lines on an image
     *
     * @param
     */
	void drawDetectedLines(cv::Mat &image, cv::Scalar color=cv::Scalar(255));
    /**
     * Eliminates lines that do not have an orientation equals to the ones
     * specified in the input matrix of orientations
     *
     * At least the given percentage of pixels on the line must be within
     * plus or minus delta of the corresponding orientation
     *
     * @param
     *
     * @return
     */
	std::vector<cv::Vec4i> removeLinesOfInconsistentOrientations(
            const cv::Mat &orientations, double percentage, double delta);
    /**
     * Reads in the image to process from the terminal
     */
    void displayVideo();
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
     * Performs a bitwise AND operation on both hough images
     *
     * regular hough transform does not find endpoints and
     * probabilistic hough transform finds endpoints but several
     * other unwanted lines, so bitwise AND to output the ideal lines
     */
    void houghImageAndPhoughImage();

private:

    // original image
    cv::Mat img;

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