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

LineDetect::LineDetect() : initalLineDetectThreshold(200),
                           white(255),
                           initialWindowWidth(10),
                           numVerticalSlice(10) {}

std::vector<int> LineDetect::getHistogram(cv::Mat& windowImage) {

    std::vector<int> histogram(windowImage.cols, 0);

    for (int j = 0; j < windowImage.rows; j++) {
        for (int i = 0; i < windowImage.cols; i++) {
            int pixelValue = (int)windowImage.at<uchar>(i, j);
            if (pixelValue == white) {
                histogram[i]++;
            }
        }
    }

    return histogram;
}

std::vector<Polynomial2D> getLines(cv::Mat& binaryImage) {

    std::vector<int> baseHistogram = getHistogram(binaryImage);
    std::vector<Window> windows;

    for (int i = 0; i < baseHistogram.size; i++) {
        if (baseHistogram[i] > initialLineDetectThreshold) {
            Window window{i, initialWindowWidth};
            windows.emplace_back(window);
        }
    }

    std::vector<std::vector<Point>> linePoints( windows.size, {} );

    // TODO refactor into functions
    for (int verticalSliceIndex = 0; verticalSliceIndex < numVerticalSlice; verticalSliceIndex++) {
        for (int windowIndex = 0; windowIndex < windows.size; windowIndex++) {
            Window window = windows.at(windowIndex);
            Point point{window.center, (int)verticalSliceIndex*binaryImage.rows/numVerticalSlice};
            linePoints[windowIndex].emplace_back(point);
            // TODO replace with getWindowMat
            cv::Mat windowMat = binaryImage(cv::Range( window.center - window.width/2, window.center + window.width/2),
                                              cv::Range((int)verticalSliceIndex*binaryImage.rows/numVerticalSlice,
                                                        (int)(verticalSliceIndex+1)*binaryImage.rows/numVerticalSlice));

            std::vector<int> windowHistogram = getHistogram(windowMat);
            std::pair<int, int> peak = getHistogramPeak(windowHistogram);

            window.center = peak.second + (window.center - window.width/2);

        }
    }

    std::vector<Polynomial2D> polyLines;
    for (std::vector<Point> points : linePoints) {
        Polynomial2D polyPoints = constructPolyLine(points);
        polyLines.emplace_back(polyPoints);
    }

    return polyLines;
}






