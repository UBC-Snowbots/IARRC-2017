/*
 * Created by: Raad Khan
 * Created On: April 23, 2017
 * Description: Takes in an image feed and generates lane lines.
 * Usage:
 */

#include <LineDetect.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

LineDetect::LineDetect() : initialLineDetectThreshold(200),
                           white(255),
                           initialWindowWidth(10),
                           numVerticalSlice(10),
                           degree(3) {}

intVec LineDetect::getHistogram(cv::Mat& image) {

    intVec histogram(image.cols, 0);

    for (int j = 0; j < image.rows; j++) {
        for (int i = 0; i < image.cols; i++) {
            int pixelValue = (int)image.at<uchar>(i, j);
            if (pixelValue == white) {
                histogram[i]++;
            }
        }
    }

    return histogram;
}

std::vector<Polynomial> LineDetect::getLines(cv::Mat& filteredImage) {

    intVec baseHistogram = LineDetect::getHistogram(filteredImage);
    std::vector<Window> windows;

    for (int i = 0; i < baseHistogram.size(); i++) {
        if (baseHistogram[i] > initialLineDetectThreshold) {
            Window window{i, initialWindowWidth};
            windows.emplace_back(window);
        }
    }

    std::vector<std::vector<Point>> linePoints( windows.size(), std::vector<Point>(windows.size()) );

    for (int verticalSliceIndex = 0; verticalSliceIndex < numVerticalSlice; verticalSliceIndex++) {
        for (int windowIndex = 0; windowIndex < windows.size(); windowIndex++) {
            Window window = windows.at(windowIndex);
            Point point{window.center, verticalSliceIndex*filteredImage.rows/numVerticalSlice};
            linePoints[windowIndex].emplace_back(point);

            cv::Mat windowSlice = LineDetect::getWindowSlice(filteredImage, window, verticalSliceIndex);

            intVec windowHistogram = LineDetect::getHistogram(windowSlice);
            std::pair<int, int> peak = LineDetect::getHistogramPeak(windowHistogram);

            window.center = peak.second + (window.center - window.width/2);
        }
    }

    std::vector<Polynomial> polyLines;
    Polynomial polyPoints;
    for (std::vector<Point> points : linePoints) {
        polyPoints = LineDetect::fitPolyLine(points);
        polyLines.emplace_back(polyPoints);
    }
    

    return polyLines;
}

cv::Mat LineDetect::getWindowSlice(cv::Mat& image, Window window, int verticalSliceIndex) {

    cv::Mat windowSlice = image(cv::Range(window.getLeftSide(), window.getRightSide()),
                                cv::Range(verticalSliceIndex*image.rows/numVerticalSlice,
                                          (verticalSliceIndex+1)*image.rows/numVerticalSlice));

    return windowSlice;
}

std::pair<int, int> LineDetect::getHistogramPeak(intVec histogram) {

    std::pair<int, int> peak(0, 0);

    for (int i = 0; i < (histogram.size() / 2); i++) {
        if (histogram[i] > peak.first)
            peak.first = histogram[i];
    }

    for (int i = histogram.size() / 2; i < histogram.size(); i++) {
        if (histogram[i] > peak.second)
            peak.second = histogram[i];
    }

    return peak;
}

Polynomial LineDetect::fitPolyLine(std::vector<Point> points) {

}





