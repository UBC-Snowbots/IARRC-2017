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

std::vector<Point> LineDetect::getLines(cv::Mat& filteredImage) {

    intVec baseHistogram = LineDetect::getHistogram(filteredImage);
    std::vector<Window> windows;

    for (int i = 0; i < baseHistogram.size(); i++) {
        if (baseHistogram[i] > initialLineDetectThreshold) {
            Window window{i, initialWindowWidth};
            windows.emplace_back(window);
        }
    }

    std::vector<std::vector<Point>> linePoints( windows.size(), std::vector<Point>(windows.size()));

    for (int verticalSliceIndex = 0; verticalSliceIndex < numVerticalSlice; verticalSliceIndex++) {
        for (int windowIndex = 0; windowIndex < windows.size(); windowIndex++) {
            Window window = windows.at(windowIndex);
            Point point{window.center, (int)verticalSliceIndex*filteredImage.rows/numVerticalSlice};
            linePoints[windowIndex].emplace_back(point);

            cv::Mat windowSlice = LineDetect::getWindowSlice(filteredImage, window, verticalSliceIndex);

            intVec windowHistogram = LineDetect::getHistogram(windowSlice);
            std::pair<int, int> peak = LineDetect::getHistogramPeak(windowHistogram);

            window.center = peak.second + (window.center - window.width/2);
        }
    }

    std::vector<Point> polyLines;

    for (std::vector<Point> points : linePoints) {
        polyLines = LineDetect::constructPolyLine(points, degree);
        // polyLines.emplace_back(polyPoint);
    }

    return polyLines;
}

cv::Mat LineDetect::getWindowSlice(cv::Mat& image, Window window, int verticalSliceIndex) {

    cv::Mat windowSlice = image(cv::Range(window.getLeftSide(), window.getRightSide()),
                                cv::Range((int)verticalSliceIndex*image.rows/numVerticalSlice,
                                          (int)(verticalSliceIndex+1)*image.rows/numVerticalSlice));

    return windowSlice;
}

std::pair<int, int> LineDetect::getHistogramPeak(intVec histogram) {

    std::pair<int, int> peak(0, 0);

    for (int i = 0; i < (histogram.size()/2); i++) {
        if (histogram[i] > peak.first)
            peak.first = histogram[i];
    }

    for (int i = histogram.size()/2; i < histogram.size(); i++) {
        if (histogram[i] > peak.second)
            peak.second = histogram[i];
    }

    return peak;
}

float interpolate(float n1, float n2, float prec) {
    return n1 + ((n2-n1) * prec);
}


std::vector<Point> LineDetect::constructPolyLine(std::vector<Point> anchors, float accuracy) {

    if (anchors.size() <= 2)
        return anchors;

    std::vector<Point> polyLines;
    polyLines.push_back(anchors[0]);

    for (float i = 0.f; i < 1.f; i += 1.f/accuracy) {
        std::vector<Point> temp;
        for (unsigned int j = 1; j < anchors.size(); ++j) {
            Point p;
            p.x = interpolate(anchors[j-1].x, anchors[j].x, i);
            p.y = interpolate(anchors[j-1].y, anchors[j].y, i);
            temp.push_back(p);
        }
        while(temp.size() > 1) {
            std::vector<Point> temp2;
            for (unsigned int j = 1; j < temp.size(); ++j) {
                Point p;
                p.x = interpolate(temp[j-1].x, temp[j].x, i);
                p.y = interpolate(temp[j-1].y, temp[j].y, i);
                temp.push_back(p);
            }
            temp = temp2;
        }

        polyLines.push_back(temp[0]);
    }

    return polyLines;

}





