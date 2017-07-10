/*
 * Created by: Raad Khan
 * Created On: April 23, 2017
 * Description: Takes in an image feed and generates lane lines.
 * Usage:
 */

#include <LineDetect.h>
#include <Eigen/QR>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

LineDetect::LineDetect() : initialLineDetectThreshold(200),
                           white(255),
                           windowWidth(10),
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
            Window window{i, windowWidth};
            windows.emplace_back(window);
        }
    }

    std::vector<std::vector<Point>> linePoints( windows.size(), std::vector<Point>(numVerticalSlice) );

    for (int verticalSliceIndex = 0; verticalSliceIndex < numVerticalSlice; verticalSliceIndex++) {
        for (int windowIndex = 0; windowIndex < windows.size(); windowIndex++) {
            Window window = windows.at(windowIndex);
            Point point{(double)window.center, (double)(verticalSliceIndex*filteredImage.rows/numVerticalSlice)};
            linePoints[windowIndex].emplace_back(point);

            cv::Mat windowSlice = LineDetect::getWindowSlice(filteredImage, window, verticalSliceIndex);
            intVec windowHistogram = LineDetect::getHistogram(windowSlice);
            std::pair<int, int> peak = LineDetect::getHistogramPeakPosition(windowHistogram);

            window.center = windowIndex ? window.center = peak.second : peak.first /*+ (window.center - window.width/2)*/;
        }
    }

    std::vector<Polynomial> polyLines;
    Polynomial polyPoints;

    for (std::vector<Point> points : linePoints) {
        polyPoints = LineDetect::fitPolyLine(points, degree);
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

std::pair<int, int> LineDetect::getHistogramPeakPosition(intVec histogram) {

    std::pair<int, int> peak(0, 0);
    int peakValue = 0;

    for (int i = 0; i < (int)(histogram.size()/2.0); i++) {
        if (histogram[i] > peakValue) {
            peakValue = histogram[i];
            peak.first = i;
        }
    }

    peakValue = 0;

    for (int i = (int)(histogram.size()/2.0); i < histogram.size(); i++) {
        if (histogram[i] > peakValue) {
            peakValue = histogram[i];
            peak.second = i;
        }
    }

    return peak;
}

Polynomial LineDetect::fitPolyLine(std::vector<Point> points, int order) {

    int moreOrder = order + 1;
    assert(points.size() >= moreOrder);

    std::vector<double> xv(points.size(), 0);
    std::vector<double> yv(points.size(), 0);

    for (size_t i = 0; i < points.size(); i++) {
        xv[i] = points[i].x;
        yv[i] = points[i].y;
    }

    Eigen::MatrixXd A(xv.size(), moreOrder);
    Eigen::VectorXd yvMapped = Eigen::VectorXd::Map(&yv.front(), yv.size());
    Eigen::VectorXd result;

    // create matrix
    for (size_t i = 0; i < points.size(); i++)
        for (size_t j = 0; j < moreOrder; j++)
            A(i, j) = pow((xv.at(i)), j);

    // solve for linear least squares fit
    result = A.householderQr().solve(yvMapped);

    return Polynomial{result[0], result[1], result[2], result[3]};
}





