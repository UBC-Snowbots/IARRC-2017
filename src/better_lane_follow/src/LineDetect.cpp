/*
 * Created by: Raad Khan
 * Created On: July 1, 2017
 * Description: Takes in an image feed and generates lane lines.
 * Usage: LaneFollow node instantiates this class to generate lane lines
 */

#include <LineDetect.h>
#include <Eigen/QR>
#include <opencv2/objdetect/objdetect.hpp>

using namespace cv;

LineDetect::LineDetect() : white(250),
                           numVerticalSlice(10),
                           degree(1) {}

std::vector <Polynomial> LineDetect::getLaneLines(cv::Mat &filteredImage) {

    windowWidth = filteredImage.cols / 4;

    intVec baseHistogram = this->getHistogram(filteredImage);
    std::pair<int, int> peak = this->getBaseHistogramPeakPosition(baseHistogram);

    std::vector <Window> baseWindows;
    Window windowLeft{peak.first, windowWidth};
    Window windowRight{peak.second, windowWidth};
    baseWindows.emplace_back(windowLeft);
    baseWindows.emplace_back(windowRight);

    std::vector <std::vector<cv::Point2d>> lanePoints = this->getLanePoints(filteredImage, baseWindows);

    std::vector <Polynomial> polyLines;
    Polynomial polyPoints;

    for (std::vector <cv::Point2d> points : lanePoints) {
        polyPoints = this->fitPolyLine(points, degree);
        polyLines.emplace_back(polyPoints);
    }

    return polyLines;
}

intVec LineDetect::getHistogram(cv::Mat &image) {

    intVec histogram(image.cols, 0);

    for (int j = 0; j < image.rows; j++) {
        for (int i = 0; i < image.cols; i++) {
            int pixelValue = image.at<uchar>(j, i);
            if (pixelValue >= white) {
                histogram[i]++;
            }
        }
    }

    return histogram;
}

std::pair<int, int> LineDetect::getBaseHistogramPeakPosition(intVec histogram) {

    std::pair<int, int> peak(0, 0);
    int peakValue = 0;

    for (int i = 0; i < (int) (histogram.size() / 2.0); i++) {
        if (histogram[i] > peakValue) {
            peakValue = histogram[i];
            peak.first = i;
        }
    }

    peakValue = 0;

    for (int i = (int) (histogram.size() / 2.0); i < histogram.size(); i++) {
        if (histogram[i] > peakValue) {
            peakValue = histogram[i];
            peak.second = i;
        }
    }

    return peak;
}

std::vector <std::vector<cv::Point2d>> LineDetect::getLanePoints(cv::Mat &filteredImage, std::vector<Window> windows) {

    std::vector<std::vector<cv::Point2d>> linePoints(windows.size(), std::vector<cv::Point2d>(numVerticalSlice));

    for (int verticalSliceIndex = 0; verticalSliceIndex < numVerticalSlice; verticalSliceIndex++) {
        for (int windowIndex = 0; windowIndex < windows.size(); windowIndex++) {
            Window window = windows.at(windowIndex);
            cv::Mat windowSlice = this->getWindowSlice(filteredImage, window, verticalSliceIndex);
            intVec windowHistogram = this->getHistogram(windowSlice);
            int peak = this->getWindowHistogramPeakPosition(windowHistogram);
            window.center = peak;
            cv::Point2d point{
                    (double) window.center,
                    (double) (verticalSliceIndex * filteredImage.rows / numVerticalSlice)
            };
            linePoints[windowIndex].emplace_back(point);
        }
    }

    return linePoints;
}

cv::Mat LineDetect::getWindowSlice(cv::Mat &image, Window window, int verticalSliceIndex) {

    cv::Mat windowSlice = image(Range(window.getLeftSide(), window.getRightSide()),
                                Range(verticalSliceIndex * image.rows / numVerticalSlice,
                                      (verticalSliceIndex + 1) * image.rows / numVerticalSlice));

    return windowSlice;
}

int LineDetect::getWindowHistogramPeakPosition(intVec histogram) {

    int peak = 0;
    int peakValue = 0;

    for (int i = 0; i < (int) histogram.size(); i++) {
        if (histogram[i] > peakValue) {
            peakValue = histogram[i];
            peak = i;
        }
    }

    return peak;
}

Polynomial LineDetect::fitPolyLine(std::vector <cv::Point2d> points, int order) {

    int moreOrder = order + 1;
    assert(points.size() >= moreOrder);
    assert(order <= 3);

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
            A(i, j) = std::pow((xv.at(i)), j);

    // solve for linear least squares fit
    result = A.householderQr().solve(yvMapped);

    // 3rd order
    if (result.size() == 4)
        return Polynomial{result[3], result[2], result[1], result[0]};
    // 2nd order
    else if (result.size() == 3)
        return Polynomial{0, result[2], result[1], result[0]};
    // 1st order
    else
        return Polynomial{0, 0, result[1], result[0]};
}

cv::Point2d LineDetect::getIntersection(Polynomial leftLine, Polynomial rightLine) {

    // Isolate slopes
    double combinedSlope = leftLine.c - rightLine.c;

    // Isolate y-intercepts
    double combinedYIntercept = rightLine.d - leftLine.d;

    // Solve for x
    double x = combinedYIntercept / combinedSlope;

    // Solve for y
    double y = leftLine.c * x + leftLine.d;

    cv::Point2d point;
    point.x = x;
    point.y = y;

    return point;
}

double LineDetect::getAngleFromOriginToPoint(cv::Point2d point) {

    double dy = point.y;
    double dx = point.x;

    double angle = atan(dy / dx);


    // If the endpoint is behind and to the left
    if (dx < 0 && dy > 0) angle += M_PI;
        // If the endpoint is behind and to the right
    else if (dx < 0 && dy < 0) angle -= M_PI;

    return angle;
}

cv::Point2d LineDetect::moveAwayFromLine(Polynomial line, double distanceAwayFromOrigin, double distanceAwayFromLine) {

    cv::Point2d targetPoint;

    double y = line.c * distanceAwayFromOrigin + line.d;
    double displacementAwayFromLine = distanceAwayFromLine * line.d/fabs(line.d);

    targetPoint.y -= displacementAwayFromLine;
    targetPoint.x = (y - line.d) / line.c;

    return targetPoint;
}
