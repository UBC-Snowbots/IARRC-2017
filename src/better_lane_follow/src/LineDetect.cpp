/*
 * Created by: Raad Khan
 * Created On: July 1, 2017
 * Description: Takes in an image feed and generates lane lines.
 * Usage: LaneFollow node instantiates this class to generate lane lines
 */

#include <LineDetect.h>
#include <Eigen/QR>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "../../vision/include/IPM.h"

using namespace cv;

LineDetect::LineDetect() : initialLineDetectThreshold(50),
                           white(250),
                           windowWidth(200),
                           numVerticalSlice(10),
                           degree(2) {}

std::vector <Polynomial> LineDetect::getLines(cv::Mat &filteredImage) {

    intVec baseHistogram = LineDetect::getHistogram(filteredImage);
    std::vector <Window> windows;

    for (int i = 0; i < baseHistogram.size(); i++) {
        if (baseHistogram[i] > initialLineDetectThreshold) {
            Window window{i, windowWidth};
            windows.emplace_back(window);
        }
    }

    std::vector <std::vector<cv::Point2d>> linePoints(windows.size(), std::vector<cv::Point2d>(numVerticalSlice));

    for (int verticalSliceIndex = 0; verticalSliceIndex < numVerticalSlice; verticalSliceIndex++) {
        for (int windowIndex = 0; windowIndex < windows.size(); windowIndex++) {
            Window window = windows.at(windowIndex);
            cv::Point2d point{(double) window.center,
                              (double) (verticalSliceIndex * filteredImage.rows / numVerticalSlice)};
            linePoints[windowIndex].emplace_back(point);

            cv::Mat windowSlice = LineDetect::getWindowSlice(filteredImage, window, verticalSliceIndex);
            intVec windowHistogram = LineDetect::getHistogram(windowSlice);
            std::pair<int, int> peak = LineDetect::getHistogramPeakPosition(windowHistogram);

            window.center = windowIndex ? window.center = peak.second
                                        : peak.first /*+ (window.center - window.width/2)*/;
        }
    }

    std::vector <Polynomial> polyLines;
    Polynomial polyPoints;

    for (std::vector <cv::Point2d> points : linePoints) {
        polyPoints = LineDetect::fitPolyLine(points, degree);
        polyLines.emplace_back(polyPoints);
    }

    return polyLines;
}

intVec LineDetect::getHistogram(cv::Mat &image) {

    intVec histogram(image.cols, 0);

    for (int j = 0; j < image.rows; j++) {
        for (int i = 0; i < image.cols; i++) {
            int pixelValue = image.at<uchar>(i, j);
            if (pixelValue >= white) {
                histogram[i]++;
            }
        }
    }

    return histogram;
}

cv::Mat LineDetect::getWindowSlice(cv::Mat &image, Window window, int verticalSliceIndex) {

    cv::Mat windowSlice = image(Range(window.getLeftSide(), window.getRightSide()),
                                Range(verticalSliceIndex * image.rows / numVerticalSlice,
                                      (verticalSliceIndex + 1) * image.rows / numVerticalSlice));

    return windowSlice;
}

std::pair<int, int> LineDetect::getHistogramPeakPosition(intVec histogram) {

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

    if (result.size() == 4)
        return Polynomial{result[3], result[2], result[1], result[0]};
    else
        return Polynomial{0, result[2], result[1], result[0]};
}

cv::Point2d LineDetect::getIntersection(Polynomial leftLine, Polynomial rightLine) {

    // Isolate slopes
    double bCombinedSlope = leftLine.b - rightLine.b;

    double cCombinedSlope = leftLine.c - rightLine.c;

    double dCombinedSlope = leftLine.d - rightLine.d;

    // Solve for x
    double x = cubicFormula(0, bCombinedSlope, cCombinedSlope, dCombinedSlope);

    // Solve for y
    double y = leftLine.b * std::pow(x, 2) + leftLine.c * x + leftLine.d;

    cv::Point2d point;
    point.x = x;
    point.y = y;

    return point;
}

double LineDetect::cubicFormula(double a, double b, double c, double d) {
    double p = -b / (3.0 * a);
    double q = std::pow(p, 3) + (b * c - 3.0 * a * d) / (6.0 * std::pow(a, 2));
    double r = c / (3.0 * a);
    double s = std::pow(p, 2);
    double t = std::pow(q, 2);
    double u = std::pow(r - s, 3);
    double v = std::pow(t + u, 1.0 / 2.0);
    double w = std::pow(q + v, 1.0 / 3.0);
    double x = std::pow(q - v, 1.0 / 3.0);

    return w + x + p;
}

double LineDetect::quadraticFormula(double a, double b, double c, double &x_1, double &x_2) {

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

cv::Point2d LineDetect::moveAwayFromLine(Polynomial line, double targetXDistance, double targetYDistance) {
    cv::Point2d targetPoint;

    // Move along the line and stop when targetXDistance is met.
    targetPoint.x = targetXDistance;
    targetPoint.y =
            line.a * std::pow(targetPoint.x, 3) + line.b * std::pow(targetPoint.x, 2) + line.c * targetPoint.x + line.d;

    // Find y-intercept of line.
    double y_intercept = cubicFormula(line.a, line.b, line.c, line.d);

    // Go a certain distance away from the line while staying in the course.
    if (y_intercept > 0)
        targetPoint.y -= targetYDistance; // If line is to the right move left.
    else
        targetPoint.y += targetYDistance; // If line is to the left move right.

    return targetPoint;
}

std::vector<Point2d> LineDetect::transformPoints(std::vector<cv::Point2d> points) {

    std::vector<Point2d> realPoints;

    for (int i = 0; i < points.size(); i++) {
        realPoints.push_back(applyHomographyInv(points[i]));
    }

    return realPoints;
}
