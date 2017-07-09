/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: TODO
 */

#include <LineDetect.h>
#include <gtest/gtest.h>

TEST(LineDetect, getHistogramSmallTest){

    cv::Mat testImage(cv::Size(4, 4), CV_8UC1, cv::Scalar(0));

    testImage.at<uchar>(1, 2) = 255;

    LineDetect testLineDetect;

    intVec testHistogram = testLineDetect.getHistogram(testImage);
    intVec expectedHistogram = {0, 1, 0, 0};
    EXPECT_EQ(expectedHistogram, testHistogram);
}

TEST(LineDetect, getHistogramLargeTest){

    cv::Mat testImage(cv::Size(10, 10), CV_8UC1, cv::Scalar(0));
    testImage.at<uchar>(2, 0) = 255;
    testImage.at<uchar>(2, 4) = 255;
    testImage.at<uchar>(2, 8) = 255;
    testImage.at<uchar>(2, 7) = 255;
    testImage.at<uchar>(4, 3) = 255;
    testImage.at<uchar>(4, 4) = 255;
    testImage.at<uchar>(8, 9) = 255;
    testImage.at<uchar>(8, 2) = 255;
    testImage.at<uchar>(8, 3) = 255;
    testImage.at<uchar>(9, 7) = 255;

    LineDetect testLineDetect;

    intVec testHistogram = testLineDetect.getHistogram(testImage);
    intVec expectedHistogram = {0, 0, 4, 0, 2, 0, 0, 0, 3, 1};
    EXPECT_EQ(expectedHistogram, testHistogram);
}

TEST(LineDetect, getWindowSliceLeftTest){

    cv::Mat testImage(cv::Size(10, 10), CV_8UC1, cv::Scalar(0));
    testImage.at<uchar>(2, 0) = 255;
    testImage.at<uchar>(2, 4) = 255;
    testImage.at<uchar>(2, 8) = 255;
    testImage.at<uchar>(2, 7) = 255;
    testImage.at<uchar>(4, 3) = 255;
    testImage.at<uchar>(4, 4) = 255;
    testImage.at<uchar>(8, 9) = 255;
    testImage.at<uchar>(8, 2) = 255;
    testImage.at<uchar>(8, 3) = 255;
    testImage.at<uchar>(9, 7) = 255;

    // let initialLineDetectThreshold and windowWidth be 2
    Window testWindow{2, 2};

    int verticalSliceIndex = 2;

    LineDetect testLineDetect;

    cv::Mat testWindowSlice = testLineDetect.getWindowSlice(testImage, testWindow, verticalSliceIndex);
    cv::Mat expectedWindowSlice = testImage(cv::Range(1, 3), cv::Range(2, 3));
    // Get a matrix with non-zero values at points where the two matrices have different values
    cv::Mat diff = (testWindowSlice != expectedWindowSlice);
    // Equal if no matrix elements disagree
    bool equal = cv::countNonZero(diff);
    EXPECT_EQ(false, equal);
}

TEST(LineDetect, getWindowSliceRightTest){

    cv::Mat testImage(cv::Size(10, 10), CV_8UC1, cv::Scalar(0));
    testImage.at<uchar>(2, 0) = 255;
    testImage.at<uchar>(2, 4) = 255;
    testImage.at<uchar>(2, 8) = 255;
    testImage.at<uchar>(2, 7) = 255;
    testImage.at<uchar>(4, 3) = 255;
    testImage.at<uchar>(4, 4) = 255;
    testImage.at<uchar>(8, 9) = 255;
    testImage.at<uchar>(8, 2) = 255;
    testImage.at<uchar>(8, 3) = 255;
    testImage.at<uchar>(9, 7) = 255;

    // let initialLineDetectThreshold and windowWidth be 2
    Window testWindow{8, 2};

    int verticalSliceIndex = 2;

    LineDetect testLineDetect;

    cv::Mat testWindowSlice = testLineDetect.getWindowSlice(testImage, testWindow, verticalSliceIndex);
    cv::Mat expectedWindowSlice = testImage(cv::Range(7, 9), cv::Range(2, 3));
    // Get a matrix with non-zero values at points where the two matrices have different values
    cv::Mat diff = (testWindowSlice != expectedWindowSlice);
    // Equal if no matrix elements disagree
    bool equal = cv::countNonZero(diff);
    EXPECT_EQ(false, equal);
}

TEST(LineDetect, getHistogramPeakPositionLargeTest){

    intVec testHistogram = {0, 1, 3, 5, 2, 1, 1, 2, 6, 2, 0, 3};

    LineDetect testLineDetect;

    std::pair<int, int> testPeak = testLineDetect.getHistogramPeakPosition(testHistogram);
    std::pair<int, int> expectedPeak(3, 8);
    EXPECT_EQ(expectedPeak, testPeak);
}

TEST(LineDetect, getHistogramPeakPositionSmallTest){

    intVec testHistogram = {2, 1, 7, 6, 2, 1, 6, 6, 0};

    LineDetect testLineDetect;

    std::pair<int, int> testPeak = testLineDetect.getHistogramPeakPosition(testHistogram);
    std::pair<int, int> expectedPeak(2, 6);
    EXPECT_EQ(expectedPeak, testPeak);
}

TEST(LineDetect, fitPolyLineLeftTest){

    Point testPoint1{2.0, 0.0};
    Point testPoint2{3.0, 1.0};
    Point testPoint3{4.0, 2.0};
    Point testPoint4{4.0, 3.0};

    std::vector<Point> testPoints = {testPoint1, testPoint2, testPoint3, testPoint4};
    int testOrder = 3;

    LineDetect testLineDetect;
    Polynomial testPolynomial = testLineDetect.fitPolyLine(testPoints, testOrder);
    Polynomial expectedPolynomial{2.0, 3.0, 4.0, 5.0};

    EXPECT_EQ(expectedPolynomial.a, testPolynomial.a);

    // Point point{(double)window.center, (double)(verticalSliceIndex*filteredImage.rows/numVerticalSlice)};
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}