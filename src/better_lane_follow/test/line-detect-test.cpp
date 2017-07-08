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

    testImage.at<uchar>(4, 3) = 255;
    testImage.at<uchar>(4, 4) = 255;

    testImage.at<uchar>(5, 2) = 128;

    LineDetect testLineDetect;
    std::vector<int> testHistogram = testLineDetect.getHistogram(testImage);

    std::vector<int> expectedHistogram = {0, 0, 3, 0, 2, 0, 0, 0, 0, 0};

    EXPECT_EQ(expectedHistogram, testHistogram);
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}