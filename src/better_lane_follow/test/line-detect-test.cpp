/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: TODO
 */

#include <LineDetect.h>
#include <gtest/gtest.h>

TEST(LineDetect, getHistogramSmallTest){

    cv::Mat testImage(cv::Size(4, 3), CV_8UC1, cv::Scalar(0));

    testImage.at<uchar>(2, 2) = 255;

    LineDetect testLineDetect;
    std::vector<int> testHistogram = testLineDetect.getHistogram(testImage);

    std::vector<int> expectedHistogram = {0, 1, 0, 0};

    EXPECT_EQ(expectedHistogram, testHistogram);
}

TEST(LineDetect, getHistogramLargeTest){

    cv::Mat testImage(cv::Size(10, 8), CV_8UC1, cv::Scalar(0));

    testImage.at<uchar>(3, 0) = 255;
    testImage.at<uchar>(3, 4) = 255;
    testImage.at<uchar>(3, 8) = 255;

    testImage.at<uchar>(5, 3) = 255;
    testImage.at<uchar>(5, 4) = 255;

    testImage.at<uchar>(6, 2) = 128;

    LineDetect testLineDetect;
    std::vector<int> testHistogram = testLineDetect.getHistogram(testImage);

    std::vector<int> expectedHistogram = {0, 0, 3, 0, 2, 0, 0, 0, 0, 0};

    EXPECT_EQ(expectedHistogram, testHistogram);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}