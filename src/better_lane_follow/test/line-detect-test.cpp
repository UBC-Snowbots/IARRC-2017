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
// TODO add more histogram tests

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}