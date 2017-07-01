#include <gtest/gtest.h>
#include <CircleDetection.h>
#include <GreenFilter.h>

TEST(realImage, GreenLight) {
    std::string image_path = "/home/sb/IARRC-2017/src/green_recognition/test/images/GreenLight.jpg";
    GreenFilter *greenFilter = new GreenFilter(image_path);
}

TEST(realImage, RedLight) {
    std::string image_path = "/home/sb/IARRC-2017/src/green_recognition/test/images/RedLight.jpg";
    GreenFilter *greenFilter = new GreenFilter(image_path);
}

TEST(filteredImage, oneCircle) {
    std::string image_path = "/home/sb/IARRC-2017/src/green_recognition/test/images/binaryCircles.jpg";
    CircleDetection *greenRecognition = new CircleDetection(image_path);
}

int main(int aimageTests, char **argv) {
    testing::InitGoogleTest(&aimageTests, argv);
    return RUN_ALL_TESTS();
}