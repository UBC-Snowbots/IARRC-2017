#include <gtest/gtest.h>
#include <CircleDetection.h>
#include "../../vision/include/HSVFilterNode.h"

TEST(realImage, GreenLight) {
    std::string image_path = "images/GreenLight.jpg";
    HSVFilterNode *greenFilter = new HSVFilterNode(image_path);
}

TEST(realImage, RedLight) {
    std::string image_path = "images/RedLight.jpg";
    HSVFilterNode *greenFilter = new HSVFilterNode(image_path);
}

TEST(filteredImage, oneCircle) {
    std::string image_path = "images/binaryCircles.jpg";
    CircleDetection *greenRecognition = new CircleDetection(image_path);
}

int main(int aimageTests, char **argv) {
    testing::InitGoogleTest(&aimageTests, argv);
    return RUN_ALL_TESTS();
}