/*
 * Created By: Robyn Castro
 * Created On: June 15, 2017
 * Description: Decides when the robot starts moving
 *              based on whether green is being seen.
 */

#include <GreenRecognition.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "green_recognition";
    // Create an instance of your class
    GreenRecognition green_recognition(argc, argv, node_name);
    // Start up ROS, this will continue to run until the node is killed
    ros::spin();
    // Once the node stops, return 0
    return 0;
}