/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Analyzes an image and detects lane lines, following them accordingly.
 */

#include <LineDetect.h>
#include <ros/ros.h>

int main(int argc, char* argv[]){
    std::string node_name = "line_detect";
    LineDetect line_detect(argc, argv, node_name);
    ros::spin();
    return 0;
}

