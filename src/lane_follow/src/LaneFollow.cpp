/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Gets angle of lane point of intersection
 *              and broadcasts a recommended Twist message.
 */

#include <ros/ros.h>
#include <LineDetect.h>

LaneFollow::LaneFollow(int argc, char** argv, std::string lane_follow) {

    // Instantiate LineDetect class in LaneFollow ROS node
    LineDetect line_detect(argc, argv, lane_follow);

    // Setup handles
    ros::init(argc, argv, lane_follow);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string image_topic_name = "/robot/cameras/raw_image";
    std::string filtered_topic_name = "/robot/line_detect/filtered_image";
    uint32_t refresh_rate = 10;
    uint32_t queue_size = 1;

    // Setup subscriber
    ros::Subscriber image_sub = n.subscribe(image_topic_name, refresh_rate, &LaneFollow::imageCallBack, this);

    // Setup publisher
    ros::Publisher filter_pub = n.advertise<cv::Mat>(filtered_topic_name, queue_size);

    ros::Rate loop_rate(10);

    while (ros::ok()) {

        // Wrapper function which runs LineDetect alg

        filter_pub.publish(LineDetect::cv::image);

        ros::spinOnce();
        loop_rate.sleep();
    }
}




    int angle_theta = LaneFollow::angleDetermine(LineDetect::cv::image);
    geometry_msgs::Twist stayInLane = LaneFollow::magicFunction(angle_theta);
}

