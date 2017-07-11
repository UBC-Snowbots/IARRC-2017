/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <DragRaceNode.h>

DragRaceNode::DragRaceNode(int argc, char **argv, std::string node_name):
    obstacle_manager(0.4, 1.5)
{
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string scan_topic = "/scan";
    uint32_t queue_size = 1;

    scan_subscriber = nh.subscribe( scan_topic, queue_size, 
                                    &DragRaceNode::scanCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = private_nh.resolveName("twist");
    twist_publisher = private_nh.advertise<geometry_msgs::Twist>
                                (twist_topic, queue_size);
    std::string cone_debug_topic = private_nh.resolveName("debug/cone");
    cone_debug_publisher = private_nh.advertise<visualization_msgs::Marker>
            (cone_debug_topic, queue_size);
    std::string cone_lines_debug_topic = private_nh.resolveName("debug/cone_lines");
    cone_line_debug_publisher = private_nh.advertise<visualization_msgs::Marker>
            (cone_lines_debug_topic, queue_size);
    std::string best_line_debug_topic = private_nh.resolveName("debug/best_line");
    best_line_debug_publisher = private_nh.advertise<visualization_msgs::Marker>
            (best_line_debug_topic, queue_size);

    // Get Params
    SB_getParam(nh, "target_distance", target_distance, 1.0);
    SB_getParam(nh, "angular_vel_cap", angular_vel_cap, 1.0);
    SB_getParam(nh, "linear_vel_cap", linear_vel_cap, 1.0);
    SB_getParam(nh, "theta_scaling_multiplier", theta_scaling_multiplier, 1.0);
    SB_getParam(nh, "angular_speed_multiplier", angular_speed_multiplier, 1.0);
    SB_getParam(nh, "linear_speed_multiplier", linear_speed_multiplier, 1.0);
    SB_getParam(nh, "line_to_the_right", line_to_the_right, false);

    // Setup drag race controller with given params
    drag_race_controller = DragRaceController(target_distance, line_to_the_right, theta_scaling_multiplier,
                                           angular_speed_multiplier, linear_speed_multiplier, angular_vel_cap,
                                           linear_vel_cap);
}

void DragRaceNode::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // Clear any obstacles we already have
    obstacle_manager.clearObstacles();

    // Insert the scan we just received
    obstacle_manager.addLaserScan(*scan);

    // Get the best line for us
    LineOfBestFit best_line = obstacle_manager.getBestLine(line_to_the_right);

    // Determine what we need to do to stay at the desired distance from the wall
    geometry_msgs::Twist twist = drag_race_controller.determineDesiredMotion(best_line);

    // Publish our desired twist message
    twist_publisher.publish(twist);

    // TODO: have a debug param for this
    // Broadcast a visualisable representation so we can see obstacles in RViz
    cone_debug_publisher.publish(obstacle_manager.getConeRVizMarker());
    cone_line_debug_publisher.publish(obstacle_manager.getConeLinesRVizMarker());
    best_line_debug_publisher.publish(obstacle_manager.getBestConeLineRVizMarker(line_to_the_right));
}
