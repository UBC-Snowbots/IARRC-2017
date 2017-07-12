/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <DragRaceNode.h>

DragRaceNode::DragRaceNode(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string scan_topic = "/scan";
    uint32_t queue_size = 1;

    scan_subscriber = nh.subscribe(scan_topic, queue_size,
                                   &DragRaceNode::scanCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = nh.resolveName("cmd_vel");
    twist_publisher = nh.advertise<geometry_msgs::Twist>
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
    SB_getParam(private_nh, "target_distance", target_distance, 1.0);
    SB_getParam(private_nh, "angular_vel_cap", angular_vel_cap, 1.0);
    SB_getParam(private_nh, "linear_vel_cap", linear_vel_cap, 1.0);
    SB_getParam(private_nh, "theta_scaling_multiplier", theta_scaling_multiplier, 1.0);
    SB_getParam(private_nh, "angular_speed_multiplier", angular_speed_multiplier, 1.0);
    SB_getParam(private_nh, "linear_speed_multiplier", linear_speed_multiplier, 1.0);
    SB_getParam(private_nh, "line_to_the_right", line_to_the_right, true);
    double max_obstacle_merging_distance, cone_grouping_tolerance, min_wall_length;
    SB_getParam(private_nh, "max_obstacle_merging_distance", max_obstacle_merging_distance, 0.3);
    SB_getParam(private_nh, "cone_grouping_tolerance", cone_grouping_tolerance, 1.8);
    SB_getParam(private_nh, "max_distance_from_robot_accepted", max_distance_from_robot_accepted, 2.0);
    SB_getParam(private_nh, "min_wall_length", min_wall_length, 0.4);

    // Setup drag race controller with given params
    drag_race_controller = DragRaceController(target_distance, line_to_the_right, theta_scaling_multiplier,
                                              angular_speed_multiplier, linear_speed_multiplier, angular_vel_cap,
                                              linear_vel_cap);

    // Setup the obstacle manager with given params
    obstacle_manager = LidarObstacleManager(max_obstacle_merging_distance, cone_grouping_tolerance,
                                            max_distance_from_robot_accepted, min_wall_length);
}

void DragRaceNode::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan) {
    // Clear any obstacles we already have
    obstacle_manager.clearObstacles();

    // Insert the scan we just received
    obstacle_manager.addLaserScan(*scan);

    // Get the best line for us
    LineOfBestFit best_line = obstacle_manager.getBestLine(line_to_the_right);

    geometry_msgs::Twist twist;

    // If no line then go straight.
    if (fabs(best_line.correlation) == 0) {
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
        twist.linear.x = linear_vel_cap;
        twist.linear.y = 0;
        twist.linear.z = 0;
    } else
        // Avoid the line given while staying within the boundaries
        twist = drag_race_controller.determineDesiredMotion(best_line);

    // Publish our desired twist message
    twist_publisher.publish(twist);

    // TODO: have a debug param for this
    // Broadcast a visualisable representation so we can see obstacles in RViz
    cone_debug_publisher.publish(obstacle_manager.getConeRVizMarker());
    cone_line_debug_publisher.publish(obstacle_manager.getConeLinesRVizMarker());
    best_line_debug_publisher.publish(obstacle_manager.getBestConeLineRVizMarker(line_to_the_right));
}

