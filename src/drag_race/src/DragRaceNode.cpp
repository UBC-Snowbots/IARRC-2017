/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <DragRaceNode.h>
#include <vector>

DragRaceNode::DragRaceNode(int argc, char **argv, std::string node_name) {
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
    SB_getParam(nh, "max_obstacle_merging_distance", max_obstacle_merging_distance, 0.3);
    SB_getParam(nh, "cone_grouping_tolerance", cone_grouping_tolerance, 1.8);
    SB_getParam(nh, "max_distance_from_robot_accepted", max_distance_from_robot_accepted, 2.0);
    SB_getParam(nh, "min_wall_length", min_wall_length, 0.4);
    SB_getParam(nh, "obstacle_ticks_threshold", obstacle_ticks_threshold, 10);
    SB_getParam(nh, "collision_distance", collision_distance, 3.0);

    // In DEGREES
    SB_getParam(nh, "collision_angle", collision_angle, 5.0);

    // Setup drag race controller with given params
    drag_race_controller = DragRaceController(target_distance, line_to_the_right, theta_scaling_multiplier,
                                           angular_speed_multiplier, linear_speed_multiplier, angular_vel_cap,
                                           linear_vel_cap);

    // Setup the obstacle manager with given params
    // TODO: Remove hardcoded collision_distance value
    obstacle_manager = LidarObstacleManager(max_obstacle_merging_distance, cone_grouping_tolerance,
                                            max_distance_from_robot_accepted, min_wall_length,
                                            collision_distance, collision_angle);

    end_of_course = false;
    incoming_obstacle_ticks = 0;
}

void DragRaceNode::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // Clear any obstacles we already have
    obstacle_manager.clearObstacles();

    // Insert the scan we just received
    obstacle_manager.addLaserScan(*scan);

    // TODO: Option 1
    if (obstacle_manager.collisionDetected()){
        incoming_obstacle_ticks++;
    } else {
        // False alarm
        incoming_obstacle_ticks = 0;

        // This is maybe better? Experiment
        // incoming_obstacle_ticks--;
        // if (incoming_obstacle_ticks < 0) incoming_obstacle_ticks = 0;
    }

    if (incoming_obstacle_ticks > obstacle_ticks_threshold) {
        if (!end_of_course) end_of_course = true;
    }

    // Get the best line for us
    LineOfBestFit best_line = obstacle_manager.getBestLine(line_to_the_right);

    // Determine what we need to do to stay at the desired distance from the wall
    geometry_msgs::Twist twist = drag_race_controller.determineDesiredMotion(best_line);


    // TODO: Option 2
    /*
    std::vector<LidarObstacle> obstacles = obstacle_manager.getObstacles();
    for (int i = 0; i < obstacles.size(); i++){
        LidarObstacle currObs = obstacles[i];
        if (currObs.getObstacleType() == WALL
                && currObs.getAvgDistance() < collision_distance
                && currObs.getLength() > 2.0 //TODO: If going for this option, maybe make length param
                && std::abs(currObs.getAvgAngle())*180/M_PI < collision_angle ){
            end_of_course = true;
        }
    }
    */

    if (end_of_course) {
        twist.linear.x = twist.linear.y = twist.linear.z = 0;
        twist.angular.x = twist.angular.y = twist.angular.z = 0;
    }

    // Publish our desired twist message
    twist_publisher.publish(twist);

    // TODO: have a debug param for this
    // Broadcast a visualisable representation so we can see obstacles in RViz
    cone_debug_publisher.publish(obstacle_manager.getConeRVizMarker());
    cone_line_debug_publisher.publish(obstacle_manager.getConeLinesRVizMarker());
    best_line_debug_publisher.publish(obstacle_manager.getBestConeLineRVizMarker(line_to_the_right));
}

