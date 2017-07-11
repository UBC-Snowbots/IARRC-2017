/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <DragRaceNode.h>

DragRaceNode::DragRaceNode(int argc, char **argv, std::string node_name):
    obstacle_manager(0.4, 1),
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
    std::string obstacle_debug_topic = private_nh.resolveName("debug/obstacles");
    obstacle_debug_publisher = private_nh.advertise<visualization_msgs::Marker>
            (obstacle_debug_topic, queue_size);

    // Get Params
    SB_getParam(nh, "target_distance", target_distance, 1.0);
    SB_getParam(nh, "angular_vel_cap", angular_vel_cap, 1.0);
    SB_getParam(nh, "linear_vel_cap", linear_vel_cap, 1.0);
    SB_getParam(nh, "theta_scaling_multiplier", theta_scaling_multiplier, 1.0);
    SB_getParam(nh, "angular_speed_multiplier", angular_speed_multiplier, 1.0);
    SB_getParam(nh, "linear_speed_multiplier", linear_speed_multiplier, 1.0);
    SB_getParam(nh, "line_to_the_right", line_to_the_right, true);

    // Setup drag race controller with given params
    dragRaceController = new DragRaceController(target_distance, line_to_the_right, theta_scaling_multiplier,
                                                angular_speed_multiplier, linear_speed_multiplier, angular_vel_cap,
                                                linear_vel_cap);

    // TODO: Setup the obstacle manager with given params

    // TODO: Get our current location (relative to the walls) (and just keep trying until we can succesfully get it)

    // TODO: Save our current position (relative to the walls) so that we can can try and maintain it)
}

void DragRaceNode::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // Clear any obstacles we already have
    obstacle_manager.clearObstacles();

    // Insert the scan we just received
    obstacle_manager.addLaserScan(*scan);

    // TODO: have a debug param for this
    // Broadcast a visualisable representation so we can see obstacles in RViz
    obstacle_debug_publisher.publish(obstacle_manager.getObstacleRVizMarkers());


    // Get the obstacles (do we really need to if the obstacle maanger is doing all the work:?)
    //std::vector<LidarObstacle> obstacles = obstacle_manager.get_obstacles();

    // Get the longest line of cones
//    Line longest_cone_line = obstacle_manager->getLongestConeLine();
    // TODO: get an actual line
    LineOfBestFit *longest_cone_line = new LineOfBestFit(0, 1, 0);

    // Determine what we need to do to stay at the desired distance from the wall
    geometry_msgs::Twist twist = dragRaceController->determineDesiredMotion(longest_cone_line);

    // Publish our desired twist message
    twist_publisher.publish(twist);
}

LineOfBestFit *DragRaceNode::getBestLine(std::vector<LineOfBestFit *> lines, bool lineToTheRight) {
    LineOfBestFit *bestLine = new LineOfBestFit(0, 0, 0);

    for (size_t i = 0; i < lines.size(); i++) {
        // Only check lines where the y-intercept is on the correct side.
        if ((lineToTheRight && lines[i]->getYIntercept() < 0) || (!lineToTheRight && lines[i]->getYIntercept() >= 0)) {
            // If correlation is stronger than the current best, update best line.
            if (fabs(lines[i]->correlation) > fabs(bestLine->correlation))
                bestLine = lines[i];
        }
    }
    return bestLine;
}
