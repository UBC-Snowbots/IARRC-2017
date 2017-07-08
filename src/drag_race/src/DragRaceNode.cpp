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
    std::string twist_topic = private_nh.resolveName("twist");
    twist_publisher = private_nh.advertise<geometry_msgs::Twist>
            (twist_topic, queue_size);

    // Get Params
    SB_getParam(nh, "target_distance", target_distance, 1.0);
    SB_getParam(nh, "angular_vel_cap", angular_vel_cap, 1.0);
    SB_getParam(nh, "linear_vel_cap", linear_vel_cap, 1.0);
    SB_getParam(nh, "theta_scaling_multiplier", theta_scaling_multiplier, 1.0);
    SB_getParam(nh, "angular_speed_multiplier", angular_speed_multiplier, 1.0);
    SB_getParam(nh, "linear_speed_multiplier", linear_speed_multiplier, 1.0);
    SB_getParam(nh, "line_to_the_right", line_to_the_right, true);

    // TODO: Setup the obstacle manager with given params

    // TODO: Get our current location (relative to the walls) (and just keep trying until we can succesfully get it)

    // TODO: Save our current position (relative to the walls) so that we can can try and maintain it)
}

void DragRaceNode::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan) {
//    // Clear any obstacles we already have
//    obstacle_manager->clearObstacles();
//
//    // Insert the scan we just received
//    obstacle_manager->addLaserScan(*scan);

    // Get the obstacles (do we really need to if the obstacle maanger is doing all the work:?)
    //std::vector<LidarObstacle> obstacles = obstacle_manager.get_obstacles();

    // Get the longest line of cones
//    Line longest_cone_line = obstacle_manager->getLongestConeLine();
    // TODO: get an actual line
    LineOfBestFit *longest_cone_line = new LineOfBestFit(0, 1, 0);
    // Determine what we need to do to stay at the desired distance from the wall
    geometry_msgs::Twist twist = determineDesiredMotion(longest_cone_line, target_distance, line_to_the_right,
                                                        theta_scaling_multiplier, angular_speed_multiplier, linear_speed_multiplier,
                                                        angular_vel_cap, linear_vel_cap);

    // Publish our desired twist message
    twist_publisher.publish(twist);
}

geometry_msgs::Twist DragRaceNode::determineDesiredMotion(LineOfBestFit *longestConeLine, double targetDistance,
                                                          bool lineToTheRight, double theta_scaling_multiplier,
                                                          double angular_speed_multiplier, double linear_speed_multiplier,
                                                          double angular_vel_cap, double linear_vel_cap) {

    // Determine angle of line.
    double theta = atan(longestConeLine->getSlope());

    double distanceError = targetDistance - determineDistanceFromLine(longestConeLine);
    
    geometry_msgs::Twist command;

    // Set components we don't care about to 0
    command.linear.y = 0;
    command.linear.z = 0;
    command.angular.x = 0;
    command.angular.y = 0;

    // Figure out how fast we should be turning
    command.angular.z = (theta_scaling_multiplier * theta + distanceError) * angular_speed_multiplier;
    // Limit the angular velocity
    if (fabs(command.angular.z) > angular_vel_cap)
        command.angular.z = angular_vel_cap * command.angular.z / fabs(command.angular.z);

    if (!lineToTheRight)
        command.angular.z *= -1;

    // Figure out how fast we should be moving forward
    command.linear.x = linear_speed_multiplier / fabs(command.angular.z);

    // Limit the linear velocity
    if (command.linear.x > linear_vel_cap)
        command.linear.x = linear_vel_cap;

    return command;
}

double DragRaceNode::determineDistanceFromLine(LineOfBestFit *line) {
    double negReciprocal = -1 / line->getSlope();

    /* Find the intersection between the line and its perpendicular line. */

    // Set the two sides equal then isolate x to one side.
    double isolatedXSlope = negReciprocal - line->getSlope();

    // Divide both sides by the isolated slope to get the x point intersection.
    double xIntersection = line->getYIntercept() / isolatedXSlope;

    // Plug in the xIntersection to get the y point intersection.
    double yIntersection = negReciprocal * xIntersection;

    // Return distance found
    return sqrt(pow(xIntersection, 2) + pow(yIntersection, 2));

}

