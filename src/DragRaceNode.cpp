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

    scan_subscriber = nh.subscribe( scan_topic, queue_size, 
                                    &DragRaceNode::scanCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = private_nh.resolveName("twist");
    twist_publisher = private_nh.advertise<geometry_msgs::Twist>
                                (twist_topic, queue_size);
    
    // Get Params
    //SB_getParam(nh, "")
    
    // TODO: Setup the obstacle manager with given params
    
    // TODO: Get our current location (relative to the walls) (and just keep trying until we can succesfully get it)
    
    // TODO: Save our current position (relative to the walls) so that we can can try and maintain it)
}

void DragRaceNode::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // Clear any obstacles we already have
    obstacle_manager.clear_obstacles();

    // Insert the scan we just received
    obstacle_manager.insertScan(*scan);

    // Get the obstacles (do we really need to if the obstacle maanger is doing all the work:?)
    //std::vector<LidarObstacle> obstacles = obstacle_manager.get_obstacles();

    // Get the longest line of cones
    ConeLine longest_cone_line = obstacle_manager.getLongestConeLine();

    // Determine what we need to do to stay at the desired distance from the wall
    geometry_msgs::Twist twist = determineDesiredMotion(longest_cone_line);

    // Publish our desired twist message
    twist_publisher.publish(twist); 
}

geometry_msgs::Twist DragRaceNode::determineDesiredMotion( sensor_msgs::LaserScan& scan) {

}
