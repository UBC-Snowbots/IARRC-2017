/*
 * Created By: Gareth Ellis
 * Created On: July 4, 2017
 * Description: TODO
 */

#ifndef DRAG_RACE_LIDAROBSTACLEMANAGER_H
#define DRAG_RACE_LIDAROBSTACLEMANAGER_H

// STD Includes
#include <vector>

// ROS Includes
#include <sensor_msgs/LaserScan.h>
// SB Includes
#include <LidarObstacle.h>

// A basic line of the form: `y = slope * x + intercept`
struct Line {
    float slope;
    float intercept;
};

class LidarObstacleManager {
public:
    // TODO: Constructor
    // Should take: `min_obstacle_merging_distance`
    LidarObstacleManager(double max_obstacle_merging_distance);

    /**
     * Finds a saves obstacles from the given scan
     *
     * Gets obstacles from the given scan and merges or adds them to
     * the already saved obstacles
     *
     * @param scan the scan to be merged in
     */
    void addScan(sensor_msgs::LaserScan& scan);

    /**
     * Clears all saved obstacles
     */
    void clearObstacles();

    /**
     * Gets the longest line of cones in the saved obstacles
     *
     * @return the longest line of cones in the saved obstacles
     */
    Line getLongestConeLine();

private:
    /**
     * Merges to adds the given obstacle to the already saved ones
     *
     * @param obstacle the obstacle to be added
     */
    void addObstacle(LidarObstacle obstacle);

    /**
     * Determines the minimum distance between two obstacles
     *
     * @param obstacle1
     * @param obstacle2
     * @return the minimum distance between obstacle1 and obstacle2
     */
    double minDistanceBetweenObstacles(LidarObstacle obstacle1, LidarObstacle obstacle2);

    // All the obstacles we currently have
    std::vector<LidarObstacle> obstacles;

    // The maximum distance between two obstacles for them to be considered the same
    double max_obstacle_merging_distance;
};

#endif //DRAG_RACE_LIDAROBSTACLEMANAGER_H
