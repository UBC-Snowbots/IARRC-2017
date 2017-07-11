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
#include <visualization_msgs/Marker.h>

// SB Includes
#include <LidarObstacle.h>

class SlopeInterceptLine {
public:
    SlopeInterceptLine(double slope, double y_intercept) :
        slope(slope), y_intercept(y_intercept) {}

    // TODO: DOC functions
    inline double getSlope() { return slope; }
    inline double getYIntercept() { return y_intercept; }
    inline double getXIntercept() { return -y_intercept/slope; }
    inline double getXCoorAtY(double y) {
        if (slope == 0)
            return y_intercept;
        else
            return (y - y_intercept)/slope;
    }
    inline double getYCoorAtX(double x) { return slope * x + y_intercept; }

protected:
    double slope;
    double y_intercept;
};

/**
 * A line with a Correlation Coefficient
 */
class LineOfBestFit : public SlopeInterceptLine {
public:
    LineOfBestFit(double slope, double y_intercept, double correlation) :
            SlopeInterceptLine(slope, y_intercept), correlation(correlation) {}

    double correlation;
};

struct FiniteLine {
    double start_x;
    double start_y;
    double slope;
    double length;
};

class LidarObstacleManager {
public:
    LidarObstacleManager(double max_obstacle_merging_distance, double max_distance_from_robot_accepted, double cone_grouping_tolerance);

    /**
     * Merges or adds the given obstacle to the already saved ones
     *
     * @param obstacle the obstacle to be added
     */
    void addObstacle(LidarObstacle obstacle);

    /**
     * Finds a saves obstacles from the given scan
     *
     * Gets obstacles from the given scan and merges or adds them to
     * the already saved obstacles
     *
     * @param scan the scan to be merged in
     */
    void addLaserScan(const sensor_msgs::LaserScan& scan);

    /**
     * Clears all saved obstacles
     */
    void clearObstacles();

    /**
     * Gets all obstacles
     * @return all saved obstacles
     */
     std::vector<LidarObstacle> getObstacles();

    /**
     * Gets all lines of cones in the saved obstacles
     * @return all lines of cones in the saved obstacles
     */
    std::vector<LineOfBestFit> getConeLines();

    /**
     * Gets a line of best fit for a given group of points
     *
     * @param points the points to fit the line to
     */
    static LineOfBestFit getLineOfBestFit(const std::vector<Point> &points);

    LineOfBestFit getBestLine(bool lineToTheRight);

    /**
     * Finds groups of points within a larger group of points
     *
     * @param points the points to find the groups in
     * @param tolerance the maximum distance between any two points in a group
     *
     * @return a vector of groups of points (as vectors)
     */
    std::vector<std::vector<Point>> getPointGroupings(std::vector<Point> points, double tolerance);

    /**
     * Determines the minimum distance between two obstacles
     *
     * @param obstacle1
     * @param obstacle2
     * @return the minimum distance between obstacle1 and obstacle2
     */
    static double minDistanceBetweenObstacles(LidarObstacle obstacle1, LidarObstacle obstacle2);

    /**
     * Gets all stored obstacles as a marker of points that can be rendered in RViz
     *
     * @return all stored obstacles as a marker of points that can be rendered in RViz
     */
    // TODO: when we update lidarObstacle to be a bit more obstract, improve this as well
    // TODO: to represent things like radius
    // TODO: TEST ME
    visualization_msgs::Marker getConeRVizMarker();

    /**
     * Gets the lines determined from cones as a marker of lines that can be rendered in RViz
     *
     * @return the lines determined from cones as a marker of lines that can be rendered in RViz
     */
    // TODO: Can we add line endings?
    // TODO: TEST ME
    visualization_msgs::Marker getConeLinesRVizMarker();

    /**
     * Gets the "best" line from all the cone lines as a marker we can visualize in RViz
     *
     * @return the lines determined from cones as a marker that can be rendered in RViz
     */
    // TODO: Can we add line endings?
    // TODO: TEST ME
    // TODO: Add line_to_the_right to the constructor
    visualization_msgs::Marker getBestConeLineRVizMarker(bool line_to_the_right);

private:

    // All the obstacles we currently have
    std::vector<LidarObstacle> obstacles;

    // The maximum distance between two obstacles for them to be considered the same
    double max_obstacle_merging_distance;

    // The maximum distance a line can have from the robot on the y-axis before being thrown out
    double max_distance_from_robot_accepted;

    // The maximum permitted distance between cones in the same group
    double cone_grouping_tolerance;

};

#endif //DRAG_RACE_LIDAROBSTACLEMANAGER_H
