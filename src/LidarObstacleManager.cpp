/*
 * Created By: Gareth Ellis
 * Created On: July 4, 2017
 * Description: TODO
 */

#include <LidarObstacleManager.h>

LidarObstacleManager::LidarObstacleManager(double max_obstacle_merging_distance):
        max_obstacle_merging_distance(max_obstacle_merging_distance) {}

void LidarObstacleManager::addScan(sensor_msgs::LaserScan &scan) {
    // Create an obstacle for every hit in the lidar scan
    std::vector<LidarObstacle> temp_obstacles;
    for (int i = 0; i < scan.ranges.size(); ++i) {
        // Check that the lidar hit is within acceptable bounds
        double angle = scan.angle_min + i * scan.angle_increment;
        double range = scan.ranges[i];
        if (range < scan.range_max && range > scan.range_min) {
            addObstacle(LidarObstacle(angle, range));
        }
    }
}

void LidarObstacleManager::addObstacle(LidarObstacle obstacle) {
    for (LidarObstacle saved_obstacle : obstacles) {
        if (minDistanceBetweenObstacles(saved_obstacle, obstacle)
            < max_obstacle_merging_distance)
            saved_obstacle.mergeInLidarObstacle(obstacle);
        else
            obstacles.emplace_back(obstacle);
    }
}

double LidarObstacleManager::minDistanceBetweenObstacles(
        LidarObstacle obstacle1, LidarObstacle obstacle2) {
    // TODO: This is ABSURDLY ineffecient. We're doing n^2 operations on 2 potentially
    // TODO: very large objects. We should be able to do some sort of obstacle approx to
    // TODO: circles or boxes and compare them that way (need to update LidarObstacle for that approx)

    std::vector<Point> obstacle1_points = obstacle1.getReadingsAsPoints();
    std::vector<Point> obstacle2_points = obstacle2.getReadingsAsPoints();

    // Compare every point to.... *shudders slightly* every other point..
    double min_distance = -1;
    for (Point p1 : obstacle1_points){
        for (Point p2 : obstacle2_points){
            double dx = p1.x - p2.x;
            double dy = p1.y - p2.y;
            double distance = std::sqrt(std::pow(dx,2.0) + std::pow(dy,2.0));
            if (min_distance < 0 || distance < min_distance)
                min_distance = distance;
        };
    };

    return min_distance;
}

Line LidarObstacleManager::getLongestConeLine() {

    // Get points for all our cones
    using Neighbours = std::vector<std::shared_ptr<Point>>;
    std::vector<std::pair<Point, Neighbours>> cone_points;
    for (LidarObstacle obstacle : obstacles) {
        if (obstacle.getObstacleType() == CONE) {
            cone_points.emplace_back(obstacle.getCenter());
        }
    }

    // Find all neighbours for all cones recursively
    for
}
