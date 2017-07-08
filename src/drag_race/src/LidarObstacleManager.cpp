/*
 * Created By: Gareth Ellis
 * Created On: July 4, 2017
 * Description: TODO
 */

#include <LidarObstacleManager.h>
#include <stack>

LidarObstacleManager::LidarObstacleManager(
        double max_obstacle_merging_distance,
        double cone_grouping_tolerance
):
        max_obstacle_merging_distance(max_obstacle_merging_distance),
        cone_grouping_tolerance(cone_grouping_tolerance)
{}

void LidarObstacleManager::addLaserScan(const sensor_msgs::LaserScan &scan) {
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

std::vector<LidarObstacle> LidarObstacleManager::getObstacles() {
    return obstacles;
}

void LidarObstacleManager::addObstacle(LidarObstacle obstacle) {
    // See if this obstacle is close enough to any other saved obstacle to be the same
    // TODO: Should we be instead checking for the CLOSEST saved obstcle?
    for (LidarObstacle saved_obstacle : obstacles) {
        if (minDistanceBetweenObstacles(saved_obstacle, obstacle)
            < max_obstacle_merging_distance) {
            saved_obstacle.mergeInLidarObstacle(obstacle);
            return;
        }
    }
    obstacles.emplace_back(obstacle);
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

std::vector<LineOfBestFit> LidarObstacleManager::getConeLines() {
    // Get all our cones as points
    std::vector<Point> points;
    for (LidarObstacle obstacle : obstacles){
        if (obstacle.getObstacleType() == CONE){
            points.emplace_back(obstacle.getCenter());
        }
    }

    // Get groups of lines
    std::vector<std::vector<Point>> groups = getPointGroupings(points, cone_grouping_tolerance);

    // Fit a line of best fit to each group
    std::vector<LineOfBestFit> lines;
    std::transform(groups.begin(), groups.end(), lines.begin(), getLineOfBestFit);

    return lines;
}

std::vector<std::vector<Point>> LidarObstacleManager::getPointGroupings(std::vector<Point> points, double tolerance) {

    std::vector<std::vector<Point>> groups;

    // Go through every point
    while(points.size() > 0) {
        // Start the current group off with the last point
        std::vector<Point> group;
        std::stack<Point> to_visit;
        to_visit.emplace(points.back());
        points.pop_back();
        do {
            // Visit the first point in to_visit
            Point curr_point = to_visit.top();
            to_visit.pop();

            // Figure out if there are any points within tolerance of the point we're visiting
            for (int i = 0; i < points.size(); i++){
                Point p = points[i];
                if (distanceBetweenPoints(curr_point, p) < tolerance) {
                    // Add point to to_visit and remove from the given list of points
                    to_visit.emplace(p);
                    points.erase(points.begin() + i);
                }
            }

            // Add the current point to the group
            group.emplace_back(curr_point);

            // Keep going if we've got more points to visit
        } while (to_visit.size() > 0);

        groups.emplace_back(group);
    }

    return groups;
}

LineOfBestFit LidarObstacleManager::getLineOfBestFit(const std::vector<Point> &points) {
    // Get line of best fit using linear regression formula
    // http://www.statisticshowto.com/how-to-find-a-linear-regression-equation/

    double x_sum = std::accumulate(points.begin(), points.end(), 0.0,
                                   [](double accum, Point p){ return accum + p.x; });
    double y_sum = std::accumulate(points.begin(), points.end(), 0.0,
                                   [](double accum, Point p){ return accum + p.y; });
    double x_squared_sum = std::accumulate(points.begin(), points.end(), 0.0,
                                   [](double accum, Point p){ return accum + std::pow(p.x, 2.0); });
    double x_y_product_sum = std::accumulate(points.begin(), points.end(), 0.0,
                                           [](double accum, Point p){ return accum + p.x * p.y; });
    double intercept = (y_sum * x_squared_sum - x_sum * x_y_product_sum) /
            (points.size() * x_squared_sum - std::pow(x_sum, 2.0));
    double slope = (points.size() * x_y_product_sum - x_sum * y_sum) /
            (points.size() * x_squared_sum - std::pow(x_sum, 2.0));

    // TODO: calculate correlation coeffecient and add it here

    return LineOfBestFit(slope, intercept, 0);
}

