/*
 * Created By: Gareth Ellis
 * Created On: July 4, 2017
 * Description: TODO
 */

#include <LidarObstacleManager.h>
#include <gtest/gtest.h>

class LidarObstacleManagerTest : public testing::Test {
protected:
    LidarObstacleManagerTest() :
            obstacle_manager_1(0.2, 2)
            {}

    virtual void SetUp(){
        // Setup our cones
        double angle_increment = M_PI/300;
        cone1 = LidarObstacle({
                                      {angle_increment*0,0.75},
                                      {angle_increment*1,0.75},
                                      {angle_increment*2,0.75},
                                      {angle_increment*3,0.75},
                                      {angle_increment*4,0.75},
                                      {angle_increment*5,0.75},
                                      {angle_increment*6,0.75},
                                      {angle_increment*7,0.75},
                              });
        cone2 = LidarObstacle({
                                      {angle_increment*88, 1.25},
                                      {angle_increment*89, 1.25},
                                      {angle_increment*90, 1.25},
                                      {angle_increment*91, 1.25},
                                      {angle_increment*92, 1.25},
                              });
        cone3 = LidarObstacle({
                                      {angle_increment*115,2.13},
                                      {angle_increment*115,2.13},
                                      {angle_increment*115,2.13},
                              });
        cone4 = LidarObstacle({
                                      {angle_increment*126,3.09},
                                      {angle_increment*127,3.09},
                              });

        // Create a fake lidar scan
        // TODO: Maybe a slightly more detailed description
        // scan1 simulates having 4 cones at 1 meter increments from the robots current location,
        // 0.75 meters to the left of the robot. Number of hits per-cone and the distance of hits
        // has been properly calculated, and should be representative for a lidar with 300 hits
        // over a 180 degree view
        ulong num_rays = 300;
        scan1.angle_min = (float)-M_PI/2;
        scan1.angle_max = (float)M_PI/2;
        scan1.angle_increment = (scan1.angle_max - scan1.angle_min)/num_rays;
        // Set all the ranges to 0 initially
        scan1.ranges = std::vector<float>(num_rays, 0);
        scan1.range_min = 0.02;
        scan1.range_max = 5.6;
        // Add the obstacles
        std::fill(scan1.ranges.begin(), scan1.ranges.begin()+8, 0.75);          // cone1
        std::fill(scan1.ranges.begin()+88, scan1.ranges.begin()+93, 1.25);      // cone2
        std::fill(scan1.ranges.begin()+115, scan1.ranges.begin()+118, 2.13);    // cone3
        std::fill(scan1.ranges.begin()+126, scan1.ranges.begin()+128, 3.09);    // cone4

        // Make some ranges outside the min and max of the scan
        std::fill(scan1.ranges.begin()+150, scan1.ranges.begin()+160, 45.123);
        std::fill(scan1.ranges.begin()+50, scan1.ranges.begin()+70, 0.01);

    }

    sensor_msgs::LaserScan scan1;
    LidarObstacle cone1, cone2, cone3, cone4;
    LidarObstacleManager obstacle_manager_1;
};

TEST_F(LidarObstacleManagerTest, minDistanceBetweenObstaclesTest){
    LidarObstacle obstacle1(M_PI/2, 1);
    LidarObstacle obstacle2(-M_PI/2, 1);

    EXPECT_DOUBLE_EQ(2, LidarObstacleManager::minDistanceBetweenObstacles(obstacle1, obstacle2));

    // Distance between cone1 and cone2 should be about a meter
    double distance = LidarObstacleManager::minDistanceBetweenObstacles(cone1, cone2);
    EXPECT_LE(0.93, distance);
    EXPECT_GE(1.07, distance);
}

TEST_F(LidarObstacleManagerTest, addObstacleTest){
    // Check that adding a single obstacle works
    obstacle_manager_1.addObstacle(cone1);
    std::vector<LidarObstacle> obstacles = obstacle_manager_1.getObstacles();
    ASSERT_EQ(1, obstacles.size());
    EXPECT_EQ(cone1.getCenter(), obstacles[0].getCenter());

    // Add a couple more obstacles and check that we've got the expected number of obstacles
    obstacle_manager_1.addObstacle(cone2);
    obstacle_manager_1.addObstacle(cone3);
    obstacle_manager_1.addObstacle(cone4);
    obstacles = obstacle_manager_1.getObstacles();
    EXPECT_EQ(4, obstacles.size());
}

TEST_F(LidarObstacleManagerTest, addDuplicateObstacleTest){
    // We should automatically merge any duplicate obstacles that we get
    obstacle_manager_1.addObstacle(cone1);
    obstacle_manager_1.addObstacle(cone1);
    EXPECT_EQ(1, obstacle_manager_1.getObstacles().size());
}

TEST_F(LidarObstacleManagerTest, addLaserScanTest){
    obstacle_manager_1.addLaserScan(scan1);
    std::vector<LidarObstacle> obstacles = obstacle_manager_1.getObstacles();

    EXPECT_EQ(4, obstacles.size());
}

TEST_F(LidarObstacleManagerTest, getPointGroupingsTest){
    std::vector<Point> points = {
            // First group
            {0,0},
            {0,1},
            {1,1},
            // Point just out of reach of the first group
            {1,2.2},
            // Second group
            {10,0},
            {11,0},
            // Random point off in the distance
            {15,15},
    };
    std::vector<std::vector<Point>> groups = obstacle_manager_1.getPointGroupings(points, 1.1);
    // We expect 4 groups of points
    ASSERT_EQ(4, groups.size());
    // Get the group sizes
    std::vector<int> sizes;
    sizes.resize(groups.size());
    std::transform(groups.begin(), groups.end(), sizes.begin(),
                   [](auto group) { return group.size(); });
    std::sort(sizes.begin(), sizes.end());
    EXPECT_EQ(std::vector<int>({1,1,2,3}), sizes);
}

TEST_F(LidarObstacleManagerTest, getLineOfBestFitPositiveSlopeTest){
    std::vector<Point> points = {
            {0,0},
            {1,0.5},
            {2,1},
    };
    LineOfBestFit line = LidarObstacleManager::getLineOfBestFit(points);
    EXPECT_DOUBLE_EQ(0, line.getXIntercept());
    EXPECT_DOUBLE_EQ(0.5, line.getSlope());
    EXPECT_DOUBLE_EQ(1, line.correlation);
}

TEST_F(LidarObstacleManagerTest, getLineOfBestFitNegativeSlopeTest){
    std::vector<Point> points = {
            {0,1},
            {1,0.5},
            {2,0},
    };
    LineOfBestFit line = LidarObstacleManager::getLineOfBestFit(points);
    EXPECT_DOUBLE_EQ(2, line.getXIntercept());
    EXPECT_DOUBLE_EQ(-0.5, line.getSlope());
    EXPECT_DOUBLE_EQ(-1, line.correlation);
}

TEST_F(LidarObstacleManagerTest, getLineOfBestFitRandomPoints){
    std::vector<Point> points = {
            {0,1},
            {1,0.5},
            {2,2},
            {3,-4},
            {3,-5},
    };
    LineOfBestFit line = LidarObstacleManager::getLineOfBestFit(points);
    EXPECT_DOUBLE_EQ(2.2352941176470589, line.getYIntercept());
    EXPECT_DOUBLE_EQ(-1.8529411764705883, line.getSlope());
    EXPECT_DOUBLE_EQ(-0.76208438348419327, line.correlation);
}

// TODO
// TEST(SlopeInterceptLineTest, testGetXIntercept){}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
