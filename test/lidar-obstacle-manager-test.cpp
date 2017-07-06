/*
 * Created By: Gareth Ellis
 * Created On: July 4, 2017
 * Description: TODO
 */

#include <LidarObstacleManager.h>
#include <gtest/gtest.h>

class LidarObstacleManagerTest : public testing::Test {
protected:
    virtual void SetUp(){
        /* TODO: delete me
        readings1 = {{4,44}, {3,99}, {6,2}};
        readings2 = {{-1,2}, {10,4}, {-15,6}};

        obstacle1 = LidarObstacle(0.1, 10);
        obstacle2 = LidarObstacle(0.2, 20);
        obstacle3 = LidarObstacle(0.15, 15);
        obstacle4 = LidarObstacle(readings1);
        obstacle5 = LidarObstacle(readings2);
        */
    }

    /* TODO: delete me
    std::vector<Reading> readings1, readings2;
    LidarObstacle obstacle1, obstacle2, obstacle3, obstacle4, obstacle5;
    */
};

/* TODO: delete me
TEST_F(LidarObstacleManagerTest, ConstructorTest1){
    auto readings = obstacle1.getAllLaserReadings();
    EXPECT_EQ(1, readings.size());
    EXPECT_EQ(10, readings[0].range);
    EXPECT_NEAR(0.1, readings[0].angle, 0.000001);
}
*/

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
