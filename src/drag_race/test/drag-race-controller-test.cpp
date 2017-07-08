#include <gtest/gtest.h>
#include <LidarObstacleManager.h>
#include <geometry_msgs/Twist.h>
#include <DragRaceNode.h>

/*
 * Created By: Robyn Castro
 * Created On: July 8, 2017
 * Description: Tests for Drag Race Controller
 */

// How far from the target line the robot should be
double more_than_target_distance = 0.0;
double less_than_target_distance = 10.0;

// Velocity limits
double angular_vel_cap = 1.0;
double linear_vel_cap = 1.0;

// Scaling
double theta_scaling_multiplier = 0.1;
double angular_speed_multiplier = 1.0;
double linear_speed_multiplier = 1.0;

// Line values
// Slopes
double right_angle_slope = 1.0;
double left_angle_slope = -1.0;

// X-Intercepts
double line_to_the_right = -1.0;
double line_to_the_left = 1.0;

double correlation = 1.0;

TEST(LeftLineTest, angleRightMoreThanTargetDistance) {
    LineOfBestFit *testLine = new LineOfBestFit(right_angle_slope, line_to_the_left, correlation);
    geometry_msgs::Twist testCommand = DragRaceNode::determineDesiredMotion(testLine, more_than_target_distance, false,
                                                                            theta_scaling_multiplier, angular_speed_multiplier,
                                                                            linear_speed_multiplier, angular_vel_cap, linear_vel_cap);
    EXPECT_GE(testCommand.angular.z, 0);
}

TEST(LeftLineTest, angleLeftMoreThanTargetDistance) {
    LineOfBestFit *testLine = new LineOfBestFit(left_angle_slope, line_to_the_left, correlation);
    geometry_msgs::Twist testCommand = DragRaceNode::determineDesiredMotion(testLine, more_than_target_distance, false,
                                                                            theta_scaling_multiplier, angular_speed_multiplier,
                                                                            linear_speed_multiplier, angular_vel_cap,
                                                                            linear_vel_cap);
    EXPECT_GE(testCommand.angular.z, 0);
}

TEST(LeftLineTest, angleRightLessThanTargetDistance) {
    LineOfBestFit *testLine = new LineOfBestFit(right_angle_slope, line_to_the_left, correlation);
    geometry_msgs::Twist testCommand = DragRaceNode::determineDesiredMotion(testLine, less_than_target_distance, false,
                                                                            theta_scaling_multiplier, angular_speed_multiplier,
                                                                            linear_speed_multiplier, angular_vel_cap,
                                                                            linear_vel_cap);
    EXPECT_GE(0, testCommand.angular.z);
}

TEST(LeftLineTest, angleLeftLessThanTargetDistance) {
    LineOfBestFit *testLine = new LineOfBestFit(left_angle_slope, line_to_the_left, correlation);
    geometry_msgs::Twist testCommand = DragRaceNode::determineDesiredMotion(testLine, less_than_target_distance, false,
                                                                            theta_scaling_multiplier, angular_speed_multiplier,
                                                                            linear_speed_multiplier, angular_vel_cap,
                                                                            linear_vel_cap);
    EXPECT_GE(0, testCommand.angular.z);
}

TEST(RightLineTest, angleRightMoreThanTargetDistance) {
    LineOfBestFit *testLine = new LineOfBestFit(right_angle_slope, line_to_the_right, correlation);
    geometry_msgs::Twist testCommand = DragRaceNode::determineDesiredMotion(testLine, more_than_target_distance, true,
                                                                            theta_scaling_multiplier, angular_speed_multiplier,
                                                                            linear_speed_multiplier, angular_vel_cap,
                                                                            linear_vel_cap);
    EXPECT_GE(0, testCommand.angular.z);
}

TEST(RightLineTest, angleLeftMoreThanTargetDistance) {
    LineOfBestFit *testLine = new LineOfBestFit(left_angle_slope, line_to_the_right, correlation);
    geometry_msgs::Twist testCommand = DragRaceNode::determineDesiredMotion(testLine, more_than_target_distance, true,
                                                                            theta_scaling_multiplier, angular_speed_multiplier,
                                                                            linear_speed_multiplier, angular_vel_cap,
                                                                            linear_vel_cap);
    EXPECT_GE(0, testCommand.angular.z);
}

TEST(RightLineTest, angleRightLessThanTargetDistance) {
    LineOfBestFit *testLine = new LineOfBestFit(right_angle_slope, line_to_the_right, correlation);
    geometry_msgs::Twist testCommand = DragRaceNode::determineDesiredMotion(testLine, less_than_target_distance, true,
                                                                            theta_scaling_multiplier, angular_speed_multiplier,
                                                                            linear_speed_multiplier, angular_vel_cap,
                                                                            linear_vel_cap);
    EXPECT_GE(testCommand.angular.z, 0);
}

TEST(RightLineTest, angleLeftLessThanTargetDistance) {
    LineOfBestFit *testLine = new LineOfBestFit(left_angle_slope, line_to_the_right, correlation);
    geometry_msgs::Twist testCommand = DragRaceNode::determineDesiredMotion(testLine, less_than_target_distance, true,
                                                                            theta_scaling_multiplier, angular_speed_multiplier,
                                                                            linear_speed_multiplier, angular_vel_cap,
                                                                            linear_vel_cap);
    EXPECT_GE(testCommand.angular.z, 0);
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}