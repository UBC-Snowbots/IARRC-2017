/*
 * Created By: Robyn Castro
 * Created On: July 9th, 2017
 * Description: Given a line and a set of parameters, determines
 *              how the car should move in the drag race.
 */
#include "DragRaceController.h"

DragRaceController::DragRaceController(double targetDistance, bool lineToTheRight, double theta_scaling_multiplier,
                                       double angular_speed_multiplier, double linear_speed_multiplier,
                                       double angular_vel_cap, double linear_vel_cap) {
    this->target_distance = targetDistance;
    this->line_to_the_right = lineToTheRight;
    this->theta_scaling_multiplier = theta_scaling_multiplier;
    this->angular_speed_multiplier = angular_speed_multiplier;
    this->angular_vel_cap = angular_vel_cap;
    this->linear_vel_cap = linear_vel_cap;
    this->linear_speed_multiplier = linear_speed_multiplier;
}

geometry_msgs::Twist DragRaceController::determineDesiredMotion(LineOfBestFit *longestConeLine) {

    // Determine angle of line.
    double theta = atan(longestConeLine->getSlope());


    double distanceError = target_distance - determineDistanceFromLine(longestConeLine);
    if(!line_to_the_right)
        distanceError *= -1.0;

    geometry_msgs::Twist command;

    // Set components we don't care about to 0
    command.linear.y = 0;
    command.linear.z = 0;
    command.angular.x = 0;
    command.angular.y = 0;

    // Figure out how fast we should be turning
    command.angular.z = (theta_scaling_multiplier * theta + distanceError) * angular_speed_multiplier;
    printf("Distance: %f, Theta: %f\n", distanceError, theta * theta_scaling_multiplier);
    // Limit the angular velocity
    if (fabs(command.angular.z) > angular_vel_cap)
        command.angular.z = angular_vel_cap * command.angular.z / fabs(command.angular.z);


    // Figure out how fast we should be moving forward
    command.linear.x = linear_speed_multiplier / fabs(command.angular.z);

    // Limit the linear velocity
    if (command.linear.x > linear_vel_cap)
        command.linear.x = linear_vel_cap;

    return command;
}

double DragRaceController::determineDistanceFromLine(LineOfBestFit *line) {
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