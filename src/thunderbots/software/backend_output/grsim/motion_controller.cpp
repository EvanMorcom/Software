//
// Created by evan on 18/08/18.
//

#include "motion_controller.h"
#include "ai/world/robot.h"
#include "geom/point.h"
#include "geom/angle.h"
#include <utility>
#include <iostream>
#include <chrono>
#include <ctime>
#include "shared/constants.h"


//TODO: add negative time condition

std::pair<Vector, Angle> MotionController::grSim_bang_bang(Robot robot, Point dest, double desiredFinalSpeed, Angle desiredFinalOrientation, double deltaTime) {

    // vector to hold the XY velocities of the robot
    Vector robotXYVelocities;

    // boolean value if the robot can reach it's destination at target speed based on MAX acceleration
    bool bCanStopInTime;
    bool bCanStopRotateInTime;

    // calculate the current time (will be used to calculate the time step since the last time the motion controller was run)
    const double currentTime = std::chrono::system_clock::to_time_t( std::chrono::system_clock::now() );

    // calculate the delta in time to calculate changes in speed based on acceleration

    double robotAngularVelocity;


    double distanceToDest = (robot.position() - dest).len();

    // calculates robot angle based on unit vector that points from the robot location to the destination (used to calculate the X/Y velocity magnitudes
    Angle directionAngle = (dest - robot.position()).norm().orientation();

    double angleToDest = (robot.orientation().toRadians() - desiredFinalOrientation.toRadians());

    // calculate the expected speed at the destination based on current speed and acceleration
    // Vf = sqrt( Vi^2 - 2*a*d)
    double expectedFinalSpeed;
    double expectedFinalAngSpeed;


    // variables used to hold the change in velocities based on the maximum acceleration and the change in time since the last motion controller run
    double deltaSpeedX, deltaSpeedY, deltaAngularSpeed;

    if ( pow(robot.velocity().len(), 2) <= 2*ROBOT_MAX_ACCELERATION*distanceToDest) {
        expectedFinalSpeed = -1*sqrt(  2*ROBOT_MAX_ACCELERATION*distanceToDest - pow(robot.velocity().len(), 2) );
        bCanStopInTime = true;
    }
    else {
        expectedFinalSpeed = sqrt(  pow(robot.velocity().len(), 2) - 2*ROBOT_MAX_ACCELERATION*distanceToDest );
        bCanStopInTime = expectedFinalSpeed <= desiredFinalSpeed;
    }

    if ( pow(robot.angularVelocity().toRadians(), 2) <= fabs(2*ROBOT_MAX_ANG_ACCELERATION*angleToDest) ) {
        expectedFinalAngSpeed = -1*sqrt( fabs(2*ROBOT_MAX_ANG_ACCELERATION*angleToDest) - pow(robot.angularVelocity().toRadians() , 2) );
        bCanStopRotateInTime = true;
    }
    else {
        expectedFinalAngSpeed = sqrt( pow(robot.angularVelocity().toRadians() , 2) - 2*ROBOT_MAX_ANG_ACCELERATION*angleToDest );
        bCanStopRotateInTime = expectedFinalAngSpeed <= 0.0;
    }

    // the robot can stop in time if it's desired final speed is higher than the speed if the robot maximum decelerates from the current state


    if (deltaTime < 0) { // there should be no negative changes in time except from the first run when variables are initialized
        deltaTime = fabs(deltaTime);
    }

    if( bCanStopInTime ) {

        // if the robot can stop in time and is going slower than the desired final speed then accelerate
        if( expectedFinalSpeed < desiredFinalSpeed ) {
            deltaSpeedX = (ROBOT_MAX_ACCELERATION * deltaTime)*directionAngle.cos();
            deltaSpeedY = (ROBOT_MAX_ACCELERATION * deltaTime)*directionAngle.sin();
        }

        // if not then maintain speed
        else {
            // maintain speed
            deltaSpeedX = 0.0;
            deltaSpeedY = 0.0;
        }
    }

    // if the robot can't stop in time then decelerate
    else {
        deltaSpeedX = -(ROBOT_MAX_ACCELERATION * deltaTime)*directionAngle.cos();
        deltaSpeedY = -(ROBOT_MAX_ACCELERATION * deltaTime)*directionAngle.sin();
    }

    // if the robot can stop rotating in time
    if ( bCanStopRotateInTime ) {

        // if the final expected angle is less than the desired angle then angularly accelerate
        if( expectedFinalAngSpeed < 0) {
            // angularly accelerate
            deltaAngularSpeed = ROBOT_MAX_ANG_ACCELERATION*deltaTime;
        }

        // if not then maintain angular velocity
        else {
            deltaAngularSpeed = 0.0;
        }
    }

    // if the robot can't stop rotating in time then angular decelerate
    else {
        deltaAngularSpeed = -(ROBOT_MAX_ANG_ACCELERATION*deltaTime);
    }

    robotXYVelocities = Vector(robot.velocity().x() + deltaSpeedX, robot.velocity().y() + deltaSpeedY); // calculate new X/Y/ang velocities based on the current robot speeds and delta speeds
    robotAngularVelocity = (robot.angularVelocity().toRadians() ) + deltaAngularSpeed;                  // calculate new angular velocity based on the current robot ang. velocity and delta ang. velocity

    if(robotXYVelocities.len() > ROBOT_MAX_SPEED) {
        robotXYVelocities = robotXYVelocities.norm(ROBOT_MAX_SPEED);
    }

    if( fabs(robotAngularVelocity) > ROBOT_MAX_ANG_SPEED) {
        if( robotAngularVelocity < 0) {
            robotAngularVelocity = -1*ROBOT_MAX_ANG_SPEED;
        }
        else {
            robotAngularVelocity = ROBOT_MAX_ANG_SPEED;
        }
    }

    return std::make_pair(robotXYVelocities, Angle::ofRadians(robotAngularVelocity));
}
