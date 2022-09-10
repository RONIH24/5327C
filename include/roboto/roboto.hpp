#ifndef ROBOTO_H_
#define ROBOTO_H_

#include "main.h"
#include <iostream>

extern pros::Motor driveLeftFront;
extern pros::Motor driveLeftBack;
extern pros::Motor driveRightFront;
extern pros::Motor driveRightBack;
extern pros::IMU rotational_sensor;
extern pros::Rotation leftTrackerWheel;
extern pros::Rotation rightTrackerWheel;
extern pros::Rotation horizontalTrackerWheel;
extern pros::Imu inertial_sensor;
extern pros::Controller controller;
extern pros::Motor flywheel;
extern pros::ADIDigitalOut indexer;
extern pros::Motor intake;
extern float currentX;
extern float currentY;
extern float pastFwd;
extern float pastLeftRight;

extern void reset();
extern void driveStop();
extern void move(float voltage);
extern void rotate(float angle);
extern void drive(float targetX, float targetY, float targetAngle);
extern void rotateLeft(float targetAngle);
extern void rotateRight(float targetAngle);
extern void rotateLeftAbsolute(float turnAngle);
extern void rotateRightAbsolute(float turnAngle);

#endif
