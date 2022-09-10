#include "main.h"
#include <cmath>
#include <cstdlib>
#include <string>

const float PI = 3.1415926;

pros::Motor driveLeftFront(19, pros::E_MOTOR_GEARSET_06, 1,
                           pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveLeftBack(20, pros::E_MOTOR_GEARSET_06, 1,
                          pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveRightFront(17, pros::E_MOTOR_GEARSET_06, 0,
                            pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveRightBack(18, pros::E_MOTOR_GEARSET_06, 0,
                           pros::E_MOTOR_ENCODER_COUNTS);
pros::IMU rotational_sensor(5);
pros::Rotation leftTrackerWheel(9);
pros::Rotation rightTrackerWheel(14);
pros::Rotation horizontalTrackerWheel(6);
pros::Imu inertial_sensor(6);
pros::Motor flywheel(10, pros::E_MOTOR_GEARSET_06, 0,
                     pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor flywheel2(11, pros::E_MOTOR_GEARSET_06, 0,
                     pros::E_MOTOR_ENCODER_COUNTS);
pros::ADIDigitalOut indexer('A', true);
pros::Motor intake(12, pros::E_MOTOR_GEARSET_06, 0,
                   pros::E_MOTOR_ENCODER_COUNTS);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

float currentX = 0;
float currentY = 0;
float pastLeftRight = 0;
float pastFwd = 0;

void driveStop() {
  driveLeftBack.brake();
  driveLeftFront.brake();
  driveRightBack.brake();
  driveRightFront.brake();
}
void rotate(float angle) {
  inertial_sensor.tare();
  float positiveAngle;
  if (angle < 0) {
    positiveAngle = angle + 360;
  } else {
    positiveAngle = angle;
  }
  float error = positiveAngle - inertial_sensor.get_heading();
  float voltage;
  while (std::abs(error) > 4) {
    error = positiveAngle - inertial_sensor.get_heading();
    voltage = error * 2;
    voltage = std::abs(voltage);
    if (voltage > 127)
      voltage = 127;

    if (angle < 0) {
      driveLeftBack.move(-voltage);
      driveLeftFront.move(-voltage);
      driveRightBack.move(voltage);
      driveRightFront.move(voltage);
    } else if (angle > 0) {
      driveLeftBack.move(voltage);
      driveLeftFront.move(voltage);
      driveRightBack.move(-voltage);
      driveRightFront.move(-voltage);
    }
  }
}
void rotateLeft(float targetAngle) {
  float error = targetAngle - inertial_sensor.get_heading();
  float voltage;
  error = fmod(error, 360);
  while (error > 2) {
    error = targetAngle - inertial_sensor.get_heading();
    error = fmod(error, 360);
    voltage = error * 1.5;
    if (voltage > 127)
      voltage = 127;
    driveLeftBack.move(-voltage);
    driveLeftFront.move(-voltage);
    driveRightBack.move(voltage);
    driveRightFront.move(voltage);
  }
  driveStop();
}

void rotateRight(float targetAngle) {
  float error = targetAngle - inertial_sensor.get_heading();
  float voltage;
  error = fmod(error, 360);
  while (error > 2) {
    error = targetAngle - inertial_sensor.get_heading();
    error = fmod(error, 360);
    voltage = error * 1.5;
    if (voltage > 127)
      voltage = 127;
    driveLeftBack.move(voltage);
    driveLeftFront.move(voltage);
    driveRightBack.move(-voltage);
    driveRightFront.move(-voltage);
  }
  driveStop();
}

void rotateLeftAbsolute(float turnAngle) {
  float startingAngle = inertial_sensor.get_heading();
  float endingAngle = startingAngle - turnAngle;
  if (endingAngle > 360)
    endingAngle = endingAngle - 360;
  if (endingAngle < 0)
    endingAngle = endingAngle + 360;
  rotateLeft(endingAngle);
}

void rotateRightAbsolute(float turnAngle) {
  float startingAngle = inertial_sensor.get_heading();
  float endingAngle = startingAngle + turnAngle;
  if (endingAngle > 360)
    endingAngle = endingAngle - 360;
  if (endingAngle < 0)
    endingAngle = endingAngle + 360;
  rotateRight(endingAngle);
}
void move(float voltage, float angle) {
  if ((inertial_sensor.get_heading() > (angle + 2)) &&
      (inertial_sensor.get_heading() < (angle + 10))) {
    driveLeftBack.move(voltage - 20);
    driveLeftFront.move(voltage - 20);
    driveRightBack.move(voltage);
    driveRightFront.move(voltage);
  } else if ((inertial_sensor.get_heading() < (angle - 2)) &&
             (inertial_sensor.get_heading() > (angle - 10))) {
    driveLeftBack.move(voltage);
    driveLeftFront.move(voltage);
    driveRightBack.move(voltage - 20);
    driveRightFront.move(voltage - 20);
  } else if ((inertial_sensor.get_heading() > (angle + 11))) {
    rotate(-angle);
  } else if ((inertial_sensor.get_heading() < (angle - 11))) {
    rotate(angle);
  } else {
    driveLeftBack.move(voltage);
    driveLeftFront.move(voltage);
    driveRightBack.move(voltage);
    driveRightFront.move(voltage);
  }
}
void drive(float targetX, float targetY, float targetAngle) {
  float angle = inertial_sensor.get_heading();
  float posX = currentX;
  float wheelRadius = 1.375;
  float posY = currentY;
  float posXinch = 0;
  float posYinch = 0;
  float averageFwd = 0;
  float averageLeftRight = 0;
  float lastAvgFwd = pastFwd;
  float lastLeftRight = pastLeftRight;
  float angleError = 0;
  float turnSpeed = 0;
  float travelX = targetX - (wheelRadius * (currentX * (PI / 180)));
  float travelY = targetY - (wheelRadius * (currentY * (PI / 180)));
  float driveVoltage = 0;

  float kP = 4;
  float kD = 4;
  float lastTravelDistance = 0;

  float travelDistance = sqrtf((travelX * travelX) + (travelY * travelY));

  float turnAngle =
      90 - (std::abs((asinf(travelY / travelDistance)) * (180 / PI)));
  if (travelY < 0 && travelX > 0) {
    turnAngle = turnAngle + 90;
  } else if (travelY < 0 && travelX < 0) {
    turnAngle = turnAngle + 180;
  } else if (travelY > 0 && travelX < 0) {
    turnAngle = turnAngle + 270;
  }

  float variable = 2;

  rotate(turnAngle);

  while (travelDistance > variable) {
    // position tracking
    averageFwd = (rightTrackerWheel.get_position() / 100.0000 +
                  leftTrackerWheel.get_position() / 100.0000) /
                 4;
    averageLeftRight = horizontalTrackerWheel.get_position() / 100.0000;
    angle = inertial_sensor.get_heading();
    // // coordinate tracking
    posX =
        posX - (((averageFwd - lastAvgFwd) * -sin(angle * PI / 180)) +
                (averageLeftRight - lastLeftRight) * -cos(angle * (PI / 180)));
    posY =
        posY + (((averageFwd - lastAvgFwd) * cos(angle * PI / 180)) -
                (averageLeftRight - lastLeftRight) * sin(angle * (PI / 180)));
    lastAvgFwd = averageFwd;
    lastLeftRight = averageLeftRight;

    posXinch = (wheelRadius * (posX * (PI / 180)));
    posYinch = (wheelRadius * (posY * (PI / 180)));

    if (travelX < -1) {
      variable = 11;
    }

    travelX = targetX - posXinch;
    travelY = targetY - posYinch;
    travelDistance = sqrtf((travelX * travelX) + (travelY * travelY));
    turnAngle = 90 - (std::abs((asinf(travelY / travelDistance)) * (180 / PI)));
    if (travelY < 0 && travelX > 0) {
      turnAngle = turnAngle + 90;
    } else if (travelY < 0 && travelX < 0) {
      turnAngle = turnAngle + 180;
    } else if (travelY > 0 && travelX < 0) {
      turnAngle = turnAngle + 270;
    }

    float errorDifference = travelDistance - lastTravelDistance;
    float motorSpeed = (travelDistance * kP) + (errorDifference * kD);

    if (motorSpeed > 127)
      motorSpeed = 127;
    move(motorSpeed, turnAngle);
    lastTravelDistance = travelDistance;
  }
  driveStop();

  rotate(targetAngle - inertial_sensor.get_heading());
  driveStop();
  currentX = posX;
  currentY = posY;
  pastFwd = lastAvgFwd;
  pastLeftRight = lastLeftRight;
}
