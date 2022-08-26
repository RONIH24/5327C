#include "main.h"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include <cmath>
#include <cstdlib>
#include <string>

#define PI 3.1415926

pros::Motor driveLeftFront(9, pros::E_MOTOR_GEARSET_18, 0,
                           pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveLeftBack(21, pros::E_MOTOR_GEARSET_18, 0,
                          pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveRightFront(17, pros::E_MOTOR_GEARSET_18, 1,
                            pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveRightBack(11, pros::E_MOTOR_GEARSET_18, 1,
                           pros::E_MOTOR_ENCODER_COUNTS);
// pros::IMU rotational_sensor(5);
pros::Rotation leftTrackerWheel(10);
pros::Rotation rightTrackerWheel(14);
pros::Rotation horizontalTrackerWheel(20);
pros::Imu inertial_sensor(8);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

class Roboto {
public:
  float currentX;
  float currentY;
  float pastFwd;
  float pastLeftRight;

  Roboto(float currentX = 0, float currentY = 0, float pastFwd = 0,
         float pastLeftRight = 0) {
    this->currentX = currentX;
    this->currentY = currentY;
    this->pastFwd = pastFwd;
    this->pastLeftRight = pastLeftRight;
  }

  void driveStop() {
    driveLeftBack.brake();
    driveLeftFront.brake();
    driveRightBack.brake();
    driveRightFront.brake();
  }
  void move(float voltage) {
    driveLeftBack.move(voltage);
    driveLeftFront.move(voltage);
    driveRightBack.move(voltage);
    driveRightFront.move(voltage);
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

    rotate(turnAngle);

    while (travelDistance > 5) {
      // position tracking
      averageFwd = ((leftTrackerWheel.get_position() / 100.000) +
                    rightTrackerWheel.get_position() / 100.000) /
                   2.0000;
      averageLeftRight = (horizontalTrackerWheel.get_position() / 100.000);
      angle = inertial_sensor.get_heading();
      // // coordinate tracking
      posX = posX -
             (((averageFwd - lastAvgFwd) * -sin(angle * PI / 180)) +
              (averageLeftRight - lastLeftRight) * -cos(angle * (PI / 180)));
      posY =
          posY + (((averageFwd - lastAvgFwd) * cos(angle * PI / 180)) -
                  (averageLeftRight - lastLeftRight) * sin(angle * (PI / 180)));
      lastAvgFwd = averageFwd;
      lastLeftRight = averageLeftRight;

      posXinch = (wheelRadius * (posX * (PI / 180)));
      posYinch = (wheelRadius * (posY * (PI / 180)));

      travelX = targetX - posXinch;
      travelY = targetY - posYinch;
      travelDistance = sqrtf((travelX * travelX) + (travelY * travelY));

      float errorDifference = travelDistance - lastTravelDistance;
      float motorSpeed = (travelDistance * kP) + (errorDifference * kD);

      if (motorSpeed > 127)
        motorSpeed = 127;
      move(motorSpeed);
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
};
Roboto robot;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  driveLeftBack.tare_position();
  driveLeftFront.tare_position();
  driveRightBack.tare_position();
  driveRightFront.tare_position();
  driveRightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveRightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveLeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveLeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  inertial_sensor.reset();
  leftTrackerWheel.reset();
  rightTrackerWheel.reset();
  leftTrackerWheel.reset_position();
  rightTrackerWheel.reset_position();
  horizontalTrackerWheel.reset();
  pros::lcd::initialize();
}

void pid(double distance) {
  double error;
  double kP = 0.1;
  double kI = 0.001;
  double kD = 0.01;
  double totalError = 0;
  double lastError = distance;
  while (error > 0) {
    double averageEncoderTicks =
        (driveLeftBack.get_position() + driveLeftFront.get_position() +
         driveRightBack.get_position() + driveRightFront.get_position()) /
        4;

    error = distance - averageEncoderTicks;
    double errorDifference = lastError - error;
    totalError += error;
    double motorSpeed =
        (error * kP) + (totalError * kI) + (errorDifference * kD);

    lastError = error;
  }
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void auton1() {
  robot.drive(25, 25, -2.5);
  controller.print(1, 0, "currentxpos: %f", robot.currentX);
  pros::delay(5000);
  robot.drive(35, 35, -3.5);
  controller.print(1, 0, "currentxpos: %f", robot.currentX);
  controller.print(2, 0, "currentypos: %f", robot.currentY);
  pros::delay(5000);
  robot.drive(0, 0, 0);
}

void auton2() {
  robot.rotate(90);
  pros::delay(1000);
  robot.rotate(-90);
}

void autonomous() { auton1(); }

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

  float leftJoystick;
  float rightJoystick;
  bool buttonA;
  bool buttonB;

  // odom declarations
  float leftInches;
  float rightInches;
  float averageInches;
  float wheelRadius = 1.375;
  float sL = 2;
  float sR = 2;
  float sS = 3.00000;
  float tR = 0;
  float tL = 0;
  float posX = 0;
  float posY = 0;
  float angle;
  float pastAngle = 0;
  float lastLeftTracker;
  float lastRightTracker;
  float lastBackTracker;
  float lastAvgFwd = 0;
  float lastLeftRight = 0;

  while (true) {
    // driver control
    leftJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    rightJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    buttonA = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    buttonB = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    driveRightBack.move(rightJoystick);
    driveRightFront.move(rightJoystick);
    driveLeftBack.move(leftJoystick);
    driveLeftFront.move(leftJoystick);

    // position tracking
    angle = inertial_sensor.get_heading();
    // if(angle > 360) angle = angle - 360;
    // if(angle < 0) {
    // 	angle = angle + 360;
    // }
    float averageFwd = ((leftTrackerWheel.get_position() / 100.000) +
                        rightTrackerWheel.get_position() / 100.000) /
                       2.0000;
    float averageLeftRight = (horizontalTrackerWheel.get_position() / 100.000);

    // coordinate tracking
    posX =
        posX - (((averageFwd - lastAvgFwd) * -sin(angle * PI / 180)) +
                (averageLeftRight - lastLeftRight) * -cos(angle * (PI / 180)));
    posY =
        posY + (((averageFwd - lastAvgFwd) * cos(angle * PI / 180)) -
                (averageLeftRight - lastLeftRight) * sin(angle * (PI / 180)));

    // controller.print(1, 0, "Position X: %f", (wheelRadius * (posX *
    // (PI/180)))); controller.print(2, 0, "Position Y: %f", (wheelRadius *
    // (posY * (PI/180))));

    controller.print(1, 0, "angle: %f", (angle));

    lastAvgFwd = averageFwd;
    lastLeftRight = averageLeftRight;

    lastLeftTracker = leftTrackerWheel.get_position() / 100.000;
    lastRightTracker = rightTrackerWheel.get_position() / 100.000;
    lastBackTracker = horizontalTrackerWheel.get_position() / 100.000;

    pros::delay(1);
  }
}
