#include "main.h"
#include "pros/adi.hpp"
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
pros::Motor flywheel (10, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_COUNTS);
pros::ADIDigitalOut indexer ('A', true);
pros::Motor intake(12, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_COUNTS);

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
  void move(float voltage, float angle) {
   if((inertial_sensor.get_heading() > (angle + 2)) && (inertial_sensor.get_heading() < (angle + 10))) {
		driveLeftBack.move(voltage - 20);
		driveLeftFront.move(voltage - 20);
		driveRightBack.move(voltage);
		driveRightFront.move(voltage);
	} else if((inertial_sensor.get_heading() < (angle - 2)) && (inertial_sensor.get_heading() > (angle - 10))) {
		driveLeftBack.move(voltage);
		driveLeftFront.move(voltage);
		driveRightBack.move(voltage - 20);
		driveRightFront.move(voltage - 20);
	} else if((inertial_sensor.get_heading() > (angle + 11))) {
		rotate(-angle);
	} else if((inertial_sensor.get_heading() < (angle - 11))) {
		rotate(angle);
	} else {
		driveLeftBack.move(voltage);
		driveLeftFront.move(voltage);
		driveRightBack.move(voltage);
		driveRightFront.move(voltage);
	}
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

	float variable = 2;

    rotate(turnAngle);

    while (travelDistance > variable) {
      // position tracking
      averageFwd = (rightTrackerWheel.get_position() / 100.0000 + leftTrackerWheel.get_position() / 100.0000) / 4;
      averageLeftRight = horizontalTrackerWheel.get_position() / 100.0000;
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

	  if(travelX < -1) {
				variable = 11;
			}

      travelX = targetX - posXinch;
      travelY = targetY - posYinch;
      travelDistance = sqrtf((travelX * travelX) + (travelY * travelY));
	  turnAngle =
        90 - (std::abs((asinf(travelY / travelDistance)) * (180 / PI)));
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

  pros::lcd::initialize();
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
  intake.move(127);
  flywheel.move(127);
  robot.drive(-30, 30, -315);
  pros::delay(100);
  robot.drive(-45, 45, 35);
  pros::delay(500);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  robot.drive(-60, 60, 45);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  robot.drive(-75, 75, 55);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  

}

void auton2() {
  intake.move(127);
  flywheel.move(127);
  robot.drive(90, 1, 0);
  pros::delay(300);
  robot.drive(1, 10, -330);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  robot.drive(1, 20, -325);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  robot.drive(0, 20, -320);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  


}

void auton3() {
  flywheel.move(127);
  robot.rotate(-10);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
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
  bool r1;
  int toggleFly = 0;
  bool l1;
  int toggleIntake = 0;
  bool l2;
  int reverseToggleIntake = 0;
  bool r2;



  while (true) {
    // driver control
    leftJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    rightJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    buttonA = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    buttonB = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    l1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    l2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    driveRightBack.move(rightJoystick);
    driveRightFront.move(rightJoystick);
    driveLeftBack.move(leftJoystick);
    driveLeftFront.move(leftJoystick);

    if (r1){
      toggleFly = 1;
    }
    if (toggleFly == 1){
      flywheel = 127;
    }
    if (r2){
      toggleFly = 0;
    }
    if (toggleFly == 0){
      flywheel = 0;
    }
    if (l1){
      toggleIntake = 1;
    }
    if (toggleIntake == 1){
      intake = 127;
    }
    if (l2){
      intake = -127;
    }
  
    if (buttonA){
      indexer.set_value(false);
      pros::delay(100);
      indexer.set_value(true);
    }
    pros::delay(5);
  }
}


