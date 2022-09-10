#include "main.h"
#include "roboto/roboto.hpp"
#include <cmath>
#include <cstdlib>
#include <string>

const float PI = 3.1415926;
int flySpeed = 0;
bool goingdown = false;
bool r1Engaged = false;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text
 * between "I was pressed!" and nothing.
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
void toggleFlySpeed(){
	if(flySpeed == 0){
		flySpeed = 1;
		flywheel = 90;
    flywheel2 = -90;
	}
	else if(flySpeed == 1 && goingDown == false){
		flySpeed = 2;
		flywheel = 50;
    flywheel2 = -50;
	}
	else if(flySpeed == 2){
		flySpeed = 1;
		goingDown = true;
		flywheel = 90;
    flywheel2 = -90;
	}
	else if(flySpeed == 1 && goingDown == true){
		flySpeed = 0;
		goingDown = false;
		flywheel = 127;
    flywheel2 = -127;
	}
}

void auton1() {
  intake.move(127);
  flywheel.move(127);
  drive(-30, 30, -315);
  pros::delay(100);
  drive(-45, 45, 35);
  pros::delay(500);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  drive(-60, 60, 45);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  drive(-75, 75, 55);
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
  drive(90, 1, 0);
  pros::delay(300);
  drive(1, 10, -330);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  drive(1, 20, -325);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  pros::delay(300);
  indexer.set_value(true);
  pros::delay(300);
  indexer.set_value(false);
  drive(0, 20, -320);
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
  rotate(-10);
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
  bool l1;
  int toggleIntake = 0;
  bool l2;
  int reverseToggleIntake = 0;
  bool r2;
  bool r1;
  flyspeed = 127;
  bool upArrow;

  while (true) {
    // driver control
    leftJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    rightJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    buttonA = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    buttonB = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    l1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    l2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    upArrow = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    downArrow = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    

    driveRightBack.move(rightJoystick);
    driveRightFront.move(rightJoystick);
    driveLeftBack.move(leftJoystick);
    driveLeftFront.move(leftJoystick);

    flywheel = 127;
    flywheel2 = -127;

    if (l1) {
      toggleIntake = 1;
    }
    if (toggleIntake == 1) {
      intake = 127;
    }
    if (l2) {
      intake = -127;
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && r1Engaged == false){
			toggleFlySpeed();
			r1Engaged = true;
		}
		else if(!master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			r1Engaged = false;
		}



    if (buttonA) {
      indexer.set_value(false);
      pros::delay(100);
      indexer.set_value(true);
    }
    pros::delay(5);
  }
}
