#include "main.h"
#include "roboto/roboto.hpp"
#include <cmath>
#include <cstdlib>
#include <string>

#define AUTON 0

const float PI = 3.1415926;
int flySpeed = 0;
int inSpeed = 0;
bool goingDown = false;
bool r1Engaged = false;
bool r2Engaged = false;
bool l1Engaged = false;
bool l2Engaged = false;
int angleOffset;
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

 void stopHold(){
	driveLeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveLeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveRightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveRightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	driveLeftBack.move(0);
	driveLeftFront.move(0);
	driveRightBack.move(0);
	driveRightFront.move(0);
}


void stopCoast(){
	driveLeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveLeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveRightBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveRightFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	driveLeftBack.move(0);
	driveLeftFront.move(0);
	driveRightBack.move(0);
	driveRightFront.move(0);
}

void toggleFlySpeed() {
	if(flySpeed == 0){
		flySpeed = 1;
		flywheel = 127;
    flywheel2 = -127;
	}
	else if(flySpeed == 1 && goingDown == false){
		flySpeed = 2;
		flywheel = 95;
    flywheel2 = -95;
	}
	else if(flySpeed == 2){
		flySpeed = 1;
		goingDown = true;
		flywheel = 74;
    flywheel2 = -74;
	}
	else if(flySpeed == 1 && goingDown == true){
		flySpeed = 0;
		goingDown = false;
		flywheel = 127;
    flywheel2 = -127;
	}
}
void toggleIntake() {
	if(inSpeed == 0){
		inSpeed = 1;
		intake = 127;
	}
	else if(inSpeed == 1 && goingDown == false){
		inSpeed = 2;
		intake = 0;
	}
	else if(inSpeed == 2){
		inSpeed = 1;
		goingDown = true;
		intake = -127;
	}
	else if(inSpeed == 1 && goingDown == true){
		inSpeed = 0;
		goingDown = false;
		intake = 0;
	}
}
void driveForward(int speed, int distance, int goalHeading){
	goalHeading = goalHeading-angleOffset;
	if(goalHeading < 0) goalHeading += 360;

	bool flag = false;

	int startTime = pros::millis();

	int leftStartDistance = (driveLeftFront.get_position() + driveLeftBack.get_position())/2;
	int rightStartDistance = (driveRightFront.get_position() + driveRightBack.get_position())/2;

	while(flag == false){
		double actual_turn;
		double angle = inertial_sensor.get_heading();

		int leftCurrDistance = (driveLeftFront.get_position() + driveLeftBack.get_position())/2;
		int rightCurrDistance = (driveRightFront.get_position() + driveRightBack.get_position())/2;

		if((angle < goalHeading && goalHeading-angle < 180) || (angle > goalHeading && angle-goalHeading > 180)){
			//turnRight
			actual_turn = 25;
		}
		if((angle > goalHeading && angle-goalHeading < 180) || (goalHeading > angle && goalHeading-angle > 180)){
			//turnLeft
			actual_turn = -25;
		}

		pros::lcd::set_text(1, std::to_string(actual_turn));

		driveLeftFront.move(speed + actual_turn);
		//left_wheel_middle.move(speed + actual_turn);
		driveLeftBack.move(speed + actual_turn);

		driveRightFront.move(speed - actual_turn);
		//right_wheel_middle.move(speed - actual_turn);
		driveRightBack.move(speed - actual_turn);

		if(leftCurrDistance-leftStartDistance > distance && rightCurrDistance-rightStartDistance > distance){
			flag = true;
		}

		pros::delay(20);
	}
	stopHold();
}

void driveReverse(int speed, int distance, int goalHeading){
	goalHeading = goalHeading-angleOffset;
	if(goalHeading < 0) goalHeading += 360;

	int startTime = pros::millis();

	bool flag = false;

	int leftStartDistance = (driveLeftFront.get_position() + driveLeftBack.get_position())/2;
	int rightStartDistance = (driveRightFront.get_position() + driveRightBack.get_position())/2;

	double actual_turn;

	while(flag == false){
		double angle = inertial_sensor.get_heading();

		int leftCurrDistance = (driveLeftFront.get_position() + driveLeftBack.get_position())/2;
		int rightCurrDistance = (driveRightFront.get_position() + driveRightBack.get_position())/2;

		if((angle < goalHeading && goalHeading-angle < 180) || (angle > goalHeading && angle-goalHeading > 180)){
			//turnRight
			actual_turn = 25;
		}
		if((angle > goalHeading && angle-goalHeading < 180) || (goalHeading > angle && goalHeading-angle > 180)){
			//turnLeft
			actual_turn = -25;
		}

		driveLeftFront.move(-speed + actual_turn);
		//left_wheel_middle.move(-speed + actual_turn);
		driveLeftBack.move(-speed + actual_turn);

		driveRightFront.move(-speed - actual_turn);
		//right_wheel_middle.move(-speed - actual_turn);
		driveRightBack.move(-speed - actual_turn);

		if(leftStartDistance-leftCurrDistance > distance && rightStartDistance-rightCurrDistance > distance){
			flag = true;
		}

		pros::delay(20);
	}
	stopHold();
}

void turn(int speed, int goalHeading, int angleTolerance){
	goalHeading = goalHeading-angleOffset;
	if(goalHeading < 0) goalHeading += 360;

	bool flag = false;

	//Angle bounds used to calculate when to stop turning (ex. +-3)
	int upperAngleBound = goalHeading + angleTolerance;
	int lowerAngleBound = goalHeading - angleTolerance;

	long begin_time = pros::millis();

	//Special conditions (If the angle is at 0 for example)
	bool specialDown = false;
	bool specialUp = false;
	if(lowerAngleBound < 0){
		lowerAngleBound = lowerAngleBound + 360;
		specialDown = true;}
	if(upperAngleBound > 360){
		upperAngleBound = upperAngleBound - 360;
		specialUp = true;}

	while(flag == false){
		double angle_ = inertial_sensor.get_heading();

		double actual_turn = 10;

		if((angle_ < goalHeading && goalHeading-angle_ < 180) || (angle_ > goalHeading && angle_-goalHeading > 180)){
			//turnRight
			actual_turn = speed * (abs(goalHeading-angle_)/100 + 0.3);
		}
		if((angle_ > goalHeading && angle_-goalHeading < 180) || (goalHeading > angle_ && goalHeading-angle_ > 180)){
			//turnLeft
			actual_turn = -speed * (abs(goalHeading-angle_)/100 + 0.3);
		}

		driveLeftFront.move(actual_turn);
		//left_wheel_middle.move(actual_turn);
		driveLeftBack.move(actual_turn);

		driveRightFront.move(-actual_turn);
		//right_wheel_middle.move(-actual_turn);
		driveRightBack.move(-actual_turn);

		if(!((specialDown == true && ((angle_ <  lowerAngleBound && angle_ > goalHeading+180) || (angle_ > upperAngleBound && angle_ < goalHeading+180))) ||
		(specialUp == true && ((angle_ > upperAngleBound && angle_ < goalHeading-180) || (angle_ < lowerAngleBound && angle_ > goalHeading-180))) ||
		((specialUp == false && specialDown == false) && (angle_ > upperAngleBound || angle_ < lowerAngleBound)))){
			flag = true;
		}
		pros::delay(20);
	}

	stopHold();
}

// void auton1() {
//   intake.move(127);
//   flywheel.move(127);
//   drive(-30, 30, -315);
//   pros::delay(100);
//   drive(-45, 45, 35);
//   pros::delay(500);
//   indexer.set_value(true);
//   pros::delay(300);
//   indexer.set_value(false);
//   pros::delay(300);
//   indexer.set_value(true);
//   pros::delay(300);
//   indexer.set_value(false);
//   pros::delay(300);
//   drive(-60, 60, 45);
//   indexer.set_value(true);
//   pros::delay(300);
//   indexer.set_value(false);
//   pros::delay(300);
//   indexer.set_value(true);
//   pros::delay(300);
//   indexer.set_value(false);
//   pros::delay(300);
//   drive(-75, 75, 55);
//   indexer.set_value(true);
//   pros::delay(300);
//   indexer.set_value(false);
//   pros::delay(300);
//   indexer.set_value(true);
//   pros::delay(300);
//   indexer.set_value(false);
//   pros::delay(300);
// }

// void auton2() {
//   intake.move(127);
//   flywheel.move(127);
//   drive(90, 1, 0);
//   pros::delay(300);
//   drive(1, 10, -330);
//   indexer.set_value(true);
//   pros::delay(300);
//   indexer.set_value(false);
//   pros::delay(300);
//   indexer.set_value(true);
//   pros::delay(300);
//   indexer.set_value(false);
//   drive(1, 20, -325);
//   indexer.set_value(true);
//   pros::delay(300);
//   indexer.set_value(false);
//   pros::delay(300);
//   indexer.set_value(true);
//   pros::delay(300);
//   indexer.set_value(false);
//   drive(0, 20, -320);
//   indexer.set_value(true);
//   pros::delay(300);
//   indexer.set_value(false);
//   pros::delay(300);
//   indexer.set_value(true);
//   pros::delay(300);
//   indexer.set_value(false);
// }

// void auton3() {
//   flywheel.move(127);
//   rotate(-10);
//   indexer.set_value(true);
//   pros::delay(300);
//   indexer.set_value(false);
// }

void autonomous() {
  	if(AUTON == 0){
		angleOffset = 0;

    turn(127, 270, 1);
	}

	else if(AUTON == 1){
		angleOffset = 0;

	}
}

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
  bool l2;

  while (true) {
    controller.print(1, 0, "flywheel Velocity: %d", flywheel.get_voltage());
    // driver control
    leftJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    rightJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    buttonA = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    buttonB = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    l2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    

    driveRightBack.move(rightJoystick);
    driveRightFront.move(rightJoystick);
    driveLeftBack.move(leftJoystick);
    driveLeftFront.move(leftJoystick);

    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && r1Engaged == false){
			toggleFlySpeed();
			r1Engaged = true;
		}
		else if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			r1Engaged = false;
		}

    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && l1Engaged == false){
			toggleIntake();
			l1Engaged = true;
		}
		else if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			l1Engaged = false;
		}

    if(l2){
      intake = -35;
    }



    if (buttonA) {
      indexer.set_value(true);
      pros::delay(100);
      indexer.set_value(false);
    }
    pros::delay(5);
  }
}
