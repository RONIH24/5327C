#include "main.h"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/screen.h"
#include <string>

#define PI 3.1415926


pros::Motor driveLeftFront(9, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveLeftBack(21, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveRightFront(17, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveRightBack(11, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_COUNTS);
// pros::IMU rotational_sensor(5);
pros::Rotation leftTrackerWheel(10);
pros::Rotation rightTrackerWheel(14);
pros::Rotation horizontalTrackerWheel(20);
pros::Imu inertial_sensor(8);

pros::Controller controller(pros::E_CONTROLLER_MASTER);


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
	inertial_sensor.tare();
	leftTrackerWheel.reset();
	rightTrackerWheel.reset();
	leftTrackerWheel.reset_position();
	rightTrackerWheel.reset_position();
	horizontalTrackerWheel.reset();
	pros::lcd::initialize();


	
}

void drive(float targetX, float targetY) {

}

void driveStop() {
	driveLeftBack.brake();
	driveLeftFront.brake();
	driveRightBack.brake();
	driveRightFront.brake();
}



void turnLeft(float targetAngle) {
	double error = targetAngle - inertial_sensor.get_heading();
	if(error > 360) {
		error = error - 360;
	} else if(error < 0) {
		error = error + 360;
	}
	while(error > 5) {
		error = targetAngle - inertial_sensor.get_heading();
		driveLeftBack.move(-error);
		driveLeftFront.move(-error);
		driveRightBack.move(error);
		driveRightFront.move(error);
	}
}

void turnRight(float targetAngle) {
	double error = targetAngle - inertial_sensor.get_heading();
	if(error > 360) {
		error = error - 360;
	} else if(error < 0) {
		error = error + 360;
	}
	while(error > 5) {
		error = targetAngle - inertial_sensor.get_heading();
		driveLeftBack.move(error);
		driveLeftFront.move(error);
		driveRightBack.move(-error);
		driveRightFront.move(-error);
	}
}
	
	




void odom() {
	
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
void autonomous() {
	turnRight(90);
	driveStop();
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
	float deltaL;
	float deltaR;
	float deltaS;
	float deltaAngle;
	float angle;
	float pastAngle = 0;
	float distanceLY;
	float lastLeftTracker;
	float lastRightTracker;
	float lastBackTracker;
	float deltaAngleRad;

	while(true) {
		// driver control
		leftJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		rightJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		buttonA = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
		buttonB = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

		driveRightBack.move(rightJoystick);
		driveRightFront.move(rightJoystick);
		driveLeftBack.move(leftJoystick);
		driveLeftFront.move(leftJoystick);

		//position tracking
		// deltaL = (wheelRadius * ((leftTrackerWheel.get_position() / 100.000000) * (PI/180)));
		// deltaR = (wheelRadius * ((rightTrackerWheel.get_position() / 100.000000) * (PI/180)));
		// deltaS = (wheelRadius * ((horizontalTrackerWheel.get_position() / 100.0000) * (PI/180)));


		// deltaAngle = ((deltaL - deltaR) / (sL + sR)) * (180/PI);
		// deltaAngleRad = ((deltaL - deltaR) / (sL + sR));

		angle = inertial_sensor.get_heading();
		if(angle > 360) angle = 0;
		if(angle < 0) {
			angle = angle + 360;
		}
		float averageFwd = ((leftTrackerWheel.get_position() / 100.000) + rightTrackerWheel.get_position() / 100.000) / 2.0000;
		float averageLeftRight = (horizontalTrackerWheel.get_position() / 100.000);

		// anshuls version
		posX = posX - (((averageFwd) * -sin(angle * PI/180)) + (averageLeftRight) * -cos(angle * (PI/180)));
		posY = posY + (((averageFwd) * cos(angle * PI/180)) - (averageLeftRight) * cos(angle * (PI/180)));

		// controller.print(1, 0, "Position Y: %f", (wheelRadius * (posY * (PI/180))));
		// controller.print(2, 0, "Position X: %f", (wheelRadius * (posX * (PI/180))));
		controller.print(1, 0, "angle: %f" , (angle));

		if(buttonB) {
			turnRight(90);
		}



		lastLeftTracker = leftTrackerWheel.get_position() / 100.000;
		lastRightTracker = rightTrackerWheel.get_position() / 100.000;
		lastBackTracker = horizontalTrackerWheel.get_position() / 100.000;
		
		leftTrackerWheel.reset_position();
		rightTrackerWheel.reset_position();
		horizontalTrackerWheel.reset_position();
		
		pros::delay(5);

	}

	

	
}