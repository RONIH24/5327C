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
	leftTrackerWheel.reset();
	rightTrackerWheel.reset();
	leftTrackerWheel.reset_position();
	rightTrackerWheel.reset_position();
	horizontalTrackerWheel.reset();
	pros::lcd::initialize();


	
}



void driveStop() {
	driveLeftBack.brake();
	driveLeftFront.brake();
	driveRightBack.brake();
	driveRightFront.brake();
}

void rotateLeft(float targetAngle) {
	float error = targetAngle - inertial_sensor.get_heading();
	float voltage;
	if(error > 360) {
		error = error - 360;
	} else if(error < 0) {
		error = error + 360;
	}
	while(error > 2) {
		error = targetAngle - inertial_sensor.get_heading();
		if(error > 360) {
			error = error - 360;
		} else if(error < 0) {
			error = error + 360;
		}
		voltage = error * 1.5;
		if(voltage > 127) voltage = 127;
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
	if(error > 360) {
		error = error - 360;
	} else if(error < 0) {
		error = error + 360;
	}
	while(error > 2) {
		error = targetAngle - inertial_sensor.get_heading();
		if(error > 360) {
			error = error - 360;
		} else if(error < 0) {
			error = error + 360;
		}
		voltage = error * 1.5;
		if(voltage > 127) voltage = 127;
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
	if(endingAngle > 360) endingAngle = endingAngle - 360;
	if(endingAngle < 0) endingAngle = endingAngle + 360;
	rotateLeft(endingAngle);
	
}

void rotateRightAbsolute(float turnAngle) {
	float startingAngle = inertial_sensor.get_heading();
	float endingAngle = startingAngle + turnAngle;
	if(endingAngle > 360) endingAngle = endingAngle - 360;
	if(endingAngle < 0) endingAngle = endingAngle + 360;
	rotateRight(endingAngle);
}

void move(float voltage) {
	driveLeftBack.move(voltage);
	driveLeftFront.move(voltage);
	driveRightBack.move(voltage);
	driveRightFront.move(voltage);
}



void drive(float targetX, float targetY, float targetAngle, float currentX, float currentY, float pastFwd, float pastLeftRight) {
		float angle = inertial_sensor.get_heading();
		if(angle > 360) angle = angle - 360;
		if(angle < 0) angle = angle + 360;

		float posX = currentX;
		float posY = currentY;
		float posXinch;
		float posYinch;
		float averageFwd;
		float averageLeftRight;
		float lastAvgFwd = pastFwd;
		float lastLeftRight = pastLeftRight;
		float angleError;
		float turnSpeed;
		float travelX = targetX - currentX;
		float travelY = targetY - currentY;
		float driveVoltage;
		float wheelRadius = 1.375;

		float travelDistance = sqrtf((travelX * travelX) + (travelY * travelY));

		float turnAngle = 90 - (std::abs((asinf(travelY / travelDistance)) * (180/PI)));
		if(travelY < 0 && travelX > 0) {
			turnAngle = turnAngle + 90;
		} else if(travelY < 0 && travelX < 0) {
			turnAngle = turnAngle + 180;
		} else if(travelY > 0 && travelX < 0) {
			turnAngle = turnAngle + 270;
		}

		if((angle - turnAngle) > 180) {
			rotateRight(turnAngle);
		} else if((angle - turnAngle) <= 180) {
			rotateLeft(turnAngle);
		}
	
		while((std::abs(targetX - posX) > 2) && (std::abs(targetY - posY) > 2)) {
			// position tracking
			averageFwd = ((leftTrackerWheel.get_position() / 100.000) + rightTrackerWheel.get_position() / 100.000) / 2.0000;
			averageLeftRight = (horizontalTrackerWheel.get_position() / 100.000);
			angle = inertial_sensor.get_heading();
			// // coordinate tracking
			posX = posX - (((averageFwd - lastAvgFwd) * -sin(angle * PI/180)) + (averageLeftRight - lastLeftRight) * -cos(angle * (PI/180)));
			posY = posY + (((averageFwd - lastAvgFwd) * cos(angle * PI/180)) - (averageLeftRight - lastLeftRight) * sin(angle * (PI/180)));
			lastAvgFwd = averageFwd;
			lastLeftRight = averageLeftRight;

			posXinch = (wheelRadius * (posX * (PI/180)));
			posYinch = (wheelRadius * (posY * (PI/180)));
			travelX = targetX - posXinch;
			travelY = targetY - posYinch;

			travelDistance = sqrtf((travelX * travelX) + (travelY * travelY));
			driveVoltage = travelDistance * 5;
			if(driveVoltage > 127) driveVoltage = 127;
			move(driveVoltage);
		

			
		}

		if((std::abs(targetX - posXinch) <= 5) && (std::abs(targetY - posYinch) <= 5)) {
			angle = inertial_sensor.get_heading();
			angleError = targetAngle - angle;
			if(angleError > 0) {
				rotateRight(targetAngle);
		
			}
			if(angleError < 0) {
				rotateLeft(targetAngle);
			}
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
void autonomous() {
	// rotateRight(90);
	// pros::delay(100);
	// rotateRight(90);
	pros::delay(2000);
	rotateLeft(180);

	// pros::delay(200);
	// rotateLeftAbsolute(90);
	// pros::delay(100);
	// rotateRightAbsolute(180);
	
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
	float angle;
	float pastAngle = 0;
	float lastLeftTracker;
	float lastRightTracker;
	float lastBackTracker;
	float lastAvgFwd = 0;
	float lastLeftRight = 0;
	

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
		angle = inertial_sensor.get_heading();
		// if(angle > 360) angle = angle - 360;
		// if(angle < 0) {
		// 	angle = angle + 360;
		// }
		float averageFwd = ((leftTrackerWheel.get_position() / 100.000) + rightTrackerWheel.get_position() / 100.000) / 2.0000;
		float averageLeftRight = (horizontalTrackerWheel.get_position() / 100.000);

		// coordinate tracking
		posX = posX - (((averageFwd - lastAvgFwd) * -sin(angle * PI/180)) + (averageLeftRight - lastLeftRight) * -cos(angle * (PI/180)));
		posY = posY + (((averageFwd - lastAvgFwd) * cos(angle * PI/180)) - (averageLeftRight - lastLeftRight) * sin(angle * (PI/180)));

		// controller.print(1, 0, "Position X: %f", (wheelRadius * (posX * (PI/180))));
		// controller.print(2, 0, "Position Y: %f", (wheelRadius * (posY * (PI/180))));

		controller.print(1, 0, "angle: %f" , (angle));

		lastAvgFwd = averageFwd;
		lastLeftRight = averageLeftRight;
		
		lastLeftTracker = leftTrackerWheel.get_position() / 100.000;
		lastRightTracker = rightTrackerWheel.get_position() / 100.000;
		lastBackTracker = horizontalTrackerWheel.get_position() / 100.000;
		
		
		pros::delay(1);

	}

	

	
}