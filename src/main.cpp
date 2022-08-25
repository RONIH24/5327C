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

float currentXPos = 0;
float currentYPos = 0;
float pastFwdDistance = 0;
float pastLeftRightDistance = 0;



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

void rotate(float angle) {
	float positiveAngle;
	if(angle < 0) {
		positiveAngle = angle + 360;
	} else {
		positiveAngle = angle;
	}
	float error = positiveAngle - inertial_sensor.get_heading();
	float voltage;
	while(std::abs(error) > 5) {
		error = positiveAngle - inertial_sensor.get_heading();
		voltage = error * 2;
		voltage = std::abs(voltage);
		if(voltage > 127) voltage = 127;
	
		if(angle < 0) {
			driveLeftBack.move(-voltage);
			driveLeftFront.move(-voltage);
			driveRightBack.move(voltage);
			driveRightFront.move(voltage);
		} else if(angle > 0) {
			driveLeftBack.move(voltage);
			driveLeftFront.move(voltage);
			driveRightBack.move(-voltage);
			driveRightFront.move(-voltage);

	 }

	 
	}

}

void pid(double distance) {
	double error;
	double kP = 0.1;
	double kI = 0.001;
	double kD = 0.01;
	double totalError = 0;
	double lastError = distance;
	while(error > 0){	
		double averageEncoderTicks = (driveLeftBack.get_position() + driveLeftFront.get_position() + driveRightBack.get_position() + driveRightFront.get_position()) / 4;

		error = distance - averageEncoderTicks;
		double errorDifference = lastError - error;
		totalError += error;
		double motorSpeed = (error * kP) + (totalError * kI) + (errorDifference * kD);
		

		lastError = error;
	}
}



void drive(float targetX, float targetY, float targetAngle, float currentX, float currentY, float pastFwd, float pastLeftRight) {
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
		float travelX = targetX - (wheelRadius * (currentX * (PI/180)));
		float travelY = targetY - (wheelRadius * (currentY * (PI/180)));
		float driveVoltage = 0;

		float kP = 4;
		float kD = 4;
		float lastTravelDistance = 0;

		float travelDistance = sqrtf((travelX * travelX) + (travelY * travelY));

		float turnAngle = 90 - (std::abs((asinf(travelY / travelDistance)) * (180/PI)));
		if(travelY < 0 && travelX > 0) {
			turnAngle = turnAngle + 90;
		} else if(travelY < 0 && travelX < 0) {
			turnAngle = turnAngle + 180;
		} else if(travelY > 0 && travelX < 0) {
			turnAngle = turnAngle + 270;
		}

		float variable = 2;


		rotate(turnAngle);
	
		while(travelDistance > variable) {
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
			travelDistance = sqrtf(std::abs((travelX * travelX)) + std::abs((travelY * travelY)));

			if(travelX < -1) {
				variable = 11;
			}

			if(std::abs((turnAngle-inertial_sensor.get_heading())) >= 5) {
				rotate(turnAngle);
			}


			
			float errorDifference = travelDistance - lastTravelDistance;
			float motorSpeed = (travelDistance * kP) + (errorDifference * kD);

			if(motorSpeed > 127) motorSpeed = 127;
			move(motorSpeed);
			lastTravelDistance = travelDistance;

	

			controller.print(1, 0, "travel distance: %f", travelDistance);

			if(travelDistance < 6) {
				travelDistance = 0;
			}
		

		
			

			
		}
		driveStop();
		
		rotate(targetAngle);
		driveStop();
		currentXPos = posX;
		currentYPos = posY;
		pastFwdDistance = lastAvgFwd;
		pastLeftRightDistance = lastLeftRight;
		


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
	drive(50, 50, -360, currentXPos, currentYPos, pastFwdDistance, pastLeftRightDistance);
	pros::delay(1000);
	drive(1, 1, 2, currentXPos, currentYPos, pastFwdDistance, pastLeftRightDistance );

}

void auton2() {
	rotate(90);
	pros::delay(1000);
	rotate(180);
}




void autonomous() {
	auton1();
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