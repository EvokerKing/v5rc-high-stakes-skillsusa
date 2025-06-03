#include "../include/main.h"
#include <cmath>
#include "lemlib/api.hpp"

// initialize robot
pros::MotorGroup left_motors({ -1, 2, -3 }, pros::MotorGearset::green); // setup a variable to store the controls for the left side drivetrain motors
pros::MotorGroup right_motors({ 4, -5, 6 }, pros::MotorGearset::green); // setup a variable to store the controls for the right side drivetrain motors
lemlib::Drivetrain drivetrain( // create a drivetrain object that declares the following:
	&left_motors, // the left motor group
	&right_motors, // the right motor group
	12.75, // track width in inches
	lemlib::Omniwheel::NEW_4, // wheel type and diameter
	320, // calculated drivetrain RPM
	4 //TODO: figure out horizontal drift, 2 is recommended for all omni, 8 is recommended for all traction
	//         higher value means faster but overshoots more on turns
);
pros::Imu imu(11); // setup a variable to store the inertial sensor
lemlib::OdomSensors sensors( // create an odom object that declares the following:
	nullptr, // no tracking wheels, same for all nullptrs
	nullptr,
	nullptr,
	nullptr,
	&imu // inertial sensor
);
//TODO: all of pid stuff
lemlib::ControllerSettings lateral_controller(
	0, // proportional gain (kP)
	0, // integral gain (kI)
	0, // derivative gain (kD)
	0, // anti windup
	0, // small error range in inches
	0, // small error range timeout in milliseconds
	0, // large error range in inches
	0, // large error range timeout in milliseconds
	0 // maximum acceleration (slew)
);
lemlib::ControllerSettings angular_controller(
	0, // proportional gain (kP)
	0, // integral gain (kI)
	0, // derivative gain (kD)
	0, // anti windup
	0, // small error range in inches
	0, // small error range timeout in milliseconds
	0, // large error range in inches
	0, // large error range timeout in milliseconds
	0 // maximum acceleration (slew)
);
lemlib::Chassis chassis( // create a chassis object that declares the following:
	drivetrain, // the drivetrain controls
	lateral_controller, // pid lateral information
	angular_controller, // pid angular information
	sensors // drivetrain sensors
);
pros::Controller controller(pros::E_CONTROLLER_MASTER); // get the paired controller
pros::Motor conveyor(-10); // set the motor for the conveyor to port 10 reversed
pros::adi::DigitalOut clamp(1); // set the pneumatics solenoid controlling the clamp

// curve settings
float sens = 1.02;
int deadzone = 1;
int minimum = 3;
// function to apply input curve to an integer
int curve(const int start) {
	if (abs(start) <= deadzone) { // return 0 if within deadzone
		return 0;
	}
	const int g = abs(start) - deadzone; // get absolute value of number and remove deadzone
	// get a multiply value for if the number is negative or positive
	int sign;
	if (start < 0) {
		sign = -1;
	} else {
		sign = 1;
	}
	const int i = pow(sens, (g - 127)) * g * sign; // exponential equation of sensitivity to the absolute value minus the maximum value, then multiply it by the absolute value and the sign
	const int k = (127 - minimum) / 127 * (i * (127 / i)) + minimum * sign; // get the final value from the total possible values and i variable and add the minimum
	return k;
}

void initialize() {
	pros::lcd::initialize(); // initialize the robot screen
	chassis.calibrate(); // calibrate the sensors
	pros::Task screen_task([&]() { // display robot information on screen
		while (true) { // forever loop
			// print each:
			pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
			pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
			pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
			pros::delay(20); // every 20 milliseconds
		}
	});
}

void autonomous() {}

// driver control function
void opcontrol() {
	bool conveyorMoving = false; // variable to track if the conveyor is actively moving. used for checks when no button is pressed but power draw is low
	bool clamped = false; // variable to track if the clamp is currently down. used to allow both actions to be mapped to 1 button
	bool last_clamped = false; // variable to track if the clamp was activated or deactivated on the last cycle. used to prevent the clamp going up and down too quickly on accident
	
	while (true) { // main loop
		// get thumbstick positions
		int leftThumbstick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightThumbstick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.curvature(curve(leftThumbstick), curve(rightThumbstick), false); // set up arcade mode driving

		// checks if buttons a, b, r1, or l1 are being pressed and stores them in corresponding variables
		int a = controller.get_digital(DIGITAL_A); // start button
		int b = controller.get_digital(DIGITAL_B); // stop button
		int r1 = controller.get_digital(DIGITAL_R1); // slow move button
		int l1 = controller.get_digital(DIGITAL_L1); // slow reverse button
		// control flow logic for different controls regarding the conveyor
		if (b == 1 && conveyor.get_power() > 0.1) { // if the b button is pressed and the conveyor is being powered NOTE: THIS TAKES MASSIVE PRIORITY OVER ALL CONTROLS
			conveyor.brake(); // then brake the conveyor motor
			conveyorMoving = false; // and update variables
		} else if (a == 1) { // or if the a button is pressed
			conveyor.move(127); // begin moving the conveyor at full speed
			conveyorMoving = true; // and update variables
		} else if (r1 == 1) { // or if the r1 button is pressed
			conveyor.move_voltage(9000); // move at 9v out of 12v to be at a slower pace for fixing issues mid run
			conveyorMoving = false; // and update variables
		} else if (l1 == 1) { // or if l1 is pressed
			conveyor.move_voltage(-9000); // move reverse at 9v out of 12v to fix issues mid run
			conveyorMoving = false; // and update variables
		} else if (abs(conveyor.get_current_draw()) <= 5000 && !conveyorMoving) { // and finally if the conveyor power draw is lower than 5v and the conveyor isnt supposed to be moving
			conveyor.brake(); // then brake the conveyor motor
			// note this is mainly to prevent issues with the conveyor not stopping after l1 or r1 is pressed or not stopping for the main a button
		}

		// checks if the x button is pressed
		int x = controller.get_digital(DIGITAL_X); // clamp interact button
		if (x == 1) { // if x is pressed
			// enter another logic flow
			if (last_clamped) {} /* prevent flow from continuing if it x was pressed last cycle */ else if (clamped) { // if it is already clamped
				clamp.set_value(false); // unclamp it
				clamped = false; // and update variables
			} else { // if it isn't already clamped
				clamp.set_value(true); // clamp it
				clamped = true; // and update variables
			}
			last_clamped = true; // update variables at end to say x was pressed this (which will be last) cycle
		} else { // or if x isn't pressed
			last_clamped = false; // update variables to reflect this for next cycle
		}

		pros::delay(20); // run main loop every 20ms
	}
}