#include "../include/main.h"
#include <cmath>
#include <cstring>
#include <string>
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
pros::Imu imu(7); // setup a variable to store the inertial sensor
lemlib::OdomSensors sensors( // create an odom object that declares the following:
	nullptr, // no tracking wheels, same for all nullptrs
	nullptr,
	nullptr,
	nullptr,
	&imu // inertial sensor
);
lemlib::ControllerSettings lateral_controller(
	20, // proportional gain (kP)
	0, // integral gain (kI)
	8, // derivative gain (kD)
	3, // anti windup
	1, // small error range in inches
	100, // small error range timeout in milliseconds
	3, // large error range in inches
	500, // large error range timeout in milliseconds
	70 // maximum acceleration (slew)
);
lemlib::ControllerSettings angular_controller(
	5, // proportional gain (kP)
	0, // integral gain (kI)
	20, // derivative gain (kD)
	6, // anti windup
	1, // small error range in inches
	100, // small error range timeout in milliseconds
	3, // large error range in inches
	500, // large error range timeout in milliseconds
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
pros::Rotation rotation(20);
pros::MotorGroup arm({ 8, -9 });

// auton recording variables
FILE* file;
int error = 0;

void initialize() {
	pros::lcd::initialize(); // initialize the robot screen
	chassis.calibrate(); // calibrate the sensors

	// auton recording stuff
	file = fopen("/usd/recording2.txt", "w"); // open the recording file with write mode
	if (file == nullptr) {
		pros::lcd::print(4, "COULD NOT OPEN FILE");
		error = 1;
	}
}

void autonomous() {}

// driver control function
void opcontrol() {
	// check for error
	if (error == 1) {
		return;
	}

	bool conveyorMoving = false; // variable to track if the conveyor is actively moving. used for checks when no button is pressed but power draw is low
	bool clamped = false; // variable to track if the clamp is currently down. used to allow both actions to be mapped to 1 button
	bool last_clamped = false; // variable to track if the clamp was activated or deactivated on the last cycle. used to prevent the clamp going up and down too quickly on accident

	uint last_time = pros::millis();
	int until_update = 0;
	std::string buttons = "";

	while (pros::millis() <= 60000) { // main loop
		std::string output = ""; // initialize the string that will be written this cycle

		// print each:
		pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
		pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
		pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
		pros::lcd::print(3, "Time: %u", pros::millis()); // screen time info

		// get thumbstick positions
		int leftThumbstick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightThumbstick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.curvature(leftThumbstick, rightThumbstick, false); // set up arcade mode driving

		// write pose information to output
		output += std::to_string(round(chassis.getPose().x * 100.0) / 100.0) + ":" + std::to_string(round(chassis.getPose().y * 100.0) / 100.0) + ":" + std::to_string(round(chassis.getPose().theta * 1.0) / 1.0) + ":" + std::to_string(pros::millis() - last_time);

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
		// write each button that is pressed this cycle
		if (a) { buttons += "a"; }
		if (b) { buttons += "b"; }
		if (r1) { buttons += "r"; }
		if (l1) { buttons += "l"; }

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
		// write if x is pressed this cycle
		if (x) { buttons += "x"; }

		// checks if y, l2, or r2 are pressed
		int y = controller.get_digital(DIGITAL_Y); // prime button
		int l2 = controller.get_digital(DIGITAL_L2); // reverse button
		int r2 = controller.get_digital(DIGITAL_R2); // forward button
		if (y == 1) { // if y is pressed
			const int current_angle = rotation.get_position(); // get the current angle of the arm
			if (current_angle != 1500) { // and check to make sure it is not already at the ideal angle (not possible btw)
				arm.move_relative(1500 + current_angle, 100); // and then move it the proper relative angle to move toward the ideal angle set earlier
			}
		} else if (l2 == 1) { // or if l2 is pressed
			arm.move(-30); // reverse the arm at 30/127 speed
		} else if (r2 == 1) { // or if r2 is pressed
			arm.move(30); // move the arm forward at 30/127 speed
		} else { // if none are pressed
			arm.move(0); // stop the arm from moving
			arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // set the brake mode to hold, which technically doesn't exist on the mc55 but others said it worked so just leaving it in case
			arm.brake(); // and brake
			// the holding brakes should prevent the arm from falling from the force of gravity if in the air
		}
		if (y) { buttons += "y"; }
		if (l2) { buttons += "L"; }
		if (r2) { buttons += "R"; }


		output += buttons;
        output += "\n"; // add a newline to signal the next cycle
		if (until_update == 0) {
			if (fputs(output.c_str(), file) == EOF) {
				pros::lcd::print(4, "ERROR WRITING TO FILE"); // print to signal an error occurred
			}
			buttons = "";
			until_update = 5;
			last_time = pros::millis(); // update last_time variable
		} else {
			until_update -= 1;
		}

		pros::delay(20); // run main loop every 20ms
	}

	std::printf("WHILE LOOP FINISHED\n");

	fclose(file);
}